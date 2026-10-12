--!native

-- Pure-Luau DetourTileCache-style obstacle state machine over a Recast compact field.
-- The compressed-layer binary pipeline is still separate work, but obstacle refs/states and
-- cylinder/AABB/OBB semantics intentionally follow upstream DetourTileCache.

local Recast = require('@recast/core')
local Detour = require('@recast/detour')
local NavMeshBuilder = require('@recast/navmesh-builder')

local TileCache = {}

TileCache.DT_FAILURE = 0x80000000
TileCache.DT_SUCCESS = 0x40000000
TileCache.DT_OUT_OF_MEMORY = 0x04
TileCache.DT_INVALID_PARAM = 0x08
TileCache.DT_BUFFER_TOO_SMALL = 0x10
TileCache.DT_WRONG_MAGIC = 0x01
TileCache.DT_WRONG_VERSION = 0x02
TileCache.DT_ALREADY_OCCUPIED = 0x80

TileCache.DT_TILECACHE_MAGIC = 0x44544c52 -- 'DTLR'
TileCache.DT_TILECACHE_VERSION = 1
TileCache.DT_COMPRESSEDTILE_FREE_DATA = 0x01
TileCache.DT_TILECACHE_NULL_AREA = 0
TileCache.DT_TILECACHE_WALKABLE_AREA = 63
TileCache.DT_TILECACHE_NULL_IDX = 0xffff
TileCache.DT_MAX_TOUCHED_TILES = 8
TileCache.DT_MAX_REQUESTS = 64
TileCache.DT_MAX_UPDATE = 64
local TILECACHE_HEADER_SIZE = 56 -- sizeof(dtTileCacheLayerHeader), already 4-byte aligned.

TileCache.DT_OBSTACLE_EMPTY = 0
TileCache.DT_OBSTACLE_PROCESSING = 1
TileCache.DT_OBSTACLE_PROCESSED = 2
TileCache.DT_OBSTACLE_REMOVING = 3

TileCache.DT_OBSTACLE_CYLINDER = 0
TileCache.DT_OBSTACLE_BOX = 1
TileCache.DT_OBSTACLE_ORIENTED_BOX = 2

local function bswap32(v)
    return bit32.bor(
        bit32.lshift(bit32.band(v, 0xff), 24),
        bit32.lshift(bit32.band(bit32.rshift(v, 8), 0xff), 16),
        bit32.lshift(bit32.band(bit32.rshift(v, 16), 0xff), 8),
        bit32.band(bit32.rshift(v, 24), 0xff)
    )
end

local function swap2(data, offset)
    local a, b = buffer.readu8(data, offset), buffer.readu8(data, offset + 1)
    buffer.writeu8(data, offset, b); buffer.writeu8(data, offset + 1, a)
end

local function swap4(data, offset)
    local a, b = buffer.readu8(data, offset), buffer.readu8(data, offset + 1)
    local c, d = buffer.readu8(data, offset + 2), buffer.readu8(data, offset + 3)
    buffer.writeu8(data, offset, d); buffer.writeu8(data, offset + 1, c)
    buffer.writeu8(data, offset + 2, b); buffer.writeu8(data, offset + 3, a)
end

local function cloneBuffer(source)
    local out = buffer.create(buffer.len(source))
    buffer.copy(out, 0, source, 0, buffer.len(source))
    return out
end

local function bufferSlice(source, offset, size)
    local out = buffer.create(size)
    if size > 0 then buffer.copy(out, 0, source, offset, size) end
    return out
end

local function nextPow2(v)
    local p = 1
    while p < v do p *= 2 end
    return p
end

local function ilog2(v)
    local n = 0
    while v > 1 do v = math.floor(v / 2); n += 1 end
    return n
end

-- Same position lookup hash as DetourTileCache.  Only the low log2(tileLutSize) bits
-- can affect the result, so reducing both factors by the mask keeps the arithmetic
-- exactly representable as a Luau number even for negative/large tile coordinates.
local function computeTileHash(x, y, mask)
    local h1 = bit32.band(0x8da6b343, mask)
    local h2 = bit32.band(0xd8163841, mask)
    return bit32.band(h1 * bit32.band(x, mask) + h2 * bit32.band(y, mask), mask)
end

local function writeLayerHeader(target, header)
    buffer.writeu32(target, 0, header.magic or TileCache.DT_TILECACHE_MAGIC)
    buffer.writeu32(target, 4, header.version or TileCache.DT_TILECACHE_VERSION)
    buffer.writei32(target, 8, header.tx or 0)
    buffer.writei32(target, 12, header.ty or 0)
    buffer.writei32(target, 16, header.tlayer or 0)
    local bmin, bmax = header.bmin, header.bmax
    buffer.writef32(target, 20, bmin.X); buffer.writef32(target, 24, bmin.Y); buffer.writef32(target, 28, bmin.Z)
    buffer.writef32(target, 32, bmax.X); buffer.writef32(target, 36, bmax.Y); buffer.writef32(target, 40, bmax.Z)
    buffer.writeu16(target, 44, header.hmin or 0); buffer.writeu16(target, 46, header.hmax or 0)
    buffer.writeu8(target, 48, header.width); buffer.writeu8(target, 49, header.height)
    buffer.writeu8(target, 50, header.minx or 0); buffer.writeu8(target, 51, header.maxx or 0)
    buffer.writeu8(target, 52, header.miny or 0); buffer.writeu8(target, 53, header.maxy or 0)
end

local function readLayerHeader(source)
    if source == nil or buffer.len(source) < TILECACHE_HEADER_SIZE then return nil end
    return {
        magic = buffer.readu32(source, 0), version = buffer.readu32(source, 4),
        tx = buffer.readi32(source, 8), ty = buffer.readi32(source, 12), tlayer = buffer.readi32(source, 16),
        bmin = vector.create(buffer.readf32(source,20), buffer.readf32(source,24), buffer.readf32(source,28)),
        bmax = vector.create(buffer.readf32(source,32), buffer.readf32(source,36), buffer.readf32(source,40)),
        hmin = buffer.readu16(source,44), hmax = buffer.readu16(source,46),
        width = buffer.readu8(source,48), height = buffer.readu8(source,49),
        minx = buffer.readu8(source,50), maxx = buffer.readu8(source,51),
        miny = buffer.readu8(source,52), maxy = buffer.readu8(source,53),
    }
end

-- RecastNavigation leaves compression to the host.  The default here is the valid identity
-- compressor: it preserves Detour's exact 56-byte layer header and stores the byte grids verbatim.
local function compressPayload(compressor, raw)
    if compressor ~= nil and compressor.compress ~= nil then return compressor.compress(raw) end
    return cloneBuffer(raw)
end
local function decompressPayload(compressor, packed, expectedSize)
    local raw = if compressor ~= nil and compressor.decompress ~= nil then compressor.decompress(packed, expectedSize) else cloneBuffer(packed)
    if raw == nil or buffer.len(raw) < expectedSize then return nil end
    return raw
end

function TileCache.buildTileCacheLayer(header, heights, areas, cons, compressor)
    if header == nil or header.bmin == nil or header.bmax == nil then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_INVALID_PARAM) end
    local width, height = math.floor(header.width or 0), math.floor(header.height or 0)
    if width <= 0 or height <= 0 or width > 255 or height > 255 then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_INVALID_PARAM) end
    local gridSize = width * height
    if heights == nil or areas == nil or cons == nil or buffer.len(heights) < gridSize or buffer.len(areas) < gridSize or buffer.len(cons) < gridSize then
        return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_INVALID_PARAM)
    end
    local raw = buffer.create(gridSize * 3)
    buffer.copy(raw, 0, heights, 0, gridSize)
    buffer.copy(raw, gridSize, areas, 0, gridSize)
    buffer.copy(raw, gridSize * 2, cons, 0, gridSize)
    local packed = compressPayload(compressor, raw)
    if packed == nil then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_OUT_OF_MEMORY) end
    local data = buffer.create(TILECACHE_HEADER_SIZE + buffer.len(packed))
    writeLayerHeader(data, header)
    if buffer.len(packed) > 0 then buffer.copy(data, TILECACHE_HEADER_SIZE, packed, 0, buffer.len(packed)) end
    return data, TileCache.DT_SUCCESS
end

function TileCache.decompressTileCacheLayer(data, compressor)
    local header = readLayerHeader(data)
    if header == nil then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_INVALID_PARAM) end
    if header.magic ~= TileCache.DT_TILECACHE_MAGIC then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_WRONG_MAGIC) end
    if header.version ~= TileCache.DT_TILECACHE_VERSION then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_WRONG_VERSION) end
    local gridSize = header.width * header.height
    local packed = bufferSlice(data, TILECACHE_HEADER_SIZE, buffer.len(data) - TILECACHE_HEADER_SIZE)
    local raw = decompressPayload(compressor, packed, gridSize * 3)
    if raw == nil then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_BUFFER_TOO_SMALL) end
    return {
        header = header, regCount = 0,
        heights = bufferSlice(raw, 0, gridSize),
        areas = bufferSlice(raw, gridSize, gridSize),
        cons = bufferSlice(raw, gridSize * 2, gridSize),
        regs = buffer.create(gridSize),
    }, TileCache.DT_SUCCESS
end

function TileCache.tileCacheLayerHeader(data)
    return readLayerHeader(data)
end

-- Byte-for-byte port of dtTileCacheHeaderSwapEndian().  The 8-bit dimensions/bounds are
-- intentionally untouched; only the integer/float scalar fields in dtTileCacheLayerHeader swap.
function TileCache.tileCacheHeaderSwapEndian(data)
    if data == nil or buffer.len(data) < TILECACHE_HEADER_SIZE then return false end
    local magic, version = buffer.readu32(data, 0), buffer.readu32(data, 4)
    local native = magic == TileCache.DT_TILECACHE_MAGIC and version == TileCache.DT_TILECACHE_VERSION
    local swapped = magic == bswap32(TileCache.DT_TILECACHE_MAGIC) and version == bswap32(TileCache.DT_TILECACHE_VERSION)
    if not native and not swapped then return false end

    for o = 0, 40, 4 do swap4(data, o) end
    swap2(data, 44); swap2(data, 46)
    return true
end

-- Upstream-style spelling for callers mirroring the C API.
TileCache.headerSwapEndian = TileCache.tileCacheHeaderSwapEndian

function TileCache.statusSucceed(status) return bit32.band(status, TileCache.DT_SUCCESS) ~= 0 end
function TileCache.statusFailed(status) return bit32.band(status, TileCache.DT_FAILURE) ~= 0 end

-- Convenience mesh process for Jess callers that just want every non-null TileCache polygon
-- traversable.  dtTileCache itself does not apply flags automatically; callers seeking exact
-- upstream behavior should pass their own meshProcess (or nil) to init().
function TileCache.defaultMeshProcess(params, polyAreas, polyFlags)
    local count=params and params.polyCount or 0
    for i0=0,count-1 do
        if buffer.readu8(polyAreas,i0)~=TileCache.DT_TILECACHE_NULL_AREA then buffer.writeu16(polyFlags,i0*2,1) end
    end
end

function TileCache.encodeObstacleId(salt, index0)
    return salt * 65536 + index0
end
function TileCache.decodeObstacleIdSalt(ref) return math.floor(ref / 65536) % 65536 end
function TileCache.decodeObstacleIdObstacle(ref) return ref % 65536 end

local function makeObstaclePool(maxObstacles)
    if maxObstacles <= 0 or maxObstacles > 65536 then error('maxObstacles must be in [1, 65536]', 3) end
    local obstacles = table.create(maxObstacles)
    local free = table.create(maxObstacles)
    for i = maxObstacles - 1, 0, -1 do
        obstacles[i + 1] = {_slot=i, salt=1, state=TileCache.DT_OBSTACLE_EMPTY, type=TileCache.DT_OBSTACLE_CYLINDER}
        free[#free + 1] = i
    end
    return obstacles, free
end

function TileCache.init(params, options)
    params, options = params or {}, options or {}
    local maxTiles = math.floor(params.maxTiles or 0)
    local maxObstacles = math.floor(params.maxObstacles or 128)
    local width, height = math.floor(params.width or 0), math.floor(params.height or 0)
    local cs, ch = params.cs or 0, params.ch or 0
    if maxTiles <= 0 or width <= 0 or height <= 0 or cs <= 0 or ch <= 0 then
        return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_INVALID_PARAM)
    end
    local tileBits = ilog2(nextPow2(maxTiles))
    local saltBits = math.min(31, 32 - tileBits)
    if saltBits < 10 then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_INVALID_PARAM) end
    local obstacles, freeObstacles = makeObstaclePool(maxObstacles)
    local tiles, freeTiles = table.create(maxTiles), table.create(maxTiles)
    for i = maxTiles - 1, 0, -1 do
        tiles[i + 1] = {_slot=i, salt=1, header=nil, data=nil, flags=0, _nextPos=-1}
        freeTiles[#freeTiles + 1] = i
    end
    -- dtTileCache uses a power-of-two hash table sized to maxTiles/4 (integer division).
    local tileLutSize = nextPow2(math.floor(maxTiles / 4))
    if tileLutSize < 1 then tileLutSize = 1 end
    local posLookup = table.create(tileLutSize, -1)
    local normalized = {
        orig = params.orig or vector.zero, cs = cs, ch = ch, width = width, height = height,
        walkableHeight = params.walkableHeight or 2, walkableRadius = params.walkableRadius or 0,
        walkableClimb = params.walkableClimb or 0, maxSimplificationError = params.maxSimplificationError or 1.3,
        maxTiles = maxTiles, maxObstacles = maxObstacles,
    }
    local navmesh = options.navmesh or Detour.newTiledNavMesh({
        orig = normalized.orig, tileWidth = width * cs, tileHeight = height * cs,
        maxTiles = options.navMaxTiles or maxTiles, maxPolys = options.maxPolys or 2048,
        maxVertsPerPoly = options.maxVertsPerPoly or 6,
    })
    return {
        params = normalized, allocator = options.allocator, compressor = options.compressor, meshProcess = options.meshProcess,
        tileBits = tileBits, saltBits = saltBits, tiles = tiles, freeTiles = freeTiles,
        tileLutSize = tileLutSize, tileLutMask = tileLutSize - 1, posLookup = posLookup,
        obstacles = obstacles, freeObstacles = freeObstacles, maxObstacles = maxObstacles,
        requests = {}, updates = {}, navmesh = navmesh, dirty = false,
        _compressed = true,
    }, TileCache.DT_SUCCESS
end

function TileCache.getAlloc(cache) return cache.allocator end
function TileCache.getCompressor(cache) return cache.compressor end
function TileCache.getParams(cache) return cache.params end
function TileCache.getTileCount(cache) return cache.params and cache.params.maxTiles or 0 end
function TileCache.getTile(cache, index0)
    if cache.tiles == nil or index0 < 0 or index0 >= #cache.tiles then return nil end
    return cache.tiles[index0 + 1]
end

function TileCache.encodeTileId(cache, salt, index0)
    return salt * 2^cache.tileBits + index0
end
function TileCache.decodeTileIdSalt(cache, ref)
    return math.floor(ref / 2^cache.tileBits) % 2^cache.saltBits
end
function TileCache.decodeTileIdTile(cache, ref)
    return ref % 2^cache.tileBits
end

function TileCache.getTileRef(cache, tile)
    if tile == nil or tile._slot == nil or cache.tiles == nil or cache.tiles[tile._slot + 1] ~= tile then return 0 end
    return TileCache.encodeTileId(cache, tile.salt, tile._slot)
end
function TileCache.getTileByRef(cache, ref)
    if cache.tiles == nil or ref == nil or ref == 0 then return nil end
    local slot = TileCache.decodeTileIdTile(cache, ref)
    if slot >= #cache.tiles then return nil end
    local tile = cache.tiles[slot + 1]
    if tile.salt ~= TileCache.decodeTileIdSalt(cache, ref) then return nil end
    return tile
end
function TileCache.getTileAt(cache, tx, ty, tlayer)
    if cache.tiles == nil or cache.posLookup == nil then return nil end
    local slot = cache.posLookup[computeTileHash(tx, ty, cache.tileLutMask) + 1]
    while slot ~= nil and slot >= 0 do
        local tile = cache.tiles[slot + 1]
        local h = tile.header
        if h ~= nil and h.tx == tx and h.ty == ty and h.tlayer == tlayer then return tile end
        slot = tile._nextPos
    end
    return nil
end
function TileCache.getTilesAt(cache, tx, ty, maxTiles)
    local out = {}
    if cache.tiles == nil or cache.posLookup == nil then return out end
    maxTiles = maxTiles or math.huge
    local slot = cache.posLookup[computeTileHash(tx, ty, cache.tileLutMask) + 1]
    while slot ~= nil and slot >= 0 do
        local tile = cache.tiles[slot + 1]
        local h = tile.header
        if h ~= nil and h.tx == tx and h.ty == ty and #out < maxTiles then
            out[#out + 1] = TileCache.getTileRef(cache, tile)
        end
        slot = tile._nextPos
    end
    return out
end

function TileCache.addTile(cache, data, flags)
    if cache.tiles == nil then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_INVALID_PARAM) end
    local header = readLayerHeader(data)
    if header == nil then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_INVALID_PARAM) end
    if header.magic ~= TileCache.DT_TILECACHE_MAGIC then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_WRONG_MAGIC) end
    if header.version ~= TileCache.DT_TILECACHE_VERSION then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_WRONG_VERSION) end
    if TileCache.getTileAt(cache, header.tx, header.ty, header.tlayer) ~= nil then return nil, TileCache.DT_FAILURE end
    local n = #cache.freeTiles
    if n == 0 then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_OUT_OF_MEMORY) end
    local slot = cache.freeTiles[n]; cache.freeTiles[n] = nil
    local tile = cache.tiles[slot + 1]
    local hash = computeTileHash(header.tx, header.ty, cache.tileLutMask)
    tile._nextPos = cache.posLookup[hash + 1]
    cache.posLookup[hash + 1] = slot
    tile.header, tile.data, tile.flags = header, data, flags or 0
    return TileCache.getTileRef(cache, tile), TileCache.DT_SUCCESS
end

function TileCache.removeTile(cache, ref)
    local tile = TileCache.getTileByRef(cache, ref)
    if tile == nil or tile.header == nil then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_INVALID_PARAM) end
    local data = if bit32.band(tile.flags, TileCache.DT_COMPRESSEDTILE_FREE_DATA) ~= 0 then nil else tile.data
    local hash = computeTileHash(tile.header.tx, tile.header.ty, cache.tileLutMask)
    local slot, prev = cache.posLookup[hash + 1], -1
    while slot ~= nil and slot >= 0 do
        if slot == tile._slot then
            if prev >= 0 then cache.tiles[prev + 1]._nextPos = tile._nextPos else cache.posLookup[hash + 1] = tile._nextPos end
            break
        end
        prev = slot
        slot = cache.tiles[slot + 1]._nextPos
    end
    tile.header, tile.data, tile.flags, tile._nextPos = nil, nil, 0, -1
    local mask = 2^cache.saltBits - 1
    tile.salt = bit32.band(tile.salt + 1, mask)
    if tile.salt == 0 then tile.salt = 1 end
    cache.freeTiles[#cache.freeTiles + 1] = tile._slot
    return data, TileCache.DT_SUCCESS
end

function TileCache.calcTightTileBounds(cache, header)
    local cs = cache.params.cs
    return vector.create(header.bmin.X + header.minx * cs, header.bmin.Y, header.bmin.Z + header.miny * cs),
        vector.create(header.bmin.X + (header.maxx + 1) * cs, header.bmax.Y, header.bmin.Z + (header.maxy + 1) * cs)
end

local function overlapBounds(amin, amax, bmin, bmax)
    return amin.X <= bmax.X and amax.X >= bmin.X and amin.Y <= bmax.Y and amax.Y >= bmin.Y and amin.Z <= bmax.Z and amax.Z >= bmin.Z
end

function TileCache.queryTiles(cache, bmin, bmax, maxResults)
    if cache.params == nil or cache.tiles == nil then return {}, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_INVALID_PARAM) end
    maxResults = maxResults or math.huge
    local p = cache.params
    local tw, th = p.width * p.cs, p.height * p.cs
    local tx0, tx1 = math.floor((bmin.X - p.orig.X) / tw), math.floor((bmax.X - p.orig.X) / tw)
    local ty0, ty1 = math.floor((bmin.Z - p.orig.Z) / th), math.floor((bmax.Z - p.orig.Z) / th)
    local out = {}
    for ty = ty0, ty1 do
        for tx = tx0, tx1 do
            local refs = TileCache.getTilesAt(cache, tx, ty, 32)
            for _,ref in refs do
                local tile = TileCache.getTileByRef(cache, ref)
                local tbmin, tbmax = TileCache.calcTightTileBounds(cache, tile.header)
                if overlapBounds(bmin, bmax, tbmin, tbmax) and #out < maxResults then out[#out + 1] = ref end
            end
        end
    end
    return out, TileCache.DT_SUCCESS
end

local function allocObstacle(cache)
    local n = #cache.freeObstacles
    if n == 0 then return nil end
    local slot = cache.freeObstacles[n]
    cache.freeObstacles[n] = nil
    local ob = cache.obstacles[slot + 1]
    ob.state = TileCache.DT_OBSTACLE_PROCESSING
    ob.type = TileCache.DT_OBSTACLE_CYLINDER
    ob.touched = {}
    ob.pending = {}
    return ob
end

local function releaseObstacle(cache, ob)
    ob.state = TileCache.DT_OBSTACLE_EMPTY
    ob.kind = nil
    ob.position = nil
    ob.radius = nil
    ob.height = nil
    ob.bmin = nil
    ob.bmax = nil
    ob.center = nil
    ob.halfExtents = nil
    ob.yRadians = nil
    ob.rotAux0 = nil
    ob.rotAux1 = nil
    ob.salt = (ob.salt + 1) % 65536
    if ob.salt == 0 then ob.salt = 1 end
    cache.freeObstacles[#cache.freeObstacles + 1] = ob._slot
end

function TileCache.new(build, options)
    options = options or {}
    local chf = build.compact or build
    if chf == nil or chf.areas == nil then error('tile cache needs a compact heightfield or build result', 2) end
    local maxObstacles = options.maxObstacles or 128
    if maxObstacles <= 0 or maxObstacles > 65536 then error('maxObstacles must be in [1, 65536]', 2) end
    local obstacles = table.create(maxObstacles)
    local free = table.create(maxObstacles)
    -- Upstream freelist yields slot 0 first.
    for i = maxObstacles - 1, 0, -1 do
        obstacles[i + 1] = {_slot=i, salt=1, state=TileCache.DT_OBSTACLE_EMPTY, type=TileCache.DT_OBSTACLE_CYLINDER}
        free[#free + 1] = i
    end
    return {
        compact = chf,
        baseAreas = cloneBuffer(chf.areas),
        obstacles = obstacles,
        freeObstacles = free,
        maxObstacles = maxObstacles,
        requests = {},
        navmesh = build.navmesh,
        dirty = false,
    }
end

function TileCache.getObstacleCount(cache) return cache.maxObstacles end
function TileCache.getObstacle(cache, index0)
    if index0 < 0 or index0 >= cache.maxObstacles then return nil end
    return cache.obstacles[index0 + 1]
end
function TileCache.getObstacleRef(cache, ob)
    if ob == nil or ob._slot == nil or cache.obstacles[ob._slot + 1] ~= ob then return 0 end
    return TileCache.encodeObstacleId(ob.salt, ob._slot)
end
function TileCache.getObstacleByRef(cache, ref)
    if ref == nil or ref == 0 then return nil end
    local slot = TileCache.decodeObstacleIdObstacle(ref)
    if slot >= cache.maxObstacles then return nil end
    local ob = cache.obstacles[slot + 1]
    if ob.salt ~= TileCache.decodeObstacleIdSalt(ref) then return nil end
    return ob
end

local function enqueue(cache, action, ref)
    cache.requests[#cache.requests + 1] = {action=action, ref=ref}
    cache.dirty = true
end

function TileCache.addCylinder(cache, position, radius, height)
    if cache._compressed and #cache.requests >= TileCache.DT_MAX_REQUESTS then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_BUFFER_TOO_SMALL) end
    local ob = allocObstacle(cache)
    if ob == nil then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_OUT_OF_MEMORY) end
    ob.type = TileCache.DT_OBSTACLE_CYLINDER
    ob.kind = 'cylinder'
    ob.position, ob.radius, ob.height = position, radius, height
    local ref = TileCache.getObstacleRef(cache, ob)
    enqueue(cache, 'add', ref)
    return ref, TileCache.DT_SUCCESS
end

TileCache.addObstacle = TileCache.addCylinder

function TileCache.addBox(cache, bmin, bmax)
    if cache._compressed and #cache.requests >= TileCache.DT_MAX_REQUESTS then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_BUFFER_TOO_SMALL) end
    local ob = allocObstacle(cache)
    if ob == nil then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_OUT_OF_MEMORY) end
    ob.type = TileCache.DT_OBSTACLE_BOX
    ob.kind = 'box'
    ob.bmin, ob.bmax = bmin, bmax
    local ref = TileCache.getObstacleRef(cache, ob)
    enqueue(cache, 'add', ref)
    return ref, TileCache.DT_SUCCESS
end

function TileCache.addOrientedBox(cache, center, halfExtents, yRadians)
    if cache._compressed and #cache.requests >= TileCache.DT_MAX_REQUESTS then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_BUFFER_TOO_SMALL) end
    local ob = allocObstacle(cache)
    if ob == nil then return nil, bit32.bor(TileCache.DT_FAILURE, TileCache.DT_OUT_OF_MEMORY) end
    ob.type = TileCache.DT_OBSTACLE_ORIENTED_BOX
    ob.kind = 'orientedBox'
    ob.center, ob.halfExtents, ob.yRadians = center, halfExtents, yRadians
    local coshalf = math.cos(0.5 * yRadians)
    local sinhalf = math.sin(-0.5 * yRadians)
    ob.rotAux0 = coshalf * sinhalf
    ob.rotAux1 = coshalf * coshalf - 0.5
    local ref = TileCache.getObstacleRef(cache, ob)
    enqueue(cache, 'add', ref)
    return ref, TileCache.DT_SUCCESS
end

-- Upstream exposes two addBoxObstacle overloads.  Keep the explicit Jess spellings too.
function TileCache.addBoxObstacle(cache, a, b, yRadians)
    if yRadians == nil then return TileCache.addBox(cache, a, b) end
    return TileCache.addOrientedBox(cache, a, b, yRadians)
end

function TileCache.removeObstacle(cache, ref)
    if ref == nil or ref == 0 then return TileCache.DT_SUCCESS end
    if cache._compressed and #cache.requests >= TileCache.DT_MAX_REQUESTS then return bit32.bor(TileCache.DT_FAILURE, TileCache.DT_BUFFER_TOO_SMALL) end
    enqueue(cache, 'remove', ref)
    return TileCache.DT_SUCCESS
end

function TileCache.getObstacleBounds(cache, ob)
    if ob == nil or ob.state == TileCache.DT_OBSTACLE_EMPTY then return nil end
    if ob.type == TileCache.DT_OBSTACLE_CYLINDER then
        local p,r = ob.position,ob.radius
        return vector.create(p.X-r,p.Y,p.Z-r), vector.create(p.X+r,p.Y+ob.height,p.Z+r)
    elseif ob.type == TileCache.DT_OBSTACLE_BOX then
        return ob.bmin,ob.bmax
    else
        local c,e = ob.center,ob.halfExtents
        local maxr = 1.41 * math.max(e.X,e.Z)
        return vector.create(c.X-maxr,c.Y-e.Y,c.Z-maxr), vector.create(c.X+maxr,c.Y+e.Y,c.Z+maxr)
    end
end

local function cylinderTouchesCell(obstacle, x0, x1, z0, z1, y)
    local p = obstacle.position
    if y < p.Y - 0.001 or y > p.Y + obstacle.height + 0.001 then return false end
    local dx = 0
    if p.X < x0 then dx = x0 - p.X elseif p.X > x1 then dx = p.X - x1 end
    local dz = 0
    if p.Z < z0 then dz = z0 - p.Z elseif p.Z > z1 then dz = p.Z - z1 end
    return dx * dx + dz * dz <= obstacle.radius * obstacle.radius
end

local function boxTouchesCell(obstacle, x0, x1, z0, z1, y)
    return x1 >= obstacle.bmin.X and x0 <= obstacle.bmax.X
       and z1 >= obstacle.bmin.Z and z0 <= obstacle.bmax.Z
       and y >= obstacle.bmin.Y and y <= obstacle.bmax.Y
end

-- Equivalent to DetourTileCacheBuilder's rotated-box test, adapted to compact-field spans.
local function orientedBoxTouchesCell(obstacle, chf, x, z, spanY)
    local c,e = obstacle.center,obstacle.halfExtents
    local miny = math.floor((c.Y - e.Y - chf.bmin.Y) / chf.ch)
    local maxy = math.floor((c.Y + e.Y - chf.bmin.Y) / chf.ch)
    if spanY < miny or spanY > maxy then return false end
    local cx = (c.X - chf.bmin.X) / chf.cs
    local cz = (c.Z - chf.bmin.Z) / chf.cs
    local x2 = 2 * (x - cx)
    local z2 = 2 * (z - cz)
    local xr = obstacle.rotAux1 * x2 + obstacle.rotAux0 * z2
    local zr = obstacle.rotAux1 * z2 - obstacle.rotAux0 * x2
    local xhalf = e.X / chf.cs + 0.5
    local zhalf = e.Z / chf.cs + 0.5
    return xr >= -xhalf and xr <= xhalf and zr >= -zhalf and zr <= zhalf
end

local function processRequests(cache)
    for _,req in cache.requests do
        local ob = TileCache.getObstacleByRef(cache, req.ref)
        if ob ~= nil then
            if req.action == 'add' then
                if ob.state == TileCache.DT_OBSTACLE_PROCESSING then
                    -- Single compact field is one logical tile until compressed tile layers land.
                    ob.touched[1] = 1
                    ob.pending[1] = 1
                end
            elseif req.action == 'remove' then
                ob.state = TileCache.DT_OBSTACLE_REMOVING
                ob.pending[1] = 1
            end
        end
    end
    table.clear(cache.requests)
end

local function rebuildInternal(cache)
    local chf = cache.compact
    buffer.copy(chf.areas, 0, cache.baseAreas, 0, buffer.len(cache.baseAreas))

    for z = 0, chf.height - 1 do
        for x = 0, chf.width - 1 do
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            if count == 0 then continue end
            local x0, x1 = chf.bmin.X + x * chf.cs, chf.bmin.X + (x + 1) * chf.cs
            local z0, z1 = chf.bmin.Z + z * chf.cs, chf.bmin.Z + (z + 1) * chf.cs
            for li = 0, count - 1 do
                local span = first + li
                if buffer.readu8(chf.areas, span) == Recast.RC_NULL_AREA then continue end
                local sy = buffer.readu16(chf.y, span * 2)
                local y = chf.bmin.Y + sy * chf.ch
                for _, obstacle in cache.obstacles do
                    if obstacle.state == TileCache.DT_OBSTACLE_EMPTY or obstacle.state == TileCache.DT_OBSTACLE_REMOVING then continue end
                    local hit
                    if obstacle.type == TileCache.DT_OBSTACLE_CYLINDER then
                        hit = cylinderTouchesCell(obstacle, x0, x1, z0, z1, y)
                    elseif obstacle.type == TileCache.DT_OBSTACLE_BOX then
                        hit = boxTouchesCell(obstacle, x0, x1, z0, z1, y)
                    else
                        hit = orientedBoxTouchesCell(obstacle, chf, x, z, sy)
                    end
                    if hit then
                        buffer.writeu8(chf.areas, span, Recast.RC_NULL_AREA)
                        break
                    end
                end
            end
        end
    end

    Recast.buildConnectedRegions(chf)
    cache.navmesh = Detour.buildCompactNavMesh(chf)
end

local function finalizeStates(cache)
    for _,ob in cache.obstacles do
        if ob.state == TileCache.DT_OBSTACLE_PROCESSING then
            ob.state = TileCache.DT_OBSTACLE_PROCESSED
            table.clear(ob.pending)
        elseif ob.state == TileCache.DT_OBSTACLE_REMOVING then
            table.clear(ob.pending)
            releaseObstacle(cache, ob)
        end
    end
    cache.dirty = false
end

function TileCache.layerFromHeightfieldLayer(layer, tx, ty, tlayer, compressor)
    local header = {
        magic = TileCache.DT_TILECACHE_MAGIC, version = TileCache.DT_TILECACHE_VERSION,
        tx = tx or 0, ty = ty or 0, tlayer = tlayer or 0,
        bmin = layer.bmin, bmax = layer.bmax,
        hmin = layer.hmin or 0, hmax = layer.hmax or 0,
        width = layer.width, height = layer.height,
        minx = layer.minx or 0, maxx = layer.maxx or (layer.width - 1),
        miny = layer.miny or 0, maxy = layer.maxy or (layer.height - 1),
    }
    return TileCache.buildTileCacheLayer(header, layer.heights, layer.areas, layer.cons, compressor)
end

local function markLayerCylinder(layer, cs, ch, pos, radius, height, areaId)
    local header = layer.header
    local orig, w, h = header.bmin, header.width, header.height
    local ics, ich = 1 / cs, 1 / ch
    local px, pz = (pos.X - orig.X) * ics, (pos.Z - orig.Z) * ics
    local minx = math.floor((pos.X - radius - orig.X) * ics)
    local maxx = math.floor((pos.X + radius - orig.X) * ics)
    local minz = math.floor((pos.Z - radius - orig.Z) * ics)
    local maxz = math.floor((pos.Z + radius - orig.Z) * ics)
    local miny = math.floor((pos.Y - orig.Y) * ich)
    local maxy = math.floor((pos.Y + height - orig.Y) * ich)
    if maxx < 0 or minx >= w or maxz < 0 or minz >= h then return TileCache.DT_SUCCESS end
    minx, maxx = math.max(0,minx), math.min(w-1,maxx)
    minz, maxz = math.max(0,minz), math.min(h-1,maxz)
    local r2 = (radius * ics + 0.5)^2
    for z=minz,maxz do
        for x=minx,maxx do
            local dx,dz=(x+0.5)-px,(z+0.5)-pz
            if dx*dx+dz*dz <= r2 then
                local idx=x+z*w; local y=buffer.readu8(layer.heights,idx)
                if y>=miny and y<=maxy then buffer.writeu8(layer.areas,idx,areaId) end
            end
        end
    end
    return TileCache.DT_SUCCESS
end

local function markLayerBox(layer, cs, ch, bmin, bmax, areaId)
    local header=layer.header; local orig,w,h=header.bmin,header.width,header.height
    local ics,ich=1/cs,1/ch
    local minx,maxx=math.floor((bmin.X-orig.X)*ics),math.floor((bmax.X-orig.X)*ics)
    local minz,maxz=math.floor((bmin.Z-orig.Z)*ics),math.floor((bmax.Z-orig.Z)*ics)
    local miny,maxy=math.floor((bmin.Y-orig.Y)*ich),math.floor((bmax.Y-orig.Y)*ich)
    if maxx<0 or minx>=w or maxz<0 or minz>=h then return TileCache.DT_SUCCESS end
    minx,maxx=math.max(0,minx),math.min(w-1,maxx); minz,maxz=math.max(0,minz),math.min(h-1,maxz)
    for z=minz,maxz do for x=minx,maxx do
        local idx=x+z*w; local y=buffer.readu8(layer.heights,idx)
        if y>=miny and y<=maxy then buffer.writeu8(layer.areas,idx,areaId) end
    end end
    return TileCache.DT_SUCCESS
end

local function markLayerOrientedBox(layer, cs, ch, center, halfExtents, rotAux0, rotAux1, areaId)
    local header=layer.header; local orig,w,h=header.bmin,header.width,header.height
    local ics,ich=1/cs,1/ch
    local cx,cz=(center.X-orig.X)*ics,(center.Z-orig.Z)*ics
    local maxr=1.41*math.max(halfExtents.X,halfExtents.Z)
    local minx,maxx=math.floor(cx-maxr*ics),math.floor(cx+maxr*ics)
    local minz,maxz=math.floor(cz-maxr*ics),math.floor(cz+maxr*ics)
    local miny,maxy=math.floor((center.Y-halfExtents.Y-orig.Y)*ich),math.floor((center.Y+halfExtents.Y-orig.Y)*ich)
    if maxx<0 or minx>=w or maxz<0 or minz>=h then return TileCache.DT_SUCCESS end
    minx,maxx=math.max(0,minx),math.min(w-1,maxx); minz,maxz=math.max(0,minz),math.min(h-1,maxz)
    local xhalf,zhalf=halfExtents.X*ics+0.5,halfExtents.Z*ics+0.5
    for z=minz,maxz do for x=minx,maxx do
        local x2,z2=2*(x-cx),2*(z-cz)
        local xr=rotAux1*x2+rotAux0*z2
        if xr<=xhalf and xr>=-xhalf then
            local zr=rotAux1*z2-rotAux0*x2
            if zr<=zhalf and zr>=-zhalf then
                local idx=x+z*w; local y=buffer.readu8(layer.heights,idx)
                if y>=miny and y<=maxy then buffer.writeu8(layer.areas,idx,areaId) end
            end
        end
    end end
    return TileCache.DT_SUCCESS
end

function TileCache.markCylinderArea(layer, cs, ch, pos, radius, height, areaId)
    return markLayerCylinder(layer,cs,ch,pos,radius,height,areaId or TileCache.DT_TILECACHE_NULL_AREA)
end
function TileCache.markBoxArea(layer, cs, ch, bmin, bmax, areaId)
    return markLayerBox(layer,cs,ch,bmin,bmax,areaId or TileCache.DT_TILECACHE_NULL_AREA)
end
function TileCache.markOrientedBoxArea(layer, cs, ch, center, halfExtents, yRadians, areaId)
    local c,s=math.cos(0.5*yRadians),math.sin(-0.5*yRadians)
    return markLayerOrientedBox(layer,cs,ch,center,halfExtents,c*s,c*c-0.5,areaId or TileCache.DT_TILECACHE_NULL_AREA)
end

local function tileLayerConnected(layer, ia, ib, walkableClimb)
    if buffer.readu8(layer.areas,ia)~=buffer.readu8(layer.areas,ib) then return false end
    return math.abs(buffer.readu8(layer.heights,ia)-buffer.readu8(layer.heights,ib))<=walkableClimb
end

local function addUniqueLast(list,v)
    if #list>0 and list[#list]==v then return end
    list[#list+1]=v
end

local function tileRegionCanMerge(oldId,newId,regs)
    local count=0
    for _,reg in regs do
        if reg.regId==oldId then
            for _,nei0 in reg.neis do if regs[nei0+1].regId==newId then count+=1 end end
        end
    end
    return count==1
end

-- Direct Luau translation of dtBuildTileCacheRegions.  Keep this separate from Recast's generic
-- region builder: TileCache intentionally partitions/merges its byte-grid layers differently.
function TileCache.buildTileCacheRegions(layer, walkableClimb)
    local w,h=layer.header.width,layer.header.height
    buffer.fill(layer.regs,0,0xff,w*h)
    local sweeps=table.create(w)
    for i=1,w do sweeps[i]={ns=0,id=0,nei=0xff} end
    local prevCount=table.create(256,0)
    local regId=0
    for y=0,h-1 do
        if regId>0 then for i=1,regId do prevCount[i]=0 end end
        local sweepId=0
        for x=0,w-1 do
            local idx=x+y*w
            if buffer.readu8(layer.areas,idx)==TileCache.DT_TILECACHE_NULL_AREA then continue end
            local sid=0xff
            local xidx=(x-1)+y*w
            if x>0 and tileLayerConnected(layer,idx,xidx,walkableClimb) then
                local xr=buffer.readu8(layer.regs,xidx); if xr~=0xff then sid=xr end
            end
            if sid==0xff then
                sid=sweepId; sweepId+=1
                local sw=sweeps[sid+1]; sw.nei=0xff; sw.ns=0
            end
            local yidx=x+(y-1)*w
            if y>0 and tileLayerConnected(layer,idx,yidx,walkableClimb) then
                local nr=buffer.readu8(layer.regs,yidx)
                if nr~=0xff then
                    local sw=sweeps[sid+1]
                    if sw.ns==0 then sw.nei=nr end
                    if sw.nei==nr then
                        sw.ns+=1; prevCount[nr+1]=(prevCount[nr+1] or 0)+1
                    else sw.nei=0xff end
                end
            end
            buffer.writeu8(layer.regs,idx,sid)
        end
        for i=0,sweepId-1 do
            local sw=sweeps[i+1]
            if sw.nei~=0xff and (prevCount[sw.nei+1] or 0)==sw.ns then sw.id=sw.nei
            else
                if regId==255 then return bit32.bor(TileCache.DT_FAILURE,TileCache.DT_BUFFER_TOO_SMALL) end
                sw.id=regId; regId+=1
            end
        end
        for x=0,w-1 do
            local idx=x+y*w; local sr=buffer.readu8(layer.regs,idx)
            if sr~=0xff then buffer.writeu8(layer.regs,idx,sweeps[sr+1].id) end
        end
    end

    local nregs=regId; local regs=table.create(nregs)
    for i=0,nregs-1 do regs[i+1]={area=0,neis={},nneis=0,regId=0xff,areaId=0} end
    for y=0,h-1 do for x=0,w-1 do
        local idx=x+y*w; local ri=buffer.readu8(layer.regs,idx); if ri==0xff then continue end
        local reg=regs[ri+1]; reg.area+=1; reg.areaId=buffer.readu8(layer.areas,idx)
        local ymi=x+(y-1)*w
        if y>0 and tileLayerConnected(layer,idx,ymi,walkableClimb) then
            local rai=buffer.readu8(layer.regs,ymi)
            if rai~=0xff and rai~=ri then
                addUniqueLast(reg.neis,rai); addUniqueLast(regs[rai+1].neis,ri)
            end
        end
    end end
    for i=0,nregs-1 do regs[i+1].regId=i end
    for i=0,nregs-1 do
        local reg=regs[i+1]; local merge=-1; local mergea=0
        for _,nei in reg.neis do
            local regn=regs[nei+1]
            if reg.regId~=regn.regId and reg.areaId==regn.areaId and regn.area>mergea and tileRegionCanMerge(reg.regId,regn.regId,regs) then
                mergea=regn.area; merge=nei
            end
        end
        if merge~=-1 then
            local oldId,newId=reg.regId,regs[merge+1].regId
            for _,r in regs do if r.regId==oldId then r.regId=newId end end
        end
    end
    local remap=table.create(256,0)
    for _,r in regs do remap[r.regId+1]=1 end
    regId=0
    for i=0,nregs-1 do if remap[i+1]~=0 then remap[i+1]=regId; regId+=1 end end
    for _,r in regs do r.regId=remap[r.regId+1] end
    layer.regCount=regId
    for i=0,w*h-1 do
        local r=buffer.readu8(layer.regs,i)
        if r~=0xff then buffer.writeu8(layer.regs,i,regs[r+1].regId) end
    end
    return TileCache.DT_SUCCESS
end

local TC_DIR_X={-1,0,1,0}
local TC_DIR_Y={0,1,0,-1}

local function tcAppendVertex(cont,x,y,z,r)
    if cont.nverts>1 then
        local pa=(cont.nverts-2)*4; local pb=(cont.nverts-1)*4
        if buffer.readu8(cont.verts,pb+3)==r then
            if buffer.readu8(cont.verts,pa)==buffer.readu8(cont.verts,pb) and buffer.readu8(cont.verts,pb)==x then
                buffer.writeu8(cont.verts,pb+1,y); buffer.writeu8(cont.verts,pb+2,z); return true
            elseif buffer.readu8(cont.verts,pa+2)==buffer.readu8(cont.verts,pb+2) and buffer.readu8(cont.verts,pb+2)==z then
                buffer.writeu8(cont.verts,pb,x); buffer.writeu8(cont.verts,pb+1,y); return true
            end
        end
    end
    if cont.nverts+1>cont.cverts then return false end
    local o=cont.nverts*4
    buffer.writeu8(cont.verts,o,x); buffer.writeu8(cont.verts,o+1,y); buffer.writeu8(cont.verts,o+2,z); buffer.writeu8(cont.verts,o+3,r)
    cont.nverts+=1; return true
end

local function tcNeighbourReg(layer,ax,ay,dir0)
    local w=layer.header.width; local ia=ax+ay*w
    local c=buffer.readu8(layer.cons,ia); local con=bit32.band(c,0x0f); local portal=bit32.rshift(c,4); local mask=bit32.lshift(1,dir0)
    if bit32.band(con,mask)==0 then
        if bit32.band(portal,mask)~=0 then return 0xf8+dir0 end
        return 0xff
    end
    local bx,by=ax+TC_DIR_X[dir0+1],ay+TC_DIR_Y[dir0+1]
    return buffer.readu8(layer.regs,bx+by*w)
end

local function tcWalkContour(layer,x,y,cont)
    local w,h=layer.header.width,layer.header.height; cont.nverts=0
    local startX,startY,startDir=x,y,-1
    for i=0,3 do
        local dir0=bit32.band(i+3,3); local rn=tcNeighbourReg(layer,x,y,dir0)
        if rn~=buffer.readu8(layer.regs,x+y*w) then startDir=dir0; break end
    end
    if startDir==-1 then return true end
    local dir0=startDir; local maxIter=w*h; local iter=0
    while iter<maxIter do
        local rn=tcNeighbourReg(layer,x,y,dir0); local nx,ny,ndir=x,y,dir0
        if rn~=buffer.readu8(layer.regs,x+y*w) then
            local px,pz=x,y
            if dir0==0 then pz+=1 elseif dir0==1 then px+=1; pz+=1 elseif dir0==2 then px+=1 end
            if not tcAppendVertex(cont,px,buffer.readu8(layer.heights,x+y*w),pz,rn) then return false end
            ndir=bit32.band(dir0+1,3)
        else
            nx=x+TC_DIR_X[dir0+1]; ny=y+TC_DIR_Y[dir0+1]; ndir=bit32.band(dir0+3,3)
        end
        if iter>0 and x==startX and y==startY and dir0==startDir then break end
        x,y,dir0=nx,ny,ndir; iter+=1
    end
    if cont.nverts>0 then
        local pa=(cont.nverts-1)*4
        if buffer.readu8(cont.verts,pa)==buffer.readu8(cont.verts,0) and buffer.readu8(cont.verts,pa+2)==buffer.readu8(cont.verts,2) then cont.nverts-=1 end
    end
    return true
end

local function tcDistancePtSeg(x,z,px,pz,qx,qz)
    -- Upstream performs this routine entirely in float.  Luau numbers are doubles, which changes
    -- contour decisions exactly at maxError boundaries; vector arithmetic gives us the runtime's
    -- native float32 lane without a buffer round-trip.
    local pq=vector.create(qx-px,0,qz-pz)
    local delta=vector.create(x-px,0,z-pz)
    local d=vector.dot(pq,pq); local t=vector.dot(pq,delta)
    if d>0 then t=vector.create(t/d,0,0).X end
    t=math.clamp(t,0,1)
    local e=vector.create(px,0,pz)+pq*t-vector.create(x,0,z)
    return vector.dot(e,e)
end

local function tcSimplifyContour(cont,maxError)
    cont.npoly=0
    local maxErrorF=vector.create(maxError,0,0)
    local maxErrorSq=vector.dot(maxErrorF,maxErrorF)
    for i=0,cont.nverts-1 do
        local j=(i+1)%cont.nverts; local ra=buffer.readu8(cont.verts,j*4+3); local rb=buffer.readu8(cont.verts,i*4+3)
        if ra~=rb then buffer.writeu16(cont.poly,cont.npoly*2,i); cont.npoly+=1 end
    end
    if cont.npoly<2 then
        local llx,llz,lli=buffer.readu8(cont.verts,0),buffer.readu8(cont.verts,2),0
        local urx,urz,uri=llx,llz,0
        for i=1,cont.nverts-1 do
            local x,z=buffer.readu8(cont.verts,i*4),buffer.readu8(cont.verts,i*4+2)
            if x<llx or (x==llx and z<llz) then llx,llz,lli=x,z,i end
            if x>urx or (x==urx and z>urz) then urx,urz,uri=x,z,i end
        end
        cont.npoly=2; buffer.writeu16(cont.poly,0,lli); buffer.writeu16(cont.poly,2,uri)
    end
    local i=0
    while i<cont.npoly do
        local ii=(i+1)%cont.npoly; local ai=buffer.readu16(cont.poly,i*2); local bi=buffer.readu16(cont.poly,ii*2)
        local ax,az=buffer.readu8(cont.verts,ai*4),buffer.readu8(cont.verts,ai*4+2)
        local bx,bz=buffer.readu8(cont.verts,bi*4),buffer.readu8(cont.verts,bi*4+2)
        local maxd,maxi=0,-1; local ci,cinc,endi
        if bx>ax or (bx==ax and bz>az) then cinc=1; ci=(ai+cinc)%cont.nverts; endi=bi
        else cinc=cont.nverts-1; ci=(bi+cinc)%cont.nverts; endi=ai end
        while ci~=endi do
            local d=tcDistancePtSeg(buffer.readu8(cont.verts,ci*4),buffer.readu8(cont.verts,ci*4+2),ax,az,bx,bz)
            if d>maxd then maxd,maxi=d,ci end
            ci=(ci+cinc)%cont.nverts
        end
        if maxi~=-1 and maxd>maxErrorSq then
            cont.npoly+=1
            for j=cont.npoly-1,i+1,-1 do buffer.writeu16(cont.poly,j*2,buffer.readu16(cont.poly,(j-1)*2)) end
            buffer.writeu16(cont.poly,(i+1)*2,maxi)
        else i+=1 end
    end
    local start=0
    for j=1,cont.npoly-1 do if buffer.readu16(cont.poly,j*2)<buffer.readu16(cont.poly,start*2) then start=j end end
    local remap=buffer.create(cont.npoly*4)
    for j=0,cont.npoly-1 do
        local pi=buffer.readu16(cont.poly,((start+j)%cont.npoly)*2); buffer.copy(remap,j*4,cont.verts,pi*4,4)
    end
    cont.nverts=cont.npoly; buffer.copy(cont.verts,0,remap,0,cont.npoly*4)
end

local function tcCornerHeight(layer,x,y,z,walkableClimb)
    local w,h=layer.header.width,layer.header.height; local n=0; local portal=0x0f; local height=0; local preg=0xff; local allSame=true
    for dz=-1,0 do for dx=-1,0 do
        local px,pz=x+dx,z+dz
        if px>=0 and pz>=0 and px<w and pz<h then
            local idx=px+pz*w; local lh=buffer.readu8(layer.heights,idx)
            if math.abs(lh-y)<=walkableClimb and buffer.readu8(layer.areas,idx)~=TileCache.DT_TILECACHE_NULL_AREA then
                height=math.max(height,lh); portal=bit32.band(portal,bit32.rshift(buffer.readu8(layer.cons,idx),4))
                local rr=buffer.readu8(layer.regs,idx); if preg~=0xff and preg~=rr then allSame=false end; preg=rr; n+=1
            end
        end
    end end
    local portalCount=0; for dir0=0,3 do if bit32.band(portal,bit32.lshift(1,dir0))~=0 then portalCount+=1 end end
    return height,n>1 and portalCount==1 and allSame
end

function TileCache.buildTileCacheContours(layer,walkableClimb,maxError)
    maxError=maxError or 1.3
    local w,h=layer.header.width,layer.header.height; local cset={nconts=layer.regCount,conts=table.create(layer.regCount)}
    for i=0,layer.regCount-1 do cset.conts[i+1]={nverts=0,verts=nil,reg=0,area=0} end
    local maxTempVerts=(w+h)*4
    local temp={verts=buffer.create(maxTempVerts*4),nverts=0,cverts=maxTempVerts,poly=buffer.create(maxTempVerts*2),npoly=0,cpoly=maxTempVerts}
    for y=0,h-1 do for x=0,w-1 do
        local idx=x+y*w; local ri=buffer.readu8(layer.regs,idx); if ri==0xff then continue end
        local cont=cset.conts[ri+1]; if cont.nverts>0 then continue end
        cont.reg=ri; cont.area=buffer.readu8(layer.areas,idx)
        if not tcWalkContour(layer,x,y,temp) then return nil,bit32.bor(TileCache.DT_FAILURE,TileCache.DT_BUFFER_TOO_SMALL) end
        tcSimplifyContour(temp,maxError); cont.nverts=temp.nverts
        if cont.nverts>0 then
            cont.verts=buffer.create(cont.nverts*4)
            local i,j=0,temp.nverts-1
            while i<temp.nverts do
                local vo=j*4; local vno=i*4; local nei=buffer.readu8(temp.verts,vno+3)
                local lh,remove=tcCornerHeight(layer,buffer.readu8(temp.verts,vo),buffer.readu8(temp.verts,vo+1),buffer.readu8(temp.verts,vo+2),walkableClimb)
                buffer.writeu8(cont.verts,vo,buffer.readu8(temp.verts,vo)); buffer.writeu8(cont.verts,vo+1,lh); buffer.writeu8(cont.verts,vo+2,buffer.readu8(temp.verts,vo+2))
                local meta=0x0f; if nei~=0xff and nei>=0xf8 then meta=nei-0xf8 end; if remove then meta=bit32.bor(meta,0x80) end
                buffer.writeu8(cont.verts,vo+3,meta); j=i; i+=1
            end
        end
    end end
    return cset,TileCache.DT_SUCCESS
end

local TC_MAX_VERTS_PER_POLY=6
local TC_MAX_REM_EDGES=48
local TC_NULL_IDX=0xffff

local function tcPrev(i,n) return if i-1>=0 then i-1 else n-1 end
local function tcNext(i,n) return if i+1<n then i+1 else 0 end
local function tcV(contVerts,i)
    local o=i*4
    return buffer.readu8(contVerts,o),buffer.readu8(contVerts,o+1),buffer.readu8(contVerts,o+2)
end
local function tcArea2(verts,a,b,c)
    local ax,_,az=tcV(verts,a); local bx,_,bz=tcV(verts,b); local cx,_,cz=tcV(verts,c)
    return (bx-ax)*(cz-az)-(cx-ax)*(bz-az)
end
local function tcLeft(verts,a,b,c) return tcArea2(verts,a,b,c)<0 end
local function tcLeftOn(verts,a,b,c) return tcArea2(verts,a,b,c)<=0 end
local function tcCollinear(verts,a,b,c) return tcArea2(verts,a,b,c)==0 end
local function tcVequal(verts,a,b)
    local ax,_,az=tcV(verts,a); local bx,_,bz=tcV(verts,b); return ax==bx and az==bz
end
local function tcBetween(verts,a,b,c)
    if not tcCollinear(verts,a,b,c) then return false end
    local ax,_,az=tcV(verts,a); local bx,_,bz=tcV(verts,b); local cx,_,cz=tcV(verts,c)
    if ax~=bx then return (ax<=cx and cx<=bx) or (ax>=cx and cx>=bx) end
    return (az<=cz and cz<=bz) or (az>=cz and cz>=bz)
end
local function tcIntersectProp(verts,a,b,c,d)
    if tcCollinear(verts,a,b,c) or tcCollinear(verts,a,b,d) or tcCollinear(verts,c,d,a) or tcCollinear(verts,c,d,b) then return false end
    return (tcLeft(verts,a,b,c)~=tcLeft(verts,a,b,d)) and (tcLeft(verts,c,d,a)~=tcLeft(verts,c,d,b))
end
local function tcIntersect(verts,a,b,c,d)
    return tcIntersectProp(verts,a,b,c,d) or tcBetween(verts,a,b,c) or tcBetween(verts,a,b,d) or tcBetween(verts,c,d,a) or tcBetween(verts,c,d,b)
end
local function tcDiagonalie(i,j,n,verts,indices)
    local d0=bit32.band(indices[i+1],0x7fff); local d1=bit32.band(indices[j+1],0x7fff)
    for k=0,n-1 do
        local k1=tcNext(k,n)
        if k~=i and k1~=i and k~=j and k1~=j then
            local p0=bit32.band(indices[k+1],0x7fff); local p1=bit32.band(indices[k1+1],0x7fff)
            if not (tcVequal(verts,d0,p0) or tcVequal(verts,d1,p0) or tcVequal(verts,d0,p1) or tcVequal(verts,d1,p1)) and tcIntersect(verts,d0,d1,p0,p1) then return false end
        end
    end
    return true
end
local function tcInCone(i,j,n,verts,indices)
    local pi=bit32.band(indices[i+1],0x7fff); local pj=bit32.band(indices[j+1],0x7fff)
    local pi1=bit32.band(indices[tcNext(i,n)+1],0x7fff); local pin1=bit32.band(indices[tcPrev(i,n)+1],0x7fff)
    if tcLeftOn(verts,pin1,pi,pi1) then return tcLeft(verts,pi,pj,pin1) and tcLeft(verts,pj,pi,pi1) end
    return not (tcLeftOn(verts,pi,pj,pi1) and tcLeftOn(verts,pj,pi,pin1))
end
local function tcDiagonal(i,j,n,verts,indices) return tcInCone(i,j,n,verts,indices) and tcDiagonalie(i,j,n,verts,indices) end

local function tcTriangulate(n,verts,indices)
    local tris={}; local ntris=0
    for i=0,n-1 do
        local i1=tcNext(i,n); local i2=tcNext(i1,n)
        if tcDiagonal(i,i2,n,verts,indices) then indices[i1+1]=bit32.bor(indices[i1+1],0x8000) end
    end
    while n>3 do
        local minLen,mini=-1,-1
        for i=0,n-1 do
            local i1=tcNext(i,n)
            if bit32.band(indices[i1+1],0x8000)~=0 then
                local p0=bit32.band(indices[i+1],0x7fff); local p2=bit32.band(indices[tcNext(i1,n)+1],0x7fff)
                local x0,_,z0=tcV(verts,p0); local x2,_,z2=tcV(verts,p2); local dx,dz=x2-x0,z2-z0; local len=dx*dx+dz*dz
                if minLen<0 or len<minLen then minLen,mini=len,i end
            end
        end
        if mini==-1 then return tris,-ntris end
        local i=mini; local i1=tcNext(i,n); local i2=tcNext(i1,n)
        tris[#tris+1]=bit32.band(indices[i+1],0x7fff); tris[#tris+1]=bit32.band(indices[i1+1],0x7fff); tris[#tris+1]=bit32.band(indices[i2+1],0x7fff); ntris+=1
        n-=1; for k=i1,n-1 do indices[k+1]=indices[k+2] end; indices[n+1]=nil
        if i1>=n then i1=0 end; i=tcPrev(i1,n)
        if tcDiagonal(tcPrev(i,n),i1,n,verts,indices) then indices[i+1]=bit32.bor(indices[i+1],0x8000) else indices[i+1]=bit32.band(indices[i+1],0x7fff) end
        if tcDiagonal(i,tcNext(i1,n),n,verts,indices) then indices[i1+1]=bit32.bor(indices[i1+1],0x8000) else indices[i1+1]=bit32.band(indices[i1+1],0x7fff) end
    end
    tris[#tris+1]=bit32.band(indices[1],0x7fff); tris[#tris+1]=bit32.band(indices[2],0x7fff); tris[#tris+1]=bit32.band(indices[3],0x7fff); ntris+=1
    return tris,ntris
end

local function tcPolyCount(p) return #p end
local function tcULeft(verts,a,b,c)
    local va,vb,vc=verts[a+1],verts[b+1],verts[c+1]
    return (vb.x-va.x)*(vc.z-va.z)-(vc.x-va.x)*(vb.z-va.z)<0
end
local function tcPolyMergeValue(pa,pb,verts)
    local na,nb=#pa,#pb; if na+nb-2>TC_MAX_VERTS_PER_POLY then return -1,-1,-1 end
    local ea,eb=-1,-1
    for i=0,na-1 do
        local va0,va1=pa[i+1],pa[(i+1)%na+1]; if va0>va1 then va0,va1=va1,va0 end
        for j=0,nb-1 do
            local vb0,vb1=pb[j+1],pb[(j+1)%nb+1]; if vb0>vb1 then vb0,vb1=vb1,vb0 end
            if va0==vb0 and va1==vb1 then ea,eb=i,j; break end
        end
    end
    if ea==-1 or eb==-1 then return -1,ea,eb end
    local va=pa[(ea+na-1)%na+1]; local vb=pa[ea+1]; local vc=pb[(eb+2)%nb+1]; if not tcULeft(verts,va,vb,vc) then return -1,ea,eb end
    va=pb[(eb+nb-1)%nb+1]; vb=pb[eb+1]; vc=pa[(ea+2)%na+1]; if not tcULeft(verts,va,vb,vc) then return -1,ea,eb end
    va=pa[ea+1]; vb=pa[(ea+1)%na+1]; local a,b=verts[va+1],verts[vb+1]; local dx,dz=a.x-b.x,a.z-b.z
    return dx*dx+dz*dz,ea,eb
end
local function tcMergePolys(pa,pb,ea,eb)
    local na,nb=#pa,#pb; local tmp={}
    for i=0,na-2 do tmp[#tmp+1]=pa[(ea+1+i)%na+1] end
    for i=0,nb-2 do tmp[#tmp+1]=pb[(eb+1+i)%nb+1] end
    return tmp
end
local function tcMergePolyList(polys,npolys,verts,areas)
    while true do
        local bestVal,bpa,bpb,bea,beb=0,0,0,0,0
        for j=0,npolys-2 do for k=j+1,npolys-1 do
            local v,ea,eb=tcPolyMergeValue(polys[j+1],polys[k+1],verts)
            if v>bestVal then bestVal,bpa,bpb,bea,beb=v,j,k,ea,eb end
        end end
        if bestVal<=0 then break end
        polys[bpa+1]=tcMergePolys(polys[bpa+1],polys[bpb+1],bea,beb)
        polys[bpb+1]=polys[npolys]
        if areas~=nil then areas[bpb+1]=areas[npolys]; areas[npolys]=nil end
        polys[npolys]=nil; npolys-=1
    end
    return npolys
end

local function tcCanRemoveVertex(polys,npolys,rem)
    local touched,remaining=0,0
    for i=1,npolys do local p=polys[i]; local removed=0; for _,v in p do if v==rem then touched+=1; removed+=1 end end; if removed>0 then remaining+=#p-(removed+1) end end
    if remaining<=2 or touched*2>TC_MAX_REM_EDGES then return false end
    local edges={}; local nedges=0
    for i=1,npolys do
        local p=polys[i]; local nv=#p
        for j=0,nv-1 do local k=tcPrev(j,nv); if p[j+1]==rem or p[k+1]==rem then
            local a,b=p[j+1],p[k+1]; if b==rem then a,b=b,a end
            local found=false
            for m=1,nedges do if edges[m].b==b then edges[m].count+=1; found=true end end
            if not found then nedges+=1; edges[nedges]={a=a,b=b,count=1} end
        end end
    end
    local open=0; for i=1,nedges do if edges[i].count<2 then open+=1 end end
    return open<=2
end

local function tcContourBufferFromVerts(vs)
    local b=buffer.create(#vs*4); for i,v in vs do local o=(i-1)*4; buffer.writeu8(b,o,v.x);buffer.writeu8(b,o+1,v.y);buffer.writeu8(b,o+2,v.z) end; return b
end

local function tcRemoveVertex(verts,polys,areas,npolys,rem,maxTris)
    local edges={}; local nedges=0; local i=1
    while i<=npolys do
        local p=polys[i]; local has=false; for _,v in p do if v==rem then has=true; break end end
        if has then
            local nv=#p
            for j=0,nv-1 do local k=tcPrev(j,nv); if p[j+1]~=rem and p[k+1]~=rem then
                if nedges>=TC_MAX_REM_EDGES then return npolys,bit32.bor(TileCache.DT_FAILURE,TileCache.DT_BUFFER_TOO_SMALL) end
                nedges+=1; edges[nedges]={a=p[k+1],b=p[j+1],area=areas[i]}
            end end
            polys[i]=polys[npolys]; areas[i]=areas[npolys]; polys[npolys]=nil; areas[npolys]=nil; npolys-=1
        else i+=1 end
    end
    table.remove(verts,rem+1)
    for pi=1,npolys do for j,v in polys[pi] do if v>rem then polys[pi][j]=v-1 end end end
    for e=1,nedges do if edges[e].a>rem then edges[e].a-=1 end; if edges[e].b>rem then edges[e].b-=1 end end
    if nedges==0 then return npolys,TileCache.DT_SUCCESS end
    -- Upstream seeds the hole with the first edge's A vertex, but intentionally leaves
    -- that edge in the work list.  The first stitching pass consumes it and appends B.
    local hole={edges[1].a}; local harea={edges[1].area}
    while nedges>0 do
        local match=false; local eidx=1
        while eidx<=nedges do
            local e=edges[eidx]; local add=false
            if hole[1]==e.b then
                if #hole>=TC_MAX_REM_EDGES then return npolys,bit32.bor(TileCache.DT_FAILURE,TileCache.DT_BUFFER_TOO_SMALL) end
                table.insert(hole,1,e.a); table.insert(harea,1,e.area); add=true
            elseif hole[#hole]==e.a then
                if #hole>=TC_MAX_REM_EDGES then return npolys,bit32.bor(TileCache.DT_FAILURE,TileCache.DT_BUFFER_TOO_SMALL) end
                hole[#hole+1]=e.b; harea[#harea+1]=e.area; add=true
            end
            if add then edges[eidx]=edges[nedges];edges[nedges]=nil;nedges-=1;match=true else eidx+=1 end
        end
        if not match then break end
    end
    local localVerts={}; local indices={}
    for hi,pi in hole do local v=verts[pi+1];localVerts[hi]={x=v.x,y=v.y,z=v.z};indices[hi]=hi-1 end
    local vbuf=tcContourBufferFromVerts(localVerts); local tris,ntris=tcTriangulate(#hole,vbuf,indices); if ntris<0 then ntris=-ntris end
    if ntris>TC_MAX_REM_EDGES then return npolys,bit32.bor(TileCache.DT_FAILURE,TileCache.DT_BUFFER_TOO_SMALL) end
    local localPolys,localAreas={},{}; local ln=0
    for t=0,ntris-1 do local a,b,c=tris[t*3+1],tris[t*3+2],tris[t*3+3]; if a~=b and a~=c and b~=c then ln+=1;localPolys[ln]={hole[a+1],hole[b+1],hole[c+1]};localAreas[ln]=harea[a+1] end end
    if ln==0 then return npolys,TileCache.DT_SUCCESS end
    ln=tcMergePolyList(localPolys,ln,verts,localAreas)
    for li=1,ln do if npolys>=maxTris then break end;npolys+=1;polys[npolys]=localPolys[li];areas[npolys]=localAreas[li];if npolys>maxTris then return npolys,bit32.bor(TileCache.DT_FAILURE,TileCache.DT_BUFFER_TOO_SMALL) end end
    return npolys,TileCache.DT_SUCCESS
end

local function tcOverlapExclusive(amin,amax,bmin,bmax) return not (amin>=bmax or amax<=bmin) end
local function tcBuildAdjacency(polys,npolys,verts,cset)
    local edges={}; local edgeCount=0
    for i=0,npolys-1 do local p=polys[i+1];local nv=#p;for j=0,nv-1 do local v0,v1=p[j+1],p[(j+1)%nv+1];if v0<v1 then edgeCount+=1;edges[edgeCount]={v0=v0,v1=v1,p0=i,e0=j,p1=i,e1=0xff} end end end
    for i=0,npolys-1 do local p=polys[i+1];local nv=#p;for j=0,nv-1 do local v0,v1=p[j+1],p[(j+1)%nv+1];if v0>v1 then
        local found=false
        for e=1,edgeCount do local ed=edges[e];if ed.v0==v1 and ed.v1==v0 and ed.p0==ed.p1 then ed.p1=i;ed.e1=j;found=true;break end end
        if not found then edgeCount+=1;edges[edgeCount]={v0=v1,v1=v0,p0=i,e0=j,p1=i,e1=0xff} end
    end end end
    for _,cont in cset.conts do if cont.nverts>=3 then
        local k=cont.nverts-1
        for j=0,cont.nverts-1 do
            local ko,jo=k*4,j*4;local dir0=bit32.band(buffer.readu8(cont.verts,ko+3),0x0f)
            if dir0~=0x0f then
                local ax,az=buffer.readu8(cont.verts,ko),buffer.readu8(cont.verts,ko+2);local bx,bz=buffer.readu8(cont.verts,jo),buffer.readu8(cont.verts,jo+2)
                if dir0==0 or dir0==2 then local zmin,zmax=az,bz;if zmin>zmax then zmin,zmax=zmax,zmin end
                    for e=1,edgeCount do local ed=edges[e];if ed.p0==ed.p1 then local va,vb=verts[ed.v0+1],verts[ed.v1+1];if va.x==ax and vb.x==ax then local emin,emax=va.z,vb.z;if emin>emax then emin,emax=emax,emin end;if tcOverlapExclusive(zmin,zmax,emin,emax) then ed.e1=dir0 end end end end
                else local xmin,xmax=ax,bx;if xmin>xmax then xmin,xmax=xmax,xmin end
                    for e=1,edgeCount do local ed=edges[e];if ed.p0==ed.p1 then local va,vb=verts[ed.v0+1],verts[ed.v1+1];if va.z==az and vb.z==az then local emin,emax=va.x,vb.x;if emin>emax then emin,emax=emax,emin end;if tcOverlapExclusive(xmin,xmax,emin,emax) then ed.e1=dir0 end end end end
                end
            end
            k=j
        end
    end end
    local neis=table.create(npolys);for i=1,npolys do neis[i]=table.create(TC_MAX_VERTS_PER_POLY,TC_NULL_IDX);for e=1,TC_MAX_VERTS_PER_POLY do neis[i][e]=TC_NULL_IDX end end
    for _,ed in edges do if ed.p0~=ed.p1 then neis[ed.p0+1][ed.e0+1]=ed.p1;neis[ed.p1+1][ed.e1+1]=ed.p0 elseif ed.e1~=0xff then neis[ed.p0+1][ed.e0+1]=bit32.bor(0x8000,ed.e1) end end
    return neis
end

function TileCache.buildTileCachePolyMesh(cset, options)
    options=options or {}
    if cset==nil or cset.conts==nil then return nil,bit32.bor(TileCache.DT_FAILURE,TileCache.DT_INVALID_PARAM) end
    local maxVertices,maxTris,maxVertsPerCont=0,0,0
    for _,cont in cset.conts do if cont.nverts>=3 then maxVertices+=cont.nverts;maxTris+=cont.nverts-2;maxVertsPerCont=math.max(maxVertsPerCont,cont.nverts) end end
    local verts,polys,areas,vflags={}, {}, {}, {}; local nverts,npolys=0,0; local buckets={}
    local function addVertex(x,y,z)
        local key=x..'|'..z;local list=buckets[key]
        if list~=nil then for _,idx in list do local v=verts[idx+1];if math.abs(v.y-y)<=2 then return idx end end else list={};buckets[key]=list end
        local idx=nverts;nverts+=1;verts[nverts]={x=x,y=y,z=z};vflags[nverts]=false;table.insert(list,1,idx);return idx
    end
    for _,cont in cset.conts do if cont.nverts>=3 then
        local indices={};for j=0,cont.nverts-1 do indices[j+1]=j end
        local tris,ntris=tcTriangulate(cont.nverts,cont.verts,indices);if ntris<=0 then ntris=-ntris end
        local mapped={};for j=0,cont.nverts-1 do local o=j*4;local vi=addVertex(buffer.readu8(cont.verts,o),buffer.readu8(cont.verts,o+1),buffer.readu8(cont.verts,o+2));mapped[j+1]=vi;if bit32.band(buffer.readu8(cont.verts,o+3),0x80)~=0 then vflags[vi+1]=true end end
        local localPolys={};local ln=0
        for j=0,ntris-1 do local a,b,c=tris[j*3+1],tris[j*3+2],tris[j*3+3];if a~=b and a~=c and b~=c then ln+=1;localPolys[ln]={mapped[a+1],mapped[b+1],mapped[c+1]} end end
        if ln>0 then ln=tcMergePolyList(localPolys,ln,verts);for j=1,ln do npolys+=1;polys[npolys]=localPolys[j];areas[npolys]=cont.area;if npolys>maxTris then return nil,bit32.bor(TileCache.DT_FAILURE,TileCache.DT_BUFFER_TOO_SMALL) end end end
    end end
    if not options.skipVertexRemoval then
        local i=0
        while i<nverts do
            if vflags[i+1] and tcCanRemoveVertex(polys,npolys,i) then
                local status
                npolys,status=tcRemoveVertex(verts,polys,areas,npolys,i,maxTris);if TileCache.statusFailed(status) then return nil,status end
                nverts-=1;table.remove(vflags,i+1)
            else i+=1 end
        end
    end
    local neis=tcBuildAdjacency(polys,npolys,verts,cset)
    local mesh={
        nvp=TC_MAX_VERTS_PER_POLY,nverts=nverts,npolys=npolys,
        verts=buffer.create(nverts*6),polys=buffer.create(npolys*TC_MAX_VERTS_PER_POLY*2*2),areas=buffer.create(npolys),flags=buffer.create(npolys*2),
        bmin=options.bmin or cset.bmin or vector.zero,bmax=options.bmax or cset.bmax or vector.zero,
        cs=options.cs or cset.cs or 1,ch=options.ch or cset.ch or 1,borderSize=0,maxEdgeError=options.maxError or cset.maxError or 1.3,
        walkableClimb=options.walkableClimb or cset.walkableClimb or 0,
    }
    buffer.fill(mesh.polys,0,0xff,buffer.len(mesh.polys))
    for vi=0,nverts-1 do local v=verts[vi+1];local o=vi*6;buffer.writeu16(mesh.verts,o,v.x);buffer.writeu16(mesh.verts,o+2,v.y);buffer.writeu16(mesh.verts,o+4,v.z) end
    for pi=0,npolys-1 do local base=pi*TC_MAX_VERTS_PER_POLY*2;local p=polys[pi+1];for e=0,#p-1 do buffer.writeu16(mesh.polys,(base+e)*2,p[e+1]);buffer.writeu16(mesh.polys,(base+TC_MAX_VERTS_PER_POLY+e)*2,neis[pi+1][e+1]) end;buffer.writeu8(mesh.areas,pi,areas[pi+1]) end
    return mesh,TileCache.DT_SUCCESS
end


local function tcCountPolyVerts(mesh, p0)
    local base=p0*mesh.nvp*2
    local n=0
    for e0=0,mesh.nvp-1 do
        if buffer.readu16(mesh.polys,(base+e0)*2)==TileCache.DT_TILECACHE_NULL_IDX then break end
        n+=1
    end
    return n
end

local function tcMarkPortalAdjacency(mesh,cset)
    -- rcBuildPolyMesh has already connected ordinary shared edges.  TileCache carries portal
    -- directions in the low nibble of contour vertex metadata; upstream overlays those directions
    -- onto otherwise-open mesh edges as 0x8000|dir.
    for _,cont in cset.conts do
        if cont.nverts<3 or cont.verts==nil then continue end
        for j0=0,cont.nverts-1 do
            local k0=if j0>0 then j0-1 else cont.nverts-1
            local ko,jo=k0*4,j0*4
            local dir0=bit32.band(buffer.readu8(cont.verts,ko+3),0x0f)
            if dir0==0x0f then continue end
            local ax,az=buffer.readu8(cont.verts,ko),buffer.readu8(cont.verts,ko+2)
            local bx,bz=buffer.readu8(cont.verts,jo),buffer.readu8(cont.verts,jo+2)
            for p0=0,mesh.npolys-1 do
                local nv=tcCountPolyVerts(mesh,p0)
                local base=p0*mesh.nvp*2
                for e0=0,nv-1 do
                    local no=(base+mesh.nvp+e0)*2
                    if buffer.readu16(mesh.polys,no)~=TileCache.DT_TILECACHE_NULL_IDX then continue end
                    local va=buffer.readu16(mesh.polys,(base+e0)*2)
                    local vb=buffer.readu16(mesh.polys,(base+((e0+1)%nv))*2)
                    local vao,vbo=va*6,vb*6
                    local eax,eaz=buffer.readu16(mesh.verts,vao),buffer.readu16(mesh.verts,vao+4)
                    local ebx,ebz=buffer.readu16(mesh.verts,vbo),buffer.readu16(mesh.verts,vbo+4)
                    local match=false
                    if dir0==0 or dir0==2 then
                        if eax==ax and ebx==ax then
                            local amin,amax=math.min(az,bz),math.max(az,bz)
                            local bmin,bmax=math.min(eaz,ebz),math.max(eaz,ebz)
                            match=amin<bmax and amax>bmin
                        end
                    else
                        if eaz==az and ebz==az then
                            local amin,amax=math.min(ax,bx),math.max(ax,bx)
                            local bmin,bmax=math.min(eax,ebx),math.max(eax,ebx)
                            match=amin<bmax and amax>bmin
                        end
                    end
                    if match then buffer.writeu16(mesh.polys,no,bit32.bor(0x8000,dir0)) end
                end
            end
        end
    end
end

function TileCache.buildTileCachePolyMeshGeneric(cset, options)
    options=options or {}
    if cset==nil or cset.conts==nil then return nil,bit32.bor(TileCache.DT_FAILURE,TileCache.DT_INVALID_PARAM) end
    local generic={
        conts={}, nconts=0,
        bmin=options.bmin or cset.bmin or vector.zero,
        bmax=options.bmax or cset.bmax or vector.zero,
        cs=options.cs or cset.cs or 1,
        ch=options.ch or cset.ch or 1,
        width=options.width or cset.width or 0,
        height=options.height or cset.height or 0,
        borderSize=0,
        maxError=options.maxError or cset.maxError or 1.3,
    }
    for _,cont in cset.conts do
        if cont.nverts<3 or cont.verts==nil then continue end
        local verts=table.create(cont.nverts*4)
        for i0=0,cont.nverts-1 do
            local o=i0*4
            local meta=buffer.readu8(cont.verts,o+3)
            verts[o+1]=buffer.readu8(cont.verts,o)
            verts[o+2]=buffer.readu8(cont.verts,o+1)
            verts[o+3]=buffer.readu8(cont.verts,o+2)
            verts[o+4]=if bit32.band(meta,0x80)~=0 then Recast.RC_BORDER_VERTEX else 0
        end
        generic.conts[#generic.conts+1]={verts=verts,nverts=cont.nverts,reg=cont.reg,area=cont.area}
    end
    generic.nconts=#generic.conts
    local mesh=Recast.buildPolyMesh(generic,6)
    tcMarkPortalAdjacency(mesh,cset)
    return mesh,TileCache.DT_SUCCESS
end

local function layerToCompact(layer, params)
    local header=layer.header; local w,h=header.width,header.height; local cells=w*h
    local spanCount=0
    for i=0,cells-1 do if buffer.readu8(layer.heights,i)~=0xff then spanCount+=1 end end
    local cellIndex=buffer.create(cells*4); local cellCount=buffer.create(cells)
    local y=buffer.create(spanCount*2); local spanH=buffer.create(spanCount); local areas=buffer.create(spanCount); local con=buffer.create(spanCount*4)
    local cellSpan=buffer.create(cells*4); buffer.fill(cellSpan,0,0xff,buffer.len(cellSpan))
    local nextSpan=0
    for z=0,h-1 do for x=0,w-1 do
        local ci=x+z*w; buffer.writeu32(cellIndex,ci*4,nextSpan)
        local hv=buffer.readu8(layer.heights,ci)
        if hv~=0xff then
            buffer.writeu8(cellCount,ci,1); buffer.writei32(cellSpan,ci*4,nextSpan)
            buffer.writeu16(y,nextSpan*2,hv); buffer.writeu8(spanH,nextSpan,0xff); buffer.writeu8(areas,nextSpan,buffer.readu8(layer.areas,ci))
            nextSpan+=1
        end
    end end
    local dx={-1,0,1,0}; local dz={0,1,0,-1}
    for z=0,h-1 do for x=0,w-1 do
        local ci=x+z*w; local si=buffer.readi32(cellSpan,ci*4); if si<0 then continue end
        local lc=buffer.readu8(layer.cons,ci); local packed=0
        for dir0=0,3 do
            local connected=bit32.band(lc,bit32.lshift(1,dir0))~=0
            if connected then
                local nx,nz=x+dx[dir0+1],z+dz[dir0+1]
                connected=nx>=0 and nz>=0 and nx<w and nz<h and buffer.readi32(cellSpan,(nx+nz*w)*4)>=0
            end
            packed=bit32.bor(packed,bit32.lshift(connected and 0 or Recast.RC_NOT_CONNECTED,dir0*6))
        end
        buffer.writeu32(con,si*4,packed)
    end end
    return {
        width=w,height=h,spanCount=spanCount,cellIndex=cellIndex,cellCount=cellCount,y=y,h=spanH,areas=areas,con=con,
        bmin=header.bmin,bmax=header.bmax,cs=params.cs,ch=params.ch,borderSize=0,
        walkableHeight=math.max(1,math.ceil(params.walkableHeight/params.ch)), walkableClimb=math.floor(params.walkableClimb/params.ch),
    }
end

local function containsRef(list, ref)
    for _,v in list do if v==ref then return true end end
    return false
end

function TileCache.buildNavMeshTile(cache, ref, navmesh)
    if cache.params==nil then return bit32.bor(TileCache.DT_FAILURE,TileCache.DT_INVALID_PARAM) end
    local tile=TileCache.getTileByRef(cache,ref)
    if tile==nil or tile.header==nil then return bit32.bor(TileCache.DT_FAILURE,TileCache.DT_INVALID_PARAM) end
    local layer,status=TileCache.decompressTileCacheLayer(tile.data,cache.compressor)
    if TileCache.statusFailed(status) then return status end
    local p=cache.params
    for _,ob in cache.obstacles do
        if ob.state~=TileCache.DT_OBSTACLE_EMPTY and ob.state~=TileCache.DT_OBSTACLE_REMOVING and containsRef(ob.touched,ref) then
            if ob.type==TileCache.DT_OBSTACLE_CYLINDER then markLayerCylinder(layer,p.cs,p.ch,ob.position,ob.radius,ob.height,TileCache.DT_TILECACHE_NULL_AREA)
            elseif ob.type==TileCache.DT_OBSTACLE_BOX then markLayerBox(layer,p.cs,p.ch,ob.bmin,ob.bmax,TileCache.DT_TILECACHE_NULL_AREA)
            else markLayerOrientedBox(layer,p.cs,p.ch,ob.center,ob.halfExtents,ob.rotAux0,ob.rotAux1,TileCache.DT_TILECACHE_NULL_AREA) end
        end
    end
    local regionStatus=TileCache.buildTileCacheRegions(layer,math.floor(p.walkableClimb/p.ch))
    if TileCache.statusFailed(regionStatus) then return regionStatus end
    local cset,contourStatus=TileCache.buildTileCacheContours(layer,math.floor(p.walkableClimb/p.ch),p.maxSimplificationError)
    if TileCache.statusFailed(contourStatus) then return contourStatus end
    local mesh,meshStatus=TileCache.buildTileCachePolyMesh(cset,{
        bmin=layer.header.bmin,bmax=layer.header.bmax,cs=p.cs,ch=p.ch,
        width=layer.header.width,height=layer.header.height,maxError=p.maxSimplificationError,walkableClimb=p.walkableClimb,
    })
    if TileCache.statusFailed(meshStatus) then return meshStatus end

    navmesh=navmesh or cache.navmesh
    local oldRef=Detour.getTileRefAt(navmesh,layer.header.tx,layer.header.ty,layer.header.tlayer)

    -- Upstream removes any existing navmesh tile and leaves the slot empty when obstacle
    -- rasterization erases all polygons from this compressed layer.
    if mesh.npolys==0 then
        if oldRef~=0 then Detour.removeTile(navmesh,oldRef) end
        return TileCache.DT_SUCCESS
    end

    -- Mirror dtTileCache::buildNavMeshTile exactly: expose the mutable create-params plus the
    -- TileCache polymesh area/flag arrays to the mesh-process callback, then serialize through
    -- dtCreateNavMeshData-compatible code before adding the tile.  This is deliberately not the
    -- convenience buildPolyMeshNavMesh path: binary layout, userId/off-mesh fields, and callback
    -- mutations are observable Detour semantics.
    local createParams={
        verts=mesh.verts,vertCount=mesh.nverts,polys=mesh.polys,polyAreas=mesh.areas,polyFlags=mesh.flags,
        polyCount=mesh.npolys,nvp=6,
        walkableHeight=p.walkableHeight,walkableRadius=p.walkableRadius,walkableClimb=p.walkableClimb,
        tileX=layer.header.tx,tileY=layer.header.ty,tileLayer=layer.header.tlayer,
        cs=p.cs,ch=p.ch,buildBvTree=false,bmin=layer.header.bmin,bmax=layer.header.bmax,
    }
    if cache.meshProcess~=nil then cache.meshProcess(createParams,mesh.areas,mesh.flags) end

    local navData=NavMeshBuilder.createNavMeshData(createParams)
    if navData==nil then return TileCache.DT_FAILURE end

    if oldRef~=0 then Detour.removeTile(navmesh,oldRef) end
    local _,addStatus=Detour.addTileData(navmesh,navData)
    if Detour.statusFailed(addStatus) then return addStatus end
    return TileCache.DT_SUCCESS
end

function TileCache.buildNavMeshTilesAt(cache,tx,ty,navmesh)
    for _,ref in TileCache.getTilesAt(cache,tx,ty,32) do
        local status=TileCache.buildNavMeshTile(cache,ref,navmesh)
        if TileCache.statusFailed(status) then return status end
    end
    return TileCache.DT_SUCCESS
end

local function queueUpdate(cache,ref)
    -- Upstream checks capacity before duplicate suppression.  This odd ordering is observable
    -- through an obstacle's pending list when the 64-entry update queue is already full.
    if #cache.updates>=TileCache.DT_MAX_UPDATE then return false end
    if not containsRef(cache.updates,ref) then cache.updates[#cache.updates+1]=ref end
    return true
end

local function processCompressedRequests(cache)
    for _,req in cache.requests do
        local ob=TileCache.getObstacleByRef(cache,req.ref)
        if ob~=nil then
            if req.action=='add' then
                local bmin,bmax=TileCache.getObstacleBounds(cache,ob)
                local touched=TileCache.queryTiles(cache,bmin,bmax,TileCache.DT_MAX_TOUCHED_TILES)
                table.clear(ob.touched); table.clear(ob.pending)
                for _,ref in touched do
                    ob.touched[#ob.touched+1]=ref
                    if queueUpdate(cache,ref) then ob.pending[#ob.pending+1]=ref end
                end
            elseif req.action=='remove' then
                ob.state=TileCache.DT_OBSTACLE_REMOVING; table.clear(ob.pending)
                for _,ref in ob.touched do if queueUpdate(cache,ref) then ob.pending[#ob.pending+1]=ref end end
            end
        end
    end
    table.clear(cache.requests)
end

local function finishCompressedUpdate(cache,handledRef)
    for _,ob in cache.obstacles do
        if ob.state==TileCache.DT_OBSTACLE_PROCESSING or ob.state==TileCache.DT_OBSTACLE_REMOVING then
            for i=1,#ob.pending do
                if ob.pending[i]==handledRef then
                    ob.pending[i]=ob.pending[#ob.pending]; ob.pending[#ob.pending]=nil
                    break
                end
            end
            if #ob.pending==0 then
                if ob.state==TileCache.DT_OBSTACLE_PROCESSING then ob.state=TileCache.DT_OBSTACLE_PROCESSED else releaseObstacle(cache,ob) end
            end
        end
    end
end

local function updateCompressed(cache,dt,navmesh)
    if #cache.updates==0 and #cache.requests>0 then processCompressedRequests(cache) end
    local status=TileCache.DT_SUCCESS
    if #cache.updates>0 then
        local ref=table.remove(cache.updates,1)
        status=TileCache.buildNavMeshTile(cache,ref,navmesh or cache.navmesh)
        finishCompressedUpdate(cache,ref)
    end
    local upToDate=#cache.updates==0 and #cache.requests==0
    cache.dirty=not upToDate
    return status,upToDate,navmesh or cache.navmesh
end

-- Compatibility immediate rebuild; also flushes queued Detour-style obstacle requests.
function TileCache.rebuild(cache)
    if cache._compressed then
        for _,tile in cache.tiles do
            if tile.header~=nil then
                local status=TileCache.buildNavMeshTile(cache,TileCache.getTileRef(cache,tile),cache.navmesh)
                if TileCache.statusFailed(status) then return nil,status end
            end
        end
        return cache.navmesh
    end
    processRequests(cache)
    rebuildInternal(cache)
    finalizeStates(cache)
    return cache.navmesh
end

-- Upstream update() is incremental by tile.  With one compact-field backing tile this completes
-- in one call, while preserving PROCESSING/PROCESSED/REMOVING/EMPTY lifecycle semantics.
function TileCache.update(cache, dt, navmesh)
    if cache._compressed then return updateCompressed(cache,dt,navmesh) end
    if #cache.requests == 0 and not cache.dirty then return TileCache.DT_SUCCESS, true, cache.navmesh end
    processRequests(cache)
    rebuildInternal(cache)
    finalizeStates(cache)
    return TileCache.DT_SUCCESS, true, cache.navmesh
end

return table.freeze(TileCache)
