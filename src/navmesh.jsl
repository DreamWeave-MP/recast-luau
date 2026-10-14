--!native

-- Pure-Luau dtNavMesh identity/tile-management core.
-- This intentionally mirrors Detour's 32-bit dtPolyRef packing and tile lifecycle semantics.

local NavMesh = {}

NavMesh.DT_FAILURE = 0x80000000
NavMesh.DT_SUCCESS = 0x40000000
NavMesh.DT_WRONG_MAGIC = 0x01
NavMesh.DT_WRONG_VERSION = 0x02
NavMesh.DT_OUT_OF_MEMORY = 0x04
NavMesh.DT_INVALID_PARAM = 0x08
NavMesh.DT_BUFFER_TOO_SMALL = 0x10
NavMesh.DT_ALREADY_OCCUPIED = 0x80
NavMesh.DT_TILE_FREE_DATA = 0x01
NavMesh.DT_NAVMESH_MAGIC = 0x444e4156 -- 'DNAV'
NavMesh.DT_NAVMESH_VERSION = 7
NavMesh.DT_NAVMESH_STATE_MAGIC = 0x444e4d53 -- 'DNMS'
NavMesh.DT_NAVMESH_STATE_VERSION = 1

local function nextPow2(v)
    v = math.max(0, math.floor(v))
    if v == 0 then return 0 end
    local p = 1
    while p < v do p *= 2 end
    return p
end

local function ilog2(v)
    local r = 0
    while v > 1 do v = math.floor(v / 2); r += 1 end
    return r
end

local function pow2(bits)
    return 2 ^ bits
end

local function computeTileHash(x, y, mask)
    local h1 = bit32.band(0x8da6b343, mask)
    local h2 = bit32.band(0xd8163841, mask)
    return bit32.band(h1 * bit32.band(x, mask) + h2 * bit32.band(y, mask), mask)
end

local function tileKey(x, y, layer)
    return tostring(x) .. ':' .. tostring(y) .. ':' .. tostring(layer)
end

local function getTileSlot(nav, tile)
    if tile == nil then return nil end
    local slot = tile._slot
    if slot == nil or nav.tiles[slot + 1] ~= tile then return nil end
    return slot
end

function NavMesh.statusSucceed(status) return bit32.band(status, NavMesh.DT_SUCCESS) ~= 0 end
function NavMesh.statusFailed(status) return bit32.band(status, NavMesh.DT_FAILURE) ~= 0 end

function NavMesh.new(params)
    params = params or {}
    local maxTiles = math.floor(params.maxTiles or 0)
    local maxPolys = math.floor(params.maxPolys or 0)
    if maxTiles <= 0 or maxPolys <= 0 then
        return nil, bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM)
    end
    local tileBits = ilog2(nextPow2(maxTiles))
    local polyBits = ilog2(nextPow2(maxPolys))
    if tileBits + polyBits > 31 then
        return nil, bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM)
    end
    local saltBits = math.min(31, 32 - tileBits - polyBits)
    if saltBits < 10 then
        return nil, bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM)
    end

    local tiles = table.create(maxTiles)
    local free = table.create(maxTiles)
    for slot = maxTiles - 1, 0, -1 do
        tiles[slot + 1] = {_owner=false, _slot=slot, salt=1, active=false, _nextPos=-1}
        free[#free + 1] = slot
    end
    -- Represent Detour's linked freelist as a stack whose end is m_nextFree.
    local freeTop = maxTiles
    local tileLutSize = nextPow2(math.floor(maxTiles / 4))
    if tileLutSize == 0 then tileLutSize = 1 end
    local posLookup = table.create(tileLutSize, -1)

    return {
        orig = params.orig or vector.zero,
        tileWidth = params.tileWidth or 0,
        tileHeight = params.tileHeight or 0,
        maxTiles = maxTiles,
        maxPolys = maxPolys,
        tileBits = tileBits,
        polyBits = polyBits,
        saltBits = saltBits,
        tileMask = pow2(tileBits) - 1,
        polyMask = pow2(polyBits) - 1,
        saltMask = pow2(saltBits) - 1,
        tiles = tiles,
        byLocation = {},
        tileLutSize = tileLutSize, tileLutMask = tileLutSize - 1, posLookup = posLookup,
        free = free,
        freeTop = freeTop,
    }, NavMesh.DT_SUCCESS
end

function NavMesh.encodePolyId(nav, salt, tile, poly)
    return salt * pow2(nav.polyBits + nav.tileBits) + tile * pow2(nav.polyBits) + poly
end

function NavMesh.decodePolyId(nav, ref)
    local polyBase = pow2(nav.polyBits)
    local tileBase = pow2(nav.polyBits + nav.tileBits)
    local poly = ref % polyBase
    local tile = math.floor(ref / polyBase) % pow2(nav.tileBits)
    local salt = math.floor(ref / tileBase) % pow2(nav.saltBits)
    return salt, tile, poly
end

function NavMesh.decodePolyIdSalt(nav, ref)
    return math.floor(ref / pow2(nav.polyBits + nav.tileBits)) % pow2(nav.saltBits)
end

function NavMesh.decodePolyIdTile(nav, ref)
    return math.floor(ref / pow2(nav.polyBits)) % pow2(nav.tileBits)
end

function NavMesh.decodePolyIdPoly(nav, ref)
    return ref % pow2(nav.polyBits)
end

function NavMesh.calcTileLoc(nav, pos: vector)
    if nav.tileWidth == 0 or nav.tileHeight == 0 then return 0, 0 end
    return math.floor((pos.X - nav.orig.X) / nav.tileWidth), math.floor((pos.Z - nav.orig.Z) / nav.tileHeight)
end

function NavMesh.getParams(nav)
    return {
        orig = nav.orig,
        tileWidth = nav.tileWidth,
        tileHeight = nav.tileHeight,
        maxTiles = nav.maxTiles,
        maxPolys = nav.maxPolys,
    }
end

function NavMesh.getMaxTiles(nav) return nav.maxTiles end
function NavMesh.getTile(nav, index0)
    if index0 < 0 or index0 >= nav.maxTiles then return nil end
    return nav.tiles[index0 + 1]
end

function NavMesh.getTileRef(nav, tile)
    local slot = getTileSlot(nav, tile)
    if slot == nil then return 0 end
    return NavMesh.encodePolyId(nav, tile.salt, slot, 0)
end

function NavMesh.getPolyRefBase(nav, tile)
    return NavMesh.getTileRef(nav, tile)
end

function NavMesh.getTileByRef(nav, ref)
    if ref == nil or ref == 0 then return nil end
    local salt, slot = NavMesh.decodePolyId(nav, ref)
    if slot >= nav.maxTiles then return nil end
    local tile = nav.tiles[slot + 1]
    if tile.salt ~= salt then return nil end
    return tile
end

function NavMesh.getTileAt(nav, x, y, layer)
    layer = layer or 0
    local slot = nav.posLookup[computeTileHash(x, y, nav.tileLutMask) + 1]
    while slot ~= nil and slot >= 0 do
        local tile = nav.tiles[slot + 1]
        if tile.active and tile.x == x and tile.y == y and tile.layer == layer then return tile end
        slot = tile._nextPos
    end
    return nil
end

function NavMesh.getTilesAt(nav, x, y, maxTiles)
    local out = {}
    maxTiles = maxTiles or math.huge
    local slot = nav.posLookup[computeTileHash(x, y, nav.tileLutMask) + 1]
    while slot ~= nil and slot >= 0 do
        local tile = nav.tiles[slot + 1]
        if tile.active and tile.x == x and tile.y == y and #out < maxTiles then out[#out + 1] = tile end
        slot = tile._nextPos
    end
    return out
end

function NavMesh.getTileRefAt(nav, x, y, layer)
    local tile = NavMesh.getTileAt(nav, x, y, layer)
    return tile and NavMesh.getTileRef(nav, tile) or 0
end

-- Tile payload is intentionally a Luau-owned table here.  Required structural fields mirror
-- dtMeshHeader enough to make refs/state exact while the binary dtCreateNavMeshData port lands.
function NavMesh.addTile(nav, payload, flags, lastRef)
    if payload == nil then return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM) end
    local x, y, layer = payload.x or 0, payload.y or 0, payload.layer or 0
    local polyCount = payload.polyCount or (payload.polyCount == 0 and 0) or (payload.areas and buffer.len(payload.areas) or 0)
    if polyCount < 0 or polyCount > nav.maxPolys then
        return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM)
    end
    if NavMesh.getTileAt(nav, x, y, layer) ~= nil then
        return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_ALREADY_OCCUPIED)
    end

    local slot, salt
    if lastRef ~= nil and lastRef ~= 0 then
        salt, slot = NavMesh.decodePolyId(nav, lastRef)
        if slot >= nav.maxTiles then return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM) end
        local tile = nav.tiles[slot + 1]
        if tile.active then return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_ALREADY_OCCUPIED) end
        -- Remove requested slot from the freelist, matching Detour's lastRef restoration behavior.
        local found = nil
        for i = 1, nav.freeTop do if nav.free[i] == slot then found = i; break end end
        if found == nil then return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM) end
        -- Preserve freelist order exactly when removing an arbitrary lastRef slot.
        table.remove(nav.free, found)
        nav.freeTop -= 1
        tile.salt = salt
    else
        if nav.freeTop <= 0 then return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_OUT_OF_MEMORY) end
        slot = nav.free[nav.freeTop]
        nav.free[nav.freeTop] = nil
        nav.freeTop -= 1
    end

    local tile = nav.tiles[slot + 1]
    tile._owner = nav
    tile._slot = slot
    tile.active = true
    tile.x, tile.y, tile.layer = x, y, layer
    tile.userId = payload.userId or 0
    tile.polyCount = polyCount
    tile.flags = flags or 0
    tile.payload = payload
    tile.polyFlags = payload.flags
    tile.polyAreas = payload.areas
    local hash = computeTileHash(x, y, nav.tileLutMask)
    tile._nextPos = nav.posLookup[hash + 1]
    nav.posLookup[hash + 1] = slot
    nav.byLocation[tileKey(x, y, layer)] = tile
    return NavMesh.DT_SUCCESS, NavMesh.getTileRef(nav, tile), tile
end

function NavMesh.removeTile(nav, ref)
    if ref == nil or ref == 0 then return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM) end
    local salt, slot = NavMesh.decodePolyId(nav, ref)
    if slot >= nav.maxTiles then return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM) end
    local tile = nav.tiles[slot + 1]
    if not tile.active or tile.salt ~= salt then return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM) end
    local payload = tile.payload
    local hash = computeTileHash(tile.x, tile.y, nav.tileLutMask)
    local cur, prev = nav.posLookup[hash + 1], -1
    while cur ~= nil and cur >= 0 do
        if cur == slot then
            if prev >= 0 then nav.tiles[prev + 1]._nextPos = tile._nextPos else nav.posLookup[hash + 1] = tile._nextPos end
            break
        end
        prev = cur; cur = nav.tiles[cur + 1]._nextPos
    end
    nav.byLocation[tileKey(tile.x, tile.y, tile.layer)] = nil
    tile.active = false
    tile.payload, tile.polyFlags, tile.polyAreas = nil, nil, nil
    tile.flags = 0
    tile.polyCount = 0
    tile._nextPos = -1
    tile.salt = (tile.salt + 1) % pow2(nav.saltBits)
    if tile.salt == 0 then tile.salt = 1 end
    nav.freeTop += 1
    nav.free[nav.freeTop] = slot
    return NavMesh.DT_SUCCESS, payload
end

function NavMesh.isValidPolyRef(nav, ref)
    if ref == nil or ref == 0 then return false end
    local salt, slot, poly = NavMesh.decodePolyId(nav, ref)
    if slot >= nav.maxTiles then return false end
    local tile = nav.tiles[slot + 1]
    return tile.active and tile.salt == salt and poly < tile.polyCount
end

function NavMesh.getTileAndPolyByRef(nav, ref)
    if not NavMesh.isValidPolyRef(nav, ref) then return nil end
    local _, slot, poly = NavMesh.decodePolyId(nav, ref)
    return nav.tiles[slot + 1], poly
end

function NavMesh.getTileAndPolyByRefUnsafe(nav, ref)
    local _, slot, poly = NavMesh.decodePolyId(nav, ref)
    return nav.tiles[slot + 1], poly
end

function NavMesh.getPolyFlags(nav, ref)
    local tile, poly = NavMesh.getTileAndPolyByRef(nav, ref)
    if tile == nil then return nil, bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM) end
    local flags = tile.polyFlags
    return flags and buffer.readu16(flags, poly * 2) or 0, NavMesh.DT_SUCCESS
end

function NavMesh.setPolyFlags(nav, ref, value)
    local tile, poly = NavMesh.getTileAndPolyByRef(nav, ref)
    if tile == nil then return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM) end
    local flags = tile.polyFlags
    if flags == nil then flags = buffer.create(tile.polyCount * 2); tile.polyFlags = flags end
    buffer.writeu16(flags, poly * 2, value)
    if tile.payload ~= nil then tile.payload.flags = flags end
    return NavMesh.DT_SUCCESS
end

function NavMesh.getPolyArea(nav, ref)
    local tile, poly = NavMesh.getTileAndPolyByRef(nav, ref)
    if tile == nil then return nil, bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM) end
    local areas = tile.polyAreas
    return areas and buffer.readu8(areas, poly) or 0, NavMesh.DT_SUCCESS
end

function NavMesh.setPolyArea(nav, ref, area)
    if area < 0 or area > 63 then return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM) end
    local tile, poly = NavMesh.getTileAndPolyByRef(nav, ref)
    if tile == nil then return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM) end
    local areas = tile.polyAreas
    if areas == nil then areas = buffer.create(tile.polyCount); tile.polyAreas = areas end
    buffer.writeu8(areas, poly, area)
    if tile.payload ~= nil then tile.payload.areas = areas end
    return NavMesh.DT_SUCCESS
end

function NavMesh.getTileStateSize(tile)
    if tile == nil or not tile.active then return 0 end
    -- dtTileState is 12 bytes and dtPolyState is 4 bytes with C++ alignment.
    return 12 + tile.polyCount * 4
end

function NavMesh.storeTileState(nav, tile)
    if tile == nil or not tile.active then return nil, bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM) end
    local out = buffer.create(NavMesh.getTileStateSize(tile))
    buffer.writei32(out, 0, NavMesh.DT_NAVMESH_STATE_MAGIC)
    buffer.writei32(out, 4, NavMesh.DT_NAVMESH_STATE_VERSION)
    buffer.writeu32(out, 8, NavMesh.getTileRef(nav, tile))
    local flags, areas = tile.polyFlags, tile.polyAreas
    for p0 = 0, tile.polyCount - 1 do
        local o = 12 + p0 * 4
        buffer.writeu16(out, o, flags and buffer.readu16(flags, p0 * 2) or 0)
        buffer.writeu8(out, o + 2, areas and buffer.readu8(areas, p0) or 0)
    end
    return out, NavMesh.DT_SUCCESS
end

function NavMesh.restoreTileState(nav, tile, state)
    if tile == nil or not tile.active or state == nil or buffer.len(state) < NavMesh.getTileStateSize(tile) then
        return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM)
    end
    if buffer.readi32(state, 0) ~= NavMesh.DT_NAVMESH_STATE_MAGIC then
        return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_WRONG_MAGIC)
    end
    if buffer.readi32(state, 4) ~= NavMesh.DT_NAVMESH_STATE_VERSION then
        return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_WRONG_VERSION)
    end
    if buffer.readu32(state, 8) ~= NavMesh.getTileRef(nav, tile) then
        return bit32.bor(NavMesh.DT_FAILURE, NavMesh.DT_INVALID_PARAM)
    end
    local flags, areas = tile.polyFlags, tile.polyAreas
    if flags == nil then flags = buffer.create(tile.polyCount * 2); tile.polyFlags = flags end
    if areas == nil then areas = buffer.create(tile.polyCount); tile.polyAreas = areas end
    for p0 = 0, tile.polyCount - 1 do
        local o = 12 + p0 * 4
        buffer.writeu16(flags, p0 * 2, buffer.readu16(state, o))
        buffer.writeu8(areas, p0, buffer.readu8(state, o + 2))
    end
    return NavMesh.DT_SUCCESS
end

return table.freeze(NavMesh)
