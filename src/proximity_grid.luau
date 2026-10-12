--!native

-- Altered pure-Luau implementation of DetourCrowd's dtProximityGrid.
-- Pool and hash buckets use packed buffers so the runtime shape stays stable under crowd load.

local Grid = {}

local NULL = 0xffff
local ITEM_STRIDE = 8 -- coord:u32 (x:u16|y:u16), links:u32 (id:u16|next:u16)

local function nextPow2(v)
    v = math.max(1, math.floor(v))
    v -= 1
    v = bit32.bor(v, bit32.rshift(v, 1))
    v = bit32.bor(v, bit32.rshift(v, 2))
    v = bit32.bor(v, bit32.rshift(v, 4))
    v = bit32.bor(v, bit32.rshift(v, 8))
    v = bit32.bor(v, bit32.rshift(v, 16))
    return v + 1
end

local function hashPos2(x, y, n)
    return bit32.band(bit32.bxor(x * 73856093, y * 19349663), n - 1)
end

local function itemOffset(index)
    return index * ITEM_STRIDE
end

local function coordKey(x,y)
    return bit32.bor(bit32.band(x,0xffff),bit32.lshift(bit32.band(y,0xffff),16))
end

local function bucketRead(grid, hash)
    return buffer.readu16(grid.buckets, hash * 2)
end

local function bucketWrite(grid, hash, value)
    buffer.writeu16(grid.buckets, hash * 2, value)
end

function Grid.new(poolSize, cellSize)
    assert(poolSize > 0, 'poolSize must be positive')
    assert(cellSize > 0, 'cellSize must be positive')
    if poolSize >= NULL then error('poolSize must be < 65535', 2) end
    local bucketsSize = nextPow2(poolSize)
    local grid = {
        cellSize = cellSize,
        invCellSize = 1 / cellSize,
        poolSize = math.floor(poolSize),
        poolHead = 0,
        bucketsSize = bucketsSize,
        buckets = buffer.create(bucketsSize * 2),
        pool = buffer.create(math.floor(poolSize) * ITEM_STRIDE),
        minx = NULL,
        miny = NULL,
        maxx = -NULL,
        maxy = -NULL,
        seen = {},
        querySerial = 0,
    }
    return Grid.clear(grid)
end

function Grid.clear(grid)
    buffer.fill(grid.buckets, 0, 0xff)
    grid.poolHead = 0
    grid.minx, grid.miny, grid.maxx, grid.maxy = NULL, NULL, -NULL, -NULL
    return grid
end

function Grid.addItem(grid, id, minx, miny, maxx, maxy)
    if id < 0 or id >= NULL then return false end
    local inv = grid.invCellSize
    local iminx, iminy = math.floor(minx * inv), math.floor(miny * inv)
    local imaxx, imaxy = math.floor(maxx * inv), math.floor(maxy * inv)
    grid.minx, grid.miny = math.min(grid.minx, iminx), math.min(grid.miny, iminy)
    grid.maxx, grid.maxy = math.max(grid.maxx, imaxx), math.max(grid.maxy, imaxy)

    local complete = true
    for y = iminy, imaxy do
        for x = iminx, imaxx do
            if grid.poolHead >= grid.poolSize then
                complete = false
                continue
            end
            local h = hashPos2(x, y, grid.bucketsSize)
            local index = grid.poolHead
            grid.poolHead += 1
            local o = itemOffset(index)
            buffer.writeu32(grid.pool,o,coordKey(x,y))
            buffer.writeu32(grid.pool,o+4,bit32.bor(id,bit32.lshift(bucketRead(grid,h),16)))
            bucketWrite(grid, h, index)
        end
    end
    return complete
end

function Grid.queryItems(grid, minx, miny, maxx, maxy, maxIds, out)
    maxIds = maxIds or 32
    out = out or {}
    table.clear(out)
    local serial = grid.querySerial + 1
    grid.querySerial = serial
    local seen = grid.seen
    local inv = grid.invCellSize
    local iminx, iminy = math.floor(minx * inv), math.floor(miny * inv)
    local imaxx, imaxy = math.floor(maxx * inv), math.floor(maxy * inv)

    for y = iminy, imaxy do
        for x = iminx, imaxx do
            local h = hashPos2(x, y, grid.bucketsSize)
            local index = bucketRead(grid, h)
            local key=coordKey(x,y)
            while index ~= NULL do
                local o = itemOffset(index)
                local links=buffer.readu32(grid.pool,o+4)
                if buffer.readu32(grid.pool,o)==key then
                    local id = bit32.band(links,0xffff)
                    if seen[id] ~= serial then
                        seen[id] = serial
                        if #out >= maxIds then return out end
                        out[#out + 1] = id
                    end
                end
                index = bit32.rshift(links,16)
            end
        end
    end
    return out
end

function Grid.getItemCountAt(grid, x, y)
    local n = 0
    local index = bucketRead(grid, hashPos2(x, y, grid.bucketsSize))
    local key=coordKey(x,y)
    while index ~= NULL do
        local o = itemOffset(index)
        local links=buffer.readu32(grid.pool,o+4)
        if buffer.readu32(grid.pool,o)==key then n += 1 end
        index=bit32.rshift(links,16)
    end
    return n
end

function Grid.getBounds(grid)
    return grid.minx, grid.miny, grid.maxx, grid.maxy
end

function Grid.getCellSize(grid)
    return grid.cellSize
end

return table.freeze(Grid)
