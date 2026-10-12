--!native

-- Pure-Luau Recast core. Algorithmic structure follows RecastNavigation and is deliberately
-- altered to use Luau vectors/buffers instead of C++ objects and allocators.

local Common = require('@recast/common')

local function vertexAt(vertices, index0)
    local o = index0 * 12
    return vector.create(buffer.readf32(vertices,o), buffer.readf32(vertices,o+4), buffer.readf32(vertices,o+8))
end
local function triangleAt(triangles, tri0)
    local o = tri0 * 12
    return buffer.readu32(triangles,o), buffer.readu32(triangles,o+4), buffer.readu32(triangles,o+8)
end
local function writeVec3f(target, index1, v: vector)
    local o = (index1 - 1) * 12
    buffer.writef32(target,o,v.X); buffer.writef32(target,o+4,v.Y); buffer.writef32(target,o+8,v.Z)
end

local Recast = {}

Recast.RC_PI = 3.14159265
Recast.RC_SPAN_HEIGHT_BITS = 13
Recast.RC_SPAN_MAX_HEIGHT = (2 ^ 13) - 1
Recast.RC_NULL_AREA = 0
Recast.RC_WALKABLE_AREA = 63
Recast.RC_NOT_CONNECTED = 0x3f
Recast.RC_BORDER_REG = 0x8000
Recast.RC_MULTIPLE_REGS = 0
Recast.RC_BORDER_VERTEX = 0x10000
Recast.RC_AREA_BORDER = 0x20000
Recast.RC_CONTOUR_TESS_WALL_EDGES = 0x01
Recast.RC_CONTOUR_TESS_AREA_EDGES = 0x02
Recast.RC_CONTOUR_REG_MASK = 0xffff
Recast.RC_MESH_NULL_IDX = 0xffff

local RC_SPAN_MAX_HEIGHT = Recast.RC_SPAN_MAX_HEIGHT
local RC_NULL_AREA = Recast.RC_NULL_AREA
local RC_WALKABLE_AREA = Recast.RC_WALKABLE_AREA
local RC_NOT_CONNECTED = Recast.RC_NOT_CONNECTED
local RC_BORDER_REG = Recast.RC_BORDER_REG
local RC_BORDER_VERTEX = Recast.RC_BORDER_VERTEX
local RC_AREA_BORDER = Recast.RC_AREA_BORDER
local RC_CONTOUR_REG_MASK = Recast.RC_CONTOUR_REG_MASK
local RC_MESH_NULL_IDX = Recast.RC_MESH_NULL_IDX
local MAX_HEIGHTFIELD_HEIGHT = 0xffff

local DIR_X = {-1, 0, 1, 0}
local DIR_Z = {0, 1, 0, -1}

local function trunc(v)
    return if v < 0 then math.ceil(v) else math.floor(v)
end

local function component(v: vector, axis)
    if axis == 0 then return v.X end
    if axis == 1 then return v.Y end
    return v.Z
end

local function lerpVec(a, b, t)
    return a + (b - a) * t
end

local function overlapBounds(amin: vector, amax: vector, bmin: vector, bmax: vector)
    return not (
        amin.X > bmax.X or amax.X < bmin.X
        or amin.Y > bmax.Y or amax.Y < bmin.Y
        or amin.Z > bmax.Z or amax.Z < bmin.Z
    )
end

local function growSpans(hf)
    local oldCapacity = hf.capacity
    local newCapacity = oldCapacity * 2
    hf.mins = Common.ensureBuffer(hf.mins, oldCapacity * 2, newCapacity * 2)
    hf.maxs = Common.ensureBuffer(hf.maxs, oldCapacity * 2, newCapacity * 2)
    hf.areas = Common.ensureBuffer(hf.areas, oldCapacity, newCapacity)
    hf.next = Common.ensureBuffer(hf.next, oldCapacity * 4, newCapacity * 4)
    hf.capacity = newCapacity
end

local function allocSpan(hf)
    local id
    if hf.freeHead >= 0 then
        id = hf.freeHead
        hf.freeHead = buffer.readi32(hf.next, id * 4)
    else
        id = hf.spanCount
        if id >= hf.capacity then growSpans(hf) end
        hf.spanCount += 1
    end
    buffer.writei32(hf.next, id * 4, -1)
    return id
end

local function freeSpan(hf, id)
    buffer.writei32(hf.next, id * 4, hf.freeHead)
    hf.freeHead = id
end

local function spanMin(hf, id)
    return buffer.readu16(hf.mins, id * 2)
end

local function spanMax(hf, id)
    return buffer.readu16(hf.maxs, id * 2)
end

local function spanArea(hf, id)
    return buffer.readu8(hf.areas, id)
end

local function spanNext(hf, id)
    return buffer.readi32(hf.next, id * 4)
end

local function setSpanMin(hf, id, value)
    buffer.writeu16(hf.mins, id * 2, value)
end

local function setSpanMax(hf, id, value)
    buffer.writeu16(hf.maxs, id * 2, value)
end

local function setSpanArea(hf, id, value)
    buffer.writeu8(hf.areas, id, value)
end

local function setSpanNext(hf, id, value)
    buffer.writei32(hf.next, id * 4, value)
end

local function cellIndex(hf, x, z)
    return x + z * hf.width
end

function Recast.calcBounds(vertices)
    local packed, count = Common.packVertices(vertices)
    if count == 0 then error('calcBounds needs at least one vertex', 2) end
    local first: vector = vertexAt(packed, 0)
    local minx, miny, minz = first.X, first.Y, first.Z
    local maxx, maxy, maxz = minx, miny, minz
    for i = 1, count - 1 do
        local v: vector = vertexAt(packed, i)
        if v.X < minx then minx = v.X end
        if v.Y < miny then miny = v.Y end
        if v.Z < minz then minz = v.Z end
        if v.X > maxx then maxx = v.X end
        if v.Y > maxy then maxy = v.Y end
        if v.Z > maxz then maxz = v.Z end
    end
    return vector.create(minx, miny, minz), vector.create(maxx, maxy, maxz)
end

function Recast.calcGridSize(bmin: vector, bmax: vector, cellSize)
    return trunc((bmax.X - bmin.X) / cellSize + 0.5), trunc((bmax.Z - bmin.Z) / cellSize + 0.5)
end

function Recast.createHeightfield(width, height, bmin, bmax, cellSize, cellHeight)
    if width < 0 or height < 0 then error('heightfield dimensions must be non-negative', 2) end
    local cells = width * height
    local heads = buffer.create(cells * 4)
    buffer.fill(heads, 0, 0xff)
    local capacity = math.max(64, math.min(math.max(cells, 64), 4096))
    return {
        width = width,
        height = height,
        bmin = bmin,
        bmax = bmax,
        cs = cellSize,
        ch = cellHeight,
        heads = heads,
        mins = buffer.create(capacity * 2),
        maxs = buffer.create(capacity * 2),
        areas = buffer.create(capacity),
        next = buffer.create(capacity * 4),
        capacity = capacity,
        spanCount = 0,
        freeHead = -1,
    }
end

function Recast.addSpan(hf, x, z, smin, smax, area, flagMergeThreshold)
    if smin >= smax then return true end
    if x < 0 or z < 0 or x >= hf.width or z >= hf.height then return false end

    local newSpan = allocSpan(hf)
    local nmin, nmax, narea = smin, smax, area
    local ci = cellIndex(hf, x, z)
    local previous = -1
    local current = buffer.readi32(hf.heads, ci * 4)

    while current >= 0 do
        local cmin = spanMin(hf, current)
        local cmax = spanMax(hf, current)

        if cmin > nmax then
            break
        elseif cmax < nmin then
            previous = current
            current = spanNext(hf, current)
        else
            if cmin < nmin then nmin = cmin end
            if cmax > nmax then nmax = cmax end
            if math.abs(nmax - cmax) <= flagMergeThreshold then
                narea = math.max(narea, spanArea(hf, current))
            end

            local nextId = spanNext(hf, current)
            freeSpan(hf, current)
            if previous >= 0 then
                setSpanNext(hf, previous, nextId)
            else
                buffer.writei32(hf.heads, ci * 4, nextId)
            end
            current = nextId
        end
    end

    setSpanMin(hf, newSpan, nmin)
    setSpanMax(hf, newSpan, nmax)
    setSpanArea(hf, newSpan, narea)
    if previous >= 0 then
        setSpanNext(hf, newSpan, spanNext(hf, previous))
        setSpanNext(hf, previous, newSpan)
    else
        setSpanNext(hf, newSpan, buffer.readi32(hf.heads, ci * 4))
        buffer.writei32(hf.heads, ci * 4, newSpan)
    end
    return true
end

function Recast.getSpans(hf, x, z)
    local out = {}
    if x < 0 or z < 0 or x >= hf.width or z >= hf.height then return out end
    local id = buffer.readi32(hf.heads, cellIndex(hf, x, z) * 4)
    while id >= 0 do
        out[#out + 1] = {
            smin = spanMin(hf, id),
            smax = spanMax(hf, id),
            area = spanArea(hf, id),
        }
        id = spanNext(hf, id)
    end
    return out
end

local function dividePolyX(input: { vector }, inputCount, out1, out2, axisOffset)
    local n1, n2 = 0, 0
    local b = inputCount
    local db = axisOffset - input[b].X
    for a = 1, inputCount do
        local da = axisOffset - input[a].X
        local sameSide = (da >= 0) == (db >= 0)
        if not sameSide then
            local p = lerpVec(input[b], input[a], db / (db - da))
            n1 += 1; out1[n1] = p
            n2 += 1; out2[n2] = p
            if da > 0 then n1 += 1; out1[n1] = input[a]
            elseif da < 0 then n2 += 1; out2[n2] = input[a] end
        else
            if da >= 0 then
                n1 += 1; out1[n1] = input[a]
                if da ~= 0 then b = a; db = da; continue end
            end
            n2 += 1; out2[n2] = input[a]
        end
        b = a
        db = da
    end
    return n1, n2
end

local function dividePolyZ(input: { vector }, inputCount, out1, out2, axisOffset)
    local n1, n2 = 0, 0
    local b = inputCount
    local db = axisOffset - input[b].Z
    for a = 1, inputCount do
        local da = axisOffset - input[a].Z
        local sameSide = (da >= 0) == (db >= 0)
        if not sameSide then
            local p = lerpVec(input[b], input[a], db / (db - da))
            n1 += 1; out1[n1] = p
            n2 += 1; out2[n2] = p
            if da > 0 then n1 += 1; out1[n1] = input[a]
            elseif da < 0 then n2 += 1; out2[n2] = input[a] end
        else
            if da >= 0 then
                n1 += 1; out1[n1] = input[a]
                if da ~= 0 then b = a; db = da; continue end
            end
            n2 += 1; out2[n2] = input[a]
        end
        b = a
        db = da
    end
    return n1, n2
end

local function newRasterScratch()
    return {
        inputA = table.create(12), inputB = table.create(12), row = table.create(12),
        col = table.create(12), colRem = table.create(12),
    }
end

local function rasterizeTri(v0: vector, v1: vector, v2: vector, area, hf, flagMergeThreshold, scratch)
    local triMin: vector = vector.create(
        math.min(v0.X, v1.X, v2.X),
        math.min(v0.Y, v1.Y, v2.Y),
        math.min(v0.Z, v1.Z, v2.Z)
    )
    local triMax: vector = vector.create(
        math.max(v0.X, v1.X, v2.X),
        math.max(v0.Y, v1.Y, v2.Y),
        math.max(v0.Z, v1.Z, v2.Z)
    )
    if not overlapBounds(triMin, triMax, hf.bmin, hf.bmax) then return true end

    local inverseCellSize = 1 / hf.cs
    local inverseCellHeight = 1 / hf.ch
    local by = hf.bmax.Y - hf.bmin.Y
    local z0 = math.clamp(trunc((triMin.Z - hf.bmin.Z) * inverseCellSize), -1, hf.height - 1)
    local z1 = math.clamp(trunc((triMax.Z - hf.bmin.Z) * inverseCellSize), 0, hf.height - 1)

    scratch = scratch or newRasterScratch()
    local input = scratch.inputA
    local rem = scratch.inputB
    input[1], input[2], input[3] = v0, v1, v2
    local inputCount = 3
    local row: { vector } = scratch.row
    local col = scratch.col
    local colRem = scratch.colRem

    for z = z0, z1 do
        local cellZ = hf.bmin.Z + z * hf.cs
        local rowCount, remainingCount = dividePolyZ(input, inputCount, row, rem, cellZ + hf.cs)
        input, rem = rem, input
        inputCount = remainingCount
        if rowCount < 3 or z < 0 then continue end

        local minX, maxX = row[1].X, row[1].X
        for i = 2, rowCount do
            local x = row[i].X
            if x < minX then minX = x end
            if x > maxX then maxX = x end
        end
        local x0 = trunc((minX - hf.bmin.X) * inverseCellSize)
        local x1 = trunc((maxX - hf.bmin.X) * inverseCellSize)
        if x1 < 0 or x0 >= hf.width then continue end
        x0 = math.clamp(x0, -1, hf.width - 1)
        x1 = math.clamp(x1, 0, hf.width - 1)

        local inRow = row
        local inRowCount = rowCount
        local p1: { vector } = col
        local p2 = colRem
        for x = x0, x1 do
            local cellX = hf.bmin.X + x * hf.cs
            local nv, nv2 = dividePolyX(inRow, inRowCount, p1, p2, cellX + hf.cs)
            inRow, p2 = p2, inRow
            inRowCount = nv2
            if nv < 3 or x < 0 then continue end

            local minY, maxY = p1[1].Y, p1[1].Y
            for i = 2, nv do
                local y = p1[i].Y
                if y < minY then minY = y end
                if y > maxY then maxY = y end
            end
            minY -= hf.bmin.Y
            maxY -= hf.bmin.Y
            if maxY < 0 or minY > by then continue end
            minY = math.max(minY, 0)
            maxY = math.min(maxY, by)

            local smin = math.clamp(math.floor(minY * inverseCellHeight), 0, RC_SPAN_MAX_HEIGHT)
            local smax = math.clamp(math.ceil(maxY * inverseCellHeight), smin + 1, RC_SPAN_MAX_HEIGHT)
            if not Recast.addSpan(hf, x, z, smin, smax, area, flagMergeThreshold) then return false end
        end
    end
    return true
end

function Recast.rasterizeTriangle(v0, v1, v2, area, hf, flagMergeThreshold)
    return rasterizeTri(v0, v1, v2, area, hf, flagMergeThreshold or 1, nil)
end

function Recast.rasterizeTriangles(vertices, triangles, areas, hf, flagMergeThreshold)
    local verts, _ = Common.packVertices(vertices)
    local tris, count = Common.packTriangles(triangles)
    local areaBuffer
    if typeof(areas) == 'buffer' then
        areaBuffer = areas
    else
        areaBuffer = buffer.create(count)
        for i = 1, count do buffer.writeu8(areaBuffer, i - 1, areas[i] or RC_NULL_AREA) end
    end
    local scratch = newRasterScratch()
    for i = 0, count - 1 do
        local ia, ib, ic = triangleAt(tris, i)
        if not rasterizeTri(
            vertexAt(verts, ia), vertexAt(verts, ib), vertexAt(verts, ic),
            buffer.readu8(areaBuffer, i), hf, flagMergeThreshold or 1, scratch
        ) then return false end
    end
    return true
end

local function triNormal(a, b, c)
    local e0: vector = b - a
    local e1: vector = c - a
    local cross: vector = vector.cross(e0, e1)
    local mag = vector.magnitude(cross)
    if mag <= 1e-20 then return vector.create(0, 0, 0) end
    return cross / mag
end

function Recast.markWalkableTriangles(walkableSlopeAngle, vertices, triangles, areas)
    local verts, _ = Common.packVertices(vertices)
    local tris, count = Common.packTriangles(triangles)
    local out = if typeof(areas) == 'buffer' then areas else buffer.create(count)
    if typeof(areas) ~= 'buffer' and areas ~= nil then
        for i = 1, count do buffer.writeu8(out, i - 1, areas[i] or RC_NULL_AREA) end
    end
    local threshold = math.cos(walkableSlopeAngle / 180 * Recast.RC_PI)
    for i = 0, count - 1 do
        local ia, ib, ic = triangleAt(tris, i)
        local n: vector = triNormal(vertexAt(verts, ia), vertexAt(verts, ib), vertexAt(verts, ic))
        if n.Y > threshold then buffer.writeu8(out, i, RC_WALKABLE_AREA) end
    end
    return out
end

function Recast.rasterizeWalkableTriangles(walkableSlopeAngle, vertices, triangles, hf, flagMergeThreshold, areas)
    local verts, _ = Common.packVertices(vertices)
    local tris, count = Common.packTriangles(triangles)
    local out = if typeof(areas) == 'buffer' then areas else buffer.create(count)
    if typeof(areas) ~= 'buffer' and areas ~= nil then
        for i = 1, count do buffer.writeu8(out, i - 1, areas[i] or RC_NULL_AREA) end
    end
    local threshold = math.cos(walkableSlopeAngle / 180 * Recast.RC_PI)
    local scratch = newRasterScratch()
    local mergeThreshold = flagMergeThreshold or 1
    for i = 0, count - 1 do
        local ia, ib, ic = triangleAt(tris, i)
        local a, b, c = vertexAt(verts, ia), vertexAt(verts, ib), vertexAt(verts, ic)
        local area = buffer.readu8(out, i)
        if areas == nil or area == RC_NULL_AREA then
            local n: vector = triNormal(a, b, c)
            if n.Y > threshold then
                area = RC_WALKABLE_AREA
                buffer.writeu8(out, i, area)
            end
        end
        if not rasterizeTri(a, b, c, area, hf, mergeThreshold, scratch) then return nil end
    end
    return out
end

function Recast.clearUnwalkableTriangles(walkableSlopeAngle, vertices, triangles, areas)
    local verts, _ = Common.packVertices(vertices)
    local tris, count = Common.packTriangles(triangles)
    local out
    if typeof(areas) == 'buffer' then
        out = areas
    else
        out = buffer.create(count)
        for i = 1, count do buffer.writeu8(out, i - 1, areas[i] or RC_NULL_AREA) end
    end
    local threshold = math.cos(walkableSlopeAngle / 180 * Recast.RC_PI)
    for i = 0, count - 1 do
        local ia, ib, ic = triangleAt(tris, i)
        local n: vector = triNormal(vertexAt(verts, ia), vertexAt(verts, ib), vertexAt(verts, ic))
        if n.Y <= threshold then buffer.writeu8(out, i, RC_NULL_AREA) end
    end
    return out
end

function Recast.filterLowHangingWalkableObstacles(walkableClimb, hf)
    for z = 0, hf.height - 1 do
        for x = 0, hf.width - 1 do
            local previous = -1
            local previousWasWalkable = false
            local previousArea = RC_NULL_AREA
            local id = buffer.readi32(hf.heads, cellIndex(hf, x, z) * 4)
            while id >= 0 do
                local walkable = spanArea(hf, id) ~= RC_NULL_AREA
                if not walkable and previousWasWalkable and spanMax(hf, id) - spanMax(hf, previous) <= walkableClimb then
                    setSpanArea(hf, id, previousArea)
                end
                previousWasWalkable = walkable
                previousArea = spanArea(hf, id)
                previous = id
                id = spanNext(hf, id)
            end
        end
    end
end

function Recast.filterWalkableLowHeightSpans(walkableHeight, hf)
    for z = 0, hf.height - 1 do
        for x = 0, hf.width - 1 do
            local id = buffer.readi32(hf.heads, cellIndex(hf, x, z) * 4)
            while id >= 0 do
                local nextId = spanNext(hf, id)
                local floor = spanMax(hf, id)
                local ceiling = if nextId >= 0 then spanMin(hf, nextId) else MAX_HEIGHTFIELD_HEIGHT
                if ceiling - floor < walkableHeight then setSpanArea(hf, id, RC_NULL_AREA) end
                id = nextId
            end
        end
    end
end

function Recast.filterLedgeSpans(walkableHeight, walkableClimb, hf)
    local w, h = hf.width, hf.height
    for z = 0, h - 1 do
        for x = 0, w - 1 do
            local id = buffer.readi32(hf.heads, cellIndex(hf, x, z) * 4)
            while id >= 0 do
                local nextId = spanNext(hf, id)
                if spanArea(hf, id) ~= RC_NULL_AREA then
                    local floor = spanMax(hf, id)
                    local ceiling = if nextId >= 0 then spanMin(hf, nextId) else MAX_HEIGHTFIELD_HEIGHT
                    local lowestDiff = MAX_HEIGHTFIELD_HEIGHT
                    local lowestFloor, highestFloor = floor, floor

                    for d = 1, 4 do
                        local nx, nz = x + DIR_X[d], z + DIR_Z[d]
                        if nx < 0 or nz < 0 or nx >= w or nz >= h then
                            lowestDiff = -walkableClimb - 1
                            break
                        end
                        local nid = buffer.readi32(hf.heads, cellIndex(hf, nx, nz) * 4)
                        local neighborCeiling = if nid >= 0 then spanMin(hf, nid) else MAX_HEIGHTFIELD_HEIGHT
                        if math.min(ceiling, neighborCeiling) - floor >= walkableHeight then
                            lowestDiff = -walkableClimb - 1
                            break
                        end

                        while nid >= 0 do
                            local nnext = spanNext(hf, nid)
                            local neighborFloor = spanMax(hf, nid)
                            neighborCeiling = if nnext >= 0 then spanMin(hf, nnext) else MAX_HEIGHTFIELD_HEIGHT
                            if math.min(ceiling, neighborCeiling) - math.max(floor, neighborFloor) >= walkableHeight then
                                local diff = neighborFloor - floor
                                if diff < lowestDiff then lowestDiff = diff end
                                if math.abs(diff) <= walkableClimb then
                                    if neighborFloor < lowestFloor then lowestFloor = neighborFloor end
                                    if neighborFloor > highestFloor then highestFloor = neighborFloor end
                                elseif diff < -walkableClimb then
                                    break
                                end
                            end
                            nid = nnext
                        end
                    end

                    if lowestDiff < -walkableClimb or highestFloor - lowestFloor > walkableClimb then
                        setSpanArea(hf, id, RC_NULL_AREA)
                    end
                end
                id = nextId
            end
        end
    end
end

function Recast.getHeightfieldSpanCount(hf)
    local count = 0
    for cell = 0, hf.width * hf.height - 1 do
        local id = buffer.readi32(hf.heads, cell * 4)
        while id >= 0 do
            if spanArea(hf, id) ~= RC_NULL_AREA then count += 1 end
            id = spanNext(hf, id)
        end
    end
    return count
end

local function setCon(cons, span, dir0, value)
    local offset = span * 4
    local bits = buffer.readu32(cons, offset)
    local shift = dir0 * 6
    local mask = bit32.lshift(0x3f, shift)
    bits = bit32.bor(bit32.band(bits, bit32.bnot(mask)), bit32.lshift(bit32.band(value, 0x3f), shift))
    buffer.writeu32(cons, offset, bits)
end

local function getCon(cons, span, dir0)
    return bit32.band(bit32.rshift(buffer.readu32(cons, span * 4), dir0 * 6), 0x3f)
end

function Recast.buildCompactHeightfield(walkableHeight, walkableClimb, hf)
    local spanCount = Recast.getHeightfieldSpanCount(hf)
    local cellsIndex = buffer.create(hf.width * hf.height * 4)
    local cellsCount = buffer.create(hf.width * hf.height)
    local ys = buffer.create(spanCount * 2)
    local hs = buffer.create(spanCount)
    local cons = buffer.create(spanCount * 4)
    local areas = buffer.create(spanCount)
    local spanX = buffer.create(spanCount * 4)
    local spanZ = buffer.create(spanCount * 4)

    local currentIndex = 0
    for cell = 0, hf.width * hf.height - 1 do
        local id = buffer.readi32(hf.heads, cell * 4)
        if id >= 0 then
            buffer.writeu32(cellsIndex, cell * 4, currentIndex)
            local count = 0
            while id >= 0 do
                if spanArea(hf, id) ~= RC_NULL_AREA then
                    local nextId = spanNext(hf, id)
                    local bot = spanMax(hf, id)
                    local top = if nextId >= 0 then spanMin(hf, nextId) else MAX_HEIGHTFIELD_HEIGHT
                    buffer.writeu16(ys, currentIndex * 2, math.clamp(bot, 0, 0xffff))
                    buffer.writeu8(hs, currentIndex, math.clamp(top - bot, 0, 0xff))
                    buffer.writeu8(areas, currentIndex, spanArea(hf, id))
                    buffer.writeu32(spanX, currentIndex * 4, cell % hf.width)
                    buffer.writeu32(spanZ, currentIndex * 4, cell // hf.width)
                    buffer.writeu32(cons, currentIndex * 4, 0x00ffffff) -- four RC_NOT_CONNECTED fields
                    currentIndex += 1
                    count += 1
                end
                id = spanNext(hf, id)
            end
            buffer.writeu8(cellsCount, cell, count)
        end
    end

    local chf = {
        width = hf.width,
        height = hf.height,
        spanCount = spanCount,
        walkableHeight = walkableHeight,
        walkableClimb = walkableClimb,
        borderSize = 0,
        maxDistance = 0,
        maxRegions = 0,
        bmin = hf.bmin,
        bmax = vector.create(hf.bmax.X, hf.bmax.Y + walkableHeight * hf.ch, hf.bmax.Z),
        cs = hf.cs,
        ch = hf.ch,
        cellIndex = cellsIndex,
        cellCount = cellsCount,
        y = ys,
        h = hs,
        con = cons,
        areas = areas,
        spanX = spanX,
        spanZ = spanZ,
    }

    local maxLayers = RC_NOT_CONNECTED - 1
    local maxLayerIndex = 0
    for z = 0, chf.height - 1 do
        for x = 0, chf.width - 1 do
            local c = x + z * chf.width
            local first = buffer.readu32(cellsIndex, c * 4)
            local count = buffer.readu8(cellsCount, c)
            for localIndex = 0, count - 1 do
                local i = first + localIndex
                local sy = buffer.readu16(ys, i * 2)
                local sh = buffer.readu8(hs, i)
                for dir0 = 0, 3 do
                    local nx, nz = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
                    setCon(cons, i, dir0, RC_NOT_CONNECTED)
                    if nx < 0 or nz < 0 or nx >= chf.width or nz >= chf.height then continue end
                    local nc = nx + nz * chf.width
                    local nfirst = buffer.readu32(cellsIndex, nc * 4)
                    local ncount = buffer.readu8(cellsCount, nc)
                    for nlocal = 0, ncount - 1 do
                        local k = nfirst + nlocal
                        local ny = buffer.readu16(ys, k * 2)
                        local nh = buffer.readu8(hs, k)
                        local bot = math.max(sy, ny)
                        local top = math.min(sy + sh, ny + nh)
                        if top - bot >= walkableHeight and math.abs(ny - sy) <= walkableClimb then
                            if nlocal <= maxLayers then
                                setCon(cons, i, dir0, nlocal)
                            elseif nlocal > maxLayerIndex then
                                maxLayerIndex = nlocal
                            end
                            break
                        end
                    end
                end
            end
        end
    end
    chf.maxLayerIndex = maxLayerIndex
    return chf
end

function Recast.getCompactCell(chf, x, z)
    if x < 0 or z < 0 or x >= chf.width or z >= chf.height then return nil end
    local c = x + z * chf.width
    return { index = buffer.readu32(chf.cellIndex, c * 4), count = buffer.readu8(chf.cellCount, c) }
end

function Recast.getCompactSpan(chf, index0)
    if index0 < 0 or index0 >= chf.spanCount then return nil end
    return {
        y = buffer.readu16(chf.y, index0 * 2),
        h = buffer.readu8(chf.h, index0),
        area = buffer.readu8(chf.areas, index0),
        connections = {
            getCon(chf.con, index0, 0), getCon(chf.con, index0, 1),
            getCon(chf.con, index0, 2), getCon(chf.con, index0, 3),
        },
    }
end

function Recast.compactNeighbor(chf, x, z, spanIndex0, dir0)
    local layer = getCon(chf.con, spanIndex0, dir0)
    if layer == RC_NOT_CONNECTED then return nil end
    local nx, nz = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
    if nx < 0 or nz < 0 or nx >= chf.width or nz >= chf.height then return nil end
    local c = nx + nz * chf.width
    return buffer.readu32(chf.cellIndex, c * 4) + layer
end


function Recast.erodeWalkableArea(erosionRadius, chf)
    if erosionRadius <= 0 then return true end
    local dist = buffer.create(chf.spanCount)
    buffer.fill(dist, 0, 0xff)

    -- Boundary spans are zero distance from an obstacle or missing connection.
    for z = 0, chf.height - 1 do
        for x = 0, chf.width - 1 do
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for localIndex = 0, count - 1 do
                local span = first + localIndex
                if buffer.readu8(chf.areas, span) == RC_NULL_AREA then
                    buffer.writeu8(dist, span, 0)
                    continue
                end
                local neighbors = 0
                for dir0 = 0, 3 do
                    local con = getCon(chf.con, span, dir0)
                    if con == RC_NOT_CONNECTED then break end
                    local nx, nz = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
                    local ncell = nx + nz * chf.width
                    local nspan = buffer.readu32(chf.cellIndex, ncell * 4) + con
                    if buffer.readu8(chf.areas, nspan) == RC_NULL_AREA then break end
                    neighbors += 1
                end
                if neighbors ~= 4 then buffer.writeu8(dist, span, 0) end
            end
        end
    end

    local function relax(target, source, weight)
        local candidate = math.min(buffer.readu8(dist, source) + weight, 255)
        if candidate < buffer.readu8(dist, target) then buffer.writeu8(dist, target, candidate) end
    end

    -- Chamfer distance transform, forward pass.
    for z = 0, chf.height - 1 do
        for x = 0, chf.width - 1 do
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local con0 = getCon(chf.con, span, 0)
                if con0 ~= RC_NOT_CONNECTED then
                    local ax, az = x - 1, z
                    local a = buffer.readu32(chf.cellIndex, (ax + az * chf.width) * 4) + con0
                    relax(span, a, 2)
                    local con3 = getCon(chf.con, a, 3)
                    if con3 ~= RC_NOT_CONNECTED then
                        local b = buffer.readu32(chf.cellIndex, (ax + (az - 1) * chf.width) * 4) + con3
                        relax(span, b, 3)
                    end
                end
                local con3 = getCon(chf.con, span, 3)
                if con3 ~= RC_NOT_CONNECTED then
                    local ax, az = x, z - 1
                    local a = buffer.readu32(chf.cellIndex, (ax + az * chf.width) * 4) + con3
                    relax(span, a, 2)
                    local con2 = getCon(chf.con, a, 2)
                    if con2 ~= RC_NOT_CONNECTED then
                        local b = buffer.readu32(chf.cellIndex, ((ax + 1) + az * chf.width) * 4) + con2
                        relax(span, b, 3)
                    end
                end
            end
        end
    end

    -- Reverse pass.
    for z = chf.height - 1, 0, -1 do
        for x = chf.width - 1, 0, -1 do
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local con2 = getCon(chf.con, span, 2)
                if con2 ~= RC_NOT_CONNECTED then
                    local ax, az = x + 1, z
                    local a = buffer.readu32(chf.cellIndex, (ax + az * chf.width) * 4) + con2
                    relax(span, a, 2)
                    local con1 = getCon(chf.con, a, 1)
                    if con1 ~= RC_NOT_CONNECTED then
                        local b = buffer.readu32(chf.cellIndex, (ax + (az + 1) * chf.width) * 4) + con1
                        relax(span, b, 3)
                    end
                end
                local con1 = getCon(chf.con, span, 1)
                if con1 ~= RC_NOT_CONNECTED then
                    local ax, az = x, z + 1
                    local a = buffer.readu32(chf.cellIndex, (ax + az * chf.width) * 4) + con1
                    relax(span, a, 2)
                    local con0 = getCon(chf.con, a, 0)
                    if con0 ~= RC_NOT_CONNECTED then
                        local b = buffer.readu32(chf.cellIndex, ((ax - 1) + az * chf.width) * 4) + con0
                        relax(span, b, 3)
                    end
                end
            end
        end
    end

    local minimum = erosionRadius * 2
    for i = 0, chf.spanCount - 1 do
        if buffer.readu8(dist, i) < minimum then buffer.writeu8(chf.areas, i, RC_NULL_AREA) end
    end
    chf.distanceToBoundary = dist
    return true
end

local function truncTowardZero(value)
    if value < 0 then return math.ceil(value) end
    return math.floor(value)
end

function Recast.medianFilterWalkableArea(chf)
    local filtered = buffer.create(chf.spanCount)
    local neighborAreas = table.create(9, 0)
    for z = 0, chf.height - 1 do
        for x = 0, chf.width - 1 do
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local ownArea = buffer.readu8(chf.areas, span)
                if ownArea == RC_NULL_AREA then
                    buffer.writeu8(filtered, span, ownArea)
                    continue
                end
                for i = 1, 9 do neighborAreas[i] = ownArea end
                for dir0 = 0, 3 do
                    local con = getCon(chf.con, span, dir0)
                    if con == RC_NOT_CONNECTED then continue end
                    local ax, az = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
                    local a = buffer.readu32(chf.cellIndex, (ax + az * chf.width) * 4) + con
                    local aArea = buffer.readu8(chf.areas, a)
                    if aArea ~= RC_NULL_AREA then neighborAreas[dir0 * 2 + 1] = aArea end
                    local dir2 = (dir0 + 1) % 4
                    local con2 = getCon(chf.con, a, dir2)
                    if con2 ~= RC_NOT_CONNECTED then
                        local bx, bz = ax + DIR_X[dir2 + 1], az + DIR_Z[dir2 + 1]
                        local b = buffer.readu32(chf.cellIndex, (bx + bz * chf.width) * 4) + con2
                        local bArea = buffer.readu8(chf.areas, b)
                        if bArea ~= RC_NULL_AREA then neighborAreas[dir0 * 2 + 2] = bArea end
                    end
                end
                -- Upstream uses insertion sort over exactly nine area IDs.
                for i = 2, 9 do
                    local value = neighborAreas[i]
                    local j = i - 1
                    while j >= 1 and neighborAreas[j] > value do
                        neighborAreas[j + 1] = neighborAreas[j]
                        j -= 1
                    end
                    neighborAreas[j + 1] = value
                end
                buffer.writeu8(filtered, span, neighborAreas[5])
            end
        end
    end
    buffer.copy(chf.areas, 0, filtered, 0, chf.spanCount)
    return true
end

function Recast.markBoxArea(boxMin: vector, boxMax: vector, areaId, chf)
    local minX = truncTowardZero((boxMin.X - chf.bmin.X) / chf.cs)
    local minY = truncTowardZero((boxMin.Y - chf.bmin.Y) / chf.ch)
    local minZ = truncTowardZero((boxMin.Z - chf.bmin.Z) / chf.cs)
    local maxX = truncTowardZero((boxMax.X - chf.bmin.X) / chf.cs)
    local maxY = truncTowardZero((boxMax.Y - chf.bmin.Y) / chf.ch)
    local maxZ = truncTowardZero((boxMax.Z - chf.bmin.Z) / chf.cs)
    if maxX < 0 or minX >= chf.width or maxZ < 0 or minZ >= chf.height then return chf end
    minX = math.clamp(minX, 0, chf.width - 1)
    maxX = math.clamp(maxX, 0, chf.width - 1)
    minZ = math.clamp(minZ, 0, chf.height - 1)
    maxZ = math.clamp(maxZ, 0, chf.height - 1)
    for z = minZ, maxZ do
        for x = minX, maxX do
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local y = buffer.readu16(chf.y, span * 2)
                if y >= minY and y <= maxY and buffer.readu8(chf.areas, span) ~= RC_NULL_AREA then
                    buffer.writeu8(chf.areas, span, areaId)
                end
            end
        end
    end
    return chf
end

local function pointInPolyXZ(vertices, point: vector)
    local inside = false
    local n = #vertices
    local j = n
    for i = 1, n do
        local vi: vector, vj: vector = vertices[i], vertices[j]
        if (vi.Z > point.Z) ~= (vj.Z > point.Z) then
            local crossX = (vj.X - vi.X) * (point.Z - vi.Z) / (vj.Z - vi.Z) + vi.X
            if point.X < crossX then inside = not inside end
        end
        j = i
    end
    return inside
end

local function unpackVertexList(vertices)
    if typeof(vertices) == 'buffer' then
        local count = buffer.len(vertices) // 12
        local out = table.create(count)
        for i = 0, count - 1 do out[i + 1] = vertexAt(vertices, i) end
        return out
    end
    return vertices
end

function Recast.markConvexPolyArea(vertices, minYWorld, maxYWorld, areaId, chf)
    local verts: { vector } = unpackVertexList(vertices)
    if #verts < 3 then return chf end
    local minXWorld, maxXWorld = verts[1].X, verts[1].X
    local minZWorld, maxZWorld = verts[1].Z, verts[1].Z
    for i = 2, #verts do
        local v: vector = verts[i]
        minXWorld = math.min(minXWorld, v.X); maxXWorld = math.max(maxXWorld, v.X)
        minZWorld = math.min(minZWorld, v.Z); maxZWorld = math.max(maxZWorld, v.Z)
    end
    local minX = truncTowardZero((minXWorld - chf.bmin.X) / chf.cs)
    local minY = truncTowardZero((minYWorld - chf.bmin.Y) / chf.ch)
    local minZ = truncTowardZero((minZWorld - chf.bmin.Z) / chf.cs)
    local maxX = truncTowardZero((maxXWorld - chf.bmin.X) / chf.cs)
    local maxY = truncTowardZero((maxYWorld - chf.bmin.Y) / chf.ch)
    local maxZ = truncTowardZero((maxZWorld - chf.bmin.Z) / chf.cs)
    if maxX < 0 or minX >= chf.width or maxZ < 0 or minZ >= chf.height then return chf end
    minX = math.clamp(minX, 0, chf.width - 1); maxX = math.clamp(maxX, 0, chf.width - 1)
    minZ = math.clamp(minZ, 0, chf.height - 1); maxZ = math.clamp(maxZ, 0, chf.height - 1)
    for z = minZ, maxZ do
        for x = minX, maxX do
            local center = vector.create(chf.bmin.X + (x + 0.5) * chf.cs, 0, chf.bmin.Z + (z + 0.5) * chf.cs)
            if not pointInPolyXZ(verts, center) then continue end
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local y = buffer.readu16(chf.y, span * 2)
                if y >= minY and y <= maxY and buffer.readu8(chf.areas, span) ~= RC_NULL_AREA then
                    buffer.writeu8(chf.areas, span, areaId)
                end
            end
        end
    end
    return chf
end

function Recast.markCylinderArea(position: vector, radius, height, areaId, chf)
    local minX = truncTowardZero((position.X - radius - chf.bmin.X) / chf.cs)
    local minY = truncTowardZero((position.Y - chf.bmin.Y) / chf.ch)
    local minZ = truncTowardZero((position.Z - radius - chf.bmin.Z) / chf.cs)
    local maxX = truncTowardZero((position.X + radius - chf.bmin.X) / chf.cs)
    local maxY = truncTowardZero((position.Y + height - chf.bmin.Y) / chf.ch)
    local maxZ = truncTowardZero((position.Z + radius - chf.bmin.Z) / chf.cs)
    if maxX < 0 or minX >= chf.width or maxZ < 0 or minZ >= chf.height then return chf end
    minX = math.clamp(minX, 0, chf.width - 1); maxX = math.clamp(maxX, 0, chf.width - 1)
    minZ = math.clamp(minZ, 0, chf.height - 1); maxZ = math.clamp(maxZ, 0, chf.height - 1)
    local radiusSq = radius * radius
    for z = minZ, maxZ do
        for x = minX, maxX do
            local cellX = chf.bmin.X + (x + 0.5) * chf.cs
            local cellZ = chf.bmin.Z + (z + 0.5) * chf.cs
            local dx, dz = cellX - position.X, cellZ - position.Z
            if dx * dx + dz * dz >= radiusSq then continue end
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local y = buffer.readu16(chf.y, span * 2)
                if y >= minY and y <= maxY and buffer.readu8(chf.areas, span) ~= RC_NULL_AREA then
                    buffer.writeu8(chf.areas, span, areaId)
                end
            end
        end
    end
    return chf
end

local OFFSET_EPSILON = 1e-6
function Recast.offsetPoly(vertices, offset, maxOutVerts)
    local verts = unpackVertexList(vertices)
    local n = #verts
    maxOutVerts = maxOutVerts or n * 2
    if n < 3 then return {} end
    local out = {}
    local miterLimit = 1.20
    for i = 1, n do
        local a: vector = verts[((i - 2) % n) + 1]
        local b: vector = verts[i]
        local c: vector = verts[(i % n) + 1]
        local prevX, prevZ = b.X - a.X, b.Z - a.Z
        local currX, currZ = c.X - b.X, c.Z - b.Z
        local prevMag = prevX * prevX + prevZ * prevZ
        local currMag = currX * currX + currZ * currZ
        if prevMag > OFFSET_EPSILON then local inv = 1 / math.sqrt(prevMag); prevX *= inv; prevZ *= inv end
        if currMag > OFFSET_EPSILON then local inv = 1 / math.sqrt(currMag); currX *= inv; currZ *= inv end
        local cross = currX * prevZ - prevX * currZ
        local prevNormX, prevNormZ = -prevZ, prevX
        local currNormX, currNormZ = -currZ, currX
        local miterX = (prevNormX + currNormX) * 0.5
        local miterZ = (prevNormZ + currNormZ) * 0.5
        local miterSq = miterX * miterX + miterZ * miterZ
        local bevel = miterSq * miterLimit * miterLimit < 1
        if miterSq > OFFSET_EPSILON then local scale = 1 / miterSq; miterX *= scale; miterZ *= scale end
        if bevel and cross < 0 then
            if #out + 2 > maxOutVerts then return nil, 'output vertex limit exceeded' end
            local d = (1 - (prevX * currX + prevZ * currZ)) * 0.5
            out[#out + 1] = vector.create(b.X + (-prevNormX + prevX * d) * offset, b.Y, b.Z + (-prevNormZ + prevZ * d) * offset)
            out[#out + 1] = vector.create(b.X + (-currNormX - currX * d) * offset, b.Y, b.Z + (-currNormZ - currZ * d) * offset)
        else
            if #out + 1 > maxOutVerts then return nil, 'output vertex limit exceeded' end
            out[#out + 1] = vector.create(b.X - miterX * offset, b.Y, b.Z - miterZ * offset)
        end
    end
    return out
end


-- Exact Recast-style distance transform used by watershed region construction.
-- Distances are 16-bit chamfer units (orthogonal=2, diagonal=3), matching rcBuildDistanceField.
function Recast.buildDistanceField(chf)
    local src = buffer.create(chf.spanCount * 2)
    local dst = buffer.create(chf.spanCount * 2)
    buffer.fill(src, 0, 0xff)

    local w, h = chf.width, chf.height
    for z = 0, h - 1 do
        for x = 0, w - 1 do
            local cell = x + z * w
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local area = buffer.readu8(chf.areas, span)
                local nc = 0
                for dir0 = 0, 3 do
                    local con = getCon(chf.con, span, dir0)
                    if con ~= RC_NOT_CONNECTED then
                        local nx, nz = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
                        local neighbor = buffer.readu32(chf.cellIndex, (nx + nz * w) * 4) + con
                        if buffer.readu8(chf.areas, neighbor) == area then nc += 1 end
                    end
                end
                if nc ~= 4 then buffer.writeu16(src, span * 2, 0) end
            end
        end
    end

    local function relax(target, source, weight)
        local sv = buffer.readu16(src, source * 2)
        local candidate = math.min(0xffff, sv + weight)
        if candidate < buffer.readu16(src, target * 2) then buffer.writeu16(src, target * 2, candidate) end
    end

    for z = 0, h - 1 do
        for x = 0, w - 1 do
            local cell = x + z * w
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local con0 = getCon(chf.con, span, 0)
                if con0 ~= RC_NOT_CONNECTED then
                    local ax, az = x - 1, z
                    local a = buffer.readu32(chf.cellIndex, (ax + az * w) * 4) + con0
                    relax(span, a, 2)
                    local con3 = getCon(chf.con, a, 3)
                    if con3 ~= RC_NOT_CONNECTED then
                        local b = buffer.readu32(chf.cellIndex, (ax + (az - 1) * w) * 4) + con3
                        relax(span, b, 3)
                    end
                end
                local con3 = getCon(chf.con, span, 3)
                if con3 ~= RC_NOT_CONNECTED then
                    local ax, az = x, z - 1
                    local a = buffer.readu32(chf.cellIndex, (ax + az * w) * 4) + con3
                    relax(span, a, 2)
                    local con2 = getCon(chf.con, a, 2)
                    if con2 ~= RC_NOT_CONNECTED then
                        local b = buffer.readu32(chf.cellIndex, ((ax + 1) + az * w) * 4) + con2
                        relax(span, b, 3)
                    end
                end
            end
        end
    end

    for z = h - 1, 0, -1 do
        for x = w - 1, 0, -1 do
            local cell = x + z * w
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local con2 = getCon(chf.con, span, 2)
                if con2 ~= RC_NOT_CONNECTED then
                    local ax, az = x + 1, z
                    local a = buffer.readu32(chf.cellIndex, (ax + az * w) * 4) + con2
                    relax(span, a, 2)
                    local con1 = getCon(chf.con, a, 1)
                    if con1 ~= RC_NOT_CONNECTED then
                        local b = buffer.readu32(chf.cellIndex, (ax + (az + 1) * w) * 4) + con1
                        relax(span, b, 3)
                    end
                end
                local con1 = getCon(chf.con, span, 1)
                if con1 ~= RC_NOT_CONNECTED then
                    local ax, az = x, z + 1
                    local a = buffer.readu32(chf.cellIndex, (ax + az * w) * 4) + con1
                    relax(span, a, 2)
                    local con0 = getCon(chf.con, a, 0)
                    if con0 ~= RC_NOT_CONNECTED then
                        local b = buffer.readu32(chf.cellIndex, ((ax - 1) + az * w) * 4) + con0
                        relax(span, b, 3)
                    end
                end
            end
        end
    end

    local maxDist = 0
    for i = 0, chf.spanCount - 1 do maxDist = math.max(maxDist, buffer.readu16(src, i * 2)) end
    chf.maxDistance = maxDist

    -- rcBoxBlur(chf, 1, ...)
    local threshold = 2
    for z = 0, h - 1 do
        for x = 0, w - 1 do
            local cell = x + z * w
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local cd = buffer.readu16(src, span * 2)
                if cd <= threshold then
                    buffer.writeu16(dst, span * 2, cd)
                    continue
                end
                local d = cd
                for dir0 = 0, 3 do
                    local con = getCon(chf.con, span, dir0)
                    if con ~= RC_NOT_CONNECTED then
                        local ax, az = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
                        local a = buffer.readu32(chf.cellIndex, (ax + az * w) * 4) + con
                        d += buffer.readu16(src, a * 2)
                        local dir2 = (dir0 + 1) % 4
                        local con2 = getCon(chf.con, a, dir2)
                        if con2 ~= RC_NOT_CONNECTED then
                            local bx, bz = ax + DIR_X[dir2 + 1], az + DIR_Z[dir2 + 1]
                            local b = buffer.readu32(chf.cellIndex, (bx + bz * w) * 4) + con2
                            d += buffer.readu16(src, b * 2)
                        else
                            d += cd
                        end
                    else
                        d += cd * 2
                    end
                end
                buffer.writeu16(dst, span * 2, (d + 5) // 9)
            end
        end
    end
    chf.dist = dst
    return true
end

local function contourPoint(points, i0, component0)
    return points[i0 * 4 + component0 + 1]
end

local function contourSetPoint(points, i0, x, y, z, r)
    local o = i0 * 4 + 1
    points[o], points[o + 1], points[o + 2], points[o + 3] = x, y, z, r
end

local function contourPush(points, x, y, z, r)
    local n = #points
    points[n + 1], points[n + 2], points[n + 3], points[n + 4] = x, y, z, r
end

local function contourInsert(points, i0, x, y, z, r)
    local o = i0 * 4 + 1
    table.insert(points, o, x)
    table.insert(points, o + 1, y)
    table.insert(points, o + 2, z)
    table.insert(points, o + 3, r)
end

local function contourRemove(points, i0)
    local o = i0 * 4 + 1
    table.remove(points, o); table.remove(points, o); table.remove(points, o); table.remove(points, o)
end

local function contourRegionAt(chf, span)
    if chf.regions == nil then return 0 end
    return buffer.readu16(chf.regions, span * 2)
end

local function getCornerHeight(x, z, span, dir0, chf)
    local height = buffer.readu16(chf.y, span * 2)
    local dirp = (dir0 + 1) % 4
    local regs = {0, 0, 0, 0}
    regs[1] = bit32.bor(contourRegionAt(chf, span), bit32.lshift(buffer.readu8(chf.areas, span), 16))

    local con = getCon(chf.con, span, dir0)
    if con ~= RC_NOT_CONNECTED then
        local ax, az = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
        local a = buffer.readu32(chf.cellIndex, (ax + az * chf.width) * 4) + con
        height = math.max(height, buffer.readu16(chf.y, a * 2))
        regs[2] = bit32.bor(contourRegionAt(chf, a), bit32.lshift(buffer.readu8(chf.areas, a), 16))
        local con2 = getCon(chf.con, a, dirp)
        if con2 ~= RC_NOT_CONNECTED then
            local bx, bz = ax + DIR_X[dirp + 1], az + DIR_Z[dirp + 1]
            local b = buffer.readu32(chf.cellIndex, (bx + bz * chf.width) * 4) + con2
            height = math.max(height, buffer.readu16(chf.y, b * 2))
            regs[3] = bit32.bor(contourRegionAt(chf, b), bit32.lshift(buffer.readu8(chf.areas, b), 16))
        end
    end

    con = getCon(chf.con, span, dirp)
    if con ~= RC_NOT_CONNECTED then
        local ax, az = x + DIR_X[dirp + 1], z + DIR_Z[dirp + 1]
        local a = buffer.readu32(chf.cellIndex, (ax + az * chf.width) * 4) + con
        height = math.max(height, buffer.readu16(chf.y, a * 2))
        regs[4] = bit32.bor(contourRegionAt(chf, a), bit32.lshift(buffer.readu8(chf.areas, a), 16))
        local con2 = getCon(chf.con, a, dir0)
        if con2 ~= RC_NOT_CONNECTED then
            local bx, bz = ax + DIR_X[dir0 + 1], az + DIR_Z[dir0 + 1]
            local b = buffer.readu32(chf.cellIndex, (bx + bz * chf.width) * 4) + con2
            height = math.max(height, buffer.readu16(chf.y, b * 2))
            regs[3] = bit32.bor(contourRegionAt(chf, b), bit32.lshift(buffer.readu8(chf.areas, b), 16))
        end
    end

    local border = false
    for j0 = 0, 3 do
        local a = j0 + 1
        local b = ((j0 + 1) % 4) + 1
        local c = ((j0 + 2) % 4) + 1
        local d = ((j0 + 3) % 4) + 1
        local twoSameExts = bit32.band(bit32.band(regs[a], regs[b]), RC_BORDER_REG) ~= 0 and regs[a] == regs[b]
        local twoInts = bit32.band(bit32.bor(regs[c], regs[d]), RC_BORDER_REG) == 0
        local intsSameArea = bit32.rshift(regs[c], 16) == bit32.rshift(regs[d], 16)
        local noZeros = regs[a] ~= 0 and regs[b] ~= 0 and regs[c] ~= 0 and regs[d] ~= 0
        if twoSameExts and twoInts and intsSameArea and noZeros then border = true; break end
    end
    return height, border
end

local function walkBuildContour(x0, z0, span0, chf, flags)
    local dir0 = 0
    while dir0 < 4 and bit32.band(buffer.readu8(flags, span0), bit32.lshift(1, dir0)) == 0 do dir0 += 1 end
    if dir0 >= 4 then return {} end
    local startDir, startSpan = dir0, span0
    local x, z, span = x0, z0, span0
    local area = buffer.readu8(chf.areas, span)
    local points = {}
    local iter = 0
    while iter < 40000 do
        iter += 1
        local f = buffer.readu8(flags, span)
        local bit = bit32.lshift(1, dir0)
        if bit32.band(f, bit) ~= 0 then
            local py, isBorderVertex = getCornerHeight(x, z, span, dir0, chf)
            local px, pz = x, z
            if dir0 == 0 then pz += 1
            elseif dir0 == 1 then px += 1; pz += 1
            elseif dir0 == 2 then px += 1 end
            local r = 0
            local isAreaBorder = false
            local con = getCon(chf.con, span, dir0)
            if con ~= RC_NOT_CONNECTED then
                local ax, az = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
                local a = buffer.readu32(chf.cellIndex, (ax + az * chf.width) * 4) + con
                r = contourRegionAt(chf, a)
                if area ~= buffer.readu8(chf.areas, a) then isAreaBorder = true end
            end
            if isBorderVertex then r = bit32.bor(r, RC_BORDER_VERTEX) end
            if isAreaBorder then r = bit32.bor(r, RC_AREA_BORDER) end
            contourPush(points, px, py, pz, r)
            buffer.writeu8(flags, span, bit32.band(f, bit32.bnot(bit)))
            dir0 = (dir0 + 1) % 4
        else
            local con = getCon(chf.con, span, dir0)
            if con == RC_NOT_CONNECTED then break end
            x += DIR_X[dir0 + 1]
            z += DIR_Z[dir0 + 1]
            span = buffer.readu32(chf.cellIndex, (x + z * chf.width) * 4) + con
            dir0 = (dir0 + 3) % 4
        end
        if span == startSpan and dir0 == startDir then break end
    end
    return points
end

local function distancePtSegSq(x, z, px, pz, qx, qz)
    local pqx, pqz = qx - px, qz - pz
    local dx, dz = x - px, z - pz
    local d = pqx * pqx + pqz * pqz
    local t = pqx * dx + pqz * dz
    if d > 0 then t /= d end
    t = math.clamp(t, 0, 1)
    dx, dz = px + t * pqx - x, pz + t * pqz - z
    return dx * dx + dz * dz
end

local function simplifyContour(points, maxError, maxEdgeLen, buildFlags)
    local simplified = {}
    local pn = #points // 4
    if pn == 0 then return simplified end
    local hasConnections = false
    for i0 = 0, pn - 1 do
        if bit32.band(contourPoint(points, i0, 3), RC_CONTOUR_REG_MASK) ~= 0 then hasConnections = true; break end
    end
    if hasConnections then
        for i0 = 0, pn - 1 do
            local ii = (i0 + 1) % pn
            local a, b = contourPoint(points, i0, 3), contourPoint(points, ii, 3)
            local differentRegs = bit32.band(a, RC_CONTOUR_REG_MASK) ~= bit32.band(b, RC_CONTOUR_REG_MASK)
            local areaBorders = bit32.band(a, RC_AREA_BORDER) ~= bit32.band(b, RC_AREA_BORDER)
            if differentRegs or areaBorders then
                contourPush(simplified, contourPoint(points, i0, 0), contourPoint(points, i0, 1), contourPoint(points, i0, 2), i0)
            end
        end
    end
    if #simplified == 0 then
        local lli, uri = 0, 0
        local llx, llz = contourPoint(points, 0, 0), contourPoint(points, 0, 2)
        local urx, urz = llx, llz
        for i0 = 1, pn - 1 do
            local x, z = contourPoint(points, i0, 0), contourPoint(points, i0, 2)
            if x < llx or (x == llx and z < llz) then llx, llz, lli = x, z, i0 end
            if x > urx or (x == urx and z > urz) then urx, urz, uri = x, z, i0 end
        end
        contourPush(simplified, contourPoint(points, lli, 0), contourPoint(points, lli, 1), contourPoint(points, lli, 2), lli)
        contourPush(simplified, contourPoint(points, uri, 0), contourPoint(points, uri, 1), contourPoint(points, uri, 2), uri)
    end

    local i0 = 0
    while i0 < #simplified // 4 do
        local sn = #simplified // 4
        local ii = (i0 + 1) % sn
        local ax, az, ai = contourPoint(simplified, i0, 0), contourPoint(simplified, i0, 2), contourPoint(simplified, i0, 3)
        local bx, bz, bi = contourPoint(simplified, ii, 0), contourPoint(simplified, ii, 2), contourPoint(simplified, ii, 3)
        local ci, cinc, endi
        if bx > ax or (bx == ax and bz > az) then
            cinc = 1; ci = (ai + 1) % pn; endi = bi
        else
            cinc = pn - 1; ci = (bi + cinc) % pn; endi = ai
            ax, bx = bx, ax; az, bz = bz, az
        end
        local maxd, maxi = 0, -1
        local cr = contourPoint(points, ci, 3)
        if bit32.band(cr, RC_CONTOUR_REG_MASK) == 0 or bit32.band(cr, RC_AREA_BORDER) ~= 0 then
            while ci ~= endi do
                local d = distancePtSegSq(contourPoint(points, ci, 0), contourPoint(points, ci, 2), ax, az, bx, bz)
                if d > maxd then maxd, maxi = d, ci end
                ci = (ci + cinc) % pn
            end
        end
        if maxi ~= -1 and maxd > maxError * maxError then
            contourInsert(simplified, i0 + 1, contourPoint(points, maxi, 0), contourPoint(points, maxi, 1), contourPoint(points, maxi, 2), maxi)
        else
            i0 += 1
        end
    end

    if maxEdgeLen > 0 and bit32.band(buildFlags, bit32.bor(Recast.RC_CONTOUR_TESS_WALL_EDGES, Recast.RC_CONTOUR_TESS_AREA_EDGES)) ~= 0 then
        i0 = 0
        while i0 < #simplified // 4 do
            local sn = #simplified // 4
            local ii = (i0 + 1) % sn
            local ax, az, ai = contourPoint(simplified, i0, 0), contourPoint(simplified, i0, 2), contourPoint(simplified, i0, 3)
            local bx, bz, bi = contourPoint(simplified, ii, 0), contourPoint(simplified, ii, 2), contourPoint(simplified, ii, 3)
            local ci = (ai + 1) % pn
            local rawFlags = contourPoint(points, ci, 3)
            local tess = bit32.band(buildFlags, Recast.RC_CONTOUR_TESS_WALL_EDGES) ~= 0 and bit32.band(rawFlags, RC_CONTOUR_REG_MASK) == 0
            if bit32.band(buildFlags, Recast.RC_CONTOUR_TESS_AREA_EDGES) ~= 0 and bit32.band(rawFlags, RC_AREA_BORDER) ~= 0 then tess = true end
            local maxi = -1
            if tess then
                local dx, dz = bx - ax, bz - az
                if dx * dx + dz * dz > maxEdgeLen * maxEdgeLen then
                    local n = if bi < ai then bi + pn - ai else bi - ai
                    if n > 1 then
                        if bx > ax or (bx == ax and bz > az) then maxi = (ai + n // 2) % pn
                        else maxi = (ai + (n + 1) // 2) % pn end
                    end
                end
            end
            if maxi ~= -1 then
                contourInsert(simplified, i0 + 1, contourPoint(points, maxi, 0), contourPoint(points, maxi, 1), contourPoint(points, maxi, 2), maxi)
            else
                i0 += 1
            end
        end
    end

    local sn = #simplified // 4
    for j0 = 0, sn - 1 do
        local raw = contourPoint(simplified, j0, 3)
        local ai, bi = (raw + 1) % pn, raw
        local flags = bit32.bor(
            bit32.band(contourPoint(points, ai, 3), bit32.bor(RC_CONTOUR_REG_MASK, RC_AREA_BORDER)),
            bit32.band(contourPoint(points, bi, 3), RC_BORDER_VERTEX)
        )
        simplified[j0 * 4 + 4] = flags
    end
    return simplified
end

local function sameContourXZ(points, a0, b0)
    return contourPoint(points, a0, 0) == contourPoint(points, b0, 0) and contourPoint(points, a0, 2) == contourPoint(points, b0, 2)
end

local function removeDegenerateSegments(points)
    local i0 = 0
    while i0 < #points // 4 and #points >= 8 do
        local n = #points // 4
        local ni = (i0 + 1) % n
        if sameContourXZ(points, i0, ni) then contourRemove(points, i0)
        else i0 += 1 end
    end
end

local function contourArea2D(points)
    local n = #points // 4
    local area, j0 = 0, n - 1
    for i0 = 0, n - 1 do
        area += contourPoint(points, i0, 0) * contourPoint(points, j0, 2) - contourPoint(points, j0, 0) * contourPoint(points, i0, 2)
        j0 = i0
    end
    return (area + 1) // 2
end

local function area2Flat(points, a0, b0, c0)
    return (contourPoint(points, b0, 0) - contourPoint(points, a0, 0)) * (contourPoint(points, c0, 2) - contourPoint(points, a0, 2))
        - (contourPoint(points, c0, 0) - contourPoint(points, a0, 0)) * (contourPoint(points, b0, 2) - contourPoint(points, a0, 2))
end

local function pointArea2(ax, az, bx, bz, cx, cz)
    return (bx - ax) * (cz - az) - (cx - ax) * (bz - az)
end

local function segmentIntersect(ax, az, bx, bz, cx, cz, dx, dz)
    local function collinear(px,pz,qx,qz,rx,rz) return pointArea2(px,pz,qx,qz,rx,rz) == 0 end
    local function left(px,pz,qx,qz,rx,rz) return pointArea2(px,pz,qx,qz,rx,rz) < 0 end
    local function between(px,pz,qx,qz,rx,rz)
        if not collinear(px,pz,qx,qz,rx,rz) then return false end
        if px ~= qx then return (px <= rx and rx <= qx) or (px >= rx and rx >= qx) end
        return (pz <= rz and rz <= qz) or (pz >= rz and rz >= qz)
    end
    if not collinear(ax,az,bx,bz,cx,cz) and not collinear(ax,az,bx,bz,dx,dz)
       and not collinear(cx,cz,dx,dz,ax,az) and not collinear(cx,cz,dx,dz,bx,bz)
       and (left(ax,az,bx,bz,cx,cz) ~= left(ax,az,bx,bz,dx,dz))
       and (left(cx,cz,dx,dz,ax,az) ~= left(cx,cz,dx,dz,bx,bz)) then return true end
    return between(ax,az,bx,bz,cx,cz) or between(ax,az,bx,bz,dx,dz) or between(cx,cz,dx,dz,ax,az) or between(cx,cz,dx,dz,bx,bz)
end

local function pointInCone(points, i0, px, pz)
    local n = #points // 4
    local prev0 = if i0 > 0 then i0 - 1 else n - 1
    local next0 = if i0 + 1 < n then i0 + 1 else 0
    local ax, az = contourPoint(points, prev0, 0), contourPoint(points, prev0, 2)
    local bx, bz = contourPoint(points, i0, 0), contourPoint(points, i0, 2)
    local cx, cz = contourPoint(points, next0, 0), contourPoint(points, next0, 2)
    local convex = pointArea2(ax, az, bx, bz, cx, cz) <= 0
    if convex then return pointArea2(bx,bz,px,pz,ax,az) < 0 and pointArea2(px,pz,bx,bz,cx,cz) < 0 end
    return not (pointArea2(bx,bz,px,pz,cx,cz) <= 0 and pointArea2(px,pz,bx,bz,ax,az) <= 0)
end

local function intersectSegContour(ax, az, bx, bz, skipVertex0, points)
    local n = #points // 4
    for k0 = 0, n - 1 do
        local k1 = (k0 + 1) % n
        if skipVertex0 == k0 or skipVertex0 == k1 then continue end
        local cx, cz = contourPoint(points, k0, 0), contourPoint(points, k0, 2)
        local dx, dz = contourPoint(points, k1, 0), contourPoint(points, k1, 2)
        if (ax == cx and az == cz) or (bx == cx and bz == cz) or (ax == dx and az == dz) or (bx == dx and bz == dz) then continue end
        if segmentIntersect(ax,az,bx,bz,cx,cz,dx,dz) then return true end
    end
    return false
end

local function mergeContourPair(outline, hole, ia, ib)
    local merged = {}
    for i = 0, outline.nverts do
        local src = (ia + i) % outline.nverts
        contourPush(merged, contourPoint(outline.verts, src, 0), contourPoint(outline.verts, src, 1), contourPoint(outline.verts, src, 2), contourPoint(outline.verts, src, 3))
    end
    for i = 0, hole.nverts do
        local src = (ib + i) % hole.nverts
        contourPush(merged, contourPoint(hole.verts, src, 0), contourPoint(hole.verts, src, 1), contourPoint(hole.verts, src, 2), contourPoint(hole.verts, src, 3))
    end
    outline.verts = merged; outline.nverts = #merged // 4
    hole.verts = {}; hole.nverts = 0
end

local function mergeRegionHoles(region)
    table.sort(region.holes, function(a, b)
        if a.minx == b.minx then return a.minz < b.minz end
        return a.minx < b.minx
    end)
    local outline = region.outline
    if outline == nil then return end
    for holeIndex = 1, #region.holes do
        local holeInfo = region.holes[holeIndex]
        local hole = holeInfo.contour
        local bestVertex = holeInfo.leftmost
        local index = -1
        for _ = 1, hole.nverts do
            local cx, cz = contourPoint(hole.verts, bestVertex, 0), contourPoint(hole.verts, bestVertex, 2)
            local diags = {}
            for j0 = 0, outline.nverts - 1 do
                if pointInCone(outline.verts, j0, cx, cz) then
                    local dx, dz = contourPoint(outline.verts, j0, 0) - cx, contourPoint(outline.verts, j0, 2) - cz
                    diags[#diags + 1] = {vert = j0, dist = dx * dx + dz * dz}
                end
            end
            table.sort(diags, function(a,b) return a.dist < b.dist end)
            for _, diag in diags do
                local px, pz = contourPoint(outline.verts, diag.vert, 0), contourPoint(outline.verts, diag.vert, 2)
                local hit = intersectSegContour(px, pz, cx, cz, diag.vert, outline.verts)
                if not hit then
                    for k = holeIndex, #region.holes do
                        if intersectSegContour(px, pz, cx, cz, -1, region.holes[k].contour.verts) then hit = true; break end
                    end
                end
                if not hit then index = diag.vert; break end
            end
            if index ~= -1 then break end
            bestVertex = (bestVertex + 1) % hole.nverts
        end
        if index ~= -1 then mergeContourPair(outline, hole, index, bestVertex) end
    end
end

function Recast.buildContours(chf, maxError, maxEdgeLen, buildFlags)
    maxError = maxError or 1.3
    maxEdgeLen = maxEdgeLen or 0
    buildFlags = if buildFlags == nil then Recast.RC_CONTOUR_TESS_WALL_EDGES else buildFlags
    if chf.regions == nil then Recast.buildConnectedRegions(chf) end
    local borderSize = chf.borderSize or 0
    local pad = borderSize * chf.cs
    local cset = {
        conts = {}, nconts = 0,
        bmin = vector.create(chf.bmin.X + pad, chf.bmin.Y, chf.bmin.Z + pad),
        bmax = vector.create(chf.bmax.X - pad, chf.bmax.Y, chf.bmax.Z - pad),
        cs = chf.cs, ch = chf.ch,
        width = chf.width - borderSize * 2,
        height = chf.height - borderSize * 2,
        borderSize = borderSize,
        maxError = maxError,
    }
    local flags = buffer.create(chf.spanCount)
    for z = 0, chf.height - 1 do
        for x = 0, chf.width - 1 do
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local reg = contourRegionAt(chf, span)
                if reg == 0 or bit32.band(reg, RC_BORDER_REG) ~= 0 then buffer.writeu8(flags, span, 0); continue end
                local res = 0
                for dir0 = 0, 3 do
                    local r = 0
                    local con = getCon(chf.con, span, dir0)
                    if con ~= RC_NOT_CONNECTED then
                        local nx, nz = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
                        local neighbor = buffer.readu32(chf.cellIndex, (nx + nz * chf.width) * 4) + con
                        r = contourRegionAt(chf, neighbor)
                    end
                    if r == reg then res = bit32.bor(res, bit32.lshift(1, dir0)) end
                end
                buffer.writeu8(flags, span, bit32.bxor(res, 0xf))
            end
        end
    end

    for z = 0, chf.height - 1 do
        for x = 0, chf.width - 1 do
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local f = buffer.readu8(flags, span)
                if f == 0 or f == 0xf then buffer.writeu8(flags, span, 0); continue end
                local reg = contourRegionAt(chf, span)
                if reg == 0 or bit32.band(reg, RC_BORDER_REG) ~= 0 then continue end
                local raw = walkBuildContour(x, z, span, chf, flags)
                local simplified = simplifyContour(raw, maxError, maxEdgeLen, buildFlags)
                removeDegenerateSegments(simplified)
                if #simplified // 4 >= 3 then
                    if borderSize > 0 then
                        for j0 = 0, #simplified // 4 - 1 do
                            simplified[j0 * 4 + 1] -= borderSize
                            simplified[j0 * 4 + 3] -= borderSize
                        end
                        for j0 = 0, #raw // 4 - 1 do
                            raw[j0 * 4 + 1] -= borderSize
                            raw[j0 * 4 + 3] -= borderSize
                        end
                    end
                    cset.conts[#cset.conts + 1] = {
                        verts = simplified, nverts = #simplified // 4,
                        rverts = raw, nrverts = #raw // 4,
                        reg = reg, area = buffer.readu8(chf.areas, span),
                    }
                end
            end
        end
    end
    cset.nconts = #cset.conts

    -- Match rcBuildContours hole stitching: negative winding contours are holes of the region outline.
    local byRegion = {}
    for _, cont in cset.conts do
        if cont.nverts < 3 then continue end
        local group = byRegion[cont.reg]
        if group == nil then group = {outline = nil, holes = {}}; byRegion[cont.reg] = group end
        if contourArea2D(cont.verts) < 0 then
            local minx, minz, leftmost = contourPoint(cont.verts, 0, 0), contourPoint(cont.verts, 0, 2), 0
            for i0 = 1, cont.nverts - 1 do
                local x, z = contourPoint(cont.verts, i0, 0), contourPoint(cont.verts, i0, 2)
                if x < minx or (x == minx and z < minz) then minx, minz, leftmost = x, z, i0 end
            end
            group.holes[#group.holes + 1] = {contour = cont, minx = minx, minz = minz, leftmost = leftmost}
        elseif group.outline == nil then
            group.outline = cont
        end
    end
    for _, group in byRegion do if group.outline ~= nil and #group.holes > 0 then mergeRegionHoles(group) end end
    return cset
end

local function meshTriArea2(points, a0, b0, c0)
    return area2Flat(points, a0, b0, c0)
end

local function meshLeft(points, a0, b0, c0) return meshTriArea2(points,a0,b0,c0) < 0 end
local function meshLeftOn(points, a0, b0, c0) return meshTriArea2(points,a0,b0,c0) <= 0 end
local function meshCollinear(points, a0,b0,c0) return meshTriArea2(points,a0,b0,c0) == 0 end

local function meshBetween(points, a0,b0,c0)
    if not meshCollinear(points,a0,b0,c0) then return false end
    local ax,bx,cx = contourPoint(points,a0,0),contourPoint(points,b0,0),contourPoint(points,c0,0)
    if ax ~= bx then return (ax <= cx and cx <= bx) or (ax >= cx and cx >= bx) end
    local az,bz,cz = contourPoint(points,a0,2),contourPoint(points,b0,2),contourPoint(points,c0,2)
    return (az <= cz and cz <= bz) or (az >= cz and cz >= bz)
end

local function meshIntersect(points, a0,b0,c0,d0, properOnly)
    local prop = not meshCollinear(points,a0,b0,c0) and not meshCollinear(points,a0,b0,d0)
        and not meshCollinear(points,c0,d0,a0) and not meshCollinear(points,c0,d0,b0)
        and (meshLeft(points,a0,b0,c0) ~= meshLeft(points,a0,b0,d0))
        and (meshLeft(points,c0,d0,a0) ~= meshLeft(points,c0,d0,b0))
    if prop or properOnly then return prop end
    return meshBetween(points,a0,b0,c0) or meshBetween(points,a0,b0,d0) or meshBetween(points,c0,d0,a0) or meshBetween(points,c0,d0,b0)
end

local function meshVEqual(points,a0,b0)
    return contourPoint(points,a0,0)==contourPoint(points,b0,0) and contourPoint(points,a0,2)==contourPoint(points,b0,2)
end

local function diagonalie(i,j,n,points,indices,loose)
    local di,dj = indices[i+1],indices[j+1]
    for k=0,n-1 do
        local k1=(k+1)%n
        if k==i or k1==i or k==j or k1==j then continue end
        local p0,p1=indices[k+1],indices[k1+1]
        if meshVEqual(points,di,p0) or meshVEqual(points,dj,p0) or meshVEqual(points,di,p1) or meshVEqual(points,dj,p1) then continue end
        if meshIntersect(points,di,dj,p0,p1,loose) then return false end
    end
    return true
end

local function inConeMesh(i,j,n,points,indices,loose)
    local pi,pj=indices[i+1],indices[j+1]
    local pi1=indices[((i+1)%n)+1]
    local pin1=indices[((i-1+n)%n)+1]
    if loose then
        if meshLeftOn(points,pin1,pi,pi1) then return meshLeftOn(points,pi,pj,pin1) and meshLeftOn(points,pj,pi,pi1) end
        return not (meshLeftOn(points,pi,pj,pi1) and meshLeftOn(points,pj,pi,pin1))
    end
    if meshLeftOn(points,pin1,pi,pi1) then return meshLeft(points,pi,pj,pin1) and meshLeft(points,pj,pi,pi1) end
    return not (meshLeftOn(points,pi,pj,pi1) and meshLeftOn(points,pj,pi,pin1))
end

local function validDiagonal(i,j,n,points,indices,loose)
    return inConeMesh(i,j,n,points,indices,loose) and diagonalie(i,j,n,points,indices,loose)
end

local function triangulateContour(points)
    local n=#points//4
    local indices, ears = table.create(n), table.create(n)
    for i=0,n-1 do indices[i+1]=i end
    for i=0,n-1 do ears[((i+1)%n)+1]=validDiagonal(i,(i+2)%n,n,points,indices,false) end
    local tris={}
    while n>3 do
        local minLen,mini=nil,-1
        for i=0,n-1 do
            local i1=(i+1)%n
            if ears[i1+1] then
                local p0,p2=indices[i+1],indices[((i1+1)%n)+1]
                local dx=contourPoint(points,p2,0)-contourPoint(points,p0,0)
                local dz=contourPoint(points,p2,2)-contourPoint(points,p0,2)
                local len=dx*dx+dz*dz
                if minLen==nil or len<minLen then minLen,mini=len,i end
            end
        end
        if mini==-1 then
            for i=0,n-1 do
                local i1=(i+1)%n
                local i2=(i1+1)%n
                if validDiagonal(i,i2,n,points,indices,true) then
                    local p0,p2=indices[i+1],indices[((i2+1)%n)+1]
                    local dx=contourPoint(points,p2,0)-contourPoint(points,p0,0)
                    local dz=contourPoint(points,p2,2)-contourPoint(points,p0,2)
                    local len=dx*dx+dz*dz
                    if minLen==nil or len<minLen then minLen,mini=len,i end
                end
            end
            if mini==-1 then return tris,false end
        end
        local i=mini; local i1=(i+1)%n; local i2=(i1+1)%n
        tris[#tris+1]={indices[i+1],indices[i1+1],indices[i2+1]}
        table.remove(indices,i1+1); table.remove(ears,i1+1); n-=1
        if i1>=n then i1=0 end
        i=(i1-1+n)%n
        ears[i+1]=validDiagonal((i-1+n)%n,i1,n,points,indices,false)
        ears[i1+1]=validDiagonal(i,(i1+1)%n,n,points,indices,false)
    end
    tris[#tris+1]={indices[1],indices[2],indices[3]}
    return tris,true
end

local function polyMergeValue(pa,pb,verts,nvp)
    local na,nb=#pa,#pb
    if na+nb-2>nvp then return -1 end
    local ea,eb=-1,-1
    for i0=0,na-1 do
        local a0,a1=pa[i0+1],pa[((i0+1)%na)+1]
        if a0>a1 then a0,a1=a1,a0 end
        for j0=0,nb-1 do
            local b0,b1=pb[j0+1],pb[((j0+1)%nb)+1]
            if b0>b1 then b0,b1=b1,b0 end
            if a0==b0 and a1==b1 then ea,eb=i0,j0; break end
        end
        if ea~=-1 then break end
    end
    if ea==-1 then return -1 end
    local function uleft(ia,ib,ic)
        local a,b,c=verts[ia+1],verts[ib+1],verts[ic+1]
        return (b.x-a.x)*(c.z-a.z)-(c.x-a.x)*(b.z-a.z)<0
    end
    if not uleft(pa[((ea-1+na)%na)+1],pa[ea+1],pb[((eb+2)%nb)+1]) then return -1 end
    if not uleft(pb[((eb-1+nb)%nb)+1],pb[eb+1],pa[((ea+2)%na)+1]) then return -1 end
    local va,vb=verts[pa[ea+1]+1],verts[pa[((ea+1)%na)+1]+1]
    local dx,dz=va.x-vb.x,va.z-vb.z
    return dx*dx+dz*dz,ea,eb
end

local function mergePoly(pa,pb,ea,eb)
    local out={}
    for i=0,#pa-2 do out[#out+1]=pa[((ea+1+i)%#pa)+1] end
    for i=0,#pb-2 do out[#out+1]=pb[((eb+1+i)%#pb)+1] end
    return out
end


local function polyMergeValueAt(pa, pb, verts, nvp, ea, eb)
    local na, nb = #pa, #pb
    if na + nb - 2 > nvp then return -1 end
    local function uleft(ia, ib, ic)
        local a, b, c = verts[ia + 1], verts[ib + 1], verts[ic + 1]
        return (b.x - a.x) * (c.z - a.z) - (c.x - a.x) * (b.z - a.z) < 0
    end
    if not uleft(pa[((ea - 1 + na) % na) + 1], pa[ea + 1], pb[((eb + 2) % nb) + 1]) then return -1 end
    if not uleft(pb[((eb - 1 + nb) % nb) + 1], pb[eb + 1], pa[((ea + 2) % na) + 1]) then return -1 end
    local va, vb = verts[pa[ea + 1] + 1], verts[pa[((ea + 1) % na) + 1] + 1]
    local dx, dz = va.x - vb.x, va.z - vb.z
    return dx * dx + dz * dz
end

-- rcBuildPolyMesh checks every polygon pair to discover the tiny subset that share an edge.
-- In Luau that dominates complex contour builds.  Build the adjacency candidates first, then
-- evaluate them in the same lexicographic (pa,pb) order so > tie-breaking remains byte-compatible.
local function bestPolyMerge(polys, verts, nvp)
    local edgeOwners, pairEdges, pairKeys = {}, {}, {}
    for p = 1, #polys do
        local poly = polys[p]
        for e0 = 0, #poly - 1 do
            local a, b = poly[e0 + 1], poly[((e0 + 1) % #poly) + 1]
            local lo, hi = math.min(a, b), math.max(a, b)
            local edgeKey0 = lo * 65536 + hi
            local prev = edgeOwners[edgeKey0]
            if prev == nil then
                edgeOwners[edgeKey0] = p * 65536 + e0
            else
                local pp, pe = prev // 65536, prev % 65536
                if pp ~= p then
                    local pa, pb, ea, eb = pp, p, pe, e0
                    if pa > pb then pa, pb, ea, eb = pb, pa, eb, ea end
                    local pairKey = pa * 65536 + pb
                    if pairEdges[pairKey] == nil then pairKeys[#pairKeys + 1] = pairKey end
                    pairEdges[pairKey] = ea * 65536 + eb + 1
                end
            end
        end
    end
    -- Numeric pair keys sort exactly like rcBuildPolyMesh's nested (pa,pb) loops, so this keeps
    -- equal-value tie breaking byte-compatible without probing every non-adjacent polygon pair.
    table.sort(pairKeys)
    local best, bpa, bpb, bea, beb = 0, -1, -1, -1, -1
    for _, pairKey in pairKeys do
        local pa, pb = pairKey // 65536, pairKey % 65536
        local packed = pairEdges[pairKey] - 1
        local ea, eb = packed // 65536, packed % 65536
        local value = polyMergeValueAt(polys[pa], polys[pb], verts, nvp, ea, eb)
        if value > best then best, bpa, bpb, bea, beb = value, pa, pb, ea, eb end
    end
    return best, bpa, bpb, bea, beb
end

local function canRemovePolyVertex(polys, rem)
    local numTouchedVerts, numRemainingEdges = 0, 0
    for _, poly in polys do
        local numRemoved = 0
        for _, vi in poly do
            if vi == rem then numTouchedVerts += 1; numRemoved += 1 end
        end
        if numRemoved > 0 then numRemainingEdges += #poly - (numRemoved + 1) end
    end
    if numRemainingEdges <= 2 then return false end

    local edges = {}
    for _, poly in polys do
        local nv = #poly
        for j = 1, nv do
            local k = if j == 1 then nv else j - 1
            if poly[j] == rem or poly[k] == rem then
                local a, b = poly[j], poly[k]
                if b == rem then a, b = b, a end
                local found = false
                for _, edge in edges do
                    if edge.b == b then edge.count += 1; found = true; break end
                end
                if not found then edges[#edges + 1] = {a = a, b = b, count = 1} end
            end
        end
    end
    local numOpenEdges = 0
    for _, edge in edges do if edge.count < 2 then numOpenEdges += 1 end end
    return numOpenEdges <= 2
end

local function removePolyVertex(verts, polys, regs, areas, rem, nvp, maxTris)
    local edges = {}
    local keptPolys, keptRegs, keptAreas = {}, {}, {}
    for p = 1, #polys do
        local poly = polys[p]
        local hasRem = false
        for _, vi in poly do if vi == rem then hasRem = true; break end end
        if hasRem then
            local nv = #poly
            for j = 1, nv do
                local k = if j == 1 then nv else j - 1
                if poly[j] ~= rem and poly[k] ~= rem then
                    edges[#edges + 1] = {a = poly[k], b = poly[j], reg = regs[p], area = areas[p]}
                end
            end
        else
            keptPolys[#keptPolys + 1] = poly
            keptRegs[#keptRegs + 1] = regs[p]
            keptAreas[#keptAreas + 1] = areas[p]
        end
    end
    table.clear(polys); table.clear(regs); table.clear(areas)
    for i = 1, #keptPolys do polys[i], regs[i], areas[i] = keptPolys[i], keptRegs[i], keptAreas[i] end

    table.remove(verts, rem + 1)
    for _, poly in polys do
        for j = 1, #poly do if poly[j] > rem then poly[j] -= 1 end end
    end
    for _, edge in edges do
        if edge.a > rem then edge.a -= 1 end
        if edge.b > rem then edge.b -= 1 end
    end
    if #edges == 0 then return true end

    local hole, hreg, harea = {edges[1].a}, {edges[1].reg}, {edges[1].area}
    while #edges > 0 do
        local matched = false
        local i = 1
        while i <= #edges do
            local edge = edges[i]
            local add = false
            if hole[1] == edge.b then
                table.insert(hole, 1, edge.a)
                table.insert(hreg, 1, edge.reg)
                table.insert(harea, 1, edge.area)
                add = true
            elseif hole[#hole] == edge.a then
                hole[#hole + 1] = edge.b
                hreg[#hreg + 1] = edge.reg
                harea[#harea + 1] = edge.area
                add = true
            end
            if add then table.remove(edges, i); matched = true else i += 1 end
        end
        if not matched then break end
    end
    if #hole < 3 then return true end

    local tverts = table.create(#hole * 4)
    for i = 1, #hole do
        local v = verts[hole[i] + 1]
        local o = (i - 1) * 4
        tverts[o + 1], tverts[o + 2], tverts[o + 3], tverts[o + 4] = v.x, v.y, v.z, 0
    end
    local tris = triangulateContour(tverts)
    local rebuilt, pregs, pareas = {}, {}, {}
    for _, tri in tris do
        local a, b, c = tri[1] + 1, tri[2] + 1, tri[3] + 1
        if a ~= b and a ~= c and b ~= c then
            rebuilt[#rebuilt + 1] = {hole[a], hole[b], hole[c]}
            pregs[#pregs + 1] = if hreg[a] ~= hreg[b] or hreg[b] ~= hreg[c] then RC_MULTIPLE_REGS else hreg[a]
            pareas[#pareas + 1] = harea[a]
        end
    end
    if #rebuilt == 0 then return true end

    if nvp > 3 then
        while true do
            local best, bpa, bpb, bea, beb = bestPolyMerge(rebuilt, verts, nvp)
            if best <= 0 then break end
            rebuilt[bpa] = mergePoly(rebuilt[bpa], rebuilt[bpb], bea, beb)
            if pregs[bpa] ~= pregs[bpb] then pregs[bpa] = RC_MULTIPLE_REGS end
            table.remove(rebuilt, bpb); table.remove(pregs, bpb); table.remove(pareas, bpb)
        end
    end

    for i = 1, #rebuilt do
        if #polys >= maxTris then break end
        polys[#polys + 1], regs[#regs + 1], areas[#areas + 1] = rebuilt[i], pregs[i], pareas[i]
    end
    return true
end

function Recast.buildPolyMesh(cset, nvp)
    nvp=nvp or 6
    if nvp<3 then error('nvp must be at least 3',2) end
    local verts, vertexBuckets, polys, regs, areas, vflags = {}, {}, {}, {}, {}, {}
    local maxTris = 0
    for _, cont in cset.conts do if cont.nverts >= 3 then maxTris += cont.nverts - 2 end end
    local function addVertex(x,y,z)
        local key=x..'|'..z
        local bucket=vertexBuckets[key]
        if bucket~=nil then
            for _,idx in bucket do local v=verts[idx+1]; if math.abs(v.y-y)<=2 then return idx end end
        else bucket={}; vertexBuckets[key]=bucket end
        local idx=#verts; verts[idx+1]={x=x,y=y,z=z}; vflags[idx+1]=false; bucket[#bucket+1]=idx; return idx
    end
    for _,cont in cset.conts do
        if cont.nverts<3 then continue end
        local tris=triangulateContour(cont.verts)
        local remap=table.create(cont.nverts)
        for i0=0,cont.nverts-1 do
            local vi = addVertex(contourPoint(cont.verts,i0,0),contourPoint(cont.verts,i0,1),contourPoint(cont.verts,i0,2))
            remap[i0+1]=vi
            if bit32.band(contourPoint(cont.verts,i0,3), RC_BORDER_VERTEX) ~= 0 then vflags[vi+1] = true end
        end
        local localPolys={}
        for _,t in tris do
            local a,b,c=remap[t[1]+1],remap[t[2]+1],remap[t[3]+1]
            if a~=b and a~=c and b~=c then localPolys[#localPolys+1]={a,b,c} end
        end
        if nvp>3 then
            while true do
                local best,bpa,bpb,bea,beb=bestPolyMerge(localPolys,verts,nvp)
                if best<=0 then break end
                localPolys[bpa]=mergePoly(localPolys[bpa],localPolys[bpb],bea,beb)
                table.remove(localPolys,bpb)
            end
        end
        for _,poly in localPolys do polys[#polys+1]=poly; regs[#regs+1]=cont.reg; areas[#areas+1]=cont.area end
    end

    -- Recast marks certain contour vertices introduced at tile-region boundaries for removal.
    -- Preserve its topology guards and hole re-triangulation instead of merely keeping them.
    local i0 = 0
    while i0 < #verts do
        if vflags[i0 + 1] and canRemovePolyVertex(polys, i0) then
            removePolyVertex(verts, polys, regs, areas, i0, nvp, maxTris)
            table.remove(vflags, i0 + 1)
        else
            i0 += 1
        end
    end

    local vertBuf=buffer.create(#verts*6)
    for vi=0,#verts-1 do local v=verts[vi+1]; local o=vi*6; buffer.writeu16(vertBuf,o,v.x); buffer.writeu16(vertBuf,o+2,v.y); buffer.writeu16(vertBuf,o+4,v.z) end
    local capacity = math.max(maxTris, #polys)
    local polyBuf=buffer.create(capacity*nvp*2*2)
    buffer.fill(polyBuf,0,0xff,buffer.len(polyBuf))
    local regBuf=buffer.create(capacity*2); local areaBuf=buffer.create(capacity); local flagBuf=buffer.create(capacity*2)
    local edgeMap={}
    for p0=0,#polys-1 do
        local poly=polys[p0+1]
        for e0=0,#poly-1 do
            local va,vb=poly[e0+1],poly[((e0+1)%#poly)+1]
            buffer.writeu16(polyBuf,(p0*nvp*2+e0)*2,va)
            local low,high=math.min(va,vb),math.max(va,vb); local key=low..'>'..high
            local prev=edgeMap[key]
            if prev==nil then edgeMap[key]={poly=p0,edge=e0}
            else
                buffer.writeu16(polyBuf,(p0*nvp*2+nvp+e0)*2,prev.poly)
                buffer.writeu16(polyBuf,(prev.poly*nvp*2+nvp+prev.edge)*2,p0)
            end
        end
        buffer.writeu16(regBuf,p0*2,regs[p0+1]); buffer.writeu8(areaBuf,p0,areas[p0+1])
    end
    if cset.borderSize>0 then
        for p0=0,#polys-1 do
            local poly=polys[p0+1]
            for e0=0,#poly-1 do
                local no=(p0*nvp*2+nvp+e0)*2
                if buffer.readu16(polyBuf,no)~=RC_MESH_NULL_IDX then continue end
                local a,b=verts[poly[e0+1]+1],verts[poly[((e0+1)%#poly)+1]+1]
                if a.x==0 and b.x==0 then buffer.writeu16(polyBuf,no,bit32.bor(0x8000,0))
                elseif a.z==cset.height and b.z==cset.height then buffer.writeu16(polyBuf,no,bit32.bor(0x8000,1))
                elseif a.x==cset.width and b.x==cset.width then buffer.writeu16(polyBuf,no,bit32.bor(0x8000,2))
                elseif a.z==0 and b.z==0 then buffer.writeu16(polyBuf,no,bit32.bor(0x8000,3)) end
            end
        end
    end
    return {
        verts=vertBuf, polys=polyBuf, regs=regBuf, flags=flagBuf, areas=areaBuf,
        nverts=#verts, npolys=#polys, maxpolys=capacity, nvp=nvp,
        bmin=cset.bmin,bmax=cset.bmax,cs=cset.cs,ch=cset.ch,borderSize=cset.borderSize,maxEdgeError=cset.maxError,
    }
end


function Recast.copyPolyMesh(src)
    local function cloneBuffer(b)
        local out = buffer.create(buffer.len(b))
        if buffer.len(b) > 0 then buffer.copy(out, 0, b, 0, buffer.len(b)) end
        return out
    end
    return {
        nverts = src.nverts,
        npolys = src.npolys,
        maxpolys = src.npolys,
        nvp = src.nvp,
        bmin = src.bmin,
        bmax = src.bmax,
        cs = src.cs,
        ch = src.ch,
        borderSize = src.borderSize,
        maxEdgeError = src.maxEdgeError,
        verts = cloneBuffer(src.verts),
        polys = cloneBuffer(src.polys),
        regs = cloneBuffer(src.regs),
        areas = cloneBuffer(src.areas),
        flags = cloneBuffer(src.flags),
    }
end

local function rebuildPolyMeshAdjacency(mesh)
    -- Clear only the neighbour half. Vertex indices remain untouched.
    for p0 = 0, mesh.npolys - 1 do
        for e0 = 0, mesh.nvp - 1 do
            buffer.writeu16(mesh.polys, (p0 * mesh.nvp * 2 + mesh.nvp + e0) * 2, RC_MESH_NULL_IDX)
        end
    end
    local edges = {}
    for p0 = 0, mesh.npolys - 1 do
        local nv = 0
        for e0 = 0, mesh.nvp - 1 do
            if buffer.readu16(mesh.polys, (p0 * mesh.nvp * 2 + e0) * 2) == RC_MESH_NULL_IDX then break end
            nv += 1
        end
        for e0 = 0, nv - 1 do
            local a = buffer.readu16(mesh.polys, (p0 * mesh.nvp * 2 + e0) * 2)
            local b = buffer.readu16(mesh.polys, (p0 * mesh.nvp * 2 + ((e0 + 1) % nv)) * 2)
            local low, high = math.min(a, b), math.max(a, b)
            local key = low .. '>' .. high
            local previous = edges[key]
            if previous == nil then
                edges[key] = {poly = p0, edge = e0}
            else
                buffer.writeu16(mesh.polys, (p0 * mesh.nvp * 2 + mesh.nvp + e0) * 2, previous.poly)
                buffer.writeu16(mesh.polys, (previous.poly * mesh.nvp * 2 + mesh.nvp + previous.edge) * 2, p0)
                edges[key] = false
            end
        end
    end
end

function Recast.mergePolyMeshes(meshes)
    if meshes == nil or #meshes == 0 then return nil end
    local first = meshes[1]
    local nvp, cs, ch = first.nvp, first.cs, first.ch
    local bmin: vector, bmax: vector = first.bmin, first.bmax
    local maxVerts, maxPolys = 0, 0
    for _, src in meshes do
        if src.nvp ~= nvp then error('all polymeshes must use the same nvp', 2) end
        if math.abs(src.cs - cs) > 1e-6 or math.abs(src.ch - ch) > 1e-6 then error('all polymeshes must use the same cell size/height', 2) end
        bmin = vector.create(math.min(bmin.X, src.bmin.X), math.min(bmin.Y, src.bmin.Y), math.min(bmin.Z, src.bmin.Z))
        bmax = vector.create(math.max(bmax.X, src.bmax.X), math.max(bmax.Y, src.bmax.Y), math.max(bmax.Z, src.bmax.Z))
        maxVerts += src.nverts
        maxPolys += src.npolys
    end

    local vertList, buckets = {}, {}
    local function addVertex(x, y, z)
        local key = x .. '|' .. z
        local bucket = buckets[key]
        if bucket ~= nil then
            for _, idx in bucket do if math.abs(vertList[idx + 1].y - y) <= 2 then return idx end end
        else
            bucket = {}; buckets[key] = bucket
        end
        local idx = #vertList
        vertList[idx + 1] = {x = x, y = y, z = z}
        bucket[#bucket + 1] = idx
        return idx
    end

    local outPolys, outRegs, outAreas, outFlags, sourcePortal = {}, {}, {}, {}, {}
    for _, src in meshes do
        local ox = math.floor((src.bmin.X - bmin.X) / cs + 0.5)
        local oz = math.floor((src.bmin.Z - bmin.Z) / cs + 0.5)
        local isMinX = ox == 0
        local isMinZ = oz == 0
        local isMaxX = math.floor((bmax.X - src.bmax.X) / cs + 0.5) == 0
        local isMaxZ = math.floor((bmax.Z - src.bmax.Z) / cs + 0.5) == 0
        local isOnBorder = isMinX or isMinZ or isMaxX or isMaxZ
        local remap = table.create(src.nverts)
        for v0 = 0, src.nverts - 1 do
            local vo = v0 * 6
            remap[v0 + 1] = addVertex(buffer.readu16(src.verts, vo) + ox, buffer.readu16(src.verts, vo + 2), buffer.readu16(src.verts, vo + 4) + oz)
        end
        for p0 = 0, src.npolys - 1 do
            local poly = {}
            local portals = {}
            local base = p0 * nvp * 2
            for e0 = 0, nvp - 1 do
                local vi = buffer.readu16(src.polys, (base + e0) * 2)
                if vi == RC_MESH_NULL_IDX then break end
                poly[#poly + 1] = remap[vi + 1]
                if isOnBorder then
                    local nei = buffer.readu16(src.polys, (base + nvp + e0) * 2)
                    if nei ~= RC_MESH_NULL_IDX and bit32.band(nei, 0x8000) ~= 0 then
                        local dir = bit32.band(nei, 0xf)
                        if (dir == 0 and isMinX) or (dir == 1 and isMaxZ) or (dir == 2 and isMaxX) or (dir == 3 and isMinZ) then portals[e0 + 1] = nei end
                    end
                end
            end
            outPolys[#outPolys + 1] = poly
            sourcePortal[#sourcePortal + 1] = portals
            outRegs[#outRegs + 1] = buffer.readu16(src.regs, p0 * 2)
            outAreas[#outAreas + 1] = buffer.readu8(src.areas, p0)
            outFlags[#outFlags + 1] = buffer.readu16(src.flags, p0 * 2)
        end
    end

    local verts = buffer.create(#vertList * 6)
    for v0 = 0, #vertList - 1 do
        local v = vertList[v0 + 1]; local o = v0 * 6
        buffer.writeu16(verts, o, v.x); buffer.writeu16(verts, o + 2, v.y); buffer.writeu16(verts, o + 4, v.z)
    end
    local polys = buffer.create(#outPolys * nvp * 4); buffer.fill(polys, 0, 0xff, buffer.len(polys))
    local regs, areas, flags = buffer.create(#outPolys * 2), buffer.create(#outPolys), buffer.create(#outPolys * 2)
    for p0 = 0, #outPolys - 1 do
        local poly = outPolys[p0 + 1]
        for e0 = 0, #poly - 1 do buffer.writeu16(polys, (p0 * nvp * 2 + e0) * 2, poly[e0 + 1]) end
        buffer.writeu16(regs, p0 * 2, outRegs[p0 + 1]); buffer.writeu8(areas, p0, outAreas[p0 + 1]); buffer.writeu16(flags, p0 * 2, outFlags[p0 + 1])
    end
    local mesh = {
        verts = verts, polys = polys, regs = regs, areas = areas, flags = flags,
        nverts = #vertList, npolys = #outPolys, maxpolys = #outPolys, nvp = nvp,
        bmin = bmin, bmax = bmax, cs = cs, ch = ch, borderSize = first.borderSize, maxEdgeError = first.maxEdgeError,
    }
    rebuildPolyMeshAdjacency(mesh)
    -- Adjacency rebuild intentionally destroys internal-tile portal markers. Re-apply only portal
    -- markers that are still on the outer boundary of the merged mesh, matching rcMergePolyMeshes.
    for p0 = 0, mesh.npolys - 1 do
        local portals = sourcePortal[p0 + 1]
        for e0, portal in portals do
            local edge0 = e0 - 1
            local no = (p0 * nvp * 2 + nvp + edge0) * 2
            if buffer.readu16(mesh.polys, no) == RC_MESH_NULL_IDX then buffer.writeu16(mesh.polys, no, portal) end
        end
    end
    return mesh
end

function Recast.buildPolyMeshDetail(mesh, chf, sampleDist, sampleMaxError)
    return require('@recast/detail').buildPolyMeshDetail(mesh, chf, sampleDist, sampleMaxError)
end

function Recast.mergePolyMeshDetails(meshes)
    return require('@recast/detail').mergePolyMeshDetails(meshes)
end

function Recast.polyMeshToTriangles(mesh)
    local worldVerts=buffer.create(mesh.nverts*12)
    for i0=0,mesh.nverts-1 do
        local o=i0*6
        writeVec3f(worldVerts,i0+1,vector.create(
            mesh.bmin.X+buffer.readu16(mesh.verts,o)*mesh.cs,
            mesh.bmin.Y+buffer.readu16(mesh.verts,o+2)*mesh.ch,
            mesh.bmin.Z+buffer.readu16(mesh.verts,o+4)*mesh.cs
        ))
    end
    local triCount=0
    for p0=0,mesh.npolys-1 do
        local nv=0
        for j0=0,mesh.nvp-1 do if buffer.readu16(mesh.polys,(p0*mesh.nvp*2+j0)*2)==RC_MESH_NULL_IDX then break else nv+=1 end end
        if nv>=3 then triCount+=nv-2 end
    end
    local tris=buffer.create(triCount*12); local areas=buffer.create(triCount); local flags=buffer.create(triCount*2)
    local t0=0
    for p0=0,mesh.npolys-1 do
        local base=p0*mesh.nvp*2
        local first=buffer.readu16(mesh.polys,base*2)
        local nv=0
        for j0=0,mesh.nvp-1 do if buffer.readu16(mesh.polys,(base+j0)*2)==RC_MESH_NULL_IDX then break else nv+=1 end end
        for j0=1,nv-2 do
            local o=t0*12
            buffer.writeu32(tris,o,first); buffer.writeu32(tris,o+4,buffer.readu16(mesh.polys,(base+j0)*2)); buffer.writeu32(tris,o+8,buffer.readu16(mesh.polys,(base+j0+1)*2))
            buffer.writeu8(areas,t0,buffer.readu8(mesh.areas,p0)); buffer.writeu16(flags,t0*2,buffer.readu16(mesh.flags,p0*2))
            t0+=1
        end
    end
    return worldVerts,tris,areas,flags
end


-- Recast region partitioning -------------------------------------------------
-- The functions below follow RecastRegion.cpp closely.  Region IDs live in a
-- separate u16 buffer because compact spans are structure-of-arrays in Luau.

local RC_NULL_NEI = 0xffff

local function paintRectRegion(minx, maxx, minz, maxz, regId, chf, srcReg)
    for z = minz, maxz - 1 do
        for x = minx, maxx - 1 do
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                if buffer.readu8(chf.areas, span) ~= RC_NULL_AREA then
                    buffer.writeu16(srcReg, span * 2, regId)
                end
            end
        end
    end
end

local function regionSolidEdge(chf, srcReg, x, z, span, dir0)
    local r = 0
    local con = getCon(chf.con, span, dir0)
    if con ~= RC_NOT_CONNECTED then
        local nx, nz = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
        local ni = buffer.readu32(chf.cellIndex, (nx + nz * chf.width) * 4) + con
        r = buffer.readu16(srcReg, ni * 2)
    end
    return r ~= buffer.readu16(srcReg, span * 2)
end

local function removeAdjacentRegionNeighbours(connections)
    local i = 1
    while i <= #connections and #connections > 1 do
        local ni = (i % #connections) + 1
        if connections[i] == connections[ni] then
            table.remove(connections, i)
        else
            i += 1
        end
    end
end

local function walkRegionContour(x, z, span, dir0, chf, srcReg, out)
    local startDir, startSpan = dir0, span
    local con = getCon(chf.con, span, dir0)
    local curReg = 0
    if con ~= RC_NOT_CONNECTED then
        local nx, nz = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
        local ni = buffer.readu32(chf.cellIndex, (nx + nz * chf.width) * 4) + con
        curReg = buffer.readu16(srcReg, ni * 2)
    end
    out[#out + 1] = curReg

    local iter = 0
    while iter < 40000 do
        iter += 1
        if regionSolidEdge(chf, srcReg, x, z, span, dir0) then
            local r = 0
            con = getCon(chf.con, span, dir0)
            if con ~= RC_NOT_CONNECTED then
                local nx, nz = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
                local ni = buffer.readu32(chf.cellIndex, (nx + nz * chf.width) * 4) + con
                r = buffer.readu16(srcReg, ni * 2)
            end
            if r ~= curReg then
                curReg = r
                out[#out + 1] = curReg
            end
            dir0 = (dir0 + 1) % 4
        else
            con = getCon(chf.con, span, dir0)
            if con == RC_NOT_CONNECTED then break end -- should never happen
            x += DIR_X[dir0 + 1]
            z += DIR_Z[dir0 + 1]
            span = buffer.readu32(chf.cellIndex, (x + z * chf.width) * 4) + con
            dir0 = (dir0 + 3) % 4
        end
        if span == startSpan and dir0 == startDir then break end
    end
    removeAdjacentRegionNeighbours(out)
end

local function newRegion(id)
    return {
        spanCount = 0,
        id = id,
        areaType = 0,
        remap = false,
        visited = false,
        overlap = false,
        connectsToBorder = false,
        ymin = 0xffff,
        ymax = 0,
        connections = {},
        floors = {},
    }
end

local function addUniqueFloorRegion(reg, id)
    for i = 1, #reg.floors do if reg.floors[i] == id then return end end
    reg.floors[#reg.floors + 1] = id
end

local function replaceRegionNeighbour(reg, oldId, newId)
    local changed = false
    for i = 1, #reg.connections do
        if reg.connections[i] == oldId then reg.connections[i] = newId; changed = true end
    end
    for i = 1, #reg.floors do
        if reg.floors[i] == oldId then reg.floors[i] = newId end
    end
    if changed then removeAdjacentRegionNeighbours(reg.connections) end
end

local function canMergeRegion(a, b)
    if a.areaType ~= b.areaType then return false end
    local n = 0
    for i = 1, #a.connections do if a.connections[i] == b.id then n += 1 end end
    if n > 1 then return false end
    for i = 1, #a.floors do if a.floors[i] == b.id then return false end end
    return true
end

local function mergeRegionInto(a, b)
    local aid, bid = a.id, b.id
    local insa, insb
    for i = 1, #a.connections do if a.connections[i] == bid then insa = i; break end end
    for i = 1, #b.connections do if b.connections[i] == aid then insb = i; break end end
    if not insa or not insb then return false end

    local acon = table.clone(a.connections)
    local bcon = b.connections
    table.clear(a.connections)
    local na, nb = #acon, #bcon
    for k = 1, na - 1 do a.connections[#a.connections + 1] = acon[((insa - 1 + k) % na) + 1] end
    for k = 1, nb - 1 do a.connections[#a.connections + 1] = bcon[((insb - 1 + k) % nb) + 1] end
    removeAdjacentRegionNeighbours(a.connections)
    for i = 1, #b.floors do addUniqueFloorRegion(a, b.floors[i]) end
    a.spanCount += b.spanCount
    b.spanCount = 0
    table.clear(b.connections)
    return true
end

local function regionConnectedToBorder(reg)
    for i = 1, #reg.connections do if reg.connections[i] == 0 then return true end end
    return false
end

local function mergeAndFilterRegions(minRegionArea, mergeRegionArea, maxRegionId, chf, srcReg)
    local nreg = maxRegionId + 1
    local regions = table.create(nreg)
    for id = 0, maxRegionId do regions[id + 1] = newRegion(id) end

    for z = 0, chf.height - 1 do
        for x = 0, chf.width - 1 do
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            local finish = first + count - 1
            for span = first, finish do
                local rid = buffer.readu16(srcReg, span * 2)
                if rid == 0 or rid >= nreg then continue end
                local reg = regions[rid + 1]
                reg.spanCount += 1

                for other = first, finish do
                    if other == span then continue end
                    local floorId = buffer.readu16(srcReg, other * 2)
                    if floorId == 0 or floorId >= nreg then continue end
                    if floorId == rid then reg.overlap = true end
                    addUniqueFloorRegion(reg, floorId)
                end

                if #reg.connections > 0 then continue end
                reg.areaType = buffer.readu8(chf.areas, span)
                local ndir = -1
                for dir0 = 0, 3 do
                    if regionSolidEdge(chf, srcReg, x, z, span, dir0) then ndir = dir0; break end
                end
                if ndir >= 0 then walkRegionContour(x, z, span, ndir, chf, srcReg, reg.connections) end
            end
        end
    end

    -- Remove connected islands whose aggregate size is below minRegionArea.
    local stack, trace = {}, {}
    for id = 0, maxRegionId do
        local reg = regions[id + 1]
        if reg.id == 0 or bit32.band(reg.id, RC_BORDER_REG) ~= 0 or reg.spanCount == 0 or reg.visited then continue end
        local connectsToBorder, spanCount = false, 0
        table.clear(stack); table.clear(trace)
        reg.visited = true
        stack[1] = id
        while #stack > 0 do
            local ri = table.remove(stack)
            local current = regions[ri + 1]
            spanCount += current.spanCount
            trace[#trace + 1] = ri
            for j = 1, #current.connections do
                local nid = current.connections[j]
                if bit32.band(nid, RC_BORDER_REG) ~= 0 then connectsToBorder = true; continue end
                local neighbor = regions[nid + 1]
                if neighbor and not neighbor.visited and neighbor.id ~= 0 and bit32.band(neighbor.id, RC_BORDER_REG) == 0 then
                    neighbor.visited = true
                    stack[#stack + 1] = nid
                end
            end
        end
        if spanCount < minRegionArea and not connectsToBorder then
            for j = 1, #trace do local dead = regions[trace[j] + 1]; dead.spanCount = 0; dead.id = 0 end
        end
    end

    -- Merge small compatible regions exactly like Recast's iterative pass.
    while true do
        local mergeCount = 0
        for i = 0, maxRegionId do
            local reg = regions[i + 1]
            if reg.id == 0 or bit32.band(reg.id, RC_BORDER_REG) ~= 0 or reg.overlap or reg.spanCount == 0 then continue end
            if reg.spanCount > mergeRegionArea and regionConnectedToBorder(reg) then continue end
            local smallest = 0x7fffffff
            local mergeId = reg.id
            for j = 1, #reg.connections do
                local nid = reg.connections[j]
                if bit32.band(nid, RC_BORDER_REG) ~= 0 then continue end
                local candidate = regions[nid + 1]
                if candidate and candidate.id ~= 0 and bit32.band(candidate.id, RC_BORDER_REG) == 0 and not candidate.overlap and candidate.spanCount < smallest
                    and canMergeRegion(reg, candidate) and canMergeRegion(candidate, reg) then
                    smallest = candidate.spanCount
                    mergeId = candidate.id
                end
            end
            if mergeId ~= reg.id then
                local oldId = reg.id
                local target = regions[mergeId + 1]
                if mergeRegionInto(target, reg) then
                    for j = 0, maxRegionId do
                        local fix = regions[j + 1]
                        if fix.id ~= 0 and bit32.band(fix.id, RC_BORDER_REG) == 0 then
                            if fix.id == oldId then fix.id = mergeId end
                            replaceRegionNeighbour(fix, oldId, mergeId)
                        end
                    end
                    mergeCount += 1
                end
            end
        end
        if mergeCount == 0 then break end
    end

    -- Compress IDs while keeping border regions untouched.
    for i = 0, maxRegionId do
        local reg = regions[i + 1]
        reg.remap = reg.id ~= 0 and bit32.band(reg.id, RC_BORDER_REG) == 0
    end
    local generated = 0
    for i = 0, maxRegionId do
        local reg = regions[i + 1]
        if not reg.remap then continue end
        local oldId = reg.id
        generated += 1
        for j = i, maxRegionId do
            local r = regions[j + 1]
            if r.id == oldId then r.id = generated; r.remap = false end
        end
    end
    for span = 0, chf.spanCount - 1 do
        local rid = buffer.readu16(srcReg, span * 2)
        if bit32.band(rid, RC_BORDER_REG) == 0 then
            buffer.writeu16(srcReg, span * 2, regions[rid + 1].id)
        end
    end
    local overlaps = [for index in range(1, maxRegionId + 1) if regions[index].overlap => regions[index].id]

    return generated, overlaps
end


local function addUniqueRegionConnection(reg, id)
    for i = 1, #reg.connections do if reg.connections[i] == id then return end end
    reg.connections[#reg.connections + 1] = id
end

local function mergeAndFilterLayerRegions(minRegionArea, maxRegionId, chf, srcReg)
    local nreg = maxRegionId + 1
    local regions = table.create(nreg)
    for i = 0, maxRegionId do regions[i + 1] = newRegion(i) end

    for z = 0, chf.height - 1 do
        for x = 0, chf.width - 1 do
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            local localRegs = {}
            for li = 0, count - 1 do
                local span = first + li
                local ri = buffer.readu16(srcReg, span * 2)
                if ri == 0 or ri >= nreg then continue end
                local reg = regions[ri + 1]
                reg.spanCount += 1
                reg.areaType = buffer.readu8(chf.areas, span)
                local sy = buffer.readu16(chf.y, span * 2)
                reg.ymin = math.min(reg.ymin, sy)
                reg.ymax = math.max(reg.ymax, sy)
                localRegs[#localRegs + 1] = ri
                for dir0 = 0, 3 do
                    local con = getCon(chf.con, span, dir0)
                    if con == RC_NOT_CONNECTED then continue end
                    local nx, nz = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
                    local ni = buffer.readu32(chf.cellIndex, (nx + nz * chf.width) * 4) + con
                    local nri = buffer.readu16(srcReg, ni * 2)
                    if nri > 0 and nri < nreg and nri ~= ri then addUniqueRegionConnection(reg, nri) end
                    if bit32.band(nri, RC_BORDER_REG) ~= 0 then reg.connectsToBorder = true end
                end
            end
            for a = 1, #localRegs - 1 do
                for b = a + 1, #localRegs do
                    local ra, rb = localRegs[a], localRegs[b]
                    if ra ~= rb then
                        addUniqueFloorRegion(regions[ra + 1], rb)
                        addUniqueFloorRegion(regions[rb + 1], ra)
                    end
                end
            end
        end
    end

    local layerId = 1
    for i = 0, maxRegionId do regions[i + 1].id = 0 end
    local queue = {}
    for i = 1, maxRegionId do
        local root = regions[i + 1]
        if root.id ~= 0 then continue end
        root.id = layerId
        table.clear(queue); queue[1] = i
        local head = 1
        while head <= #queue do
            local reg = regions[queue[head] + 1]; head += 1
            for j = 1, #reg.connections do
                local nei = reg.connections[j]
                if nei <= 0 or nei > maxRegionId then continue end
                local regn = regions[nei + 1]
                if regn.id ~= 0 or reg.areaType ~= regn.areaType then continue end
                local overlap = false
                for k = 1, #root.floors do if root.floors[k] == nei then overlap = true; break end end
                if overlap then continue end
                queue[#queue + 1] = nei
                regn.id = layerId
                for k = 1, #regn.floors do addUniqueFloorRegion(root, regn.floors[k]) end
                root.ymin = math.min(root.ymin, regn.ymin)
                root.ymax = math.max(root.ymax, regn.ymax)
                root.spanCount += regn.spanCount
                regn.spanCount = 0
                root.connectsToBorder = root.connectsToBorder or regn.connectsToBorder
            end
        end
        layerId += 1
    end

    for i = 0, maxRegionId do
        local reg = regions[i + 1]
        if reg.spanCount > 0 and reg.spanCount < minRegionArea and not reg.connectsToBorder then
            local deadId = reg.id
            for j = 0, maxRegionId do if regions[j + 1].id == deadId then regions[j + 1].id = 0 end end
        end
    end

    for i = 0, maxRegionId do
        local reg = regions[i + 1]
        reg.remap = reg.id ~= 0 and bit32.band(reg.id, RC_BORDER_REG) == 0
    end
    local generated = 0
    for i = 0, maxRegionId do
        local reg = regions[i + 1]
        if not reg.remap then continue end
        local oldId = reg.id
        generated += 1
        for j = i, maxRegionId do
            local r = regions[j + 1]
            if r.id == oldId then r.id = generated; r.remap = false end
        end
    end
    for span = 0, chf.spanCount - 1 do
        local rid = buffer.readu16(srcReg, span * 2)
        if bit32.band(rid, RC_BORDER_REG) == 0 then buffer.writeu16(srcReg, span * 2, regions[rid + 1].id) end
    end
    return generated
end

function Recast.buildLayerRegions(chf, borderSize, minRegionArea)
    borderSize = borderSize or 0
    minRegionArea = minRegionArea or 0
    local w, h = chf.width, chf.height
    local srcReg = buffer.create(chf.spanCount * 2)
    local id = 1

    if borderSize > 0 then
        local bw, bh = math.min(w, borderSize), math.min(h, borderSize)
        paintRectRegion(0, bw, 0, h, bit32.bor(id, RC_BORDER_REG), chf, srcReg); id += 1
        paintRectRegion(w - bw, w, 0, h, bit32.bor(id, RC_BORDER_REG), chf, srcReg); id += 1
        paintRectRegion(0, w, 0, bh, bit32.bor(id, RC_BORDER_REG), chf, srcReg); id += 1
        paintRectRegion(0, w, h - bh, h, bit32.bor(id, RC_BORDER_REG), chf, srcReg); id += 1
    end
    chf.borderSize = borderSize

    local sweeps = table.create(math.max(w, h) + 1)
    local prev = {}
    for z = borderSize, h - borderSize - 1 do
        table.clear(prev)
        local rid = 1
        for x = borderSize, w - borderSize - 1 do
            local cell = x + z * w
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local area = buffer.readu8(chf.areas, span)
                if area == RC_NULL_AREA then continue end
                local previd = 0
                local con0 = getCon(chf.con, span, 0)
                if con0 ~= RC_NOT_CONNECTED then
                    local ni = buffer.readu32(chf.cellIndex, ((x - 1) + z * w) * 4) + con0
                    local nr = buffer.readu16(srcReg, ni * 2)
                    if bit32.band(nr, RC_BORDER_REG) == 0 and area == buffer.readu8(chf.areas, ni) then previd = nr end
                end
                if previd == 0 then
                    previd = rid; rid += 1
                    sweeps[previd] = {rid = previd, id = 0, ns = 0, nei = 0}
                end
                local con3 = getCon(chf.con, span, 3)
                if con3 ~= RC_NOT_CONNECTED then
                    local ni = buffer.readu32(chf.cellIndex, (x + (z - 1) * w) * 4) + con3
                    local nr = buffer.readu16(srcReg, ni * 2)
                    local sw = sweeps[previd]
                    if nr ~= 0 and bit32.band(nr, RC_BORDER_REG) == 0 and area == buffer.readu8(chf.areas, ni) then
                        if sw.nei == 0 or sw.nei == nr then
                            sw.nei = nr; sw.ns += 1; prev[nr] = (prev[nr] or 0) + 1
                        else
                            sw.nei = RC_NULL_NEI
                        end
                    end
                end
                buffer.writeu16(srcReg, span * 2, previd)
            end
        end
        for sweepId = 1, rid - 1 do
            local sw = sweeps[sweepId]
            if sw.nei ~= RC_NULL_NEI and sw.nei ~= 0 and (prev[sw.nei] or 0) == sw.ns then sw.id = sw.nei
            else sw.id = id; id += 1 end
        end
        for x = borderSize, w - borderSize - 1 do
            local cell = x + z * w
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local sid = buffer.readu16(srcReg, span * 2)
                if sid > 0 and sid < rid then buffer.writeu16(srcReg, span * 2, sweeps[sid].id) end
            end
        end
    end

    local maxRegions = mergeAndFilterLayerRegions(minRegionArea, id, chf, srcReg)
    chf.regions, chf.maxRegions = srcReg, maxRegions
    chf.regionOverlaps = {}
    return srcReg, maxRegions
end

function Recast.buildRegionsMonotone(chf, borderSize, minRegionArea, mergeRegionArea)
    borderSize = borderSize or 0
    minRegionArea = minRegionArea or 0
    mergeRegionArea = mergeRegionArea or 0
    local w, h = chf.width, chf.height
    local srcReg = buffer.create(chf.spanCount * 2)
    local id = 1

    if borderSize > 0 then
        local bw, bh = math.min(w, borderSize), math.min(h, borderSize)
        paintRectRegion(0, bw, 0, h, bit32.bor(id, RC_BORDER_REG), chf, srcReg); id += 1
        paintRectRegion(w - bw, w, 0, h, bit32.bor(id, RC_BORDER_REG), chf, srcReg); id += 1
        paintRectRegion(0, w, 0, bh, bit32.bor(id, RC_BORDER_REG), chf, srcReg); id += 1
        paintRectRegion(0, w, h - bh, h, bit32.bor(id, RC_BORDER_REG), chf, srcReg); id += 1
    end
    chf.borderSize = borderSize

    local sweeps = table.create(math.max(w, h) + 1)
    local prev = {}
    for z = borderSize, h - borderSize - 1 do
        table.clear(prev)
        local rid = 1
        for x = borderSize, w - borderSize - 1 do
            local cell = x + z * w
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local area = buffer.readu8(chf.areas, span)
                if area == RC_NULL_AREA then continue end
                local previd = 0
                local con0 = getCon(chf.con, span, 0)
                if con0 ~= RC_NOT_CONNECTED then
                    local ni = buffer.readu32(chf.cellIndex, ((x - 1) + z * w) * 4) + con0
                    local nr = buffer.readu16(srcReg, ni * 2)
                    if bit32.band(nr, RC_BORDER_REG) == 0 and area == buffer.readu8(chf.areas, ni) then previd = nr end
                end
                if previd == 0 then
                    previd = rid; rid += 1
                    sweeps[previd] = { rid = previd, id = 0, ns = 0, nei = 0 }
                end
                local con3 = getCon(chf.con, span, 3)
                if con3 ~= RC_NOT_CONNECTED then
                    local ni = buffer.readu32(chf.cellIndex, (x + (z - 1) * w) * 4) + con3
                    local nr = buffer.readu16(srcReg, ni * 2)
                    local sw = sweeps[previd]
                    if nr ~= 0 and bit32.band(nr, RC_BORDER_REG) == 0 and area == buffer.readu8(chf.areas, ni) then
                        if sw.nei == 0 or sw.nei == nr then
                            sw.nei = nr; sw.ns += 1; prev[nr] = (prev[nr] or 0) + 1
                        else
                            sw.nei = RC_NULL_NEI
                        end
                    end
                end
                buffer.writeu16(srcReg, span * 2, previd)
            end
        end
        for sweepId = 1, rid - 1 do
            local sw = sweeps[sweepId]
            if sw.nei ~= RC_NULL_NEI and sw.nei ~= 0 and (prev[sw.nei] or 0) == sw.ns then
                sw.id = sw.nei
            else
                sw.id = id; id += 1
            end
        end
        for x = borderSize, w - borderSize - 1 do
            local cell = x + z * w
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local sid = buffer.readu16(srcReg, span * 2)
                if sid > 0 and sid < rid then buffer.writeu16(srcReg, span * 2, sweeps[sid].id) end
            end
        end
    end

    local maxRegions, overlaps = mergeAndFilterRegions(minRegionArea, mergeRegionArea, id, chf, srcReg)
    chf.regions, chf.maxRegions = srcReg, maxRegions
    chf.regionOverlaps = overlaps
    return srcReg, maxRegions
end

local function sortCellsByLevel(startLevel, chf, srcReg, nbStacks)
    local stacks = table.create(nbStacks)
    for i = 1, nbStacks do stacks[i] = {} end
    startLevel = bit32.rshift(startLevel, 1)
    for z = 0, chf.height - 1 do
        for x = 0, chf.width - 1 do
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                if buffer.readu8(chf.areas, span) == RC_NULL_AREA or buffer.readu16(srcReg, span * 2) ~= 0 then continue end
                local level = bit32.rshift(buffer.readu16(chf.dist, span * 2), 1)
                local sid = startLevel - level
                if sid >= nbStacks then continue end
                if sid < 0 then sid = 0 end
                local out = stacks[sid + 1]
                local n = #out
                out[n + 1], out[n + 2], out[n + 3] = x, z, span
            end
        end
    end
    return stacks
end

local function appendRegionStack(source, destination, srcReg)
    table.clear(destination)
    local dn = 0
    for j = 1, #source, 3 do
        local span = source[j + 2]
        if span >= 0 and buffer.readu16(srcReg, span * 2) == 0 then
            dn += 3
            destination[dn - 2], destination[dn - 1], destination[dn] = source[j], source[j + 1], span
        end
    end
end

local function expandRegions(maxIter, level, chf, srcReg, srcDist, stack, fillStack)
    if fillStack then
        table.clear(stack)
        local sn = 0
        for z = 0, chf.height - 1 do
            for x = 0, chf.width - 1 do
                local cell = x + z * chf.width
                local first = buffer.readu32(chf.cellIndex, cell * 4)
                local count = buffer.readu8(chf.cellCount, cell)
                for li = 0, count - 1 do
                    local span = first + li
                    if buffer.readu16(chf.dist, span * 2) >= level and buffer.readu16(srcReg, span * 2) == 0 and buffer.readu8(chf.areas, span) ~= RC_NULL_AREA then
                        sn += 3
                        stack[sn - 2], stack[sn - 1], stack[sn] = x, z, span
                    end
                end
            end
        end
    else
        -- Keep only unresolved candidates.  Upstream marks resolved entries with a
        -- tombstone and scans them again on every expansion pass; compacting here
        -- preserves candidate order while avoiding repeated dead table entries.
        local wn = 0
        for j = 1, #stack, 3 do
            local span = stack[j + 2]
            if span >= 0 and buffer.readu16(srcReg, span * 2) == 0 then
                wn += 3
                if wn ~= j + 2 then
                    stack[wn - 2], stack[wn - 1], stack[wn] = stack[j], stack[j + 1], span
                end
            end
        end
        for j = #stack, wn + 1, -1 do stack[j] = nil end
    end

    local original = stack
    local active = stack
    local spare = {}
    local dirty = {}
    local iter = 0
    while #active > 0 do
        table.clear(spare)
        table.clear(dirty)
        local sn, dn = 0, 0
        for j = 1, #active, 3 do
            local x, z, span = active[j], active[j + 1], active[j + 2]
            local r, d2 = buffer.readu16(srcReg, span * 2), 0xffff
            local area = buffer.readu8(chf.areas, span)
            local conBits = buffer.readu32(chf.con, span * 4)
            for dir0 = 0, 3 do
                local con = bit32.band(bit32.rshift(conBits, dir0 * 6), 0x3f)
                if con == RC_NOT_CONNECTED then continue end
                local nx, nz = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
                local ni = buffer.readu32(chf.cellIndex, (nx + nz * chf.width) * 4) + con
                if buffer.readu8(chf.areas, ni) ~= area then continue end
                local nr = buffer.readu16(srcReg, ni * 2)
                if nr > 0 and bit32.band(nr, RC_BORDER_REG) == 0 then
                    local nd = buffer.readu16(srcDist, ni * 2) + 2
                    if nd < d2 then r, d2 = nr, nd end
                end
            end
            if r ~= 0 then
                dn += 3
                dirty[dn - 2], dirty[dn - 1], dirty[dn] = span, r, d2
            else
                sn += 3
                spare[sn - 2], spare[sn - 1], spare[sn] = x, z, span
            end
        end
        for j = 1, #dirty, 3 do
            buffer.writeu16(srcReg, dirty[j] * 2, dirty[j + 1])
            buffer.writeu16(srcDist, dirty[j] * 2, dirty[j + 2])
        end
        if #dirty == 0 then break end

        -- Only unresolved spans can participate in the next pass.  Swap scratch
        -- tables instead of copying them every iteration; synchronize the caller's
        -- stack once when we return because buildRegions floods its survivors.
        active, spare = spare, active
        if level > 0 then
            iter += 1
            if iter >= maxIter then break end
        end
    end

    if active ~= original then
        table.clear(original)
        for j = 1, #active do original[j] = active[j] end
    end
end

local function floodRegion(x, z, seed, level, regionId, chf, srcReg, srcDist, stack)
    local area = buffer.readu8(chf.areas, seed)
    table.clear(stack)
    stack[1], stack[2], stack[3] = x, z, seed
    buffer.writeu16(srcReg, seed * 2, regionId); buffer.writeu16(srcDist, seed * 2, 0)
    local lev = if level >= 2 then level - 2 else 0
    local count = 0
    while #stack > 0 do
        local n = #stack
        local cx, cz, span = stack[n - 2], stack[n - 1], stack[n]
        stack[n], stack[n - 1], stack[n - 2] = nil, nil, nil
        local conflict = 0
        local conBits = buffer.readu32(chf.con, span * 4)
        for dir0 = 0, 3 do
            local con = bit32.band(bit32.rshift(conBits, dir0 * 6), 0x3f)
            if con == RC_NOT_CONNECTED then continue end
            local nx, nz = cx + DIR_X[dir0 + 1], cz + DIR_Z[dir0 + 1]
            local ni = buffer.readu32(chf.cellIndex, (nx + nz * chf.width) * 4) + con
            if buffer.readu8(chf.areas, ni) ~= area then continue end
            local nr = buffer.readu16(srcReg, ni * 2)
            if bit32.band(nr, RC_BORDER_REG) ~= 0 then continue end
            if nr ~= 0 and nr ~= regionId then conflict = nr; break end
            local dir2 = (dir0 + 1) % 4
            local con2 = getCon(chf.con, ni, dir2)
            if con2 ~= RC_NOT_CONNECTED then
                local nnx, nnz = nx + DIR_X[dir2 + 1], nz + DIR_Z[dir2 + 1]
                local nni = buffer.readu32(chf.cellIndex, (nnx + nnz * chf.width) * 4) + con2
                if buffer.readu8(chf.areas, nni) == area then
                    local nr2 = buffer.readu16(srcReg, nni * 2)
                    if nr2 ~= 0 and nr2 ~= regionId then conflict = nr2; break end
                end
            end
        end
        if conflict ~= 0 then buffer.writeu16(srcReg, span * 2, 0); continue end
        count += 1
        for dir0 = 0, 3 do
            local con = bit32.band(bit32.rshift(conBits, dir0 * 6), 0x3f)
            if con == RC_NOT_CONNECTED then continue end
            local nx, nz = cx + DIR_X[dir0 + 1], cz + DIR_Z[dir0 + 1]
            local ni = buffer.readu32(chf.cellIndex, (nx + nz * chf.width) * 4) + con
            if buffer.readu8(chf.areas, ni) ~= area then continue end
            if buffer.readu16(chf.dist, ni * 2) >= lev and buffer.readu16(srcReg, ni * 2) == 0 then
                buffer.writeu16(srcReg, ni * 2, regionId); buffer.writeu16(srcDist, ni * 2, 0)
                local sn = #stack
                stack[sn + 1], stack[sn + 2], stack[sn + 3] = nx, nz, ni
            end
        end
    end
    return count > 0
end

function Recast.buildRegions(chf, borderSize, minRegionArea, mergeRegionArea)
    borderSize = borderSize or 0
    minRegionArea = minRegionArea or 0
    mergeRegionArea = mergeRegionArea or 0
    if chf.dist == nil then error('buildRegions requires buildDistanceField first', 2) end
    local w, h = chf.width, chf.height
    local srcReg, srcDist = buffer.create(chf.spanCount * 2), buffer.create(chf.spanCount * 2)
    local regionId = 1
    local level = bit32.band(chf.maxDistance + 1, 0xfffe)
    local expandIters = 8

    if borderSize > 0 then
        local bw, bh = math.min(w, borderSize), math.min(h, borderSize)
        paintRectRegion(0, bw, 0, h, bit32.bor(regionId, RC_BORDER_REG), chf, srcReg); regionId += 1
        paintRectRegion(w - bw, w, 0, h, bit32.bor(regionId, RC_BORDER_REG), chf, srcReg); regionId += 1
        paintRectRegion(0, w, 0, bh, bit32.bor(regionId, RC_BORDER_REG), chf, srcReg); regionId += 1
        paintRectRegion(0, w, h - bh, h, bit32.bor(regionId, RC_BORDER_REG), chf, srcReg); regionId += 1
    end
    chf.borderSize = borderSize

    local NB_STACKS = 8
    local levelStacks = table.create(NB_STACKS)
    for i = 1, NB_STACKS do levelStacks[i] = {} end
    local floodStack = {}
    local sid = -1
    while level > 0 do
        level = if level >= 2 then level - 2 else 0
        sid = (sid + 1) % NB_STACKS
        if sid == 0 then
            levelStacks = sortCellsByLevel(level, chf, srcReg, NB_STACKS)
        else
            appendRegionStack(levelStacks[sid], levelStacks[sid + 1], srcReg)
        end
        local current = levelStacks[sid + 1]
        expandRegions(expandIters, level, chf, srcReg, srcDist, current, false)
        for j = 1, #current, 3 do
            local span = current[j + 2]
            if span >= 0 and buffer.readu16(srcReg, span * 2) == 0 then
                if floodRegion(current[j], current[j + 1], span, level, regionId, chf, srcReg, srcDist, floodStack) then
                    if regionId == 0xffff then error('Recast region id overflow', 2) end
                    regionId += 1
                end
            end
        end
    end
    expandRegions(expandIters * 8, 0, chf, srcReg, srcDist, floodStack, true)
    local maxRegions, overlaps = mergeAndFilterRegions(minRegionArea, mergeRegionArea, regionId, chf, srcReg)
    chf.regions, chf.maxRegions, chf.regionOverlaps = srcReg, maxRegions, overlaps
    return srcReg, maxRegions, overlaps
end


local RC_MAX_LAYERS = 63
local RC_MAX_NEIS = 16

local function arrayContains(a, v)
    for i = 1, #a do if a[i] == v then return true end end
    return false
end

local function addUniqueLimited(a, v, maxCount)
    if arrayContains(a, v) then return true end
    if #a >= maxCount then return false end
    a[#a + 1] = v
    return true
end

local function rangesOverlap(amin, amax, bmin, bmax)
    return not (amin > bmax or amax < bmin)
end

function Recast.buildHeightfieldLayers(chf, borderSize, walkableHeight)
    borderSize = borderSize or 0
    walkableHeight = walkableHeight or chf.walkableHeight
    local w, h = chf.width, chf.height
    local srcReg = buffer.create(chf.spanCount)
    buffer.fill(srcReg, 0, 0xff, chf.spanCount)
    local sweeps = table.create(w)
    local prevCount = {}
    local regId = 0

    -- Partition the compact field into monotone 8-bit regions, matching RecastLayers.cpp.
    for z = borderSize, h - borderSize - 1 do
        table.clear(prevCount)
        local sweepId = 0
        for x = borderSize, w - borderSize - 1 do
            local cell = x + z * w
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                if buffer.readu8(chf.areas, span) == RC_NULL_AREA then continue end
                local sid = 0xff
                local con0 = getCon(chf.con, span, 0)
                if con0 ~= RC_NOT_CONNECTED then
                    local ni = buffer.readu32(chf.cellIndex, ((x - 1) + z * w) * 4) + con0
                    if buffer.readu8(chf.areas, ni) ~= RC_NULL_AREA and buffer.readu8(srcReg, ni) ~= 0xff then sid = buffer.readu8(srcReg, ni) end
                end
                if sid == 0xff then
                    sid = sweepId; sweepId += 1
                    sweeps[sid + 1] = {nei = 0xff, ns = 0, id = 0}
                end
                local con3 = getCon(chf.con, span, 3)
                if con3 ~= RC_NOT_CONNECTED then
                    local ni = buffer.readu32(chf.cellIndex, (x + (z - 1) * w) * 4) + con3
                    local nr = buffer.readu8(srcReg, ni)
                    if nr ~= 0xff then
                        local sw = sweeps[sid + 1]
                        if sw.ns == 0 then sw.nei = nr end
                        if sw.nei == nr then
                            sw.ns += 1; prevCount[nr] = (prevCount[nr] or 0) + 1
                        else
                            sw.nei = 0xff
                        end
                    end
                end
                buffer.writeu8(srcReg, span, sid)
            end
        end
        for sid = 0, sweepId - 1 do
            local sw = sweeps[sid + 1]
            if sw.nei ~= 0xff and (prevCount[sw.nei] or 0) == sw.ns then
                sw.id = sw.nei
            else
                if regId == 255 then error('Recast heightfield layer region id overflow', 2) end
                sw.id = regId; regId += 1
            end
        end
        for x = borderSize, w - borderSize - 1 do
            local cell = x + z * w
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local sid = buffer.readu8(srcReg, span)
                if sid ~= 0xff then buffer.writeu8(srcReg, span, sweeps[sid + 1].id) end
            end
        end
    end

    local nregs = regId
    local regs = table.create(nregs)
    for i = 0, nregs - 1 do regs[i + 1] = {layers = {}, neis = {}, ymin = 0xffff, ymax = 0, layerId = 0xff, base = false} end

    -- Region neighbours and vertical overlap sets.
    for z = 0, h - 1 do
        for x = 0, w - 1 do
            local cell = x + z * w
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            local localRegs = {}
            for li = 0, count - 1 do
                local span = first + li
                local ri = buffer.readu8(srcReg, span)
                if ri == 0xff then continue end
                local reg = regs[ri + 1]
                local sy = buffer.readu16(chf.y, span * 2)
                reg.ymin = math.min(reg.ymin, sy); reg.ymax = math.max(reg.ymax, sy)
                if #localRegs < RC_MAX_LAYERS then localRegs[#localRegs + 1] = ri end
                for dir0 = 0, 3 do
                    local con = getCon(chf.con, span, dir0)
                    if con ~= RC_NOT_CONNECTED then
                        local nx, nz = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
                        local ni = buffer.readu32(chf.cellIndex, (nx + nz * w) * 4) + con
                        local nri = buffer.readu8(srcReg, ni)
                        if nri ~= 0xff and nri ~= ri then addUniqueLimited(reg.neis, nri, RC_MAX_NEIS) end
                    end
                end
            end
            for a = 1, #localRegs - 1 do
                for b = a + 1, #localRegs do
                    local ra, rb = localRegs[a], localRegs[b]
                    if ra ~= rb then
                        if not addUniqueLimited(regs[ra + 1].layers, rb, RC_MAX_LAYERS) or not addUniqueLimited(regs[rb + 1].layers, ra, RC_MAX_LAYERS) then
                            error('Recast layer overflow: too many overlapping walkable platforms', 2)
                        end
                    end
                end
            end
        end
    end

    -- Merge neighbouring non-overlapping regions into 2D layers.
    local layerId = 0
    local queue = {}
    for i = 0, nregs - 1 do
        local root = regs[i + 1]
        if root.layerId ~= 0xff then continue end
        root.layerId = layerId; root.base = true
        table.clear(queue); queue[1] = i
        local head = 1
        while head <= #queue do
            local reg = regs[queue[head] + 1]; head += 1
            for j = 1, #reg.neis do
                local ni = reg.neis[j]
                local neighbor = regs[ni + 1]
                if neighbor.layerId ~= 0xff or arrayContains(root.layers, ni) then continue end
                local ymin, ymax = math.min(root.ymin, neighbor.ymin), math.max(root.ymax, neighbor.ymax)
                if ymax - ymin >= 255 then continue end
                if #queue < 64 then
                    queue[#queue + 1] = ni; neighbor.layerId = layerId
                    for k = 1, #neighbor.layers do
                        if not addUniqueLimited(root.layers, neighbor.layers[k], RC_MAX_LAYERS) then error('Recast layer overflow: too many overlapping walkable platforms', 2) end
                    end
                    root.ymin, root.ymax = ymin, ymax
                end
            end
        end
        layerId += 1
    end

    -- Merge separate base layers that are close in height and do not overlap in XZ.
    local mergeHeight = walkableHeight * 4
    for i = 0, nregs - 1 do
        local ri = regs[i + 1]
        if not ri.base then continue end
        local newId = ri.layerId
        while true do
            local oldId = 0xff
            for j = 0, nregs - 1 do
                if i == j then continue end
                local rj = regs[j + 1]
                if not rj.base then continue end
                if not rangesOverlap(ri.ymin, ri.ymax + mergeHeight, rj.ymin, rj.ymax + mergeHeight) then continue end
                if math.max(ri.ymax, rj.ymax) - math.min(ri.ymin, rj.ymin) >= 255 then continue end
                local overlap = false
                for k = 0, nregs - 1 do
                    if regs[k + 1].layerId == rj.layerId and arrayContains(ri.layers, k) then overlap = true; break end
                end
                if not overlap then oldId = rj.layerId; break end
            end
            if oldId == 0xff then break end
            for j = 0, nregs - 1 do
                local rj = regs[j + 1]
                if rj.layerId == oldId then
                    rj.base = false; rj.layerId = newId
                    for k = 1, #rj.layers do
                        if not addUniqueLimited(ri.layers, rj.layers[k], RC_MAX_LAYERS) then error('Recast layer overflow: too many overlapping walkable platforms', 2) end
                    end
                    ri.ymin = math.min(ri.ymin, rj.ymin); ri.ymax = math.max(ri.ymax, rj.ymax)
                end
            end
        end
    end

    local used, remap = {}, {}
    for i = 0, nregs - 1 do used[regs[i + 1].layerId] = true end
    local compactCount = 0
    for id = 0, 255 do if used[id] then remap[id] = compactCount; compactCount += 1 else remap[id] = 0xff end end
    for i = 0, nregs - 1 do regs[i + 1].layerId = remap[regs[i + 1].layerId] end
    if compactCount == 0 then return {nlayers = 0, layers = {}} end

    local lw, lh = w - borderSize * 2, h - borderSize * 2
    local baseMin: vector = vector.create(chf.bmin.X + borderSize * chf.cs, chf.bmin.Y, chf.bmin.Z + borderSize * chf.cs)
    local baseMax: vector = vector.create(chf.bmax.X - borderSize * chf.cs, chf.bmax.Y, chf.bmax.Z - borderSize * chf.cs)
    local layers = table.create(compactCount)
    for curId = 0, compactCount - 1 do
        local hmin, hmax = 0, 0
        for j = 0, nregs - 1 do
            local r = regs[j + 1]
            if r.base and r.layerId == curId then hmin, hmax = r.ymin, r.ymax end
        end
        local heights, areas, cons = buffer.create(lw * lh), buffer.create(lw * lh), buffer.create(lw * lh)
        buffer.fill(heights, 0, 0xff, lw * lh)
        local layer = {
            width = lw, height = lh, cs = chf.cs, ch = chf.ch,
            bmin = vector.create(baseMin.X, baseMin.Y + hmin * chf.ch, baseMin.Z),
            bmax = vector.create(baseMax.X, baseMin.Y + hmax * chf.ch, baseMax.Z),
            hmin = hmin, hmax = hmax, minx = lw, maxx = 0, miny = lh, maxy = 0,
            heights = heights, areas = areas, cons = cons,
        }
        for z = 0, lh - 1 do
            for x = 0, lw - 1 do
                local cx, cz = borderSize + x, borderSize + z
                local cell = cx + cz * w
                local first = buffer.readu32(chf.cellIndex, cell * 4)
                local count = buffer.readu8(chf.cellCount, cell)
                for li = 0, count - 1 do
                    local span = first + li
                    local sri = buffer.readu8(srcReg, span)
                    if sri == 0xff or regs[sri + 1].layerId ~= curId then continue end
                    layer.minx = math.min(layer.minx, x); layer.maxx = math.max(layer.maxx, x)
                    layer.miny = math.min(layer.miny, z); layer.maxy = math.max(layer.maxy, z)
                    local idx = x + z * lw
                    buffer.writeu8(heights, idx, buffer.readu16(chf.y, span * 2) - hmin)
                    buffer.writeu8(areas, idx, buffer.readu8(chf.areas, span))
                    local portal, connection = 0, 0
                    for dir0 = 0, 3 do
                        local con = getCon(chf.con, span, dir0)
                        if con ~= RC_NOT_CONNECTED then
                            local nx, nz = cx + DIR_X[dir0 + 1], cz + DIR_Z[dir0 + 1]
                            local ni = buffer.readu32(chf.cellIndex, (nx + nz * w) * 4) + con
                            local nsri = buffer.readu8(srcReg, ni)
                            local nlid = if nsri ~= 0xff then regs[nsri + 1].layerId else 0xff
                            local narea = buffer.readu8(chf.areas, ni)
                            if narea ~= RC_NULL_AREA and curId ~= nlid then
                                portal = bit32.bor(portal, bit32.lshift(1, dir0))
                                local ny = buffer.readu16(chf.y, ni * 2)
                                if ny > hmin then buffer.writeu8(heights, idx, math.max(buffer.readu8(heights, idx), ny - hmin)) end
                            end
                            if narea ~= RC_NULL_AREA and curId == nlid then
                                local lx, lz = nx - borderSize, nz - borderSize
                                if lx >= 0 and lz >= 0 and lx < lw and lz < lh then connection = bit32.bor(connection, bit32.lshift(1, dir0)) end
                            end
                        end
                    end
                    buffer.writeu8(cons, idx, bit32.bor(bit32.lshift(portal, 4), connection))
                end
            end
        end
        if layer.minx > layer.maxx then layer.minx, layer.maxx = 0, 0 end
        if layer.miny > layer.maxy then layer.miny, layer.maxy = 0, 0 end
        layers[curId + 1] = layer
    end
    return {nlayers = compactCount, layers = layers}
end

-- Recast detail-mesh generation ------------------------------------------------
function Recast.buildConnectedRegions(chf, minRegionArea, removeSmall)
    minRegionArea = minRegionArea or 0
    local regions = buffer.create(chf.spanCount * 2)
    local qSpan, qX, qZ = {}, {}, {}
    local region = 0

    for z0 = 0, chf.height - 1 do
        for x0 = 0, chf.width - 1 do
            local cell = x0 + z0 * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local seed = first + li
                if buffer.readu8(chf.areas, seed) == RC_NULL_AREA or buffer.readu16(regions, seed * 2) ~= 0 then continue end
                region += 1
                if region >= 0xffff then error('too many compact regions', 2) end
                local head, tail = 1, 1
                qSpan[1], qX[1], qZ[1] = seed, x0, z0
                buffer.writeu16(regions, seed * 2, region)
                while head <= tail do
                    local span, x, z = qSpan[head], qX[head], qZ[head]
                    head += 1
                    for dir0 = 0, 3 do
                        local con = getCon(chf.con, span, dir0)
                        if con == RC_NOT_CONNECTED then continue end
                        local nx, nz = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
                        local nc = nx + nz * chf.width
                        local neighbor = buffer.readu32(chf.cellIndex, nc * 4) + con
                        if buffer.readu8(chf.areas, neighbor) == RC_NULL_AREA then continue end
                        if buffer.readu16(regions, neighbor * 2) ~= 0 then continue end
                        buffer.writeu16(regions, neighbor * 2, region)
                        tail += 1
                        qSpan[tail], qX[tail], qZ[tail] = neighbor, nx, nz
                    end
                end

                local size = tail
                if removeSmall and size < minRegionArea then
                    for qi = 1, tail do
                        local span = qSpan[qi]
                        buffer.writeu16(regions, span * 2, 0)
                        buffer.writeu8(chf.areas, span, RC_NULL_AREA)
                    end
                    region -= 1
                end
            end
        end
    end
    chf.regions = regions
    chf.maxRegions = region
    return regions, region
end

function Recast.buildCompactFromTriangles(vertices, triangles, config)
    config = config or {}
    local verts, _ = Common.packVertices(vertices)
    local tris, _ = Common.packTriangles(triangles)
    local bmin, bmax
    if config.bmin and config.bmax then
        bmin, bmax = config.bmin, config.bmax
    else
        bmin, bmax = Recast.calcBounds(verts)
    end
    local cs = config.cs or config.cellSize or 0.3
    local ch = config.ch or config.cellHeight or 0.2
    local width, height = Recast.calcGridSize(bmin, bmax, cs)
    if width <= 0 or height <= 0 then error('Recast grid is empty; check bounds/cell size', 2) end
    local hf = Recast.createHeightfield(width, height, bmin, bmax, cs, ch)
    local areas = Recast.rasterizeWalkableTriangles(
        config.walkableSlopeAngle or 45, verts, tris, hf, config.flagMergeThreshold or 1, config.areas
    )
    local walkableHeight = config.walkableHeight or math.ceil((config.agentHeight or 2.0) / ch)
    local walkableClimb = config.walkableClimb or math.floor((config.agentMaxClimb or 0.9) / ch)
    Recast.filterLowHangingWalkableObstacles(walkableClimb, hf)
    Recast.filterLedgeSpans(walkableHeight, walkableClimb, hf)
    Recast.filterWalkableLowHeightSpans(walkableHeight, hf)
    local chf = Recast.buildCompactHeightfield(walkableHeight, walkableClimb, hf)
    local radius = config.walkableRadius
    if radius == nil then radius = math.ceil((config.agentRadius or 0.6) / cs) end
    if radius > 0 then Recast.erodeWalkableArea(radius, chf) end
    if config.areaVolumes ~= nil then
        for _, volume in config.areaVolumes do
            local kind = volume.kind
            local area = volume.area or RC_WALKABLE_AREA
            if kind == 'box' then
                Recast.markBoxArea(volume.bmin, volume.bmax, area, chf)
            elseif kind == 'cylinder' then
                Recast.markCylinderArea(volume.position, volume.radius, volume.height, area, chf)
            elseif kind == 'convex' then
                Recast.markConvexPolyArea(volume.vertices, volume.minY, volume.maxY, area, chf)
            else
                error(`unknown Recast area volume kind '{tostring(kind)}'`, 2)
            end
        end
    end
    if config.medianFilterAreas == true then Recast.medianFilterWalkableArea(chf) end

    local partition = config.partitionType or config.partition or 'watershed'
    local borderSize = config.borderSize or 0
    local minRegionArea = config.minRegionArea or 0
    local mergeRegionArea = config.mergeRegionArea or 0
    if partition == 'watershed' then
        Recast.buildDistanceField(chf)
        Recast.buildRegions(chf, borderSize, minRegionArea, mergeRegionArea)
    elseif partition == 'monotone' then
        Recast.buildRegionsMonotone(chf, borderSize, minRegionArea, mergeRegionArea)
    elseif partition == 'layers' or partition == 'layer' then
        Recast.buildLayerRegions(chf, borderSize, minRegionArea)
    elseif partition == 'connected' then
        Recast.buildConnectedRegions(chf, minRegionArea, config.removeSmallRegions == true)
    else
        error(`unknown Recast partition type '{tostring(partition)}'`, 2)
    end
    return { heightfield = hf, compact = chf, areas = areas, config = config, partitionType = partition }
end

return table.freeze(Recast)
