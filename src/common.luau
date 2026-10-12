--!native

-- Pure-Luau support code for the Recast/Detour port.
-- Altered from RecastNavigation concepts; no C/C++ code is loaded or called.

local Common = {}

function Common.clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

function Common.min(a, b)
    return if a < b then a else b
end

function Common.max(a, b)
    return if a > b then a else b
end

function Common.abs(a)
    return if a < 0 then -a else a
end

function Common.sqr(a)
    return a * a
end

function Common.vec(x, y, z)
    return vector.create(x, y, z)
end

function Common.distanceSquared(a: vector, b: vector)
    local dx = a.X - b.X
    local dy = a.Y - b.Y
    local dz = a.Z - b.Z
    return dx * dx + dy * dy + dz * dz
end

function Common.distanceSquared2D(a: vector, b: vector)
    local dx = a.X - b.X
    local dz = a.Z - b.Z
    return dx * dx + dz * dz
end

function Common.triArea2D(a: vector, b: vector, c: vector)
    return (b.X - a.X) * (c.Z - a.Z) - (c.X - a.X) * (b.Z - a.Z)
end

function Common.vequal2D(a: vector, b: vector, eps)
    eps = eps or 1e-5
    local dx = a.X - b.X
    local dz = a.Z - b.Z
    return dx * dx + dz * dz <= eps * eps
end

function Common.ensureBuffer(old, oldBytes, newBytes)
    local out = buffer.create(newBytes)
    if old ~= nil and oldBytes > 0 then
        buffer.copy(out, 0, old, 0, oldBytes)
    end
    return out
end

function Common.readVec3f(b, index)
    local o = (index - 1) * 12
    return vector.create(buffer.readf32(b, o), buffer.readf32(b, o + 4), buffer.readf32(b, o + 8))
end

function Common.writeVec3f(b, index, v: vector)
    local o = (index - 1) * 12
    buffer.writef32(b, o, v.X)
    buffer.writef32(b, o + 4, v.Y)
    buffer.writef32(b, o + 8, v.Z)
end

function Common.packVertices(vertices)
    if typeof(vertices) == "buffer" then
        return vertices, buffer.len(vertices) // 12
    end
    local n = #vertices
    local out = buffer.create(n * 12)
    for i = 1, n do
        local v = vertices[i]
        local o = (i - 1) * 12
        if typeof(v) == "vector" then
            buffer.writef32(out, o, v.X)
            buffer.writef32(out, o + 4, v.Y)
            buffer.writef32(out, o + 8, v.Z)
        else
            buffer.writef32(out, o, v[1] or v.x)
            buffer.writef32(out, o + 4, v[2] or v.y)
            buffer.writef32(out, o + 8, v[3] or v.z)
        end
    end
    return out, n
end

function Common.packTriangles(triangles)
    if typeof(triangles) == "buffer" then
        return triangles, buffer.len(triangles) // 12
    end
    local flat = #triangles > 0 and type(triangles[1]) == "number"
    local count = if flat then #triangles // 3 else #triangles
    local out = buffer.create(count * 12)
    if flat then
        for i = 1, count * 3 do
            -- Public table indices are 1-based; packed indices are 0-based.
            buffer.writeu32(out, (i - 1) * 4, triangles[i] - 1)
        end
    else
        for i = 1, count do
            local t = triangles[i]
            local o = (i - 1) * 12
            buffer.writeu32(out, o, (t[1] or t.a) - 1)
            buffer.writeu32(out, o + 4, (t[2] or t.b) - 1)
            buffer.writeu32(out, o + 8, (t[3] or t.c) - 1)
        end
    end
    return out, count
end

function Common.vertexAt(vertices, index0)
    local o = index0 * 12
    return vector.create(buffer.readf32(vertices, o), buffer.readf32(vertices, o + 4), buffer.readf32(vertices, o + 8))
end

function Common.triangleAt(triangles, tri0)
    local o = tri0 * 12
    return buffer.readu32(triangles, o), buffer.readu32(triangles, o + 4), buffer.readu32(triangles, o + 8)
end

-- Binary min heap over integer refs. The priority callback is deliberately in Luau so callers can
-- store their scores in packed buffers without materializing node objects.
function Common.heapNew(priority)
    return { values = {}, priority = priority }
end

function Common.heapPush(heap, value)
    local values = heap.values
    local i = #values + 1
    values[i] = value
    local priority = heap.priority
    while i > 1 do
        local p = i // 2
        if priority(values[p]) <= priority(value) then break end
        values[i] = values[p]
        i = p
    end
    values[i] = value
end

function Common.heapPop(heap)
    local values = heap.values
    local n = #values
    if n == 0 then return nil end
    local root = values[1]
    local last = values[n]
    values[n] = nil
    n -= 1
    if n == 0 then return root end
    local priority = heap.priority
    local i = 1
    while true do
        local left = i * 2
        if left > n then break end
        local right = left + 1
        local child = left
        if right <= n and priority(values[right]) < priority(values[left]) then
            child = right
        end
        if priority(last) <= priority(values[child]) then break end
        values[i] = values[child]
        i = child
    end
    values[i] = last
    return root
end


-- Min-heap that carries the score beside the integer node. This is the Detour query hot path:
-- keeping the score in the heap avoids a Luau callback + packed-buffer read for every comparison.
function Common.scoreHeapNew()
    return { nodes = {}, scores = {} }
end

function Common.scoreHeapPush(heap, node, score)
    local nodes, scores = heap.nodes, heap.scores
    local i = #nodes + 1
    while i > 1 do
        local parent = i // 2
        if scores[parent] <= score then break end
        nodes[i], scores[i] = nodes[parent], scores[parent]
        i = parent
    end
    nodes[i], scores[i] = node, score
end

function Common.scoreHeapPop(heap)
    local nodes, scores = heap.nodes, heap.scores
    local n = #nodes
    if n == 0 then return nil end
    local rootNode, rootScore = nodes[1], scores[1]
    local lastNode, lastScore = nodes[n], scores[n]
    nodes[n], scores[n] = nil, nil
    n -= 1
    if n == 0 then return rootNode, rootScore end
    local i = 1
    while true do
        local left = i * 2
        if left > n then break end
        local right = left + 1
        local child = left
        if right <= n and scores[right] < scores[left] then child = right end
        if lastScore <= scores[child] then break end
        nodes[i], scores[i] = nodes[child], scores[child]
        i = child
    end
    nodes[i], scores[i] = lastNode, lastScore
    return rootNode, rootScore
end

return table.freeze(Common)
