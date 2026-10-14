--!native

local jt = require('@jess/test')
local suite = jt.new()
local rc = require('@recast/core')

local function expect(condition, message)
    if not condition then error(message or 'expectation failed', 2) end
end

local function approx(a, b, epsilon)
    return math.abs(a - b) <= (epsilon or 1e-5)
end

suite.it('ports rcCalcBounds and rcCalcGridSize semantics', function()
    local bmin, bmax = rc.calcBounds({vector.create(1, 2, 3), vector.create(0, 2, 6)})
    expect(bmin == vector.create(0, 2, 3), 'bad min bounds')
    expect(bmax == vector.create(1, 2, 6), 'bad max bounds')
    local w, h = rc.calcGridSize(bmin, bmax, 1.5)
    jt.expect(w).toBe(1)
    jt.expect(h).toBe(2)
end)

suite.it('ports rcAddSpan sorted insertion and overlap merging', function()
    local hf = rc.createHeightfield(4, 4, vector.zero, vector.create(4, 20, 4), 1, 2)
    expect(rc.addSpan(hf, 0, 0, 2, 3, 42, 1))
    expect(rc.addSpan(hf, 0, 0, 0, 1, 42, 1))
    expect(rc.addSpan(hf, 0, 0, 6, 7, 42, 1))
    local spans = rc.getSpans(hf, 0, 0)
    jt.expect(#spans).toBe(3)
    jt.expect(spans[1].smin).toBe(0)
    jt.expect(spans[2].smin).toBe(2)
    jt.expect(spans[3].smin).toBe(6)

    expect(rc.addSpan(hf, 0, 0, 1, 2, 24, 1))
    spans = rc.getSpans(hf, 0, 0)
    jt.expect(#spans).toBe(2)
    jt.expect(spans[1].smin).toBe(0)
    jt.expect(spans[1].smax).toBe(3)
    jt.expect(spans[1].area).toBe(42)
end)

suite.it('matches upstream simple rcRasterizeTriangle coverage', function()
    local hf = rc.createHeightfield(10, 10, vector.zero, vector.create(10, 10, 10), 1, 1)
    expect(rc.rasterizeTriangle(
        vector.create(0, 0, 0), vector.create(2, 0, 0), vector.create(0, 0, 2),
        rc.RC_WALKABLE_AREA, hf, 1
    ))
    for x = 0, 9 do
        for z = 0, 9 do
            local spans = rc.getSpans(hf, x, z)
            local expected = (x == 0 and z == 0) or (x == 0 and z == 1) or (x == 1 and z == 0)
            if expected then
                jt.expect(#spans).toBe(1)
                jt.expect(spans[1].smin).toBe(0)
                jt.expect(spans[1].smax).toBe(1)
                jt.expect(spans[1].area).toBe(rc.RC_WALKABLE_AREA)
            else
                jt.expect(#spans).toBe(0)
            end
        end
    end
end)

suite.it('clamps vertical raster spans like Recast', function()
    local hf = rc.createHeightfield(10, 10, vector.zero, vector.create(10, 10, 10), 1, 1)
    expect(rc.rasterizeTriangle(
        vector.create(0, -5, 0), vector.create(1, -5, 1), vector.create(0.5, 15, 0.5),
        rc.RC_WALKABLE_AREA, hf, 1
    ))
    local spans = rc.getSpans(hf, 0, 0)
    jt.expect(#spans).toBe(1)
    jt.expect(spans[1].smin).toBe(0)
    jt.expect(spans[1].smax).toBe(10)
end)

suite.it('marks walkable triangles by slope', function()
    local verts = {
        vector.create(0, 0, 0), vector.create(0, 0, 1), vector.create(1, 0, 0),
        vector.create(0, 0, 0), vector.create(0, 1, 0), vector.create(0, 0, 1),
    }
    local tris = {{1,2,3}, {4,5,6}}
    local areas = rc.markWalkableTriangles(45, verts, tris)
    jt.expect(buffer.readu8(areas, 0)).toBe(rc.RC_WALKABLE_AREA)
    jt.expect(buffer.readu8(areas, 1)).toBe(rc.RC_NULL_AREA)
end)

suite.it('builds compact span connectivity', function()
    local hf = rc.createHeightfield(2, 1, vector.zero, vector.create(2, 10, 1), 1, 1)
    rc.addSpan(hf, 0, 0, 0, 1, rc.RC_WALKABLE_AREA, 1)
    rc.addSpan(hf, 1, 0, 0, 1, rc.RC_WALKABLE_AREA, 1)
    local chf = rc.buildCompactHeightfield(2, 1, hf)
    jt.expect(chf.spanCount).toBe(2)
    local left = rc.getCompactSpan(chf, 0)
    local right = rc.getCompactSpan(chf, 1)
    jt.expect(left.connections[3]).toBe(0) -- +X, dir 2, stored as Lua index 3
    jt.expect(right.connections[1]).toBe(0) -- -X, dir 0
end)


suite.it('erodes compact walkable area by agent radius', function()
    local hf = rc.createHeightfield(5, 5, vector.zero, vector.create(5, 10, 5), 1, 1)
    for z = 0, 4 do
        for x = 0, 4 do rc.addSpan(hf, x, z, 0, 1, rc.RC_WALKABLE_AREA, 1) end
    end
    local chf = rc.buildCompactHeightfield(2, 1, hf)
    rc.erodeWalkableArea(1, chf)
    local walkable = 0
    for i = 0, chf.spanCount - 1 do
        if buffer.readu8(chf.areas, i) ~= rc.RC_NULL_AREA then walkable += 1 end
    end
    jt.expect(walkable).toBe(9)
end)

suite.it('labels disconnected compact components as separate regions', function()
    local hf = rc.createHeightfield(5, 1, vector.zero, vector.create(5, 10, 1), 1, 1)
    rc.addSpan(hf, 0, 0, 0, 1, rc.RC_WALKABLE_AREA, 1)
    rc.addSpan(hf, 1, 0, 0, 1, rc.RC_WALKABLE_AREA, 1)
    rc.addSpan(hf, 3, 0, 0, 1, rc.RC_WALKABLE_AREA, 1)
    rc.addSpan(hf, 4, 0, 0, 1, rc.RC_WALKABLE_AREA, 1)
    local chf = rc.buildCompactHeightfield(2, 1, hf)
    local regions, count = rc.buildConnectedRegions(chf)
    jt.expect(count).toBe(2)
    expect(buffer.readu16(regions, 0) ~= buffer.readu16(regions, 2 * 2), 'islands should differ')
end)


local function flatCompact(size)
    local hf = rc.createHeightfield(size, size, vector.zero, vector.create(size, 10, size), 1, 1)
    for z = 0, size - 1 do
        for x = 0, size - 1 do rc.addSpan(hf, x, z, 0, 1, rc.RC_WALKABLE_AREA, 1) end
    end
    return rc.buildCompactHeightfield(2, 1, hf)
end

suite.it('marks box, cylinder, and convex polygon areas in compact fields', function()
    local chf = flatCompact(5)
    rc.markBoxArea(vector.create(1, 0, 1), vector.create(1.9, 2, 1.9), 7, chf)
    local center = buffer.readu32(chf.cellIndex, (1 + 1 * chf.width) * 4)
    jt.expect(buffer.readu8(chf.areas, center)).toBe(7)

    rc.markCylinderArea(vector.create(3.5, 0, 1.5), 0.75, 2, 8, chf)
    local cyl = buffer.readu32(chf.cellIndex, (3 + 1 * chf.width) * 4)
    jt.expect(buffer.readu8(chf.areas, cyl)).toBe(8)

    rc.markConvexPolyArea({
        vector.create(1,0,3), vector.create(1,0,5), vector.create(3,0,5), vector.create(3,0,3),
    }, 0, 2, 9, chf)
    local poly = buffer.readu32(chf.cellIndex, (1 + 3 * chf.width) * 4)
    jt.expect(buffer.readu8(chf.areas, poly)).toBe(9)
end)

suite.it('median-filters compact area ids like Recast', function()
    local chf = flatCompact(3)
    local center = buffer.readu32(chf.cellIndex, (1 + chf.width) * 4)
    buffer.writeu8(chf.areas, center, 3)
    rc.medianFilterWalkableArea(chf)
    jt.expect(buffer.readu8(chf.areas, center)).toBe(rc.RC_WALKABLE_AREA)
end)

suite.it('offsets a convex polygon in Luau', function()
    local out, err = rc.offsetPoly({
        vector.create(0,0,0), vector.create(0,0,1), vector.create(1,0,1), vector.create(1,0,0),
    }, 0.25)
    expect(out ~= nil, err)
    jt.expect(#out).toBe(4)
    expect(approx(out[1].X, 0.25) and approx(out[1].Z, 0.25))
end)


suite.it('builds the Recast 16-bit distance field and blur', function()
    local chf = flatCompact(5)
    expect(rc.buildDistanceField(chf))
    jt.expect(chf.maxDistance).toBe(4)
    local center = buffer.readu32(chf.cellIndex, (2 + 2 * chf.width) * 4)
    jt.expect(buffer.readu16(chf.dist, center * 2)).toBe(2)
end)

suite.it('partitions compact fields with Recast watershed regions', function()
    local chf = flatCompact(9)
    rc.buildDistanceField(chf)
    local regions, maxRegions, overlaps = rc.buildRegions(chf, 0, 0, 0)
    expect(maxRegions >= 1)
    jt.expect(#overlaps).toBe(0)
    local first = buffer.readu16(regions, 0)
    expect(first ~= 0 and bit32.band(first, rc.RC_BORDER_REG) == 0)
    for i = 1, chf.spanCount - 1 do jt.expect(buffer.readu16(regions, i * 2)).toBe(first) end
end)

suite.it('partitions compact fields with Recast monotone regions and border ids', function()
    local chf = flatCompact(7)
    local regions = rc.buildRegionsMonotone(chf, 1, 0, 0)
    local edge = buffer.readu32(chf.cellIndex, 0)
    local center = buffer.readu32(chf.cellIndex, (3 + 3 * chf.width) * 4)
    expect(bit32.band(buffer.readu16(regions, edge * 2), rc.RC_BORDER_REG) ~= 0)
    expect(buffer.readu16(regions, center * 2) ~= 0)
    expect(bit32.band(buffer.readu16(regions, center * 2), rc.RC_BORDER_REG) == 0)
end)

suite.it('traces and simplifies a flat region contour', function()
    local chf = flatCompact(5)
    rc.buildConnectedRegions(chf)
    local cset = rc.buildContours(chf, 1.3, 0)
    jt.expect(cset.nconts).toBe(1)
    local cont = cset.conts[1]
    jt.expect(cont.nverts).toBe(4)
    jt.expect(cont.nrverts).toBe(20)
    jt.expect(cont.reg).toBe(1)
end)

suite.it('merges contour holes before polygonization', function()
    local hf = rc.createHeightfield(7, 7, vector.zero, vector.create(7, 10, 7), 1, 1)
    for z = 0, 6 do
        for x = 0, 6 do
            if not (x >= 2 and x <= 4 and z >= 2 and z <= 4) then
                rc.addSpan(hf, x, z, 0, 1, rc.RC_WALKABLE_AREA, 1)
            end
        end
    end
    local chf = rc.buildCompactHeightfield(2, 1, hf)
    rc.buildConnectedRegions(chf)
    local cset = rc.buildContours(chf, 0, 0)
    jt.expect(cset.nconts).toBe(2)
    local live = 0
    for _, cont in cset.conts do if cont.nverts >= 3 then live += 1 end end
    jt.expect(live).toBe(1)
    local mesh = rc.buildPolyMesh(cset, 6)
    jt.expect(mesh.nverts).toBe(8)
    jt.expect(mesh.npolys).toBe(4)
end)

suite.it('builds one convex quad from a rectangular contour', function()
    local chf = flatCompact(5)
    rc.buildConnectedRegions(chf)
    local mesh = rc.buildPolyMesh(rc.buildContours(chf, 1.3, 0), 6)
    jt.expect(mesh.nverts).toBe(4)
    jt.expect(mesh.npolys).toBe(1)
    jt.expect(buffer.readu16(mesh.polys, 0)).toBe(0)
    jt.expect(buffer.readu16(mesh.polys, 4 * 2)).toBe(rc.RC_MESH_NULL_IDX)
end)



local function squareContourSet(originX)
    return {
        conts = {{
            verts = {0,1,0,0, 0,1,2,0, 2,1,2,0, 2,1,0,0}, nverts = 4,
            rverts = {}, nrverts = 0, reg = 1, area = rc.RC_WALKABLE_AREA,
        }},
        nconts = 1, width = 2, height = 2, borderSize = 0, maxError = 1.3,
        bmin = vector.create(originX,0,0), bmax = vector.create(originX+2,2,2), cs = 1, ch = 1,
    }
end

suite.it('builds separate Recast heightfield layers for overlapping platforms', function()
    local hf = rc.createHeightfield(3, 3, vector.zero, vector.create(3, 20, 3), 1, 1)
    for z=0,2 do for x=0,2 do
        rc.addSpan(hf, x, z, 0, 1, rc.RC_WALKABLE_AREA, 1)
        rc.addSpan(hf, x, z, 10, 11, rc.RC_WALKABLE_AREA, 1)
    end end
    local chf = rc.buildCompactHeightfield(2, 1, hf)
    local layers = rc.buildHeightfieldLayers(chf, 0, 2)
    jt.expect(layers.nlayers).toBe(2)
    jt.expect(layers.layers[1].hmin).toBe(1)
    jt.expect(layers.layers[2].hmin).toBe(11)
    jt.expect(buffer.readu8(layers.layers[1].areas, 0)).toBe(rc.RC_WALKABLE_AREA)
    jt.expect(buffer.readu8(layers.layers[2].areas, 0)).toBe(rc.RC_WALKABLE_AREA)
end)

suite.it('builds Recast detail meshes and samples interior height error', function()
    local chf = flatCompact(7)
    rc.buildConnectedRegions(chf)
    local mesh = rc.buildPolyMesh(rc.buildContours(chf, 1.3, 0), 6)
    for z=2,4 do for x=2,4 do
        local span = buffer.readu32(chf.cellIndex, (x + z * chf.width) * 4)
        buffer.writeu16(chf.y, span * 2, 4)
    end end
    local detail = rc.buildPolyMeshDetail(mesh, chf, 1, 0.1)
    jt.expect(detail.nmeshes).toBe(mesh.npolys)
    expect(detail.nverts > mesh.nverts, 'height bump should force detail samples')
    expect(detail.ntris > 2, 'height bump should force detail tessellation')
end)

suite.it('merges Recast detail meshes with corrected base offsets', function()
    local chf = flatCompact(5)
    rc.buildConnectedRegions(chf)
    local mesh = rc.buildPolyMesh(rc.buildContours(chf, 1.3, 0), 6)
    local detail = rc.buildPolyMeshDetail(mesh, chf, 0, 0)
    local merged = rc.mergePolyMeshDetails({detail, detail})
    jt.expect(merged.nmeshes).toBe(detail.nmeshes * 2)
    jt.expect(merged.nverts).toBe(detail.nverts * 2)
    jt.expect(merged.ntris).toBe(detail.ntris * 2)
    jt.expect(buffer.readu32(merged.meshes, 16)).toBe(detail.nverts)
    jt.expect(buffer.readu32(merged.meshes, 24)).toBe(detail.ntris)
end)

suite.it('removes RC_BORDER_VERTEX exactly like rcBuildPolyMesh', function()
    local cset = {
        conts = {{
            verts = {
                0,0,0,0,
                0,0,2,0,
                2,0,2,0,
                2,0,0,0,
                1,0,0,rc.RC_BORDER_VERTEX,
            },
            nverts = 5,
            reg = 1,
            area = rc.RC_WALKABLE_AREA,
        }},
        nconts = 1,
        bmin = vector.zero,
        bmax = vector.create(2,1,2),
        cs = 1, ch = 1, width = 2, height = 2, borderSize = 0, maxError = 1.3,
    }
    local mesh = rc.buildPolyMesh(cset, 6)
    -- Differential fixture from upstream RecastNavigation 9f4ce644.
    jt.expect(mesh.nverts).toBe(4)
    jt.expect(mesh.npolys).toBe(1)
    jt.expect(mesh.maxpolys).toBe(3)
    for i0 = 0, 3 do jt.expect(buffer.readu16(mesh.polys, i0 * 2)).toBe(i0) end
    jt.expect(buffer.readu16(mesh.regs, 0)).toBe(1)
    jt.expect(buffer.readu8(mesh.areas, 0)).toBe(rc.RC_WALKABLE_AREA)
end)

suite.it('copies polymesh storage without aliasing source buffers', function()
    local src = rc.buildPolyMesh(squareContourSet(0), 6)
    buffer.writeu16(src.flags, 0, 37)
    local copy = rc.copyPolyMesh(src)
    jt.expect(copy.nverts).toBe(src.nverts)
    jt.expect(copy.npolys).toBe(src.npolys)
    jt.expect(buffer.readu16(copy.flags, 0)).toBe(37)
    buffer.writeu16(copy.flags, 0, 99)
    jt.expect(buffer.readu16(src.flags, 0)).toBe(37)
end)

suite.it('merges adjacent Recast polymesh tiles and welds their seam', function()
    local a = rc.buildPolyMesh(squareContourSet(0), 6)
    local b = rc.buildPolyMesh(squareContourSet(2), 6)
    local merged = rc.mergePolyMeshes({a,b})
    jt.expect(merged.nverts).toBe(6)
    jt.expect(merged.npolys).toBe(2)
    local connected = false
    for e0=0,merged.nvp-1 do
        if buffer.readu16(merged.polys,(merged.nvp+e0)*2) == 1 then connected = true end
    end
    expect(connected, 'merged tile seam should become polygon adjacency')
end)



suite.it('partitions overlapping platforms with Recast layer regions', function()
    local hf = rc.createHeightfield(1, 1, vector.zero, vector.create(1, 20, 1), 1, 1)
    rc.addSpan(hf, 0, 0, 0, 1, rc.RC_WALKABLE_AREA, 1)
    rc.addSpan(hf, 0, 0, 10, 11, rc.RC_WALKABLE_AREA, 1)
    local chf = rc.buildCompactHeightfield(2, 1, hf)
    local regions, count = rc.buildLayerRegions(chf, 0, 0)
    jt.expect(count).toBe(3) -- upstream reports the next unused monotone id here
    local a = buffer.readu16(regions, 0)
    local b = buffer.readu16(regions, 2)
    expect(a ~= 0 and b ~= 0 and a ~= b, 'overlapping floors must remain distinct layers')
end)

suite.it('accepts layers as a high-level partition type', function()
    local verts = {vector.create(0,0,0), vector.create(0,0,2), vector.create(2,0,0), vector.create(2,0,2)}
    local tris = {{1,2,3},{3,2,4}}
    local build = rc.buildCompactFromTriangles(verts, tris, {cs=1,ch=0.5,walkableHeight=2,walkableClimb=1,walkableRadius=0,partition='layers'})
    jt.expect(build.partitionType).toBe('layers')
    expect(build.compact.maxRegions >= 1)
end)



suite.it('builds Recast polymesh detail for a flat polygon', function()
    local chf = flatCompact(5)
    rc.buildConnectedRegions(chf)
    local mesh = rc.buildPolyMesh(rc.buildContours(chf, 1.3, 0), 6)
    local detail = rc.buildPolyMeshDetail(mesh, chf, 1, 0.1)
    jt.expect(detail.nmeshes).toBe(mesh.npolys)
    expect(detail.nverts >= 4)
    expect(detail.ntris >= 2)
    jt.expect(buffer.readu32(detail.meshes, 4)).toBe(detail.nverts)
end)

suite.it('merges Recast polymesh details with corrected submesh offsets', function()
    local chf = flatCompact(3)
    rc.buildConnectedRegions(chf)
    local mesh = rc.buildPolyMesh(rc.buildContours(chf, 1.3, 0), 6)
    local a = rc.buildPolyMeshDetail(mesh, chf, 0, 0)
    local merged = rc.mergePolyMeshDetails({a,a})
    jt.expect(merged.nmeshes).toBe(a.nmeshes * 2)
    jt.expect(merged.nverts).toBe(a.nverts * 2)
    jt.expect(merged.ntris).toBe(a.ntris * 2)
    jt.expect(buffer.readu32(merged.meshes, a.nmeshes * 16)).toBe(a.nverts)
    jt.expect(buffer.readu32(merged.meshes, a.nmeshes * 16 + 8)).toBe(a.ntris)
end)

return suite
