--!native

local jt = require('@jess/test')
local suite = jt.new()
local dt = require('@recast/detour')

local function expect(condition, message)
    if not condition then error(message or 'expectation failed', 2) end
end

local function plane()
    -- Four triangles in a 2x1 strip, all wound for +Y normals.
    local v = {
        vector.create(0,0,0), vector.create(0,0,1), vector.create(1,0,0),
        vector.create(1,0,1), vector.create(2,0,0), vector.create(2,0,1),
    }
    local t = {
        {1,2,3}, {3,2,4}, {3,4,5}, {5,4,6},
    }
    return dt.buildTriangleNavMesh(v, t, {walkableSlopeAngle = 45})
end

suite.it('builds triangle adjacency without a native Detour mesh', function()
    local nav = plane()
    jt.expect(nav.polyCount).toBe(4)
    jt.expect(nav.walkableCount).toBe(4)
    local n2 = dt.neighbors(nav, 2)
    expect(#n2 >= 2, 'middle triangle should have at least two neighbors')
end)

suite.it('findNearestPoly clamps to triangle surfaces', function()
    local nav = plane()
    local ref, point, dist = dt.findNearestPoly(nav, vector.create(0.25, 3, 0.25))
    expect(ref ~= nil)
    expect(point ~= nil)
    expect(math.abs(point.Y) < 1e-5)
    expect(dist > 8.9 and dist < 9.1)
end)

suite.it('A* crosses a connected triangle corridor', function()
    local nav = plane()
    local path = dt.findPath(nav, 1, 4)
    expect(path ~= nil)
    jt.expect(path[1]).toBe(1)
    jt.expect(path[#path]).toBe(4)
    expect(#path >= 3 and #path <= 4)
end)

suite.it('funnel query returns a straight path across a flat strip', function()
    local nav = plane()
    local result, err = dt.query(nav, vector.create(0.1, 0, 0.5), vector.create(1.9, 0, 0.5))
    expect(result ~= nil, err)
    jt.expect(result.startRef ~= nil).toBeTruthy()
    jt.expect(result.endRef ~= nil).toBeTruthy()
    expect(#result.corridor >= 3)
    expect(#result.points >= 2)
    local first = result.points[1]
    local last = result.points[#result.points]
    expect(math.abs(first.X - 0.1) < 1e-4)
    expect(math.abs(last.X - 1.9) < 1e-4)
end)

suite.it('reports no path across disconnected islands', function()
    local v = {
        vector.create(0,0,0), vector.create(0,0,1), vector.create(1,0,0),
        vector.create(10,0,0), vector.create(10,0,1), vector.create(11,0,0),
    }
    local nav = dt.buildTriangleNavMesh(v, {{1,2,3},{4,5,6}})
    local p = dt.findPath(nav, 1, 2)
    jt.expect(p).toBeNil()
end)


suite.it('runs triangle soup through the Luau voxel pipeline and queries the generated navmesh', function()
    local size = 6
    local verts = {}
    for z = 0, size do
        for x = 0, size do
            verts[#verts + 1] = vector.create(x, 0, z)
        end
    end
    local function vi(x, z) return z * (size + 1) + x + 1 end
    local tris = {}
    for z = 0, size - 1 do
        for x = 0, size - 1 do
            local a, b = vi(x, z), vi(x, z + 1)
            local c, d = vi(x + 1, z), vi(x + 1, z + 1)
            tris[#tris + 1] = {a, b, c}
            tris[#tris + 1] = {c, b, d}
        end
    end
    local build = dt.buildFromTriangles(verts, tris, {
        cs = 1,
        ch = 0.5,
        walkableSlopeAngle = 45,
        walkableHeight = 2,
        walkableClimb = 1,
        walkableRadius = 0,
    })
    expect(build.compact.spanCount > 0)
    expect(build.navmesh.polyCount > 0)
    local result, err = dt.query(build.navmesh, vector.create(1.5, 0, 1.5), vector.create(4.5, 0, 4.5))
    expect(result ~= nil, err)
    expect(#result.corridor >= 1)
    expect(build.polyMesh ~= nil and build.polyMesh.npolys == 1, 'flat field should collapse to one Recast polygon')
    jt.expect(build.navmesh.polyCount).toBe(1)
    expect(#result.points >= 2)
end)


suite.it('supports Detour-style include/exclude filters and area costs', function()
    local nav = plane()
    local filter = dt.newQueryFilter({includeFlags = 0x2})
    jt.expect(dt.findPath(nav, 1, 4, filter)).toBeNil()
    for ref = 1, nav.polyCount do dt.setPolyFlags(nav, ref, 0x2) end
    local path = dt.findPath(nav, 1, 4, filter)
    expect(path ~= nil)

    dt.setAreaCost(filter, 7, 3.5)
    expect(math.abs(dt.getAreaCost(filter, 7) - 3.5) < 1e-5)
    expect(dt.setPolyArea(nav, 2, 7))
    jt.expect(dt.getPolyArea(nav, 2)).toBe(7)
end)


suite.it('exposes dtQueryFilter accessors and default filter cost semantics', function()
    local nav = plane()
    local filter = dt.newQueryFilter()
    jt.expect(dt.getIncludeFlags(filter)).toBe(0xffff)
    jt.expect(dt.getExcludeFlags(filter)).toBe(0)
    dt.setIncludeFlags(filter, 0x12345); dt.setExcludeFlags(filter, 0x20002)
    jt.expect(dt.getIncludeFlags(filter)).toBe(0x2345)
    jt.expect(dt.getExcludeFlags(filter)).toBe(0x0002)
    dt.setPolyFlags(nav, 1, 0x0001)
    expect(dt.passFilter(nav, 1, filter))
    dt.setExcludeFlags(filter, 0x0001)
    expect(not dt.passFilter(nav, 1, filter))
    dt.setExcludeFlags(filter, 0); dt.setIncludeFlags(filter, 0xffff)
    dt.setPolyArea(nav, 1, 5); dt.setAreaCost(filter, 5, 2.5)
    local a,b=vector.create(0,0,0),vector.create(3,4,0)
    expect(math.abs(dt.getCost(nav, filter, a, b, 1)-12.5)<1e-5)
    jt.expect(dt.getAttachedNavMesh(nav)).toBe(nav)
end)


suite.it('routes around a Recast contour hole instead of filling it', function()
    local size = 9
    local verts = {}
    for z = 0, size do for x = 0, size do verts[#verts + 1] = vector.create(x, 0, z) end end
    local function vi(x, z) return z * (size + 1) + x + 1 end
    local tris = {}
    for z = 0, size - 1 do
        for x = 0, size - 1 do
            local a, b, c, d = vi(x,z), vi(x,z+1), vi(x+1,z), vi(x+1,z+1)
            tris[#tris + 1] = {a,b,c}; tris[#tris + 1] = {c,b,d}
        end
    end
    local build = dt.buildFromTriangles(verts, tris, {
        cs = 1, ch = 0.5, walkableHeight = 2, walkableClimb = 1, walkableRadius = 0,
        maxSimplificationError = 0,
        areaVolumes = {{kind = 'box', bmin = vector.create(3,0,3), bmax = vector.create(6,2,6), area = 0}},
    })
    jt.expect(build.polyMesh.npolys).toBe(4)
    local result, err = dt.query(build.navmesh, vector.create(1,0,4.5), vector.create(8,0,4.5))
    expect(result ~= nil, err)
    expect(#result.points >= 3, 'path should bend around the hole')
    for _, p in result.points do
        expect(not (p.X > 3 and p.X < 6 and p.Z > 3 and p.Z < 6), 'straight path crossed carved hole')
    end
end)


suite.it('builds Recast detail data in the standard triangle pipeline', function()
    local verts = {vector.create(0,0,0), vector.create(0,0,6), vector.create(6,0,0), vector.create(6,0,6)}
    local tris = {{1,2,3},{3,2,4}}
    local build = dt.buildFromTriangles(verts, tris, {cs=1,ch=0.5,walkableHeight=2,walkableClimb=1,walkableRadius=0})
    expect(build.detailMesh ~= nil, 'standard Recast build should produce rcPolyMeshDetail data')
    jt.expect(build.detailMesh.nmeshes).toBe(build.polyMesh.npolys)
    jt.expect(build.navmesh.detailMesh).toBe(build.detailMesh)

    local lean = dt.buildFromTriangles(verts, tris, {cs=1,ch=0.5,walkableHeight=2,walkableClimb=1,walkableRadius=0,buildDetailMesh=false})
    jt.expect(lean.detailMesh).toBe(nil)
end)


suite.it('supports Detour boundary, height, overlap, portal, and wall queries', function()
    local nav = plane()
    local inside = vector.create(0.25, 7, 0.25)
    jt.expect(dt.closestPointOnPolyBoundary(nav, 1, inside)).toBe(inside)
    local outside = dt.closestPointOnPolyBoundary(nav, 1, vector.create(-2, 0, 0.25))
    expect(outside ~= nil and math.abs(outside.X) < 1e-5)
    local h = dt.getPolyHeight(nav, 1, vector.create(0.25, 99, 0.25))
    expect(h ~= nil and math.abs(h) < 1e-5)
    expect(dt.isValidPolyRef(nav, 1, nil))
    expect(not dt.isValidPolyRef(nav, 999, nil))

    local refs = dt.queryPolygons(nav, vector.create(1,0,0.5), vector.create(2,1,1), nil)
    jt.expect(#refs).toBe(4)
    local left, right = dt.getPortalPoints(nav, 1, 2)
    expect(left ~= nil and right ~= nil)
    local walls = dt.getPolyWallSegments(nav, 1, nil, false)
    jt.expect(#walls).toBe(2)
    local allEdges = dt.getPolyWallSegments(nav, 1, nil, true)
    jt.expect(#allEdges).toBe(3)
end)


suite.it('raycasts across portals and stops on filtered or outer walls', function()
    local nav = plane()
    local clear = dt.raycast(nav, 1, vector.create(0.1,0,0.2), vector.create(1.9,0,0.8))
    expect(clear ~= nil)
    jt.expect(clear.t).toBe(math.huge)
    expect(#clear.path >= 2)

    local hit = dt.raycast(nav, 1, vector.create(0.2,0,0.2), vector.create(-1,0,0.2))
    expect(hit ~= nil and hit.t > 0 and hit.t < 1)
    expect(math.abs(hit.hitPos.X) < 1e-4)
    expect(hit.hitNormal.X > 0.9)

    local blocked = dt.newQueryFilter({excludeFlags = 0x2})
    dt.setPolyFlags(nav, 2, 0x2)
    local filtered = dt.raycast(nav, 1, vector.create(0.2,0,0.2), vector.create(0.8,0,0.8), blocked)
    expect(filtered ~= nil and filtered.t < 1)
end)

suite.it('finds distance and normal to the nearest reachable wall', function()
    local nav = plane()
    local dist, pos, normal = dt.findDistanceToWall(nav, 1, vector.create(0.25,0,0.25), 10, nil)
    expect(dist ~= nil and math.abs(dist - 0.25) < 1e-4)
    expect(pos ~= nil and normal ~= nil)
    expect(normal.X > 0.9 or normal.Z > 0.9)
    local capped, noPos = dt.findDistanceToWall(nav, 1, vector.create(0.25,0,0.25), 0.1, nil)
    expect(math.abs(capped - 0.1) < 1e-5 and noPos == nil)
end)


suite.it('runs Detour-style Dijkstra searches around circles and convex shapes', function()
    local nav = plane()
    local circle = dt.findPolysAroundCircle(nav, 1, vector.create(0.25,0,0.25), 3, nil)
    expect(circle ~= nil)
    jt.expect(#circle.refs).toBe(4)
    jt.expect(circle.refs[1]).toBe(1)
    for i = 2, #circle.costs do expect(circle.costs[i] >= circle.costs[i-1]) end
    local path = dt.getPathFromDijkstraSearch(nav, 4)
    expect(path ~= nil and path[1] == 1 and path[#path] == 4)

    local shape = {
        vector.create(-0.1,0,-0.1), vector.create(-0.1,0,1.1),
        vector.create(2.1,0,1.1), vector.create(2.1,0,-0.1),
    }
    local around = dt.findPolysAroundShape(nav, 1, shape, nil)
    expect(around ~= nil and #around.refs == 4)
    local refs, parents = dt.findLocalNeighbourhood(nav, 1, vector.create(1,0,0.5), 3, nil)
    expect(refs ~= nil and parents ~= nil and #refs == 4)
end)


suite.it('incrementally advances and finalizes sliced Detour A* queries', function()
    local nav = plane()
    local status = dt.initSlicedFindPath(nav, 1, 4, vector.create(0.1,0,0.2), vector.create(1.9,0,0.8), nil, 0)
    expect(dt.statusInProgress(status))
    local totalIters = 0
    while dt.statusInProgress(status) do
        local done
        status, done = dt.updateSlicedFindPath(nav, 1)
        totalIters += done
        expect(totalIters < 16)
    end
    expect(dt.statusSucceed(status))
    local path, finalStatus = dt.finalizeSlicedFindPath(nav)
    expect(path ~= nil and path[1] == 1 and path[#path] == 4)
    expect(dt.statusSucceed(finalStatus) and not dt.statusDetail(finalStatus, dt.DT_PARTIAL_RESULT))
end)

suite.it('finalizes a sliced query against the furthest existing visited polygon', function()
    local nav = plane()
    dt.initSlicedFindPath(nav, 1, 4, vector.create(0.1,0,0.2), vector.create(1.9,0,0.8), nil, 0)
    dt.updateSlicedFindPath(nav, 1)
    local path, status = dt.finalizeSlicedFindPathPartial(nav, {1,2,4})
    expect(path ~= nil and path[1] == 1)
    expect(dt.statusSucceed(status))
end)


suite.it('moves locally along the navmesh and reports visited corridor polygons', function()
    local nav = plane()
    local result, visited = dt.moveAlongSurface(nav, 1, vector.create(0.1,0,0.2), vector.create(1.9,0,0.8), nil)
    expect(result ~= nil and visited ~= nil)
    expect(math.abs(result.X - 1.9) < 1e-5 and #visited >= 2)
    local clamped, wallPath = dt.moveAlongSurface(nav, 1, vector.create(0.2,0,0.2), vector.create(-2,0,0.2), nil)
    expect(clamped ~= nil and math.abs(clamped.X) < 1e-5)
    expect(wallPath ~= nil and wallPath[1] == 1)
end)



suite.it('raycasts across portals and reports filtered walls', function()
    local nav = plane()
    local hit, err = dt.raycast(nav, 1, vector.create(0.1,0,0.2), vector.create(1.9,0,0.8), nil, dt.DT_RAYCAST_USE_COSTS)
    expect(hit ~= nil, err)
    expect(hit.t == math.huge, 'unobstructed ray should reach its end')
    expect(#hit.path >= 2)
    expect(hit.pathCost > 1.7 and hit.pathCost < 2.1)

    local filter = dt.newQueryFilter({includeFlags=0x1, excludeFlags=0x2})
    dt.setPolyFlags(nav, 3, 0x2)
    dt.setPolyFlags(nav, 4, 0x2)
    local blocked = dt.raycast(nav, 1, vector.create(0.1,0,0.2), vector.create(1.9,0,0.8), filter)
    expect(blocked ~= nil and blocked.t > 0 and blocked.t < 1, 'filtered neighbour should become a wall')
    expect(blocked.hitNormal ~= vector.zero)
end)

suite.it('runs Detour-style Dijkstra circle and wall-distance searches', function()
    local nav = plane()
    local around, err = dt.findPolysAroundCircle(nav, 1, vector.create(0.1,0,0.2), 4, nil)
    expect(around ~= nil, err)
    jt.expect(#around.refs).toBe(4)
    jt.expect(around.refs[1]).toBe(1)
    jt.expect(around.parents[1]).toBe(0)
    for i = 2, #around.costs do expect(around.costs[i] >= around.costs[i-1]) end

    local distance, position, normal = dt.findDistanceToWall(nav, 2, vector.create(0.75,0,0.5), 10, nil)
    expect(position ~= nil and normal ~= nil)
    expect(math.abs(distance - 0.5) < 1e-5)
    expect(math.abs(normal.Z) > 0.99)
end)

suite.it('matches upstream convex random-point helper edge cases', function()
    local verts = {vector.create(0,0,0), vector.create(0,0,1), vector.create(1,0,0)}
    local a = dt.randomPointInConvexPoly(verts, 0.0, 1.0)
    local b = dt.randomPointInConvexPoly(verts, 0.5, 1.0)
    local c = dt.randomPointInConvexPoly(verts, 1.0, 1.0)
    expect(math.abs(a.X) < 1e-6 and math.abs(a.Z - 1) < 1e-6)
    expect(math.abs(b.X - 0.5) < 1e-6 and math.abs(b.Z - 0.5) < 1e-6)
    expect(math.abs(c.X - 1) < 1e-6 and math.abs(c.Z) < 1e-6)
end)

suite.it('finds weighted random navmesh points and circle-local points', function()
    local nav = plane()
    local seq = {0.2,0.8,0.4,0.25,0.75,0.1,0.6,0.3,0.7,0.2}
    local at = 0
    local function rnd() at += 1; return seq[((at - 1) % #seq) + 1] end
    local ref, point = dt.findRandomPoint(nav, nil, rnd)
    expect(ref ~= nil and point ~= nil and dt.isValidPolyRef(nav, ref, nil))
    expect(point.X >= -1e-5 and point.X <= 2.00001 and point.Z >= -1e-5 and point.Z <= 1.00001)
    local localRef, localPoint = dt.findRandomPointAroundCircle(nav, 1, vector.create(0.1,0,0.1), 0.01, nil, rnd)
    jt.expect(localRef).toBe(1)
    expect(localPoint ~= nil)
end)

suite.it('exposes portal midpoints and query closed-list membership', function()
    local nav = plane()
    local mid = dt.getEdgeMidPoint(nav, 1, 2)
    expect(mid ~= nil)
    dt.findPolysAroundCircle(nav, 1, vector.create(0.25,0,0.25), 3, nil)
    expect(dt.isInClosedList(nav, 1))
    expect(dt.isInClosedList(nav, 4))
    expect(not dt.isInClosedList(nav, 999))
end)



suite.it('keeps Detour area zero walkable after runtime area edits', function()
    local nav = plane()
    expect(dt.setPolyArea(nav, 2, 0))
    jt.expect(dt.getPolyArea(nav, 2)).toBe(0)
    expect(dt.isValidPolyRef(nav, 2, nil), 'Detour area id 0 is valid and must not inherit RC_NULL_AREA semantics')
    local path = dt.findPath(nav, 1, 4, nil)
    expect(path ~= nil and path[#path] == 4)
end)

suite.it('traverses Detour off-mesh polygons with directional endpoint semantics', function()
    local verts = {
        vector.create(0,0,0),vector.create(0,0,1),vector.create(1,0,0),vector.create(1,0,1),
        vector.create(4,1,0),vector.create(4,1,1),vector.create(5,1,0),vector.create(5,1,1),
    }
    local nav = dt.buildTriangleNavMesh(verts, {{1,2,3},{3,2,4},{5,6,7},{7,6,8}})
    dt.addOffMeshConnections(nav, {{
        start=vector.create(0.8,0,0.5), ['end']=vector.create(4.2,1,0.5), radius=0.6,
        bidirectional=false, area=0, flags=1, userId=77,
    }}, {walkableClimb=2})
    jt.expect(nav.polyCount).toBe(5)
    local offRef = 5
    jt.expect(dt.getPolyType(nav, offRef)).toBe(dt.DT_POLYTYPE_OFFMESH_CONNECTION)
    local con = dt.getOffMeshConnectionByRef(nav, offRef)
    expect(con ~= nil and con.userId == 77 and con.startRef > 0 and con.endRef > 0 and not con.bidirectional)
    expect(dt.isValidPolyRef(nav, offRef, nil), 'area zero off-mesh poly must stay valid')

    local forward = dt.findPath(nav, 1, 4, nil)
    expect(forward ~= nil, 'one-way off-mesh connection should bridge the islands forward')
    local seenOff=false
    for _,ref in forward do if ref==offRef then seenOff=true end end
    expect(seenOff)
    jt.expect(dt.findPath(nav, 4, 1, nil)).toBeNil()

    local a,b = dt.getOffMeshConnectionPolyEndPoints(nav, con.startRef, offRef)
    expect(a ~= nil and b ~= nil and a.X < b.X)
    local rb,ra = dt.getOffMeshConnectionPolyEndPoints(nav, con.endRef, offRef)
    expect(ra ~= nil and rb ~= nil and rb.X > ra.X, 'endpoint order must reverse when entered from the far polygon')

    local points,flags,refs = dt.findStraightPath(nav, vector.create(0.2,0,0.5), vector.create(4.8,1,0.5), forward)
    expect(points ~= nil and #points == #flags and #flags == #refs)
    local marked=false
    for i=1,#flags do
        if bit32.band(flags[i],dt.DT_STRAIGHTPATH_OFFMESH_CONNECTION)~=0 then marked=true; jt.expect(refs[i]).toBe(offRef) end
    end
    expect(marked, 'straight path must expose the off-mesh transition flag/ref')
end)

suite.it('allows reverse traversal for bidirectional off-mesh connections', function()
    local verts = {
        vector.create(0,0,0),vector.create(0,0,1),vector.create(1,0,0),vector.create(1,0,1),
        vector.create(4,0,0),vector.create(4,0,1),vector.create(5,0,0),vector.create(5,0,1),
    }
    local nav = dt.buildTriangleNavMesh(verts, {{1,2,3},{3,2,4},{5,6,7},{7,6,8}})
    dt.addOffMeshConnections(nav, {{start=vector.create(0.8,0,0.5),finish=vector.create(4.2,0,0.5),radius=0.6,bidirectional=true}})
    local reverse=dt.findPath(nav,4,1,nil)
    expect(reverse~=nil)
    local seen=false; for _,ref in reverse do if ref==5 then seen=true end end
    expect(seen)
end)


suite.it('advances sliced and Dijkstra searches through off-mesh links', function()
    local verts={
        vector.create(0,0,0),vector.create(0,0,1),vector.create(1,0,0),vector.create(1,0,1),
        vector.create(4,0,0),vector.create(4,0,1),vector.create(5,0,0),vector.create(5,0,1),
    }
    local nav=dt.buildTriangleNavMesh(verts,{{1,2,3},{3,2,4},{5,6,7},{7,6,8}})
    dt.addOffMeshConnections(nav,{{start=vector.create(0.8,0,0.5),finish=vector.create(4.2,0,0.5),radius=0.6,bidirectional=true}})
    local status=dt.initSlicedFindPath(nav,1,4,vector.create(0.2,0,0.5),vector.create(4.8,0,0.5),nil,0)
    local guard=0
    while dt.statusInProgress(status) do status=select(1,dt.updateSlicedFindPath(nav,1)); guard+=1; expect(guard<32) end
    local path=select(1,dt.finalizeSlicedFindPath(nav)); expect(path~=nil)
    local seen=false; for _,ref in path do if ref==5 then seen=true end end; expect(seen)
    local around=dt.findPolysAroundCircle(nav,1,vector.create(2.5,0,0.5),10,nil)
    expect(around~=nil)
    local found=false; for _,ref in around.refs do if ref==4 then found=true end end; expect(found,'Dijkstra circle query should traverse an off-mesh link')
end)


suite.it('routes encoded refs across separately built Detour tiles', function()
    local function tile(x0)
        local v={
            vector.create(x0,0,0),vector.create(x0,0,1),
            vector.create(x0+1,0,0),vector.create(x0+1,0,1),
        }
        local n=dt.buildTriangleNavMesh(v,{{1,2,3},{3,2,4}})
        n.cellSize=0.25
        return n
    end
    local nav=dt.newTiledNavMesh({orig=vector.zero,tileWidth=1,tileHeight=1,maxTiles=4,maxPolys=8})
    local refA,statusA=dt.addTile(nav,tile(0),0,0,0)
    local refB,statusB=dt.addTile(nav,tile(1),1,0,0)
    expect(dt.statusSucceed(statusA) and dt.statusSucceed(statusB))
    local sa,ta,pa=dt.decodePolyId(nav,refA); local sb,tb,pb=dt.decodePolyId(nav,refB)
    expect(sa==1 and sb==1 and ta~=tb and pa==0 and pb==0)
    jt.expect(dt.getTileRefAt(nav,0,0,0)).toBe(refA); jt.expect(dt.getTileRefAt(nav,1,0,0)).toBe(refB)

    local startRef,startPos=dt.findNearestPoly(nav,vector.create(0.1,0,0.5),vector.create(0.25,1,0.25),nil)
    local endRef,endPos=dt.findNearestPoly(nav,vector.create(1.9,0,0.5),vector.create(0.25,1,0.25),nil)
    expect(startRef~=nil and endRef~=nil and startPos~=nil and endPos~=nil)
    jt.expect(dt.decodePolyIdTile(nav,startRef)).toBe(ta); jt.expect(dt.decodePolyIdTile(nav,endRef)).toBe(tb)
    local path=dt.findPath(nav,startRef,endRef,nil); expect(path~=nil and #path>=3)
    jt.expect(path[1]).toBe(startRef); jt.expect(path[#path]).toBe(endRef)
    local crossed=false
    for i=1,#path-1 do if dt.decodePolyIdTile(nav,path[i])~=dt.decodePolyIdTile(nav,path[i+1]) then
        local l,r=dt.getPortalPoints(nav,path[i],path[i+1]); expect(l~=nil and r~=nil); crossed=true
    end end
    expect(crossed,'corridor must cross an external tile link')
    local points=dt.findStraightPath(nav,startPos,endPos,path); expect(points~=nil and #points>=2)
    local ray=dt.raycast(nav,startRef,startPos,endPos,nil); expect(ray~=nil and ray.t==math.huge,'raycast should follow the external tile edge')
    local around=dt.findPolysAroundCircle(nav,startRef,startPos,4,nil); expect(around~=nil)
    local reached=false; for _,ref in around.refs do if dt.decodePolyIdTile(nav,ref)==tb then reached=true end end
    expect(reached,'Dijkstra queries must preserve encoded refs across tile links')
end)

suite.it('invalidates encoded query refs when a tiled slot salt changes', function()
    local t=dt.buildTriangleNavMesh({vector.create(0,0,0),vector.create(0,0,1),vector.create(1,0,0)},{{1,2,3}}); t.cellSize=0.25
    local nav=dt.newTiledNavMesh({maxTiles=2,maxPolys=4,tileWidth=1,tileHeight=1})
    local tileRef=assert(dt.addTile(nav,t,0,0,0)); local polyRef=tileRef
    expect(dt.isValidPolyRef(nav,polyRef,nil))
    local removed,status=dt.removeTile(nav,tileRef); expect(removed~=nil and dt.statusSucceed(status)); expect(not dt.isValidPolyRef(nav,polyRef,nil))
    local newRef=assert(dt.addTile(nav,t,0,0,0)); expect(newRef~=tileRef); expect(dt.isValidPolyRef(nav,newRef,nil))
end)


suite.it('supports Detour position-aware findPath status details and public overload', function()
    local nav=plane()
    local sp=vector.create(.1,0,.1);local ep=vector.create(1.9,0,.9)
    local path,status=dt.findPath(nav,1,4,sp,ep,nil)
    expect(path~=nil and dt.statusSucceed(status));jt.expect(path[1]).toBe(1);jt.expect(path[#path]).toBe(4)

    local clipped,clipStatus=dt.findPath(nav,1,4,sp,ep,nil,2)
    jt.expect(#clipped).toBe(2);expect(dt.statusDetail(clipStatus,dt.DT_BUFFER_TOO_SMALL),'expected DT_BUFFER_TOO_SMALL')

    local starved,starvedStatus=dt.findPath(nav,1,4,sp,ep,nil,nil,1)
    jt.expect(#starved).toBe(1);expect(dt.statusDetail(starvedStatus,dt.DT_OUT_OF_NODES),'expected DT_OUT_OF_NODES')
    expect(dt.statusDetail(starvedStatus,dt.DT_PARTIAL_RESULT),'node starvation must be partial')
end)


suite.it('matches upstream sliced any-angle parent detachment and finalized corridor', function()
    local N,nvp=4,4
    local nverts=(N+1)*(N+1); local np=N*N
    local verts=buffer.create(nverts*6)
    local function vi(x,z) return z*(N+1)+x end
    for z=0,N do for x=0,N do
        local o=vi(x,z)*6
        buffer.writeu16(verts,o,x); buffer.writeu16(verts,o+2,0); buffer.writeu16(verts,o+4,z)
    end end
    local polys=buffer.create(np*nvp*4); buffer.fill(polys,0,0xff,buffer.len(polys))
    local areas=buffer.create(np); buffer.fill(areas,0,1,np)
    local flags=buffer.create(np*2); for i=0,np-1 do buffer.writeu16(flags,i*2,1) end
    local function pi(x,z) return z*N+x end
    for z=0,N-1 do for x=0,N-1 do
        local p0=pi(x,z); local b=p0*nvp*2
        buffer.writeu16(polys,(b+0)*2,vi(x,z)); buffer.writeu16(polys,(b+1)*2,vi(x+1,z))
        buffer.writeu16(polys,(b+2)*2,vi(x+1,z+1)); buffer.writeu16(polys,(b+3)*2,vi(x,z+1))
        if z>0 then buffer.writeu16(polys,(b+nvp+0)*2,pi(x,z-1)) end
        if x<N-1 then buffer.writeu16(polys,(b+nvp+1)*2,pi(x+1,z)) end
        if z<N-1 then buffer.writeu16(polys,(b+nvp+2)*2,pi(x,z+1)) end
        if x>0 then buffer.writeu16(polys,(b+nvp+3)*2,pi(x-1,z)) end
    end end
    local nav=dt.buildPolyMeshNavMesh({verts=verts,nverts=nverts,polys=polys,areas=areas,flags=flags,npolys=np,nvp=nvp,bmin=vector.zero,bmax=vector.create(N,1,N),cs=1,ch=1})
    nav.walkableRadius=10
    local sp,ep=vector.create(.2,0,.2),vector.create(3.8,0,3.8)
    local sr=assert(dt.findNearestPoly(nav,sp,vector.create(.5,2,.5)))
    local er=assert(dt.findNearestPoly(nav,ep,vector.create(.5,2,.5)))
    local status=dt.initSlicedFindPath(nav,sr,er,sp,ep,nil,dt.DT_FINDPATH_ANY_ANGLE)
    while dt.statusInProgress(status) do status=select(1,dt.updateSlicedFindPath(nav,2)) end
    local detached=0
    for _,v in nav._slicedQuery.detached do if v then detached+=1 end end
    expect(detached>0,'any-angle search should detach at least one parent on the oracle grid')
    local path,finalStatus=dt.finalizeSlicedFindPath(nav)
    expect(dt.statusSucceed(finalStatus))
    local expected={1,2,6,7,11,12,16} -- upstream refs 16,17,21,22,26,27,31 rebased to this one-tile Luau mesh.
    jt.expect(#path).toBe(#expected)
    for i=1,#expected do jt.expect(path[i]).toBe(expected[i]) end
end)

return suite
