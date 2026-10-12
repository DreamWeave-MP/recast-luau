--!native
local jt=require('@jess/test')
local suite=jt.new()
local dt=require('@recast/detour')
local pc=require('@recast/path-corridor')
local function expect(v,m) if not v then error(m or 'expectation failed',2) end end
local function plane()
    local v={vector.create(0,0,0),vector.create(0,0,1),vector.create(1,0,0),vector.create(1,0,1),vector.create(2,0,0),vector.create(2,0,1),vector.create(3,0,0),vector.create(3,0,1)}
    return dt.buildTriangleNavMesh(v,{{1,2,3},{3,2,4},{3,4,5},{5,4,6},{5,6,7},{7,6,8}})
end
suite.it('merges moved and shortcut corridor paths like DetourPathCorridor',function()
    local p={1,2,3,4,5}; pc.mergeCorridorStartMoved(p,{8,7,3,2},8)
    -- Upstream dtMergeCorridorStartMoved reverses visited from its tail back to the furthest shared poly.
    jt.expect(p[1]).toBe(2); jt.expect(p[2]).toBe(3); jt.expect(p[3]).toBe(4); jt.expect(p[#p]).toBe(5)
    local q={1,2,3,4,5}; pc.mergeCorridorStartShortcut(q,{1,3,4},8)
    jt.expect(q[1]).toBe(1); jt.expect(q[2]).toBe(3); jt.expect(q[#q]).toBe(5)
end)
suite.it('moves both ends of a corridor while keeping positions constrained',function()
    local nav=plane(); local filter=dt.newQueryFilter(); local c=pc.new(32)
    pc.reset(c,1,vector.create(0.1,0,0.5))
    local path=assert(dt.findPath(nav,1,6,filter)); pc.setCorridor(c,vector.create(2.9,0,0.5),path)
    expect(pc.movePosition(c,vector.create(1.2,0,0.5),nav,filter)); expect(pc.getPos(c).X>1)
    expect(pc.moveTargetPosition(c,vector.create(2.7,0,0.4),nav,filter)); expect(pc.getTarget(c).X>2.5)
    expect(pc.isValid(c,10,nav,filter))
end)
suite.it('finds corners, visibility-shortcuts, topology-optimizes, and trims invalid refs',function()
    local nav=plane(); local filter=dt.newQueryFilter({excludeFlags=0x2}); local c=pc.new(32)
    pc.reset(c,1,vector.create(0.1,0,0.5)); local path=assert(dt.findPath(nav,1,6,filter)); pc.setCorridor(c,vector.create(2.9,0,0.5),path)
    local pts,flags,refs=pc.findCorners(c,nav,filter,4); expect(pts~=nil and #pts>=1 and #pts==#flags and #pts==#refs)
    pc.optimizePathVisibility(c,vector.create(2.9,0,0.5),10,nav,filter)
    pc.optimizePathTopology(c,nav,filter)
    dt.setPolyFlags(nav,pc.getLastPoly(c),0x2)
    expect(not pc.isValid(c,32,nav,filter)); expect(pc.trimInvalidPath(c,pc.getFirstPoly(c),pc.getPos(c),nav,filter)); expect(pc.isValid(c,32,nav,filter))
end)
suite.it('returns straight-path flags and refs alongside funnel points',function()
    local nav=plane(); local path=assert(dt.findPath(nav,1,6,nil))
    local points,flags,refs=dt.findStraightPath(nav,vector.create(0.1,0,0.5),vector.create(2.9,0,0.5),path)
    expect(#points==#flags and #points==#refs)
    expect(bit32.band(flags[1],dt.DT_STRAIGHTPATH_START)~=0)
    expect(bit32.band(flags[#flags],dt.DT_STRAIGHTPATH_END)~=0 and refs[#refs]==0)
end)

suite.it('moves a corridor across a Detour off-mesh connection',function()
    local verts={
        vector.create(0,0,0),vector.create(0,0,1),vector.create(1,0,0),vector.create(1,0,1),
        vector.create(4,1,0),vector.create(4,1,1),vector.create(5,1,0),vector.create(5,1,1),
    }
    local nav=dt.buildTriangleNavMesh(verts,{{1,2,3},{3,2,4},{5,6,7},{7,6,8}})
    dt.addOffMeshConnections(nav,{{start=vector.create(0.8,0,0.5),finish=vector.create(4.2,1,0.5),radius=0.6,bidirectional=true}}, {walkableClimb=2})
    local path=assert(dt.findPath(nav,1,4,nil)); local offRef=5
    local c=pc.new(16); pc.reset(c,path[1],vector.create(0.2,0,0.5)); pc.setCorridor(c,vector.create(4.8,1,0.5),path)
    local ok,refs,startPos,endPos=pc.moveOverOffmeshConnection(c,offRef,nav)
    expect(ok and refs[2]==offRef and startPos~=nil and endPos~=nil)
    expect(math.abs(pc.getPos(c).X-endPos.X)<1e-5)
    expect(pc.getFirstPoly(c)~=offRef,'consumed off-mesh polygon must be removed from the corridor')
end)

return suite
