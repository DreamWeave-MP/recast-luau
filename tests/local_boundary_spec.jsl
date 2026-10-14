--!native
local jt=require('@jess/test')
local suite=jt.new()
local dt=require('@recast/detour')
local lb=require('@recast/local-boundary')
local function expect(v,m) if not v then error(m or 'expectation failed',2) end end
local function navPlane()
    local v={vector.create(0,0,0),vector.create(0,0,2),vector.create(2,0,0),vector.create(2,0,2),vector.create(4,0,0),vector.create(4,0,2)}
    return dt.buildTriangleNavMesh(v,{{1,2,3},{3,2,4},{3,4,5},{5,4,6}})
end
suite.it('caches the nearest eight local wall segments across a polygon neighbourhood',function()
    local nav=navPlane(); local b=lb.new(); local filter=dt.newQueryFilter()
    lb.update(b,nav,1,vector.create(1,0,1),4,filter)
    expect(lb.isValid(b,nav,filter)); expect(#b.polys>=2); expect(lb.getSegmentCount(b)>0 and lb.getSegmentCount(b)<=8)
    for i=2,#b.segments do expect(b.segments[i].distance>=b.segments[i-1].distance) end
end)
suite.it('invalidates when a cached polygon no longer passes the filter',function()
    local nav=navPlane(); local b=lb.new(); local filter=dt.newQueryFilter({excludeFlags=0x2})
    lb.update(b,nav,1,vector.create(1,0,1),4,filter)
    local victim=b.polys[#b.polys]; dt.setPolyFlags(nav,victim,0x2)
    expect(not lb.isValid(b,nav,filter))
    lb.reset(b); jt.expect(lb.getSegmentCount(b)).toBe(0)
end)
return suite
