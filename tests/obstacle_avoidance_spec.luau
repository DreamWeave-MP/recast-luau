--!native
local jt=require('@jess/test')
local suite=jt.new()
local oa=require('@recast/obstacle-avoidance')
local function expect(v,m) if not v then error(m or 'expectation failed',2) end end

suite.it('stores fixed circle and segment obstacle pools',function()
    local q=oa.newQuery(2,1)
    expect(oa.addCircle(q,vector.create(1,0,0),0.5,vector.zero,vector.zero))
    expect(oa.addCircle(q,vector.create(2,0,0),0.5,vector.zero,vector.zero))
    expect(not oa.addCircle(q,vector.create(3,0,0),0.5,vector.zero,vector.zero))
    expect(oa.addSegment(q,vector.create(0,0,1),vector.create(1,0,1)))
    jt.expect(oa.getObstacleCircleCount(q)).toBe(2)
    jt.expect(oa.getObstacleSegmentCount(q)).toBe(1)
    oa.reset(q); jt.expect(oa.getObstacleCircleCount(q)).toBe(0)
end)

suite.it('samples bounded grid and adaptive avoidance velocities',function()
    local q=oa.newQuery(6,8)
    oa.addCircle(q,vector.create(1,0,0),0.35,vector.zero,vector.zero)
    local params=oa.defaultParams()
    local desired=vector.create(1.5,0,0)
    local grid,gn=oa.sampleVelocityGrid(q,vector.zero,0.35,2,vector.zero,desired,params)
    local adaptive,an=oa.sampleVelocityAdaptive(q,vector.zero,0.35,2,vector.zero,desired,params)
    expect(gn>0 and an>0)
    expect(vector.magnitude(grid)<=2.01 and vector.magnitude(adaptive)<=2.01)
    expect(math.abs(adaptive.Z)>1e-4 or adaptive.X<desired.X, 'adaptive sampler should react to the head-on circle')
end)

suite.it('collects and normalizes avoidance debug samples',function()
    local q=oa.newQuery(1,0)
    oa.addCircle(q,vector.create(1,0,0),0.25,vector.zero,vector.zero)
    local debug=oa.newDebugData(512)
    oa.sampleVelocityAdaptive(q,vector.zero,0.25,1,vector.zero,vector.create(1,0,0),oa.defaultParams(),debug)
    expect(debug.count>0)
    oa.normalizeDebugSamples(debug)
    for i=1,debug.count do expect(debug.pen[i]>=0 and debug.pen[i]<=1) end
end)


suite.it('matches upstream Detour obstacle-avoidance oracle fixtures',function()
    local function near(a,b,eps) return math.abs(a-b)<=(eps or 2e-6) end
    local p=oa.defaultParams()

    local q=oa.newQuery(6,8)
    oa.addCircle(q,vector.create(1,0,0),0.35,vector.zero,vector.zero)
    local debug=oa.newDebugData(4096)
    local grid,gn=oa.sampleVelocityGrid(q,vector.zero,0.35,2,vector.create(0.2,0,0.1),vector.create(1.5,0,0.25),p,debug)
    jt.expect(gn).toBe(1075); jt.expect(oa.getSampleCount(debug)).toBe(213)
    expect(near(grid.X,0.675000072) and near(grid.Z,0.625),'head-on grid oracle mismatch')
    oa.normalizeDebugSamples(debug)
    expect(near(oa.getSamplePenalty(debug,1),0.958281457,3e-6),'head-on normalized penalty mismatch')

    local adaptive,an=oa.sampleVelocityAdaptive(q,vector.zero,0.35,2,vector.create(0.2,0,0.1),vector.create(1.5,0,0.25),p,debug)
    jt.expect(an).toBe(75); jt.expect(oa.getSampleCount(debug)).toBe(8)
    expect(near(adaptive.X,0.489587367) and near(adaptive.Z,0.436838686),'head-on adaptive oracle mismatch')

    q=oa.newQuery(6,8)
    oa.addCircle(q,vector.create(1,0,0.2),0.3,vector.create(-0.1,0,0.05),vector.create(-0.2,0,0.1))
    oa.addCircle(q,vector.create(0.3,0,-0.8),0.25,vector.create(0.05,0,0.15),vector.create(0.1,0,0.2))
    oa.addSegment(q,vector.create(-0.8,0,0.6),vector.create(0.8,0,0.6))
    oa.addSegment(q,vector.create(-0.7,0,-0.6),vector.create(-0.7,0,0.6))
    grid,gn=oa.sampleVelocityGrid(q,vector.zero,0.35,2,vector.create(0.2,0,0.1),vector.create(1.5,0,0.25),p,debug)
    jt.expect(gn).toBe(1075); jt.expect(oa.getSampleCount(debug)).toBe(93)
    expect(near(grid.X,0.0750000477) and near(grid.Z,0.100000024),'mixed grid oracle mismatch')
    adaptive,an=oa.sampleVelocityAdaptive(q,vector.zero,0.35,2,vector.create(0.2,0,0.1),vector.create(1.5,0,0.25),p,debug)
    jt.expect(an).toBe(75); jt.expect(oa.getSampleCount(debug)).toBe(31)
    expect(near(adaptive.X,0.113867618) and near(adaptive.Z,0.101524323),'mixed adaptive oracle mismatch')
end)

return suite
