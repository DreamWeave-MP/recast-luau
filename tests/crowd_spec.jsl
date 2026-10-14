--!native

local jt = require('@jess/test')
local suite = jt.new()
local dt = require('@recast/detour')
local crowdlib = require('@recast/crowd')
local pq = require('@recast/path-queue')
local avoidance = require('@recast/obstacle-avoidance')
local builder = require('@recast/navmesh-builder')

local function expect(condition, message)
    if not condition then error(message or 'expectation failed', 2) end
end

local function navPlane()
    local v = {
        vector.create(0,0,0), vector.create(0,0,1), vector.create(1,0,0),
        vector.create(1,0,1), vector.create(2,0,0), vector.create(2,0,1),
        vector.create(3,0,0), vector.create(3,0,1),
    }
    local t = {
        {1,2,3},{3,2,4}, {3,4,5},{5,4,6}, {5,6,7},{7,6,8},
    }
    return dt.buildTriangleNavMesh(v, t)
end


local function oracleGridNav()
    local N,nvp=4,4
    local vw=N+1
    local verts=buffer.create(vw*vw*3*2)
    local function vi(x,z) return z*vw+x end
    for z=0,N do for x=0,N do
        local o=vi(x,z)*6
        buffer.writeu16(verts,o,x);buffer.writeu16(verts,o+2,0);buffer.writeu16(verts,o+4,z)
    end end
    local np=N*N
    local polys=buffer.create(np*nvp*2*2);buffer.fill(polys,0,0xff)
    local flags=buffer.create(np*2);local areas=buffer.create(np)
    local function pi(x,z) return z*N+x end
    for z=0,N-1 do for x=0,N-1 do
        local poly=pi(x,z);local base=poly*nvp*4
        local vs={vi(x,z),vi(x+1,z),vi(x+1,z+1),vi(x,z+1)}
        for j=0,3 do buffer.writeu16(polys,base+j*2,vs[j+1]) end
        if z>0 then buffer.writeu16(polys,base+nvp*2,pi(x,z-1)) end
        if x<N-1 then buffer.writeu16(polys,base+(nvp+1)*2,pi(x+1,z)) end
        if z<N-1 then buffer.writeu16(polys,base+(nvp+2)*2,pi(x,z+1)) end
        if x>0 then buffer.writeu16(polys,base+(nvp+3)*2,pi(x-1,z)) end
        buffer.writeu16(flags,poly*2,1)
    end end
    local data=assert(builder.createNavMeshData({verts=verts,vertCount=vw*vw,polys=polys,polyFlags=flags,polyAreas=areas,polyCount=np,nvp=nvp,
        walkableHeight=2,walkableRadius=.2,walkableClimb=.5,bmin=vector.zero,bmax=vector.create(N,1,N),cs=1,ch=1,buildBvTree=true}))
    local nav,status=dt.loadNavMeshData(data)
    expect(nav~=nil and dt.statusSucceed(status),'oracle navmesh failed to load')
    return nav
end

suite.it('moves an agent along a Luau Detour path', function()
    local nav = navPlane()
    local crowd = crowdlib.new(nav, 4, {separationWeight = 0})
    local id = crowdlib.addAgent(crowd, vector.create(0.1,0,0.5), {maxSpeed=2, maxAcceleration=20, radius=0.2})
    expect(id ~= nil)
    local ok, err = crowdlib.requestMoveTarget(crowd, id, vector.create(2.8,0,0.5))
    expect(ok, err)
    for _ = 1, 30 do crowdlib.update(crowd, 0.1) end
    local agent = crowdlib.getAgent(crowd, id)
    expect(agent.position.X > 2.4, `agent stopped at {agent.position.X}`)
end)

suite.it('reuses removed crowd slots', function()
    local crowd = crowdlib.new(navPlane(), 1)
    local a = crowdlib.addAgent(crowd, vector.zero)
    jt.expect(a).toBe(1)
    local b = crowdlib.addAgent(crowd, vector.zero)
    jt.expect(b).toBeNil()
    jt.expect(crowdlib.removeAgent(crowd, 1)).toBeTruthy()
    local c = crowdlib.addAgent(crowd, vector.zero)
    jt.expect(c).toBe(1)
end)


suite.it('uses the Detour proximity grid and keeps six nearest neighbours ordered', function()
    local nav = navPlane()
    local crowd = crowdlib.new(nav, 8, {maxAgentRadius=0.25, constrainToNavmesh=false})
    local center = assert(crowdlib.addAgent(crowd, vector.create(1.5,0,0.5), {radius=0.2, collisionQueryRange=2, updateFlags=0}))
    for i=1,7 do
        local angle = i * math.pi * 2 / 7
        assert(crowdlib.addAgent(crowd, vector.create(1.5+math.cos(angle)*(0.3+i*0.03),0,0.5+math.sin(angle)*(0.3+i*0.03)), {radius=0.2, collisionQueryRange=2, updateFlags=0}))
    end
    crowdlib.update(crowd, 1/60)
    local neighbours = crowdlib.getAgentNeighbours(crowd, center)
    jt.expect(#neighbours).toBe(crowdlib.DT_CROWDAGENT_MAX_NEIGHBOURS)
    for i=2,#neighbours do expect(neighbours[i].dist >= neighbours[i-1].dist) end
    expect(crowdlib.getGrid(crowd) ~= nil)
end)

suite.it('matches Detour direct velocity requests without maxSpeed clamping', function()
    local crowd = crowdlib.new(navPlane(), 1, {constrainToNavmesh=false})
    local id = assert(crowdlib.addAgent(crowd, vector.create(0.2,0,0.5), {maxSpeed=1, maxAcceleration=100, updateFlags=0}))
    expect(crowdlib.requestMoveVelocity(crowd,id,vector.create(5,0,0)))
    crowdlib.update(crowd,0.25)
    local agent = crowdlib.getAgent(crowd,id)
    jt.expect(agent.targetState).toBe(crowdlib.DT_CROWDAGENT_TARGET_VELOCITY)
    expect(math.abs(agent.velocity.X-5)<1e-5, `velocity request was clamped to {agent.velocity.X}`)
    expect(math.abs(agent.position.X-1.45)<1e-4, `unexpected integrated position {agent.position.X}`)
    jt.expect(agent.targetRef).toBe(0)
end)

suite.it('recovers an invalid current corridor polygon like DetourCrowd', function()
    local nav=navPlane()
    local crowd=crowdlib.new(nav,1,{constrainToNavmesh=false})
    local id=assert(crowdlib.addAgent(crowd,vector.create(0.2,0,0.5),{radius=.2,maxSpeed=0,updateFlags=0}))
    local before=crowdlib.getAgent(crowd,id)
    expect(dt.setPolyFlags(nav,before.currentRef,0))
    crowdlib.update(crowd,1/60)
    local after=crowdlib.getAgent(crowd,id)
    jt.expect(after.state).toBe(crowdlib.DT_CROWDAGENT_STATE_WALKING)
    expect(after.currentRef~=before.currentRef,'invalid start ref was not recovered')
    expect(dt.isValidPolyRef(nav,after.currentRef,nil))
end)


suite.it('runs Detour adaptive obstacle avoidance through the crowd update flag', function()
    local nav = navPlane()
    local crowd = crowdlib.new(nav, 2, {maxAgentRadius=0.25, constrainToNavmesh=false})
    local flags = crowdlib.DT_CROWD_OBSTACLE_AVOIDANCE
    local a = assert(crowdlib.addAgent(crowd, vector.create(1.0,0,0.5), {radius=0.2,maxSpeed=1,maxAcceleration=100,collisionQueryRange=2,updateFlags=flags}))
    local b = assert(crowdlib.addAgent(crowd, vector.create(2.0,0,0.5), {radius=0.2,maxSpeed=1,maxAcceleration=100,collisionQueryRange=2,updateFlags=flags}))
    crowdlib.requestMoveVelocity(crowd,a,vector.create(1,0,0))
    crowdlib.requestMoveVelocity(crowd,b,vector.create(-1,0,0))
    crowdlib.update(crowd,0.05)
    local aa,bb=crowdlib.getAgent(crowd,a),crowdlib.getAgent(crowd,b)
    expect(crowdlib.getVelocitySampleCount(crowd)>0)
    expect(aa.adjustedVelocity ~= aa.desiredVelocity or bb.adjustedVelocity ~= bb.desiredVelocity, 'avoidance should alter a head-on request')
end)

suite.it('stores all eight Detour obstacle-avoidance parameter presets', function()
    local crowd=crowdlib.new(navPlane(),1)
    local p=crowdlib.getObstacleAvoidanceParams(crowd,3)
    jt.expect(p.adaptiveDivs).toBe(7)
    expect(crowdlib.setObstacleAvoidanceParams(crowd,3,{adaptiveDivs=5,adaptiveDepth=3}))
    local changed=crowdlib.getObstacleAvoidanceParams(crowd,3)
    jt.expect(changed.adaptiveDivs).toBe(5)
    jt.expect(changed.adaptiveDepth).toBe(3)
    jt.expect(changed.weightToi).toBe(2.5)
end)


suite.it('maintains a Detour local boundary cache for walking agents', function()
    local crowd=crowdlib.new(navPlane(),1,{maxAgentRadius=0.25})
    local id=assert(crowdlib.addAgent(crowd,vector.create(1.2,0,0.5),{radius=0.2,collisionQueryRange=2,updateFlags=0}))
    crowdlib.update(crowd,1/60)
    local agent=crowdlib.getAgent(crowd,id)
    expect(agent.boundary~=nil and #agent.boundary.polys>0 and #agent.boundary.segments>0)
end)


suite.it('runs long move requests through the Detour path queue lifecycle', function()
    local n=180
    local verts={}
    for x=0,n do verts[#verts+1]=vector.create(x,0,0);verts[#verts+1]=vector.create(x,0,1) end
    local tris={}
    for x=0,n-1 do local a=x*2+1;tris[#tris+1]={a,a+1,a+2};tris[#tris+1]={a+2,a+1,a+3} end
    local nav=dt.buildTriangleNavMesh(verts,tris)
    local crowd=crowdlib.new(nav,1,{maxPath=512,maxPathQueueNodes=2048,constrainToNavmesh=false})
    local id=assert(crowdlib.addAgent(crowd,vector.create(.1,0,.5),{radius=.2,maxSpeed=0,updateFlags=0,collisionQueryRange=1}))
    expect(crowdlib.requestMoveTarget(crowd,id,vector.create(n-.1,0,.5)))
    jt.expect(crowdlib.getAgent(crowd,id).targetState).toBe(crowdlib.DT_CROWDAGENT_TARGET_REQUESTING)
    crowdlib.update(crowd,1/60)
    local first=crowdlib.getAgent(crowd,id)
    expect(first.targetState==crowdlib.DT_CROWDAGENT_TARGET_WAITING_FOR_PATH or first.targetState==crowdlib.DT_CROWDAGENT_TARGET_VALID,
        `unexpected async state {first.targetState}`)
    local sawWaiting=first.targetState==crowdlib.DT_CROWDAGENT_TARGET_WAITING_FOR_PATH
    local guard=0
    while crowdlib.getAgent(crowd,id).targetState~=crowdlib.DT_CROWDAGENT_TARGET_VALID do
        crowdlib.update(crowd,1/60);guard+=1;expect(guard<32,'queued path did not complete')
        if crowdlib.getAgent(crowd,id).targetState==crowdlib.DT_CROWDAGENT_TARGET_WAITING_FOR_PATH then sawWaiting=true end
    end
    expect(sawWaiting,'fixture must exercise the path queue, not only the quick search')
    local agent=crowdlib.getAgent(crowd,id)
    jt.expect(agent.targetPathqRef).toBe(0)
    expect(agent.corridor[#agent.corridor]==agent.targetRef)
    local query=crowdlib.getNavMeshQuery(crowd)
    expect(crowdlib.getPathQueue(crowd)~=nil and query~=nil and query:getAttachedNavMesh()==nav)
    expect(pq.getNavQuery(crowdlib.getPathQueue(crowd))~=query,'crowd and path queue must own independent query contexts')
end)

suite.it('enters and completes DetourCrowd off-mesh animation state', function()
    local verts={
        vector.create(0,0,0),vector.create(0,0,1),vector.create(1,0,0),vector.create(1,0,1),
        vector.create(4,1,0),vector.create(4,1,1),vector.create(5,1,0),vector.create(5,1,1),
    }
    local nav=dt.buildTriangleNavMesh(verts,{{1,2,3},{3,2,4},{5,6,7},{7,6,8}})
    dt.addOffMeshConnections(nav,{{start=vector.create(.8,0,.5),finish=vector.create(4.2,1,.5),radius=.6,bidirectional=true}},{walkableClimb=2})
    local crowd=crowdlib.new(nav,1,{maxAgentRadius=.25})
    local id=assert(crowdlib.addAgent(crowd,vector.create(.2,0,.5),{radius=.2,maxSpeed=2,maxAcceleration=20,updateFlags=0,collisionQueryRange=1}))
    expect(crowdlib.requestMoveTarget(crowd,id,vector.create(4.8,1,.5)))
    local sawOffmesh=false
    for _=1,80 do
        crowdlib.update(crowd,.1)
        local ag=crowdlib.getAgent(crowd,id)
        if ag.state==crowdlib.DT_CROWDAGENT_STATE_OFFMESH then sawOffmesh=true end
        if sawOffmesh and ag.state==crowdlib.DT_CROWDAGENT_STATE_WALKING and ag.position.X>4.1 then break end
    end
    local ag=crowdlib.getAgent(crowd,id)
    expect(sawOffmesh,'crowd never entered off-mesh animation state')
    jt.expect(ag.state).toBe(crowdlib.DT_CROWDAGENT_STATE_WALKING)
    expect(ag.position.X>4.1,`off-mesh traversal stopped at {ag.position.X}`)
end)

suite.it('accepts the Detour poly-ref move-target form without nearest-poly substitution', function()
    local nav=navPlane()
    local crowd=crowdlib.new(nav,1,{constrainToNavmesh=false})
    local id=assert(crowdlib.addAgent(crowd,vector.create(.2,0,.5),{maxSpeed=0,updateFlags=0}))
    local targetPos=vector.create(2.75,0,.4)
    local targetRef=assert(dt.findNearestPoly(nav,targetPos,vector.create(.5,1,.5)))
    expect(crowdlib.requestMoveTarget(crowd,id,targetRef,targetPos))
    local ag=crowdlib.getAgent(crowd,id)
    jt.expect(ag.targetRef).toBe(targetRef)
    jt.expect(ag.target).toBe(targetPos)
    jt.expect(ag.targetState).toBe(crowdlib.DT_CROWDAGENT_TARGET_REQUESTING)
    expect(not crowdlib.requestMoveTarget(crowd,id,0,targetPos),'upstream requestMoveTarget rejects a zero poly ref')
end)

suite.it('exposes Crowd agent state and query-half-extents parity fields', function()
    local crowd=crowdlib.new(navPlane(),1,{maxAgentRadius=.4})
    local id=assert(crowdlib.addAgent(crowd,vector.create(.2,0,.5),{radius=.2,maxSpeed=2,maxAcceleration=8,collisionQueryRange=3,pathOptimizationRange=6,separationWeight=2,obstacleAvoidanceType=3,queryFilterType=0}))
    local ag=crowdlib.getAgent(crowd,id)
    expect(ag.active)
    jt.expect(ag.collisionQueryRange).toBe(3)
    jt.expect(ag.pathOptimizationRange).toBe(6)
    jt.expect(ag.separationWeight).toBe(2)
    jt.expect(ag.obstacleAvoidanceType).toBe(3)
    jt.expect(ag.desiredSpeed).toBe(0)
    jt.expect(ag.targetReplan).toBeFalsy()
    jt.expect(crowdlib.getQueryHalfExtents(crowd)).toBe(crowdlib.getQueryExtents(crowd))
end)



suite.it('matches an upstream dtCrowd head-on steering trajectory', function()
    local nav=oracleGridNav()
    local crowd=crowdlib.new(nav,4,{maxAgentRadius=.25,constrainToNavmesh=false})
    local params={radius=.2,height=2,maxAcceleration=100,maxSpeed=1,collisionQueryRange=2,pathOptimizationRange=4,separationWeight=1.25,
        updateFlags=crowdlib.DT_CROWD_OBSTACLE_AVOIDANCE,obstacleAvoidanceType=0,queryFilterType=0}
    local a=assert(crowdlib.addAgent(crowd,vector.create(1,0,2),params))
    local b=assert(crowdlib.addAgent(crowd,vector.create(3,0,2),params))
    assert(crowdlib.requestMoveVelocity(crowd,a,vector.create(1,0,0)))
    assert(crowdlib.requestMoveVelocity(crowd,b,vector.create(-1,0,0)))
    crowdlib.update(crowd,.05)
    local aa,bb=crowdlib.getAgent(crowd,a),crowdlib.getAgent(crowd,b)
    local function near(got,want,label) expect(math.abs(got-want)<2e-6,`{label}: got {got}, upstream {want}`) end
    near(aa.position.X,1.03979814,'a step1 x');near(aa.position.Z,1.99177408,'a step1 z')
    near(aa.adjustedVelocity.X,.795962155,'a step1 nvel x');near(aa.adjustedVelocity.Z,-.164519086,'a step1 nvel z')
    jt.expect(#aa.neighbours).toBe(1);jt.expect(aa.neighbours[1].id).toBe(b);near(aa.neighbours[1].dist,4,'a step1 neighbour dist2')
    near(bb.position.X,2.96020198,'b step1 x');near(bb.adjustedVelocity.Z,.164519086,'b step1 nvel z')
    crowdlib.update(crowd,.05)
    aa,bb=crowdlib.getAgent(crowd,a),crowdlib.getAgent(crowd,b)
    near(aa.position.X,1.08109796,'a step2 x');near(aa.position.Z,1.98330069,'a step2 z')
    near(aa.adjustedVelocity.X,.825995445,'a step2 nvel x');near(aa.adjustedVelocity.Z,-.169466913,'a step2 nvel z')
    near(bb.position.X,2.91890216,'b step2 x');near(bb.position.Z,2.01669931,'b step2 z')
end)


suite.it('matches upstream dtCrowd separation and overlap-resolution trajectory', function()
    local nav=oracleGridNav()
    local crowd=crowdlib.new(nav,4,{maxAgentRadius=.25,constrainToNavmesh=false})
    local params={radius=.2,height=2,maxAcceleration=100,maxSpeed=1,collisionQueryRange=2,pathOptimizationRange=4,separationWeight=1.25,
        updateFlags=crowdlib.DT_CROWD_SEPARATION,obstacleAvoidanceType=0,queryFilterType=0}
    local a=assert(crowdlib.addAgent(crowd,vector.create(2,0,2),params))
    local b=assert(crowdlib.addAgent(crowd,vector.create(2,0,2),params))
    assert(crowdlib.requestMoveVelocity(crowd,a,vector.create(1,0,0)))
    assert(crowdlib.requestMoveVelocity(crowd,b,vector.create(1,0,0)))
    local function near(got,want,label) expect(math.abs(got-want)<2e-6,`{label}: got {got}, upstream {want}`) end
    crowdlib.update(crowd,.05)
    local aa,bb=crowdlib.getAgent(crowd,a),crowdlib.getAgent(crowd,b)
    near(aa.position.X,2.04999995,'a overlap step1 x');near(aa.position.Z,1.80513012,'a overlap step1 z')
    near(bb.position.X,2.04999995,'b overlap step1 x');near(bb.position.Z,2.19487023,'b overlap step1 z')
    jt.expect(#aa.neighbours).toBe(1);near(aa.neighbours[1].dist,0,'overlap step1 neighbour dist2')
    crowdlib.update(crowd,.05)
    aa,bb=crowdlib.getAgent(crowd,a),crowdlib.getAgent(crowd,b)
    near(aa.position.X,2.07044077,'a overlap step2 x');near(aa.position.Z,1.78054941,'a overlap step2 z')
    near(aa.desiredVelocity.X,.408816814,'a overlap step2 dvel x');near(aa.desiredVelocity.Z,-.491615325,'a overlap step2 dvel z')
    near(bb.position.Z,2.21945095,'b overlap step2 z')
end)

suite.it('fills DetourCrowd debug visibility endpoints and avoidance samples', function()
    local nav=navPlane()
    local flags=bit32.bor(crowdlib.DT_CROWD_OPTIMIZE_VIS,crowdlib.DT_CROWD_OBSTACLE_AVOIDANCE)
    local crowd=crowdlib.new(nav,2,{maxAgentRadius=.25,constrainToNavmesh=false})
    local a=assert(crowdlib.addAgent(crowd,vector.create(.2,0,.35),{radius=.2,maxSpeed=1,maxAcceleration=100,collisionQueryRange=2,pathOptimizationRange=4,updateFlags=flags}))
    local b=assert(crowdlib.addAgent(crowd,vector.create(1.2,0,.65),{radius=.2,maxSpeed=0,maxAcceleration=100,collisionQueryRange=2,updateFlags=0}))
    local target=vector.create(2.8,0,.35)
    expect(crowdlib.requestMoveTarget(crowd,a,target))
    local debug={idx=a-1,vod=avoidance.newDebugData(256)}
    crowdlib.update(crowd,1/60,debug)
    expect(debug.optStart~=nil and debug.optEnd~=nil,'visibility debug endpoints were not populated')
    expect(vector.magnitude(debug.optEnd-debug.optStart)>0,'visibility debug endpoints were not meaningful')
    expect(avoidance.getSampleCount(debug.vod)>0,'avoidance debug sampler stayed empty')
    -- Keep the second agent live so the avoidance fixture is not optimized into an empty-neighbour case.
    expect(crowdlib.getAgent(crowd,b)~=nil)
end)


suite.it('retains Detour agent userData and honors getActiveAgents capacity', function()
    local crowd=crowdlib.new(navPlane(),3)
    local marker={name='npc'}
    local a=assert(crowdlib.addAgent(crowd,vector.zero,{userData=marker}))
    local b=assert(crowdlib.addAgent(crowd,vector.create(.1,0,.5),{}))
    local ag=crowdlib.getAgent(crowd,a)
    jt.expect(ag.userData).toBe(marker);jt.expect(ag.params.userData).toBe(marker)
    local marker2={name='updated'};expect(crowdlib.updateAgentParameters(crowd,a,{userData=marker2}))
    jt.expect(crowdlib.getAgent(crowd,a).params.userData).toBe(marker2)
    local active=crowdlib.getActiveAgents(crowd,1);jt.expect(#active).toBe(1);jt.expect(active[1].id).toBe(a)
    expect(crowdlib.removeAgent(crowd,b))
end)

return suite
