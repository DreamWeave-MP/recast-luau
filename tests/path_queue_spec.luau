--!native
local jt=require('@jess/test')
local suite=jt.new()
local dt=require('@recast/detour')
local pq=require('@recast/path-queue')
local function expect(v,m) if not v then error(m or 'expectation failed',2) end end
local function plane()
    local v={vector.create(0,0,0),vector.create(0,0,1),vector.create(1,0,0),vector.create(1,0,1),vector.create(2,0,0),vector.create(2,0,1),vector.create(3,0,0),vector.create(3,0,1)}
    return dt.buildTriangleNavMesh(v,{{1,2,3},{3,2,4},{3,4,5},{5,4,6},{5,6,7},{7,6,8}})
end
suite.it('queues eight Detour sliced requests and rejects a ninth',function()
    local nav=plane();local q=assert(pq.new(32,256,nav));local refs={}
    for i=1,8 do refs[i]=pq.request(q,1,6,vector.create(.1,0,.5),vector.create(2.9,0,.5),nil); expect(refs[i]~=0) end
    jt.expect(pq.request(q,1,6,vector.zero,vector.create(2.9,0,.5),nil)).toBe(0)
end)
suite.it('advances sliced requests with a shared iteration budget and returns paths',function()
    local nav=plane();local q=assert(pq.new(32,256,nav));local ref=pq.request(q,1,6,vector.create(.1,0,.5),vector.create(2.9,0,.5),nil)
    local guard=0
    while not dt.statusSucceed(pq.getRequestStatus(q,ref)) and not dt.statusFailed(pq.getRequestStatus(q,ref)) do pq.update(q,1); guard+=1; expect(guard<64) end
    local path,status=pq.getPathResult(q,ref,32);expect(dt.statusSucceed(status));jt.expect(path[1]).toBe(1);jt.expect(path[#path]).toBe(6)
    expect(dt.statusFailed(pq.getRequestStatus(q,ref)))
end)
suite.it('expires unread completed results after two keepalive updates',function()
    local nav=plane();local q=assert(pq.new(32,256,nav));local ref=pq.request(q,1,2,vector.create(.1,0,.5),vector.create(.8,0,.5),nil)
    for _=1,16 do pq.update(q,64); if dt.statusSucceed(pq.getRequestStatus(q,ref)) then break end end
    expect(dt.statusSucceed(pq.getRequestStatus(q,ref)))
    pq.update(q,64);pq.update(q,64);pq.update(q,64)
    expect(dt.statusFailed(pq.getRequestStatus(q,ref)))
end)
suite.it('owns an independent dtNavMeshQuery context',function()
    local nav=plane();local q=assert(pq.new(32,256,nav));
    local query=pq.getNavQuery(q)
    expect(query~=nil and query:getAttachedNavMesh()==nav,'path queue must own a query object')
    local external=dt.newNavMeshQuery(nav,256)
    local status=external:initSlicedFindPath(1,4,vector.create(.1,0,.5),vector.create(1.8,0,.5),nil)
    expect(dt.statusInProgress(status) or dt.statusSucceed(status))
    local externalState=external._slicedQuery
    local ref=pq.request(q,1,6,vector.create(.1,0,.5),vector.create(2.9,0,.5),nil);pq.update(q,1)
    expect(external._slicedQuery==externalState,'path queue must not borrow external query state')
    expect(nav._slicedQuery==nil,'query-object sliced state must not leak onto navmesh')
    expect(pq.getRequestStatus(q,ref)~=dt.DT_FAILURE)
end)
suite.it('interleaves two query objects on one navmesh without state collision',function()
    local nav=plane(); local a=assert(dt.newNavMeshQuery(nav,256)); local b=assert(dt.newNavMeshQuery(nav,256))
    expect(a:getNodePool()==a:getNodePool() and a:getNodePool()~=b:getNodePool(),'query node-pool handles must have stable independent identity')
    local sa=a:initSlicedFindPath(1,6,vector.create(.1,0,.5),vector.create(2.9,0,.5),nil)
    local sb=b:initSlicedFindPath(1,4,vector.create(.1,0,.5),vector.create(1.8,0,.5),nil)
    expect(dt.statusInProgress(sa) or dt.statusSucceed(sa)); expect(dt.statusInProgress(sb) or dt.statusSucceed(sb))
    local guard=0
    while dt.statusInProgress(sa) or dt.statusInProgress(sb) do
        if dt.statusInProgress(sa) then sa=a:updateSlicedFindPath(1) end
        if dt.statusInProgress(sb) then sb=b:updateSlicedFindPath(1) end
        guard+=1; expect(guard<64)
    end
    local pa,sta=a:finalizeSlicedFindPath(); local pb,stb=b:finalizeSlicedFindPath()
    expect(dt.statusSucceed(sta) and dt.statusSucceed(stb)); jt.expect(pa[1]).toBe(1);jt.expect(pa[#pa]).toBe(6);jt.expect(pb[#pb]).toBe(4)
    expect(nav._slicedQuery==nil and nav._lastDijkstra==nil,'query object state must remain detached from navmesh')
end)
suite.it('honors dtNavMeshQuery maxNodes and reports DT_OUT_OF_NODES',function()
    local nav=plane(); local q=assert(dt.newNavMeshQuery(nav,2))
    local status=q:initSlicedFindPath(1,6,vector.create(.1,0,.5),vector.create(2.9,0,.5),nil)
    local guard=0
    while dt.statusInProgress(status) do status=q:updateSlicedFindPath(8); guard+=1; expect(guard<16) end
    expect(dt.statusDetail(status,dt.DT_OUT_OF_NODES),'sliced query must expose node-pool exhaustion')
    local path,finalStatus=q:finalizeSlicedFindPath()
    expect(path~=nil and #path>=1); expect(dt.statusDetail(finalStatus,dt.DT_OUT_OF_NODES)); expect(dt.statusDetail(finalStatus,dt.DT_PARTIAL_RESULT))
end)
return suite
