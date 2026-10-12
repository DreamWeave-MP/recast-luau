--!native

-- Altered pure-Luau port of DetourCrowd's dtPathQueue.  The queue intentionally keeps the
-- upstream eight-request shape and drives Detour's sliced query API incrementally.

local Detour=require('@recast/detour')

local PathQueue={}
PathQueue.DT_PATHQ_INVALID=0
PathQueue.MAX_QUEUE=8
PathQueue.MAX_KEEP_ALIVE=2

local function deactivate(q)
    q.ref=PathQueue.DT_PATHQ_INVALID
    q.status=0
    q.path=nil
    q.npath=0
    q.filter=nil
    q.keepAlive=0
end

function PathQueue.new(maxPathSize,maxSearchNodeCount,navmesh)
    local q={}
    if not PathQueue.init(q,maxPathSize,maxSearchNodeCount,navmesh) then return nil end
    return q
end

function PathQueue.init(queue,maxPathSize,maxSearchNodeCount,navmesh)
    if queue==nil or navmesh==nil or (maxPathSize or 0)<=0 or (maxSearchNodeCount or 0)<=0 then return false end
    queue.navmesh=navmesh
    queue.maxPathSize=math.floor(maxPathSize)
    queue.maxSearchNodeCount=math.floor(maxSearchNodeCount)
    queue.nextHandle=1
    queue.queueHead=0
    queue.navquery=Detour.newNavMeshQuery(navmesh,maxSearchNodeCount)
    if queue.navquery==nil then return false end
    queue.entries=table.create(PathQueue.MAX_QUEUE)
    for i=1,PathQueue.MAX_QUEUE do
        queue.entries[i]={ref=PathQueue.DT_PATHQ_INVALID,status=0,path=nil,npath=0,keepAlive=0,filter=nil}
    end
    return true
end

function PathQueue.purge(queue)
    if queue==nil or queue.entries==nil then return end
    for i=1,PathQueue.MAX_QUEUE do deactivate(queue.entries[i]) end
    queue.queueHead=0
    queue.navquery=Detour.newNavMeshQuery(queue.navmesh,queue.maxSearchNodeCount)
end

function PathQueue.request(queue,startRef,endRef,startPos,endPos,filter)
    local slot=nil
    for i=1,PathQueue.MAX_QUEUE do
        if queue.entries[i].ref==PathQueue.DT_PATHQ_INVALID then slot=i break end
    end
    if slot==nil then return PathQueue.DT_PATHQ_INVALID end

    local ref=queue.nextHandle
    queue.nextHandle=(queue.nextHandle+1)%0x100000000
    if queue.nextHandle==PathQueue.DT_PATHQ_INVALID then queue.nextHandle=1 end

    local q=queue.entries[slot]
    q.ref=ref
    q.startPos=startPos; q.startRef=startRef
    q.endPos=endPos; q.endRef=endRef
    q.status=0; q.path=nil; q.npath=0
    q.filter=filter; q.keepAlive=0
    return ref
end

function PathQueue.update(queue,maxIters)
    local iterCount=math.max(0,math.floor(maxIters or 0))
    for _=1,PathQueue.MAX_QUEUE do
        local index=(queue.queueHead%PathQueue.MAX_QUEUE)+1
        local q=queue.entries[index]

        if q.ref==PathQueue.DT_PATHQ_INVALID then
            queue.queueHead+=1
            continue
        end

        if Detour.statusSucceed(q.status) or Detour.statusFailed(q.status) then
            q.keepAlive+=1
            if q.keepAlive>PathQueue.MAX_KEEP_ALIVE then deactivate(q) end
            queue.queueHead+=1
            continue
        end

        if q.status==0 then
            q.status=queue.navquery:initSlicedFindPath(q.startRef,q.endRef,q.startPos,q.endPos,q.filter)
        end
        if Detour.statusInProgress(q.status) then
            local status,iters=queue.navquery:updateSlicedFindPath(iterCount)
            q.status=status
            iterCount-=iters
        end
        if Detour.statusSucceed(q.status) then
            local path,status=queue.navquery:finalizeSlicedFindPath()
            q.path=path or {}
            q.npath=#q.path
            if q.npath>queue.maxPathSize then
                local trimmed=table.create(queue.maxPathSize)
                for i=1,queue.maxPathSize do trimmed[i]=q.path[i] end
                q.path=trimmed; q.npath=queue.maxPathSize
                status=bit32.bor(status,Detour.DT_BUFFER_TOO_SMALL)
            end
            q.status=status
        end

        if iterCount<=0 then break end
        queue.queueHead+=1
    end
end

function PathQueue.getRequestStatus(queue,ref)
    for i=1,PathQueue.MAX_QUEUE do
        local q=queue.entries[i]
        if q.ref==ref then return q.status end
    end
    return Detour.DT_FAILURE
end

function PathQueue.getPathResult(queue,ref,maxPath)
    maxPath=math.max(0,math.floor(maxPath or queue.maxPathSize))
    for i=1,PathQueue.MAX_QUEUE do
        local q=queue.entries[i]
        if q.ref==ref then
            local status=bit32.bor(bit32.band(q.status,Detour.DT_STATUS_DETAIL_MASK),Detour.DT_SUCCESS)
            local n=math.min(q.npath or 0,maxPath)
            local out=table.create(n)
            for j=1,n do out[j]=q.path[j] end
            deactivate(q)
            return out,status
        end
    end
    return nil,Detour.DT_FAILURE
end

function PathQueue.getNavQuery(queue)
    return queue.navquery
end

return table.freeze(PathQueue)
