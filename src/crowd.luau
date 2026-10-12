--!native

-- Pure-Luau crowd/path-following layer over @recast/detour.
-- This intentionally keeps ownership in Luau buffers; no DetourCrowd C++ state is wrapped.

local Common = require('@recast/common')
local Detour = require('@recast/detour')
local ProximityGrid = require('@recast/proximity-grid')
local ObstacleAvoidance = require('@recast/obstacle-avoidance')
local LocalBoundary = require('@recast/local-boundary')
local PathQueue = require('@recast/path-queue')
local PathCorridor = require('@recast/path-corridor')

local Crowd = {}

Crowd.DT_CROWDAGENT_STATE_INVALID = 0
Crowd.DT_CROWDAGENT_STATE_WALKING = 1
Crowd.DT_CROWDAGENT_STATE_OFFMESH = 2

Crowd.DT_CROWDAGENT_TARGET_NONE = 0
Crowd.DT_CROWDAGENT_TARGET_FAILED = 1
Crowd.DT_CROWDAGENT_TARGET_VALID = 2
Crowd.DT_CROWDAGENT_TARGET_REQUESTING = 3
Crowd.DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE = 4
Crowd.DT_CROWDAGENT_TARGET_WAITING_FOR_PATH = 5
Crowd.DT_CROWDAGENT_TARGET_VELOCITY = 6

Crowd.DT_CROWD_ANTICIPATE_TURNS = 0x01
Crowd.DT_CROWD_OBSTACLE_AVOIDANCE = 0x02
Crowd.DT_CROWD_SEPARATION = 0x04
Crowd.DT_CROWD_OPTIMIZE_VIS = 0x08
Crowd.DT_CROWD_OPTIMIZE_TOPO = 0x10
Crowd.DT_CROWDAGENT_MAX_NEIGHBOURS = 6
Crowd.DT_CROWDAGENT_MAX_CORNERS = 4
Crowd.DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS = 8
Crowd.DT_CROWD_MAX_QUERY_FILTER_TYPE = 16

local function readVec(b, id)
    local o = (id - 1) * 12
    return vector.create(buffer.readf32(b,o), buffer.readf32(b,o+4), buffer.readf32(b,o+8))
end

local function writeVec(b, id, v)
    local o = (id - 1) * 12
    buffer.writef32(b,o,v.X); buffer.writef32(b,o+4,v.Y); buffer.writef32(b,o+8,v.Z)
end

local function copyInto(dst, src)
    table.clear(dst)
    for i = 1, #src do dst[i] = src[i] end
    return dst
end

function Crowd.mergeCorridorStartMoved(path, visited, maxPath)
    maxPath = maxPath or math.huge
    local furthestPath, furthestVisited = -1, -1
    for i = #path, 1, -1 do
        local found = false
        for j = #visited, 1, -1 do
            if path[i] == visited[j] then furthestPath, furthestVisited, found = i, j, true end
        end
        if found then break end
    end
    if furthestPath < 0 then return path end
    local out = {}
    for j = #visited, furthestVisited, -1 do if #out < maxPath then out[#out+1] = visited[j] end end
    for i = furthestPath + 1, #path do if #out < maxPath then out[#out+1] = path[i] end end
    return copyInto(path, out)
end

function Crowd.mergeCorridorEndMoved(path, visited, maxPath)
    maxPath = maxPath or math.huge
    local furthestPath, furthestVisited = -1, -1
    for i = 1, #path do
        local found = false
        for j = #visited, 1, -1 do
            if path[i] == visited[j] then furthestPath, furthestVisited, found = i, j, true end
        end
        if found then break end
    end
    if furthestPath < 0 then return path end
    local out = {}
    for i = 1, furthestPath do if #out < maxPath then out[#out+1] = path[i] end end
    for j = furthestVisited + 1, #visited do if #out < maxPath then out[#out+1] = visited[j] end end
    return copyInto(path, out)
end

function Crowd.mergeCorridorStartShortcut(path, visited, maxPath)
    maxPath = maxPath or math.huge
    local furthestPath, furthestVisited = -1, -1
    for i = #path, 1, -1 do
        local found = false
        for j = #visited, 1, -1 do
            if path[i] == visited[j] then furthestPath, furthestVisited, found = i, j, true end
        end
        if found then break end
    end
    if furthestPath < 0 or furthestVisited <= 1 then return path end
    local out = {}
    for j = 1, furthestVisited - 1 do if #out < maxPath then out[#out+1] = visited[j] end end
    for i = furthestPath, #path do if #out < maxPath then out[#out+1] = path[i] end end
    return copyInto(path, out)
end

function Crowd.new(navmesh, maxAgents, options)
    maxAgents = maxAgents or 128
    options = options or {}
    local maxAgentRadius = options.maxAgentRadius or 1.0
    local queryExtents = options.queryExtents or vector.create(maxAgentRadius*2, maxAgentRadius*1.5, maxAgentRadius*2)
    local filters = table.create(Crowd.DT_CROWD_MAX_QUERY_FILTER_TYPE)
    filters[1] = options.filter or Detour.newQueryFilter()
    for i = 2, Crowd.DT_CROWD_MAX_QUERY_FILTER_TYPE do filters[i] = Detour.newQueryFilter() end
    local avoidanceParams = table.create(Crowd.DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS)
    for i = 1, Crowd.DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS do avoidanceParams[i] = ObstacleAvoidance.defaultParams() end
    local crowd = {
        navmesh = navmesh, maxAgents = maxAgents, maxPath = options.maxPath or 256,
        active = buffer.create(maxAgents), state = buffer.create(maxAgents), partial = buffer.create(maxAgents),
        positions = buffer.create(maxAgents * 12), velocities = buffer.create(maxAgents * 12),
        desired = buffer.create(maxAgents * 12), adjusted = buffer.create(maxAgents * 12), targets = buffer.create(maxAgents * 12),
        displacement = buffer.create(maxAgents * 12), desiredSpeed = buffer.create(maxAgents * 4),
        radius = buffer.create(maxAgents * 4), height = buffer.create(maxAgents * 4),
        maxSpeed = buffer.create(maxAgents * 4), maxAcceleration = buffer.create(maxAgents * 4),
        collisionQueryRange = buffer.create(maxAgents * 4), pathOptimizationRange = buffer.create(maxAgents * 4),
        separationWeights = buffer.create(maxAgents * 4), updateFlags = buffer.create(maxAgents),
        obstacleAvoidanceType = buffer.create(maxAgents), queryFilterType = buffer.create(maxAgents),
        targetState = buffer.create(maxAgents), currentRef = buffer.create(maxAgents * 4), targetRef = buffer.create(maxAgents * 4),
        topologyOptTime = buffer.create(maxAgents * 4), targetReplanTime = buffer.create(maxAgents * 4), targetReplan = buffer.create(maxAgents), targetPathqRef = buffer.create(maxAgents * 4),
        corridors = {}, corridorTargets = {}, corners = {}, cornerFlags = {}, cornerRefs = {}, boundaries = {}, animations = {}, userData = {},
        neighborCount = buffer.create(maxAgents),
        neighborIds = buffer.create(maxAgents * Crowd.DT_CROWDAGENT_MAX_NEIGHBOURS * 2),
        neighborDists = buffer.create(maxAgents * Crowd.DT_CROWDAGENT_MAX_NEIGHBOURS * 4),
        filters = filters,
        separationWeight = options.separationWeight or 1.25, separationRange = options.separationRange or 2.5,
        constrainToNavmesh = options.constrainToNavmesh ~= false,
        queryExtents = queryExtents,
        topologyOptimizationInterval = options.topologyOptimizationInterval or 0.5,
        maxAgentRadius = maxAgentRadius,
        obstacleQuery = ObstacleAvoidance.newQuery(Crowd.DT_CROWDAGENT_MAX_NEIGHBOURS, 8),
        obstacleAvoidanceParams = avoidanceParams,
        velocitySampleCount = 0,
        navquery = assert(Detour.newNavMeshQuery(navmesh, options.maxQueryNodes or 512)),
        pathQueue = assert(PathQueue.new(options.maxPath or 256, options.maxPathQueueNodes or 4096, navmesh)),
    }
    crowd.grid = ProximityGrid.new(maxAgents * 4, crowd.maxAgentRadius * 3)
    crowd.gridQueryScratch = table.create(32)
    buffer.fill(crowd.neighborIds, 0, 0xff)
    return crowd
end

function Crowd.addAgent(crowd, position, params)
    params = params or {}
    local slot = nil
    for i = 0, crowd.maxAgents - 1 do if buffer.readu8(crowd.active, i) == 0 then slot = i break end end
    if slot == nil then return nil, 'crowd is full' end
    local id = slot + 1
    local radius, height = params.radius or 0.6, params.height or 2.0
    if radius > crowd.maxAgentRadius then return nil, 'agent radius exceeds crowd maxAgentRadius' end
    local filterType = math.clamp(params.queryFilterType or 0, 0, Crowd.DT_CROWD_MAX_QUERY_FILTER_TYPE - 1)
    local filter = crowd.filters[filterType + 1]
    local extents = vector.create(math.max(radius * 2, crowd.queryExtents.X), math.max(height, crowd.queryExtents.Y), math.max(radius * 2, crowd.queryExtents.Z))
    local ref, nearest = Detour.findNearestPoly(crowd.navmesh, position, extents, filter)
    if ref == nil then return nil, 'agent position is not near navigation mesh' end

    buffer.writeu8(crowd.active, slot, 1); buffer.writeu8(crowd.state, slot, Crowd.DT_CROWDAGENT_STATE_WALKING)
    writeVec(crowd.positions, id, nearest); writeVec(crowd.velocities, id, vector.zero); writeVec(crowd.desired, id, vector.zero); writeVec(crowd.adjusted, id, vector.zero); writeVec(crowd.targets, id, nearest)
    buffer.writef32(crowd.radius, slot*4, radius); buffer.writef32(crowd.height, slot*4, height)
    buffer.writef32(crowd.maxSpeed, slot*4, params.maxSpeed or 3.5); buffer.writef32(crowd.maxAcceleration, slot*4, params.maxAcceleration or 8)
    buffer.writef32(crowd.collisionQueryRange, slot*4, params.collisionQueryRange or radius*12)
    buffer.writef32(crowd.pathOptimizationRange, slot*4, params.pathOptimizationRange or radius*30)
    buffer.writef32(crowd.separationWeights, slot*4, params.separationWeight or crowd.separationWeight)
    buffer.writeu8(crowd.updateFlags, slot, params.updateFlags or bit32.bor(Crowd.DT_CROWD_ANTICIPATE_TURNS, Crowd.DT_CROWD_SEPARATION, Crowd.DT_CROWD_OPTIMIZE_VIS, Crowd.DT_CROWD_OPTIMIZE_TOPO))
    buffer.writeu8(crowd.obstacleAvoidanceType, slot, math.clamp(params.obstacleAvoidanceType or 0,0,7))
    buffer.writeu8(crowd.queryFilterType, slot, filterType)
    buffer.writeu8(crowd.targetState, slot, Crowd.DT_CROWDAGENT_TARGET_NONE)
    buffer.writeu32(crowd.currentRef, slot*4, ref); buffer.writeu32(crowd.targetRef, slot*4, 0)
    crowd.corridors[id] = {ref}; crowd.corridorTargets[id]=nearest; crowd.corners[id] = nil; crowd.boundaries[id] = LocalBoundary.new(); buffer.writeu8(crowd.neighborCount, slot, 0)
    crowd.cornerFlags[id], crowd.cornerRefs[id] = nil, nil
    crowd.animations[id] = {active=false,initPos=nearest,startPos=nearest,endPos=nearest,polyRef=0,t=0,tmax=0}
    crowd.userData[id] = params.userData
    return id
end

function Crowd.removeAgent(crowd, id)
    local slot = id - 1
    if slot < 0 or slot >= crowd.maxAgents or buffer.readu8(crowd.active, slot) == 0 then return false end
    buffer.writeu8(crowd.active, slot, 0)
    buffer.writeu8(crowd.targetState, slot, Crowd.DT_CROWDAGENT_TARGET_NONE)
    buffer.writeu8(crowd.state, slot, Crowd.DT_CROWDAGENT_STATE_INVALID)
    crowd.corridors[id], crowd.corridorTargets[id], crowd.corners[id], crowd.cornerFlags[id], crowd.cornerRefs[id], crowd.boundaries[id], crowd.animations[id], crowd.userData[id] = nil, nil, nil, nil, nil, nil, nil, nil
    buffer.writeu8(crowd.neighborCount, slot, 0)
    return true
end

local function agentFilter(crowd, slot)
    return crowd.filters[buffer.readu8(crowd.queryFilterType, slot) + 1]
end

local function setMoveTargetRequest(crowd,id,targetRef,target,replan,allowZeroRef)
    local slot=id-1
    if slot<0 or slot>=crowd.maxAgents or buffer.readu8(crowd.active,slot)==0 then return false,'invalid agent' end
    if not allowZeroRef and (targetRef==nil or targetRef==0) then return false,'invalid target polygon' end
    writeVec(crowd.targets,id,target)
    buffer.writeu32(crowd.targetRef,slot*4,targetRef or 0)
    buffer.writeu32(crowd.targetPathqRef,slot*4,0)
    buffer.writeu8(crowd.targetReplan,slot,if replan then 1 else 0)
    buffer.writeu8(crowd.targetState,slot,if targetRef~=nil and targetRef~=0 then Crowd.DT_CROWDAGENT_TARGET_REQUESTING else Crowd.DT_CROWDAGENT_TARGET_FAILED)
    return true
end

local function requestMoveTargetNearest(crowd,id,target,extents,replan)
    local slot=id-1
    if slot<0 or slot>=crowd.maxAgents or buffer.readu8(crowd.active,slot)==0 then return false,'invalid agent' end
    local filter=agentFilter(crowd,slot)
    extents=extents or crowd.queryExtents
    local targetRef,nearestEnd=Detour.findNearestPoly(crowd.navmesh,target,extents,filter)
    if targetRef==nil then
        buffer.writeu32(crowd.targetRef,slot*4,0); buffer.writeu32(crowd.targetPathqRef,slot*4,0)
        buffer.writeu8(crowd.targetReplan,slot,if replan then 1 else 0)
        buffer.writeu8(crowd.targetState,slot,Crowd.DT_CROWDAGENT_TARGET_FAILED)
        return false,'no target polygon'
    end
    return setMoveTargetRequest(crowd,id,targetRef,nearestEnd,replan,replan)
end

-- Detour-compatible form: requestMoveTarget(crowd,id,polyRef,pos).
-- Jess convenience form is retained: requestMoveTarget(crowd,id,pos[,extents]).
function Crowd.requestMoveTarget(crowd,id,targetOrRef,posOrExtents)
    if type(targetOrRef)=='number' then
        return setMoveTargetRequest(crowd,id,targetOrRef,posOrExtents,false,false)
    end
    return requestMoveTargetNearest(crowd,id,targetOrRef,posOrExtents,false)
end

-- Upstream keeps this private, but exposing it is useful for parity testing and
-- for advanced callers that explicitly drive Crowd replans.
function Crowd.requestMoveTargetReplan(crowd,id,targetOrRef,posOrExtents)
    if type(targetOrRef)=='number' then
        return setMoveTargetRequest(crowd,id,targetOrRef,posOrExtents,true,true)
    end
    return requestMoveTargetNearest(crowd,id,targetOrRef,posOrExtents,true)
end

function Crowd.requestMoveVelocity(crowd, id, velocity)
    local slot=id-1; if slot<0 or slot>=crowd.maxAgents or buffer.readu8(crowd.active,slot)==0 then return false,'invalid agent' end
    writeVec(crowd.targets,id,velocity); buffer.writeu32(crowd.targetRef,slot*4,0); buffer.writeu32(crowd.targetPathqRef,slot*4,0)
    buffer.writeu8(crowd.targetState,slot,Crowd.DT_CROWDAGENT_TARGET_VELOCITY); crowd.corners[id]=nil
    return true
end

function Crowd.resetMoveTarget(crowd, id)
    local slot = id - 1
    if slot < 0 or slot >= crowd.maxAgents or buffer.readu8(crowd.active, slot) == 0 then return false end
    buffer.writeu8(crowd.targetState, slot, Crowd.DT_CROWDAGENT_TARGET_NONE); buffer.writeu32(crowd.targetRef,slot*4,0); buffer.writeu32(crowd.targetPathqRef,slot*4,0)
    crowd.corners[id] = nil; writeVec(crowd.desired, id, vector.zero); writeVec(crowd.adjusted,id,vector.zero)
    return true
end

function Crowd.updateAgentParameters(crowd,id,params)
    local slot=id-1; if slot<0 or slot>=crowd.maxAgents or buffer.readu8(crowd.active,slot)==0 then return false end
    params=params or {}
    if params.radius~=nil then buffer.writef32(crowd.radius,slot*4,params.radius) end
    if params.height~=nil then buffer.writef32(crowd.height,slot*4,params.height) end
    if params.maxSpeed~=nil then buffer.writef32(crowd.maxSpeed,slot*4,params.maxSpeed) end
    if params.maxAcceleration~=nil then buffer.writef32(crowd.maxAcceleration,slot*4,params.maxAcceleration) end
    if params.collisionQueryRange~=nil then buffer.writef32(crowd.collisionQueryRange,slot*4,params.collisionQueryRange) end
    if params.pathOptimizationRange~=nil then buffer.writef32(crowd.pathOptimizationRange,slot*4,params.pathOptimizationRange) end
    if params.separationWeight~=nil then buffer.writef32(crowd.separationWeights,slot*4,params.separationWeight) end
    if params.updateFlags~=nil then buffer.writeu8(crowd.updateFlags,slot,params.updateFlags) end
    if params.obstacleAvoidanceType~=nil then buffer.writeu8(crowd.obstacleAvoidanceType,slot,math.clamp(params.obstacleAvoidanceType,0,7)) end
    if params.queryFilterType~=nil then buffer.writeu8(crowd.queryFilterType,slot,math.clamp(params.queryFilterType,0,15)) end
    if params.userData~=nil then crowd.userData[id]=params.userData end
    return true
end


function Crowd.setObstacleAvoidanceParams(crowd, index, params)
    if index < 0 or index >= Crowd.DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS or params == nil then return false end
    local base = ObstacleAvoidance.defaultParams()
    for k, v in params do base[k] = v end
    crowd.obstacleAvoidanceParams[index + 1] = base
    return true
end

function Crowd.getObstacleAvoidanceParams(crowd, index)
    if index < 0 or index >= Crowd.DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS then return nil end
    return crowd.obstacleAvoidanceParams[index + 1]
end


local function neighborIdOffset(slot, index0)
    return (slot * Crowd.DT_CROWDAGENT_MAX_NEIGHBOURS + index0) * 2
end

local function neighborDistOffset(slot, index0)
    return (slot * Crowd.DT_CROWDAGENT_MAX_NEIGHBOURS + index0) * 4
end

local function insertNeighbour(crowd, slot, otherSlot, dist2)
    local count = buffer.readu8(crowd.neighborCount, slot)
    local idBase = slot * Crowd.DT_CROWDAGENT_MAX_NEIGHBOURS * 2
    local distBase = slot * Crowd.DT_CROWDAGENT_MAX_NEIGHBOURS * 4
    local insertAt = count
    for i0 = 0, count - 1 do
        if dist2 < buffer.readf32(crowd.neighborDists, distBase + i0 * 4) then
            insertAt = i0
            break
        end
    end
    if insertAt >= Crowd.DT_CROWDAGENT_MAX_NEIGHBOURS then return end
    local newCount = if count < Crowd.DT_CROWDAGENT_MAX_NEIGHBOURS then count + 1 else Crowd.DT_CROWDAGENT_MAX_NEIGHBOURS
    for i0 = newCount - 1, insertAt + 1, -1 do
        buffer.writeu16(crowd.neighborIds, idBase + i0 * 2, buffer.readu16(crowd.neighborIds, idBase + (i0 - 1) * 2))
        buffer.writef32(crowd.neighborDists, distBase + i0 * 4, buffer.readf32(crowd.neighborDists, distBase + (i0 - 1) * 4))
    end
    buffer.writeu16(crowd.neighborIds, idBase + insertAt * 2, otherSlot)
    buffer.writef32(crowd.neighborDists, distBase + insertAt * 4, dist2)
    buffer.writeu8(crowd.neighborCount, slot, newCount)
end

local function updateNeighbours(crowd)
    ProximityGrid.clear(crowd.grid)
    for id = 1, crowd.maxAgents do
        local slot = id - 1
        buffer.writeu8(crowd.neighborCount, slot, 0)
        if buffer.readu8(crowd.active, slot) ~= 0 then
            local p = readVec(crowd.positions, id)
            local r = buffer.readf32(crowd.radius, slot * 4)
            ProximityGrid.addItem(crowd.grid, slot, p.X - r, p.Z - r, p.X + r, p.Z + r)
        end
    end

    for id = 1, crowd.maxAgents do
        local slot = id - 1
        if buffer.readu8(crowd.active, slot) == 0 or buffer.readu8(crowd.state, slot) ~= Crowd.DT_CROWDAGENT_STATE_WALKING then continue end
        local p = readVec(crowd.positions, id)
        local range = buffer.readf32(crowd.collisionQueryRange, slot * 4)
        local height = buffer.readf32(crowd.height, slot * 4)
        local ids = ProximityGrid.queryItems(crowd.grid, p.X - range, p.Z - range, p.X + range, p.Z + range, 32, crowd.gridQueryScratch)
        local range2 = range * range
        for _, otherSlot in ids do
            if otherSlot == slot or buffer.readu8(crowd.active, otherSlot) == 0 then continue end
            local otherId = otherSlot + 1
            local q = readVec(crowd.positions, otherId)
            if math.abs(p.Y - q.Y) >= (height + buffer.readf32(crowd.height, otherSlot * 4)) * 0.5 then continue end
            local dx, dz = p.X - q.X, p.Z - q.Z
            local dist2 = dx * dx + dz * dz
            if dist2 <= range2 then insertNeighbour(crowd, slot, otherSlot, dist2) end
        end
    end
end


local function insertPathQueueAgent(crowd, queue, id)
    local slot=id-1; local t=buffer.readf32(crowd.targetReplanTime,slot*4)
    local at=#queue+1
    for i=1,#queue do
        if t>=buffer.readf32(crowd.targetReplanTime,(queue[i]-1)*4) then at=i break end
    end
    table.insert(queue,at,id)
    if #queue>8 then table.remove(queue) end
end

local function removeTrackbacks(path)
    local i=2
    while i<#path do
        if path[i-1]==path[i+1] then
            table.remove(path,i+1);table.remove(path,i);i=math.max(2,i-1)
        else i+=1 end
    end
    return path
end

local function resetCorridor(crowd,id,ref,pos)
    local path=crowd.corridors[id]
    if path==nil then path={} crowd.corridors[id]=path else table.clear(path) end
    path[1]=ref
    crowd.corridorTargets[id]=pos
    buffer.writeu32(crowd.currentRef,(id-1)*4,ref or 0)
end

local function fixCorridorStart(crowd,id,safeRef,safePos)
    local path=crowd.corridors[id]
    if path==nil then path={} crowd.corridors[id]=path end
    local n=#path
    if n<3 and n>0 then
        local last=path[n]
        path[1],path[2],path[3]=safeRef,0,last
    elseif n>0 then
        path[1],path[2]=safeRef,0
    else
        path[1]=safeRef
    end
    writeVec(crowd.positions,id,safePos)
    buffer.writeu32(crowd.currentRef,(id-1)*4,safeRef)
end

local function checkPathValidity(crowd,dt)
    local CHECK_LOOKAHEAD=10
    local TARGET_REPLAN_DELAY=1.0
    for id=1,crowd.maxAgents do
        local slot=id-1
        if buffer.readu8(crowd.active,slot)==0 or buffer.readu8(crowd.state,slot)~=Crowd.DT_CROWDAGENT_STATE_WALKING then continue end
        local filter=agentFilter(crowd,slot)
        local replan=false
        local replanTime=buffer.readf32(crowd.targetReplanTime,slot*4)+dt
        buffer.writef32(crowd.targetReplanTime,slot*4,replanTime)
        local path=crowd.corridors[id] or {}
        local agentPos=readVec(crowd.positions,id)
        local agentRef=path[1] or buffer.readu32(crowd.currentRef,slot*4)

        if not Detour.isValidPolyRef(crowd.navmesh,agentRef,filter) then
            local nearestRef,nearest=Detour.findNearestPoly(crowd.navmesh,agentPos,crowd.queryExtents,filter)
            if nearestRef==nil then
                resetCorridor(crowd,id,0,agentPos)
                buffer.writeu8(crowd.partial,slot,0)
                LocalBoundary.reset(crowd.boundaries[id])
                buffer.writeu8(crowd.state,slot,Crowd.DT_CROWDAGENT_STATE_INVALID)
                continue
            end
            agentRef,agentPos=nearestRef,nearest
            fixCorridorStart(crowd,id,agentRef,agentPos)
            LocalBoundary.reset(crowd.boundaries[id])
            replan=true
            path=crowd.corridors[id]
        end

        local targetState=buffer.readu8(crowd.targetState,slot)
        if targetState==Crowd.DT_CROWDAGENT_TARGET_NONE or targetState==Crowd.DT_CROWDAGENT_TARGET_VELOCITY then continue end

        if targetState~=Crowd.DT_CROWDAGENT_TARGET_FAILED then
            local targetRef=buffer.readu32(crowd.targetRef,slot*4)
            if not Detour.isValidPolyRef(crowd.navmesh,targetRef,filter) then
                local nearestRef,nearest=Detour.findNearestPoly(crowd.navmesh,readVec(crowd.targets,id),crowd.queryExtents,filter)
                buffer.writeu32(crowd.targetRef,slot*4,nearestRef or 0)
                if nearest~=nil then writeVec(crowd.targets,id,nearest) end
                replan=true
            end
            if buffer.readu32(crowd.targetRef,slot*4)==0 then
                resetCorridor(crowd,id,agentRef,agentPos)
                buffer.writeu8(crowd.partial,slot,0)
                buffer.writeu8(crowd.targetState,slot,Crowd.DT_CROWDAGENT_TARGET_NONE)
                continue
            end
        end

        path=crowd.corridors[id] or {}
        local ncheck=math.min(#path,CHECK_LOOKAHEAD)
        if ncheck==0 then replan=true else
            for i=1,ncheck do if not Detour.isValidPolyRef(crowd.navmesh,path[i],filter) then replan=true break end end
        end
        targetState=buffer.readu8(crowd.targetState,slot)
        if targetState==Crowd.DT_CROWDAGENT_TARGET_VALID and replanTime>TARGET_REPLAN_DELAY and #path<CHECK_LOOKAHEAD and path[#path]~=buffer.readu32(crowd.targetRef,slot*4) then
            replan=true
        end
        if replan and buffer.readu8(crowd.targetState,slot)~=Crowd.DT_CROWDAGENT_TARGET_NONE then
            buffer.writeu32(crowd.targetPathqRef,slot*4,0)
            buffer.writeu8(crowd.targetReplan,slot,1)
            buffer.writeu8(crowd.targetState,slot,Crowd.DT_CROWDAGENT_TARGET_REQUESTING)
        end
    end
end

local function updateTopologyOptimization(crowd,dt)
    local bestId=nil
    local bestTime=-math.huge
    local threshold=crowd.topologyOptimizationInterval
    for id=1,crowd.maxAgents do
        local slot=id-1
        if buffer.readu8(crowd.active,slot)==0 or buffer.readu8(crowd.state,slot)~=Crowd.DT_CROWDAGENT_STATE_WALKING then continue end
        local targetState=buffer.readu8(crowd.targetState,slot)
        if targetState==Crowd.DT_CROWDAGENT_TARGET_NONE or targetState==Crowd.DT_CROWDAGENT_TARGET_VELOCITY then continue end
        if bit32.band(buffer.readu8(crowd.updateFlags,slot),Crowd.DT_CROWD_OPTIMIZE_TOPO)==0 then continue end
        local t=buffer.readf32(crowd.topologyOptTime,slot*4)+dt
        buffer.writef32(crowd.topologyOptTime,slot*4,t)
        if t>=threshold and t>=bestTime then bestId,bestTime=id,t end
    end
    if bestId~=nil then
        local slot=bestId-1
        local path=crowd.corridors[bestId]
        if path~=nil and #path>=3 then
            local c={path=path,position=readVec(crowd.positions,bestId),target=crowd.corridorTargets[bestId] or readVec(crowd.targets,bestId),maxPath=crowd.maxPath}
            PathCorridor.optimizePathTopology(c,crowd.navmesh,agentFilter(crowd,slot))
        end
        buffer.writef32(crowd.topologyOptTime,slot*4,0)
    end
end

local function updateCornersAndVisibility(crowd, debugId, debug)
    for id=1,crowd.maxAgents do
        local slot=id-1
        if buffer.readu8(crowd.active,slot)==0 or buffer.readu8(crowd.state,slot)~=Crowd.DT_CROWDAGENT_STATE_WALKING then continue end
        local targetState=buffer.readu8(crowd.targetState,slot)
        if targetState==Crowd.DT_CROWDAGENT_TARGET_NONE or targetState==Crowd.DT_CROWDAGENT_TARGET_VELOCITY then crowd.corners[id],crowd.cornerFlags[id],crowd.cornerRefs[id]=nil,nil,nil continue end
        local path=crowd.corridors[id]
        if path==nil or #path==0 then crowd.corners[id],crowd.cornerFlags[id],crowd.cornerRefs[id]=nil,nil,nil continue end
        local pos=readVec(crowd.positions,id); local target=crowd.corridorTargets[id] or readVec(crowd.targets,id)
        local points,flags,refs=Detour.findStraightPath(crowd.navmesh,pos,target,path)
        if points==nil then crowd.corners[id],crowd.cornerFlags[id],crowd.cornerRefs[id]=nil,nil,nil continue end
        local outP,outF,outR={},{},{}
        for i=1,#points do
            local flag=flags[i] or 0
            local dx,dz=points[i].X-pos.X,points[i].Z-pos.Z
            if #outP==0 and bit32.band(flag,Detour.DT_STRAIGHTPATH_OFFMESH_CONNECTION)==0 and dx*dx+dz*dz<=0.0001 then continue end
            outP[#outP+1],outF[#outF+1],outR[#outR+1]=points[i],flag,refs[i] or 0
            if bit32.band(flag,Detour.DT_STRAIGHTPATH_OFFMESH_CONNECTION)~=0 or #outP>=Crowd.DT_CROWDAGENT_MAX_CORNERS then break end
        end
        crowd.corners[id],crowd.cornerFlags[id],crowd.cornerRefs[id]=outP,outF,outR
        if bit32.band(buffer.readu8(crowd.updateFlags,slot),Crowd.DT_CROWD_OPTIMIZE_VIS)~=0 and #outP>0 then
            local nextPos=outP[math.min(2,#outP)]
            local c={path=path,position=pos,target=target,maxPath=crowd.maxPath}
            PathCorridor.optimizePathVisibility(c,nextPos,buffer.readf32(crowd.pathOptimizationRange,slot*4),crowd.navmesh,agentFilter(crowd,slot))
            if debugId == id and debug ~= nil then
                debug.optStart = pos
                debug.optEnd = nextPos
            end
        elseif debugId == id and debug ~= nil then
            debug.optStart = vector.zero
            debug.optEnd = vector.zero
        end
    end
end

local function triggerOffmeshConnections(crowd)
    for id=1,crowd.maxAgents do
        local slot=id-1
        if buffer.readu8(crowd.active,slot)==0 or buffer.readu8(crowd.state,slot)~=Crowd.DT_CROWDAGENT_STATE_WALKING then continue end
        local targetState=buffer.readu8(crowd.targetState,slot)
        if targetState==Crowd.DT_CROWDAGENT_TARGET_NONE or targetState==Crowd.DT_CROWDAGENT_TARGET_VELOCITY then continue end
        local corners,flags,refs=crowd.corners[id],crowd.cornerFlags[id],crowd.cornerRefs[id]
        if corners==nil or #corners==0 or bit32.band(flags[#flags] or 0,Detour.DT_STRAIGHTPATH_OFFMESH_CONNECTION)==0 then continue end
        local pos=readVec(crowd.positions,id); local corner=corners[#corners]; local dx,dz=pos.X-corner.X,pos.Z-corner.Z
        local trigger=buffer.readf32(crowd.radius,slot*4)*2.25
        if dx*dx+dz*dz>=trigger*trigger then continue end
        local offRef=refs[#refs] or 0; local path=crowd.corridors[id]; if offRef==0 or path==nil then continue end
        local c={path=path,position=pos,target=crowd.corridorTargets[id] or readVec(crowd.targets,id),maxPath=crowd.maxPath}
        local ok,connRefs,startPos,endPos=PathCorridor.moveOverOffmeshConnection(c,offRef,crowd.navmesh)
        if ok then
            local anim=crowd.animations[id]
            anim.active=true;anim.initPos=pos;anim.startPos=startPos;anim.endPos=endPos;anim.polyRef=connRefs[2];anim.t=0
            local speed=buffer.readf32(crowd.maxSpeed,slot*4);local ddx,ddz=endPos.X-startPos.X,endPos.Z-startPos.Z
            anim.tmax=if speed>0 then math.sqrt(ddx*ddx+ddz*ddz)/speed*0.5 else 0
            if #path>0 then buffer.writeu32(crowd.currentRef,slot*4,path[1]) end
            buffer.writeu8(crowd.state,slot,Crowd.DT_CROWDAGENT_STATE_OFFMESH)
            crowd.corners[id],crowd.cornerFlags[id],crowd.cornerRefs[id]=nil,nil,nil
            buffer.writeu8(crowd.neighborCount,slot,0)
        end
    end
end

local function updateMoveRequests(crowd)
    local waiting={}
    for id=1,crowd.maxAgents do
        local slot=id-1
        if buffer.readu8(crowd.active,slot)==0 or buffer.readu8(crowd.state,slot)==Crowd.DT_CROWDAGENT_STATE_INVALID then continue end
        local state=buffer.readu8(crowd.targetState,slot)
        if state==Crowd.DT_CROWDAGENT_TARGET_NONE or state==Crowd.DT_CROWDAGENT_TARGET_VELOCITY then continue end
        if state==Crowd.DT_CROWDAGENT_TARGET_REQUESTING then
            local path=crowd.corridors[id]
            if path==nil or #path==0 then path={buffer.readu32(crowd.currentRef,slot*4)} end
            local targetRef=buffer.readu32(crowd.targetRef,slot*4); local targetPos=readVec(crowd.targets,id); local pos=readVec(crowd.positions,id)
            local filter=agentFilter(crowd,slot)
            local status=crowd.navquery:initSlicedFindPath(path[1],targetRef,pos,targetPos,filter)
            if not Detour.statusFailed(status) then status=select(1,crowd.navquery:updateSlicedFindPath(20)) end
            local reqPath,finalStatus
            if buffer.readu8(crowd.targetReplan,slot)~=0 then reqPath,finalStatus=crowd.navquery:finalizeSlicedFindPathPartial(path) else reqPath,finalStatus=crowd.navquery:finalizeSlicedFindPath() end
            if reqPath~=nil and #reqPath>32 then while #reqPath>32 do table.remove(reqPath) end end
            local reqPos=nil
            if reqPath~=nil and #reqPath>0 and not Detour.statusFailed(finalStatus or Detour.DT_FAILURE) then
                if reqPath[#reqPath]~=targetRef then reqPos=select(1,Detour.closestPointOnPoly(crowd.navmesh,reqPath[#reqPath],targetPos)) else reqPos=targetPos end
            end
            if reqPath==nil or #reqPath==0 or reqPos==nil then reqPath={path[1]};reqPos=pos end
            crowd.corridors[id]=reqPath;crowd.corridorTargets[id]=reqPos;LocalBoundary.reset(crowd.boundaries[id]);buffer.writeu8(crowd.partial,slot,0)
            if reqPath[#reqPath]==targetRef then buffer.writeu8(crowd.targetState,slot,Crowd.DT_CROWDAGENT_TARGET_VALID);buffer.writef32(crowd.targetReplanTime,slot*4,0)
            else buffer.writeu8(crowd.targetState,slot,Crowd.DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE) end
            state=buffer.readu8(crowd.targetState,slot)
        end
        if state==Crowd.DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE then insertPathQueueAgent(crowd,waiting,id) end
    end

    for _,id in waiting do
        local slot=id-1;local path=crowd.corridors[id];local startRef=path[#path]
        local handle=PathQueue.request(crowd.pathQueue,startRef,buffer.readu32(crowd.targetRef,slot*4),crowd.corridorTargets[id],readVec(crowd.targets,id),agentFilter(crowd,slot))
        if handle~=PathQueue.DT_PATHQ_INVALID then buffer.writeu32(crowd.targetPathqRef,slot*4,handle);buffer.writeu8(crowd.targetState,slot,Crowd.DT_CROWDAGENT_TARGET_WAITING_FOR_PATH) end
    end
    PathQueue.update(crowd.pathQueue,100)

    for id=1,crowd.maxAgents do
        local slot=id-1
        if buffer.readu8(crowd.active,slot)==0 or buffer.readu8(crowd.targetState,slot)~=Crowd.DT_CROWDAGENT_TARGET_WAITING_FOR_PATH then continue end
        local handle=buffer.readu32(crowd.targetPathqRef,slot*4);local status=PathQueue.getRequestStatus(crowd.pathQueue,handle)
        if Detour.statusFailed(status) then
            buffer.writeu32(crowd.targetPathqRef,slot*4,0);buffer.writeu8(crowd.targetState,slot,if buffer.readu32(crowd.targetRef,slot*4)~=0 then Crowd.DT_CROWDAGENT_TARGET_REQUESTING else Crowd.DT_CROWDAGENT_TARGET_FAILED);buffer.writef32(crowd.targetReplanTime,slot*4,0)
        elseif Detour.statusSucceed(status) then
            local old=crowd.corridors[id];local res,resultStatus=PathQueue.getPathResult(crowd.pathQueue,handle,crowd.maxPath);local valid=res~=nil and #res>0 and old~=nil and #old>0 and old[#old]==res[1]
            buffer.writeu8(crowd.partial,slot,if Detour.statusDetail(resultStatus,Detour.DT_PARTIAL_RESULT) then 1 else 0)
            if valid then
                local merged={}
                for i=1,#old-1 do if #merged<crowd.maxPath then merged[#merged+1]=old[i] end end
                for i=1,#res do if #merged<crowd.maxPath then merged[#merged+1]=res[i] end end
                removeTrackbacks(merged)
                local targetPos=readVec(crowd.targets,id);local targetRef=buffer.readu32(crowd.targetRef,slot*4)
                if merged[#merged]~=targetRef then targetPos=select(1,Detour.closestPointOnPoly(crowd.navmesh,merged[#merged],targetPos));if targetPos==nil then valid=false end end
                if valid then crowd.corridors[id]=merged;crowd.corridorTargets[id]=targetPos;LocalBoundary.reset(crowd.boundaries[id]);buffer.writeu8(crowd.targetState,slot,Crowd.DT_CROWDAGENT_TARGET_VALID) end
            end
            if not valid then buffer.writeu8(crowd.targetState,slot,Crowd.DT_CROWDAGENT_TARGET_FAILED) end
            buffer.writeu32(crowd.targetPathqRef,slot*4,0);buffer.writef32(crowd.targetReplanTime,slot*4,0)
        end
    end
end

local function desiredForAgent(crowd, id)
    local slot=id-1; local state=buffer.readu8(crowd.targetState,slot)
    if state==Crowd.DT_CROWDAGENT_TARGET_VELOCITY then
        local v=readVec(crowd.targets,id)
        buffer.writef32(crowd.desiredSpeed,slot*4,vector.magnitude(v))
        return v
    end
    if state~=Crowd.DT_CROWDAGENT_TARGET_VALID then buffer.writef32(crowd.desiredSpeed,slot*4,0); return vector.zero end
    local corners=crowd.corners[id]
    if corners==nil or #corners==0 then buffer.writef32(crowd.desiredSpeed,slot*4,0); return vector.zero end
    local pos=readVec(crowd.positions,id)
    local dir=vector.zero
    if bit32.band(buffer.readu8(crowd.updateFlags,slot),Crowd.DT_CROWD_ANTICIPATE_TURNS)~=0 then
        local p0=corners[1];local p1=corners[math.min(2,#corners)]
        local d0=vector.create(p0.X-pos.X,0,p0.Z-pos.Z);local d1=vector.create(p1.X-pos.X,0,p1.Z-pos.Z)
        local len0=vector.magnitude(d0);local len1=vector.magnitude(d1)
        if len1>0.001 then d1/=len1 end
        dir=d0-d1*(len0*0.5)
        local dl=vector.magnitude(dir);if dl>1e-8 then dir/=dl else dir=vector.zero end
    else
        local p0=corners[1];dir=vector.create(p0.X-pos.X,0,p0.Z-pos.Z)
        local dl=vector.magnitude(dir);if dl>1e-8 then dir/=dl else dir=vector.zero end
    end
    local radius=buffer.readf32(crowd.radius,slot*4);local slowDownRadius=radius*2
    local distToGoal=slowDownRadius;local flags=crowd.cornerFlags[id]
    if flags~=nil and bit32.band(flags[#flags] or 0,Detour.DT_STRAIGHTPATH_END)~=0 then
        local goal=corners[#corners];local dx,dz=goal.X-pos.X,goal.Z-pos.Z
        distToGoal=math.min(math.sqrt(dx*dx+dz*dz),slowDownRadius)
    end
    local speedScale=if slowDownRadius>0 then distToGoal/slowDownRadius else 1
    local speed=buffer.readf32(crowd.maxSpeed,slot*4)
    buffer.writef32(crowd.desiredSpeed,slot*4,speed)
    return dir*(speed*speedScale)
end

function Crowd.update(crowd, dt, debug)
    if dt <= 0 then return end
    local debugId = nil
    if debug ~= nil then
        if debug.id ~= nil then
            debugId = debug.id
        elseif debug.idx ~= nil then
            -- Detour's debug.idx is the zero-based pool index; the Luau public agent id is one-based.
            debugId = debug.idx + 1
        end
    end
    checkPathValidity(crowd,dt)
    updateMoveRequests(crowd)
    updateTopologyOptimization(crowd,dt)

    -- Detour updates local neighbours/boundaries before corner finding and steering.
    updateNeighbours(crowd)

    for id = 1, crowd.maxAgents do
        local slot = id - 1
        if buffer.readu8(crowd.active, slot) == 0 or buffer.readu8(crowd.state, slot) ~= Crowd.DT_CROWDAGENT_STATE_WALKING then continue end
        local boundary = crowd.boundaries[id]
        local position = readVec(crowd.positions, id)
        local range = buffer.readf32(crowd.collisionQueryRange, slot * 4)
        local center = LocalBoundary.getCenter(boundary)
        local updateThreshold = range * 0.25
        local movedFar = center == nil
        if center ~= nil then
            local dx, dz = position.X-center.X, position.Z-center.Z
            movedFar = dx*dx+dz*dz > updateThreshold*updateThreshold
        end
        local filter = agentFilter(crowd, slot)
        if movedFar or not LocalBoundary.isValid(boundary, crowd.navmesh, filter) then
            LocalBoundary.update(boundary, crowd.navmesh, buffer.readu32(crowd.currentRef, slot*4), position, range, filter)
        end
    end

    updateCornersAndVisibility(crowd, debugId, debug)
    triggerOffmeshConnections(crowd)

    -- Desired velocity from the path corridor.
    for id = 1, crowd.maxAgents do
        local slot = id - 1
        if buffer.readu8(crowd.active, slot) ~= 0 and buffer.readu8(crowd.state,slot)==Crowd.DT_CROWDAGENT_STATE_WALKING then
            writeVec(crowd.desired, id, desiredForAgent(crowd, id))
        else
            writeVec(crowd.desired,id,vector.zero);buffer.writef32(crowd.desiredSpeed,slot*4,0)
        end
    end

    -- Per-agent separation, following DetourCrowd's collisionQueryRange/weight formulation.
    for id = 1, crowd.maxAgents do
        local slot = id - 1
        if buffer.readu8(crowd.active, slot) == 0 then continue end
        if bit32.band(buffer.readu8(crowd.updateFlags, slot), Crowd.DT_CROWD_SEPARATION) == 0 then continue end
        local separationDist = buffer.readf32(crowd.collisionQueryRange, slot * 4)
        if separationDist <= 0 then continue end
        local invSeparationDist = 1 / separationDist
        local separationWeight = buffer.readf32(crowd.separationWeights, slot * 4)
        local pos = readVec(crowd.positions, id)
        local disp = vector.zero
        local weightCount = 0
        local count = buffer.readu8(crowd.neighborCount, slot)
        local neighborBase = slot * Crowd.DT_CROWDAGENT_MAX_NEIGHBOURS * 2
        local separationDist2 = separationDist * separationDist
        for i0 = 0, count - 1 do
            local otherSlot = buffer.readu16(crowd.neighborIds, neighborBase + i0 * 2)
            local other = readVec(crowd.positions, otherSlot + 1)
            local dx, dz = pos.X - other.X, pos.Z - other.Z
            local dist2 = dx * dx + dz * dz
            if dist2 < 0.00001 or dist2 > separationDist2 then continue end
            local dist = math.sqrt(dist2)
            local weight = separationWeight * (1 - (dist * invSeparationDist) ^ 2)
            local scale = weight / dist
            disp += vector.create(dx * scale, 0, dz * scale)
            weightCount += 1
        end
        if weightCount > 0 then
            local desired = readVec(crowd.desired, id) + disp / weightCount
            local desiredSpeed = buffer.readf32(crowd.desiredSpeed, slot * 4)
            local speed2 = desired.X * desired.X + desired.Y * desired.Y + desired.Z * desired.Z
            local desired2 = desiredSpeed * desiredSpeed
            -- Preserve upstream DetourCrowd's exact clamp expression.
            if speed2 > desired2 and speed2 > 1e-8 then desired *= desired2 / speed2 end
            writeVec(crowd.desired, id, desired)
        end
    end

    -- Velocity planning. dvel remains the steering request; adjusted is DetourCrowd's nvel.
    crowd.velocitySampleCount = 0
    for id = 1, crowd.maxAgents do
        local slot = id - 1
        if buffer.readu8(crowd.active, slot) == 0 or buffer.readu8(crowd.state, slot) ~= Crowd.DT_CROWDAGENT_STATE_WALKING then continue end
        local desired = readVec(crowd.desired, id)
        local flags = buffer.readu8(crowd.updateFlags, slot)
        if bit32.band(flags, Crowd.DT_CROWD_OBSTACLE_AVOIDANCE) == 0 then
            writeVec(crowd.adjusted, id, desired)
            continue
        end

        local query = crowd.obstacleQuery
        ObstacleAvoidance.reset(query)
        local nneis = buffer.readu8(crowd.neighborCount, slot)
        for i0 = 0, nneis - 1 do
            local otherSlot = buffer.readu16(crowd.neighborIds, neighborIdOffset(slot, i0))
            local otherId = otherSlot + 1
            ObstacleAvoidance.addCircle(query,
                readVec(crowd.positions, otherId), buffer.readf32(crowd.radius, otherSlot * 4),
                readVec(crowd.velocities, otherId), readVec(crowd.desired, otherId))
        end

        local boundary = crowd.boundaries[id]
        if boundary ~= nil then
            local pos = readVec(crowd.positions, id)
            for i0 = 0, LocalBoundary.getSegmentCount(boundary) - 1 do
                local a, b = LocalBoundary.getSegment(boundary, i0)
                -- Exact dtTriArea2D(pos,a,b) sign used by DetourCrowd.  Detour's helper is
                -- intentionally the negative of the conventional XZ cross-product orientation.
                local abx, abz = a.X-pos.X, a.Z-pos.Z
                local acx, acz = b.X-pos.X, b.Z-pos.Z
                if acx*abz-abx*acz >= 0 then ObstacleAvoidance.addSegment(query, a, b) end
            end
        end

        local params = crowd.obstacleAvoidanceParams[buffer.readu8(crowd.obstacleAvoidanceType, slot) + 1]
        local vod = if debugId == id and debug ~= nil then debug.vod else nil
        local safe, samples = ObstacleAvoidance.sampleVelocityAdaptive(query,
            readVec(crowd.positions, id), buffer.readf32(crowd.radius, slot * 4), buffer.readf32(crowd.desiredSpeed, slot * 4),
            readVec(crowd.velocities, id), desired, params, vod)
        crowd.velocitySampleCount += samples
        writeVec(crowd.adjusted, id, safe)
    end

    -- Integrate with DetourCrowd's acceleration-only dynamic constraint.  Direct
    -- velocity requests are intentionally not capped by params.maxSpeed.
    for id = 1, crowd.maxAgents do
        local slot = id - 1
        if buffer.readu8(crowd.active, slot) == 0 or buffer.readu8(crowd.state,slot)~=Crowd.DT_CROWDAGENT_STATE_WALKING then continue end
        local velocity = readVec(crowd.velocities, id)
        local desired = readVec(crowd.adjusted, id)
        local delta = desired - velocity
        local deltaMag = vector.magnitude(delta)
        local maxDelta = buffer.readf32(crowd.maxAcceleration, slot * 4) * dt
        if deltaMag > maxDelta and deltaMag > 1e-8 then delta = delta / deltaMag * maxDelta end
        velocity += delta
        local position=readVec(crowd.positions,id)
        if vector.magnitude(velocity)>0.0001 then position+=velocity*dt else velocity=vector.zero end
        writeVec(crowd.positions,id,position);writeVec(crowd.velocities,id,velocity)
    end

    -- Four DetourCrowd overlap-resolution passes before corridor constraint.
    local COLLISION_RESOLVE_FACTOR=0.7
    for _=1,4 do
        for id=1,crowd.maxAgents do
            local slot=id-1
            if buffer.readu8(crowd.active,slot)==0 or buffer.readu8(crowd.state,slot)~=Crowd.DT_CROWDAGENT_STATE_WALKING then continue end
            local pos=readVec(crowd.positions,id);local disp=vector.zero;local w=0
            local nneis=buffer.readu8(crowd.neighborCount,slot)
            local neighborBase=slot*Crowd.DT_CROWDAGENT_MAX_NEIGHBOURS*2
            local radius0=buffer.readf32(crowd.radius,slot*4)
            for i0=0,nneis-1 do
                local otherSlot=buffer.readu16(crowd.neighborIds,neighborBase+i0*2);local otherId=otherSlot+1
                local other=readVec(crowd.positions,otherId)
                local dx,dz=pos.X-other.X,pos.Z-other.Z
                local dist2=dx*dx+dz*dz
                local r=radius0+buffer.readf32(crowd.radius,otherSlot*4)
                if dist2>r*r then continue end
                local dist=math.sqrt(dist2);local pen=r-dist
                if dist<0.0001 then
                    local dvel=readVec(crowd.desired,id)
                    if slot>otherSlot then dx,dz=-dvel.Z,dvel.X else dx,dz=dvel.Z,-dvel.X end
                    pen=0.01
                else
                    pen=(1/dist)*(pen*0.5)*COLLISION_RESOLVE_FACTOR
                end
                disp+=vector.create(dx*pen,0,dz*pen);w+=1
            end
            if w>0.0001 then disp/=w end
            writeVec(crowd.displacement,id,disp)
        end
        for id=1,crowd.maxAgents do
            local slot=id-1
            if buffer.readu8(crowd.active,slot)~=0 and buffer.readu8(crowd.state,slot)==Crowd.DT_CROWDAGENT_STATE_WALKING then
                writeVec(crowd.positions,id,readVec(crowd.positions,id)+readVec(crowd.displacement,id))
            end
        end
    end

    -- Move the resolved position along the current corridor and recover its height.
    for id=1,crowd.maxAgents do
        local slot=id-1
        if buffer.readu8(crowd.active,slot)==0 or buffer.readu8(crowd.state,slot)~=Crowd.DT_CROWDAGENT_STATE_WALKING then continue end
        local position=readVec(crowd.positions,id)
        local corridor=crowd.corridors[id]
        if crowd.constrainToNavmesh and corridor~=nil and #corridor>0 then
            local firstRef=corridor[1]
            local oldPos=position-readVec(crowd.velocities,id)*dt
            local moved,visited=Detour.moveAlongSurface(crowd.navmesh,firstRef,oldPos,position,agentFilter(crowd,slot))
            if moved~=nil then
                position=moved
                if visited~=nil and #visited>0 then PathCorridor.mergeCorridorStartMoved(corridor,visited,crowd.maxPath) end
                if #corridor>0 then
                    buffer.writeu32(crowd.currentRef,slot*4,corridor[1])
                    local h=Detour.getPolyHeight(crowd.navmesh,corridor[1],position);if h~=nil then position=vector.create(position.X,h,position.Z) end
                end
            end
        end
        writeVec(crowd.positions,id,position)
        local targetState=buffer.readu8(crowd.targetState,slot)
        if targetState==Crowd.DT_CROWDAGENT_TARGET_NONE or targetState==Crowd.DT_CROWDAGENT_TARGET_VELOCITY then
            local ref=(corridor~=nil and corridor[1]) or buffer.readu32(crowd.currentRef,slot*4)
            resetCorridor(crowd,id,ref,position);buffer.writeu8(crowd.partial,slot,0)
        end
    end

    -- Animate agents currently traversing an off-mesh connection.
    for id=1,crowd.maxAgents do
        local slot=id-1;local anim=crowd.animations[id]
        if buffer.readu8(crowd.active,slot)==0 or anim==nil or not anim.active then continue end
        anim.t+=dt
        if anim.t>anim.tmax then
            anim.active=false;buffer.writeu8(crowd.state,slot,Crowd.DT_CROWDAGENT_STATE_WALKING)
            writeVec(crowd.positions,id,anim.endPos)
        else
            local ta=anim.tmax*0.15;local p
            if anim.t<ta and ta>0 then
                local u=math.clamp(anim.t/ta,0,1);p=anim.initPos+(anim.startPos-anim.initPos)*u
            else
                local denom=anim.tmax-ta;local u=if denom>0 then math.clamp((anim.t-ta)/denom,0,1) else 1
                p=anim.startPos+(anim.endPos-anim.startPos)*u
            end
            writeVec(crowd.positions,id,p)
        end
        writeVec(crowd.velocities,id,vector.zero);writeVec(crowd.desired,id,vector.zero);writeVec(crowd.adjusted,id,vector.zero)
    end
end

function Crowd.getAgentNeighbours(crowd, id)
    local slot = id - 1
    if slot < 0 or slot >= crowd.maxAgents or buffer.readu8(crowd.active, slot) == 0 then return nil end
    local count = buffer.readu8(crowd.neighborCount, slot)
    local out = table.create(count)
    for i0 = 0, count - 1 do
        out[i0 + 1] = {
            id = buffer.readu16(crowd.neighborIds, neighborIdOffset(slot, i0)) + 1,
            dist = buffer.readf32(crowd.neighborDists, neighborDistOffset(slot, i0)),
        }
    end
    return out
end


function Crowd.getAgent(crowd, id)
    local slot = id - 1
    if slot < 0 or slot >= crowd.maxAgents or buffer.readu8(crowd.active, slot) == 0 then return nil end
    return {
        id = id,
        active = true,
        position = readVec(crowd.positions, id),
        displacement = readVec(crowd.displacement, id),
        velocity = readVec(crowd.velocities, id),
        desiredVelocity = readVec(crowd.desired, id),
        adjustedVelocity = readVec(crowd.adjusted, id),
        desiredSpeed = buffer.readf32(crowd.desiredSpeed, slot*4),
        target = readVec(crowd.targets, id),
        state = buffer.readu8(crowd.state,slot),
        targetState = buffer.readu8(crowd.targetState, slot),
        currentRef = buffer.readu32(crowd.currentRef,slot*4),
        targetRef = buffer.readu32(crowd.targetRef,slot*4),
        targetPathqRef = buffer.readu32(crowd.targetPathqRef,slot*4),
        targetReplan = buffer.readu8(crowd.targetReplan,slot)~=0,
        targetReplanTime = buffer.readf32(crowd.targetReplanTime,slot*4),
        topologyOptTime = buffer.readf32(crowd.topologyOptTime,slot*4),
        partial = buffer.readu8(crowd.partial,slot)~=0,
        corridor = crowd.corridors[id],
        corners = crowd.corners[id],
        cornerFlags = crowd.cornerFlags[id],
        cornerRefs = crowd.cornerRefs[id],
        boundary = crowd.boundaries[id],
        neighbours = Crowd.getAgentNeighbours(crowd, id),
        radius = buffer.readf32(crowd.radius, slot * 4),
        height = buffer.readf32(crowd.height, slot * 4),
        maxSpeed = buffer.readf32(crowd.maxSpeed, slot * 4),
        maxAcceleration = buffer.readf32(crowd.maxAcceleration, slot * 4),
        collisionQueryRange = buffer.readf32(crowd.collisionQueryRange,slot*4),
        pathOptimizationRange = buffer.readf32(crowd.pathOptimizationRange,slot*4),
        separationWeight = buffer.readf32(crowd.separationWeights,slot*4),
        updateFlags = buffer.readu8(crowd.updateFlags,slot),
        obstacleAvoidanceType = buffer.readu8(crowd.obstacleAvoidanceType,slot),
        queryFilterType = buffer.readu8(crowd.queryFilterType,slot),
        userData = crowd.userData[id],
        params = {
            radius = buffer.readf32(crowd.radius, slot * 4),
            height = buffer.readf32(crowd.height, slot * 4),
            maxAcceleration = buffer.readf32(crowd.maxAcceleration, slot * 4),
            maxSpeed = buffer.readf32(crowd.maxSpeed, slot * 4),
            collisionQueryRange = buffer.readf32(crowd.collisionQueryRange, slot * 4),
            pathOptimizationRange = buffer.readf32(crowd.pathOptimizationRange, slot * 4),
            separationWeight = buffer.readf32(crowd.separationWeights, slot * 4),
            updateFlags = buffer.readu8(crowd.updateFlags, slot),
            obstacleAvoidanceType = buffer.readu8(crowd.obstacleAvoidanceType, slot),
            queryFilterType = buffer.readu8(crowd.queryFilterType, slot),
            userData = crowd.userData[id],
        },
    }
end

function Crowd.getAgentCount(crowd) return crowd.maxAgents end

function Crowd.getActiveAgents(crowd, maxAgents)
    local limit = math.min(crowd.maxAgents, math.max(0, math.floor(maxAgents or crowd.maxAgents)))
    local out={}
    for id=1,crowd.maxAgents do
        if buffer.readu8(crowd.active,id-1)~=0 then
            out[#out+1]=Crowd.getAgent(crowd,id)
            if #out>=limit then break end
        end
    end
    return out
end

function Crowd.getEditableAgent(crowd,id) return Crowd.getAgent(crowd,id) end

function Crowd.getFilter(crowd,index)
    index=index or 0; if index<0 or index>=Crowd.DT_CROWD_MAX_QUERY_FILTER_TYPE then return nil end
    return crowd.filters[index+1]
end

function Crowd.getEditableFilter(crowd,index) return Crowd.getFilter(crowd,index) end
function Crowd.getPathQueue(crowd) return crowd.pathQueue end
function Crowd.getNavMeshQuery(crowd) return crowd.navquery end
function Crowd.getQueryHalfExtents(crowd) return crowd.queryExtents end
function Crowd.getQueryExtents(crowd) return crowd.queryExtents end
function Crowd.getGrid(crowd) return crowd.grid end
function Crowd.getVelocitySampleCount(crowd) return crowd.velocitySampleCount end

return table.freeze(Crowd)
