--!native

-- Altered pure-Luau port of DetourObstacleAvoidance.
-- The query keeps a fixed reusable obstacle pool and implements both grid and adaptive samplers.

local Avoidance = {}

Avoidance.DT_MAX_PATTERN_DIVS = 32
Avoidance.DT_MAX_PATTERN_RINGS = 4

local function dot2(a: vector, b: vector) return a.X*b.X + a.Z*b.Z end
local function dist2D(a: vector, b: vector)
    local dx, dz = a.X-b.X, a.Z-b.Z
    return math.sqrt(dx*dx + dz*dz)
end
local function normalize2(v: vector)
    local d = math.sqrt(v.X*v.X + v.Z*v.Z)
    if d <= 1e-12 then return vector.zero end
    return vector.create(v.X/d, 0, v.Z/d)
end
local function triArea2D(a: vector, b: vector, c: vector)
    local abx, abz = b.X-a.X, b.Z-a.Z
    local acx, acz = c.X-a.X, c.Z-a.Z
    -- Detour's dtTriArea2D uses the opposite sign from the conventional xz cross product:
    -- ac.x * ab.z - ab.x * ac.z.  The sign selects the preferred RVO side.
    return acx*abz - abx*acz
end
local function closestPtSeg2DSqr(p: vector, a: vector, b: vector)
    local dx, dz = b.X-a.X, b.Z-a.Z
    local d = dx*dx+dz*dz
    local t = 0
    if d > 1e-12 then t = math.clamp(((p.X-a.X)*dx+(p.Z-a.Z)*dz)/d, 0, 1) end
    local qx, qz = a.X+dx*t, a.Z+dz*t
    local ex, ez = p.X-qx, p.Z-qz
    return ex*ex+ez*ez, t
end

local function sweepCircleCircle(c0: vector, r0, v: vector, c1: vector, r1)
    local sx, sz = c1.X-c0.X, c1.Z-c0.Z
    local r = r0+r1
    local c = sx*sx+sz*sz-r*r
    local a = v.X*v.X+v.Z*v.Z
    if a < 0.0001 then return nil end
    local b = v.X*sx+v.Z*sz
    local d = b*b-a*c
    if d < 0 then return nil end
    local inv = 1/a
    local rd = math.sqrt(d)
    return (b-rd)*inv, (b+rd)*inv
end

local function isectRaySeg(ap: vector, u: vector, bp: vector, bq: vector)
    local vx, vz = bq.X-bp.X, bq.Z-bp.Z
    local wx, wz = ap.X-bp.X, ap.Z-bp.Z
    local d = u.X*vz-u.Z*vx
    if math.abs(d) < 1e-6 then return nil end
    local inv = 1/d
    local t = (vx*wz-vz*wx)*inv
    if t < 0 or t > 1 then return nil end
    local s = (u.X*wz-u.Z*wx)*inv
    if s < 0 or s > 1 then return nil end
    return t
end

function Avoidance.defaultParams()
    return {
        velBias = 0.4,
        weightDesVel = 2.0,
        weightCurVel = 0.75,
        weightSide = 0.75,
        weightToi = 2.5,
        horizTime = 2.5,
        gridSize = 33,
        adaptiveDivs = 7,
        adaptiveRings = 2,
        adaptiveDepth = 5,
    }
end

function Avoidance.copyParams(params)
    return {
        velBias=params.velBias, weightDesVel=params.weightDesVel,
        weightCurVel=params.weightCurVel, weightSide=params.weightSide,
        weightToi=params.weightToi, horizTime=params.horizTime,
        gridSize=params.gridSize, adaptiveDivs=params.adaptiveDivs,
        adaptiveRings=params.adaptiveRings, adaptiveDepth=params.adaptiveDepth,
    }
end

function Avoidance.newDebugData(maxSamples)
    local d={maxSamples=maxSamples or 2048,count=0,vel={},ssize={},pen={},vpen={},vcpen={},spen={},tpen={}}
    return d
end
function Avoidance.resetDebug(debug) debug.count=0 return debug end
function Avoidance.addDebugSample(debug, vel, ssize, pen, vpen, vcpen, spen, tpen)
    if debug==nil or debug.count>=debug.maxSamples then return end
    local i=debug.count+1; debug.count=i
    debug.vel[i],debug.ssize[i],debug.pen[i]=vel,ssize,pen
    debug.vpen[i],debug.vcpen[i],debug.spen[i],debug.tpen[i]=vpen,vcpen,spen,tpen
end
local function normalizeArray(arr,n)
    if n==0 then return end
    local lo,hi=math.huge,-math.huge
    for i=1,n do lo=math.min(lo,arr[i]); hi=math.max(hi,arr[i]) end
    local scale=if hi-lo>0.001 then 1/(hi-lo) else 1
    for i=1,n do arr[i]=math.clamp((arr[i]-lo)*scale,0,1) end
end
function Avoidance.normalizeDebugSamples(debug)
    normalizeArray(debug.pen,debug.count); normalizeArray(debug.vpen,debug.count); normalizeArray(debug.vcpen,debug.count)
    normalizeArray(debug.spen,debug.count); normalizeArray(debug.tpen,debug.count)
    return debug
end
function Avoidance.getSampleCount(debug) return debug.count end
function Avoidance.getSampleVelocity(debug,index0) if index0<0 or index0>=debug.count then return nil end return debug.vel[index0+1] end
function Avoidance.getSampleSize(debug,index0) if index0<0 or index0>=debug.count then return nil end return debug.ssize[index0+1] end
function Avoidance.getSamplePenalty(debug,index0) if index0<0 or index0>=debug.count then return nil end return debug.pen[index0+1] end
function Avoidance.getSampleDesiredVelocityPenalty(debug,index0) if index0<0 or index0>=debug.count then return nil end return debug.vpen[index0+1] end
function Avoidance.getSampleCurrentVelocityPenalty(debug,index0) if index0<0 or index0>=debug.count then return nil end return debug.vcpen[index0+1] end
function Avoidance.getSamplePreferredSidePenalty(debug,index0) if index0<0 or index0>=debug.count then return nil end return debug.spen[index0+1] end
function Avoidance.getSampleCollisionTimePenalty(debug,index0) if index0<0 or index0>=debug.count then return nil end return debug.tpen[index0+1] end

function Avoidance.newQuery(maxCircles,maxSegments)
    maxCircles=maxCircles or 6; maxSegments=maxSegments or 8
    local circles=table.create(maxCircles)
    for i=1,maxCircles do circles[i]={p=vector.zero,rad=0,vel=vector.zero,dvel=vector.zero,dp=vector.zero,np=vector.zero} end
    local segments=table.create(maxSegments)
    for i=1,maxSegments do segments[i]={p=vector.zero,q=vector.zero,touch=false} end
    return {maxCircles=maxCircles,maxSegments=maxSegments,ncircles=0,nsegments=0,circles=circles,segments=segments}
end

function Avoidance.reset(query) query.ncircles=0; query.nsegments=0; return query end
function Avoidance.addCircle(query,pos,rad,vel,dvel)
    if query.ncircles>=query.maxCircles then return false end
    local i=query.ncircles+1; query.ncircles=i; local c=query.circles[i]
    c.p,c.rad,c.vel,c.dvel=pos,rad,vel,dvel
    return true
end
function Avoidance.addSegment(query,p,q)
    if query.nsegments>=query.maxSegments then return false end
    local i=query.nsegments+1; query.nsegments=i; local s=query.segments[i]
    s.p,s.q,s.touch=p,q,false
    return true
end
function Avoidance.getObstacleCircleCount(query) return query.ncircles end
function Avoidance.getObstacleSegmentCount(query) return query.nsegments end
function Avoidance.getObstacleCircle(query,index) if index<0 or index>=query.ncircles then return nil end return query.circles[index+1] end
function Avoidance.getObstacleSegment(query,index) if index<0 or index>=query.nsegments then return nil end return query.segments[index+1] end

local function prepare(query,pos,dvel)
    for i=1,query.ncircles do
        local c=query.circles[i]
        c.dp=normalize2(c.p-pos)
        local dv=c.dvel-dvel
        local a=triArea2D(vector.zero,c.dp,dv)
        c.np=if a<0.01 then vector.create(-c.dp.Z,0,c.dp.X) else vector.create(c.dp.Z,0,-c.dp.X)
    end
    for i=1,query.nsegments do
        local s=query.segments[i]
        local d2=closestPtSeg2DSqr(pos,s.p,s.q)
        s.touch=d2<0.0001
    end
end

local function processSample(query,vcand,cs,pos,rad,vel,dvel,minPenalty,params,invHorizTime,invVmax,debug)
    local vpen=params.weightDesVel*(dist2D(vcand,dvel)*invVmax)
    local vcpen=params.weightCurVel*(dist2D(vcand,vel)*invVmax)
    local minPen=minPenalty-vpen-vcpen
    local threshold
    if minPen==0 then threshold=math.huge else threshold=(params.weightToi/minPen-0.1)*params.horizTime end
    if threshold-params.horizTime > -1e-7 then return minPenalty end
    local tmin=params.horizTime
    local side,nside=0,0
    for i=1,query.ncircles do
        local c=query.circles[i]
        local vab=vcand*2-vel-c.vel
        side+=math.clamp(math.min(dot2(c.dp,vab)*0.5+0.5,dot2(c.np,vab)*2),0,1); nside+=1
        local htmin,htmax=sweepCircleCircle(pos,rad,vab,c.p,c.rad)
        if htmin~=nil then
            if htmin<0 and htmax>0 then htmin=-htmin*0.5 end
            if htmin>=0 and htmin<tmin then
                tmin=htmin
                if tmin<threshold then return minPenalty end
            end
        end
    end
    for i=1,query.nsegments do
        local s=query.segments[i]
        local htmin
        if s.touch then
            local sd=s.q-s.p; local sn=vector.create(-sd.Z,0,sd.X)
            if dot2(sn,vcand)<0 then continue end
            htmin=0
        else
            htmin=isectRaySeg(pos,vcand,s.p,s.q)
            if htmin==nil then continue end
        end
        htmin*=2
        if htmin<tmin then
            tmin=htmin
            if tmin<threshold then return minPenalty end
        end
    end
    if nside>0 then side/=nside end
    local spen=params.weightSide*side
    local tpen=params.weightToi*(1/(0.1+tmin*invHorizTime))
    local penalty=vpen+vcpen+spen+tpen
    Avoidance.addDebugSample(debug,vcand,cs,penalty,vpen,vcpen,spen,tpen)
    return penalty
end

function Avoidance.sampleVelocityGrid(query,pos,rad,vmax,vel,dvel: vector,params,debug)
    params=params or Avoidance.defaultParams(); prepare(query,pos,dvel)
    local invHorizTime=1/params.horizTime
    local invVmax=if vmax>0 then 1/vmax else math.huge
    if debug then Avoidance.resetDebug(debug) end
    local gridSize=math.max(2,math.floor(params.gridSize))
    local cvx,cvz=dvel.X*params.velBias,dvel.Z*params.velBias
    local cs=vmax*2*(1-params.velBias)/(gridSize-1)
    local half=(gridSize-1)*cs*0.5
    local best=vector.zero; local minPenalty=math.huge; local ns=0
    local limit=(vmax+cs/2)^2
    for y0=0,gridSize-1 do
        for x0=0,gridSize-1 do
            local vc=vector.create(cvx+x0*cs-half,0,cvz+y0*cs-half)
            if vc.X*vc.X+vc.Z*vc.Z>limit then continue end
            local pen=processSample(query,vc,cs,pos,rad,vel,dvel,minPenalty,params,invHorizTime,invVmax,debug); ns+=1
            if pen<minPenalty then minPenalty,best=pen,vc end
        end
    end
    return best,ns
end

local function rotate2(v: vector,ang)
    local c,s=math.cos(ang),math.sin(ang)
    return vector.create(v.X*c-v.Z*s,0,v.X*s+v.Z*c)
end

function Avoidance.sampleVelocityAdaptive(query,pos,rad,vmax,vel,dvel: vector,params,debug)
    params=params or Avoidance.defaultParams(); prepare(query,pos,dvel)
    local invHorizTime=1/params.horizTime
    local invVmax=if vmax>0 then 1/vmax else math.huge
    if debug then Avoidance.resetDebug(debug) end
    local nd=math.clamp(math.floor(params.adaptiveDivs),1,Avoidance.DT_MAX_PATTERN_DIVS)
    local nr=math.clamp(math.floor(params.adaptiveRings),1,Avoidance.DT_MAX_PATTERN_RINGS)
    local depth=math.max(1,math.floor(params.adaptiveDepth))
    local da=math.pi*2/nd
    local ca,sa=math.cos(da),math.sin(da)
    local d0=normalize2(dvel); local d1=rotate2(d0,da*0.5)
    local patX,patZ={0},{0}
    local npat=1
    for j0=0,nr-1 do
        local r=(nr-j0)/nr
        local base=if j0%2==0 then d0 else d1
        npat+=1; patX[npat],patZ[npat]=base.X*r,base.Z*r
        local l1x,l1z,l2x,l2z=patX[npat],patZ[npat],patX[npat],patZ[npat]
        local i0=1
        while i0<nd-1 do
            local r1x,r1z=l1x*ca+l1z*sa,-l1x*sa+l1z*ca
            local r2x,r2z=l2x*ca-l2z*sa,l2x*sa+l2z*ca
            npat+=1; patX[npat],patZ[npat]=r1x,r1z
            npat+=1; patX[npat],patZ[npat]=r2x,r2z
            l1x,l1z,l2x,l2z=r1x,r1z,r2x,r2z
            i0+=2
        end
        if nd%2==0 then
            local r2x,r2z=l2x*ca-l2z*sa,l2x*sa+l2z*ca
            npat+=1; patX[npat],patZ[npat]=r2x,r2z
        end
    end
    local cr=vmax*(1-params.velBias)
    local res=vector.create(dvel.X*params.velBias,0,dvel.Z*params.velBias)
    local ns=0
    local limit=(vmax+0.001)^2
    for _=1,depth do
        local minPenalty=math.huge; local best=vector.zero
        for i=1,npat do
            local vc=vector.create(res.X+patX[i]*cr,0,res.Z+patZ[i]*cr)
            if vc.X*vc.X+vc.Z*vc.Z>limit then continue end
            local pen=processSample(query,vc,cr/10,pos,rad,vel,dvel,minPenalty,params,invHorizTime,invVmax,debug); ns+=1
            if pen<minPenalty then minPenalty,best=pen,vc end
        end
        res=best; cr*=0.5
    end
    return res,ns
end

return table.freeze(Avoidance)
