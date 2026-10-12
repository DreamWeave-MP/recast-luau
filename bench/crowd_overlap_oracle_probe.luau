--!native
local Builder=require('@recast/navmesh-builder')
local Detour=require('@recast/detour')
local Crowd=require('@recast/crowd')
local N,nvp=4,4
local vw,vh=N+1,N+1
local verts=buffer.create(vw*vh*3*2)
local function vi(x,z)return z*vw+x end
for z=0,N do for x=0,N do local i=vi(x,z)*6;buffer.writeu16(verts,i,x);buffer.writeu16(verts,i+2,0);buffer.writeu16(verts,i+4,z) end end
local np=N*N
local polys=buffer.create(np*nvp*2*2);buffer.fill(polys,0,0xff)
local flags=buffer.create(np*2);local areas=buffer.create(np)
local function pi(x,z)return z*N+x end
for z=0,N-1 do for x=0,N-1 do
 local p=pi(x,z);local b=p*nvp*4
 local vs={vi(x,z),vi(x+1,z),vi(x+1,z+1),vi(x,z+1)}
 for j=0,3 do buffer.writeu16(polys,b+j*2,vs[j+1]) end
 if z>0 then buffer.writeu16(polys,b+(nvp+0)*2,pi(x,z-1)) end
 if x<N-1 then buffer.writeu16(polys,b+(nvp+1)*2,pi(x+1,z)) end
 if z<N-1 then buffer.writeu16(polys,b+(nvp+2)*2,pi(x,z+1)) end
 if x>0 then buffer.writeu16(polys,b+(nvp+3)*2,pi(x-1,z)) end
 buffer.writeu16(flags,p*2,1)
end end
local data=assert(Builder.createNavMeshData({verts=verts,vertCount=vw*vh,polys=polys,polyFlags=flags,polyAreas=areas,polyCount=np,nvp=nvp,
 walkableHeight=2,walkableRadius=.2,walkableClimb=.5,bmin=vector.zero,bmax=vector.create(N,1,N),cs=1,ch=1,buildBvTree=true}))
local nav,status=Detour.loadNavMeshData(data);assert(nav and Detour.statusSucceed(status))
local crowd=Crowd.new(nav,4,{maxAgentRadius=.25,constrainToNavmesh=false})
local params={radius=.2,height=2,maxAcceleration=100,maxSpeed=1,collisionQueryRange=2,pathOptimizationRange=4,separationWeight=1.25,updateFlags=Crowd.DT_CROWD_SEPARATION,obstacleAvoidanceType=0,queryFilterType=0}
local a=assert(Crowd.addAgent(crowd,vector.create(2,0,2),params));local b=assert(Crowd.addAgent(crowd,vector.create(2,0,2),params))
assert(Crowd.requestMoveVelocity(crowd,a,vector.create(1,0,0)));assert(Crowd.requestMoveVelocity(crowd,b,vector.create(1,0,0)))
local function dump(tag,step,id)
 local ag=Crowd.getAgent(crowd,id)
 local parts={string.format('%s step=%d idx=%d state=%d target=%d nneis=%d desiredSpeed=%.9g pos=%.9g,%.9g,%.9g vel=%.9g,%.9g,%.9g dvel=%.9g,%.9g,%.9g nvel=%.9g,%.9g,%.9g',tag,step,id-1,ag.state,ag.targetState,#ag.neighbours,ag.desiredSpeed,ag.position.X,ag.position.Y,ag.position.Z,ag.velocity.X,ag.velocity.Y,ag.velocity.Z,ag.desiredVelocity.X,ag.desiredVelocity.Y,ag.desiredVelocity.Z,ag.adjustedVelocity.X,ag.adjustedVelocity.Y,ag.adjustedVelocity.Z)}
 for _,n in ag.neighbours do parts[#parts+1]=string.format('nei=%d:%.9g',n.id-1,n.dist) end
 print(table.concat(parts,' '))
end
dump('before',0,a);dump('before',0,b)
for step=1,10 do Crowd.update(crowd,.05);dump('after',step,a);dump('after',step,b) end
