--!native
local d=require('@recast/detour')
local N=4;local nvp=4;local nverts=(N+1)*(N+1);local np=N*N
local verts=buffer.create(nverts*6);local function vi(x,z)return z*(N+1)+x end
for z=0,N do for x=0,N do local o=vi(x,z)*6;buffer.writeu16(verts,o,x);buffer.writeu16(verts,o+2,0);buffer.writeu16(verts,o+4,z) end end
local polys=buffer.create(np*nvp*4);buffer.fill(polys,0,0xff,buffer.len(polys));local areas=buffer.create(np);buffer.fill(areas,0,1,np);local flags=buffer.create(np*2);for i=0,np-1 do buffer.writeu16(flags,i*2,1) end
local function pi(x,z)return z*N+x end
for z=0,N-1 do for x=0,N-1 do local p=pi(x,z);local b=p*nvp*2
buffer.writeu16(polys,(b+0)*2,vi(x,z));buffer.writeu16(polys,(b+1)*2,vi(x+1,z));buffer.writeu16(polys,(b+2)*2,vi(x+1,z+1));buffer.writeu16(polys,(b+3)*2,vi(x,z+1))
if z>0 then buffer.writeu16(polys,(b+nvp+0)*2,pi(x,z-1)) end;if x<N-1 then buffer.writeu16(polys,(b+nvp+1)*2,pi(x+1,z)) end;if z<N-1 then buffer.writeu16(polys,(b+nvp+2)*2,pi(x,z+1)) end;if x>0 then buffer.writeu16(polys,(b+nvp+3)*2,pi(x-1,z)) end end end
local mesh={verts=verts,nverts=nverts,polys=polys,areas=areas,flags=flags,npolys=np,nvp=nvp,bmin=vector.zero,bmax=vector.create(N,1,N),cs=1,ch=1}
local nav=d.buildPolyMeshNavMesh(mesh);nav.walkableRadius=10
local sp=vector.create(.2,0,.2);local ep=vector.create(3.8,0,3.8);local sr=assert(d.findNearestPoly(nav,sp,vector.create(.5,2,.5)));local er=assert(d.findNearestPoly(nav,ep,vector.create(.5,2,.5)))
for _,opt in {0,d.DT_FINDPATH_ANY_ANGLE} do
 local st=d.initSlicedFindPath(nav,sr,er,sp,ep,nil,opt);local guard=0
 while d.statusInProgress(st) do st=select(1,d.updateSlicedFindPath(nav,2));guard+=1;if guard>100 then error('guard') end end
 local detached=0;for _,v in nav._slicedQuery.detached do if v then detached+=1 end end
 local p,fs=d.finalizeSlicedFindPath(nav);print('opt',opt,'detached',detached,'status',fs,'n',#p);for i,r in p do print(i,r) end
end
