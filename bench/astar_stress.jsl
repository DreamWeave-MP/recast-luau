--!native
local d=require('@recast/detour')
local N=tonumber(arg and arg[1]) or 128
local nvp=4
local nverts=(N+1)*(N+1)
local np=N*N
local verts=buffer.create(nverts*6)
local function vi(x,z) return z*(N+1)+x end
for z=0,N do for x=0,N do local i=vi(x,z)*6; buffer.writeu16(verts,i,x); buffer.writeu16(verts,i+2,0); buffer.writeu16(verts,i+4,z) end end
local polys=buffer.create(np*nvp*2*2); buffer.fill(polys,0,0xff,buffer.len(polys))
local areas=buffer.create(np); buffer.fill(areas,0,1,np); local flags=buffer.create(np*2); buffer.fill(flags,0,1,buffer.len(flags))
-- correct flags: u16=1
for p0=0,np-1 do buffer.writeu16(flags,p0*2,1) end
local function pi(x,z) return z*N+x end
for z=0,N-1 do for x=0,N-1 do
  local p=pi(x,z); local base=p*nvp*2
  buffer.writeu16(polys,(base+0)*2,vi(x,z)); buffer.writeu16(polys,(base+1)*2,vi(x+1,z)); buffer.writeu16(polys,(base+2)*2,vi(x+1,z+1)); buffer.writeu16(polys,(base+3)*2,vi(x,z+1))
  if z>0 then buffer.writeu16(polys,(base+nvp+0)*2,pi(x,z-1)) end
  if x<N-1 then buffer.writeu16(polys,(base+nvp+1)*2,pi(x+1,z)) end
  if z<N-1 then buffer.writeu16(polys,(base+nvp+2)*2,pi(x,z+1)) end
  if x>0 then buffer.writeu16(polys,(base+nvp+3)*2,pi(x-1,z)) end
end end
local mesh={verts=verts,nverts=nverts,polys=polys,areas=areas,flags=flags,npolys=np,nvp=nvp,bmin=vector.zero,bmax=vector.create(N,1,N),cs=1,ch=1,walkableClimb=1}
local t0=os.clock(); local nav=d.buildPolyMeshNavMesh(mesh); local t1=os.clock()
local start=vector.create(.2,0,.2); local finish=vector.create(N-.2,0,N-.2)
local sr=assert((d.findNearestPoly(nav,start,vector.create(.5,2,.5))))
local er=assert((d.findNearestPoly(nav,finish,vector.create(.5,2,.5))))
local t2=os.clock(); local p=assert(d.findPath(nav,sr,er)); local t3=os.clock()
local t4=os.clock(); local pp,st; for i=1,100 do pp,st=d.findPathPositioned(nav,sr,er,start,finish,nil) end; local t5=os.clock()
print('N',N,'polys',np,'build_ms',(t1-t0)*1000,'ref_path_ms',(t3-t2)*1000,'ref_path_len',#p,'pos100_ms',(t5-t4)*1000,'pos_path_len',pp and #pp or -1,'status',st)
