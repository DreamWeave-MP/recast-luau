--!native
local dt=require('@recast/detour')
local n=tonumber(...) or 96
local nvp=4;local vcount=(n+1)*(n+1);local pcount=n*n
local verts=buffer.create(vcount*6);local function vi(x,z)return z*(n+1)+x end
for z=0,n do for x=0,n do local o=vi(x,z)*6;buffer.writeu16(verts,o,x);buffer.writeu16(verts,o+2,0);buffer.writeu16(verts,o+4,z) end end
local polys=buffer.create(pcount*nvp*4);buffer.fill(polys,0,0xff,buffer.len(polys));local areas=buffer.create(pcount);local flags=buffer.create(pcount*2)
for z=0,n-1 do for x=0,n-1 do local p=z*n+x;local base=p*nvp*2
 buffer.writeu16(polys,(base+0)*2,vi(x,z));buffer.writeu16(polys,(base+1)*2,vi(x,z+1));buffer.writeu16(polys,(base+2)*2,vi(x+1,z+1));buffer.writeu16(polys,(base+3)*2,vi(x+1,z))
 if x>0 then buffer.writeu16(polys,(base+nvp+0)*2,p-1) end;if z<n-1 then buffer.writeu16(polys,(base+nvp+1)*2,p+n) end;if x<n-1 then buffer.writeu16(polys,(base+nvp+2)*2,p+1) end;if z>0 then buffer.writeu16(polys,(base+nvp+3)*2,p-n) end
 buffer.writeu8(areas,p,1);buffer.writeu16(flags,p*2,1)
end end
local mesh={verts=verts,nverts=vcount,polys=polys,npolys=pcount,nvp=nvp,areas=areas,flags=flags,bmin=vector.zero,bmax=vector.create(n,1,n),cs=1,ch=1,borderSize=0,maxEdgeError=0}
local nav=dt.buildPolyMeshNavMesh(mesh);nav.walkableClimb=.5
local s,e=1,pcount;local sp=vector.create(.25,0,.25);local ep=vector.create(n-.25,0,n-.25)
local p=dt.findPathPositioned(nav,s,e,sp,ep,nil);if p==nil then error('path missing') end
local t0=os.clock();local total=0
for i=1,100 do local path=dt.findPathPositioned(nav,s,e,sp,ep,nil);total+=#path end
local t1=os.clock();local t2=os.clock();local total2=0
for i=1,100 do local path=dt.findPath(nav,s,e,nil);total2+=#path end
local t3=os.clock();print(`n={n} polys={pcount} positioned100_ms={(t1-t0)*1000} legacy100_ms={(t3-t2)*1000} path={#p} totals={total}/{total2}`)
