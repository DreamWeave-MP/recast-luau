--!native
local dt=require('@recast/detour')
local n=tonumber(arg and arg[1]) or 256
local iters=tonumber(arg and arg[2]) or 10000
local nvp=4; local vcount=(n+1)*(n+1); local pcount=n*n
local verts=buffer.create(vcount*6); local function vi(x,z)return z*(n+1)+x end
for z=0,n do for x=0,n do local o=vi(x,z)*6; buffer.writeu16(verts,o,x);buffer.writeu16(verts,o+2,0);buffer.writeu16(verts,o+4,z) end end
local polys=buffer.create(pcount*nvp*4);buffer.fill(polys,0,0xff,buffer.len(polys)); local areas=buffer.create(pcount);local flags=buffer.create(pcount*2)
for z=0,n-1 do for x=0,n-1 do local p=z*n+x;local b=p*nvp*2
 buffer.writeu16(polys,(b+0)*2,vi(x,z));buffer.writeu16(polys,(b+1)*2,vi(x,z+1));buffer.writeu16(polys,(b+2)*2,vi(x+1,z+1));buffer.writeu16(polys,(b+3)*2,vi(x+1,z))
 if x>0 then buffer.writeu16(polys,(b+nvp+0)*2,p-1) end;if z<n-1 then buffer.writeu16(polys,(b+nvp+1)*2,p+n) end;if x<n-1 then buffer.writeu16(polys,(b+nvp+2)*2,p+1) end;if z>0 then buffer.writeu16(polys,(b+nvp+3)*2,p-n) end
 buffer.writeu8(areas,p,63);buffer.writeu16(flags,p*2,1)
end end
local mesh={verts=verts,nverts=vcount,polys=polys,npolys=pcount,nvp=nvp,areas=areas,flags=flags,bmin=vector.zero,bmax=vector.create(n,1,n),cs=1,ch=1,borderSize=0,maxEdgeError=0}
local nav=dt.buildPolyMeshNavMesh(mesh)
local hits=0; local t0=os.clock()
for i=1,iters do local x=(i*97)%n+0.5;local z=(i*193)%n+0.5;hits+=#dt.queryPolygons(nav,vector.create(x,0,z),vector.create(0.51,1,0.51),nil) end
local t1=os.clock(); print(`n={n} polys={pcount} q={iters} ms={(t1-t0)*1000} hits={hits}`)
