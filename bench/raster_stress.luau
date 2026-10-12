--!native
local rc=require('@recast/core')
local n=tonumber(...) or 256
local vcount=(n+1)*(n+1);local tcount=n*n*2
local verts=buffer.create(vcount*12);local tris=buffer.create(tcount*12)
local function vi(x,z)return z*(n+1)+x end
for z=0,n do for x=0,n do local o=vi(x,z)*12;buffer.writef32(verts,o,x);buffer.writef32(verts,o+4,0);buffer.writef32(verts,o+8,z) end end
local t=0
for z=0,n-1 do for x=0,n-1 do local a,b,c,d=vi(x,z),vi(x,z+1),vi(x+1,z),vi(x+1,z+1);local o=t*12;buffer.writeu32(tris,o,a);buffer.writeu32(tris,o+4,b);buffer.writeu32(tris,o+8,c);t+=1;o=t*12;buffer.writeu32(tris,o,c);buffer.writeu32(tris,o+4,b);buffer.writeu32(tris,o+8,d);t+=1 end end
local hf=rc.createHeightfield(n,n,vector.create(0,-1,0),vector.create(n,2,n),1,.5)
local t0=os.clock();local areas=rc.rasterizeWalkableTriangles(45,verts,tris,hf,1);local t1=os.clock()
print(`n={n} tris={tcount} spans={hf.spanCount} raster_ms={(t1-t0)*1000} areas={buffer.len(areas)}`)
