--!native
local recast = require('@recast')
local size = tonumber(...) or 32
local verts = table.create((size+1)*(size+1))
for z=0,size do for x=0,size do verts[#verts+1]=vector.create(x,0,z) end end
local function vi(x,z) return z*(size+1)+x+1 end
local tris = table.create(size*size*2)
for z=0,size-1 do for x=0,size-1 do
  local a,b,c,d=vi(x,z),vi(x,z+1),vi(x+1,z),vi(x+1,z+1)
  tris[#tris+1]={a,b,c}; tris[#tris+1]={c,b,d}
end end
local t0=os.clock()
local build=recast.build(verts,tris,{cs=1,ch=0.5,walkableHeight=2,walkableClimb=1,walkableRadius=0})
local t1=os.clock()
local ok=0
for i=1,100 do
  local a=1.2 + ((i*17)%(size-3))
  local b=1.2 + ((i*29)%(size-3))
  local result=recast.query(build.navmesh,vector.create(a,0,1.2),vector.create(b,0,size-1.2))
  if result then ok+=1 end
end
local t2=os.clock()
print(`grid={size} tris={#tris} spans={build.compact.spanCount} polys={build.navmesh.polyCount} build_ms={(t1-t0)*1000} query100_ms={(t2-t1)*1000} ok={ok}`)
