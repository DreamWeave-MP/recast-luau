--!native
local recast = require('@recast')
local size = tonumber(...) or 64
local verts = table.create((size+1)*(size+1))
for z=0,size do for x=0,size do verts[#verts+1]=vector.create(x,0,z) end end
local function vi(x,z) return z*(size+1)+x+1 end
local function blocked(x,z)
    if x <= 0 or z <= 0 or x >= size-1 or z >= size-1 then return false end
    if x % 8 ~= 4 then return false end
    local stripe = (x - 4) // 8
    if stripe % 2 == 0 then return z < size - 6 end
    return z > 5
end
local tris={}
for z=0,size-1 do for x=0,size-1 do
    if not blocked(x,z) then
        local a,b,c,d=vi(x,z),vi(x,z+1),vi(x+1,z),vi(x+1,z+1)
        tris[#tris+1]={a,b,c}; tris[#tris+1]={c,b,d}
    end
end end
local cfg={cs=1,ch=0.5,walkableHeight=2,walkableClimb=1,walkableRadius=0,maxSimplificationError=0.1,maxEdgeLen=0}
local t0=os.clock(); local build=recast.build(verts,tris,cfg); local t1=os.clock()
local start=vector.create(1.5,0,1.5); local finish=vector.create(size-1.5,0,size-1.5)
local r0=recast.query(build.navmesh,start,finish,vector.create(2,4,2)); local t2=os.clock()
local ok=0
for i=1,100 do local r=recast.query(build.navmesh,start,finish,vector.create(2,4,2)); if r then ok+=1 end end
local t3=os.clock()
print(`maze={size} source_tris={#tris} spans={build.compact.spanCount} rc_polys={build.polyMesh.npolys} query_polys={build.navmesh.polyCount} build_ms={(t1-t0)*1000} first_ms={(t2-t1)*1000} query100_ms={(t3-t2)*1000} ok={ok} path_polys={if r0 then #r0.corridor else 0} path_points={if r0 then #r0.points else 0}`)
