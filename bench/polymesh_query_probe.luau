--!native
local dt=require('@recast/detour')
local size=9
local verts={}
for z=0,size do for x=0,size do verts[#verts+1]=vector.create(x,0,z) end end
local function vi(x,z) return z*(size+1)+x+1 end
local tris={}
for z=0,size-1 do for x=0,size-1 do local a,b,c,d=vi(x,z),vi(x,z+1),vi(x+1,z),vi(x+1,z+1); tris[#tris+1]={a,b,c}; tris[#tris+1]={c,b,d} end end
local b=dt.buildFromTriangles(verts,tris,{cs=1,ch=0.5,walkableHeight=2,walkableClimb=1,walkableRadius=0,areaVolumes={{kind='box',bmin=vector.create(3,0,3),bmax=vector.create(6,2,6),area=0}},maxSimplificationError=0})
print('conts',b.contours.nconts,'pmesh',b.polyMesh.npolys,'navtris',b.navmesh.polyCount)
local r,e=dt.query(b.navmesh,vector.create(1,0,4.5),vector.create(8,0,4.5))
print('query',r~=nil,e,r and #r.corridor,r and #r.points)
if r then for i,p in r.points do print(i,p) end end
