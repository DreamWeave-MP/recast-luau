local rc=require('@recast/core')
local verts={vector.create(0,0,0),vector.create(0,0,5),vector.create(5,0,0),vector.create(5,0,5)}
local tris={{1,2,3},{3,2,4}}
local b=rc.buildCompactFromTriangles(verts,tris,{cs=1,ch=.2,agentRadius=0,walkableHeight=1,walkableClimb=0})
local c=rc.buildContours(b.compact,1.3,0)
local m=rc.buildPolyMesh(c,6)
print('mesh',m.nverts,m.npolys,'regions',b.compact.maxRegions)
local d=rc.buildPolyMeshDetail(m,b.compact,1,0.1)
print('detail',d.nmeshes,d.nverts,d.ntris)
for i=0,d.nmeshes-1 do local o=i*16; print('sub',i,buffer.readu32(d.meshes,o),buffer.readu32(d.meshes,o+4),buffer.readu32(d.meshes,o+8),buffer.readu32(d.meshes,o+12)) end
for i=0,math.min(d.nverts,8)-1 do local o=i*12; print('v',i,buffer.readf32(d.verts,o),buffer.readf32(d.verts,o+4),buffer.readf32(d.verts,o+8)) end
