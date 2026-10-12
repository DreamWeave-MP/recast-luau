--!native
local dt=require('@recast/detour')
local verts={
 vector.create(0,0,0),vector.create(0,0,2),vector.create(2,0,0),vector.create(2,0,2),
 vector.create(6,0,0),vector.create(6,0,2),vector.create(8,0,0),vector.create(8,0,2),
}
local tris={{1,2,3},{3,2,4},{5,6,7},{7,6,8}}
local nav=dt.buildTriangleNavMesh(verts,tris)
print('before',dt.findPath(nav,1,4,nil))
dt.addOffMeshConnections(nav,{{start=vector.create(1.8,0,1),finish=vector.create(6.2,0,1),radius=0.5,bidirectional=true,area=5,flags=1,userId=77}})
print('counts',nav.polyCount,nav.offMeshConCount,nav.offMeshPolyBase)
local conRef=nav.offMeshPolyBase+1
local con=dt.getOffMeshConnectionByRef(nav,conRef)
print('con',conRef,con.startRef,con.endRef,con.userId,con.bidirectional,con.start,con.finish)
local path=dt.findPath(nav,1,4,nil)
print('path',path and table.concat(path,','))
if path then
 local points,flags,refs=dt.findStraightPath(nav,vector.create(0.2,0,0.5),vector.create(7.8,0,0.5),path)
 print('straight',#points)
 for i,p in points do print(i,p,flags[i],refs[i]) end
end
