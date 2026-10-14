--!native
local dt=require('@recast/detour')
local function tile(x0)
 local v={vector.create(x0,0,0),vector.create(x0,0,1),vector.create(x0+1,0,0),vector.create(x0+1,0,1)}
 local n=dt.buildTriangleNavMesh(v,{{1,2,3},{3,2,4}}); n.cellSize=0.25; return n
end
local nav=dt.newTiledNavMesh({orig=vector.zero,tileWidth=1,tileHeight=1,maxTiles=4,maxPolys=8})
local refA,statusA=dt.addTile(nav,tile(0),0,0,0)
local refB,statusB=dt.addTile(nav,tile(1),1,0,0)
print('refs',refA,statusA,refB,statusB,'polys',nav.polyCount,'links',nav.extraLinkCount)
print('decA',dt.decodePolyId(nav,refA)); print('decB',dt.decodePolyId(nav,refB))
local qs=dt.queryPolygons(nav,vector.create(0.1,0,0.5),vector.create(0.25,1,0.25),nil); print('query',#qs); for i,r in qs do print('q',i,r,dt.decodePolyId(nav,r)) end
for _,r in qs do print('cp',r,dt.closestPointOnPoly(nav,r,vector.create(0.1,0,0.5)),dt.isValidPolyRef(nav,r,nil)) end
local sr,sp=dt.findNearestPoly(nav,vector.create(0.1,0,0.5),vector.create(0.25,1,0.25),nil)
local er,ep=dt.findNearestPoly(nav,vector.create(1.9,0,0.5),vector.create(0.25,1,0.25),nil)
print('near',sr,sp,er,ep); if sr then print('sdec',dt.decodePolyId(nav,sr)) end; if er then print('edec',dt.decodePolyId(nav,er)) end
local path=sr and er and dt.findPath(nav,sr,er,nil); print('path',path and #path or -1); if path then for i,r in path do print(i,r,dt.decodePolyId(nav,r)) end end
if path then local pts,fl,refs=dt.findStraightPath(nav,sp,ep,path); print('straight',pts and #pts or -1); if pts then for i,p in pts do print('p',i,p,fl[i],refs[i]) end end end
local ray=sr and er and dt.raycast(nav,sr,sp,ep,nil); print('ray',ray and ray.t,ray and #ray.path)
