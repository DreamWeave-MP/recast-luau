--!native
local dt=require('@recast/detour')
local n=tonumber(...) or 8
local function tile(tx,tz)
    local x0,z0=tx,tz
    local v={vector.create(x0,0,z0),vector.create(x0,0,z0+1),vector.create(x0+1,0,z0),vector.create(x0+1,0,z0+1)}
    local nav=dt.buildTriangleNavMesh(v,{{1,2,3},{3,2,4}}); nav.cellSize=0.25; return nav
end
local world=dt.newTiledNavMesh({orig=vector.zero,tileWidth=1,tileHeight=1,maxTiles=n*n,maxPolys=4})
local t0=os.clock()
for z=0,n-1 do for x=0,n-1 do local _,status=dt.addTile(world,tile(x,z),x,z,0); assert(dt.statusSucceed(status)) end end
local t1=os.clock()
local startRef,startPos=dt.findNearestPoly(world,vector.create(0.1,0,0.1),vector.create(0.3,1,0.3),nil)
local endRef,endPos=dt.findNearestPoly(world,vector.create(n-0.1,0,n-0.1),vector.create(0.3,1,0.3),nil)
local first0=os.clock(); local first=assert(dt.findPath(world,startRef,endRef,nil)); local first1=os.clock()
local ok=0; local q0=os.clock()
for i=1,100 do local p=dt.findPath(world,startRef,endRef,nil); if p then ok+=1 end end
local q1=os.clock()
print(string.format('tiles=%d polys=%d links=%d add_ms=%.3f first_ms=%.3f query100_ms=%.3f ok=%d path=%d',n*n,world.polyCount,world.extraLinkCount or 0,(t1-t0)*1000,(first1-first0)*1000,(q1-q0)*1000,ok,#first))
