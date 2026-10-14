--!native
local dt=require('@recast/detour')
local cycles=tonumber(...) or 100
local n=8
local function makeTile(tx,tz)
    local v={vector.create(tx,0,tz),vector.create(tx,0,tz+1),vector.create(tx+1,0,tz),vector.create(tx+1,0,tz+1)}
    local nav=dt.buildTriangleNavMesh(v,{{1,2,3},{3,2,4}}); nav.cellSize=0.25; return nav
end
local world=dt.newTiledNavMesh({orig=vector.zero,tileWidth=1,tileHeight=1,maxTiles=n*n,maxPolys=4})
local refs={}
for z=0,n-1 do for x=0,n-1 do local r,s=dt.addTile(world,makeTile(x,z),x,z,0); assert(dt.statusSucceed(s)); refs[z*n+x+1]=r end end
local cx,cz=3,3; local slot=cz*n+cx+1
local function query100()
    local sr,sp=dt.findNearestPoly(world,vector.create(0.1,0,0.1),vector.create(0.3,1,0.3),nil)
    local er,ep=dt.findNearestPoly(world,vector.create(n-0.1,0,n-0.1),vector.create(0.3,1,0.3),nil)
    local q0=os.clock(); local ok=0
    for i=1,100 do if dt.findPath(world,sr,er,nil) then ok+=1 end end
    return (os.clock()-q0)*1000,ok
end
local before=select(1,query100())
local t0=os.clock()
for i=1,cycles do
    local old=refs[slot]; local removed,s=dt.removeTile(world,old); assert(removed and dt.statusSucceed(s))
    local r,s2=dt.addTile(world,removed,cx,cz,0); assert(dt.statusSucceed(s2)); refs[slot]=r
end
local churn=(os.clock()-t0)*1000
local after,ok=query100()
print(string.format('cycles=%d links=%d highPolys=%d activePolys=%d churn_ms=%.3f query100_before_ms=%.3f query100_after_ms=%.3f ok=%d',cycles,world.extraLinkCount or 0,world.polyCount,world.activePolyCount or -1,churn,before,after,ok))
