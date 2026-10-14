--!native
local dt=require('@recast/detour')
local N=tonumber((...)) or 64
local count=N*N
local nav=dt.newTiledNavMesh({orig=vector.zero,tileWidth=1,tileHeight=1,maxTiles=count,maxPolys=4,maxVertsPerPoly=3})
local tile=dt.buildTriangleNavMesh({vector.create(.1,0,.1),vector.create(.9,0,.1),vector.create(.1,0,.9)},{{1,2,3}})
tile.nvp=tile.nvp or 3; tile.portalDirs=buffer.create(tile.polyCount*(tile.nvp or 3)); buffer.fill(tile.portalDirs,0,0xff,buffer.len(tile.portalDirs))
local ta=os.clock()
for y=0,N-1 do for x=0,N-1 do assert(dt.addTile(nav,tile,x,y,0)) end end
local addms=(os.clock()-ta)*1000
local Q=100000
local function target(i)
    local x=(i*1103515245+12345)%N
    local y=(i*2654435761+97)%N
    return x,y
end
local t0=os.clock(); local sum=0
for i=1,Q do local x,y=target(i); local t=dt.getTileAt(nav,x,y,0); if t then sum+=t.index end end
local th=(os.clock()-t0)*1000
local t1=os.clock(); local sum2=0
for i=1,Q do
    local x,y=target(i); local found=nil
    for j=1,nav.maxTiles do local t=nav.tiles[j]; if t~=nil and t.x==x and t.y==y and t.layer==0 then found=t; break end end
    if found then sum2+=found.index end
end
local tl=(os.clock()-t1)*1000
assert(sum==sum2)
print(string.format('tile_lookup n=%d tiles=%d add=%.3fms q=%d hash=%.3fms linear=%.3fms speedup=%.2fx checksum=%d',N,count,addms,Q,th,tl,tl/th,sum))
