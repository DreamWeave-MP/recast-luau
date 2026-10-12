--!native
local tc=require('@recast/tilecache')

local GRID=tonumber((...)) or 8
local TILE=4
local function layerData(tx,ty)
    local w,h=TILE,TILE
    local n=w*h
    local heights,areas,cons=buffer.create(n),buffer.create(n),buffer.create(n)
    for z=0,h-1 do for x=0,w-1 do
        local i=x+z*w
        buffer.writeu8(areas,i,63)
        local c=0
        if x>0 then c=bit32.bor(c,1) else c=bit32.bor(c,0x10) end
        if z<h-1 then c=bit32.bor(c,2) else c=bit32.bor(c,0x20) end
        if x<w-1 then c=bit32.bor(c,4) else c=bit32.bor(c,0x40) end
        if z>0 then c=bit32.bor(c,8) else c=bit32.bor(c,0x80) end
        buffer.writeu8(cons,i,c)
    end end
    local bmin=vector.create(tx*TILE,0,ty*TILE)
    return assert(tc.buildTileCacheLayer({
        tx=tx,ty=ty,tlayer=0,bmin=bmin,bmax=bmin+vector.create(TILE,1,TILE),
        hmin=0,hmax=0,width=w,height=h,minx=0,maxx=w-1,miny=0,maxy=h-1,
    },heights,areas,cons))
end

local cache,status=tc.init({
    orig=vector.zero,cs=1,ch=.5,width=TILE,height=TILE,
    walkableHeight=1,walkableRadius=0,walkableClimb=.5,maxSimplificationError=1.3,
    maxTiles=GRID*GRID,maxObstacles=GRID,
},{maxPolys=64,meshProcess=tc.defaultMeshProcess,navMaxTiles=GRID*GRID})
assert(cache and tc.statusSucceed(status))
for y=0,GRID-1 do for x=0,GRID-1 do assert(tc.addTile(cache,layerData(x,y),0)) end end
for y=0,GRID-1 do
    assert(tc.addBoxObstacle(cache,
        vector.create(.1,-1,y*TILE+.1),
        vector.create(GRID*TILE-.1,2,y*TILE+TILE-.1)))
end
local t0=os.clock()
local calls=0
while true do
    local st,up=tc.update(cache,0)
    assert(tc.statusSucceed(st)); calls+=1
    if up then break end
end
local elapsed=(os.clock()-t0)*1000
print(string.format('tilecache grid=%dx%d tiles=%d updates=%d total=%.3fms per_update=%.4fms',GRID,GRID,GRID*GRID,calls,elapsed,elapsed/calls))
