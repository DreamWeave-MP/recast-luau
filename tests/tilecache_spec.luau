--!native

local jt = require('@jess/test')
local suite = jt.new()
local dt = require('@recast/detour')
local cachelib = require('@recast/tilecache')

local function expect(condition, message)
    if not condition then error(message or 'expectation failed', 2) end
end

local function grid(size)
    local verts = {}
    for z=0,size do for x=0,size do verts[#verts+1]=vector.create(x,0,z) end end
    local function vi(x,z) return z*(size+1)+x+1 end
    local tris = {}
    for z=0,size-1 do for x=0,size-1 do
        local a,b,c,d=vi(x,z),vi(x,z+1),vi(x+1,z),vi(x+1,z+1)
        tris[#tris+1]={a,b,c}; tris[#tris+1]={c,b,d}
    end end
    return verts,tris
end

suite.it('rebuilds navigation around dynamic Luau obstacles', function()
    local verts,tris = grid(7)
    local build = dt.buildFromTriangles(verts,tris,{cs=1,ch=0.5,walkableHeight=2,walkableClimb=1,walkableRadius=0})
    local start = vector.create(1.5,0,3.5)
    local finish = vector.create(5.5,0,3.5)
    local before = dt.query(build.navmesh,start,finish)
    expect(before ~= nil, 'baseline path missing')

    local cache = cachelib.new(build)
    cachelib.addBox(cache, vector.create(2.9,-1,0), vector.create(4.1,2,7))
    local blocked = cachelib.rebuild(cache)
    local after = dt.query(blocked,start,finish)
    jt.expect(after).toBeNil()
end)

suite.it('removing an obstacle restores the cached base walkability', function()
    local verts,tris = grid(7)
    local build = dt.buildFromTriangles(verts,tris,{cs=1,ch=0.5,walkableHeight=2,walkableClimb=1,walkableRadius=0})
    local cache = cachelib.new(build)
    local id = cachelib.addCylinder(cache, vector.create(3.5,0,3.5), 1.2, 2)
    cachelib.rebuild(cache)
    jt.expect(cachelib.removeObstacle(cache,id)).toBeTruthy()
    local restored = cachelib.rebuild(cache)
    local result = dt.query(restored,vector.create(1.5,0,3.5),vector.create(5.5,0,3.5))
    expect(result ~= nil, 'restored path missing')
end)


suite.it('matches Detour ref helpers for empty pool slots', function()
    local cache,status=cachelib.init({
        orig=vector.zero,cs=1,ch=0.5,width=4,height=4,
        walkableHeight=1,walkableRadius=0,walkableClimb=0.5,maxSimplificationError=1.3,
        maxTiles=4,maxObstacles=4,
    },{maxPolys=64})
    expect(cache~=nil and cachelib.statusSucceed(status))
    local tile=cachelib.getTile(cache,0); local tref=cachelib.getTileRef(cache,tile)
    expect(tref~=0); jt.expect(cachelib.getTileByRef(cache,tref)).toBe(tile)
    local ob=cachelib.getObstacle(cache,0); local oref=cachelib.getObstacleRef(cache,ob)
    expect(oref~=0); jt.expect(cachelib.getObstacleByRef(cache,oref)).toBe(ob)
end)

suite.it('uses Detour obstacle refs, lifecycle states, and stale-ref invalidation', function()
    local verts,tris=grid(7)
    local build=dt.buildFromTriangles(verts,tris,{cs=1,ch=0.5,walkableHeight=2,walkableClimb=1,walkableRadius=0})
    local cache=cachelib.new(build,{maxObstacles=4})
    local ref,status=cachelib.addCylinder(cache,vector.create(3.5,0,3.5),1,2)
    expect(ref~=nil and cachelib.statusSucceed(status))
    jt.expect(cachelib.decodeObstacleIdObstacle(ref)).toBe(0)
    jt.expect(cachelib.decodeObstacleIdSalt(ref)).toBe(1)
    local ob=cachelib.getObstacleByRef(cache,ref); expect(ob~=nil); jt.expect(ob.state).toBe(cachelib.DT_OBSTACLE_PROCESSING)
    local us,up=cachelib.update(cache,0); expect(cachelib.statusSucceed(us) and up); jt.expect(ob.state).toBe(cachelib.DT_OBSTACLE_PROCESSED)
    expect(cachelib.statusSucceed(cachelib.removeObstacle(cache,ref))); jt.expect(ob.state).toBe(cachelib.DT_OBSTACLE_PROCESSED)
    cachelib.update(cache,0); jt.expect(ob.state).toBe(cachelib.DT_OBSTACLE_EMPTY); jt.expect(cachelib.getObstacleByRef(cache,ref)).toBeNil()
    local ref2=assert(cachelib.addCylinder(cache,vector.create(3.5,0,3.5),1,2)); expect(ref2~=ref,'reused slot must receive a new salt')
end)

suite.it('rasterizes Detour oriented-box obstacles', function()
    local verts,tris=grid(9)
    local build=dt.buildFromTriangles(verts,tris,{cs=1,ch=0.5,walkableHeight=2,walkableClimb=1,walkableRadius=0})
    local cache=cachelib.new(build)
    local ref=assert(cachelib.addOrientedBox(cache,vector.create(4.5,0,4.5),vector.create(0.55,2,5),math.pi/4))
    local ob=cachelib.getObstacleByRef(cache,ref); jt.expect(ob.type).toBe(cachelib.DT_OBSTACLE_ORIENTED_BOX)
    local bmin,bmax=cachelib.getObstacleBounds(cache,ob); expect(bmin~=nil and bmax~=nil and bmin.X<4.5 and bmax.X>4.5)
    local nav=cachelib.rebuild(cache)
    local blocked=dt.query(nav,vector.create(1,0,1),vector.create(8,0,8))
    jt.expect(blocked).toBeNil()
end)


local function flatLayerData(tx, tlayer)
    local w,h=4,4
    local heights,areas,cons=buffer.create(w*h),buffer.create(w*h),buffer.create(w*h)
    for z=0,h-1 do for x=0,w-1 do
        local i=x+z*w
        buffer.writeu8(heights,i,0); buffer.writeu8(areas,i,63)
        local c=0
        if x>0 then c=bit32.bor(c,1) else c=bit32.bor(c,0x10) end
        if z<h-1 then c=bit32.bor(c,2) else c=bit32.bor(c,0x20) end
        if x<w-1 then c=bit32.bor(c,4) else c=bit32.bor(c,0x40) end
        if z>0 then c=bit32.bor(c,8) else c=bit32.bor(c,0x80) end
        buffer.writeu8(cons,i,c)
    end end
    local layer={
        width=w,height=h,bmin=vector.create(tx*w,0,0),bmax=vector.create((tx+1)*w,1,h),
        hmin=0,hmax=0,minx=0,maxx=w-1,miny=0,maxy=h-1,
        heights=heights,areas=areas,cons=cons,
    }
    return assert(cachelib.layerFromHeightfieldLayer(layer,tx,0,tlayer or 0)), layer
end

local function compressedPair()
    local cache,status=cachelib.init({
        orig=vector.zero,cs=1,ch=0.5,width=4,height=4,
        walkableHeight=1,walkableRadius=0,walkableClimb=0.5,maxSimplificationError=1.3,
        maxTiles=4,maxObstacles=8,
    },{maxPolys=64,meshProcess=cachelib.defaultMeshProcess})
    expect(cache~=nil and cachelib.statusSucceed(status))
    local r0,s0=cachelib.addTile(cache,flatLayerData(0),0)
    local r1,s1=cachelib.addTile(cache,flatLayerData(1),0)
    expect(cachelib.statusSucceed(s0) and cachelib.statusSucceed(s1))
    return cache,r0,r1
end

suite.it('round-trips the exact 56-byte Detour tile-cache layer header', function()
    local data,layer=flatLayerData(0)
    jt.expect(buffer.len(data)).toBe(56+4*4*3)
    jt.expect(buffer.readu32(data,0)).toBe(cachelib.DT_TILECACHE_MAGIC)
    jt.expect(buffer.readu32(data,4)).toBe(cachelib.DT_TILECACHE_VERSION)
    jt.expect(buffer.readu8(data,48)).toBe(4)
    jt.expect(buffer.readu8(data,49)).toBe(4)
    local unpacked,status=cachelib.decompressTileCacheLayer(data)
    expect(unpacked~=nil and cachelib.statusSucceed(status))
    jt.expect(unpacked.header.tx).toBe(0)
    jt.expect(buffer.readu8(unpacked.areas,5)).toBe(buffer.readu8(layer.areas,5))
    jt.expect(buffer.readu8(unpacked.cons,5)).toBe(buffer.readu8(layer.cons,5))
end)

suite.it('matches Detour hashed same-coordinate layer order and 32-layer query cap', function()
    local cache,status=cachelib.init({
        orig=vector.zero,cs=1,ch=0.5,width=4,height=4,
        walkableHeight=1,walkableRadius=0,walkableClimb=0.5,maxSimplificationError=1.3,
        maxTiles=64,maxObstacles=4,
    },{maxPolys=64,meshProcess=cachelib.defaultMeshProcess})
    expect(cache~=nil and cachelib.statusSucceed(status))
    local refs={}
    for layer=0,39 do refs[#refs+1]=assert(cachelib.addTile(cache,flatLayerData(0,layer),0)) end

    -- dtTileCache inserts each compressed tile at the head of its position-hash chain.
    local all=cachelib.getTilesAt(cache,0,0)
    jt.expect(#all).toBe(40); jt.expect(all[1]).toBe(refs[40]); jt.expect(all[40]).toBe(refs[1])
    local capped=cachelib.getTilesAt(cache,0,0,2)
    jt.expect(#capped).toBe(2); jt.expect(capped[1]).toBe(refs[40]); jt.expect(capped[2]).toBe(refs[39])

    -- queryTiles/buildNavMeshTilesAt use upstream's local MAX_TILES=32 even when more layers exist.
    local hits,qstatus=cachelib.queryTiles(cache,vector.create(0,-1,0),vector.create(4,2,4),64)
    expect(cachelib.statusSucceed(qstatus)); jt.expect(#hits).toBe(32)
    jt.expect(hits[1]).toBe(refs[40]); jt.expect(hits[32]).toBe(refs[9])
end)

suite.it('exposes both upstream addBoxObstacle overloads', function()
    local cache,status=cachelib.init({
        orig=vector.zero,cs=1,ch=0.5,width=4,height=4,
        walkableHeight=1,walkableRadius=0,walkableClimb=0.5,maxSimplificationError=1.3,
        maxTiles=2,maxObstacles=4,
    },{maxPolys=64})
    expect(cache~=nil and cachelib.statusSucceed(status))
    local a=assert(cachelib.addBoxObstacle(cache,vector.create(0,0,0),vector.create(1,1,1)))
    local b=assert(cachelib.addBoxObstacle(cache,vector.create(2,0,2),vector.create(1,1,1),math.pi/4))
    jt.expect(cachelib.getObstacleByRef(cache,a).type).toBe(cachelib.DT_OBSTACLE_BOX)
    jt.expect(cachelib.getObstacleByRef(cache,b).type).toBe(cachelib.DT_OBSTACLE_ORIENTED_BOX)
end)

suite.it('stores compressed tiles with Detour salt refs and tight-bound queries', function()
    local cache,r0,r1=compressedPair()
    jt.expect(r0).toBe(4)
    jt.expect(r1).toBe(5)
    jt.expect(cachelib.decodeTileIdSalt(cache,r1)).toBe(1)
    jt.expect(cachelib.decodeTileIdTile(cache,r1)).toBe(1)
    local hits,status=cachelib.queryTiles(cache,vector.create(4.1,-1,0.1),vector.create(7.9,2,3.9),8)
    expect(cachelib.statusSucceed(status)); jt.expect(#hits).toBe(1); jt.expect(hits[1]).toBe(r1)
    local data,removeStatus=cachelib.removeTile(cache,r1)
    expect(data~=nil and cachelib.statusSucceed(removeStatus))
    jt.expect(cachelib.getTileByRef(cache,r1)).toBe(nil)
    local r2=assert(cachelib.addTile(cache,data,0))
    expect(r2~=r1,'reused compressed tile slot must bump salt')
end)

suite.it('builds adjacent compressed layers into a cross-tile Detour navmesh', function()
    local cache,r0,r1=compressedPair()
    expect(cachelib.statusSucceed(cachelib.buildNavMeshTile(cache,r0)))
    expect(cachelib.statusSucceed(cachelib.buildNavMeshTile(cache,r1)))
    jt.expect(cache.navmesh.polyCount).toBe(2)
    jt.expect(cache.navmesh.extraLinkCount).toBe(2)
    -- Differential Detour oracle: x+ uses side 0 with a full source edge; x- uses side 4
    -- and this fixture's removable border vertex clips the target edge to 191..255.
    local l0=buffer.readi32(cache.navmesh.extraFirstLinks,0)
    local l1=buffer.readi32(cache.navmesh.extraFirstLinks,4)
    jt.expect(buffer.readu8(cache.navmesh.extraLinks,l0*12+9)).toBe(0)
    jt.expect(buffer.readu8(cache.navmesh.extraLinks,l0*12+10)).toBe(0); jt.expect(buffer.readu8(cache.navmesh.extraLinks,l0*12+11)).toBe(255)
    jt.expect(buffer.readu8(cache.navmesh.extraLinks,l1*12+9)).toBe(4)
    jt.expect(buffer.readu8(cache.navmesh.extraLinks,l1*12+10)).toBe(191); jt.expect(buffer.readu8(cache.navmesh.extraLinks,l1*12+11)).toBe(255)
    local result,err=dt.query(cache.navmesh,vector.create(0.5,0,0.5),vector.create(7.5,0,3.5),vector.create(1,2,1))
    expect(result~=nil,err)
    jt.expect(#result.corridor).toBe(2)
end)

suite.it('updates one touched compressed tile per call and restores after obstacle removal', function()
    local cache,r0,r1=compressedPair()
    cachelib.buildNavMeshTile(cache,r0); cachelib.buildNavMeshTile(cache,r1)
    local ref,status=cachelib.addBox(cache,vector.create(3.5,-1,0),vector.create(4.5,2,4))
    expect(ref~=nil and cachelib.statusSucceed(status))
    local s1,up1=cachelib.update(cache,0); expect(cachelib.statusSucceed(s1)); jt.expect(up1).toBeFalsy()
    local s2,up2=cachelib.update(cache,0); expect(cachelib.statusSucceed(s2)); jt.expect(up2).toBeTruthy()
    jt.expect(cachelib.getObstacleByRef(cache,ref).state).toBe(cachelib.DT_OBSTACLE_PROCESSED)
    jt.expect(dt.query(cache.navmesh,vector.create(0.5,0,0.5),vector.create(7.5,0,3.5),vector.create(1,2,1))).toBeNil()

    expect(cachelib.statusSucceed(cachelib.removeObstacle(cache,ref)))
    local _,rup1=cachelib.update(cache,0); jt.expect(rup1).toBeFalsy()
    local _,rup2=cachelib.update(cache,0); jt.expect(rup2).toBeTruthy()
    jt.expect(cachelib.getObstacleByRef(cache,ref)).toBeNil()
    expect(dt.query(cache.navmesh,vector.create(0.5,0,0.5),vector.create(7.5,0,3.5),vector.create(1,2,1))~=nil,'removed obstacle should restore seam path')
end)



suite.it('clips TileCache external portals for raycasts and wall queries', function()
    local cache,r0,r1=compressedPair()
    expect(cachelib.statusSucceed(cachelib.buildNavMeshTile(cache,r0)))
    expect(cachelib.statusSucceed(cachelib.buildNavMeshTile(cache,r1)))
    local nav=cache.navmesh
    local lowStart=vector.create(0.5,0,0.5); local lowEnd=vector.create(7.5,0,0.5)
    local lowRef=dt.findNearestPoly(nav,lowStart,vector.create(1,2,1)); expect(lowRef~=nil)
    local lowRay=assert(dt.raycast(nav,lowRef,lowStart,lowEnd))
    expect(lowRay.t<1,'ray through the non-overlapping part of a TileCache seam must hit the wall')

    local highStart=vector.create(0.5,0,3.5); local highEnd=vector.create(7.5,0,3.5)
    local highRef=dt.findNearestPoly(nav,highStart,vector.create(1,2,1)); expect(highRef~=nil)
    local highRay=assert(dt.raycast(nav,highRef,highStart,highEnd))
    jt.expect(highRay.t).toBe(math.huge)

    local segs=assert(dt.getPolyWallSegments(nav,lowRef,nil,true))
    local portalCount,wallCount=0,0
    for _,seg in segs do if seg.ref~=0 then portalCount+=1 else wallCount+=1 end end
    expect(portalCount>=1 and wallCount>=1,'partial external edge must expose both portal and wall segments')
end)

suite.it('matches upstream TileCache contour and polymesh portal encoding', function()
    local w,h=4,4; local n=w*h
    local heights,areas,cons,regs=buffer.create(n),buffer.create(n),buffer.create(n),buffer.create(n)
    for z=0,h-1 do for x=0,w-1 do
        local i=x+z*w; buffer.writeu8(heights,i,0); buffer.writeu8(areas,i,63); buffer.writeu8(regs,i,0xff)
        local c=0
        if x>0 then c=bit32.bor(c,1) else c=bit32.bor(c,0x10) end
        if z<h-1 then c=bit32.bor(c,2) else c=bit32.bor(c,0x20) end
        if x<w-1 then c=bit32.bor(c,4) else c=bit32.bor(c,0x40) end
        if z>0 then c=bit32.bor(c,8) else c=bit32.bor(c,0x80) end
        buffer.writeu8(cons,i,c)
    end end
    local layer={header={width=w,height=h,bmin=vector.zero,bmax=vector.create(4,1,4)},heights=heights,areas=areas,cons=cons,regs=regs}
    expect(cachelib.statusSucceed(cachelib.buildTileCacheRegions(layer,0))); jt.expect(layer.regCount).toBe(1)
    local cset,cstatus=cachelib.buildTileCacheContours(layer,0,1.3); expect(cset~=nil and cachelib.statusSucceed(cstatus)); jt.expect(cset.nconts).toBe(1)
    local c=cset.conts[1]; jt.expect(c.nverts).toBe(4)
    local oracle={0,0,0,0, 0,0,4,1, 4,0,4,2, 4,0,3,131}
    for i=0,15 do jt.expect(buffer.readu8(c.verts,i)).toBe(oracle[i+1]) end
    local mesh,mstatus=cachelib.buildTileCachePolyMesh(cset,{bmin=layer.header.bmin,bmax=layer.header.bmax,cs=1,ch=1,width=w,height=h,maxError=1.3})
    expect(mesh~=nil and cachelib.statusSucceed(mstatus)); jt.expect(mesh.nverts).toBe(4); jt.expect(mesh.npolys).toBe(1); jt.expect(mesh.nvp).toBe(6)
    local ov={0,0,0, 0,0,4, 4,0,4, 4,0,3}
    for i=0,11 do jt.expect(buffer.readu16(mesh.verts,i*2)).toBe(ov[i+1]) end
    local op={1,2,3,0,65535,65535}; local on={32769,32770,65535,32768,65535,65535}
    for i=0,5 do jt.expect(buffer.readu16(mesh.polys,i*2)).toBe(op[i+1]); jt.expect(buffer.readu16(mesh.polys,(6+i)*2)).toBe(on[i+1]) end
    jt.expect(buffer.readu8(mesh.areas,0)).toBe(63); jt.expect(buffer.readu16(mesh.flags,0)).toBe(0)
end)


suite.it('matches upstream TileCache polymesh byte layout on removable-vertex fixture', function()
    -- Differential oracle generated with upstream DetourTileCacheBuilder.cpp from the same 6x5 layer.
    local w,h,climb=6,5,1; local n=w*h
    local heights,areas,cons,regs=buffer.create(n),buffer.create(n),buffer.create(n),buffer.create(n)
    for z=0,h-1 do for x=0,w-1 do
        local i=x+z*w; buffer.writeu8(heights,i,x>=3 and 3 or 0); buffer.writeu8(areas,i,63); buffer.writeu8(regs,i,0xff)
    end end
    for _,i in {2+1*w,2+2*w,2+3*w} do buffer.writeu8(areas,i,0) end
    for _,i in {0+3*w,1+3*w,0+4*w,1+4*w} do buffer.writeu8(areas,i,5) end
    buffer.writeu8(heights,4+2*w,1); buffer.writeu8(heights,4+3*w,1)
    local dx,dz={-1,0,1,0},{0,1,0,-1}
    for z=0,h-1 do for x=0,w-1 do
        local i=x+z*w
        if buffer.readu8(areas,i)~=0 then
            local c=0
            for d=0,3 do
                local nx,nz=x+dx[d+1],z+dz[d+1]
                if nx>=0 and nz>=0 and nx<w and nz<h then
                    local ni=nx+nz*w
                    if buffer.readu8(areas,ni)~=0 and math.abs(buffer.readu8(heights,i)-buffer.readu8(heights,ni))<=climb then c=bit32.bor(c,bit32.lshift(1,d)) end
                else c=bit32.bor(c,bit32.lshift(1,d+4)) end
            end
            buffer.writeu8(cons,i,c)
        end
    end end
    local layer={header={width=w,height=h,bmin=vector.zero,bmax=vector.create(w,4,h)},heights=heights,areas=areas,cons=cons,regs=regs}
    expect(cachelib.statusSucceed(cachelib.buildTileCacheRegions(layer,climb)))
    local cset,cstatus=cachelib.buildTileCacheContours(layer,climb,1.3); expect(cset~=nil and cachelib.statusSucceed(cstatus))
    local mesh,mstatus=cachelib.buildTileCachePolyMesh(cset,{bmin=layer.header.bmin,bmax=layer.header.bmax,cs=1,ch=1,maxError=1.3})
    expect(mesh~=nil and cachelib.statusSucceed(mstatus)); jt.expect(mesh.nverts).toBe(16); jt.expect(mesh.npolys).toBe(8); jt.expect(mesh.nvp).toBe(6)
    local oracleVerts={
        0,0,0, 0,0,3, 2,0,3, 3,0,0, 5,3,2, 5,3,4, 6,3,4, 6,3,2,
        0,0,5, 2,0,5, 2,0,4, 3,0,5, 3,3,0, 3,3,5, 6,3,5, 6,3,1,
    }
    for i=0,#oracleVerts-1 do jt.expect(buffer.readu16(mesh.verts,i*2)).toBe(oracleVerts[i+1]) end
    local oracleAreas={63,63,5,5,63,63,63,63}
    local oraclePolys={
        {0,1,2,3,65535,65535, 32768,3,65535,32771,65535,65535},
        {4,5,6,7,65535,65535, 7,5,32770,6,65535,65535},
        {8,9,10,1,65535,65535, 32769,4,3,32768,65535,65535},
        {1,10,2,65535,65535,65535, 2,65535,0,65535,65535,65535},
        {10,9,11,65535,65535,65535, 2,32769,65535,65535,65535,65535},
        {14,6,5,13,65535,65535, 32770,1,7,32769,65535,65535},
        {4,7,15,12,65535,65535, 1,32770,65535,7,65535,65535},
        {13,5,4,12,65535,65535, 5,1,6,65535,65535,65535},
    }
    for p0=0,7 do
        jt.expect(buffer.readu8(mesh.areas,p0)).toBe(oracleAreas[p0+1]); jt.expect(buffer.readu16(mesh.flags,p0*2)).toBe(0)
        local base=p0*12
        for j0=0,11 do jt.expect(buffer.readu16(mesh.polys,(base+j0)*2)).toBe(oraclePolys[p0+1][j0+1]) end
    end
end)




suite.it('routes TileCache mesh-process mutations through serialized dtCreateNavMeshData semantics', function()
    local callbackCount=0
    local cache,status=cachelib.init({
        orig=vector.zero,cs=1,ch=0.5,width=4,height=4,
        walkableHeight=1,walkableRadius=0,walkableClimb=0.5,maxSimplificationError=1.3,
        maxTiles=2,maxObstacles=4,
    },{
        maxPolys=64,
        meshProcess=function(params,areas,flags)
            callbackCount+=1
            jt.expect(params.nvp).toBe(6)
            jt.expect(params.tileX).toBe(0); jt.expect(params.tileY).toBe(0); jt.expect(params.tileLayer).toBe(0)
            jt.expect(params.buildBvTree).toBeFalsy()
            params.userId=0x12345678
            for p0=0,params.polyCount-1 do
                buffer.writeu8(areas,p0,7)
                buffer.writeu16(flags,p0*2,0x20)
            end
        end,
    })
    expect(cache~=nil and cachelib.statusSucceed(status))
    local ref=assert(cachelib.addTile(cache,flatLayerData(0),0))
    expect(cachelib.statusSucceed(cachelib.buildNavMeshTile(cache,ref)))
    jt.expect(callbackCount).toBe(1)
    local tile=dt.getTileAt(cache.navmesh,0,0,0); expect(tile~=nil)
    jt.expect(tile.nav.header.userId).toBe(0x12345678)
    jt.expect(tile.nav.header.bvNodeCount).toBe(0)
    local pref=dt.encodePolyId(cache.navmesh,tile.salt,tile.index,0)
    jt.expect(dt.getPolyArea(cache.navmesh,pref)).toBe(7)
    jt.expect(dt.getPolyFlags(cache.navmesh,pref)).toBe(0x20)
end)

suite.it('matches dtTileCacheHeaderSwapEndian field semantics', function()
    local data = select(1, flatLayerData(3))
    local original = buffer.create(buffer.len(data)); buffer.copy(original, 0, data, 0, buffer.len(data))
    local oneByteFields = {}
    for o=48,53 do oneByteFields[o] = buffer.readu8(data,o) end
    expect(cachelib.tileCacheHeaderSwapEndian(data), 'tile-cache header swap rejected native header')
    for o=48,53 do jt.expect(buffer.readu8(data,o)).toBe(oneByteFields[o]) end
    expect(cachelib.tileCacheHeaderSwapEndian(data), 'tile-cache header swap rejected swapped header')
    expect(buffer.len(data) == buffer.len(original))
    for i=0,buffer.len(data)-1 do
        if buffer.readu8(data,i) ~= buffer.readu8(original,i) then error('tile-cache endian roundtrip mismatch at byte '..i,2) end
    end
    local bad=buffer.create(56); buffer.copy(bad,0,original,0,56); buffer.writeu32(bad,0,0)
    jt.expect(cachelib.tileCacheHeaderSwapEndian(bad)).toBeFalsy()
end)

suite.it('exposes TileCache allocator/compressor handles like upstream', function()
    local alloc={tag='alloc'}; local comp={tag='compressor'}
    local cache,status=cachelib.init({orig=vector.zero,cs=1,ch=1,width=4,height=4,maxTiles=4,maxObstacles=8},{allocator=alloc,compressor=comp})
    expect(cachelib.statusSucceed(status)); jt.expect(cachelib.getAlloc(cache)).toBe(alloc); jt.expect(cachelib.getCompressor(cache)).toBe(comp)
end)
return suite
