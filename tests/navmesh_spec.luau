--!native
local jt=require('@jess/test')
local suite=jt.new()
local nm=require('@recast/navmesh')
local function expect(v,m) if not v then error(m or 'expectation failed',2) end end

suite.it('matches upstream Detour 32-bit poly-ref packing',function()
    local cases={
        {1,1,3},
        {16,128,6274},
        {1024,2048,6293506},
    }
    for _,c in cases do
        local nav,status=nm.new({maxTiles=c[1],maxPolys=c[2],tileWidth=32,tileHeight=32})
        expect(nav~=nil and nm.statusSucceed(status))
        local ref=nm.encodePolyId(nav,3,if c[1]>1 then 1 else 0,if c[2]>2 then 2 else 0)
        jt.expect(ref).toBe(c[3])
        local salt,tile,poly=nm.decodePolyId(nav,ref)
        jt.expect(salt).toBe(3); jt.expect(tile).toBe(if c[1]>1 then 1 else 0); jt.expect(poly).toBe(if c[2]>2 then 2 else 0)
        jt.expect(nm.decodePolyIdSalt(nav,ref)).toBe(salt)
        jt.expect(nm.decodePolyIdTile(nav,ref)).toBe(tile)
        jt.expect(nm.decodePolyIdPoly(nav,ref)).toBe(poly)
    end
    local invalid,status=nm.new({maxTiles=32768,maxPolys=65536})
    jt.expect(invalid).toBeNil(); expect(nm.statusFailed(status)); expect(bit32.band(status,nm.DT_INVALID_PARAM)~=0)
end)

suite.it('allocates tiles, invalidates stale refs with salt, and restores lastRef slots',function()
    local nav=assert(nm.new({orig=vector.create(10,0,20),tileWidth=8,tileHeight=4,maxTiles=4,maxPolys=16}))
    local flags=buffer.create(4); buffer.writeu16(flags,0,7); buffer.writeu16(flags,2,9)
    local areas=buffer.create(2); buffer.writeu8(areas,0,3); buffer.writeu8(areas,1,5)
    local status,ref,tile=nm.addTile(nav,{x=2,y=3,layer=0,polyCount=2,flags=flags,areas=areas})
    expect(nm.statusSucceed(status)); expect(ref~=0); jt.expect(nm.getTileRefAt(nav,2,3,0)).toBe(ref)
    local base=nm.getPolyRefBase(nav,tile); expect(nm.isValidPolyRef(nav,base)); expect(nm.isValidPolyRef(nav,base+1))
    local tx,ty=nm.calcTileLoc(nav,vector.create(26.1,0,32.1)); jt.expect(tx).toBe(2); jt.expect(ty).toBe(3)
    local oldRef=ref
    status=select(1,nm.removeTile(nav,ref)); expect(nm.statusSucceed(status)); expect(not nm.isValidPolyRef(nav,oldRef))
    local status2,newRef=nm.addTile(nav,{x=2,y=3,layer=0,polyCount=2})
    expect(nm.statusSucceed(status2)); expect(newRef~=oldRef,'salt must change when a slot is reused')
    local removed=select(1,nm.removeTile(nav,newRef)); expect(nm.statusSucceed(removed))
    local status3,restoredRef=nm.addTile(nav,{x=2,y=3,layer=0,polyCount=2},0,oldRef)
    expect(nm.statusSucceed(status3)); jt.expect(restoredRef).toBe(oldRef)
end)

suite.it('matches Detour free-slot refs, hash layer order, caps, and lastRef freelist order',function()
    local nav=assert(nm.new({maxTiles=8,maxPolys=8,tileWidth=4,tileHeight=4}))
    -- Pool accessors operate on free slots too; occupancy is not part of ref validation.
    local free0=nm.getTile(nav,0); expect(free0~=nil and not free0.active)
    local freeRef=nm.getTileRef(nav,free0); expect(freeRef~=0); jt.expect(nm.getTileByRef(nav,freeRef)).toBe(free0)

    local refs={}
    for layer=0,2 do
        local st,ref=nm.addTile(nav,{x=7,y=-3,layer=layer,polyCount=1})
        expect(nm.statusSucceed(st)); refs[#refs+1]=ref
    end
    local layers=nm.getTilesAt(nav,7,-3,2)
    jt.expect(#layers).toBe(2); jt.expect(layers[1].layer).toBe(2); jt.expect(layers[2].layer).toBe(1)

    -- Removing an arbitrary lastRef slot from Detour's freelist must not reorder the other free slots.
    local nav2=assert(nm.new({maxTiles=6,maxPolys=8}))
    nm.addTile(nav2,{x=0,y=0,layer=0,polyCount=1}) -- slot 0
    nm.addTile(nav2,{x=1,y=0,layer=0,polyCount=1}) -- slot 1, free order now 2,3,4,5
    local slot4ref=nm.encodePolyId(nav2,1,4,0)
    local st4,r4=nm.addTile(nav2,{x=4,y=0,layer=0,polyCount=1},0,slot4ref)
    expect(nm.statusSucceed(st4)); jt.expect(nm.decodePolyIdTile(nav2,r4)).toBe(4)
    local _,r2=nm.addTile(nav2,{x=2,y=0,layer=0,polyCount=1}); jt.expect(nm.decodePolyIdTile(nav2,r2)).toBe(2)
    local _,r3=nm.addTile(nav2,{x=3,y=0,layer=0,polyCount=1}); jt.expect(nm.decodePolyIdTile(nav2,r3)).toBe(3)
    local _,r5=nm.addTile(nav2,{x=5,y=0,layer=0,polyCount=1}); jt.expect(nm.decodePolyIdTile(nav2,r5)).toBe(5)
end)

suite.it('gets and mutates polygon flags and areas through encoded refs',function()
    local nav=assert(nm.new({maxTiles=2,maxPolys=8}))
    local flags=buffer.create(4); buffer.writeu16(flags,0,1); buffer.writeu16(flags,2,2)
    local areas=buffer.create(2); buffer.writeu8(areas,0,4); buffer.writeu8(areas,1,5)
    local _,base,tile=nm.addTile(nav,{x=0,y=0,layer=0,polyCount=2,flags=flags,areas=areas})
    local p1=base+1
    local f,fs=nm.getPolyFlags(nav,p1); jt.expect(f).toBe(2); expect(nm.statusSucceed(fs))
    local a,as=nm.getPolyArea(nav,p1); jt.expect(a).toBe(5); expect(nm.statusSucceed(as))
    expect(nm.statusSucceed(nm.setPolyFlags(nav,p1,0x1234))); expect(nm.statusSucceed(nm.setPolyArea(nav,p1,17)))
    jt.expect(select(1,nm.getPolyFlags(nav,p1))).toBe(0x1234); jt.expect(select(1,nm.getPolyArea(nav,p1))).toBe(17)
    local params=nm.getParams(nav); jt.expect(params.maxTiles).toBe(2); jt.expect(params.maxPolys).toBe(8)
    local rt,rp=nm.getTileAndPolyByRefUnsafe(nav,p1); jt.expect(rt).toBe(tile); jt.expect(rp).toBe(1)
end)

suite.it('round-trips Detour tile state and rejects it after ref changes',function()
    local nav=assert(nm.new({maxTiles=2,maxPolys=8}))
    local flags=buffer.create(4); buffer.writeu16(flags,0,0x12); buffer.writeu16(flags,2,0x34)
    local areas=buffer.create(2); buffer.writeu8(areas,0,7); buffer.writeu8(areas,1,9)
    local _,ref,tile=nm.addTile(nav,{x=0,y=0,layer=0,polyCount=2,flags=flags,areas=areas})
    local state,status=nm.storeTileState(nav,tile); expect(state~=nil and nm.statusSucceed(status)); jt.expect(buffer.len(state)).toBe(20)
    buffer.writeu16(flags,0,0xffff); buffer.writeu8(areas,1,63)
    expect(nm.statusSucceed(nm.restoreTileState(nav,tile,state)))
    jt.expect(buffer.readu16(flags,0)).toBe(0x12); jt.expect(buffer.readu8(areas,1)).toBe(9)
    expect(nm.statusSucceed(select(1,nm.removeTile(nav,ref))))
    local _,_,tile2=nm.addTile(nav,{x=0,y=0,layer=0,polyCount=2,flags=flags,areas=areas})
    local bad=nm.restoreTileState(nav,tile2,state); expect(nm.statusFailed(bad) and bit32.band(bad,nm.DT_INVALID_PARAM)~=0)
end)

return suite
