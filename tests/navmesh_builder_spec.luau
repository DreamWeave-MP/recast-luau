--!native
local jt=require('@jess/test')
local suite=jt.new()
local b=require('@recast/navmesh-builder')
local d=require('@recast/detour')

local function fromHex(s)
    s=string.gsub(s,'%s+','')
    local out=buffer.create(#s//2)
    for i=1,#s,2 do buffer.writeu8(out,(i-1)//2,tonumber(string.sub(s,i,i+1),16)) end
    return out
end
local function sameBuffer(a,c)
    if buffer.len(a)~=buffer.len(c) then return false,'length '..buffer.len(a)..' ~= '..buffer.len(c) end
    for i=0,buffer.len(a)-1 do
        local av,cv=buffer.readu8(a,i),buffer.readu8(c,i)
        if av~=cv then return false,'byte '..i..': '..av..' ~= '..cv end
    end
    return true
end
local function fixture(offmesh)
    local verts=buffer.create(24)
    local vals={0,0,0,4,0,0,4,0,4,0,0,4}
    for i=1,#vals do buffer.writeu16(verts,(i-1)*2,vals[i]) end
    local polys=buffer.create(24); buffer.fill(polys,0,0xff,24)
    for i=0,3 do buffer.writeu16(polys,i*2,i) end
    local flags=buffer.create(2); buffer.writeu16(flags,0,7)
    local areas=buffer.create(1); buffer.writeu8(areas,0,3)
    local p={verts=verts,vertCount=4,polys=polys,polyFlags=flags,polyAreas=areas,polyCount=1,nvp=6,
        userId=0x12345678,tileX=-2,tileY=5,tileLayer=1,bmin=vector.create(10,2,20),bmax=vector.create(14,4,24),
        walkableHeight=2,walkableRadius=.6,walkableClimb=.9,cs=1,ch=.5,buildBvTree=true}
    if offmesh then
        local ov=buffer.create(24); local fv={11,2,21,16,2,21}; for i=1,6 do buffer.writef32(ov,(i-1)*4,fv[i]) end
        local rad=buffer.create(4);buffer.writef32(rad,0,.5)
        local of=buffer.create(2);buffer.writeu16(of,0,9)
        local oa=buffer.create(1);buffer.writeu8(oa,0,5)
        local od=buffer.create(1);buffer.writeu8(od,0,1)
        local uid=buffer.create(4);buffer.writeu32(uid,0,0xdeadbeef)
        p.offMeshConVerts=ov;p.offMeshConRad=rad;p.offMeshConFlags=of;p.offMeshConAreas=oa;p.offMeshConDir=od;p.offMeshConUserID=uid;p.offMeshConCount=1
    end
    return p
end

local BASE=[[
56414e4407000000feffffff0500000001000000785634120100000004000000
0400000001000000000000000200000002000000000000000100000000000040
9a99193f6666663f00002041000000400000a04100006041000080400000c041
0000803f00002041000000400000a04100006041000000400000a04100006041
000000400000c04100002041000000400000c041000000000000010002000300
0000000000000000000000000000000007000403000000000000000000000000
0000000000000000000000000000000000000000000000000000000000000000
0000000000000000000000000002000000010205000203140000000000000400
000004000000000000000000000000000000000000000000
]]
local OFF=[[
56414e4407000000feffffff0500000001000000785634120200000006000000
0600000001000000000000000200000002000000010000000100000000000040
9a99193f6666663f00002041000000400000a04100006041000080400000c041
0000803f00002041000000400000a04100006041000000400000a04100006041
000000400000c04100002041000000400000c04100003041000000400000a841
00008041000000400000a8410000000000000100020003000000000000000000
0000000000000000070004030000000004000500000000000000000000000000
0000000000000000090002450000000000000000000000000000000000000000
0000000000000000000000000000000000000000000000000000000000000000
0000000000000000000000000000000000000000000000000000000000020000
0001020500020314000000000000040000000400000000000000000000000000
000000000000000000003041000000400000a84100008041000000400000a841
0000003f01000100efbeadde
]]

local DETAIL=[[
56414e4407000000feffffff0500000001000000785634120100000004000000
0400000001000000010000000400000002000000000000000100000000000040
9a99193f6666663f00002041000000400000a04100006041000080400000c041
0000803f00002041000000400000a04100006041000000400000a04100006041
000000400000c04100002041000000400000c041000000000000010002000300
0000000000000000000000000000000007000403000000000000000000000000
0000000000000000000000000000000000000000000000000000000000000000
0000000000000000000000000104000000004041000040400000b04100010401
0102040102030401030004010000000000000400010004000000000000000000
000000000000000000000000
]]

suite.it('matches upstream dtCreateNavMeshData bytes for a BV-tree quad',function()
    local got,err=b.createNavMeshData(fixture(false)); if got==nil then error(err,2) end
    local want=fromHex(BASE); local ok,msg=sameBuffer(got,want); if not ok then error(msg,2) end
    local h=b.navMeshHeader(got); jt.expect(h.magic).toBe(b.DT_NAVMESH_MAGIC);jt.expect(h.polyCount).toBe(1);jt.expect(h.bvNodeCount).toBe(2)
end)

suite.it('matches upstream dtCreateNavMeshData bytes with a bidirectional offmesh connection',function()
    local got,err=b.createNavMeshData(fixture(true)); if got==nil then error(err,2) end
    local want=fromHex(OFF); local ok,msg=sameBuffer(got,want); if not ok then error(msg,2) end
    local h=b.navMeshHeader(got);jt.expect(h.polyCount).toBe(2);jt.expect(h.vertCount).toBe(6);jt.expect(h.offMeshConCount).toBe(1)
end)


suite.it('matches upstream detail-mesh compression bytes and reconstructs detail height',function()
    local p=fixture(false)
    local dm=buffer.create(16);buffer.writeu32(dm,0,0);buffer.writeu32(dm,4,5);buffer.writeu32(dm,8,0);buffer.writeu32(dm,12,4)
    local dv=buffer.create(60);local vv={10,2,20,14,2,20,14,2,24,10,2,24,12,3,22};for i=1,#vv do buffer.writef32(dv,(i-1)*4,vv[i]) end
    local dt=buffer.create(16);local tv={0,1,4,1,1,2,4,1,2,3,4,1,3,0,4,1};for i=1,#tv do buffer.writeu8(dt,i-1,tv[i]) end
    p.detailMeshes=dm;p.detailVerts=dv;p.detailVertsCount=5;p.detailTris=dt;p.detailTriCount=4
    local got=assert(b.createNavMeshData(p));local want=fromHex(DETAIL);local ok,msg=sameBuffer(got,want);if not ok then error(msg,2) end
    local nav,status=d.loadNavMeshData(got);if nav==nil or not d.statusSucceed(status) then error('detail tile load failed',2) end
    jt.expect(nav.detailMesh.nverts).toBe(5)
    local h=d.getPolyHeight(nav,1,vector.create(12,0,22));if h==nil or math.abs(h-3)>1e-5 then error('reconstructed detail apex height mismatch',2) end
    local closest,over=d.closestPointOnPoly(nav,1,vector.create(12,10,22));if closest==nil or math.abs(closest.Y-3)>1e-5 or not over then error('closestPointOnPoly ignored Detour detail height',2) end
    local edge,edgeOver=d.closestPointOnPoly(nav,1,vector.create(15,10,22));if edge==nil or edgeOver or math.abs(edge.X-14)>1e-5 or math.abs(edge.Y-2)>1e-5 or math.abs(edge.Z-22)>1e-5 then error('closestPointOnPoly detail boundary differs from upstream',2) end
end)

suite.it('round-trips navmesh header/data endian swaps',function()
    local data=assert(b.createNavMeshData(fixture(true)))
    local original=buffer.create(buffer.len(data));buffer.copy(original,0,data,0,buffer.len(data))
    if not b.navMeshDataSwapEndian(data) then error('data swap failed',2) end
    if not b.navMeshHeaderSwapEndian(data) then error('header swap failed',2) end
    if not b.navMeshHeaderSwapEndian(data) then error('header unswap failed',2) end
    if not b.navMeshDataSwapEndian(data) then error('data unswap failed',2) end
    local ok,msg=sameBuffer(data,original);if not ok then error(msg,2) end
end)


suite.it('loads emitted/upstream-compatible tile bytes and queries the decoded detail mesh',function()
    local data=assert(b.createNavMeshData(fixture(false)))
    local nav,status=d.loadNavMeshData(data)
    if nav==nil or not d.statusSucceed(status) then error('loadNavMeshData failed',2) end
    jt.expect(nav.polyCount).toBe(1);jt.expect(nav.vertexCount).toBe(4);jt.expect(nav.detailMesh.ntris).toBe(2)
    local ref,pt=d.findNearestPoly(nav,vector.create(11.25,3,21.25),vector.create(2,4,2))
    jt.expect(ref).toBe(1)
    if pt==nil or math.abs(pt.Y-2)>1e-5 then error('decoded nearest/detail height mismatch',2) end
    local h=d.getPolyHeight(nav,ref,vector.create(12,0,22));if h==nil or math.abs(h-2)>1e-5 then error('decoded detail mesh height mismatch',2) end
end)


suite.it('adds serialized Detour tiles directly and pathfinds across binary portal edges',function()
    local function tileData(tx)
        local verts=buffer.create(24);local vals={0,0,0,1,0,0,1,0,1,0,0,1};for i=1,#vals do buffer.writeu16(verts,(i-1)*2,vals[i]) end
        local polys=buffer.create(24);buffer.fill(polys,0,0xff,24);for i=0,3 do buffer.writeu16(polys,i*2,i) end
        if tx==0 then buffer.writeu16(polys,(6+1)*2,bit32.bor(0x8000,2)) else buffer.writeu16(polys,(6+3)*2,bit32.bor(0x8000,0)) end
        local flags=buffer.create(2);buffer.writeu16(flags,0,1);local areas=buffer.create(1);buffer.writeu8(areas,0,1)
        return assert(b.createNavMeshData({verts=verts,vertCount=4,polys=polys,polyFlags=flags,polyAreas=areas,polyCount=1,nvp=6,
            tileX=tx,tileY=0,tileLayer=0,bmin=vector.create(tx,0,0),bmax=vector.create(tx+1,1,1),walkableHeight=2,walkableRadius=.2,walkableClimb=.5,cs=1,ch=1,buildBvTree=true}))
    end
    local nav=d.newTiledNavMesh({orig=vector.zero,tileWidth=1,tileHeight=1,maxTiles=2,maxPolys=8,maxVertsPerPoly=6})
    local r0,s0=d.addTileData(nav,tileData(0));if r0==nil or not d.statusSucceed(s0) then error('first binary tile add failed',2) end
    local r1,s1=d.addTileData(nav,tileData(1));if r1==nil or not d.statusSucceed(s1) then error('second binary tile add failed',2) end
    local sr=d.findNearestPoly(nav,vector.create(.25,0,.5),vector.create(.4,1,.4))
    local er=d.findNearestPoly(nav,vector.create(1.75,0,.5),vector.create(.4,1,.4))
    local path=d.findPath(nav,sr,er)
    if path==nil or #path~=2 then error('serialized portal tiles did not connect',2) end
end)


suite.it('connects serialized cross-tile offmesh links with Detour side semantics',function()
    local function groundTile(tx,withJump)
        local verts=buffer.create(24);local vals={0,0,0,1,0,0,1,0,1,0,0,1};for i=1,#vals do buffer.writeu16(verts,(i-1)*2,vals[i]) end
        local polys=buffer.create(24);buffer.fill(polys,0,0xff,24);for i=0,3 do buffer.writeu16(polys,i*2,i) end
        local flags=buffer.create(2);buffer.writeu16(flags,0,1);local areas=buffer.create(1);buffer.writeu8(areas,0,1)
        local p={verts=verts,vertCount=4,polys=polys,polyFlags=flags,polyAreas=areas,polyCount=1,nvp=6,tileX=tx,tileY=0,
            bmin=vector.create(tx,0,0),bmax=vector.create(tx+1,1,1),walkableHeight=2,walkableRadius=.2,walkableClimb=.5,cs=1,ch=1,buildBvTree=true}
        if withJump then
            local ov=buffer.create(24);local v={.5,0,.5,1.5,0,.5};for i=1,6 do buffer.writef32(ov,(i-1)*4,v[i]) end
            local rad=buffer.create(4);buffer.writef32(rad,0,.4);local of=buffer.create(2);buffer.writeu16(of,0,1)
            local oa=buffer.create(1);buffer.writeu8(oa,0,1);local od=buffer.create(1);buffer.writeu8(od,0,1)
            p.offMeshConVerts=ov;p.offMeshConRad=rad;p.offMeshConFlags=of;p.offMeshConAreas=oa;p.offMeshConDir=od;p.offMeshConCount=1
        end
        return assert(b.createNavMeshData(p))
    end
    local nav=d.newTiledNavMesh({orig=vector.zero,tileWidth=1,tileHeight=1,maxTiles=2,maxPolys=8,maxVertsPerPoly=6})
    local a,sa=d.addTileData(nav,groundTile(0,true));if a==nil or not d.statusSucceed(sa) then error('jump tile add failed',2) end
    local z,sz=d.addTileData(nav,groundTile(1,false));if z==nil or not d.statusSucceed(sz) then error('landing tile add failed',2) end
    local sr=d.findNearestPoly(nav,vector.create(.25,0,.5),vector.create(.3,1,.3))
    local er=d.findNearestPoly(nav,vector.create(1.75,0,.5),vector.create(.3,1,.3))
    local path=d.findPath(nav,sr,er)
    if path==nil or #path~=3 then error('serialized cross-tile offmesh path expected 3 polys',2) end
    if d.getPolyType(nav,path[2])~=d.DT_POLYTYPE_OFFMESH_CONNECTION then error('middle path poly is not offmesh',2) end
    local reverse=d.findPath(nav,er,sr);if reverse==nil or #reverse~=3 then error('bidirectional serialized offmesh did not connect backward',2) end
end)


suite.it('reports Detour binary load failure detail bits accurately',function()
    local valid=assert(b.createNavMeshData(fixture(false)))
    local short=buffer.create(99);buffer.copy(short,0,valid,0,99)
    local n,s=d.loadNavMeshData(short);jt.expect(n).toBeNil();if not d.statusDetail(s,d.DT_BUFFER_TOO_SMALL) then error('short header must report DT_BUFFER_TOO_SMALL',2) end

    local wrongMagic=buffer.create(buffer.len(valid));buffer.copy(wrongMagic,0,valid,0,buffer.len(valid));buffer.writeu32(wrongMagic,0,0)
    n,s=d.loadNavMeshData(wrongMagic);jt.expect(n).toBeNil();if not d.statusDetail(s,d.DT_WRONG_MAGIC) then error('bad magic detail mismatch',2) end

    local wrongVersion=buffer.create(buffer.len(valid));buffer.copy(wrongVersion,0,valid,0,buffer.len(valid));buffer.writei32(wrongVersion,4,999)
    n,s=d.loadNavMeshData(wrongVersion);jt.expect(n).toBeNil();if not d.statusDetail(s,d.DT_WRONG_VERSION) then error('bad version detail mismatch',2) end

    local negative=buffer.create(buffer.len(valid));buffer.copy(negative,0,valid,0,buffer.len(valid));buffer.writei32(negative,24,-1)
    n,s=d.loadNavMeshData(negative);jt.expect(n).toBeNil();if not d.statusDetail(s,d.DT_INVALID_PARAM) then error('negative counts must report DT_INVALID_PARAM',2) end

    local truncated=buffer.create(buffer.len(valid)-1);buffer.copy(truncated,0,valid,0,buffer.len(truncated))
    n,s=d.loadNavMeshData(truncated);jt.expect(n).toBeNil();if not d.statusDetail(s,d.DT_BUFFER_TOO_SMALL) then error('truncated tile must report DT_BUFFER_TOO_SMALL',2) end
end)

suite.it('round-trips a real Recast detail build through Detour binary tiles and tiled queries',function()
    local size=8
    local verts={}
    for z=0,size do
        for x=0,size do
            local dx,dz=x-size/2,z-size/2
            local y=if math.abs(dx)<=1 and math.abs(dz)<=1 then 0.5 else 0
            verts[#verts+1]=vector.create(x,y,z)
        end
    end
    local function vi(x,z) return z*(size+1)+x+1 end
    local tris={}
    for z=0,size-1 do for x=0,size-1 do
        local a,c=vi(x,z),vi(x+1,z);local bb,dd=vi(x,z+1),vi(x+1,z+1)
        tris[#tris+1]={a,bb,c};tris[#tris+1]={c,bb,dd}
    end end
    local build=d.buildFromTriangles(verts,tris,{cs=.5,ch=.25,walkableSlopeAngle=50,walkableHeight=4,walkableClimb=2,walkableRadius=0,
        maxSimplificationError=1.3,detailSampleDist=2,detailSampleMaxError=.1})
    if build.polyMesh==nil or build.detailMesh==nil or build.polyMesh.npolys<1 then error('Recast build did not produce mesh/detail data',2) end
    if build.detailMesh.nverts<=build.polyMesh.nverts then error('stress terrain did not produce interior detail samples',2) end
    for p0=0,build.polyMesh.npolys-1 do buffer.writeu16(build.polyMesh.flags,p0*2,1) end
    local bytes,err=b.createNavMeshDataFromPolyMesh(build.polyMesh,build.detailMesh,{walkableHeight=1,walkableRadius=0,walkableClimb=.5,buildBvTree=true})
    if bytes==nil then error(err,2) end
    local h=b.navMeshHeader(bytes);if h.detailVertCount<=0 or h.detailTriCount<=0 then error('serialized Recast detail payload is unexpectedly empty',2) end

    local decoded,status=d.loadNavMeshData(bytes);if decoded==nil or not d.statusSucceed(status) then error('direct binary reload failed',2) end
    local sample=vector.create(size/2,10,size/2)
    local directRef=d.findNearestPoly(decoded,sample,vector.create(2,20,2));if directRef==nil then error('decoded tile nearest-poly failed',2) end
    local directHeight=d.getPolyHeight(decoded,directRef,sample);if directHeight==nil then error('decoded detail height failed',2) end

    local tiled=d.newTiledNavMesh({orig=build.polyMesh.bmin,tileWidth=size,tileHeight=size,maxTiles=1,maxPolys=math.max(32,build.polyMesh.npolys+8),maxVertsPerPoly=6})
    local tileRef,addStatus=d.addTileData(tiled,bytes);if tileRef==nil or not d.statusSucceed(addStatus) then error('serialized Recast tile add failed',2) end
    local sr=d.findNearestPoly(tiled,vector.create(.75,0,.75),vector.create(1,4,1))
    local er=d.findNearestPoly(tiled,vector.create(size-.75,0,size-.75),vector.create(1,4,1))
    if sr==nil or er==nil then error('tiled nearest-poly failed after binary roundtrip',2) end
    local path=d.findPath(tiled,sr,er);if path==nil or #path<1 then error('tiled A* failed after Recast binary roundtrip',2) end
    local tiledRef=d.findNearestPoly(tiled,sample,vector.create(2,20,2));local tiledHeight=d.getPolyHeight(tiled,tiledRef,sample)
    if tiledHeight==nil or math.abs(tiledHeight-directHeight)>1e-4 then error('detail height changed across binary tiled roundtrip',2) end
end)

return suite
