--!native
local Builder=require('@recast/navmesh-builder')
local N=tonumber(arg and arg[1]) or 16
local nvp=4; local side=N+1; local vc=side*side; local pc=N*N
local verts=buffer.create(vc*6); local function vi(x,z)return z*side+x end
for z=0,N do for x=0,N do local o=vi(x,z)*6;buffer.writeu16(verts,o,x);buffer.writeu16(verts,o+2,0);buffer.writeu16(verts,o+4,z)end end
local polys=buffer.create(pc*nvp*4);buffer.fill(polys,0,0xff,buffer.len(polys));local flags=buffer.create(pc*2);local areas=buffer.create(pc);buffer.fill(areas,0,1,pc)
local function pi(x,z)return z*N+x end
for z=0,N-1 do for x=0,N-1 do local p,base=pi(x,z),pi(x,z)*nvp*4;buffer.writeu16(flags,p*2,1);buffer.writeu16(polys,base,vi(x,z));buffer.writeu16(polys,base+2,vi(x+1,z));buffer.writeu16(polys,base+4,vi(x+1,z+1));buffer.writeu16(polys,base+6,vi(x,z+1));if z>0 then buffer.writeu16(polys,base+8,pi(x,z-1))end;if x<N-1 then buffer.writeu16(polys,base+10,pi(x+1,z))end;if z<N-1 then buffer.writeu16(polys,base+12,pi(x,z+1))end;if x>0 then buffer.writeu16(polys,base+14,pi(x-1,z))end end end
local data,err=Builder.createNavMeshData({verts=verts,vertCount=vc,polys=polys,polyFlags=flags,polyAreas=areas,polyCount=pc,nvp=nvp,bmin=vector.zero,bmax=vector.create(N,1,N),walkableHeight=2,walkableRadius=.5,walkableClimb=.5,cs=1,ch=1,buildBvTree=true});assert(data,err)

local function fnv1a(data)
    local h=2166136261
    for i=0,buffer.len(data)-1 do
        h=bit32.bxor(h,buffer.readu8(data,i))
        local lo=bit32.band(h,0xffff); local hi=bit32.rshift(h,16)
        local p0=lo*403; local p1=hi*403+lo*256
        h=(p0+(p1%65536)*65536)%4294967296
    end
    return h
end

local dec=assert(Builder.decodeNavMeshData(data));local bv=dec.bvTree
print('N',N,'verts',vc,'polys',pc,'bytes',buffer.len(data),'fnv',string.format('%08x',fnv1a(data)),'bvcount',dec.bvNodeCount)
for i=0,math.min(11,dec.bvNodeCount-1) do local o=i*16;print(string.format('%d:%d,%d,%d-%d,%d,%d:%d',i,buffer.readu16(bv,o),buffer.readu16(bv,o+2),buffer.readu16(bv,o+4),buffer.readu16(bv,o+6),buffer.readu16(bv,o+8),buffer.readu16(bv,o+10),buffer.readi32(bv,o+12))) end
