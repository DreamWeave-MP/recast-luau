--!native
local dt = require('@recast/detour')
local n = tonumber(...) or 128
local nvp = 4
local vcount = (n + 1) * (n + 1)
local pcount = n * n
local verts = buffer.create(vcount * 6)
local function vi(x,z) return z * (n + 1) + x end
for z=0,n do for x=0,n do
    local o = vi(x,z) * 6
    buffer.writeu16(verts,o,x); buffer.writeu16(verts,o+2,0); buffer.writeu16(verts,o+4,z)
end end
local polys = buffer.create(pcount * nvp * 2 * 2)
buffer.fill(polys,0,0xff,buffer.len(polys))
local areas = buffer.create(pcount)
local flags = buffer.create(pcount*2)
for z=0,n-1 do for x=0,n-1 do
    local p = z*n+x
    local base = p*nvp*2
    buffer.writeu16(polys,(base+0)*2,vi(x,z))
    buffer.writeu16(polys,(base+1)*2,vi(x,z+1))
    buffer.writeu16(polys,(base+2)*2,vi(x+1,z+1))
    buffer.writeu16(polys,(base+3)*2,vi(x+1,z))
    -- adjacency half of rcPolyMesh polys
    if x>0 then buffer.writeu16(polys,(base+nvp+0)*2,p-1) end
    if z<n-1 then buffer.writeu16(polys,(base+nvp+1)*2,p+n) end
    if x<n-1 then buffer.writeu16(polys,(base+nvp+2)*2,p+1) end
    if z>0 then buffer.writeu16(polys,(base+nvp+3)*2,p-n) end
    buffer.writeu8(areas,p,63); buffer.writeu16(flags,p*2,1)
end end
local mesh={verts=verts,nverts=vcount,polys=polys,npolys=pcount,nvp=nvp,areas=areas,flags=flags,
    bmin=vector.create(0,0,0),bmax=vector.create(n,1,n),cs=1,ch=1,borderSize=0,maxEdgeError=0}
local t0=os.clock(); local nav=dt.buildPolyMeshNavMesh(mesh); local t1=os.clock()
local hits=0
local q0=os.clock()
for i=1,1000 do
    local x=(i*97)%n+0.5; local z=(i*193)%n+0.5
    local r=dt.queryPolygons(nav,vector.create(x,0,z),vector.create(0.51,1,0.51),nil)
    hits += #r
end
local q1=os.clock()
local tree=nav.bvTree; nav.bvTree=nil
local linearHits=0
local l0=os.clock()
for i=1,1000 do
    local x=(i*97)%n+0.5; local z=(i*193)%n+0.5
    local r=dt.queryPolygons(nav,vector.create(x,0,z),vector.create(0.51,1,0.51),nil)
    linearHits += #r
end
local l1=os.clock(); nav.bvTree=tree
print(`n={n} polys={pcount} build_ms={(t1-t0)*1000} bv1000_ms={(q1-q0)*1000} linear1000_ms={(l1-l0)*1000} hits={hits}/{linearHits} nodes={nav.bvNodeCount}`)
