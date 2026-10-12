--!native
local Builder = require('@recast/navmesh-builder')

local N = tonumber(arg and arg[1]) or 127
local runs = tonumber(arg and arg[2]) or 1
assert(N >= 1 and N <= 254, 'N must be 1..254 (dtCreateNavMeshData requires < 65535 verts)')
local nvp = 4
local side = N + 1
local vertCount = side * side
local polyCount = N * N

local verts = buffer.create(vertCount * 6)
local function vi(x,z) return z * side + x end
for z = 0, N do
    for x = 0, N do
        local o = vi(x,z) * 6
        buffer.writeu16(verts,o,x)
        buffer.writeu16(verts,o+2,0)
        buffer.writeu16(verts,o+4,z)
    end
end

local polys = buffer.create(polyCount * nvp * 4)
buffer.fill(polys,0,0xff,buffer.len(polys))
local flags = buffer.create(polyCount * 2)
local areas = buffer.create(polyCount)
buffer.fill(areas,0,1,polyCount)
for p0 = 0, polyCount - 1 do buffer.writeu16(flags,p0*2,1) end

local function pi(x,z) return z * N + x end
for z = 0, N - 1 do
    for x = 0, N - 1 do
        local p0 = pi(x,z)
        local base = p0 * nvp * 4
        buffer.writeu16(polys,base+0,vi(x,z))
        buffer.writeu16(polys,base+2,vi(x+1,z))
        buffer.writeu16(polys,base+4,vi(x+1,z+1))
        buffer.writeu16(polys,base+6,vi(x,z+1))
        -- Standard rcPolyMesh neighbour encoding (0-based polygon index; 0xffff = no neighbour).
        if z > 0 then buffer.writeu16(polys,base+8,pi(x,z-1)) end
        if x < N-1 then buffer.writeu16(polys,base+10,pi(x+1,z)) end
        if z < N-1 then buffer.writeu16(polys,base+12,pi(x,z+1)) end
        if x > 0 then buffer.writeu16(polys,base+14,pi(x-1,z)) end
    end
end

local baseParams = {
    verts=verts, vertCount=vertCount,
    polys=polys, polyFlags=flags, polyAreas=areas,
    polyCount=polyCount, nvp=nvp,
    bmin=vector.zero, bmax=vector.create(N,1,N),
    walkableHeight=2, walkableRadius=.5, walkableClimb=.5,
    cs=1, ch=1,
}

local function run(buildBvTree)
    local p = {}
    for k,v in baseParams do p[k]=v end
    p.buildBvTree = buildBvTree
    local best = math.huge
    local sum = 0
    local size = 0
    local nodes = 0
    for _ = 1, runs do
        local t0 = os.clock()
        local data, err = Builder.createNavMeshData(p)
        local ms = (os.clock()-t0)*1000
        assert(data ~= nil, err)
        best = math.min(best,ms); sum += ms; size = buffer.len(data)
        nodes = Builder.navMeshHeader(data).bvNodeCount
    end
    return best, sum/runs, size, nodes
end

local noBest,noAvg,noSize,noNodes = run(false)
local bvBest,bvAvg,bvSize,bvNodes = run(true)
print('N',N,'verts',vertCount,'polys',polyCount,'runs',runs)
print(string.format('no_bv best_ms %.3f avg_ms %.3f bytes %d nodes %d',noBest,noAvg,noSize,noNodes))
print(string.format('with_bv best_ms %.3f avg_ms %.3f bytes %d nodes %d',bvBest,bvAvg,bvSize,bvNodes))
print(string.format('bv_delta_best_ms %.3f bv_delta_avg_ms %.3f',bvBest-noBest,bvAvg-noAvg))
