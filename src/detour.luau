--!native

-- Pure-Luau Detour-style navigation over a triangle navmesh.
-- The storage and query engine are an altered Luau implementation inspired by RecastNavigation's
-- Detour algorithms; there is no C ABI and no native Recast/Detour dependency.

local Common = require('@recast/common')
local Recast = require('@recast/core')

local function vertexAt(vertices, index0)
    local o = index0 * 12
    return vector.create(buffer.readf32(vertices,o), buffer.readf32(vertices,o+4), buffer.readf32(vertices,o+8))
end
local function triangleAt(triangles, tri0)
    local o = tri0 * 12
    return buffer.readu32(triangles,o), buffer.readu32(triangles,o+4), buffer.readu32(triangles,o+8)
end
local function readVec3f(source, index1)
    local o = (index1 - 1) * 12
    return vector.create(buffer.readf32(source,o), buffer.readf32(source,o+4), buffer.readf32(source,o+8))
end
local function writeVec3f(target, index1, v: vector)
    local o = (index1 - 1) * 12
    buffer.writef32(target,o,v.X); buffer.writef32(target,o+4,v.Y); buffer.writef32(target,o+8,v.Z)
end

local Detour = {}

Detour.DT_NULL_LINK = 0xffffffff
Detour.DT_STRAIGHTPATH_START = 0x01
Detour.DT_STRAIGHTPATH_END = 0x02
Detour.DT_STRAIGHTPATH_OFFMESH_CONNECTION = 0x04
Detour.DT_STRAIGHTPATH_AREA_CROSSINGS = 0x01
Detour.DT_STRAIGHTPATH_ALL_CROSSINGS = 0x02
Detour.DT_POLYTYPE_GROUND = 0
Detour.DT_POLYTYPE_OFFMESH_CONNECTION = 1
Detour.DT_OFFMESH_CON_BIDIR = 1

Detour.DT_MAX_AREAS = 64

-- Detour status bit layout (DetourStatus.h).  Luau numbers represent all 32-bit flags exactly.
Detour.DT_FAILURE = 0x80000000
Detour.DT_SUCCESS = 0x40000000
Detour.DT_IN_PROGRESS = 0x20000000
Detour.DT_STATUS_DETAIL_MASK = 0x0ffffff
Detour.DT_WRONG_MAGIC = 0x01
Detour.DT_WRONG_VERSION = 0x02
Detour.DT_OUT_OF_MEMORY = 0x04
Detour.DT_INVALID_PARAM = 0x08
Detour.DT_BUFFER_TOO_SMALL = 0x10
Detour.DT_OUT_OF_NODES = 0x20
Detour.DT_PARTIAL_RESULT = 0x40
Detour.DT_ALREADY_OCCUPIED = 0x80
Detour.DT_FINDPATH_ANY_ANGLE = 0x02
Detour.DT_RAYCAST_USE_COSTS = 0x01

function Detour.statusSucceed(status) return bit32.band(status, Detour.DT_SUCCESS) ~= 0 end
function Detour.statusFailed(status) return bit32.band(status, Detour.DT_FAILURE) ~= 0 end
function Detour.statusInProgress(status) return bit32.band(status, Detour.DT_IN_PROGRESS) ~= 0 end
function Detour.statusDetail(status, detail) return bit32.band(status, detail) ~= 0 end

function Detour.newQueryFilter(options)
    options = options or {}
    local costs = buffer.create(Detour.DT_MAX_AREAS * 4)
    for area = 0, Detour.DT_MAX_AREAS - 1 do buffer.writef32(costs, area * 4, 1) end
    local filter = {
        includeFlags = options.includeFlags or 0xffff,
        excludeFlags = options.excludeFlags or 0,
        areaCosts = costs,
        minAreaCost = 1,
    }
    if options.areaCosts ~= nil then
        for area, cost in options.areaCosts do
            local index = if type(area) == 'number' then area else tonumber(area)
            if index ~= nil and index >= 0 and index < Detour.DT_MAX_AREAS then
                buffer.writef32(costs, index * 4, cost)
                if cost < filter.minAreaCost then filter.minAreaCost = cost end
            end
        end
    end
    return filter
end

function Detour.setAreaCost(filter, area, cost)
    if area < 0 or area >= Detour.DT_MAX_AREAS then error('area must be in [0, 63]', 2) end
    if cost <= 0 then error('area cost must be positive', 2) end
    buffer.writef32(filter.areaCosts, area * 4, cost)
    local minimum = math.huge
    for i = 0, Detour.DT_MAX_AREAS - 1 do minimum = math.min(minimum, buffer.readf32(filter.areaCosts, i * 4)) end
    filter.minAreaCost = minimum
    return filter
end

function Detour.getAreaCost(filter, area)
    if area < 0 or area >= Detour.DT_MAX_AREAS then error('area must be in [0, 63]', 2) end
    return buffer.readf32(filter.areaCosts, area * 4)
end

-- dtQueryFilter public data accessors.  The functional Luau API keeps the filter as an
-- explicit value rather than a userdata object, but preserves the same 16-bit flag semantics.
function Detour.getIncludeFlags(filter) return filter.includeFlags end
function Detour.setIncludeFlags(filter, flags) filter.includeFlags = bit32.band(flags, 0xffff); return filter end
function Detour.getExcludeFlags(filter) return filter.excludeFlags end
function Detour.setExcludeFlags(filter, flags) filter.excludeFlags = bit32.band(flags, 0xffff); return filter end

local function passFilter(nav, tri0, filter)
    if not (tri0 >= 0 and tri0 < nav.polyCount) then return false end
    if nav.polyValid ~= nil and buffer.readu8(nav.polyValid, tri0) == 0 then return false end
    if filter == nil then return true end
    local flags = buffer.readu16(nav.flags, tri0 * 2)
    return bit32.band(flags, filter.includeFlags) ~= 0 and bit32.band(flags, filter.excludeFlags) == 0
end

local function areaCost(nav, tri0, filter)
    if filter == nil then return 1 end
    return buffer.readf32(filter.areaCosts, buffer.readu8(nav.areas, tri0) * 4)
end

local function triIndices(nav, tri0)
    return triangleAt(nav.triangles, tri0)
end

local function triVertex(nav, tri0, edgeVertex)
    local a, b, c = triIndices(nav, tri0)
    local index0 = if edgeVertex == 0 then a elseif edgeVertex == 1 then b else c
    return vertexAt(nav.vertices, index0), index0
end

local function polyVertCount(nav, poly0)
    if nav.polyVertCounts ~= nil then return buffer.readu8(nav.polyVertCounts, poly0) end
    return 3
end

local function polyVertexIndex(nav, poly0, edge0)
    if nav.polyIndices ~= nil then
        local bytes=nav.polyIndexBytes or 2
        local o=(poly0*nav.nvp+edge0)*bytes
        return if bytes==4 then buffer.readu32(nav.polyIndices,o) else buffer.readu16(nav.polyIndices,o)
    end
    local a, b, c = triIndices(nav, poly0)
    return if edge0 == 0 then a elseif edge0 == 1 then b else c
end

local function polyNeighbor(nav, poly0, edge0)
    local stride = nav.nvp or 3
    return buffer.readi32(nav.neighbors, (poly0 * stride + edge0) * 4)
end

local function polyType(nav, poly0)
    if nav.polyTypes == nil then return Detour.DT_POLYTYPE_GROUND end
    return buffer.readu8(nav.polyTypes, poly0)
end

-- Extra links model Detour's dtLink records which are not tied to a ground polygon edge.
-- They are used first by off-mesh connections, and later by external/tile links.  Records are
-- packed to keep traversal data contiguous: target poly (i32), next link (i32), edge (u8).
local EXTRA_LINK_BYTES = 12
local function extraLinkTarget(nav, link0)
    local o=link0*EXTRA_LINK_BYTES
    if nav.extraLinkTargetsAreRefs then
        local ref=buffer.readu32(nav.extraLinks,o)
        return nav.refToPoly[ref] or -1
    end
    return buffer.readi32(nav.extraLinks,o)
end
local function extraLinkNext(nav, link0) return buffer.readi32(nav.extraLinks, link0 * EXTRA_LINK_BYTES + 4) end
local function extraLinkEdge(nav, link0) return buffer.readu8(nav.extraLinks, link0 * EXTRA_LINK_BYTES + 8) end
local function extraLinkSide(nav, link0) return buffer.readu8(nav.extraLinks, link0 * EXTRA_LINK_BYTES + 9) end
local function extraLinkBMin(nav, link0) return buffer.readu8(nav.extraLinks, link0 * EXTRA_LINK_BYTES + 10) end
local function extraLinkBMax(nav, link0) return buffer.readu8(nav.extraLinks, link0 * EXTRA_LINK_BYTES + 11) end
local function firstExtraLink(nav, poly0)
    if nav.extraFirstLinks == nil then return -1 end
    return buffer.readi32(nav.extraFirstLinks, poly0 * 4)
end

local function findExtraLink(nav, from0, to0)
    local link0 = firstExtraLink(nav, from0)
    while link0 >= 0 do
        if extraLinkTarget(nav, link0) == to0 then return link0 end
        link0 = extraLinkNext(nav, link0)
    end
    return -1
end

local function linkedNeighborAtEdge(nav, poly0, edge0)
    local neighbor=polyNeighbor(nav,poly0,edge0)
    if neighbor>=0 then return neighbor end
    local link0=firstExtraLink(nav,poly0)
    while link0>=0 do
        if extraLinkEdge(nav,link0)==edge0 then
            local target=extraLinkTarget(nav,link0)
            -- Incremental tiled meshes deliberately leave stale salt-versioned links in the
            -- append-only link pool.  Skip dead refs so a later re-added tile can install a new
            -- live link at the same physical edge without compacting the whole world.
            if target>=0 then return target end
        end
        link0=extraLinkNext(nav,link0)
    end
    return -1
end




-- Queries operate on dense global polygon indices internally.  Public refs are intentionally
-- decoded/encoded only at the API/topology boundary so tiled salt/tile/poly refs do not infect
-- the hot geometry loops.  Untiled meshes retain the historical poly0+1 representation exactly.
local function refToPoly0(nav, ref)
    if ref == nil or ref == 0 then return -1 end
    local map = nav.refToPoly
    if map == nil then return ref - 1 end
    return map[ref] or -1
end

local function poly0ToRef(nav, poly0)
    if poly0 == nil or poly0 < 0 then return 0 end
    local refs = nav.polyRefs
    if refs == nil then return poly0 + 1 end
    return buffer.readu32(refs, poly0 * 4)
end

-- Explicit dtQueryFilter operations for callers porting Detour code directly.
-- Internal hot paths still call the local helpers above to avoid module dispatch.
function Detour.passFilter(nav, ref, filter)
    local poly0 = refToPoly0(nav, ref)
    return passFilter(nav, poly0, filter)
end

function Detour.getCost(nav, filter, pa, pb, curRef)
    local poly0 = refToPoly0(nav, curRef)
    if poly0 < 0 or poly0 >= nav.polyCount then return math.huge end
    return vector.magnitude(pb - pa) * areaCost(nav, poly0, filter)
end

-- Functional equivalent of dtNavMeshQuery::getAttachedNavMesh().
function Detour.getAttachedNavMesh(nav) return nav end

local function edgeTraversableIntervals(nav,poly0,edge0,filter)
    local internal=polyNeighbor(nav,poly0,edge0)
    if internal>=0 then
        if passFilter(nav,internal,filter) then return {{tmin=0,tmax=255,poly=internal,ref=poly0ToRef(nav,internal)}} end
        return {}
    end
    local out={}
    local link0=firstExtraLink(nav,poly0)
    while link0>=0 do
        if extraLinkEdge(nav,link0)==edge0 then
            local target=extraLinkTarget(nav,link0)
            if target>=0 and passFilter(nav,target,filter) then
                local side=extraLinkSide(nav,link0)
                out[#out+1]={
                    tmin=if side==0xff then 0 else extraLinkBMin(nav,link0),
                    tmax=if side==0xff then 255 else extraLinkBMax(nav,link0),
                    poly=target,ref=poly0ToRef(nav,target),link=link0,
                }
            end
        end
        link0=extraLinkNext(nav,link0)
    end
    table.sort(out,function(a,b) if a.tmin==b.tmin then return a.tmax<b.tmax end return a.tmin<b.tmin end)
    return out
end

local function linkedNeighborAtEdgeT(nav,poly0,edge0,t,filter)
    local internal=polyNeighbor(nav,poly0,edge0)
    if internal>=0 then return passFilter(nav,internal,filter) and internal or -1 end
    local q=math.clamp(math.floor(t*255+0.5),0,255)
    local link0=firstExtraLink(nav,poly0)
    while link0>=0 do
        if extraLinkEdge(nav,link0)==edge0 then
            local target=extraLinkTarget(nav,link0)
            if target>=0 and passFilter(nav,target,filter) then
                local side=extraLinkSide(nav,link0)
                if side==0xff or (q>=extraLinkBMin(nav,link0) and q<=extraLinkBMax(nav,link0)) then return target end
            end
        end
        link0=extraLinkNext(nav,link0)
    end
    return -1
end

local function appendEdgeSegment(out,a,b,t0,t1,ref)
    if t1<=t0 then return end
    local d=b-a
    out[#out+1]={a=a+d*t0,b=a+d*t1,ref=ref or 0}
end

local function triangleCenter(nav, tri0)
    return readVec3f(nav.centers, tri0 + 1)
end

local function polyCenter(nav, poly0)
    return readVec3f(nav.centers, poly0 + 1)
end

local function isWalkable(nav, tri0)
    return tri0 >= 0 and tri0 < nav.polyCount and (nav.polyValid == nil or buffer.readu8(nav.polyValid, tri0) ~= 0)
end

local function vertexKey(v: vector)
    -- Float32 values have a stable textual spelling here because source vertices are packed before
    -- welding. The fallback weld lets importer-produced meshes connect even when they duplicate
    -- vertices on material/UV seams.
    return string.format('%.9g|%.9g|%.9g', v.X, v.Y, v.Z)
end

local function edgeKey(a, b)
    if a < b then return a .. '>' .. b end
    return b .. '>' .. a
end

-- Cache world-space polygon AABBs once. Upstream Detour's dtCreateNavMeshData builds a
-- quantized BV tree over equivalent bounds; this packed cache is also the leaf payload for the
-- Luau BV tree and avoids rebuilding a polygon box from vertices on every overlap query.
local function buildPolyBounds(nav)
    local bounds = buffer.create(nav.polyCount * 24)
    for poly0 = 0, nav.polyCount - 1 do
        local nv = polyVertCount(nav, poly0)
        if nv > 0 then
            local v = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, 0))
            local minx, miny, minz = v.X, v.Y, v.Z
            local maxx, maxy, maxz = minx, miny, minz
            for edge0 = 1, nv - 1 do
                v = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, edge0))
                minx, miny, minz = math.min(minx, v.X), math.min(miny, v.Y), math.min(minz, v.Z)
                maxx, maxy, maxz = math.max(maxx, v.X), math.max(maxy, v.Y), math.max(maxz, v.Z)
            end
            local o = poly0 * 24
            buffer.writef32(bounds, o, minx); buffer.writef32(bounds, o + 4, miny); buffer.writef32(bounds, o + 8, minz)
            buffer.writef32(bounds, o + 12, maxx); buffer.writef32(bounds, o + 16, maxy); buffer.writef32(bounds, o + 20, maxz)
        end
    end
    nav.polyBounds = bounds
    return bounds
end

local function boundsOverlap(nav, poly0, qmin: vector, qmax: vector)
    local bounds = nav.polyBounds
    if bounds == nil then bounds = buildPolyBounds(nav) end
    local o = poly0 * 24
    return buffer.readf32(bounds, o) <= qmax.X and buffer.readf32(bounds, o + 12) >= qmin.X
       and buffer.readf32(bounds, o + 4) <= qmax.Y and buffer.readf32(bounds, o + 16) >= qmin.Y
       and buffer.readf32(bounds, o + 8) <= qmax.Z and buffer.readf32(bounds, o + 20) >= qmin.Z
end

-- Detour-style quantized BV tree. Nodes use the same escape-index convention as dtBVNode:
-- a non-negative index is a polygon leaf; a negative index skips the entire subtree.
local BV_NODE_BYTES = 16
local function bvWriteNode(nodes, node0, bminx, bminy, bminz, bmaxx, bmaxy, bmaxz, index)
    local o = node0 * BV_NODE_BYTES
    buffer.writeu16(nodes, o, bminx); buffer.writeu16(nodes, o + 2, bminy); buffer.writeu16(nodes, o + 4, bminz)
    buffer.writeu16(nodes, o + 6, bmaxx); buffer.writeu16(nodes, o + 8, bmaxy); buffer.writeu16(nodes, o + 10, bmaxz)
    buffer.writei32(nodes, o + 12, index)
end

local function partitionBVMedian(order, keys, polyIds, lo, hi, kth)
    -- Partition an integer permutation only. Bounds live in structure-of-arrays scratch below, so
    -- the hot loop swaps one number instead of whole Lua records.
    local l, r = lo, hi
    while l < r do
        local pivotIndex = (l + r) // 2
        local pivotId = order[pivotIndex]
        local pv, pi = keys[pivotId], polyIds[pivotId]
        order[pivotIndex], order[r] = order[r], pivotId
        local store = l
        for i = l, r - 1 do
            local itemId = order[i]
            local iv = keys[itemId]
            if iv < pv or (iv == pv and polyIds[itemId] < pi) then
                if i ~= store then order[i], order[store] = order[store], itemId end
                store += 1
            end
        end
        order[store], order[r] = order[r], order[store]
        if store == kth then return end
        if kth < store then r = store - 1 else l = store + 1 end
    end
end

local function buildBVTree(nav, cellSize)
    local groundCount = nav.groundPolyCount or nav.polyCount
    if groundCount <= 0 or cellSize == nil or cellSize <= 0 then return nil end
    local qfac = 1 / cellSize
    local order=table.create(groundCount); local scratch=table.create(groundCount); local polyIds=table.create(groundCount)
    local mins,maxs=table.create(groundCount),table.create(groundCount)
    local bounds = nav.polyBounds or buildPolyBounds(nav)
    local ni=0
    local rootMin,rootMax=vector.create(0xffff,0xffff,0xffff),vector.zero
    for poly0 = 0, nav.polyCount - 1 do
        if polyType(nav,poly0)~=Detour.DT_POLYTYPE_GROUND then continue end
        local o = poly0 * 24
        ni+=1
        order[ni]=ni; polyIds[ni]=poly0
        local mn=vector.create(
            math.clamp(math.floor((buffer.readf32(bounds,o)-nav.bmin.X)*qfac),0,0xffff),
            math.clamp(math.floor((buffer.readf32(bounds,o+4)-nav.bmin.Y)*qfac),0,0xffff),
            math.clamp(math.floor((buffer.readf32(bounds,o+8)-nav.bmin.Z)*qfac),0,0xffff))
        local mx=vector.create(
            math.clamp(math.ceil((buffer.readf32(bounds,o+12)-nav.bmin.X)*qfac),0,0xffff),
            math.clamp(math.ceil((buffer.readf32(bounds,o+16)-nav.bmin.Y)*qfac),0,0xffff),
            math.clamp(math.ceil((buffer.readf32(bounds,o+20)-nav.bmin.Z)*qfac),0,0xffff))
        mins[ni],maxs[ni]=mn,mx
        rootMin,rootMax=vector.min(rootMin,mn),vector.max(rootMax,mx)
    end

    local nodes = buffer.create(math.max(1, groundCount * 2) * BV_NODE_BYTES)
    local curNode = 0
    local function subdivide(imin, imax, bmin: vector, bmax: vector) -- 1-based, inclusive-exclusive
        local inum = imax - imin
        local icur = curNode
        curNode += 1
        if inum == 1 then
            local id=order[imin]; local mn,mx=mins[id],maxs[id]
            bvWriteNode(nodes,icur,mn.X,mn.Y,mn.Z,mx.X,mx.Y,mx.Z,polyIds[id])
            return
        end
        local ext=bmax-bmin
        local axis=if ext.Y>ext.X and ext.Y>=ext.Z then 2 elseif ext.Z>ext.X and ext.Z>ext.Y then 3 else 1
        local amin,amax
        if axis==1 then amin,amax=bmin.X,bmax.X elseif axis==2 then amin,amax=bmin.Y,bmax.Y else amin,amax=bmin.Z,bmax.Z end
        local threshold = (amin + amax + 1) // 2

        local l,r=imin,imax-1
        local lmin,lmax=vector.create(0xffff,0xffff,0xffff),vector.zero
        local rmin,rmax=vector.create(0xffff,0xffff,0xffff),vector.zero
        for i=imin,imax-1 do
            local id=order[i]; local mn,mx=mins[id],maxs[id]
            local key=if axis==1 then mn.X elseif axis==2 then mn.Y else mn.Z
            if key < threshold then
                scratch[l]=id; l+=1
                lmin,lmax=vector.min(lmin,mn),vector.max(lmax,mx)
            else
                scratch[r]=id; r-=1
                rmin,rmax=vector.min(rmin,mn),vector.max(rmax,mx)
            end
        end
        local split=l
        table.move(scratch,imin,imax-1,imin,order)

        if split <= imin or split >= imax then
            -- No useful spatial separation on the selected axis.  Every candidate lies on the same
            -- side of the midpoint, so preserving arbitrary stable membership is equivalent here;
            -- split by count to keep the tree balanced instead of running another nth-element pass.
            split=imin+inum//2
            lmin,lmax=vector.create(0xffff,0xffff,0xffff),vector.zero
            rmin,rmax=vector.create(0xffff,0xffff,0xffff),vector.zero
            for i=imin,split-1 do local id=order[i]; lmin,lmax=vector.min(lmin,mins[id]),vector.max(lmax,maxs[id]) end
            for i=split,imax-1 do local id=order[i]; rmin,rmax=vector.min(rmin,mins[id]),vector.max(rmax,maxs[id]) end
        end
        subdivide(imin,split,lmin,lmax)
        subdivide(split,imax,rmin,rmax)
        bvWriteNode(nodes,icur,bmin.X,bmin.Y,bmin.Z,bmax.X,bmax.Y,bmax.Z,-(curNode-icur))
    end
    subdivide(1, groundCount + 1, rootMin,rootMax)
    nav.bvTree, nav.bvNodeCount, nav.bvQuantFactor = nodes, curNode, qfac
    return nodes
end

local function overlapQuantBounds(nodes, node0, bminx,bminy,bminz,bmaxx,bmaxy,bmaxz)
    local o = node0 * BV_NODE_BYTES
    return bminx <= buffer.readu16(nodes,o+6) and bmaxx >= buffer.readu16(nodes,o)
       and bminy <= buffer.readu16(nodes,o+8) and bmaxy >= buffer.readu16(nodes,o+2)
       and bminz <= buffer.readu16(nodes,o+10) and bmaxz >= buffer.readu16(nodes,o+4)
end

local function makeAreas(vertices, triangles, triCount, options)
    if options and options.areas ~= nil then
        if typeof(options.areas) == 'buffer' then
            if buffer.len(options.areas) < triCount then error('areas buffer is shorter than triangle count', 3) end
            local out = buffer.create(triCount)
            buffer.copy(out, 0, options.areas, 0, triCount)
            return out
        end
        local out = buffer.create(triCount)
        for i = 1, triCount do buffer.writeu8(out, i - 1, options.areas[i] or Recast.RC_NULL_AREA) end
        return out
    end
    return Recast.markWalkableTriangles((options and options.walkableSlopeAngle) or 45, vertices, triangles)
end

function Detour.buildTriangleNavMesh(vertices, triangles, options)
    local verts, vertCount = Common.packVertices(vertices)
    local tris, triCount = Common.packTriangles(triangles)
    local areas = makeAreas(verts, tris, triCount, options)
    local neighbors = buffer.create(triCount * 3 * 4)
    local centers = buffer.create(triCount * 12)
    local flags = buffer.create(triCount * 2)
    local polyTypes = buffer.create(triCount)
    local polyValid = buffer.create(triCount)

    for i = 0, triCount * 3 - 1 do buffer.writei32(neighbors, i * 4, -1) end

    local vkeys = table.create(vertCount)
    for i = 0, vertCount - 1 do vkeys[i + 1] = vertexKey(vertexAt(verts, i)) end

    local edges = {}
    local walkableCount = 0
    for tri0 = 0, triCount - 1 do
        local ia, ib, ic = triangleAt(tris, tri0)
        local a, b, c = vertexAt(verts, ia), vertexAt(verts, ib), vertexAt(verts, ic)
        writeVec3f(centers, tri0 + 1, (a + b + c) / 3)
        if buffer.readu8(areas, tri0) ~= Recast.RC_NULL_AREA then
            walkableCount += 1
            buffer.writeu8(polyValid, tri0, 1)
            buffer.writeu16(flags, tri0 * 2, 1)
            local ids = {ia, ib, ic}
            for edge0 = 0, 2 do
                local va0 = ids[edge0 + 1]
                local vb0 = ids[((edge0 + 1) % 3) + 1]
                local key = edgeKey(vkeys[va0 + 1], vkeys[vb0 + 1])
                local previous = edges[key]
                if previous == nil then
                    edges[key] = {tri = tri0, edge = edge0}
                elseif previous.tri ~= tri0 then
                    buffer.writei32(neighbors, (tri0 * 3 + edge0) * 4, previous.tri)
                    buffer.writei32(neighbors, (previous.tri * 3 + previous.edge) * 4, tri0)
                    -- More than two polys on an edge is non-manifold. Keep the first pair stable.
                    edges[key] = false
                end
            end
        end
    end

    local bmin, bmax = Recast.calcBounds(verts)
    local nav = {
        vertices = verts,
        vertexCount = vertCount,
        triangles = tris,
        polyCount = triCount,
        walkableCount = walkableCount,
        areas = areas,
        flags = flags,
        polyTypes = polyTypes, polyValid = polyValid, groundPolyCount = triCount,
        neighbors = neighbors,
        centers = centers,
        bmin = bmin,
        bmax = bmax,
    }
    buildPolyBounds(nav)
    return nav
end

local function closestPointOnTriangle(p: vector, a: vector, b: vector, c: vector)
    -- Real-Time Collision Detection region tests, kept branchy on purpose: it avoids temporary
    -- tables and gives exact edge/vertex clamping for nearest-poly queries.
    local ab = b - a
    local ac = c - a
    local ap = p - a
    local d1 = vector.dot(ab, ap)
    local d2 = vector.dot(ac, ap)
    if d1 <= 0 and d2 <= 0 then return a end

    local bp = p - b
    local d3 = vector.dot(ab, bp)
    local d4 = vector.dot(ac, bp)
    if d3 >= 0 and d4 <= d3 then return b end

    local vc = d1 * d4 - d3 * d2
    if vc <= 0 and d1 >= 0 and d3 <= 0 then
        local v = d1 / (d1 - d3)
        return a + ab * v
    end

    local cp = p - c
    local d5 = vector.dot(ab, cp)
    local d6 = vector.dot(ac, cp)
    if d6 >= 0 and d5 <= d6 then return c end

    local vb = d5 * d2 - d1 * d6
    if vb <= 0 and d2 >= 0 and d6 <= 0 then
        local w = d2 / (d2 - d6)
        return a + ac * w
    end

    local va = d3 * d6 - d5 * d4
    if va <= 0 and (d4 - d3) >= 0 and (d5 - d6) >= 0 then
        local w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return b + (c - b) * w
    end

    local denom = 1 / (va + vb + vc)
    local v = vb * denom
    local w = vc * denom
    return a + ab * v + ac * w
end

local function closestPointSeg2D(p: vector, a: vector, b: vector)
    local dx, dz = b.X - a.X, b.Z - a.Z
    local denom = dx * dx + dz * dz
    local t = 0
    if denom > 0 then t = ((p.X - a.X) * dx + (p.Z - a.Z) * dz) / denom end
    t = math.clamp(t, 0, 1)
    local q = a + (b - a) * t
    local qx, qz = p.X - q.X, p.Z - q.Z
    return q, qx * qx + qz * qz, t
end

local function closestPointOnDetailEdges(nav, poly0, position, onlyBoundary)
    local dm=nav.detailMesh
    if dm==nil or poly0<0 or poly0>=dm.nmeshes then return nil end
    local mo=poly0*16
    local vertBase=buffer.readu32(dm.meshes,mo)
    local triBase=buffer.readu32(dm.meshes,mo+8)
    local triCount=buffer.readu32(dm.meshes,mo+12)
    local best,bestDist=nil,math.huge
    for t0=0,triCount-1 do
        local to=(triBase+t0)*4
        local ia=buffer.readu8(dm.tris,to); local ib=buffer.readu8(dm.tris,to+1); local ic=buffer.readu8(dm.tris,to+2)
        local flags=buffer.readu8(dm.tris,to+3)
        local ids={ia,ib,ic}
        local vs={vertexAt(dm.verts,vertBase+ia),vertexAt(dm.verts,vertBase+ib),vertexAt(dm.verts,vertBase+ic)}
        local j=2
        for k=0,2 do
            local edgeFlags=bit32.band(bit32.rshift(flags,j*2),0x3)
            if bit32.band(edgeFlags,0x1)~=0 or (not onlyBoundary and ids[j+1]>=ids[k+1]) then
                local q,d=closestPointSeg2D(position,vs[j+1],vs[k+1])
                if d<bestDist then best,bestDist=q,d end
            end
            j=k
        end
    end
    return best
end

local function pointInPolyXZ(nav, poly0, p: vector)
    local inside = false
    local nv = polyVertCount(nav, poly0)
    local j = nv - 1
    for i = 0, nv - 1 do
        local vi = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, i))
        local vj = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, j))
        if (vi.Z > p.Z) ~= (vj.Z > p.Z) then
            local x = (vj.X - vi.X) * (p.Z - vi.Z) / (vj.Z - vi.Z) + vi.X
            if p.X < x then inside = not inside end
        end
        j = i
    end
    return inside
end

local function heightOnTriangleXZ(p: vector, a: vector, b: vector, c: vector)
    local v0x, v0z = b.X - a.X, b.Z - a.Z
    local v1x, v1z = c.X - a.X, c.Z - a.Z
    local v2x, v2z = p.X - a.X, p.Z - a.Z
    local denom = v0x * v1z - v1x * v0z
    if math.abs(denom) < 1e-8 then return nil end
    local v = (v2x * v1z - v1x * v2z) / denom
    local w = (v0x * v2z - v2x * v0z) / denom
    local u = 1 - v - w
    if u < -1e-5 or v < -1e-5 or w < -1e-5 then return nil end
    return a.Y * u + b.Y * v + c.Y * w
end

local function extentAllows(extents, p: vector, closest: vector)
    if extents == nil then return true end
    return math.abs(closest.X - p.X) <= extents.X
       and math.abs(closest.Y - p.Y) <= extents.Y
       and math.abs(closest.Z - p.Z) <= extents.Z
end

function Detour.closestPointOnPoly(nav, ref, position: vector)
    -- dtNavMesh stores detail meshes per tile, not in the aggregate ref/index view.  Resolve tiled
    -- refs back to their owning tile so serialized dtPolyDetail data remains authoritative.
    if nav._tiled then
        local salt,it,ip=Detour.decodePolyId(nav,ref)
        local tile=it<nav.maxTiles and nav.tiles[it+1] or nil
        if tile==nil or tile.salt~=salt or ip>=tile.polyCount then return nil,false end
        return Detour.closestPointOnPoly(tile.nav,ip+1,position)
    end
    local poly0 = refToPoly0(nav, ref)
    if not isWalkable(nav, poly0) then return nil,false end

    -- Match dtNavMesh::closestPointOnPoly(): if the xz point is over a ground polygon, detail data
    -- owns the surface height.  This is materially different from projecting onto the coarse fan.
    if polyType(nav,poly0)~=Detour.DT_POLYTYPE_OFFMESH_CONNECTION then
        local h=Detour.getPolyHeight(nav,ref,position)
        if h~=nil then return vector.create(position.X,h,position.Z),true end
        local q=closestPointOnDetailEdges(nav,poly0,position,true)
        if q~=nil then return q,false end
    else
        local a=vertexAt(nav.vertices,polyVertexIndex(nav,poly0,0))
        local b=vertexAt(nav.vertices,polyVertexIndex(nav,poly0,1))
        local q=closestPointSeg2D(position,a,b)
        return q,false
    end

    -- Meshes without detail data use the coarse polygon boundary as the compatibility fallback.
    local nv=polyVertCount(nav,poly0)
    local best,bestDist=nil,math.huge
    for edge0=0,nv-1 do
        local a=vertexAt(nav.vertices,polyVertexIndex(nav,poly0,edge0))
        local b=vertexAt(nav.vertices,polyVertexIndex(nav,poly0,(edge0+1)%nv))
        local q,d=closestPointSeg2D(position,a,b)
        if d<bestDist then best,bestDist=q,d end
    end
    return best,false
end

function Detour.closestPointOnPolyBoundary(nav, ref, position)
    local poly0 = refToPoly0(nav, ref)
    if not isWalkable(nav, poly0) then return nil end
    if polyType(nav, poly0) == Detour.DT_POLYTYPE_OFFMESH_CONNECTION then
        local a=vertexAt(nav.vertices,polyVertexIndex(nav,poly0,0))
        local b=vertexAt(nav.vertices,polyVertexIndex(nav,poly0,1))
        local q=closestPointSeg2D(position,a,b)
        return q
    end
    if pointInPolyXZ(nav, poly0, position) then return position end
    local nv = polyVertCount(nav, poly0)
    local best, bestDist = nil, math.huge
    for edge0 = 0, nv - 1 do
        local a = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, edge0))
        local b = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, (edge0 + 1) % nv))
        local q, d = closestPointSeg2D(position, a, b)
        if d < bestDist then best, bestDist = q, d end
    end
    return best
end

function Detour.getPolyHeight(nav, ref, position)
    if nav._tiled then
        local salt,it,ip=Detour.decodePolyId(nav,ref)
        local tile=it<nav.maxTiles and nav.tiles[it+1] or nil
        if tile==nil or tile.salt~=salt or ip>=tile.polyCount then return nil end
        return Detour.getPolyHeight(tile.nav,ip+1,position)
    end
    local poly0 = refToPoly0(nav, ref)
    if not isWalkable(nav, poly0) then return nil end
    if polyType(nav, poly0) == Detour.DT_POLYTYPE_OFFMESH_CONNECTION then
        local a=vertexAt(nav.vertices,polyVertexIndex(nav,poly0,0))
        local b=vertexAt(nav.vertices,polyVertexIndex(nav,poly0,1))
        local _,_,t=closestPointSeg2D(position,a,b)
        return a.Y+(b.Y-a.Y)*t
    end
    if not pointInPolyXZ(nav, poly0, position) then return nil end

    local dm = nav.detailMesh
    if dm ~= nil and nav.polyIndices ~= nil and poly0 < dm.nmeshes then
        local mo = poly0 * 16
        local vertBase = buffer.readu32(dm.meshes, mo)
        local triBase = buffer.readu32(dm.meshes, mo + 8)
        local triCount = buffer.readu32(dm.meshes, mo + 12)
        for t0 = 0, triCount - 1 do
            local to = (triBase + t0) * 4
            local ia = buffer.readu8(dm.tris, to)
            local ib = buffer.readu8(dm.tris, to + 1)
            local ic = buffer.readu8(dm.tris, to + 2)
            local a = vertexAt(dm.verts, vertBase + ia)
            local b = vertexAt(dm.verts, vertBase + ib)
            local c = vertexAt(dm.verts, vertBase + ic)
            local h = heightOnTriangleXZ(position, a, b, c)
            if h ~= nil then return h end
        end
        -- Upstream falls back to the closest detail edge for degenerate/on-edge triangle cases.
        local q=closestPointOnDetailEdges(nav,poly0,position,false)
        if q~=nil then return q.Y end
    end

    local nv = polyVertCount(nav, poly0)
    local a = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, 0))
    for i = 1, nv - 2 do
        local b = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, i))
        local c = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, i + 1))
        local h = heightOnTriangleXZ(position, a, b, c)
        if h ~= nil then return h end
    end
    return nil
end

function Detour.isValidPolyRef(nav, ref, filter)
    local poly0 = refToPoly0(nav, ref)
    return poly0 >= 0 and poly0 < nav.polyCount and passFilter(nav, poly0, filter)
end

function Detour.queryPolygons(nav, center, halfExtents, filter)
    local qmin, qmax = center - halfExtents, center + halfExtents
    local out = {}
    if nav._tiled and nav.tileWidth>0 and nav.tileHeight>0 then
        local minx=math.floor((qmin.X-nav.orig.X)/nav.tileWidth); local maxx=math.floor((qmax.X-nav.orig.X)/nav.tileWidth)
        local miny=math.floor((qmin.Z-nav.orig.Z)/nav.tileHeight); local maxy=math.floor((qmax.Z-nav.orig.Z)/nav.tileHeight)
        for ty=miny,maxy do for tx=minx,maxx do
            local tiles=Detour.getTilesAt(nav,tx,ty)
            for _,tile in tiles do
                local localRefs=Detour.queryPolygons(tile.nav,center,halfExtents,nil)
                for _,localRef in localRefs do
                    local local0=localRef-1
                    local ref=Detour.encodePolyId(nav,tile.salt,tile.index,local0)
                    local gp=refToPoly0(nav,ref)
                    if gp>=0 and passFilter(nav,gp,filter) then out[#out+1]=ref end
                end
            end
        end end
        return out
    end
    local nodes, nodeCount, qfac = nav.bvTree, nav.bvNodeCount, nav.bvQuantFactor
    if nodes ~= nil and nodeCount ~= nil and qfac ~= nil then
        local bminx = bit32.band(math.clamp(math.floor((math.clamp(qmin.X,nav.bmin.X,nav.bmax.X)-nav.bmin.X)*qfac),0,0xffff),0xfffe)
        local bminy = bit32.band(math.clamp(math.floor((math.clamp(qmin.Y,nav.bmin.Y,nav.bmax.Y)-nav.bmin.Y)*qfac),0,0xffff),0xfffe)
        local bminz = bit32.band(math.clamp(math.floor((math.clamp(qmin.Z,nav.bmin.Z,nav.bmax.Z)-nav.bmin.Z)*qfac),0,0xffff),0xfffe)
        local bmaxx = bit32.bor(math.clamp(math.floor((math.clamp(qmax.X,nav.bmin.X,nav.bmax.X)-nav.bmin.X)*qfac+1),0,0xffff),1)
        local bmaxy = bit32.bor(math.clamp(math.floor((math.clamp(qmax.Y,nav.bmin.Y,nav.bmax.Y)-nav.bmin.Y)*qfac+1),0,0xffff),1)
        local bmaxz = bit32.bor(math.clamp(math.floor((math.clamp(qmax.Z,nav.bmin.Z,nav.bmax.Z)-nav.bmin.Z)*qfac+1),0,0xffff),1)
        local node0 = 0
        while node0 < nodeCount do
            local o = node0 * BV_NODE_BYTES
            local index = buffer.readi32(nodes, o + 12)
            local overlap = overlapQuantBounds(nodes,node0,bminx,bminy,bminz,bmaxx,bmaxy,bmaxz)
            local leaf = index >= 0
            if leaf and overlap and passFilter(nav,index,filter) then
                out[#out + 1] = poly0ToRef(nav,index)
            end
            if overlap or leaf then node0 += 1 else node0 += -index end
        end
        return out
    end
    for poly0 = 0, nav.polyCount - 1 do
        if polyType(nav, poly0) == Detour.DT_POLYTYPE_GROUND and passFilter(nav, poly0, filter) and boundsOverlap(nav, poly0, qmin, qmax) then out[#out + 1] = poly0ToRef(nav,poly0) end
    end
    return out
end

local function nearestCandidate(nav, position: vector, extents, tri0, bestRef, bestPoint, bestDistance, filter)
    if tri0 < 0 or tri0 >= nav.polyCount then return bestRef, bestPoint, bestDistance end
    if filter == nil then
        if not isWalkable(nav, tri0) then return bestRef, bestPoint, bestDistance end
    elseif not passFilter(nav, tri0, filter) then
        return bestRef, bestPoint, bestDistance
    end
    local point,over
    if nav.polyIndices == nil then
        local ia, ib, ic = triIndices(nav, tri0)
        point = closestPointOnTriangle(position, vertexAt(nav.vertices, ia), vertexAt(nav.vertices, ib), vertexAt(nav.vertices, ic))
        over = false
    else
        point,over = Detour.closestPointOnPoly(nav, poly0ToRef(nav,tri0), position)
    end
    if point ~= nil then
        local ref=poly0ToRef(nav,tri0)
        local d
        if over then
            local climb=nav.walkableClimb or 0
            if nav._tiled then
                local salt,it=Detour.decodePolyId(nav,ref); local tile=it<nav.maxTiles and nav.tiles[it+1] or nil
                if tile~=nil and tile.salt==salt then climb=tile.nav.walkableClimb or climb end
            end
            local dy=math.abs(position.Y-point.Y)-climb
            d=if dy>0 then dy*dy else 0
        else
            d=Common.distanceSquared(position,point)
        end
        if d < bestDistance then return ref, point, d end
    end
    return bestRef, bestPoint, bestDistance
end

local function nearestCompact(nav, position: vector, extents, filter)
    local chf = nav.compact
    local map = nav.spanToBase
    local bestRef, bestPoint, bestDistance = nil, nil, math.huge

    local function visitCell(x, z)
        if x < 0 or z < 0 or x >= chf.width or z >= chf.height then return end
        local cell = x + z * chf.width
        local first = buffer.readu32(chf.cellIndex, cell * 4)
        local count = buffer.readu8(chf.cellCount, cell)
        for li = 0, count - 1 do
            local base = buffer.readi32(map, (first + li) * 4)
            if base >= 0 then
                bestRef, bestPoint, bestDistance = nearestCandidate(nav, position, extents, base, bestRef, bestPoint, bestDistance, filter)
                bestRef, bestPoint, bestDistance = nearestCandidate(nav, position, extents, base + 1, bestRef, bestPoint, bestDistance, filter)
            end
        end
    end

    if extents ~= nil then
        local minX = Common.clamp(math.floor((position.X - extents.X - chf.bmin.X) / chf.cs), 0, chf.width - 1)
        local maxX = Common.clamp(math.floor((position.X + extents.X - chf.bmin.X) / chf.cs), 0, chf.width - 1)
        local minZ = Common.clamp(math.floor((position.Z - extents.Z - chf.bmin.Z) / chf.cs), 0, chf.height - 1)
        local maxZ = Common.clamp(math.floor((position.Z + extents.Z - chf.bmin.Z) / chf.cs), 0, chf.height - 1)
        for z = minZ, maxZ do for x = minX, maxX do visitCell(x, z) end end
        return bestRef, bestPoint, bestDistance
    end

    local cx = Common.clamp(math.floor((position.X - chf.bmin.X) / chf.cs), 0, chf.width - 1)
    local cz = Common.clamp(math.floor((position.Z - chf.bmin.Z) / chf.cs), 0, chf.height - 1)
    local maxRadius = math.max(chf.width, chf.height)
    for radius = 0, maxRadius do
        if radius == 0 then
            visitCell(cx, cz)
        else
            local x0, x1 = cx - radius, cx + radius
            local z0, z1 = cz - radius, cz + radius
            for x = x0, x1 do visitCell(x, z0); visitCell(x, z1) end
            for z = z0 + 1, z1 - 1 do visitCell(x0, z); visitCell(x1, z) end
        end
        if bestRef ~= nil then
            -- Any cell beyond this ring has at least roughly this much horizontal separation;
            -- keep expanding only while its lower bound can still beat the 3D candidate.
            local lower = math.max(0, radius - 1) * chf.cs
            if lower * lower > bestDistance then break end
        end
    end
    return bestRef, bestPoint, bestDistance
end

function Detour.findNearestPoly(nav, position, extents, filter)
    if nav.compact ~= nil and nav.spanToBase ~= nil then
        return nearestCompact(nav, position, extents, filter)
    end
    local bestRef, bestPoint, bestDistance = nil, nil, math.huge
    if extents ~= nil then
        local candidates = Detour.queryPolygons(nav, position, extents, filter)
        for _, ref in candidates do
            bestRef, bestPoint, bestDistance = nearestCandidate(nav, position, extents, refToPoly0(nav,ref), bestRef, bestPoint, bestDistance, filter)
        end
        return bestRef, bestPoint, bestDistance
    end
    for tri0 = 0, nav.polyCount - 1 do
        bestRef, bestPoint, bestDistance = nearestCandidate(nav, position, extents, tri0, bestRef, bestPoint, bestDistance, filter)
    end
    return bestRef, bestPoint, bestDistance
end

local function reconstruct(nav, parent, start0, end0)
    local reverse = {}
    local current = end0
    while current >= 0 do
        reverse[#reverse + 1] = poly0ToRef(nav,current)
        if current == start0 then break end
        current = buffer.readi32(parent, current * 4)
    end
    if reverse[#reverse] ~= poly0ToRef(nav,start0) then return nil end
    local path = table.create(#reverse)
    for i = 1, #reverse do path[i] = reverse[#reverse - i + 1] end
    return path
end

local function queryScratch(nav)
    local scratch = nav._queryScratch
    if scratch ~= nil and scratch.capacity >= nav.polyCount then return scratch end
    local n = nav.polyCount
    local f = buffer.create(n * 8)
    scratch = {
        capacity = n,
        g = buffer.create(n * 8),
        f = f,
        parent = buffer.create(n * 4),
        state = buffer.create(n),
        pos = buffer.create(n * 12),
        touched = {},
    }
    scratch.heap = Common.scoreHeapNew()
    nav._queryScratch = scratch
    return scratch
end

local function compactConnection(chf, span0, dir0)
    return bit32.band(bit32.rshift(buffer.readu32(chf.con, span0 * 4), dir0 * 6), 0x3f)
end

local function compactScratch(nav)
    local chf = nav.compact
    local scratch = nav._compactQueryScratch
    if scratch ~= nil and scratch.capacity >= chf.spanCount then return scratch end
    local n = chf.spanCount
    -- Generation stamps avoid O(n) clears and touched-node bookkeeping between queries.
    scratch = {
        capacity = n,
        g = buffer.create(n * 8),
        parent = buffer.create(n * 4),
        state = buffer.create(n),
        seen = buffer.create(n * 4),
        generation = 0,
        heap = Common.scoreHeapNew(),
    }
    nav._compactQueryScratch = scratch
    return scratch
end

local function compactHeuristic(chf, span0, end0)
    local dx = (buffer.readu32(chf.spanX, span0 * 4) - buffer.readu32(chf.spanX, end0 * 4)) * chf.cs
    local dz = (buffer.readu32(chf.spanZ, span0 * 4) - buffer.readu32(chf.spanZ, end0 * 4)) * chf.cs
    local dy = (buffer.readu16(chf.y, span0 * 2) - buffer.readu16(chf.y, end0 * 2)) * chf.ch
    return math.sqrt(dx * dx + dy * dy + dz * dz)
end

local function compactSpanPath(nav, startSpan, endSpan, filter)
    if startSpan == endSpan then return {startSpan} end
    local chf = nav.compact
    local scratch = compactScratch(nav)
    local g, parent, state, seen = scratch.g, scratch.parent, scratch.state, scratch.seen
    local hasFilter = filter ~= nil
    local heuristicScale = if hasFilter then filter.minAreaCost else 1

    local generation = scratch.generation + 1
    if generation >= 0xffffffff then
        buffer.fill(seen, 0, 0, buffer.len(seen))
        generation = 1
    end
    scratch.generation = generation
    table.clear(scratch.heap.nodes)
    table.clear(scratch.heap.scores)

    buffer.writeu32(seen, startSpan * 4, generation)
    buffer.writef64(g, startSpan * 8, 0)
    buffer.writei32(parent, startSpan * 4, -1)
    buffer.writeu8(state, startSpan, 1)
    local startH = compactHeuristic(chf, startSpan, endSpan) * heuristicScale
    Common.scoreHeapPush(scratch.heap, startSpan, startH)

    while true do
        local current = Common.scoreHeapPop(scratch.heap)
        if current == nil then return nil end
        if buffer.readu32(seen, current * 4) ~= generation or buffer.readu8(state, current) == 2 then continue end
        if current == endSpan then
            local reverse = {}
            local span = endSpan
            while span >= 0 do
                reverse[#reverse + 1] = span
                if span == startSpan then break end
                span = buffer.readi32(parent, span * 4)
            end
            if reverse[#reverse] ~= startSpan then return nil end
            local path = table.create(#reverse)
            for i = 1, #reverse do path[i] = reverse[#reverse - i + 1] end
            return path
        end
        buffer.writeu8(state, current, 2)

        local currentG = buffer.readf64(g, current * 8)
        local y = buffer.readu16(chf.y, current * 2)
        local currentAreaCost = 1
        if hasFilter then
            local currentBase = buffer.readi32(nav.spanToBase, current * 4)
            currentAreaCost = buffer.readf32(filter.areaCosts, buffer.readu8(nav.areas, currentBase) * 4)
        end
        for dir0 = 0, 3 do
            local neighbor = buffer.readi32(nav.spanNeighbors, (current * 4 + dir0) * 4)
            if neighbor < 0 then continue end
            local wasSeen = buffer.readu32(seen, neighbor * 4) == generation
            if wasSeen and buffer.readu8(state, neighbor) == 2 then continue end
            if hasFilter then
                local neighborBase = buffer.readi32(nav.spanToBase, neighbor * 4)
                if neighborBase < 0 or not passFilter(nav, neighborBase, filter) then continue end
            end

            if not wasSeen then
                buffer.writeu32(seen, neighbor * 4, generation)
                buffer.writef64(g, neighbor * 8, math.huge)
                buffer.writei32(parent, neighbor * 4, -1)
                buffer.writeu8(state, neighbor, 0)
            end
            local ny = buffer.readu16(chf.y, neighbor * 2)
            local vertical = (ny - y) * chf.ch
            local step = math.sqrt(chf.cs * chf.cs + vertical * vertical) * currentAreaCost
            local tentative = currentG + step
            if tentative < buffer.readf64(g, neighbor * 8) then
                buffer.writei32(parent, neighbor * 4, current)
                buffer.writef64(g, neighbor * 8, tentative)
                local score = tentative + compactHeuristic(chf, neighbor, endSpan) * heuristicScale
                buffer.writeu8(state, neighbor, 1)
                Common.scoreHeapPush(scratch.heap, neighbor, score)
            end
        end
    end
end

local function compactCorridor(nav, startRef, endRef, filter)
    local start0, end0 = refToPoly0(nav,startRef), refToPoly0(nav,endRef)
    local startSpan = buffer.readu32(nav.sourceSpan, start0 * 4)
    local endSpan = buffer.readu32(nav.sourceSpan, end0 * 4)
    if startSpan == endSpan then
        if startRef == endRef then return {startRef} end
        return {startRef, endRef}
    end

    local spans = compactSpanPath(nav, startSpan, endSpan, filter)
    if spans == nil then return nil end
    local chf = nav.compact
    local corridor = {startRef}
    local currentRef = startRef
    for i = 1, #spans - 1 do
        local span = spans[i]
        local nextSpan = spans[i + 1]
        local x = buffer.readu32(chf.spanX, span * 4)
        local z = buffer.readu32(chf.spanZ, span * 4)
        local nx = buffer.readu32(chf.spanX, nextSpan * 4)
        local nz = buffer.readu32(chf.spanZ, nextSpan * 4)
        local dir0 = if nx < x then 0 elseif nz > z then 1 elseif nx > x then 2 else 3
        local exitOffset = if dir0 == 1 or dir0 == 2 then 1 else 0
        local base = buffer.readi32(nav.spanToBase, span * 4)
        local nextBase = buffer.readi32(nav.spanToBase, nextSpan * 4)
        if base < 0 or nextBase < 0 then return nil end
        local exitRef = poly0ToRef(nav,base + exitOffset)
        if currentRef ~= exitRef then
            corridor[#corridor + 1] = exitRef
            currentRef = exitRef
        end
        local enterRef = poly0ToRef(nav,nextBase + (1 - exitOffset))
        if currentRef ~= enterRef then
            corridor[#corridor + 1] = enterRef
            currentRef = enterRef
        end
    end
    if currentRef ~= endRef then corridor[#corridor + 1] = endRef end
    return corridor
end

function Detour.findPath(nav, startRef, endRef, filter, endPos, positionedFilter, maxPath, maxNodes)
    -- Overload the functional Luau API with Detour's position-aware signature:
    -- findPath(nav,startRef,endRef,startPos,endPos,filter[,maxPath[,maxNodes]]).
    -- Existing ref-only callers keep the compatibility center-based search below.
    if typeof(filter)=='vector' and typeof(endPos)=='vector' then
        return Detour.findPathPositioned(nav,startRef,endRef,filter,endPos,positionedFilter,maxPath,maxNodes)
    end
    local start0, end0 = refToPoly0(nav,startRef), refToPoly0(nav,endRef)
    if filter == nil then
        if not isWalkable(nav, start0) or not isWalkable(nav, end0) then return nil end
    elseif not passFilter(nav, start0, filter) or not passFilter(nav, end0, filter) then
        return nil
    end
    if start0 == end0 then return {startRef} end
    if nav.compact ~= nil and nav.sourceSpan ~= nil and nav.spanToBase ~= nil and nav.spanNeighbors ~= nil
        and nav.compact.spanX ~= nil and nav.compact.spanZ ~= nil then
        return compactCorridor(nav, startRef, endRef, filter)
    end

    local scratch = queryScratch(nav)
    local g, f, parent, state = scratch.g, scratch.f, scratch.parent, scratch.state
    for _, node in scratch.touched do buffer.writeu8(state, node, 0) end
    table.clear(scratch.touched)
    table.clear(scratch.heap.nodes)
    table.clear(scratch.heap.scores)

    local function touch(node)
        if buffer.readu8(state, node) == 0 then
            buffer.writef64(g, node * 8, math.huge)
            buffer.writef64(f, node * 8, math.huge)
            buffer.writei32(parent, node * 4, -1)
            scratch.touched[#scratch.touched + 1] = node
        end
    end

    local endCenter = polyCenter(nav, end0)
    local heuristicScale = if filter ~= nil then filter.minAreaCost else 1
    touch(start0)
    buffer.writef64(g, start0 * 8, 0)
    buffer.writef64(f, start0 * 8, math.sqrt(Common.distanceSquared2D(polyCenter(nav, start0), endCenter)) * heuristicScale)
    buffer.writeu8(state, start0, 1)
    Common.scoreHeapPush(scratch.heap, start0, buffer.readf64(f, start0 * 8))

    while true do
        local current = Common.scoreHeapPop(scratch.heap)
        if current == nil then return nil end
        if buffer.readu8(state, current) == 2 then continue end
        if current == end0 then return reconstruct(nav, parent, start0, end0) end
        buffer.writeu8(state, current, 2)

        local currentCenter = polyCenter(nav, current)
        local currentG = buffer.readf64(g, current * 8)
        local nv = polyVertCount(nav, current)
        -- Keep the overwhelmingly common ground-edge loop branch-free.  Off-mesh/external links
        -- live in a second sparse chain and therefore impose only one nil/empty check per node.
        for edge0 = 0, nv - 1 do
            local neighbor = polyNeighbor(nav, current, edge0)
            local allowed = neighbor >= 0 and buffer.readu8(state, neighbor) ~= 2
            if allowed then
                if filter == nil then allowed = isWalkable(nav, neighbor) else allowed = passFilter(nav, neighbor, filter) end
            end
            if allowed then
                touch(neighbor)
                local neighborCenter = polyCenter(nav, neighbor)
                local step = math.sqrt(Common.distanceSquared2D(currentCenter, neighborCenter))
                if filter ~= nil then step *= areaCost(nav, current, filter) end
                local tentative = currentG + step
                if tentative < buffer.readf64(g, neighbor * 8) then
                    buffer.writei32(parent, neighbor * 4, current)
                    buffer.writef64(g, neighbor * 8, tentative)
                    local heuristic = math.sqrt(Common.distanceSquared2D(neighborCenter, endCenter)) * heuristicScale
                    buffer.writef64(f, neighbor * 8, tentative + heuristic)
                    buffer.writeu8(state, neighbor, 1)
                    Common.scoreHeapPush(scratch.heap, neighbor, tentative + heuristic)
                end
            end
        end
        local link0 = firstExtraLink(nav, current)
        while link0 >= 0 do
            local neighbor = extraLinkTarget(nav, link0)
            local allowed = neighbor >= 0 and buffer.readu8(state, neighbor) ~= 2
            if allowed then
                if filter == nil then allowed = isWalkable(nav, neighbor) else allowed = passFilter(nav, neighbor, filter) end
            end
            if allowed then
                touch(neighbor)
                local neighborCenter = polyCenter(nav, neighbor)
                local step = math.sqrt(Common.distanceSquared2D(currentCenter, neighborCenter))
                if filter ~= nil then step *= areaCost(nav, current, filter) end
                local tentative = currentG + step
                if tentative < buffer.readf64(g, neighbor * 8) then
                    buffer.writei32(parent, neighbor * 4, current)
                    buffer.writef64(g, neighbor * 8, tentative)
                    local heuristic = math.sqrt(Common.distanceSquared2D(neighborCenter, endCenter)) * heuristicScale
                    buffer.writef64(f, neighbor * 8, tentative + heuristic)
                    buffer.writeu8(state, neighbor, 1)
                    Common.scoreHeapPush(scratch.heap, neighbor, tentative + heuristic)
                end
            end
            link0 = extraLinkNext(nav, link0)
        end
    end
end

local function portalBetween(nav, fromRef, toRef)
    local from0, to0 = refToPoly0(nav,fromRef), refToPoly0(nav,toRef)
    local fromType,toType=polyType(nav,from0),polyType(nav,to0)
    if fromType==Detour.DT_POLYTYPE_OFFMESH_CONNECTION then
        local link0=findExtraLink(nav,from0,to0)
        if link0<0 then return nil end
        local v=vertexAt(nav.vertices,polyVertexIndex(nav,from0,extraLinkEdge(nav,link0)))
        return v,v
    end
    if toType==Detour.DT_POLYTYPE_OFFMESH_CONNECTION then
        local link0=findExtraLink(nav,to0,from0)
        if link0<0 then return nil end
        local v=vertexAt(nav.vertices,polyVertexIndex(nav,to0,extraLinkEdge(nav,link0)))
        return v,v
    end
    local edge = -1
    local portalLink = -1
    local nv = polyVertCount(nav, from0)
    for edge0 = 0, nv - 1 do
        if polyNeighbor(nav, from0, edge0) == to0 then edge = edge0; break end
    end
    if edge<0 then
        local link0=findExtraLink(nav,from0,to0)
        if link0>=0 and extraLinkEdge(nav,link0)<0xff then edge=extraLinkEdge(nav,link0); portalLink=link0 end
    end
    if edge < 0 then return nil end
    local ia = polyVertexIndex(nav, from0, edge)
    local ib = polyVertexIndex(nav, from0, (edge + 1) % nv)
    local va, vb = vertexAt(nav.vertices, ia), vertexAt(nav.vertices, ib)
    if portalLink>=0 and extraLinkSide(nav,portalLink)~=0xff then
        local t0,t1=extraLinkBMin(nav,portalLink)/255,extraLinkBMax(nav,portalLink)/255
        local delta=vb-va
        va=va+delta*t0; vb=va+delta*(if t1>=t0 then (t1-t0)/(1-t0 ~= 0 and 1-t0 or 1) else 0)
        -- Recompute directly from the original edge to avoid accumulated interpolation error.
        local oa,ob=vertexAt(nav.vertices,ia),vertexAt(nav.vertices,ib); local od=ob-oa
        va=oa+od*t0; vb=oa+od*t1
    end
    local fromCenter, toCenter = polyCenter(nav, from0), polyCenter(nav, to0)
    local dx, dz = toCenter.X - fromCenter.X, toCenter.Z - fromCenter.Z
    local ax, az = va.X - fromCenter.X, va.Z - fromCenter.Z
    local side = dx * az - dz * ax
    if side >= 0 then return va, vb end
    return vb, va
end

function Detour.getPortalPoints(nav, fromRef, toRef)
    return portalBetween(nav, fromRef, toRef)
end

function Detour.getPolyWallSegments(nav, ref, filter, includePortals)
    local poly0 = refToPoly0(nav, ref)
    if not isWalkable(nav, poly0) then return nil end
    local out = {}
    local nv = polyVertCount(nav, poly0)
    for edge0 = 0, nv - 1 do
        local a = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, edge0))
        local b = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, (edge0 + 1) % nv))
        local intervals=edgeTraversableIntervals(nav,poly0,edge0,filter)
        if #intervals==0 then
            out[#out+1]={a=a,b=b,ref=0}
            continue
        end
        local cursor=0
        for _,iv in intervals do
            local lo,hi=math.clamp(iv.tmin/255,0,1),math.clamp(iv.tmax/255,0,1)
            if lo>cursor then appendEdgeSegment(out,a,b,cursor,lo,0) end
            if includePortals==true and hi>lo then appendEdgeSegment(out,a,b,lo,hi,iv.ref) end
            cursor=math.max(cursor,hi)
        end
        if cursor<1 then appendEdgeSegment(out,a,b,cursor,1,0) end
    end
    return out
end


local function intersectSegments2D(p: vector, q: vector, a: vector, b: vector)
    local rx, rz = q.X - p.X, q.Z - p.Z
    local sx, sz = b.X - a.X, b.Z - a.Z
    local denom = rx * sz - rz * sx
    if math.abs(denom) < 1e-8 then return nil end
    local apx, apz = a.X - p.X, a.Z - p.Z
    local t = (apx * sz - apz * sx) / denom
    local u = (apx * rz - apz * rx) / denom
    if t < -1e-6 or t > 1 + 1e-6 or u < -1e-6 or u > 1 + 1e-6 then return nil end
    return t, u
end

local function inwardEdgeNormal(nav, poly0, edge0, a: vector, b: vector)
    local ex, ez = b.X - a.X, b.Z - a.Z
    local len = math.sqrt(ex * ex + ez * ez)
    if len <= 1e-8 then return vector.zero end
    local n = vector.create(ez / len, 0, -ex / len)
    local mid = (a + b) * 0.5
    local center = polyCenter(nav, poly0)
    if vector.dot(n, center - mid) < 0 then n = -n end
    return n
end

function Detour.raycast(nav, startRef, startPos, endPos, filter, options, prevRef)
    if not Detour.isValidPolyRef(nav, startRef, filter) then return nil, 'invalid start polygon' end
    if prevRef ~= nil and prevRef ~= 0 and not Detour.isValidPolyRef(nav, prevRef, nil) then return nil, 'invalid previous polygon' end
    options = options or 0
    local useCosts = bit32.band(options, Detour.DT_RAYCAST_USE_COSTS) ~= 0
    local current = refToPoly0(nav,startRef)
    local currentT = 0
    local path = {}
    local pathCost = 0
    local maxSteps = nav.polyCount + 8

    for _ = 1, maxSteps do
        path[#path + 1] = poly0ToRef(nav,current)
        if pointInPolyXZ(nav, current, endPos) then
            if useCosts then
                local from = startPos + (endPos - startPos) * currentT
                pathCost += vector.magnitude(endPos - from) * areaCost(nav, current, filter)
            end
            return {
                t = math.huge,
                hitNormal = vector.zero,
                hitPos = endPos,
                path = path,
                hitEdge = -1,
                hitEdgeIndex = -1,
                pathCost = pathCost,
            }
        end

        local bestT, bestU, bestEdge, bestA, bestB = math.huge, 0, -1, nil, nil
        local nv = polyVertCount(nav, current)
        for edge0 = 0, nv - 1 do
            local a = vertexAt(nav.vertices, polyVertexIndex(nav, current, edge0))
            local b = vertexAt(nav.vertices, polyVertexIndex(nav, current, (edge0 + 1) % nv))
            local t,u = intersectSegments2D(startPos, endPos, a, b)
            if t ~= nil and t > currentT + 1e-5 and t < bestT then
                bestT, bestU, bestEdge, bestA, bestB = t, u or 0, edge0, a, b
            end
        end
        if bestEdge < 0 then
            local hitPos = startPos + (endPos - startPos) * currentT
            return {
                t = currentT,
                hitNormal = vector.zero,
                hitPos = hitPos,
                path = path,
                hitEdge = -1,
                hitEdgeIndex = -1,
                pathCost = pathCost,
            }
        end

        local hitPos = startPos + (endPos - startPos) * bestT
        if useCosts then
            local from = startPos + (endPos - startPos) * currentT
            pathCost += vector.magnitude(hitPos - from) * areaCost(nav, current, filter)
        end

        local neighbor = linkedNeighborAtEdgeT(nav, current, bestEdge, bestU, filter)
        if neighbor >= 0 then
            current = neighbor
            currentT = bestT
        else
            return {
                t = math.clamp(bestT, 0, 1),
                hitNormal = inwardEdgeNormal(nav, current, bestEdge, bestA, bestB),
                hitPos = hitPos,
                path = path,
                hitEdge = bestEdge,
                hitEdgeIndex = bestEdge,
                pathCost = pathCost,
            }
        end
    end
    return nil, 'raycast exceeded polygon traversal limit'
end

local function slicedNodeChain(state, target0)
    local reverse, current = {}, target0
    while current ~= nil and current >= 0 do
        reverse[#reverse + 1] = current
        if current == state.start0 then break end
        current = state.parent[current]
    end
    if reverse[#reverse] ~= state.start0 then return nil end
    local chain = table.create(#reverse)
    for i = 1, #reverse do chain[i] = reverse[#reverse - i + 1] end
    return chain
end

local function slicedReconstruct(nav, state, target0)
    local chain = slicedNodeChain(state, target0)
    if chain == nil then return nil end
    local path = {}
    local function appendRef(ref)
        if #path == 0 or path[#path] ~= ref then path[#path + 1] = ref end
    end
    for i = 1, #chain - 1 do
        local node0, next0 = chain[i], chain[i + 1]
        if state.detached[next0] then
            local hit = Detour.raycast(nav, poly0ToRef(nav,node0), state.pos[node0], state.pos[next0], state.filter, 0)
            if hit == nil or hit.path == nil then return nil end
            local n = #hit.path
            if n > 0 and hit.path[n] == poly0ToRef(nav,next0) then n -= 1 end
            for j = 1, n do appendRef(hit.path[j]) end
        else
            appendRef(poly0ToRef(nav,node0))
        end
    end
    appendRef(poly0ToRef(nav,chain[#chain]))
    return path
end

local function slicedWalkableRadius(nav, startRef)
    if nav._tiled then
        local salt, it = Detour.decodePolyId(nav,startRef)
        local tile = if it < nav.maxTiles then nav.tiles[it + 1] else nil
        if tile ~= nil and tile.salt == salt and tile.nav ~= nil then
            local tn = tile.nav
            if tn.walkableRadius ~= nil then return tn.walkableRadius end
            if tn.header ~= nil and tn.header.walkableRadius ~= nil then return tn.header.walkableRadius end
        end
    end
    if nav.walkableRadius ~= nil then return nav.walkableRadius end
    if nav.header ~= nil and nav.header.walkableRadius ~= nil then return nav.header.walkableRadius end
    return 0
end

function Detour.initSlicedFindPath(nav, startRef, endRef, startPos, endPos, filter, options)
    if not Detour.isValidPolyRef(nav, startRef, filter) or not Detour.isValidPolyRef(nav, endRef, filter)
        or startPos == nil or endPos == nil then
        nav._slicedQuery = nil
        return bit32.bor(Detour.DT_FAILURE, Detour.DT_INVALID_PARAM)
    end
    local start0, end0 = refToPoly0(nav,startRef), refToPoly0(nav,endRef)
    local state = {
        start0 = start0, end0 = end0, startPos = startPos, endPos = endPos,
        filter = filter, options = options or 0,
        g = {}, total = {}, parent = {}, pos = {}, closed = {}, detached = {},
        heap = Common.scoreHeapNew(), best = start0, bestHeuristic = math.huge,
        status = Detour.DT_IN_PROGRESS, raycastLimitSqr = math.huge,
    }
    state.g[start0], state.parent[start0], state.pos[start0] = 0, -1, startPos
    if bit32.band(state.options, Detour.DT_FINDPATH_ANY_ANGLE) ~= 0 then
        local radius = slicedWalkableRadius(nav,startRef)
        state.raycastLimitSqr = (radius * 50) * (radius * 50)
    end
    local h = math.sqrt(Common.distanceSquared(startPos, endPos)) * 0.999
    state.total[start0], state.bestHeuristic = h, h
    Common.scoreHeapPush(state.heap, start0, h)
    if start0 == end0 then state.status = Detour.DT_SUCCESS; state.best = start0 end
    nav._slicedQuery = state
    return state.status
end

function Detour.updateSlicedFindPath(nav, maxIter)
    local state = nav._slicedQuery
    if state == nil then return Detour.DT_FAILURE, 0 end
    if not Detour.statusInProgress(state.status) then return state.status, 0 end
    maxIter = math.max(0, math.floor(maxIter or 0))
    local done = 0

    while done < maxIter do
        local current, score = Common.scoreHeapPop(state.heap)
        if current == nil then
            state.status = Detour.DT_SUCCESS
            break
        end
        if state.closed[current] or score > (state.total[current] or math.huge) + 1e-9 then continue end
        done += 1
        state.closed[current] = true
        if current == state.end0 then state.best = current; state.status = Detour.DT_SUCCESS; break end

        local parent0 = state.parent[current]
        local grandpa0 = if parent0 ~= nil and parent0 >= 0 then state.parent[parent0] else -1
        local currentPos = state.pos[current]
        local tryLOS = false
        if bit32.band(state.options, Detour.DT_FINDPATH_ANY_ANGLE) ~= 0 and parent0 ~= nil and parent0 >= 0 then
            local parentPos = state.pos[parent0]
            local dx,dy,dz = parentPos.X-currentPos.X,parentPos.Y-currentPos.Y,parentPos.Z-currentPos.Z
            tryLOS = dx*dx+dy*dy+dz*dz < state.raycastLimitSqr
        end
        local nv = polyVertCount(nav, current)
        local edge0,link0=0,firstExtraLink(nav,current)
        while edge0<nv or link0>=0 do
            local neighbor
            if edge0<nv then neighbor=polyNeighbor(nav,current,edge0); edge0+=1 else neighbor=extraLinkTarget(nav,link0); link0=extraLinkNext(nav,link0) end
            if neighbor < 0 or neighbor == parent0 or not passFilter(nav, neighbor, state.filter) then continue end
            if parent0 ~= nil and parent0 >= 0 and state.parent[neighbor] ~= nil and state.parent[neighbor] == parent0 then continue end
            local unseen = state.g[neighbor] == nil
            if unseen and state.maxNodes ~= nil and state.nodeCount >= state.maxNodes then
                state.outOfNodes = true
                continue
            end
            local left, right = portalBetween(nav, poly0ToRef(nav,current), poly0ToRef(nav,neighbor))
            if left == nil then continue end
            local nodePos = state.pos[neighbor]
            if nodePos == nil then nodePos = (left + right) * 0.5 end

            local foundShortcut = false
            local tentative
            if tryLOS then
                local parentRef = poly0ToRef(nav,parent0)
                local grandpaRef = if grandpa0 ~= nil and grandpa0 >= 0 then poly0ToRef(nav,grandpa0) else 0
                local hit = Detour.raycast(nav,parentRef,state.pos[parent0],nodePos,state.filter,Detour.DT_RAYCAST_USE_COSTS,grandpaRef)
                if hit ~= nil and hit.t >= 1 then
                    tentative = state.g[parent0] + (hit.pathCost or 0)
                    foundShortcut = true
                end
            end
            if not foundShortcut then
                local dx,dy,dz=currentPos.X-nodePos.X,currentPos.Y-nodePos.Y,currentPos.Z-nodePos.Z
                tentative = state.g[current] + math.sqrt(dx*dx+dy*dy+dz*dz) * areaCost(nav,current,state.filter)
            end

            local ex,ey,ez=nodePos.X-state.endPos.X,nodePos.Y-state.endPos.Y,nodePos.Z-state.endPos.Z
            local heuristic
            if neighbor == state.end0 then
                tentative += math.sqrt(ex*ex+ey*ey+ez*ez) * areaCost(nav,neighbor,state.filter)
                heuristic = 0
            else
                heuristic = math.sqrt(ex*ex+ey*ey+ez*ez) * 0.999
            end
            local total = tentative + heuristic
            if total >= (state.total[neighbor] or math.huge) then continue end
            if unseen and state.maxNodes ~= nil then state.nodeCount += 1 end
            state.g[neighbor], state.total[neighbor], state.parent[neighbor], state.pos[neighbor] = tentative, total, (if foundShortcut then parent0 else current), nodePos
            state.detached[neighbor] = if foundShortcut then true else nil
            state.closed[neighbor] = nil
            Common.scoreHeapPush(state.heap, neighbor, total)
            if heuristic < state.bestHeuristic then state.bestHeuristic, state.best = heuristic, neighbor end
        end
    end

    if state.status == Detour.DT_IN_PROGRESS and #state.heap.nodes == 0 then state.status = Detour.DT_SUCCESS end
    local status=state.status
    if state.outOfNodes then status=bit32.bor(status,Detour.DT_OUT_OF_NODES) end
    return status, done
end

function Detour.finalizeSlicedFindPath(nav)
    local state = nav._slicedQuery
    if state == nil or Detour.statusFailed(state.status) then nav._slicedQuery = nil; return nil, Detour.DT_FAILURE end
    local target = state.best
    local status = Detour.DT_SUCCESS
    if target ~= state.end0 then status = bit32.bor(status, Detour.DT_PARTIAL_RESULT) end
    if state.outOfNodes then status = bit32.bor(status, Detour.DT_OUT_OF_NODES) end
    local path = slicedReconstruct(nav, state, target)
    nav._slicedQuery = nil
    return path, status
end

function Detour.finalizeSlicedFindPathPartial(nav, existing)
    local state = nav._slicedQuery
    if state == nil or existing == nil or #existing == 0 then nav._slicedQuery = nil; return nil, bit32.bor(Detour.DT_FAILURE, Detour.DT_INVALID_PARAM) end
    local target = nil
    for i = #existing, 1, -1 do
        local p0 = refToPoly0(nav,existing[i])
        if p0 == state.start0 or state.g[p0] ~= nil then target = p0; break end
    end
    local status = Detour.DT_SUCCESS
    if target == nil then target = state.best; status = bit32.bor(status, Detour.DT_PARTIAL_RESULT) end
    if state.outOfNodes then status = bit32.bor(status, Detour.DT_OUT_OF_NODES) end
    local path = slicedReconstruct(nav, state, target)
    nav._slicedQuery = nil
    return path, status
end

-- Synchronous form of Detour's position-aware A*.  This uses the same node-position/cost model as
-- the sliced query API instead of the compatibility center-to-center ref-only search above.
function Detour.findPathPositioned(nav,startRef,endRef,startPos,endPos,filter,maxPath,maxNodes)
    if startPos==nil or endPos==nil or not Detour.isValidPolyRef(nav,startRef,nil) or not Detour.isValidPolyRef(nav,endRef,nil) then
        return nil,bit32.bor(Detour.DT_FAILURE,Detour.DT_INVALID_PARAM)
    end
    local start0,end0=refToPoly0(nav,startRef),refToPoly0(nav,endRef)
    if start0==end0 then return {startRef},Detour.DT_SUCCESS end

    local scratch=queryScratch(nav)
    local g,total,parent,state,pos=scratch.g,scratch.f,scratch.parent,scratch.state,scratch.pos
    for _,node in scratch.touched do buffer.writeu8(state,node,0) end
    table.clear(scratch.touched); table.clear(scratch.heap.nodes); table.clear(scratch.heap.scores)
    maxNodes=math.max(1,math.floor(maxNodes or scratch.capacity))

    local function touch(node,nodePos)
        if buffer.readu8(state,node)~=0 then return true end
        if #scratch.touched>=maxNodes then return false end
        buffer.writef64(g,node*8,math.huge);buffer.writef64(total,node*8,math.huge);buffer.writei32(parent,node*4,-1)
        writeVec3f(pos,node+1,nodePos)
        scratch.touched[#scratch.touched+1]=node
        return true
    end

    touch(start0,startPos)
    buffer.writef64(g,start0*8,0)
    local startH=math.sqrt(Common.distanceSquared(startPos,endPos))*0.999
    buffer.writef64(total,start0*8,startH);buffer.writeu8(state,start0,1)
    Common.scoreHeapPush(scratch.heap,start0,startH)
    local lastBest,lastBestH=start0,startH
    local outOfNodes=false

    while true do
        local current=Common.scoreHeapPop(scratch.heap)
        if current==nil then break end
        if buffer.readu8(state,current)==2 then continue end
        buffer.writeu8(state,current,2)
        if current==end0 then lastBest=current;break end

        local currentRef=poly0ToRef(nav,current)
        local currentPos=readVec3f(pos,current+1)
        local parent0=buffer.readi32(parent,current*4)
        local currentCost=buffer.readf64(g,current*8)
        local nv=polyVertCount(nav,current)
        local edge0,link0=0,firstExtraLink(nav,current)
        while edge0<nv or link0>=0 do
            local neighbor
            if edge0<nv then
                neighbor=polyNeighbor(nav,current,edge0)
                edge0+=1
            else
                neighbor=extraLinkTarget(nav,link0)
                link0=extraLinkNext(nav,link0)
            end
            if neighbor<0 or neighbor==parent0 then continue end
            if not passFilter(nav,neighbor,filter) then continue end
            local neighborRef=poly0ToRef(nav,neighbor)
            local neighborPos
            if buffer.readu8(state,neighbor)==0 then
                local left,right=portalBetween(nav,currentRef,neighborRef)
                if left==nil or right==nil then continue end
                neighborPos=(left+right)*0.5
                if not touch(neighbor,neighborPos) then outOfNodes=true;continue end
            else
                neighborPos=readVec3f(pos,neighbor+1)
            end
            local dx=currentPos.X-neighborPos.X; local dy=currentPos.Y-neighborPos.Y; local dz=currentPos.Z-neighborPos.Z
            local step=math.sqrt(dx*dx+dy*dy+dz*dz)*areaCost(nav,current,filter)
            local cost=currentCost+step
            local ex=neighborPos.X-endPos.X; local ey=neighborPos.Y-endPos.Y; local ez=neighborPos.Z-endPos.Z
            local heuristic
            if neighbor==end0 then
                cost += math.sqrt(ex*ex+ey*ey+ez*ez)*areaCost(nav,neighbor,filter)
                heuristic=0
            else
                heuristic=math.sqrt(ex*ex+ey*ey+ez*ez)*0.999
            end
            local score=cost+heuristic
            local st=buffer.readu8(state,neighbor)
            if st~=0 and score>=buffer.readf64(total,neighbor*8) then continue end
            buffer.writei32(parent,neighbor*4,current);buffer.writef64(g,neighbor*8,cost);buffer.writef64(total,neighbor*8,score)
            buffer.writeu8(state,neighbor,1);Common.scoreHeapPush(scratch.heap,neighbor,score)
            if heuristic<lastBestH then lastBestH,lastBest=heuristic,neighbor end
        end
    end

    local path=reconstruct(nav,parent,start0,lastBest)
    if path==nil then return nil,bit32.bor(Detour.DT_FAILURE,Detour.DT_INVALID_PARAM) end
    local status=Detour.DT_SUCCESS
    if lastBest~=end0 then status=bit32.bor(status,Detour.DT_PARTIAL_RESULT) end
    if outOfNodes then status=bit32.bor(status,Detour.DT_OUT_OF_NODES) end
    if maxPath~=nil then
        maxPath=math.max(0,math.floor(maxPath))
        if #path>maxPath then
            local clipped=table.create(maxPath);for i=1,maxPath do clipped[i]=path[i] end;path=clipped
            status=bit32.bor(status,Detour.DT_BUFFER_TOO_SMALL)
        end
    end
    return path,status
end

local function pointInVertexPolyXZ(vertices, p: vector)
    local inside = false
    local j = #vertices
    for i = 1, #vertices do
        local vi, vj = vertices[i], vertices[j]
        if (vi.Z > p.Z) ~= (vj.Z > p.Z) then
            local x = (vj.X - vi.X) * (p.Z - vi.Z) / (vj.Z - vi.Z) + vi.X
            if p.X < x then inside = not inside end
        end
        j = i
    end
    return inside
end

local function segmentTouchesPolyXZ(a, b, vertices)
    if pointInVertexPolyXZ(vertices, a) or pointInVertexPolyXZ(vertices, b) then return true end
    local j = #vertices
    for i = 1, #vertices do
        if intersectSegments2D(a, b, vertices[j], vertices[i]) ~= nil then return true end
        j = i
    end
    return false
end

local function dijkstraAround(nav, startRef, startPos, filter, touchesPortal)
    if not Detour.isValidPolyRef(nav, startRef, filter) then return nil end
    local start0 = refToPoly0(nav,startRef)
    local heap = Common.scoreHeapNew()
    local costs, parents, positions, closed = {}, {}, {}, {}
    costs[start0], parents[start0], positions[start0] = 0, -1, startPos
    Common.scoreHeapPush(heap, start0, 0)
    local refs, parentRefs, resultCosts = {}, {}, {}

    while true do
        local current, poppedCost = Common.scoreHeapPop(heap)
        if current == nil then break end
        if closed[current] or poppedCost > (costs[current] or math.huge) + 1e-9 then continue end
        closed[current] = true
        refs[#refs + 1] = poly0ToRef(nav,current)
        parentRefs[#parentRefs + 1] = if parents[current] >= 0 then poly0ToRef(nav,parents[current]) else 0
        resultCosts[#resultCosts + 1] = costs[current]

        local nv = polyVertCount(nav, current)
        local edge0,link0=0,firstExtraLink(nav,current)
        while edge0<nv or link0>=0 do
            local neighbor
            if edge0<nv then neighbor=polyNeighbor(nav,current,edge0); edge0+=1 else neighbor=extraLinkTarget(nav,link0); link0=extraLinkNext(nav,link0) end
            if neighbor < 0 or closed[neighbor] or not passFilter(nav, neighbor, filter) then continue end
            local a,b=portalBetween(nav,poly0ToRef(nav,current),poly0ToRef(nav,neighbor))
            if a==nil or not touchesPortal(a,b) then continue end
            local mid = (a + b) * 0.5
            local from = positions[current]
            local step = math.sqrt(Common.distanceSquared2D(from, mid)) * areaCost(nav, current, filter)
            local total = costs[current] + step
            if total < (costs[neighbor] or math.huge) then
                costs[neighbor], parents[neighbor], positions[neighbor] = total, current, mid
                Common.scoreHeapPush(heap, neighbor, total)
            end
        end
    end

    nav._lastDijkstra = {parents = parents, closed = closed, start = start0}
    return {refs = refs, parents = parentRefs, costs = resultCosts}
end

function Detour.findPolysAroundCircle(nav, startRef, centerPos, radius, filter)
    if radius < 0 then return nil end
    local radius2 = radius * radius
    return dijkstraAround(nav, startRef, centerPos, filter, function(a, b)
        local _, d2 = closestPointSeg2D(centerPos, a, b)
        return d2 <= radius2
    end)
end

function Detour.findPolysAroundShape(nav, startRef, vertices, filter)
    if vertices == nil or #vertices < 3 then return nil end
    local center = vector.zero
    for _, v in vertices do center += v end
    center /= #vertices
    return dijkstraAround(nav, startRef, center, filter, function(a, b)
        return segmentTouchesPolyXZ(a, b, vertices)
    end)
end

function Detour.getPathFromDijkstraSearch(nav, endRef)
    local state = nav._lastDijkstra
    local end0 = refToPoly0(nav,endRef)
    if state == nil or not state.closed[end0] then return nil end
    local reverse, current = {}, end0
    while current >= 0 do
        reverse[#reverse + 1] = poly0ToRef(nav,current)
        if current == state.start then break end
        current = state.parents[current]
        if current == nil then return nil end
    end
    if reverse[#reverse] ~= poly0ToRef(nav,state.start) then return nil end
    local path = table.create(#reverse)
    for i = 1, #reverse do path[i] = reverse[#reverse - i + 1] end
    return path
end

function Detour.findLocalNeighbourhood(nav, startRef, centerPos, radius, filter)
    local result = Detour.findPolysAroundCircle(nav, startRef, centerPos, radius, filter)
    if result == nil then return nil end
    return result.refs, result.parents
end

function Detour.moveAlongSurface(nav, startRef, startPos: vector, endPos: vector, filter)
    if not Detour.isValidPolyRef(nav, startRef, filter) then return nil, nil end
    local start0 = refToPoly0(nav,startRef)
    local queue, head = {start0}, 1
    local parent, visited = {[start0] = -1}, {[start0] = true}
    local bestPos, bestDist2, bestPoly = startPos, math.huge, nil
    local searchPos = (startPos + endPos) * 0.5
    local dx, dz = endPos.X - startPos.X, endPos.Z - startPos.Z
    local searchRadius = math.sqrt(dx * dx + dz * dz) * 0.5 + 0.001
    local searchRadius2 = searchRadius * searchRadius

    while head <= #queue and head <= 48 do
        local current = queue[head]; head += 1
        if pointInPolyXZ(nav, current, endPos) then
            bestPos, bestPoly = endPos, current
            break
        end
        local nv = polyVertCount(nav, current)
        for edge0 = 0, nv - 1 do
            local a = vertexAt(nav.vertices, polyVertexIndex(nav, current, edge0))
            local b = vertexAt(nav.vertices, polyVertexIndex(nav, current, (edge0 + 1) % nv))
            local neighbor = linkedNeighborAtEdge(nav, current, edge0)
            local traversable = neighbor >= 0 and passFilter(nav, neighbor, filter)
            if not traversable then
                local q, d2 = closestPointSeg2D(endPos, a, b)
                if d2 < bestDist2 then bestPos, bestDist2, bestPoly = q, d2, current end
            elseif not visited[neighbor] then
                local _, d2 = closestPointSeg2D(searchPos, a, b)
                if d2 <= searchRadius2 then
                    visited[neighbor], parent[neighbor] = true, current
                    queue[#queue + 1] = neighbor
                end
            end
        end
    end

    if bestPoly == nil then return bestPos, {startRef} end
    local reverse, current = {}, bestPoly
    while current ~= nil and current >= 0 do
        reverse[#reverse + 1] = poly0ToRef(nav,current)
        if current == start0 then break end
        current = parent[current]
    end
    local path = table.create(#reverse)
    for i = 1, #reverse do path[i] = reverse[#reverse - i + 1] end
    return bestPos, path
end

function Detour.findDistanceToWall(nav, startRef, centerPos, maxRadius, filter)
    if maxRadius < 0 or not Detour.isValidPolyRef(nav, startRef, filter) then return nil end
    local maxDist2 = maxRadius * maxRadius
    local bestDist2, bestPos = maxDist2, nil
    local queue, head = {refToPoly0(nav,startRef)}, 1
    local visited = {[refToPoly0(nav,startRef)] = true}

    while head <= #queue do
        local poly0 = queue[head]; head += 1
        local nv = polyVertCount(nav, poly0)
        for edge0 = 0, nv - 1 do
            local a = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, edge0))
            local b = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, (edge0 + 1) % nv))
            local intervals=edgeTraversableIntervals(nav,poly0,edge0,filter)
            local d=b-a
            local cursor=0
            for _,iv in intervals do
                local lo,hi=math.clamp(iv.tmin/255,0,1),math.clamp(iv.tmax/255,0,1)
                if lo>cursor then
                    local wa,wb=a+d*cursor,a+d*lo
                    local edgePoint,edgeDist2=closestPointSeg2D(centerPos,wa,wb)
                    if edgeDist2<bestDist2 then bestDist2,bestPos=edgeDist2,edgePoint end
                end
                local pa,pb=a+d*lo,a+d*hi
                local _,portalDist2=closestPointSeg2D(centerPos,pa,pb)
                if portalDist2<=maxDist2 and not visited[iv.poly] then visited[iv.poly]=true; queue[#queue+1]=iv.poly end
                cursor=math.max(cursor,hi)
            end
            if cursor<1 then
                local wa,wb=a+d*cursor,b
                local edgePoint,edgeDist2=closestPointSeg2D(centerPos,wa,wb)
                if edgeDist2<bestDist2 then bestDist2,bestPos=edgeDist2,edgePoint end
            end
        end
    end

    if bestPos == nil then return maxRadius, nil, nil end
    local dist = math.sqrt(bestDist2)
    local delta = centerPos - bestPos
    local horizontal = math.sqrt(delta.X * delta.X + delta.Z * delta.Z)
    local normal = if horizontal > 1e-8 then vector.create(delta.X / horizontal, 0, delta.Z / horizontal) else vector.zero
    return dist, bestPos, normal
end


local function dtTriArea2D(a: vector, b: vector, c: vector)
    local abx, abz = b.X - a.X, b.Z - a.Z
    local acx, acz = c.X - a.X, c.Z - a.Z
    return acx * abz - abx * acz
end

function Detour.randomPointInConvexPoly(vertices, s, t)
    local n = #vertices
    if n < 3 then return nil end
    local areas = table.create(n)
    local areaSum = 0
    for i = 3, n do
        local area = dtTriArea2D(vertices[1], vertices[i - 1], vertices[i])
        areas[i] = area
        areaSum += math.max(0.001, area)
    end
    local threshold, acc, u, tri = (s or 0) * areaSum, 0, 1, n
    for i = 3, n do
        local dacc = areas[i]
        if threshold >= acc and threshold < acc + dacc then
            u = (threshold - acc) / dacc
            tri = i
            break
        end
        acc += dacc
    end
    local v = math.sqrt(t or 0)
    local a, b, c = 1 - v, (1 - u) * v, u * v
    return vertices[1] * a + vertices[tri - 1] * b + vertices[tri] * c
end

local function polygonArea2D(nav, poly0)
    local nv = polyVertCount(nav, poly0)
    if nv < 3 then return 0 end
    local a = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, 0))
    local area = 0
    for i = 2, nv - 1 do
        local b = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, i - 1))
        local c = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, i))
        area += dtTriArea2D(a, b, c)
    end
    return area
end

local function polygonVertices(nav, poly0)
    local nv = polyVertCount(nav, poly0)
    local verts = table.create(nv)
    for i = 0, nv - 1 do verts[i + 1] = vertexAt(nav.vertices, polyVertexIndex(nav, poly0, i)) end
    return verts
end

function Detour.findRandomPoint(nav, filter, frand)
    frand = frand or math.random
    local chosen, areaSum = nil, 0
    for poly0 = 0, nav.polyCount - 1 do
        if not passFilter(nav, poly0, filter) then continue end
        local polyArea = polygonArea2D(nav, poly0)
        areaSum += polyArea
        if frand() * areaSum <= polyArea then chosen = poly0 end
    end
    if chosen == nil then return nil end
    local point = Detour.randomPointInConvexPoly(polygonVertices(nav, chosen), frand(), frand())
    if point == nil then return nil end
    point = Detour.closestPointOnPoly(nav, poly0ToRef(nav,chosen), point) or point
    return poly0ToRef(nav,chosen), point
end

function Detour.findRandomPointAroundCircle(nav, startRef, centerPos, maxRadius, filter, frand)
    if maxRadius < 0 or not Detour.isValidPolyRef(nav, startRef, filter) then return nil end
    frand = frand or math.random
    local around = Detour.findPolysAroundCircle(nav, startRef, centerPos, maxRadius, filter)
    if around == nil then return nil end
    local chosen, areaSum = nil, 0
    for _, ref in around.refs do
        local poly0 = refToPoly0(nav, ref)
        local polyArea = polygonArea2D(nav, poly0)
        areaSum += polyArea
        if frand() * areaSum <= polyArea then chosen = poly0 end
    end
    if chosen == nil then return nil end
    local point = Detour.randomPointInConvexPoly(polygonVertices(nav, chosen), frand(), frand())
    if point == nil then return nil end
    point = Detour.closestPointOnPoly(nav, poly0ToRef(nav,chosen), point) or point
    return poly0ToRef(nav,chosen), point
end

function Detour.getEdgeMidPoint(nav, fromRef, toRef)
    local left, right = Detour.getPortalPoints(nav, fromRef, toRef)
    if left == nil then return nil end
    return (left + right) * 0.5
end

function Detour.isInClosedList(nav, ref)
    local poly0 = refToPoly0(nav, ref)
    if poly0 < 0 then return false end
    local sliced = nav._slicedQuery
    if sliced ~= nil and sliced.closed[poly0] then return true end
    local dijkstra = nav._lastDijkstra
    return dijkstra ~= nil and dijkstra.closed[poly0] == true
end

-- Luau equivalent of an independent dtNavMeshQuery instance.  The legacy functional query API
-- remains intact, but stateful sliced/Dijkstra operations can now be interleaved safely by giving
-- each caller its own query context.  Internally we temporarily lend the existing algorithms the
-- navmesh scratch slots, then restore the navmesh immediately even if the operation throws.
local NavMeshQuery = {}
NavMeshQuery.__index = NavMeshQuery

local function withQueryState(query, fn, ...)
    local nav = query.nav
    local externalSliced, externalDijkstra = nav._slicedQuery, nav._lastDijkstra
    nav._slicedQuery, nav._lastDijkstra = query._slicedQuery, query._lastDijkstra
    local packed = table.pack(pcall(fn, nav, ...))
    query._slicedQuery, query._lastDijkstra = nav._slicedQuery, nav._lastDijkstra
    nav._slicedQuery, nav._lastDijkstra = externalSliced, externalDijkstra
    if not packed[1] then error(packed[2], 0) end
    return table.unpack(packed, 2, packed.n)
end

function Detour.newNavMeshQuery(nav, maxNodes)
    if nav == nil then return nil end
    local nodes=math.max(1, math.floor(maxNodes or 2048))
    local pool={maxNodes=nodes}
    return setmetatable({
        nav = nav,
        maxNodes = nodes,
        _nodePool = pool,
        _slicedQuery = nil,
        _lastDijkstra = nil,
    }, NavMeshQuery)
end

function NavMeshQuery:getAttachedNavMesh() return self.nav end
function NavMeshQuery:getNodePool()
    -- Luau cannot expose a native dtNodePool pointer, but the handle has the same stable ownership
    -- identity/lifetime as the query object and advertises the enforced node capacity.
    return self._nodePool
end
function NavMeshQuery:findPath(startRef,endRef,startPos,endPos,filter,maxPath)
    return Detour.findPathPositioned(self.nav,startRef,endRef,startPos,endPos,filter,maxPath,self.maxNodes)
end
function NavMeshQuery:findStraightPath(startPos,endPos,corridor,options) return Detour.findStraightPath(self.nav,startPos,endPos,corridor,options) end
function NavMeshQuery:initSlicedFindPath(startRef,endRef,startPos,endPos,filter,options)
    local status=withQueryState(self,Detour.initSlicedFindPath,startRef,endRef,startPos,endPos,filter,options)
    if self._slicedQuery~=nil then
        self._slicedQuery.maxNodes=self.maxNodes
        self._slicedQuery.nodeCount=1
        self._slicedQuery.outOfNodes=false
    end
    return status
end
function NavMeshQuery:updateSlicedFindPath(maxIter) return withQueryState(self,Detour.updateSlicedFindPath,maxIter) end
function NavMeshQuery:finalizeSlicedFindPath() return withQueryState(self,Detour.finalizeSlicedFindPath) end
function NavMeshQuery:finalizeSlicedFindPathPartial(existing) return withQueryState(self,Detour.finalizeSlicedFindPathPartial,existing) end
function NavMeshQuery:findPolysAroundCircle(startRef,centerPos,radius,filter) return withQueryState(self,Detour.findPolysAroundCircle,startRef,centerPos,radius,filter) end
function NavMeshQuery:findPolysAroundShape(startRef,vertices,filter) return withQueryState(self,Detour.findPolysAroundShape,startRef,vertices,filter) end
function NavMeshQuery:getPathFromDijkstraSearch(endRef) return withQueryState(self,Detour.getPathFromDijkstraSearch,endRef) end
function NavMeshQuery:findLocalNeighbourhood(startRef,centerPos,radius,filter) return withQueryState(self,Detour.findLocalNeighbourhood,startRef,centerPos,radius,filter) end
function NavMeshQuery:moveAlongSurface(startRef,startPos,endPos,filter) return Detour.moveAlongSurface(self.nav,startRef,startPos,endPos,filter) end
function NavMeshQuery:raycast(startRef,startPos,endPos,filter,options,prevRef) return Detour.raycast(self.nav,startRef,startPos,endPos,filter,options,prevRef) end
function NavMeshQuery:findDistanceToWall(startRef,centerPos,maxRadius,filter) return Detour.findDistanceToWall(self.nav,startRef,centerPos,maxRadius,filter) end
function NavMeshQuery:getPolyWallSegments(ref,filter) return Detour.getPolyWallSegments(self.nav,ref,filter) end
function NavMeshQuery:getPortalPoints(fromRef,toRef) return Detour.getPortalPoints(self.nav,fromRef,toRef) end
function NavMeshQuery:getEdgeMidPoint(fromRef,toRef) return Detour.getEdgeMidPoint(self.nav,fromRef,toRef) end
function NavMeshQuery:findNearestPoly(center,extents,filter) return Detour.findNearestPoly(self.nav,center,extents,filter) end
function NavMeshQuery:queryPolygons(center,extents,filter,maxPolys) return Detour.queryPolygons(self.nav,center,extents,filter,maxPolys) end
function NavMeshQuery:findRandomPoint(filter,frand) return withQueryState(self,Detour.findRandomPoint,filter,frand) end
function NavMeshQuery:findRandomPointAroundCircle(startRef,centerPos,maxRadius,filter,frand) return withQueryState(self,Detour.findRandomPointAroundCircle,startRef,centerPos,maxRadius,filter,frand) end
function NavMeshQuery:closestPointOnPoly(ref,pos) return Detour.closestPointOnPoly(self.nav,ref,pos) end
function NavMeshQuery:closestPointOnPolyBoundary(ref,pos) return Detour.closestPointOnPolyBoundary(self.nav,ref,pos) end
function NavMeshQuery:getPolyHeight(ref,pos) return Detour.getPolyHeight(self.nav,ref,pos) end
function NavMeshQuery:isValidPolyRef(ref,filter) return Detour.isValidPolyRef(self.nav,ref,filter) end
function NavMeshQuery:isInClosedList(ref)
    local poly0=refToPoly0(self.nav,ref)
    if poly0<0 then return false end
    return (self._slicedQuery~=nil and self._slicedQuery.closed[poly0]==true)
        or (self._lastDijkstra~=nil and self._lastDijkstra.closed[poly0]==true)
end

local function appendStraightVertex(points: { vector }, flags, refs, point: vector, flag, ref)
    if #points == 0 or not Common.vequal2D(points[#points], point, 1e-5) or math.abs(points[#points].Y - point.Y) > 1e-5 then
        points[#points + 1] = point
        flags[#flags + 1] = flag or 0
        refs[#refs + 1] = ref or 0
    else
        -- Preserve terminal metadata if the funnel collapsed two equal vertices.
        if flag ~= nil and flag ~= 0 then flags[#flags] = bit32.bor(flags[#flags], flag) end
        if ref ~= nil then refs[#refs] = ref end
    end
end

function Detour.findStraightPath(nav, startPos, endPos, corridor, options)
    if corridor == nil or #corridor == 0 then return nil end
    local start = Detour.closestPointOnPoly(nav, corridor[1], startPos) or startPos
    local finish = Detour.closestPointOnPoly(nav, corridor[#corridor], endPos) or endPos
    local points, flags, refs = {}, {}, {}
    appendStraightVertex(points, flags, refs, start, Detour.DT_STRAIGHTPATH_START, corridor[1])
    if #corridor == 1 then
        appendStraightVertex(points, flags, refs, finish, Detour.DT_STRAIGHTPATH_END, 0)
        return points, flags, refs
    end

    local portals = table.create(#corridor)
    for i = 1, #corridor - 1 do
        local left, right = portalBetween(nav, corridor[i], corridor[i + 1])
        if left == nil then return nil end
        portals[#portals + 1] = {left = left, right = right, ref = corridor[i + 1], polyType = polyType(nav, refToPoly0(nav,corridor[i + 1]))}
    end
    portals[#portals + 1] = {left = finish, right = finish, ref = 0}

    local apex, left, right = start, start, start
    local apexIndex, leftIndex, rightIndex = 0, 0, 0
    local i = 1
    while i <= #portals do
        local newLeft, newRight = portals[i].left, portals[i].right

        if dtTriArea2D(apex, right, newRight) <= 0 then
            if Common.vequal2D(apex, right) or dtTriArea2D(apex, left, newRight) > 0 then
                right = newRight
                rightIndex = i
            else
                local portal = portals[leftIndex]
                appendStraightVertex(points, flags, refs, left, if portal ~= nil and portal.polyType == Detour.DT_POLYTYPE_OFFMESH_CONNECTION then Detour.DT_STRAIGHTPATH_OFFMESH_CONNECTION else 0, if portal ~= nil then portal.ref else corridor[1])
                apex = left
                apexIndex = leftIndex
                left, right = apex, apex
                leftIndex, rightIndex = apexIndex, apexIndex
                i = apexIndex + 1
                continue
            end
        end

        if dtTriArea2D(apex, left, newLeft) >= 0 then
            if Common.vequal2D(apex, left) or dtTriArea2D(apex, right, newLeft) < 0 then
                left = newLeft
                leftIndex = i
            else
                local portal = portals[rightIndex]
                appendStraightVertex(points, flags, refs, right, if portal ~= nil and portal.polyType == Detour.DT_POLYTYPE_OFFMESH_CONNECTION then Detour.DT_STRAIGHTPATH_OFFMESH_CONNECTION else 0, if portal ~= nil then portal.ref else corridor[1])
                apex = right
                apexIndex = rightIndex
                left, right = apex, apex
                leftIndex, rightIndex = apexIndex, apexIndex
                i = apexIndex + 1
                continue
            end
        end
        i += 1
    end
    appendStraightVertex(points, flags, refs, finish, Detour.DT_STRAIGHTPATH_END, 0)
    return points, flags, refs
end

function Detour.query(nav, startPos, endPos, extents, filter)
    local startRef, nearestStart = Detour.findNearestPoly(nav, startPos, extents, filter)
    if startRef == nil then return nil, 'no start polygon' end
    local endRef, nearestEnd = Detour.findNearestPoly(nav, endPos, extents, filter)
    if endRef == nil then return nil, 'no end polygon' end
    local corridor,pathStatus = Detour.findPath(nav,startRef,endRef,nearestStart,nearestEnd,filter)
    -- query() is the convenience "complete route" API.  Raw Detour findPath semantics return the
    -- best partial corridor with DT_PARTIAL_RESULT when the goal is unreachable; keep that behavior
    -- in findPathPositioned(), but do not silently turn a blocked convenience query into success.
    if corridor == nil or Detour.statusFailed(pathStatus) or Detour.statusDetail(pathStatus,Detour.DT_PARTIAL_RESULT) then return nil, 'no path' end
    local straight = Detour.findStraightPath(nav, nearestStart, nearestEnd, corridor)
    if straight == nil then return nil, 'invalid corridor' end
    return {
        startRef = startRef,
        endRef = endRef,
        nearestStart = nearestStart,
        nearestEnd = nearestEnd,
        corridor = corridor,
        points = straight,
    }
end

function Detour.neighbors(nav, ref, filter)
    local tri0 = refToPoly0(nav, ref)
    if not isWalkable(nav, tri0) then return {} end
    local out = {}
    for edge0 = 0, polyVertCount(nav, tri0) - 1 do
        local n = polyNeighbor(nav, tri0, edge0)
        if n >= 0 and passFilter(nav, n, filter) then out[#out + 1] = poly0ToRef(nav,n) end
    end
    local link0=firstExtraLink(nav,tri0)
    while link0>=0 do
        local n=extraLinkTarget(nav,link0)
        if n>=0 and passFilter(nav,n,filter) then out[#out+1]=poly0ToRef(nav,n) end
        link0=extraLinkNext(nav,link0)
    end
    return out
end


-- Append Detour-style off-mesh connection polygons to a single navigation mesh.
-- Each connection becomes a two-vertex polygon and is joined through packed auxiliary link
-- records, matching dtNavMesh's topology rather than treating jumps as annotations on ground edges.
local OFFMESH_CON_BYTES = 24
function Detour.addOffMeshConnections(nav, connections, options)
    options = options or {}
    if connections == nil or #connections == 0 then return nav end
    if nav.offMeshConCount ~= nil and nav.offMeshConCount > 0 then
        error('off-mesh connections are already attached to this navmesh', 2)
    end

    local oldPolyCount, oldVertCount = nav.polyCount, nav.vertexCount
    local conCount = #connections
    local nvp = nav.nvp or 3
    if nvp < 2 then error('navmesh polygon vertex capacity must be at least 2', 2) end
    if oldVertCount + conCount * 2 > 0xffff then error('Detour vertex index limit exceeded', 2) end

    -- Triangle-only importers become the same indexed-poly representation used by rcPolyMesh.
    local oldIndices, oldCounts = nav.polyIndices, nav.polyVertCounts
    if oldIndices == nil then
        oldIndices = buffer.create(oldPolyCount * nvp * 2)
        buffer.fill(oldIndices, 0, 0xff, buffer.len(oldIndices))
        oldCounts = buffer.create(oldPolyCount)
        for p0 = 0, oldPolyCount - 1 do
            local a,b,c = triIndices(nav,p0)
            local o = p0 * nvp * 2
            buffer.writeu16(oldIndices,o,a); buffer.writeu16(oldIndices,o+2,b); buffer.writeu16(oldIndices,o+4,c)
            buffer.writeu8(oldCounts,p0,3)
        end
    end

    -- Resolve endpoints against ground polygons before the off-mesh polygons become visible.
    local resolved = table.create(conCount)
    local defaultClimb = options.walkableClimb or nav.walkableClimb or 0
    for i, con in connections do
        local startPos = con.start or con.a or con[1]
        local endPos = con.finish or con['end'] or con.b or con[2]
        if startPos == nil or endPos == nil then error('off-mesh connection needs start and end vectors', 2) end
        local radius = con.radius or con.rad or options.radius or 0.5
        local climb = con.walkableClimb or defaultClimb
        local ext = vector.create(radius, math.max(climb, 0.001), radius)
        local startRef,startNearest = Detour.findNearestPoly(nav,startPos,ext,nil)
        local endRef,endNearest = Detour.findNearestPoly(nav,endPos,ext,nil)
        if startNearest ~= nil then
            local dx,dz = startNearest.X-startPos.X,startNearest.Z-startPos.Z
            if dx*dx+dz*dz > radius*radius then startRef,startNearest=nil,nil end
        end
        if endNearest ~= nil then
            local dx,dz = endNearest.X-endPos.X,endNearest.Z-endPos.Z
            if dx*dx+dz*dz > radius*radius then endRef,endNearest=nil,nil end
        end
        resolved[i] = {
            start = startNearest or startPos, finish = endNearest or endPos,
            startRef = startRef, endRef = endRef, radius = radius,
            bidirectional = con.bidirectional == true or con.bidir == true or con.direction == Detour.DT_OFFMESH_CON_BIDIR,
            area = con.area == nil and 0 or con.area,
            flags = con.flags == nil and 1 or con.flags,
            userId = con.userId or con.userID or 0,
        }
    end

    local newPolyCount, newVertCount = oldPolyCount + conCount, oldVertCount + conCount * 2
    local vertices = buffer.create(newVertCount * 12); buffer.copy(vertices,0,nav.vertices,0,oldVertCount*12)
    local indices = buffer.create(newPolyCount*nvp*2); buffer.fill(indices,0,0xff,buffer.len(indices)); buffer.copy(indices,0,oldIndices,0,oldPolyCount*nvp*2)
    local counts = buffer.create(newPolyCount); buffer.copy(counts,0,oldCounts,0,oldPolyCount)
    local neighbors = buffer.create(newPolyCount*nvp*4); buffer.fill(neighbors,0,0xff,buffer.len(neighbors))
    local oldStride = nav.nvp or 3
    for p0=0,oldPolyCount-1 do
        for e0=0,math.min(oldStride,nvp)-1 do
            buffer.writei32(neighbors,(p0*nvp+e0)*4,buffer.readi32(nav.neighbors,(p0*oldStride+e0)*4))
        end
    end
    local centers=buffer.create(newPolyCount*12); buffer.copy(centers,0,nav.centers,0,oldPolyCount*12)
    local areas=buffer.create(newPolyCount); buffer.copy(areas,0,nav.areas,0,oldPolyCount)
    local flags=buffer.create(newPolyCount*2); buffer.copy(flags,0,nav.flags,0,oldPolyCount*2)
    local types=buffer.create(newPolyCount)
    if nav.polyTypes~=nil then buffer.copy(types,0,nav.polyTypes,0,oldPolyCount) end
    local valid=buffer.create(newPolyCount)
    if nav.polyValid~=nil then buffer.copy(valid,0,nav.polyValid,0,oldPolyCount) else buffer.fill(valid,0,1,oldPolyCount) end

    local firstLinks=buffer.create(newPolyCount*4); buffer.fill(firstLinks,0,0xff,buffer.len(firstLinks))
    local maxLinks=conCount*4
    local links=buffer.create(math.max(1,maxLinks)*EXTRA_LINK_BYTES)
    local linkCount=0
    local function addLink(from0,to0,edge)
        if from0==nil or to0==nil then return end
        local o=linkCount*EXTRA_LINK_BYTES
        buffer.writei32(links,o,to0)
        buffer.writei32(links,o+4,buffer.readi32(firstLinks,from0*4))
        buffer.writeu8(links,o+8,edge)
        buffer.writei32(firstLinks,from0*4,linkCount)
        linkCount+=1
    end

    local cons=buffer.create(conCount*OFFMESH_CON_BYTES)
    local polyToCon=buffer.create(newPolyCount*4); buffer.fill(polyToCon,0,0xff,buffer.len(polyToCon))
    for i=1,conCount do
        local con=resolved[i]
        local c0=i-1; local p0=oldPolyCount+c0; local v0=oldVertCount+c0*2
        writeVec3f(vertices,v0+1,con.start); writeVec3f(vertices,v0+2,con.finish)
        local io=p0*nvp*2
        buffer.writeu16(indices,io,v0); buffer.writeu16(indices,io+2,v0+1)
        buffer.writeu8(counts,p0,2); buffer.writeu8(types,p0,Detour.DT_POLYTYPE_OFFMESH_CONNECTION); buffer.writeu8(valid,p0,1)
        buffer.writeu8(areas,p0,con.area); buffer.writeu16(flags,p0*2,con.flags)
        writeVec3f(centers,p0+1,(con.start+con.finish)*0.5)
        buffer.writei32(polyToCon,p0*4,c0)
        local co=c0*OFFMESH_CON_BYTES
        buffer.writei32(cons,co,p0); buffer.writef32(cons,co+4,con.radius)
        buffer.writeu8(cons,co+8,con.bidirectional and Detour.DT_OFFMESH_CON_BIDIR or 0)
        buffer.writeu32(cons,co+12,con.userId)
        buffer.writei32(cons,co+16,con.startRef and refToPoly0(nav,con.startRef) or -1)
        buffer.writei32(cons,co+20,con.endRef and refToPoly0(nav,con.endRef) or -1)
        if con.startRef~=nil then
            local start0=refToPoly0(nav,con.startRef)
            addLink(p0,start0,0); addLink(start0,p0,0xff)
        end
        if con.endRef~=nil then
            local end0=refToPoly0(nav,con.endRef)
            addLink(p0,end0,1)
            if con.bidirectional then addLink(end0,p0,0xff) end
        end
    end

    nav.vertices,nav.vertexCount=vertices,newVertCount
    nav.polyIndices,nav.polyVertCounts,nav.nvp=indices,counts,nvp
    nav.neighbors,nav.centers,nav.areas,nav.flags=neighbors,centers,areas,flags
    nav.polyTypes,nav.polyValid=types,valid
    nav.groundPolyCount=nav.groundPolyCount or oldPolyCount
    nav.polyCount=newPolyCount
    nav.walkableCount=(nav.walkableCount or oldPolyCount)+conCount
    nav.extraFirstLinks,nav.extraLinks,nav.extraLinkCount=firstLinks,links,linkCount
    nav.offMeshConnections,nav.offMeshPolyToCon,nav.offMeshConCount=cons,polyToCon,conCount
    nav.offMeshPolyBase=oldPolyCount
    buildPolyBounds(nav)
    nav._queryScratch=nil; nav._slicedQuery=nil; nav._lastDijkstra=nil
    return nav
end


-- Decode an upstream Detour dtCreateNavMeshData tile directly into the Luau query representation.
-- The binary parser lives in @recast/navmesh-builder; require lazily to keep the core modules acyclic.
function Detour.loadNavMeshData(data)
    local Builder=require('@recast/navmesh-builder')
    local nav,err=Builder.decodeNavMeshData(data)
    if nav==nil then
        local detail = Detour.DT_INVALID_PARAM
        if err == 'wrong navmesh magic' then detail = Detour.DT_WRONG_MAGIC
        elseif err == 'wrong navmesh version' then detail = Detour.DT_WRONG_VERSION
        elseif err == 'tile data is shorter than dtMeshHeader' or err == 'truncated navmesh tile data' then detail = Detour.DT_BUFFER_TOO_SMALL end
        return nil,bit32.bor(Detour.DT_FAILURE,detail),err
    end
    nav._tileData=data
    nav.extraLinkTargetsAreRefs=false
    nav.extraFirstLinks=buffer.create(nav.polyCount*4); buffer.fill(nav.extraFirstLinks,0,0xff,buffer.len(nav.extraFirstLinks))
    nav.extraLinks=buffer.create(math.max(1,#nav.offMeshRecords*4)*EXTRA_LINK_BYTES)
    nav.extraLinkCount=0
    nav.offMeshConCount=#nav.offMeshRecords
    nav.offMeshConnections=buffer.create(nav.offMeshConCount*OFFMESH_CON_BYTES)
    nav.offMeshPolyToCon=buffer.create(nav.polyCount*4); buffer.fill(nav.offMeshPolyToCon,0,0xff,buffer.len(nav.offMeshPolyToCon))
    nav.offMeshSides=buffer.create(nav.offMeshConCount); buffer.fill(nav.offMeshSides,0,0xff,buffer.len(nav.offMeshSides))

    local function addLink(from0,to0,edge,side)
        if from0==nil or to0==nil or from0<0 or to0<0 then return end
        local link0=nav.extraLinkCount; nav.extraLinkCount+=1
        local o=link0*EXTRA_LINK_BYTES
        buffer.writei32(nav.extraLinks,o,to0); buffer.writei32(nav.extraLinks,o+4,buffer.readi32(nav.extraFirstLinks,from0*4))
        buffer.writeu8(nav.extraLinks,o+8,edge or 0xff); buffer.writeu8(nav.extraLinks,o+9,side or 0xff)
        buffer.writei32(nav.extraFirstLinks,from0*4,link0)
    end

    -- Match dtNavMesh::baseOffMeshLinks() and the same-tile portion of connectExtOffMeshLinks().
    -- External landing links are completed by addTileData() once neighboring tiles are present.
    for c0,con in nav.offMeshRecords do
        local i0=c0-1; local p0=con.poly
        if p0<0 or p0>=nav.polyCount then continue end
        buffer.writei32(nav.offMeshPolyToCon,p0*4,i0)
        local co=i0*OFFMESH_CON_BYTES
        buffer.writei32(nav.offMeshConnections,co,p0); buffer.writef32(nav.offMeshConnections,co+4,con.radius)
        buffer.writeu8(nav.offMeshConnections,co+8,con.bidirectional and Detour.DT_OFFMESH_CON_BIDIR or 0); buffer.writeu32(nav.offMeshConnections,co+12,con.userId)
        buffer.writei32(nav.offMeshConnections,co+16,-1); buffer.writei32(nav.offMeshConnections,co+20,-1)
        buffer.writeu8(nav.offMeshSides,i0,con.side)

        local ext=vector.create(con.radius,math.max(nav.walkableClimb or 0,0.001),con.radius)
        local startRef,startPt=Detour.findNearestPoly(nav,con.start,ext,nil)
        if startPt~=nil then
            local dx,dz=startPt.X-con.start.X,startPt.Z-con.start.Z
            if dx*dx+dz*dz<=con.radius*con.radius then
                local start0=refToPoly0(nav,startRef)
                local vi=polyVertexIndex(nav,p0,0); writeVec3f(nav.vertices,vi+1,startPt)
                buffer.writei32(nav.offMeshConnections,co+16,start0)
                addLink(p0,start0,0,0xff); addLink(start0,p0,0xff,0xff)
            end
        end
        if con.side==0xff then
            local endRef,endPt=Detour.findNearestPoly(nav,con.finish,ext,nil)
            if endPt~=nil then
                local dx,dz=endPt.X-con.finish.X,endPt.Z-con.finish.Z
                if dx*dx+dz*dz<=con.radius*con.radius then
                    local end0=refToPoly0(nav,endRef)
                    local vi=polyVertexIndex(nav,p0,1); writeVec3f(nav.vertices,vi+1,endPt)
                    buffer.writei32(nav.offMeshConnections,co+20,end0)
                    addLink(p0,end0,1,0xff)
                    if con.bidirectional then addLink(end0,p0,0xff,0xff) end
                end
            end
        end
        local a=vertexAt(nav.vertices,polyVertexIndex(nav,p0,0)); local b=vertexAt(nav.vertices,polyVertexIndex(nav,p0,1)); writeVec3f(nav.centers,p0+1,(a+b)*0.5)
    end
    nav.offMeshRecords=nil
    buildPolyBounds(nav)
    return nav,Detour.DT_SUCCESS
end

function Detour.getPolyType(nav,ref)
    local p0=refToPoly0(nav,ref)
    if p0<0 or p0>=nav.polyCount then return nil end
    return polyType(nav,p0)
end

function Detour.getOffMeshConnectionByRef(nav,ref)
    local p0=refToPoly0(nav,ref)
    if p0<0 or p0>=nav.polyCount or polyType(nav,p0)~=Detour.DT_POLYTYPE_OFFMESH_CONNECTION or nav.offMeshPolyToCon==nil then return nil end
    local c0=buffer.readi32(nav.offMeshPolyToCon,p0*4)
    if c0<0 then return nil end
    local co=c0*OFFMESH_CON_BYTES
    return {
        poly=poly0ToRef(nav,p0), radius=buffer.readf32(nav.offMeshConnections,co+4),
        bidirectional=buffer.readu8(nav.offMeshConnections,co+8)~=0,
        userId=buffer.readu32(nav.offMeshConnections,co+12),
        startRef=(function() local v=buffer.readi32(nav.offMeshConnections,co+16); return v>=0 and poly0ToRef(nav,v) or 0 end)(),
        endRef=(function() local v=buffer.readi32(nav.offMeshConnections,co+20); return v>=0 and poly0ToRef(nav,v) or 0 end)(),
        start=vertexAt(nav.vertices,polyVertexIndex(nav,p0,0)), finish=vertexAt(nav.vertices,polyVertexIndex(nav,p0,1)),
    }
end

function Detour.getOffMeshConnectionPolyEndPoints(nav,prevRef,polyRef)
    local p0=refToPoly0(nav,polyRef)
    if p0<0 or p0>=nav.polyCount or polyType(nav,p0)~=Detour.DT_POLYTYPE_OFFMESH_CONNECTION then return nil end
    local a=vertexAt(nav.vertices,polyVertexIndex(nav,p0,0))
    local b=vertexAt(nav.vertices,polyVertexIndex(nav,p0,1))
    local link0=findExtraLink(nav,p0,refToPoly0(nav,prevRef))
    if link0>=0 and extraLinkEdge(nav,link0)~=0 then return b,a end
    return a,b
end


local compactCon = compactConnection

function Detour.getPolyFlags(nav, ref)
    local tri0 = refToPoly0(nav, ref)
    if tri0 < 0 or tri0 >= nav.polyCount then return nil end
    return buffer.readu16(nav.flags, tri0 * 2)
end

function Detour.setPolyFlags(nav, ref, flags)
    local tri0 = refToPoly0(nav, ref)
    if tri0 < 0 or tri0 >= nav.polyCount then return false end
    if nav.sourceSpan ~= nil and nav.spanToBase ~= nil then
        local span = buffer.readu32(nav.sourceSpan, tri0 * 4)
        local base = buffer.readi32(nav.spanToBase, span * 4)
        if base >= 0 then
            buffer.writeu16(nav.flags, base * 2, flags)
            buffer.writeu16(nav.flags, (base + 1) * 2, flags)
            return true
        end
    end
    buffer.writeu16(nav.flags, tri0 * 2, flags)
    return true
end

function Detour.getPolyArea(nav, ref)
    local tri0 = refToPoly0(nav, ref)
    if tri0 < 0 or tri0 >= nav.polyCount then return nil end
    return buffer.readu8(nav.areas, tri0)
end

function Detour.setPolyArea(nav, ref, area)
    local tri0 = refToPoly0(nav, ref)
    if tri0 < 0 or tri0 >= nav.polyCount then return false end
    if area < 0 or area >= Detour.DT_MAX_AREAS then error('area must be in [0, 63]', 2) end
    if nav.sourceSpan ~= nil and nav.spanToBase ~= nil then
        local span = buffer.readu32(nav.sourceSpan, tri0 * 4)
        local base = buffer.readi32(nav.spanToBase, span * 4)
        if base >= 0 then
            buffer.writeu8(nav.areas, base, area)
            buffer.writeu8(nav.areas, base + 1, area)
            buffer.writeu8(nav.compact.areas, span, area)
            return true
        end
    end
    buffer.writeu8(nav.areas, tri0, area)
    return true
end

function Detour.buildCompactNavMesh(chf)
    local spanToBase = buffer.create(chf.spanCount * 4)
    for i = 0, chf.spanCount - 1 do buffer.writei32(spanToBase, i * 4, -1) end

    local walkableSpans = 0
    for i = 0, chf.spanCount - 1 do
        if buffer.readu8(chf.areas, i) ~= Recast.RC_NULL_AREA then walkableSpans += 1 end
    end

    local vertexCount = walkableSpans * 4
    local polyCount = walkableSpans * 2
    local vertices = buffer.create(vertexCount * 12)
    local triangles = buffer.create(polyCount * 12)
    local areas = buffer.create(polyCount)
    local flags = buffer.create(polyCount * 2)
    local polyTypes = buffer.create(polyCount)
    local polyValid = buffer.create(polyCount)
    buffer.fill(polyValid, 0, 1, polyCount)
    local neighbors = buffer.create(polyCount * 3 * 4)
    local centers = buffer.create(polyCount * 12)
    local sourceSpan = buffer.create(polyCount * 4)
    local spanNeighbors = buffer.create(chf.spanCount * 4 * 4)
    for i = 0, polyCount * 3 - 1 do buffer.writei32(neighbors, i * 4, -1) end
    for i = 0, chf.spanCount * 4 - 1 do buffer.writei32(spanNeighbors, i * 4, -1) end

    local nextSpanOrdinal = 0
    for z = 0, chf.height - 1 do
        for x = 0, chf.width - 1 do
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local area = buffer.readu8(chf.areas, span)
                if area == Recast.RC_NULL_AREA then continue end

                local ordinal = nextSpanOrdinal
                nextSpanOrdinal += 1
                local vbase = ordinal * 4
                local pbase = ordinal * 2
                buffer.writei32(spanToBase, span * 4, pbase)
                local y = chf.bmin.Y + buffer.readu16(chf.y, span * 2) * chf.ch
                local x0 = chf.bmin.X + x * chf.cs
                local x1 = x0 + chf.cs
                local z0 = chf.bmin.Z + z * chf.cs
                local z1 = z0 + chf.cs
                local a = vector.create(x0, y, z0)
                local b = vector.create(x0, y, z1)
                local c = vector.create(x1, y, z0)
                local d = vector.create(x1, y, z1)
                writeVec3f(vertices, vbase + 1, a)
                writeVec3f(vertices, vbase + 2, b)
                writeVec3f(vertices, vbase + 3, c)
                writeVec3f(vertices, vbase + 4, d)

                local o0 = pbase * 12
                buffer.writeu32(triangles, o0, vbase)
                buffer.writeu32(triangles, o0 + 4, vbase + 1)
                buffer.writeu32(triangles, o0 + 8, vbase + 2)
                local o1 = (pbase + 1) * 12
                buffer.writeu32(triangles, o1, vbase + 2)
                buffer.writeu32(triangles, o1 + 4, vbase + 1)
                buffer.writeu32(triangles, o1 + 8, vbase + 3)

                buffer.writeu8(areas, pbase, area)
                buffer.writeu8(areas, pbase + 1, area)
                buffer.writeu16(flags, pbase * 2, 1)
                buffer.writeu16(flags, (pbase + 1) * 2, 1)
                buffer.writeu32(sourceSpan, pbase * 4, span)
                buffer.writeu32(sourceSpan, (pbase + 1) * 4, span)
                writeVec3f(centers, pbase + 1, (a + b + c) / 3)
                writeVec3f(centers, pbase + 2, (c + b + d) / 3)

                -- Internal diagonal: ABC edge B-C <-> CBD edge C-B.
                buffer.writei32(neighbors, (pbase * 3 + 1) * 4, pbase + 1)
                buffer.writei32(neighbors, ((pbase + 1) * 3 + 0) * 4, pbase)
            end
        end
    end

    -- Cardinal compact-field connections become explicit triangle portals.
    for z = 0, chf.height - 1 do
        for x = 0, chf.width - 1 do
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local base = buffer.readi32(spanToBase, span * 4)
                if base < 0 then continue end
                for dir0 = 0, 3 do
                    local con = compactCon(chf, span, dir0)
                    if con == Recast.RC_NOT_CONNECTED then continue end
                    local dx = if dir0 == 0 then -1 elseif dir0 == 2 then 1 else 0
                    local dz = if dir0 == 1 then 1 elseif dir0 == 3 then -1 else 0
                    local ncell = (x + dx) + (z + dz) * chf.width
                    local nspan = buffer.readu32(chf.cellIndex, ncell * 4) + con
                    local nbase = buffer.readi32(spanToBase, nspan * 4)
                    if nbase < 0 then continue end
                    buffer.writei32(spanNeighbors, (span * 4 + dir0) * 4, nspan)
                    if dir0 == 0 then
                        buffer.writei32(neighbors, (base * 3 + 0) * 4, nbase + 1)
                    elseif dir0 == 1 then
                        buffer.writei32(neighbors, ((base + 1) * 3 + 1) * 4, nbase)
                    elseif dir0 == 2 then
                        buffer.writei32(neighbors, ((base + 1) * 3 + 2) * 4, nbase)
                    else
                        buffer.writei32(neighbors, (base * 3 + 2) * 4, nbase + 1)
                    end
                end
            end
        end
    end

    return {
        vertices = vertices,
        vertexCount = vertexCount,
        triangles = triangles,
        polyCount = polyCount,
        walkableCount = polyCount,
        areas = areas,
        flags = flags,
        polyTypes = polyTypes, polyValid = polyValid, groundPolyCount = polyCount,
        neighbors = neighbors,
        centers = centers,
        sourceSpan = sourceSpan,
        spanToBase = spanToBase,
        spanNeighbors = spanNeighbors,
        compact = chf,
        bmin = chf.bmin,
        bmax = chf.bmax,
    }
end

function Detour.buildPolyMeshNavMesh(mesh)
    local vertices = buffer.create(mesh.nverts * 12)
    for v0 = 0, mesh.nverts - 1 do
        local o = v0 * 6
        writeVec3f(vertices, v0 + 1, vector.create(
            mesh.bmin.X + buffer.readu16(mesh.verts, o) * mesh.cs,
            mesh.bmin.Y + buffer.readu16(mesh.verts, o + 2) * mesh.ch,
            mesh.bmin.Z + buffer.readu16(mesh.verts, o + 4) * mesh.cs
        ))
    end

    local polyIndices = buffer.create(mesh.npolys * mesh.nvp * 2)
    buffer.fill(polyIndices, 0, 0xff, buffer.len(polyIndices))
    local polyVertCounts = buffer.create(mesh.npolys)
    local neighbors = buffer.create(mesh.npolys * mesh.nvp * 4)
    buffer.fill(neighbors, 0, 0xff, buffer.len(neighbors))
    local centers = buffer.create(mesh.npolys * 12)
    local areas = buffer.create(mesh.npolys)
    local flags = buffer.create(mesh.npolys * 2)
    local polyTypes = buffer.create(mesh.npolys)
    local polyValid = buffer.create(mesh.npolys)
    local portalDirs = buffer.create(mesh.npolys * mesh.nvp)
    buffer.fill(portalDirs, 0, 0xff, buffer.len(portalDirs))
    local walkableCount = 0

    for p0 = 0, mesh.npolys - 1 do
        local count = 0
        local center = vector.zero
        local base = p0 * mesh.nvp * 2
        for e0 = 0, mesh.nvp - 1 do
            local vi = buffer.readu16(mesh.polys, (base + e0) * 2)
            if vi == 0xffff then break end
            buffer.writeu16(polyIndices, (p0 * mesh.nvp + e0) * 2, vi)
            center += vertexAt(vertices, vi)
            count += 1
            local nei = buffer.readu16(mesh.polys, (base + mesh.nvp + e0) * 2)
            if nei ~= 0xffff then
                if bit32.band(nei, 0x8000) == 0 then
                    buffer.writei32(neighbors, (p0 * mesh.nvp + e0) * 4, nei)
                else
                    buffer.writeu8(portalDirs, p0 * mesh.nvp + e0, bit32.band(nei, 0x0f))
                end
            end
        end
        buffer.writeu8(polyVertCounts, p0, count)
        if count > 0 then writeVec3f(centers, p0 + 1, center / count) end
        local area = buffer.readu8(mesh.areas, p0)
        buffer.writeu8(areas, p0, area)
        local flag = buffer.readu16(mesh.flags, p0 * 2)
        if flag == 0 and area ~= Recast.RC_NULL_AREA then flag = 1 end
        buffer.writeu16(flags, p0 * 2, flag)
        if area ~= Recast.RC_NULL_AREA then walkableCount += 1; buffer.writeu8(polyValid, p0, 1) end
    end

    local nav = {
        vertices = vertices, vertexCount = mesh.nverts,
        polyIndices = polyIndices, polyVertCounts = polyVertCounts, nvp = mesh.nvp,
        polyCount = mesh.npolys, walkableCount = walkableCount,
        areas = areas, flags = flags, polyTypes = polyTypes, polyValid = polyValid, groundPolyCount = mesh.npolys, neighbors = neighbors, centers = centers,
        portalDirs = portalDirs, bmin = mesh.bmin, bmax = mesh.bmax, polyMesh = mesh, walkableClimb = mesh.walkableClimb or 0,
    }
    buildPolyBounds(nav)
    buildBVTree(nav, mesh.cs)
    return nav
end


local function nextPow2(v)
    local p=1
    while p<v do p*=2 end
    return p
end
local function ilog2(v)
    local n=0
    while v>1 do v=math.floor(v/2); n+=1 end
    return n
end
local function refMask(bits) return 2^bits-1 end
local function tiledPosHash(x,y,mask)
    local h1=bit32.band(0x8da6b343,mask); local h2=bit32.band(0xd8163841,mask)
    return bit32.band(h1*bit32.band(x,mask)+h2*bit32.band(y,mask),mask)
end

function Detour.encodePolyId(nav,salt,it,ip)
    local c=nav.refCodec
    if c==nil then return ip+1 end
    return (salt%2^c.saltBits)*2^(c.tileBits+c.polyBits)+(it%2^c.tileBits)*2^c.polyBits+(ip%2^c.polyBits)
end
function Detour.decodePolyId(nav,ref)
    local c=nav.refCodec
    if c==nil then return 0,0,ref-1 end
    local ip=ref%2^c.polyBits
    local it=math.floor(ref/2^c.polyBits)%2^c.tileBits
    local salt=math.floor(ref/2^(c.polyBits+c.tileBits))%2^c.saltBits
    return salt,it,ip
end
function Detour.decodePolyIdSalt(nav,ref) local s0=Detour.decodePolyId(nav,ref); return s0 end
function Detour.decodePolyIdTile(nav,ref) local _,it=Detour.decodePolyId(nav,ref); return it end
function Detour.decodePolyIdPoly(nav,ref) local _,_,ip=Detour.decodePolyId(nav,ref); return ip end

function Detour.newTiledNavMesh(params)
    params=params or {}
    local maxTiles=math.floor(params.maxTiles or 1)
    local maxPolys=math.floor(params.maxPolys or 2048)
    if maxTiles<=0 or maxPolys<=0 then error('maxTiles and maxPolys must be positive',2) end
    local tileBits=ilog2(nextPow2(maxTiles)); local polyBits=ilog2(nextPow2(maxPolys))
    if tileBits+polyBits>31 then error('tile/poly reference bits exceed Detour 32-bit limit',2) end
    local saltBits=math.min(31,32-tileBits-polyBits)
    if saltBits<10 then error('Detour requires at least 10 salt bits',2) end
    local salts=buffer.create(maxTiles*4)
    local freeTiles=table.create(maxTiles)
    for i=maxTiles-1,0,-1 do buffer.writeu32(salts,i*4,1); freeTiles[#freeTiles+1]=i end
    local tileLutSize=nextPow2(math.floor(maxTiles/4)); if tileLutSize<1 then tileLutSize=1 end
    return {
        _tiled=true, refCodec={tileBits=tileBits,polyBits=polyBits,saltBits=saltBits},
        maxTiles=maxTiles,maxPolys=maxPolys,tileSalts=salts,tiles=table.create(maxTiles),
        freeTiles=freeTiles,tileLutSize=tileLutSize,tileLutMask=tileLutSize-1,posLookup=table.create(tileLutSize,-1),
        orig=params.orig or vector.zero,tileWidth=params.tileWidth or 1,tileHeight=params.tileHeight or params.tileWidth or 1,
        vertices=buffer.create(0),vertexCount=0,polyIndices=buffer.create(0),polyIndexBytes=4,polyVertCounts=buffer.create(0),nvp=params.maxVertsPerPoly or 6,
        polyCount=0,groundPolyCount=0,walkableCount=0,areas=buffer.create(0),flags=buffer.create(0),polyTypes=buffer.create(0),polyValid=buffer.create(0),
        neighbors=buffer.create(0),centers=buffer.create(0),bmin=params.orig or vector.zero,bmax=params.orig or vector.zero,
        refToPoly={},polyRefs=buffer.create(0),
        -- Tiled query geometry is append-only/high-water internally.  Removing a tile invalidates
        -- its refs and leaves a hole; adding another tile never moves surviving polygons.
        polyCapacity=0,vertexCapacity=0,linkCapacity=0,polyHighWater=0,vertexHighWater=0,
        activePolyCount=0,activeVertexCount=0,activeGroundCount=0,activeWalkableCount=0,
        edgeLookup={},portalLookup={},extraLinkTargetsAreRefs=true,extraFirstLinks=buffer.create(0),extraLinks=buffer.create(0),extraLinkCount=0,
        offMeshConnections=buffer.create(0),offMeshPolyToCon=buffer.create(0),offMeshConCount=0,offMeshConCapacity=0,
    }
end

function Detour.calcTileLoc(nav,pos: vector)
    if not nav._tiled then return 0,0 end
    return math.floor((pos.X-nav.orig.X)/nav.tileWidth),math.floor((pos.Z-nav.orig.Z)/nav.tileHeight)
end
function Detour.getMaxTiles(nav) return nav._tiled and nav.maxTiles or 1 end
function Detour.getTile(nav,index0)
    if not nav._tiled then return if index0==0 then nav else nil end
    if index0<0 or index0>=nav.maxTiles then return nil end
    return nav.tiles[index0+1]
end
function Detour.getTileAt(nav,x,y,layer)
    layer=layer or 0
    if not nav._tiled then return if x==0 and y==0 and layer==0 then nav else nil end
    local index=nav.posLookup[tiledPosHash(x,y,nav.tileLutMask)+1]
    while index~=nil and index>=0 do
        local t=nav.tiles[index+1]
        if t~=nil and t.x==x and t.y==y and t.layer==layer then return t end
        index=t and t._nextPos or -1
    end
    return nil
end
function Detour.getTilesAt(nav,x,y,maxTiles)
    local out={}; maxTiles=maxTiles or math.huge
    if not nav._tiled then if x==0 and y==0 and maxTiles>0 then out[1]=nav end; return out end
    local index=nav.posLookup[tiledPosHash(x,y,nav.tileLutMask)+1]
    while index~=nil and index>=0 do
        local t=nav.tiles[index+1]
        if t~=nil and t.x==x and t.y==y and #out<maxTiles then out[#out+1]=t end
        index=t and t._nextPos or -1
    end
    return out
end
function Detour.getTileRef(nav,tile)
    if not nav._tiled then return tile==nav and 1 or 0 end
    if tile==nil or tile.index==nil then return 0 end
    return Detour.encodePolyId(nav,tile.salt,tile.index,0)
end
function Detour.getTileRefAt(nav,x,y,layer)
    local t=Detour.getTileAt(nav,x,y,layer); return t and Detour.getTileRef(nav,t) or 0
end
function Detour.getTileByRef(nav,ref)
    if not nav._tiled then return ref==1 and nav or nil end
    local salt,it,ip=Detour.decodePolyId(nav,ref)
    if ip~=0 or it>=nav.maxTiles then return nil end
    local t=nav.tiles[it+1]
    return if t~=nil and t.salt==salt then t else nil
end

local function rebuildTiledAggregate(nav)
    local active={}; local vertCount,polyCount,groundCount,walkableCount,nvp=0,0,0,0,3
    local bmin,bmax=nil,nil; local cellSize=nil; local offCount=0
    for i=1,nav.maxTiles do
        local t=nav.tiles[i]
        if t~=nil then
            t.vertexBase=vertCount; t.polyBase=polyCount
            active[#active+1]=t; vertCount+=t.nav.vertexCount; polyCount+=t.nav.polyCount
            groundCount+=t.nav.groundPolyCount or t.nav.polyCount; walkableCount+=t.nav.walkableCount or t.nav.polyCount
            nvp=math.max(nvp,t.nav.nvp or 3); offCount+=t.nav.offMeshConCount or 0
            if bmin==nil then bmin,bmax=t.nav.bmin,t.nav.bmax else
                bmin=vector.create(math.min(bmin.X,t.nav.bmin.X),math.min(bmin.Y,t.nav.bmin.Y),math.min(bmin.Z,t.nav.bmin.Z))
                bmax=vector.create(math.max(bmax.X,t.nav.bmax.X),math.max(bmax.Y,t.nav.bmax.Y),math.max(bmax.Z,t.nav.bmax.Z))
            end
            local cs=t.nav.polyMesh and t.nav.polyMesh.cs or t.nav.cellSize
            if cs~=nil and cs>0 then cellSize=cellSize and math.min(cellSize,cs) or cs end
        end
    end
    bmin,bmax=bmin or nav.orig,bmax or nav.orig
    local vertices=buffer.create(vertCount*12)
    local indices=buffer.create(polyCount*nvp*4); buffer.fill(indices,0,0xff,buffer.len(indices))
    local counts=buffer.create(polyCount); local neighbors=buffer.create(polyCount*nvp*4); buffer.fill(neighbors,0,0xff,buffer.len(neighbors))
    local centers=buffer.create(polyCount*12); local areas=buffer.create(polyCount); local flags=buffer.create(polyCount*2)
    local types=buffer.create(polyCount); local valid=buffer.create(polyCount); local polyRefs=buffer.create(polyCount*4)
    local refToPoly={}; local linkList={}; local edgeMap={}
    local offCons=if offCount>0 then buffer.create(offCount*OFFMESH_CON_BYTES) else nil
    local offMap=if offCount>0 then buffer.create(polyCount*4) else nil
    if offMap~=nil then buffer.fill(offMap,0,0xff,buffer.len(offMap)) end
    local nextOff=0

    local function queueLink(from0,to0,edge) linkList[#linkList+1]={from=from0,to=to0,edge=edge} end
    for _,t in active do
        local src=t.nav; local vb,pb=t.vertexBase,t.polyBase
        buffer.copy(vertices,vb*12,src.vertices,0,src.vertexCount*12)
        for lp=0,src.polyCount-1 do
            local gp=pb+lp; local nv=polyVertCount(src,lp)
            buffer.writeu8(counts,gp,nv); buffer.writeu8(areas,gp,buffer.readu8(src.areas,lp)); buffer.writeu16(flags,gp*2,buffer.readu16(src.flags,lp*2))
            buffer.writeu8(types,gp,polyType(src,lp)); buffer.writeu8(valid,gp,isWalkable(src,lp) and 1 or 0)
            writeVec3f(centers,gp+1,polyCenter(src,lp))
            local ref=Detour.encodePolyId(nav,t.salt,t.index,lp); buffer.writeu32(polyRefs,gp*4,ref); refToPoly[ref]=gp
            for e=0,nv-1 do
                buffer.writeu32(indices,(gp*nvp+e)*4,vb+polyVertexIndex(src,lp,e))
                local nei=polyNeighbor(src,lp,e); if nei>=0 then buffer.writei32(neighbors,(gp*nvp+e)*4,pb+nei) end
            end
            local l=firstExtraLink(src,lp)
            while l>=0 do queueLink(gp,pb+extraLinkTarget(src,l),extraLinkEdge(src,l)); l=extraLinkNext(src,l) end
            if offMap~=nil and polyType(src,lp)==Detour.DT_POLYTYPE_OFFMESH_CONNECTION and src.offMeshPolyToCon~=nil then
                local lc=buffer.readi32(src.offMeshPolyToCon,lp*4)
                if lc>=0 then
                    local so=lc*OFFMESH_CON_BYTES; local doff=nextOff*OFFMESH_CON_BYTES
                    buffer.writei32(offCons,doff,gp); buffer.writef32(offCons,doff+4,buffer.readf32(src.offMeshConnections,so+4))
                    buffer.writeu8(offCons,doff+8,buffer.readu8(src.offMeshConnections,so+8)); buffer.writeu32(offCons,doff+12,buffer.readu32(src.offMeshConnections,so+12))
                    local a=buffer.readi32(src.offMeshConnections,so+16); local b=buffer.readi32(src.offMeshConnections,so+20)
                    buffer.writei32(offCons,doff+16,a>=0 and pb+a or -1); buffer.writei32(offCons,doff+20,b>=0 and pb+b or -1)
                    buffer.writei32(offMap,gp*4,nextOff); nextOff+=1
                end
            end
        end
    end

    -- Exact full-edge external links.  Partial portal clipping is layered on this same packed-link
    -- representation later; exact shared borders already cover standard adjacent Recast tiles.
    for _,t in active do
        local src=t.nav; local pb=t.polyBase
        for lp=0,src.polyCount-1 do
            if polyType(src,lp)~=Detour.DT_POLYTYPE_GROUND then continue end
            local gp=pb+lp; local nv=polyVertCount(src,lp)
            for e=0,nv-1 do
                if polyNeighbor(src,lp,e)>=0 then continue end
                local a=vertexAt(vertices,buffer.readu32(indices,(gp*nvp+e)*4))
                local b=vertexAt(vertices,buffer.readu32(indices,(gp*nvp+(e+1)%nv)*4))
                local ka,kb=vertexKey(a),vertexKey(b); local key=if ka<kb then ka..'>'..kb else kb..'>'..ka
                local prev=edgeMap[key]
                if prev==nil then edgeMap[key]={poly=gp,edge=e,tile=t.index}
                elseif prev~=false and prev.tile~=t.index then
                    queueLink(gp,prev.poly,e); queueLink(prev.poly,gp,prev.edge); edgeMap[key]=false
                end
            end
        end
    end

    local first=nil; local links=nil
    if #linkList>0 then
        first=buffer.create(polyCount*4); buffer.fill(first,0,0xff,buffer.len(first)); links=buffer.create(#linkList*EXTRA_LINK_BYTES)
        for i,l in linkList do
            local l0 = i - 1
            local o = l0 * EXTRA_LINK_BYTES
            buffer.writei32(links,o,l.to); buffer.writei32(links,o+4,buffer.readi32(first,l.from*4)); buffer.writeu8(links,o+8,l.edge)
            buffer.writei32(first,l.from*4,l0)
        end
    end

    nav.vertices,nav.vertexCount=vertices,vertCount; nav.polyIndices,nav.polyIndexBytes,nav.polyVertCounts,nav.nvp=indices,4,counts,nvp
    nav.polyCount,nav.groundPolyCount,nav.walkableCount=polyCount,groundCount,walkableCount
    nav.neighbors,nav.centers,nav.areas,nav.flags,nav.polyTypes,nav.polyValid=neighbors,centers,areas,flags,types,valid
    nav.polyRefs,nav.refToPoly=polyRefs,refToPoly; nav.extraFirstLinks,nav.extraLinks,nav.extraLinkCount=first,links,#linkList
    nav.offMeshConnections,nav.offMeshPolyToCon,nav.offMeshConCount=offCons,offMap,nextOff
    nav.bmin,nav.bmax=bmin,bmax; nav.cellSize=cellSize
    nav.compact=nil; nav.spanToBase=nil; nav.spanNeighbors=nil; nav.sourceSpan=nil; nav.detailMesh=nil
    nav._queryScratch=nil; nav._compactQueryScratch=nil; nav._slicedQuery=nil; nav._lastDijkstra=nil
    if polyCount>0 then buildPolyBounds(nav) else nav.polyBounds=buffer.create(0) end
    nav.bvTree,nav.bvNodeCount,nav.bvQuantFactor=nil,nil,nil
    if groundCount>0 and cellSize~=nil then buildBVTree(nav,cellSize) end
end

local function growPackedBuffer(old,minBytes,fillByte)
    local oldBytes=buffer.len(old)
    if oldBytes>=minBytes then return old end
    local newBytes=math.max(minBytes,math.max(64,oldBytes*2))
    local out=buffer.create(newBytes)
    if fillByte~=nil then buffer.fill(out,0,fillByte,newBytes) end
    if oldBytes>0 then buffer.copy(out,0,old,0,oldBytes) end
    return out
end

local function ensureTiledVertexCapacity(nav,required)
    if required<=nav.vertexCapacity then return end
    local cap=math.max(required,math.max(16,nav.vertexCapacity*2))
    nav.vertices=growPackedBuffer(nav.vertices,cap*12,nil)
    nav.vertexCapacity=cap
end

local function ensureTiledPolyCapacity(nav,required)
    if required<=nav.polyCapacity then return end
    local cap=math.max(required,math.max(16,nav.polyCapacity*2))
    local nvp=nav.nvp
    nav.polyIndices=growPackedBuffer(nav.polyIndices,cap*nvp*4,0xff)
    nav.polyVertCounts=growPackedBuffer(nav.polyVertCounts,cap,nil)
    nav.neighbors=growPackedBuffer(nav.neighbors,cap*nvp*4,0xff)
    nav.centers=growPackedBuffer(nav.centers,cap*12,nil)
    nav.areas=growPackedBuffer(nav.areas,cap,nil)
    nav.flags=growPackedBuffer(nav.flags,cap*2,nil)
    nav.polyTypes=growPackedBuffer(nav.polyTypes,cap,nil)
    nav.polyValid=growPackedBuffer(nav.polyValid,cap,nil)
    nav.polyRefs=growPackedBuffer(nav.polyRefs,cap*4,nil)
    nav.extraFirstLinks=growPackedBuffer(nav.extraFirstLinks,cap*4,0xff)
    nav.offMeshPolyToCon=growPackedBuffer(nav.offMeshPolyToCon,cap*4,0xff)
    nav.polyCapacity=cap
end

local function ensureTiledLinkCapacity(nav,required)
    if required<=nav.linkCapacity then return end
    local cap=math.max(required,math.max(32,nav.linkCapacity*2))
    nav.extraLinks=growPackedBuffer(nav.extraLinks,cap*EXTRA_LINK_BYTES,nil)
    nav.linkCapacity=cap
end

local function ensureTiledOffMeshCapacity(nav,required)
    if required<=nav.offMeshConCapacity then return end
    local cap=math.max(required,math.max(8,nav.offMeshConCapacity*2))
    nav.offMeshConnections=growPackedBuffer(nav.offMeshConnections,cap*OFFMESH_CON_BYTES,nil)
    nav.offMeshConCapacity=cap
end

local function appendTiledExtraLink(nav,from0,targetRef,edge0,side,bmin,bmax)
    ensureTiledLinkCapacity(nav,nav.extraLinkCount+1)
    local link0=nav.extraLinkCount; nav.extraLinkCount+=1
    local o=link0*EXTRA_LINK_BYTES
    buffer.writeu32(nav.extraLinks,o,targetRef)
    buffer.writei32(nav.extraLinks,o+4,buffer.readi32(nav.extraFirstLinks,from0*4))
    buffer.writeu8(nav.extraLinks,o+8,edge0 or 0xff)
    buffer.writeu8(nav.extraLinks,o+9,side or 0xff)
    buffer.writeu8(nav.extraLinks,o+10,bmin or 0)
    buffer.writeu8(nav.extraLinks,o+11,bmax or 0)
    buffer.writei32(nav.extraFirstLinks,from0*4,link0)
    return link0
end

local function tiledEdgeKey(a,b)
    local ka,kb=vertexKey(a),vertexKey(b)
    return if ka<kb then ka..'>'..kb else kb..'>'..ka
end

local function attachTiledBoundaryEdge(nav,tile,poly0,localEdge,a,b)
    local key=tiledEdgeKey(a,b)
    local entries=nav.edgeLookup[key]
    if entries==nil then entries={}; nav.edgeLookup[key]=entries end
    local ref=poly0ToRef(nav,poly0)
    -- Match the first still-live edge from another tile.  Dead salt-versioned descriptors are
    -- intentionally retained: resolving the ref is enough to reject them without O(world) cleanup.
    for _,entry in entries do
        if entry.tile~=tile.index then
            local other0=refToPoly0(nav,entry.ref)
            if other0>=0 and isWalkable(nav,other0) then
                appendTiledExtraLink(nav,poly0,entry.ref,localEdge)
                appendTiledExtraLink(nav,other0,ref,entry.edge)
                break
            end
        end
    end
    entries[#entries+1]={ref=ref,edge=localEdge,tile=tile.index}
end


local function tiledPortalLong(dir0,v: vector)
    return if dir0==0 or dir0==2 then v.Z else v.X
end

local function tiledPortalSlabCoord(dir0,v: vector)
    return if dir0==0 or dir0==2 then v.X else v.Z
end

local function tiledPortalDetourSide(dir0)
    -- rcPolyMesh/TileCache portal directions are 0=x-,1=z+,2=x+,3=z-.
    -- dtCreateNavMeshData converts those to Detour's 8-way side IDs 4,2,0,6.
    if dir0==0 then return 4 elseif dir0==1 then return 2 elseif dir0==2 then return 0 else return 6 end
end

local function tiledPortalBucketKey(x,y,dir0)
    -- Layers intentionally share a bucket.  Upstream dtNavMesh connects compatible portals both
    -- between layers at the same tile coordinate and between neighboring tile coordinates.
    return tostring(x)..'|'..tostring(y)..'|'..tostring(dir0)
end

local function tiledPortalYAt(dir0,a: vector,b: vector,q)
    local aa,bb=tiledPortalLong(dir0,a),tiledPortalLong(dir0,b)
    local d=bb-aa
    if math.abs(d)<1e-6 then return (a.Y+b.Y)*0.5 end
    local t=(q-aa)/d
    return a.Y+(b.Y-a.Y)*t
end

local function tiledPortalOverlap(dir0,a,b,c,d,walkableClimb)
    local amin0,amax0=math.min(tiledPortalLong(dir0,a),tiledPortalLong(dir0,b)),math.max(tiledPortalLong(dir0,a),tiledPortalLong(dir0,b))
    local bmin0,bmax0=math.min(tiledPortalLong(dir0,c),tiledPortalLong(dir0,d)),math.max(tiledPortalLong(dir0,c),tiledPortalLong(dir0,d))
    -- Detour's overlapSlabs shrinks both segments by 0.01 before testing horizontal overlap.
    local minx=math.max(amin0+0.01,bmin0+0.01)
    local maxx=math.min(amax0-0.01,bmax0-0.01)
    if minx>maxx then return nil end
    local dmin=tiledPortalYAt(dir0,c,d,minx)-tiledPortalYAt(dir0,a,b,minx)
    local dmax=tiledPortalYAt(dir0,c,d,maxx)-tiledPortalYAt(dir0,a,b,maxx)
    if dmin*dmax>=0 then
        local thr=((walkableClimb or 0)*2)^2
        if dmin*dmin>thr and dmax*dmax>thr then return nil end
    end
    return math.max(amin0,bmin0),math.min(amax0,bmax0)
end

local function tiledPortalQuantize(dir0,a,b,lo,hi)
    local av,bv=tiledPortalLong(dir0,a),tiledPortalLong(dir0,b)
    local d=bv-av
    if math.abs(d)<1e-6 then return 0,255 end
    local t0=(lo-av)/d; local t1=(hi-av)/d
    if t0>t1 then t0,t1=t1,t0 end
    local bmin=math.floor(math.clamp(t0,0,1)*255+0.5)
    local bmax=math.floor(math.clamp(t1,0,1)*255+0.5)
    return bmin,bmax
end

local function appendDirectionalPortalLink(nav,from0,fromEdge,fromDir,fromA,fromB,targetEntry,targetClimb)
    if math.abs(tiledPortalSlabCoord(fromDir,fromA)-tiledPortalSlabCoord(targetEntry.dir,targetEntry.a))>0.01 then return false end
    local lo,hi=tiledPortalOverlap(fromDir,fromA,fromB,targetEntry.a,targetEntry.b,targetClimb)
    if lo==nil then return false end
    local bmin,bmax=tiledPortalQuantize(fromDir,fromA,fromB,lo,hi)
    appendTiledExtraLink(nav,from0,targetEntry.ref,fromEdge,tiledPortalDetourSide(fromDir),bmin,bmax)
    return true
end

local function attachTiledPortalEdge(nav,tile,poly0,localEdge,a,b,dir0,walkableClimb)
    local ref=poly0ToRef(nav,poly0)
    local opposite=bit32.band(dir0+2,3)
    local wantX,wantY=tile.x,tile.y
    if dir0==0 then wantX-=1 elseif dir0==1 then wantY+=1 elseif dir0==2 then wantX+=1 else wantY-=1 end

    -- Upstream first connects tiles sharing the same (x,y) coordinate (different layers), then
    -- neighboring coordinates.  A portal can connect to up to four polygons per target tile.
    local candidateKeys={
        tiledPortalBucketKey(tile.x,tile.y,opposite),
        tiledPortalBucketKey(wantX,wantY,opposite),
    }
    local seen={}
    local forwardCount,reverseCount=0,0
    for _,key in candidateKeys do
        local entries=nav.portalLookup[key]
        if entries~=nil then
            for _,entry in entries do
                if entry.tile==tile.index or seen[entry] then continue end
                seen[entry]=true
                local other0=refToPoly0(nav,entry.ref)
                if other0<0 or not isWalkable(nav,other0) then continue end
                -- Match the same side-coordinate restriction as findConnectingPolys().
                if math.abs(tiledPortalSlabCoord(dir0,a)-tiledPortalSlabCoord(entry.dir,entry.a))>0.01 then continue end
                if forwardCount<4 and appendDirectionalPortalLink(nav,poly0,localEdge,dir0,a,b,entry,entry.walkableClimb or 0) then forwardCount+=1 end
                if reverseCount<4 then
                    local back={ref=ref,dir=dir0,a=a,b=b}
                    if appendDirectionalPortalLink(nav,other0,entry.edge,entry.dir,entry.a,entry.b,back,walkableClimb or 0) then reverseCount+=1 end
                end
            end
        end
    end

    local ownKey=tiledPortalBucketKey(tile.x,tile.y,dir0)
    local own=nav.portalLookup[ownKey]
    if own==nil then own={};nav.portalLookup[ownKey]=own end
    own[#own+1]={ref=ref,edge=localEdge,tile=tile.index,x=tile.x,y=tile.y,layer=tile.layer,dir=dir0,a=a,b=b,walkableClimb=walkableClimb or 0}
end

local function appendTileGeometry(nav,tile)
    local src=tile.nav
    local srcNvp=src.nvp or 3
    if srcNvp>nav.nvp then return false end

    local vb=nav.vertexHighWater
    local pb=nav.polyHighWater
    tile.vertexBase,tile.polyBase=vb,pb
    tile.vertexCount,tile.polyCount=src.vertexCount,src.polyCount
    ensureTiledVertexCapacity(nav,vb+src.vertexCount)
    ensureTiledPolyCapacity(nav,pb+src.polyCount)
    if src.vertexCount>0 then buffer.copy(nav.vertices,vb*12,src.vertices,0,src.vertexCount*12) end

    local groundAdded,walkableAdded=0,0
    for lp=0,src.polyCount-1 do
        local gp=pb+lp
        local nv=polyVertCount(src,lp)
        buffer.writeu8(nav.polyVertCounts,gp,nv)
        buffer.writeu8(nav.areas,gp,buffer.readu8(src.areas,lp))
        buffer.writeu16(nav.flags,gp*2,buffer.readu16(src.flags,lp*2))
        local typ=polyType(src,lp); buffer.writeu8(nav.polyTypes,gp,typ)
        local valid=isWalkable(src,lp)
        buffer.writeu8(nav.polyValid,gp,valid and 1 or 0)
        if typ==Detour.DT_POLYTYPE_GROUND then groundAdded+=1 end
        if valid then walkableAdded+=1 end
        writeVec3f(nav.centers,gp+1,polyCenter(src,lp))
        local ref=Detour.encodePolyId(nav,tile.salt,tile.index,lp)
        buffer.writeu32(nav.polyRefs,gp*4,ref); nav.refToPoly[ref]=gp
        buffer.writei32(nav.extraFirstLinks,gp*4,-1)
        buffer.writei32(nav.offMeshPolyToCon,gp*4,-1)
        for e=0,nav.nvp-1 do
            local io=(gp*nav.nvp+e)*4
            buffer.writeu32(nav.polyIndices,io,0xffffffff)
            buffer.writei32(nav.neighbors,io,-1)
        end
        for e=0,nv-1 do
            buffer.writeu32(nav.polyIndices,(gp*nav.nvp+e)*4,vb+polyVertexIndex(src,lp,e))
            local nei=polyNeighbor(src,lp,e)
            if nei>=0 then buffer.writei32(nav.neighbors,(gp*nav.nvp+e)*4,pb+nei) end
        end
    end

    -- Copy tile-local auxiliary topology using encoded refs so later salt invalidation also
    -- invalidates stale links without touching their packed records.
    for lp=0,src.polyCount-1 do
        local gp=pb+lp
        local link0=firstExtraLink(src,lp)
        while link0>=0 do
            local targetLocal=extraLinkTarget(src,link0)
            if targetLocal>=0 then
                appendTiledExtraLink(nav,gp,Detour.encodePolyId(nav,tile.salt,tile.index,targetLocal),extraLinkEdge(src,link0))
            end
            link0=extraLinkNext(src,link0)
        end
        if polyType(src,lp)==Detour.DT_POLYTYPE_OFFMESH_CONNECTION and src.offMeshPolyToCon~=nil then
            local lc=buffer.readi32(src.offMeshPolyToCon,lp*4)
            if lc>=0 then
                local c0=nav.offMeshConCount; nav.offMeshConCount+=1; ensureTiledOffMeshCapacity(nav,nav.offMeshConCount)
                local so=lc*OFFMESH_CON_BYTES; local d=c0*OFFMESH_CON_BYTES
                buffer.writei32(nav.offMeshConnections,d,gp)
                buffer.writef32(nav.offMeshConnections,d+4,buffer.readf32(src.offMeshConnections,so+4))
                buffer.writeu8(nav.offMeshConnections,d+8,buffer.readu8(src.offMeshConnections,so+8))
                buffer.writeu32(nav.offMeshConnections,d+12,buffer.readu32(src.offMeshConnections,so+12))
                local a=buffer.readi32(src.offMeshConnections,so+16); local b=buffer.readi32(src.offMeshConnections,so+20)
                buffer.writei32(nav.offMeshConnections,d+16,a>=0 and pb+a or -1)
                buffer.writei32(nav.offMeshConnections,d+20,b>=0 and pb+b or -1)
                buffer.writei32(nav.offMeshPolyToCon,gp*4,c0)
            end
        end
    end

    nav.vertexHighWater=vb+src.vertexCount; nav.polyHighWater=pb+src.polyCount
    nav.vertexCount=nav.vertexHighWater; nav.polyCount=nav.polyHighWater
    nav.activeVertexCount+=src.vertexCount; nav.activePolyCount+=src.polyCount
    nav.activeGroundCount+=groundAdded; nav.activeWalkableCount+=walkableAdded
    nav.groundPolyCount=nav.activeGroundCount; nav.walkableCount=nav.activeWalkableCount

    if nav.activePolyCount==src.polyCount then nav.bmin,nav.bmax=src.bmin,src.bmax else
        nav.bmin=vector.create(math.min(nav.bmin.X,src.bmin.X),math.min(nav.bmin.Y,src.bmin.Y),math.min(nav.bmin.Z,src.bmin.Z))
        nav.bmax=vector.create(math.max(nav.bmax.X,src.bmax.X),math.max(nav.bmax.Y,src.bmax.Y),math.max(nav.bmax.Z,src.bmax.Z))
    end
    local cs=src.polyMesh and src.polyMesh.cs or src.cellSize
    if cs~=nil and cs>0 then nav.cellSize=nav.cellSize and math.min(nav.cellSize,cs) or cs end

    -- Install external links only after all refs and geometry from the new tile exist.  Proper
    -- Detour/Recast tiles carry explicit portal directions; those edges may only partially overlap
    -- after TileCache removes border vertices, so use slab overlap and store clipped portal limits.
    -- Legacy/simple tile navmeshes without portal metadata retain exact-edge fallback semantics.
    for lp=0,src.polyCount-1 do
        if polyType(src,lp)~=Detour.DT_POLYTYPE_GROUND then continue end
        local gp=pb+lp; local nv=polyVertCount(src,lp)
        for e=0,nv-1 do
            if polyNeighbor(src,lp,e)>=0 then continue end
            local a=vertexAt(src.vertices,polyVertexIndex(src,lp,e))
            local b=vertexAt(src.vertices,polyVertexIndex(src,lp,(e+1)%nv))
            local dir0=if src.portalDirs~=nil then buffer.readu8(src.portalDirs,lp*src.nvp+e) else 0xff
            if dir0~=0xff then attachTiledPortalEdge(nav,tile,gp,e,a,b,dir0,src.walkableClimb or 0)
            elseif src.portalDirs==nil then attachTiledBoundaryEdge(nav,tile,gp,e,a,b) end
        end
    end

    nav.polyBounds=nil; nav.bvTree,nav.bvNodeCount,nav.bvQuantFactor=nil,nil,nil
    nav._queryScratch=nil; nav._compactQueryScratch=nil; nav._slicedQuery=nil; nav._lastDijkstra=nil
    return true
end


local function externalSideTarget(x,y,side)
    if side==0 then return x+1,y end
    if side==1 then return x+1,y+1 end
    if side==2 then return x,y+1 end
    if side==3 then return x-1,y+1 end
    if side==4 then return x-1,y end
    if side==5 then return x-1,y-1 end
    if side==6 then return x,y-1 end
    if side==7 then return x+1,y-1 end
    return x,y
end

-- Connect serialized off-mesh endpoints which land in another tile.  This mirrors
-- dtNavMesh::connectExtOffMeshLinks(): the source connection always links to the landing poly,
-- while the landing poly links back only for DT_OFFMESH_CON_BIDIR.
local function connectTiledExternalOffMesh(nav,sourceTile,landingTile)
    local src=sourceTile.nav
    if src.offMeshConCount==nil or src.offMeshConCount==0 or src.offMeshSides==nil then return end
    for c0=0,src.offMeshConCount-1 do
        local side=buffer.readu8(src.offMeshSides,c0)
        local tx,ty=externalSideTarget(sourceTile.x,sourceTile.y,side)
        if side==0xff then
            if landingTile.x~=sourceTile.x or landingTile.y~=sourceTile.y or landingTile==sourceTile then continue end
        elseif landingTile.x~=tx or landingTile.y~=ty then continue end

        local so=c0*OFFMESH_CON_BYTES
        local localPoly=buffer.readi32(src.offMeshConnections,so)
        if localPoly<0 or localPoly>=src.polyCount then continue end
        -- baseOffMeshLinks must have connected the start first; otherwise upstream skips landing.
        if buffer.readi32(src.offMeshConnections,so+16)<0 then continue end
        local sourcePoly=sourceTile.polyBase+localPoly
        local localVertex=polyVertexIndex(src,localPoly,1)
        local endpoint=vertexAt(src.vertices,localVertex)
        local radius=buffer.readf32(src.offMeshConnections,so+4)
        local ext=vector.create(radius,math.max(src.walkableClimb or 0,0.001),radius)
        local landingRefLocal,nearest=Detour.findNearestPoly(landingTile.nav,endpoint,ext,nil)
        if nearest==nil then continue end
        local dx,dz=nearest.X-endpoint.X,nearest.Z-endpoint.Z
        if dx*dx+dz*dz>radius*radius then continue end
        local landingLocal=refToPoly0(landingTile.nav,landingRefLocal)
        if landingLocal<0 or polyType(landingTile.nav,landingLocal)~=Detour.DT_POLYTYPE_GROUND then continue end
        local landingPoly=landingTile.polyBase+landingLocal
        local sourceRef=poly0ToRef(nav,sourcePoly); local landingRef=poly0ToRef(nav,landingPoly)

        local globalVertex=polyVertexIndex(nav,sourcePoly,1)
        writeVec3f(nav.vertices,globalVertex+1,nearest)
        writeVec3f(src.vertices,localVertex+1,nearest)
        local globalCon=buffer.readi32(nav.offMeshPolyToCon,sourcePoly*4)
        if globalCon>=0 then buffer.writei32(nav.offMeshConnections,globalCon*OFFMESH_CON_BYTES+20,landingPoly) end

        if findExtraLink(nav,sourcePoly,landingPoly)<0 then appendTiledExtraLink(nav,sourcePoly,landingRef,1,side,0,0) end
        local bidir=buffer.readu8(src.offMeshConnections,so+8)~=0
        if bidir and findExtraLink(nav,landingPoly,sourcePoly)<0 then
            local reverseSide=if side==0xff then 0xff else bit32.band(side+4,7)
            appendTiledExtraLink(nav,landingPoly,sourceRef,0xff,reverseSide,0,0)
        end
    end
end

function Detour.addTile(nav,tileNav,x,y,layer,lastRef)
    if not nav._tiled then return nil,bit32.bor(Detour.DT_FAILURE,Detour.DT_INVALID_PARAM) end
    x,y,layer=math.floor(x or 0),math.floor(y or 0),math.floor(layer or 0)
    if tileNav==nil or tileNav.polyCount>nav.maxPolys or (tileNav.nvp or 3)>nav.nvp then return nil,bit32.bor(Detour.DT_FAILURE,Detour.DT_INVALID_PARAM) end
    if Detour.getTileAt(nav,x,y,layer)~=nil then return nil,bit32.bor(Detour.DT_FAILURE,Detour.DT_ALREADY_OCCUPIED) end
    local index,salt=nil,nil
    if lastRef~=nil and lastRef~=0 then
        local ls,li,lp=Detour.decodePolyId(nav,lastRef)
        if lp~=0 or li>=nav.maxTiles or nav.tiles[li+1]~=nil then return nil,bit32.bor(Detour.DT_FAILURE,Detour.DT_INVALID_PARAM) end
        local found=nil
        for i=1,#nav.freeTiles do if nav.freeTiles[i]==li then found=i; break end end
        if found==nil then return nil,bit32.bor(Detour.DT_FAILURE,Detour.DT_INVALID_PARAM) end
        table.remove(nav.freeTiles,found)
        index,salt=li,ls
    else
        local n=#nav.freeTiles
        if n==0 then return nil,bit32.bor(Detour.DT_FAILURE,Detour.DT_OUT_OF_MEMORY) end
        index=nav.freeTiles[n]; nav.freeTiles[n]=nil
        salt=buffer.readu32(nav.tileSalts,index*4)
    end
    if salt==0 then salt=1 end; buffer.writeu32(nav.tileSalts,index*4,salt)
    local tile={nav=tileNav,x=x,y=y,layer=layer,index=index,salt=salt,userId=tileNav.userId or 0,_nextPos=-1}
    nav.tiles[index+1]=tile
    if not appendTileGeometry(nav,tile) then nav.tiles[index+1]=nil; nav.freeTiles[#nav.freeTiles+1]=index; return nil,bit32.bor(Detour.DT_FAILURE,Detour.DT_INVALID_PARAM) end
    local hash=tiledPosHash(x,y,nav.tileLutMask)
    tile._nextPos=nav.posLookup[hash+1]; nav.posLookup[hash+1]=index
    -- dtNavMesh only reconnects same-coordinate layers plus the eight neighbouring tile
    -- coordinates.  Scanning every world tile here is both unnecessary and catastrophically
    -- expensive for streamed worlds.  Keep Detour's 32-layer-per-coordinate cap and visit each
    -- candidate once.
    local seen={}
    local function connectCandidates(tx,ty)
        for _,other in Detour.getTilesAt(nav,tx,ty,32) do
            if other~=tile and not seen[other.index] then
                seen[other.index]=true
                connectTiledExternalOffMesh(nav,tile,other)
                connectTiledExternalOffMesh(nav,other,tile)
            end
        end
    end
    connectCandidates(x,y)
    for side=0,7 do local nx,ny=externalSideTarget(x,y,side); connectCandidates(nx,ny) end
    return Detour.getTileRef(nav,tile),Detour.DT_SUCCESS
end


function Detour.addTileData(nav,data,lastRef)
    local tileNav,status,err=Detour.loadNavMeshData(data)
    if tileNav==nil then return nil,status,err end
    local h=tileNav.header
    local ref,addStatus=Detour.addTile(nav,tileNav,h.x,h.y,h.layer,lastRef)
    return ref,addStatus,tileNav
end

function Detour.removeTile(nav,tileRef)
    if not nav._tiled then return nil,bit32.bor(Detour.DT_FAILURE,Detour.DT_INVALID_PARAM) end
    local salt,it,ip=Detour.decodePolyId(nav,tileRef)
    if ip~=0 or it>=nav.maxTiles then return nil,bit32.bor(Detour.DT_FAILURE,Detour.DT_INVALID_PARAM) end
    local tile=nav.tiles[it+1]
    if tile==nil or tile.salt~=salt then return nil,bit32.bor(Detour.DT_FAILURE,Detour.DT_INVALID_PARAM) end

    local hash=tiledPosHash(tile.x,tile.y,nav.tileLutMask)
    local cur,prev=nav.posLookup[hash+1],-1
    while cur~=nil and cur>=0 do
        if cur==it then
            if prev>=0 then nav.tiles[prev+1]._nextPos=tile._nextPos else nav.posLookup[hash+1]=tile._nextPos end
            break
        end
        prev=cur; local ct=nav.tiles[cur+1]; cur=ct and ct._nextPos or -1
    end

    -- Invalidate refs and outgoing links in O(polys in removed tile). Incoming external links use
    -- encoded target refs, so their old salt simply stops resolving; no world scan is required.
    local removedWalkable,removedGround=0,0
    for lp=0,tile.polyCount-1 do
        local gp=tile.polyBase+lp
        local ref=Detour.encodePolyId(nav,tile.salt,tile.index,lp)
        nav.refToPoly[ref]=nil
        if buffer.readu8(nav.polyValid,gp)~=0 then removedWalkable+=1 end
        if polyType(nav,gp)==Detour.DT_POLYTYPE_GROUND then removedGround+=1 end
        buffer.writeu8(nav.polyValid,gp,0)
        buffer.writeu32(nav.polyRefs,gp*4,0)
        buffer.writei32(nav.extraFirstLinks,gp*4,-1)
    end
    nav.activePolyCount-=tile.polyCount; nav.activeVertexCount-=tile.vertexCount
    nav.activeWalkableCount-=removedWalkable; nav.activeGroundCount-=removedGround
    nav.walkableCount=nav.activeWalkableCount; nav.groundPolyCount=nav.activeGroundCount
    nav.tiles[it+1]=nil; tile._nextPos=-1
    local mask=refMask(nav.refCodec.saltBits); local ns=(salt+1)% (mask+1); if ns==0 then ns=1 end
    buffer.writeu32(nav.tileSalts,it*4,ns); nav.freeTiles[#nav.freeTiles+1]=it
    nav._slicedQuery=nil; nav._lastDijkstra=nil
    return tile.nav,Detour.DT_SUCCESS
end

function Detour.buildFromTriangles(vertices, triangles, config)
    config = config or {}
    local build = Recast.buildCompactFromTriangles(vertices, triangles, config)
    if config.compactCellNavmesh == true then
        -- Compatibility/debug path: the old two-triangles-per-compact-span representation.
        build.navmesh = Detour.buildCompactNavMesh(build.compact)
        return build
    end
    build.contours = Recast.buildContours(
        build.compact,
        config.maxSimplificationError or 1.3,
        config.maxEdgeLen or config.maxEdgeLength or 0,
        config.contourBuildFlags
    )
    build.polyMesh = Recast.buildPolyMesh(build.contours, config.maxVertsPerPoly or 6)

    -- Recast's standard pipeline produces a detail mesh after polygonization.  Keep the familiar
    -- RecastDemo configuration semantics: detailSampleDist and detailSampleMaxError are expressed
    -- in cell-size/cell-height units, while the *World aliases accept already-scaled distances.
    if config.buildDetailMesh ~= false then
        local sampleDist = config.detailSampleDistanceWorld
        if sampleDist == nil then
            local cells = config.detailSampleDist
            if cells == nil then cells = 6 end
            sampleDist = if cells < 0.9 then 0 else build.compact.cs * cells
        end
        local sampleMaxError = config.detailSampleMaxErrorWorld
        if sampleMaxError == nil then sampleMaxError = build.compact.ch * (config.detailSampleMaxError or 1) end
        build.detailMesh = Recast.buildPolyMeshDetail(build.polyMesh, build.compact, sampleDist, sampleMaxError)
    end

    build.navmesh = Detour.buildPolyMeshNavMesh(build.polyMesh)
    build.navmesh.compact = build.compact
    build.navmesh.detailMesh = build.detailMesh
    build.navmesh.walkableClimb = config.walkableClimbWorld or ((config.walkableClimb or 0) * build.compact.ch)
    if config.offMeshConnections ~= nil and #config.offMeshConnections > 0 then
        Detour.addOffMeshConnections(build.navmesh, config.offMeshConnections, {walkableClimb=build.navmesh.walkableClimb})
    end
    return build
end

return table.freeze(Detour)
