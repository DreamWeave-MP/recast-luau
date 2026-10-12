--!native

-- Pure-Luau implementation of DetourNavMeshBuilder's binary tile format.
-- Layout and packing intentionally match upstream Detour 1:1 (DT_NAVMESH_VERSION 7,
-- DT_VERTS_PER_POLYGON 6, 32-bit dtPolyRef configuration).
local data = require '@dream/data'

local Builder = {}

Builder.DT_NAVMESH_MAGIC = 0x444e4156 -- 'DNAV'
Builder.DT_NAVMESH_VERSION = 7
Builder.DT_VERTS_PER_POLYGON = 6
Builder.DT_EXT_LINK = 0x8000
Builder.DT_OFFMESH_CON_BIDIR = 1
Builder.DT_POLYTYPE_GROUND = 0
Builder.DT_POLYTYPE_OFFMESH_CONNECTION = 1
Builder.DT_MESH_NULL_IDX = 0xffff

local HEADER_BYTES = 100
local POLY_BYTES = 32
local LINK_BYTES = 12
local DETAIL_BYTES = 12
local BV_BYTES = 16
local OFFMESH_BYTES = 36
local MESH_NULL_IDX = 0xffff

local function align4(n)
    return bit32.band(n + 3, bit32.bnot(3))
end

local function clampi(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

local function bswap32(v)
    return bit32.bor(
        bit32.lshift(bit32.band(v, 0x000000ff), 24),
        bit32.lshift(bit32.band(v, 0x0000ff00), 8),
        bit32.rshift(bit32.band(v, 0x00ff0000), 8),
        bit32.rshift(bit32.band(v, 0xff000000), 24)
    )
end

local function swap2(data, o)
    local a = buffer.readu8(data, o)
    local b = buffer.readu8(data, o + 1)
    buffer.writeu8(data, o, b)
    buffer.writeu8(data, o + 1, a)
end

local function swap4(data, o)
    local a = buffer.readu8(data, o)
    local b = buffer.readu8(data, o + 1)
    local c = buffer.readu8(data, o + 2)
    local d = buffer.readu8(data, o + 3)
    buffer.writeu8(data, o, d)
    buffer.writeu8(data, o + 1, c)
    buffer.writeu8(data, o + 2, b)
    buffer.writeu8(data, o + 3, a)
end

local function vecFrom(value, fallback)
    if value == nil then return fallback end
    return value
end

local function classifyOffMeshPoint(p, bmin, bmax)
    local outcode = 0
    if p.X >= bmax.X then outcode = bit32.bor(outcode, 1) end
    if p.Z >= bmax.Z then outcode = bit32.bor(outcode, 2) end
    if p.X < bmin.X then outcode = bit32.bor(outcode, 4) end
    if p.Z < bmin.Z then outcode = bit32.bor(outcode, 8) end
    if outcode == 1 then return 0 end
    if outcode == 3 then return 1 end
    if outcode == 2 then return 2 end
    if outcode == 6 then return 3 end
    if outcode == 4 then return 4 end
    if outcode == 12 then return 5 end
    if outcode == 8 then return 6 end
    if outcode == 9 then return 7 end
    return 0xff
end

local function readF32Vec3(data, o)
    return vector.create(buffer.readf32(data, o), buffer.readf32(data, o + 4), buffer.readf32(data, o + 8))
end

local function writeF32Vec3(data, o, v)
    buffer.writef32(data, o, v.X)
    buffer.writef32(data, o + 4, v.Y)
    buffer.writef32(data, o + 8, v.Z)
end

local function polyVertCount(params, poly0)
    local base = poly0 * params.nvp * 4
    local n = 0
    for j0 = 0, params.nvp - 1 do
        if buffer.readu16(params.polys, base + j0 * 2) == MESH_NULL_IDX then break end
        n += 1
    end
    return n
end

-- A subtree of this many polygons or fewer is sorted here, by insertion; above it one stable
-- argsort in the data plane costs less than the loop, its calls included.
local InsertionLimit = 16

--- What the BV tree reads of a tile's normalized parameters.
type TreeParams = {
  verts: buffer,
  polys: buffer,
  polyCount: number,
  nvp: number,
  detailMeshes: buffer?,
  detailVerts: buffer?,
  bmin: vector,
  cs: number,
  ch: number,
}

--- Each polygon's quantized bounds, and its minimum again as three f32 columns (X, then Y, then
--- Z, each `polyCount` elements) for the sorts to read as keys.
local function quantizedBounds(params: TreeParams, polygonCount: number): ({ vector }, { vector }, buffer)
  local verts, polys, vertsPerPolygon = params.verts, params.polys, params.nvp
  local detailMeshes, detailVerts = params.detailMeshes, params.detailVerts
  local originX, originY, originZ = params.bmin.X, params.bmin.Y, params.bmin.Z
  local cellSize, cellHeight = params.cs, params.ch
  local mins: { vector }, maxs: { vector } = table.create(polygonCount), table.create(polygonCount)
  local keys = buffer.create(polygonCount * 12)
  local quantization = 1 / cellSize

  for polygon = 0, polygonCount - 1 do
    local minX, minY, minZ, maxX, maxY, maxZ

    if detailMeshes then
      local mesh = polygon * 16
      local firstVertex = buffer.readu32(detailMeshes, mesh)
      local vertexCount = buffer.readu32(detailMeshes, mesh + 4)
      local vertex = readF32Vec3(detailVerts, firstVertex * 12)
      local lowX, lowY, lowZ = vertex.X, vertex.Y, vertex.Z
      local highX, highY, highZ = lowX, lowY, lowZ

      for sample = 1, vertexCount - 1 do
        vertex = readF32Vec3(detailVerts, (firstVertex + sample) * 12)
        lowX, lowY, lowZ = math.min(lowX, vertex.X), math.min(lowY, vertex.Y), math.min(lowZ, vertex.Z)
        highX, highY, highZ = math.max(highX, vertex.X), math.max(highY, vertex.Y), math.max(highZ, vertex.Z)
      end

      minX = clampi(math.floor((lowX - originX) * quantization), 0, 0xffff)
      minY = clampi(math.floor((lowY - originY) * quantization), 0, 0xffff)
      minZ = clampi(math.floor((lowZ - originZ) * quantization), 0, 0xffff)
      maxX = clampi(math.floor((highX - originX) * quantization), 0, 0xffff)
      maxY = clampi(math.floor((highY - originY) * quantization), 0, 0xffff)
      maxZ = clampi(math.floor((highZ - originZ) * quantization), 0, 0xffff)
    else
      local corners = polygon * vertsPerPolygon * 4
      local vertex = buffer.readu16(polys, corners) * 6

      minX = buffer.readu16(verts, vertex)
      minY = buffer.readu16(verts, vertex + 2)
      minZ = buffer.readu16(verts, vertex + 4)
      maxX, maxY, maxZ = minX, minY, minZ

      for corner = 1, vertsPerPolygon - 1 do
        local index = buffer.readu16(polys, corners + corner * 2)

        if index == MESH_NULL_IDX then break end

        vertex = index * 6

        local cellX, cellY = buffer.readu16(verts, vertex), buffer.readu16(verts, vertex + 2)
        local cellZ = buffer.readu16(verts, vertex + 4)

        minX, minY, minZ = math.min(minX, cellX), math.min(minY, cellY), math.min(minZ, cellZ)
        maxX, maxY, maxZ = math.max(maxX, cellX), math.max(maxY, cellY), math.max(maxZ, cellZ)
      end

      minY = math.floor(minY * cellHeight / cellSize)
      maxY = math.ceil(maxY * cellHeight / cellSize)
    end

    mins[polygon + 1] = vector.create(minX, minY, minZ)
    maxs[polygon + 1] = vector.create(maxX, maxY, maxZ)
    buffer.writef32(keys, polygon * 4, minX)
    buffer.writef32(keys, (polygonCount + polygon) * 4, minY)
    buffer.writef32(keys, (polygonCount * 2 + polygon) * 4, minZ)
  end

  return mins, maxs, keys
end

local function writeBVNode(out: buffer, node: number, low: vector, high: vector, index: number)
  local offset = node * BV_BYTES

  buffer.writeu16(out, offset, low.X)
  buffer.writeu16(out, offset + 2, low.Y)
  buffer.writeu16(out, offset + 4, low.Z)
  buffer.writeu16(out, offset + 6, high.X)
  buffer.writeu16(out, offset + 8, high.Y)
  buffer.writeu16(out, offset + 10, high.Z)
  buffer.writei32(out, offset + 12, index)
end

--- The tile's dtBVNode array, in upstream's order: each subtree's polygons are sorted by their
--- minimum on the subtree's longest axis, then halved. That order is part of the tile's bytes,
--- and upstream's qsort leaves tied polygons as it found them, so every sort here is stable.
local function buildBVTree(params: TreeParams): buffer
  local nodes = buffer.create(params.polyCount * 2 * BV_BYTES)

  if params.polyCount == 0 then return nodes end

  -- A count that arrived as a numeric string has always built a tree, since arithmetic makes a
  -- number of it. The comparisons below would not.
  local polygonCount = tonumber(params.polyCount) :: number
  local mins, maxs, keys = quantizedBounds(params, polygonCount)
  local order = buffer.create(polygonCount * 4)
  -- What one argsort works in: the subtree's polygons, their keys in that order, the
  -- permutation that sorts them, and the merge's own scratch.
  local picked, pickedKeys = buffer.create(polygonCount * 4), buffer.create(polygonCount * 4)
  local permutation, scratch = buffer.create(polygonCount * 4), buffer.create(polygonCount * 4)
  local nodeCount = 0

  for polygon = 0, polygonCount - 1 do
    buffer.writeu32(order, polygon * 4, polygon)
  end

  --- Sorts the `count` polygons of `order` from `first` by the key column at byte `column`.
  local function sort(first: number, count: number, column: number)
    if count > InsertionLimit then
      buffer.copy(picked, 0, order, first * 4, count * 4)
      data.gather(keys, 'f32', column, polygonCount, picked, pickedKeys, 0, count)
      data.argsort(pickedKeys, 'f32', 0, count, permutation, 0, scratch)
      data.gather(picked, 'u32', 0, count, permutation, order, first * 4, count)

      return
    end

    for position = first + 1, first + count - 1 do
      local polygon = buffer.readu32(order, position * 4)
      local key = buffer.readf32(keys, column + polygon * 4)
      local slot = position

      while slot > first do
        local earlier = buffer.readu32(order, slot * 4 - 4)

        if buffer.readf32(keys, column + earlier * 4) <= key then break end

        buffer.writeu32(order, slot * 4, earlier)
        slot -= 1
      end

      buffer.writeu32(order, slot * 4, polygon)
    end
  end

  --- Writes the subtree over the `count` polygons of `order` from `first`. `sorted` is the axis
  --- those polygons are already in order on, when the parent's sort was on it: a stable sort of
  --- them on that axis again would move nothing.
  local function subdivide(first: number, count: number, sorted: number?)
    local node = nodeCount

    nodeCount += 1

    local polygon = buffer.readu32(order, first * 4) + 1
    local low, high = mins[polygon], maxs[polygon]

    if count == 1 then
      writeBVNode(nodes, node, low, high, polygon - 1)

      return
    end

    for position = first + 1, first + count - 1 do
      polygon = buffer.readu32(order, position * 4) + 1
      low = vector.min(low, mins[polygon])
      high = vector.max(high, maxs[polygon])
    end

    local extent = high - low
    local axis, longest = 0, extent.X

    if extent.Y > longest then
      axis, longest = 1, extent.Y
    end

    if extent.Z > longest then axis = 2 end

    if axis ~= sorted then sort(first, count, axis * polygonCount * 4) end

    local half = count // 2

    subdivide(first, half, axis)
    subdivide(first + half, count - half, axis)
    writeBVNode(nodes, node, low, high, node - nodeCount)
  end

  subdivide(0, polygonCount)

  return nodes
end

local function normalizeParams(params)
    local p = {}
    for k,v in params do p[k]=v end
    p.vertCount = p.vertCount or (p.verts and buffer.len(p.verts) // 6 or 0)
    p.polyCount = p.polyCount or (p.areas and buffer.len(p.areas) or 0)
    p.nvp = p.nvp or 6
    p.detailVertsCount = p.detailVertsCount or (p.detailVerts and buffer.len(p.detailVerts) // 12 or 0)
    p.detailTriCount = p.detailTriCount or (p.detailTris and buffer.len(p.detailTris) // 4 or 0)
    p.offMeshConCount = p.offMeshConCount or 0
    p.userId = p.userId or 0; p.tileX=p.tileX or 0; p.tileY=p.tileY or 0; p.tileLayer=p.tileLayer or 0
    p.bmin=vecFrom(p.bmin,vector.zero); p.bmax=vecFrom(p.bmax,vector.zero)
    p.walkableHeight=p.walkableHeight or 0; p.walkableRadius=p.walkableRadius or 0; p.walkableClimb=p.walkableClimb or 0
    p.cs=p.cs or 0; p.ch=p.ch or 0; p.buildBvTree=p.buildBvTree == true
    return p
end

function Builder.createNavMeshData(params)
    local p = normalizeParams(params or {})
    if p.nvp > Builder.DT_VERTS_PER_POLYGON then return nil, 'nvp exceeds DT_VERTS_PER_POLYGON' end
    if p.vertCount >= 0xffff or p.vertCount == 0 or p.verts == nil then return nil, 'invalid vertices' end
    if p.polyCount == 0 or p.polys == nil then return nil, 'invalid polygons' end
    if p.polyFlags == nil or p.polyAreas == nil then return nil, 'polyFlags and polyAreas are required' end
    if p.cs <= 0 or p.ch <= 0 then return nil, 'cs and ch must be positive' end

    local offClass = table.create(p.offMeshConCount * 2)
    local storedOffMeshConCount, offMeshConLinkCount = 0, 0
    if p.offMeshConCount > 0 then
        if p.offMeshConVerts==nil or p.offMeshConRad==nil or p.offMeshConFlags==nil or p.offMeshConAreas==nil or p.offMeshConDir==nil then
            return nil, 'incomplete off-mesh connection arrays'
        end
        local hmin, hmax = math.huge, -math.huge
        if p.detailVerts ~= nil and p.detailVertsCount > 0 then
            for i0=0,p.detailVertsCount-1 do
                local h=buffer.readf32(p.detailVerts,i0*12+4); hmin=math.min(hmin,h); hmax=math.max(hmax,h)
            end
        else
            for i0=0,p.vertCount-1 do
                local h=p.bmin.Y+buffer.readu16(p.verts,i0*6+2)*p.ch; hmin=math.min(hmin,h); hmax=math.max(hmax,h)
            end
        end
        hmin -= p.walkableClimb; hmax += p.walkableClimb
        local tightMin=vector.create(p.bmin.X,hmin,p.bmin.Z); local tightMax=vector.create(p.bmax.X,hmax,p.bmax.Z)
        for i0=0,p.offMeshConCount-1 do
            local p0=readF32Vec3(p.offMeshConVerts,i0*24)
            local p1=readF32Vec3(p.offMeshConVerts,i0*24+12)
            local c0=classifyOffMeshPoint(p0,tightMin,tightMax); local c1=classifyOffMeshPoint(p1,tightMin,tightMax)
            if c0==0xff and (p0.Y<hmin or p0.Y>hmax) then c0=0 end
            offClass[i0*2+1]=c0; offClass[i0*2+2]=c1
            if c0==0xff then offMeshConLinkCount+=1; storedOffMeshConCount+=1 end
            if c1==0xff then offMeshConLinkCount+=1 end
        end
    end

    local totalPolyCount=p.polyCount+storedOffMeshConCount
    local totalVertCount=p.vertCount+storedOffMeshConCount*2
    local edgeCount,portalCount=0,0
    for p0=0,p.polyCount-1 do
        local base=p0*p.nvp*4
        for j0=0,p.nvp-1 do
            if buffer.readu16(p.polys,base+j0*2)==MESH_NULL_IDX then break end
            edgeCount+=1
            local nei=buffer.readu16(p.polys,base+(p.nvp+j0)*2)
            if bit32.band(nei,0x8000)~=0 and bit32.band(nei,0xf)~=0xf then portalCount+=1 end
        end
    end
    local maxLinkCount=edgeCount+portalCount*2+offMeshConLinkCount*2

    local uniqueDetailVertCount,detailTriCount=0,0
    if p.detailMeshes~=nil then
        detailTriCount=p.detailTriCount
        for p0=0,p.polyCount-1 do
            local mo=p0*16
            local ndv=buffer.readu32(p.detailMeshes,mo+4)
            uniqueDetailVertCount += ndv-polyVertCount(p,p0)
        end
    else
        for p0=0,p.polyCount-1 do detailTriCount += polyVertCount(p,p0)-2 end
    end

    local headerSize=align4(HEADER_BYTES)
    local vertsSize=align4(12*totalVertCount)
    local polysSize=align4(POLY_BYTES*totalPolyCount)
    local linksSize=align4(LINK_BYTES*maxLinkCount)
    local detailMeshesSize=align4(DETAIL_BYTES*p.polyCount)
    local detailVertsSize=align4(12*uniqueDetailVertCount)
    local detailTrisSize=align4(4*detailTriCount)
    local bvTreeSize=if p.buildBvTree then align4(BV_BYTES*p.polyCount*2) else 0
    local offMeshSize=align4(OFFMESH_BYTES*storedOffMeshConCount)
    local dataSize=headerSize+vertsSize+polysSize+linksSize+detailMeshesSize+detailVertsSize+detailTrisSize+bvTreeSize+offMeshSize
    local data=buffer.create(dataSize)
    local vertsOff=headerSize
    local polysOff=vertsOff+vertsSize
    local linksOff=polysOff+polysSize
    local detailMeshesOff=linksOff+linksSize
    local detailVertsOff=detailMeshesOff+detailMeshesSize
    local detailTrisOff=detailVertsOff+detailVertsSize
    local bvOff=detailTrisOff+detailTrisSize
    local offMeshOff=bvOff+bvTreeSize

    -- dtMeshHeader
    buffer.writei32(data,0,Builder.DT_NAVMESH_MAGIC); buffer.writei32(data,4,Builder.DT_NAVMESH_VERSION)
    buffer.writei32(data,8,p.tileX); buffer.writei32(data,12,p.tileY); buffer.writei32(data,16,p.tileLayer); buffer.writeu32(data,20,p.userId)
    buffer.writei32(data,24,totalPolyCount); buffer.writei32(data,28,totalVertCount); buffer.writei32(data,32,maxLinkCount)
    buffer.writei32(data,36,p.polyCount); buffer.writei32(data,40,uniqueDetailVertCount); buffer.writei32(data,44,detailTriCount)
    buffer.writei32(data,48,if p.buildBvTree then p.polyCount*2 else 0); buffer.writei32(data,52,storedOffMeshConCount); buffer.writei32(data,56,p.polyCount)
    buffer.writef32(data,60,p.walkableHeight); buffer.writef32(data,64,p.walkableRadius); buffer.writef32(data,68,p.walkableClimb)
    writeF32Vec3(data,72,p.bmin); writeF32Vec3(data,84,p.bmax); buffer.writef32(data,96,1/p.cs)

    for i0=0,p.vertCount-1 do
        local io=i0*6; local oo=vertsOff+i0*12
        buffer.writef32(data,oo,p.bmin.X+buffer.readu16(p.verts,io)*p.cs)
        buffer.writef32(data,oo+4,p.bmin.Y+buffer.readu16(p.verts,io+2)*p.ch)
        buffer.writef32(data,oo+8,p.bmin.Z+buffer.readu16(p.verts,io+4)*p.cs)
    end
    local stored=0
    for i0=0,p.offMeshConCount-1 do
        if offClass[i0*2+1]~=0xff then continue end
        buffer.copy(data,vertsOff+(p.vertCount+stored*2)*12,p.offMeshConVerts,i0*24,24)
        stored+=1
    end

    for p0=0,p.polyCount-1 do
        local po=polysOff+p0*POLY_BYTES
        local src=p0*p.nvp*4
        local count=0
        buffer.writeu16(data,po+28,buffer.readu16(p.polyFlags,p0*2))
        buffer.writeu8(data,po+31,bit32.band(buffer.readu8(p.polyAreas,p0),0x3f))
        for j0=0,p.nvp-1 do
            local vi=buffer.readu16(p.polys,src+j0*2)
            if vi==MESH_NULL_IDX then break end
            buffer.writeu16(data,po+4+j0*2,vi)
            local nei=buffer.readu16(p.polys,src+(p.nvp+j0)*2)
            local outNei=0
            if bit32.band(nei,0x8000)~=0 then
                local dir=bit32.band(nei,0xf)
                if dir==0 then outNei=bit32.bor(Builder.DT_EXT_LINK,4)
                elseif dir==1 then outNei=bit32.bor(Builder.DT_EXT_LINK,2)
                elseif dir==2 then outNei=bit32.bor(Builder.DT_EXT_LINK,0)
                elseif dir==3 then outNei=bit32.bor(Builder.DT_EXT_LINK,6) end
            elseif nei~=MESH_NULL_IDX then outNei=nei+1 end
            buffer.writeu16(data,po+16+j0*2,outNei)
            count+=1
        end
        buffer.writeu8(data,po+30,count)
    end

    stored=0
    for i0=0,p.offMeshConCount-1 do
        if offClass[i0*2+1]~=0xff then continue end
        local po=polysOff+(p.polyCount+stored)*POLY_BYTES
        buffer.writeu16(data,po+4,p.vertCount+stored*2); buffer.writeu16(data,po+6,p.vertCount+stored*2+1)
        buffer.writeu16(data,po+28,buffer.readu16(p.offMeshConFlags,i0*2)); buffer.writeu8(data,po+30,2)
        buffer.writeu8(data,po+31,bit32.bor(bit32.band(buffer.readu8(p.offMeshConAreas,i0),0x3f),bit32.lshift(Builder.DT_POLYTYPE_OFFMESH_CONNECTION,6)))
        stored+=1
    end

    if p.detailMeshes~=nil then
        local vbase=0
        for p0=0,p.polyCount-1 do
            local mi=p0*16; local mo=detailMeshesOff+p0*DETAIL_BYTES
            local vb=buffer.readu32(p.detailMeshes,mi); local ndv=buffer.readu32(p.detailMeshes,mi+4)
            local nv=polyVertCount(p,p0); local extra=ndv-nv
            buffer.writeu32(data,mo,vbase); buffer.writeu32(data,mo+4,buffer.readu32(p.detailMeshes,mi+8))
            buffer.writeu8(data,mo+8,extra); buffer.writeu8(data,mo+9,buffer.readu32(p.detailMeshes,mi+12))
            if extra>0 then buffer.copy(data,detailVertsOff+vbase*12,p.detailVerts,(vb+nv)*12,extra*12); vbase+=extra end
        end
        if p.detailTriCount>0 then buffer.copy(data,detailTrisOff,p.detailTris,0,p.detailTriCount*4) end
    else
        local tbase=0
        for p0=0,p.polyCount-1 do
            local mo=detailMeshesOff+p0*DETAIL_BYTES; local nv=polyVertCount(p,p0)
            buffer.writeu32(data,mo,0); buffer.writeu32(data,mo+4,tbase); buffer.writeu8(data,mo+8,0); buffer.writeu8(data,mo+9,nv-2)
            for j=2,nv-1 do
                local to=detailTrisOff+tbase*4
                buffer.writeu8(data,to,0); buffer.writeu8(data,to+1,j-1); buffer.writeu8(data,to+2,j)
                local flags=bit32.lshift(1,2)
                if j==2 then flags=bit32.bor(flags,bit32.lshift(1,0)) end
                if j==nv-1 then flags=bit32.bor(flags,bit32.lshift(1,4)) end
                buffer.writeu8(data,to+3,flags); tbase+=1
            end
        end
    end

    if p.buildBvTree then
        local bv=buildBVTree(p)
        if buffer.len(bv)>0 then buffer.copy(data,bvOff,bv,0,buffer.len(bv)) end
    end

    stored=0
    for i0=0,p.offMeshConCount-1 do
        if offClass[i0*2+1]~=0xff then continue end
        local oo=offMeshOff+stored*OFFMESH_BYTES
        buffer.copy(data,oo,p.offMeshConVerts,i0*24,24)
        buffer.writef32(data,oo+24,buffer.readf32(p.offMeshConRad,i0*4))
        buffer.writeu16(data,oo+28,p.polyCount+stored)
        buffer.writeu8(data,oo+30,if buffer.readu8(p.offMeshConDir,i0)~=0 then Builder.DT_OFFMESH_CON_BIDIR else 0)
        buffer.writeu8(data,oo+31,offClass[i0*2+2])
        if p.offMeshConUserID~=nil then buffer.writeu32(data,oo+32,buffer.readu32(p.offMeshConUserID,i0*4)) end
        stored+=1
    end

    return data
end

function Builder.createNavMeshDataFromPolyMesh(mesh, detail, options)
    options=options or {}
    return Builder.createNavMeshData({
        verts=mesh.verts,vertCount=mesh.nverts,polys=mesh.polys,polyFlags=mesh.flags,polyAreas=mesh.areas,polyCount=mesh.npolys,nvp=mesh.nvp,
        detailMeshes=detail and detail.meshes or nil,detailVerts=detail and detail.verts or nil,detailVertsCount=detail and detail.nverts or 0,
        detailTris=detail and detail.tris or nil,detailTriCount=detail and detail.ntris or 0,
        offMeshConVerts=options.offMeshConVerts,offMeshConRad=options.offMeshConRad,offMeshConFlags=options.offMeshConFlags,
        offMeshConAreas=options.offMeshConAreas,offMeshConDir=options.offMeshConDir,offMeshConUserID=options.offMeshConUserID,offMeshConCount=options.offMeshConCount or 0,
        userId=options.userId or 0,tileX=options.tileX or 0,tileY=options.tileY or 0,tileLayer=options.tileLayer or 0,
        bmin=mesh.bmin,bmax=mesh.bmax,walkableHeight=options.walkableHeight or 0,walkableRadius=options.walkableRadius or 0,
        walkableClimb=options.walkableClimb or mesh.walkableClimb or 0,cs=mesh.cs,ch=mesh.ch,buildBvTree=options.buildBvTree==true,
    })
end

function Builder.navMeshHeader(data)
    if data==nil or buffer.len(data)<HEADER_BYTES then return nil end
    return {
        magic=buffer.readi32(data,0),version=buffer.readi32(data,4),x=buffer.readi32(data,8),y=buffer.readi32(data,12),layer=buffer.readi32(data,16),
        userId=buffer.readu32(data,20),polyCount=buffer.readi32(data,24),vertCount=buffer.readi32(data,28),maxLinkCount=buffer.readi32(data,32),
        detailMeshCount=buffer.readi32(data,36),detailVertCount=buffer.readi32(data,40),detailTriCount=buffer.readi32(data,44),bvNodeCount=buffer.readi32(data,48),
        offMeshConCount=buffer.readi32(data,52),offMeshBase=buffer.readi32(data,56),walkableHeight=buffer.readf32(data,60),walkableRadius=buffer.readf32(data,64),walkableClimb=buffer.readf32(data,68),
        bmin=readF32Vec3(data,72),bmax=readF32Vec3(data,84),bvQuantFactor=buffer.readf32(data,96),
    }
end


function Builder.decodeNavMeshData(data)
    local h=Builder.navMeshHeader(data)
    if h==nil then return nil,'tile data is shorter than dtMeshHeader' end
    if h.magic~=Builder.DT_NAVMESH_MAGIC then return nil,'wrong navmesh magic' end
    if h.version~=Builder.DT_NAVMESH_VERSION then return nil,'wrong navmesh version' end
    if h.polyCount<0 or h.vertCount<0 or h.detailMeshCount<0 or h.detailVertCount<0 or h.detailTriCount<0 or h.bvNodeCount<0 or h.offMeshConCount<0 then return nil,'negative navmesh counts' end

    local vertsSize=align4(12*h.vertCount); local polysSize=align4(POLY_BYTES*h.polyCount); local linksSize=align4(LINK_BYTES*h.maxLinkCount)
    local detailMeshesSize=align4(DETAIL_BYTES*h.detailMeshCount); local detailVertsSize=align4(12*h.detailVertCount); local detailTrisSize=align4(4*h.detailTriCount)
    local bvSize=align4(BV_BYTES*h.bvNodeCount); local offSize=align4(OFFMESH_BYTES*h.offMeshConCount)
    local vertsOff=HEADER_BYTES; local polysOff=vertsOff+vertsSize; local linksOff=polysOff+polysSize
    local detailMeshesOff=linksOff+linksSize; local detailVertsOff=detailMeshesOff+detailMeshesSize; local detailTrisOff=detailVertsOff+detailVertsSize
    local bvOff=detailTrisOff+detailTrisSize; local offOff=bvOff+bvSize
    if offOff+offSize>buffer.len(data) then return nil,'truncated navmesh tile data' end

    local vertices=buffer.create(h.vertCount*12)
    if h.vertCount>0 then buffer.copy(vertices,0,data,vertsOff,h.vertCount*12) end
    local nvp=Builder.DT_VERTS_PER_POLYGON
    local polyIndices=buffer.create(h.polyCount*nvp*2); buffer.fill(polyIndices,0,0xff,buffer.len(polyIndices))
    local polyVertCounts=buffer.create(h.polyCount)
    local neighbors=buffer.create(h.polyCount*nvp*4); buffer.fill(neighbors,0,0xff,buffer.len(neighbors))
    local portalDirs=buffer.create(h.polyCount*nvp); buffer.fill(portalDirs,0,0xff,buffer.len(portalDirs))
    local flags=buffer.create(h.polyCount*2); local areas=buffer.create(h.polyCount); local types=buffer.create(h.polyCount); local valid=buffer.create(h.polyCount)
    local centers=buffer.create(h.polyCount*12)
    local groundCount=0
    for p0=0,h.polyCount-1 do
        local po=polysOff+p0*POLY_BYTES
        local nv=buffer.readu8(data,po+30); buffer.writeu8(polyVertCounts,p0,nv)
        buffer.writeu16(flags,p0*2,buffer.readu16(data,po+28))
        local at=buffer.readu8(data,po+31); local typ=bit32.rshift(at,6); buffer.writeu8(areas,p0,bit32.band(at,0x3f)); buffer.writeu8(types,p0,typ); buffer.writeu8(valid,p0,1)
        if typ==Builder.DT_POLYTYPE_GROUND then groundCount+=1 end
        local center=vector.zero
        for e0=0,nv-1 do
            local vi=buffer.readu16(data,po+4+e0*2); buffer.writeu16(polyIndices,(p0*nvp+e0)*2,vi)
            center += readF32Vec3(vertices,vi*12)
            local nei=buffer.readu16(data,po+16+e0*2)
            if nei~=0 then
                if bit32.band(nei,Builder.DT_EXT_LINK)~=0 then
                    local side=bit32.band(nei,0xff)
                    local dir=if side==4 then 0 elseif side==2 then 1 elseif side==0 then 2 elseif side==6 then 3 else 0xff
                    buffer.writeu8(portalDirs,p0*nvp+e0,dir)
                else buffer.writei32(neighbors,(p0*nvp+e0)*4,nei-1) end
            end
        end
        if nv>0 then writeF32Vec3(centers,p0*12,center/nv) end
    end

    -- Expand Detour's compressed detail representation back into the rcPolyMeshDetail-shaped
    -- representation used by the Luau query engine: each submesh begins with its nav-poly verts,
    -- followed by the unique serialized detail verts.
    local detailMesh=nil
    if h.detailMeshCount>0 then
        local fullVertCount=0
        for p0=0,h.detailMeshCount-1 do fullVertCount += buffer.readu8(polyVertCounts,p0)+buffer.readu8(data,detailMeshesOff+p0*DETAIL_BYTES+8) end
        local dmMeshes=buffer.create(h.detailMeshCount*16); local dmVerts=buffer.create(fullVertCount*12); local dmTris=buffer.create(h.detailTriCount*4)
        if h.detailTriCount>0 then buffer.copy(dmTris,0,data,detailTrisOff,h.detailTriCount*4) end
        local fullBase=0
        for p0=0,h.detailMeshCount-1 do
            local so=detailMeshesOff+p0*DETAIL_BYTES; local mo=p0*16
            local extraBase=buffer.readu32(data,so); local triBase=buffer.readu32(data,so+4); local extraCount=buffer.readu8(data,so+8); local triCount=buffer.readu8(data,so+9)
            local nv=buffer.readu8(polyVertCounts,p0)
            buffer.writeu32(dmMeshes,mo,fullBase); buffer.writeu32(dmMeshes,mo+4,nv+extraCount); buffer.writeu32(dmMeshes,mo+8,triBase); buffer.writeu32(dmMeshes,mo+12,triCount)
            for e0=0,nv-1 do local vi=buffer.readu16(polyIndices,(p0*nvp+e0)*2); buffer.copy(dmVerts,(fullBase+e0)*12,vertices,vi*12,12) end
            if extraCount>0 then buffer.copy(dmVerts,(fullBase+nv)*12,data,detailVertsOff+extraBase*12,extraCount*12) end
            fullBase += nv+extraCount
        end
        detailMesh={meshes=dmMeshes,verts=dmVerts,tris=dmTris,nmeshes=h.detailMeshCount,nverts=fullVertCount,ntris=h.detailTriCount}
    end

    local bvTree=nil
    if h.bvNodeCount>0 then bvTree=buffer.create(h.bvNodeCount*BV_BYTES); buffer.copy(bvTree,0,data,bvOff,h.bvNodeCount*BV_BYTES) end
    local offMesh={}
    for i0=0,h.offMeshConCount-1 do
        local o=offOff+i0*OFFMESH_BYTES
        offMesh[i0+1]={
            start=readF32Vec3(data,o),finish=readF32Vec3(data,o+12),radius=buffer.readf32(data,o+24),poly=buffer.readu16(data,o+28),
            bidirectional=bit32.band(buffer.readu8(data,o+30),Builder.DT_OFFMESH_CON_BIDIR)~=0,side=buffer.readu8(data,o+31),userId=buffer.readu32(data,o+32),
        }
    end
    return {
        header=h,vertices=vertices,vertexCount=h.vertCount,polyIndices=polyIndices,polyIndexBytes=2,polyVertCounts=polyVertCounts,nvp=nvp,
        polyCount=h.polyCount,groundPolyCount=groundCount,walkableCount=h.polyCount,neighbors=neighbors,centers=centers,areas=areas,flags=flags,polyTypes=types,polyValid=valid,
        portalDirs=portalDirs,bmin=h.bmin,bmax=h.bmax,walkableHeight=h.walkableHeight,walkableRadius=h.walkableRadius,walkableClimb=h.walkableClimb,userId=h.userId,detailMesh=detailMesh,
        bvTree=bvTree,bvNodeCount=h.bvNodeCount,bvQuantFactor=h.bvQuantFactor,offMeshRecords=offMesh,offMeshPolyBase=h.offMeshBase,
        cellSize=if h.bvQuantFactor~=0 then 1/h.bvQuantFactor else nil,
    }
end

function Builder.navMeshHeaderSwapEndian(data)
    if data==nil or buffer.len(data)<HEADER_BYTES then return false end
    local magic,version=buffer.readu32(data,0),buffer.readu32(data,4)
    if not ((magic==Builder.DT_NAVMESH_MAGIC and version==Builder.DT_NAVMESH_VERSION) or (magic==bswap32(Builder.DT_NAVMESH_MAGIC) and version==bswap32(Builder.DT_NAVMESH_VERSION))) then return false end
    for o=0,56,4 do swap4(data,o) end
    for o=60,96,4 do swap4(data,o) end
    return true
end

function Builder.navMeshDataSwapEndian(data)
    local h=Builder.navMeshHeader(data)
    if h==nil or h.magic~=Builder.DT_NAVMESH_MAGIC or h.version~=Builder.DT_NAVMESH_VERSION then return false end
    local vertsSize=align4(12*h.vertCount); local polysSize=align4(POLY_BYTES*h.polyCount); local linksSize=align4(LINK_BYTES*h.maxLinkCount)
    local detailMeshesSize=align4(DETAIL_BYTES*h.detailMeshCount); local detailVertsSize=align4(12*h.detailVertCount); local detailTrisSize=align4(4*h.detailTriCount)
    local bvSize=align4(BV_BYTES*h.bvNodeCount)
    local vertsOff=HEADER_BYTES; local polysOff=vertsOff+vertsSize; local linksOff=polysOff+polysSize
    local detailMeshesOff=linksOff+linksSize; local detailVertsOff=detailMeshesOff+detailMeshesSize; local detailTrisOff=detailVertsOff+detailVertsSize
    local bvOff=detailTrisOff+detailTrisSize; local offOff=bvOff+bvSize
    if offOff+align4(OFFMESH_BYTES*h.offMeshConCount)>buffer.len(data) then return false end
    for i=0,h.vertCount*3-1 do swap4(data,vertsOff+i*4) end
    for p0=0,h.polyCount-1 do
        local po=polysOff+p0*POLY_BYTES
        for j=0,Builder.DT_VERTS_PER_POLYGON-1 do swap2(data,po+4+j*2); swap2(data,po+16+j*2) end
        swap2(data,po+28)
    end
    for i=0,h.detailMeshCount-1 do local o=detailMeshesOff+i*DETAIL_BYTES; swap4(data,o); swap4(data,o+4) end
    for i=0,h.detailVertCount*3-1 do swap4(data,detailVertsOff+i*4) end
    for i=0,h.bvNodeCount-1 do local o=bvOff+i*BV_BYTES; for j=0,5 do swap2(data,o+j*2) end; swap4(data,o+12) end
    for i=0,h.offMeshConCount-1 do local o=offOff+i*OFFMESH_BYTES; for j=0,6 do swap4(data,o+j*4) end; swap2(data,o+28); swap4(data,o+32) end
    return true
end

Builder.HEADER_BYTES=HEADER_BYTES
Builder.POLY_BYTES=POLY_BYTES
Builder.LINK_BYTES=LINK_BYTES
Builder.DETAIL_BYTES=DETAIL_BYTES
Builder.BV_BYTES=BV_BYTES
Builder.OFFMESH_BYTES=OFFMESH_BYTES

return table.freeze(Builder)
