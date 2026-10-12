--!native

-- Pure-Luau port of RecastMeshDetail.cpp's detail-mesh construction.
-- Uses packed buffers at the public boundary and compact tables/vectors for transient triangulation.

local Common = require('@recast/common')
local function writeVec3f(target, index1, v)
    local o = (index1 - 1) * 12
    buffer.writef32(target,o,v.X); buffer.writef32(target,o+4,v.Y); buffer.writef32(target,o+8,v.Z)
end
local Detail = {}

local RC_UNSET_HEIGHT = 0xffff
local RC_NOT_CONNECTED = 0x3f
local RC_MULTIPLE_REGS = 0
local RC_MESH_NULL_IDX = 0xffff
local DETAIL_EDGE_BOUNDARY = 0x1
local EV_UNDEF = -1
local EV_HULL = -2
local MAX_VERTS = 127
local MAX_TRIS = 255
local MAX_VERTS_PER_EDGE = 32
-- How far inside or outside a circumcircle a point may sit and still count as on it.
local CircleTolerance = 0.001
local DIR_X = {-1, 0, 1, 0}
local DIR_Z = {0, 1, 0, -1}

local function getCon(chf, span, dir0)
    return bit32.band(bit32.rshift(buffer.readu32(chf.con, span * 4), dir0 * 6), 0x3f)
end

-- Detail generation revisits compact connectivity while constructing a height patch for every
-- polygon.  Materialize direct span indices once for this phase only; doing this globally during
-- the base Recast build is a net loss, but detail sampling has enough reuse to amortize it.
local function buildDetailNeighborIndex(chf)
    local neighbors = table.create(chf.spanCount * 4, -1)
    for z = 0, chf.height - 1 do
        for x = 0, chf.width - 1 do
            local cell = x + z * chf.width
            local first = buffer.readu32(chf.cellIndex, cell * 4)
            local count = buffer.readu8(chf.cellCount, cell)
            for li = 0, count - 1 do
                local span = first + li
                local conBits = buffer.readu32(chf.con, span * 4)
                for dir0 = 0, 3 do
                    local con = bit32.band(bit32.rshift(conBits, dir0 * 6), 0x3f)
                    if con ~= RC_NOT_CONNECTED then
                        local nx, nz = x + DIR_X[dir0 + 1], z + DIR_Z[dir0 + 1]
                        local ni = buffer.readu32(chf.cellIndex, (nx + nz * chf.width) * 4) + con
                        neighbors[span * 4 + dir0 + 1] = ni
                    end
                end
            end
        end
    end
    return neighbors
end



local function vdot2(a,b) return a.X*b.X + a.Z*b.Z end
local function vdistSq2(a,b) local dx,dz=b.X-a.X,b.Z-a.Z; return dx*dx+dz*dz end
local function vdist2(a,b) return math.sqrt(vdistSq2(a,b)) end
local function vcross2(a,b,c) return (b.X-a.X)*(c.Z-a.Z)-(b.Z-a.Z)*(c.X-a.X) end

local function circumCircle(p1,p2,p3)
    local v2,v3=p2-p1,p3-p1
    local cp=v2.X*v3.Z-v2.Z*v3.X
    if math.abs(cp)<=1e-6 then return p1,0,false end
    local v2sq=vdot2(v2,v2); local v3sq=vdot2(v3,v3)
    local cx=(v2sq*v3.Z-v3sq*v2.Z)/(2*cp)
    -- p1-relative specialization of Recast's determinant.
    local cz=(-v2sq*v3.X+v3sq*v2.X)/(2*cp)
    local c=vector.create(cx,0,cz)+p1
    return c,vdist2(c,p1),true
end

-- distPtTri measures a point against a triangle: the height between them where the point is over
-- the triangle, and nothing where it is not. Most of that arithmetic is the triangle's alone, and
-- the sampling loop measures every remaining sample against every triangle on every pass, so the
-- triangle's share is worked out once per triangulation. A triangle is the corner the rest is
-- measured from and ten f64s: its edge to the third corner and to the second (X and Z of each),
-- their three dot products, the reciprocal of the determinant, and the two edges' Y.
local PlaneBytes = 80

--- The planes of `tris`, less the triangles with no area: nothing is over one of those.
local function trianglePlanes(verts: { vector }, tris: { number }): ({ vector }, buffer)
  local origins: { vector } = table.create(#tris // 4)
  local terms = buffer.create(#tris // 4 * PlaneBytes)

  for offset = 1, #tris, 4 do
    local origin = verts[tris[offset] + 1]
    local far, near = verts[tris[offset + 2] + 1] - origin, verts[tris[offset + 1] + 1] - origin
    local farFar, farNear = far.X * far.X + far.Z * far.Z, far.X * near.X + far.Z * near.Z
    local nearNear = near.X * near.X + near.Z * near.Z
    local determinant = farFar * nearNear - farNear * farNear

    if math.abs(determinant) < 1e-12 then continue end

    local plane = #origins * PlaneBytes

    table.insert(origins, origin)
    buffer.writef64(terms, plane, far.X)
    buffer.writef64(terms, plane + 8, far.Z)
    buffer.writef64(terms, plane + 16, near.X)
    buffer.writef64(terms, plane + 24, near.Z)
    buffer.writef64(terms, plane + 32, farFar)
    buffer.writef64(terms, plane + 40, farNear)
    buffer.writef64(terms, plane + 48, nearNear)
    buffer.writef64(terms, plane + 56, 1 / determinant)
    buffer.writef64(terms, plane + 64, far.Y)
    buffer.writef64(terms, plane + 72, near.Y)
  end

  return origins, terms
end

--- The least height between `point` and a triangle it is over, or -1 when it is over none.
local function distanceToMesh(point: vector, origins: { vector }, terms: buffer): number
  local best = math.huge

  for triangle, origin in origins do
    local plane = (triangle - 1) * PlaneBytes
    local reach = point - origin
    local reachX, reachZ = reach.X, reach.Z
    local farReach = buffer.readf64(terms, plane) * reachX + buffer.readf64(terms, plane + 8) * reachZ
    local nearReach = buffer.readf64(terms, plane + 16) * reachX + buffer.readf64(terms, plane + 24) * reachZ
    local farNear, inverse = buffer.readf64(terms, plane + 40), buffer.readf64(terms, plane + 56)
    local alongFar = (buffer.readf64(terms, plane + 48) * farReach - farNear * nearReach) * inverse
    local alongNear = (buffer.readf64(terms, plane + 32) * nearReach - farNear * farReach) * inverse

    -- Written as the test a point over the triangle passes: one that is not a number fails it.
    if not (alongFar >= -1e-4 and alongNear >= -1e-4 and alongFar + alongNear <= 1.0001) then continue end

    local farY, nearY = buffer.readf64(terms, plane + 64), buffer.readf64(terms, plane + 72)
    local distance = math.abs((origin.Y + farY * alongFar + nearY * alongNear) - point.Y)

    if distance < best then best = distance end
  end

  return if best == math.huge then -1 else best
end

local function distancePtSeg(pt,p,q)
    local pq=q-p; local d=vector.dot(pq,pq); local t=vector.dot(pq,pt-p)
    if d>0 then t/=d end; t=math.clamp(t,0,1)
    local delta=p+pq*t-pt
    return vector.dot(delta,delta)
end

local function distancePtSeg2d(pt,p,q)
    local pqx,pqz=q.X-p.X,q.Z-p.Z; local dx,dz=pt.X-p.X,pt.Z-p.Z
    local d=pqx*pqx+pqz*pqz; local t=pqx*dx+pqz*dz
    if d>0 then t/=d end; t=math.clamp(t,0,1)
    dx=p.X+t*pqx-pt.X; dz=p.Z+t*pqz-pt.Z
    return dx*dx+dz*dz
end

local function distToPoly(verts,p)
    local dmin=math.huge; local inside=false; local j=#verts
    for i=1,#verts do
        local vi,vj=verts[i],verts[j]
        if (vi.Z>p.Z) ~= (vj.Z>p.Z) and p.X < (vj.X-vi.X)*(p.Z-vi.Z)/(vj.Z-vi.Z)+vi.X then inside=not inside end
        dmin=math.min(dmin,distancePtSeg2d(p,vj,vi)); j=i
    end
    return if inside then -dmin else dmin
end

local function patchRead(hp,x,z) return buffer.readu16(hp.data,(x+z*hp.width)*2) end
local function patchWrite(hp,x,z,v) buffer.writeu16(hp.data,(x+z*hp.width)*2,v) end

local function getHeight(fx,fy,fz,ics,ch,radius,hp)
    local ix=math.floor(fx*ics+0.01)-hp.xmin
    local iz=math.floor(fz*ics+0.01)-hp.ymin
    ix=math.clamp(ix,0,hp.width-1); iz=math.clamp(iz,0,hp.height-1)
    local height=patchRead(hp,ix,iz)
    if height~=RC_UNSET_HEIGHT then return height end
    local x,z,dx,dz=1,0,1,0
    local maxSize=radius*2+1; local maxIter=maxSize*maxSize-1
    local nextRingStart,nextRingIters=8,16
    local dmin=math.huge
    for i=0,maxIter-1 do
        local nx,nz=ix+x,iz+z
        if nx>=0 and nz>=0 and nx<hp.width and nz<hp.height then
            local nh=patchRead(hp,nx,nz)
            if nh~=RC_UNSET_HEIGHT then
                local d=math.abs(nh*ch-fy)
                if d<dmin then height,dmin=nh,d end
            end
        end
        if i+1==nextRingStart then
            if height~=RC_UNSET_HEIGHT then break end
            nextRingStart+=nextRingIters; nextRingIters+=8
        end
        if x==z or (x<0 and x==-z) or (x>0 and x==1-z) then dx,dz=-dz,dx end
        x+=dx; z+=dz
    end
    return height
end

-- The Delaunay pass keeps its edges as four numbers each (the two points, then the face on the
-- left and on the right), and asks which edge joins two points more often than it does anything
-- else. `slots` answers that in one read: for points `from` and `to` of `count`, the u16 at
-- `from * count + to` is the edge's index plus one, or zero when no edge joins them. An edge is
-- recorded both ways round, and no pair of points ever gets a second one.
local function findEdge(slots: buffer, count: number, from: number, to: number): number
  return buffer.readu16(slots, (from * count + to) * 2) - 1
end

local function addEdge(
  edges: { number },
  slots: buffer,
  count: number,
  maxEdges: number,
  from: number,
  to: number,
  left: number,
  right: number
)
  local edge = #edges // 4

  if edge >= maxEdges or findEdge(slots, count, from, to) ~= EV_UNDEF then return end

  local offset = edge * 4 + 1

  edges[offset], edges[offset + 1], edges[offset + 2], edges[offset + 3] = from, to, left, right
  buffer.writeu16(slots, (from * count + to) * 2, edge + 1)
  buffer.writeu16(slots, (to * count + from) * 2, edge + 1)
end

local function updateLeftFace(edges: { number }, edge: number, from: number, to: number, face: number)
  local offset = edge * 4 + 1
  local start, finish = edges[offset], edges[offset + 1]

  if start == from and finish == to and edges[offset + 2] == EV_UNDEF then
    edges[offset + 2] = face
  elseif finish == from and start == to and edges[offset + 3] == EV_UNDEF then
    edges[offset + 3] = face
  end
end

local function overlapSegSeg2d(a,b,c,d)
    local a1,a2=vcross2(a,b,d),vcross2(a,b,c)
    if a1*a2<0 then local a3=vcross2(c,d,a); local a4=a3+a2-a1; if a3*a4<0 then return true end end
    return false
end

local function overlapEdges(points: { vector }, edges: { number }, from: number, to: number): boolean
  for offset = 1, #edges, 4 do
    local start, finish = edges[offset], edges[offset + 1]

    if start == from or start == to or finish == from or finish == to then continue end

    if overlapSegSeg2d(points[start + 1], points[finish + 1], points[from + 1], points[to + 1]) then return true end
  end

  return false
end

--- Closes the open side of `edge` with the point whose circumcircle holds no other, and returns
--- the face count afterwards.
local function completeFacet(
  points: { vector },
  edges: { number },
  slots: buffer,
  maxEdges: number,
  faceCount: number,
  edge: number
): number
  local offset = edge * 4 + 1
  local from, to

  if edges[offset + 2] == EV_UNDEF then
    from, to = edges[offset], edges[offset + 1]
  elseif edges[offset + 3] == EV_UNDEF then
    from, to = edges[offset + 1], edges[offset]
  else
    return faceCount
  end

  local count = #points
  local start, finish = points[from + 1], points[to + 1]
  local alongX, alongZ = finish.X - start.X, finish.Z - start.Z
  local apex, center, radius = count, vector.zero, -1

  for candidate = 0, count - 1 do
    if candidate == from or candidate == to then continue end

    local point = points[candidate + 1]

    -- Only a point to the left of the edge can close it. Written as the test such a point
    -- passes, so a cross product that is not a number rules its point out, as upstream's does.
    if not (alongX * (point.Z - start.Z) - alongZ * (point.X - start.X) > 1e-5) then continue end

    if radius < 0 then
      apex = candidate
      center, radius = circumCircle(start, finish, point)

      continue
    end

    local offsetX, offsetZ = point.X - center.X, point.Z - center.Z
    local distance = math.sqrt(offsetX * offsetX + offsetZ * offsetZ)

    if distance > radius * (1 + CircleTolerance) then continue end

    -- On the circle, within tolerance: the point takes over only if its edges cross none.
    if
      not (distance < radius * (1 - CircleTolerance))
      and (overlapEdges(points, edges, from, candidate) or overlapEdges(points, edges, to, candidate))
    then
      continue
    end

    apex = candidate
    center, radius = circumCircle(start, finish, point)
  end

  if apex >= count then
    updateLeftFace(edges, edge, from, to, EV_HULL)

    return faceCount
  end

  updateLeftFace(edges, edge, from, to, faceCount)

  local closing = findEdge(slots, count, apex, from)

  if closing == EV_UNDEF then
    addEdge(edges, slots, count, maxEdges, apex, from, faceCount, EV_UNDEF)
  else
    updateLeftFace(edges, closing, apex, from, faceCount)
  end

  closing = findEdge(slots, count, to, apex)

  if closing == EV_UNDEF then
    addEdge(edges, slots, count, maxEdges, to, apex, faceCount, EV_UNDEF)
  else
    updateLeftFace(edges, closing, to, apex, faceCount)
  end

  return faceCount + 1
end

local function delaunayHull(points: { vector }, hull: { number }): { number }
  local count = #points
  local edges: { number } = {}
  local slots = buffer.create(count * count * 2)
  local faceCount, maxEdges = 0, count * 10
  local previous = #hull

  for index = 1, #hull do
    addEdge(edges, slots, count, maxEdges, hull[previous], hull[index], EV_HULL, EV_UNDEF)
    previous = index
  end

  local current = 0

  while current < #edges // 4 do
    if edges[current * 4 + 3] == EV_UNDEF then
      faceCount = completeFacet(points, edges, slots, maxEdges, faceCount, current)
    end

    if edges[current * 4 + 4] == EV_UNDEF then
      faceCount = completeFacet(points, edges, slots, maxEdges, faceCount, current)
    end

    current += 1
  end

  local tris: { number } = table.create(faceCount * 4, -1)

  for offset = 1, #edges, 4 do
    local from, to, left, right = edges[offset], edges[offset + 1], edges[offset + 2], edges[offset + 3]

    if right >= 0 then
      local triangle = right * 4 + 1

      if tris[triangle] == -1 then
        tris[triangle], tris[triangle + 1] = from, to
      elseif tris[triangle] == to then
        tris[triangle + 2] = from
      elseif tris[triangle + 1] == from then
        tris[triangle + 2] = to
      end
    end

    if left >= 0 then
      local triangle = left * 4 + 1

      if tris[triangle] == -1 then
        tris[triangle], tris[triangle + 1] = to, from
      elseif tris[triangle] == from then
        tris[triangle + 2] = to
      elseif tris[triangle + 1] == to then
        tris[triangle + 2] = from
      end
    end
  end

  local triangle = 1

  while triangle <= #tris do
    if tris[triangle] ~= -1 and tris[triangle + 1] ~= -1 and tris[triangle + 2] ~= -1 then
      triangle += 4

      continue
    end

    for _ = 1, 4 do
      table.remove(tris, triangle)
    end
  end

  return tris
end

local function polyMinExtent(verts)
    local minDist=math.huge
    for i=1,#verts do
        local ni=(i%#verts)+1; local maxEdge=0
        for j=1,#verts do if j~=i and j~=ni then maxEdge=math.max(maxEdge,distancePtSeg2d(verts[j],verts[i],verts[ni])) end end
        minDist=math.min(minDist,maxEdge)
    end
    return math.sqrt(minDist)
end

local function triangulateHull(verts,hull,nin)
    if #hull<3 then return {} end
    local start,left,right=1,2,#hull; local dmin=math.huge
    for i=1,#hull do
        if hull[i]<nin then
            local pi=if i>1 then i-1 else #hull; local ni=if i<#hull then i+1 else 1
            local d=vdist2(verts[hull[pi]+1],verts[hull[i]+1])+vdist2(verts[hull[i]+1],verts[hull[ni]+1])+vdist2(verts[hull[ni]+1],verts[hull[pi]+1])
            if d<dmin then start,left,right,dmin=i,ni,pi,d end
        end
    end
    local tris={hull[start],hull[left],hull[right],0}
    local function nexti(i) return if i<#hull then i+1 else 1 end
    local function previ(i) return if i>1 then i-1 else #hull end
    while nexti(left)~=right do
        local nl,nr=nexti(left),previ(right)
        local dleft=vdist2(verts[hull[left]+1],verts[hull[nl]+1])+vdist2(verts[hull[nl]+1],verts[hull[right]+1])
        local dright=vdist2(verts[hull[right]+1],verts[hull[nr]+1])+vdist2(verts[hull[left]+1],verts[hull[nr]+1])
        if dleft<dright then tris[#tris+1]=hull[left];tris[#tris+1]=hull[nl];tris[#tris+1]=hull[right];tris[#tris+1]=0;left=nl
        else tris[#tris+1]=hull[left];tris[#tris+1]=hull[nr];tris[#tris+1]=hull[right];tris[#tris+1]=0;right=nr end
    end
    return tris
end

local function jitterX(i) return (bit32.band(i*0x8da6b343,0xffff)/65535*2)-1 end
local function jitterZ(i) return (bit32.band(i*0xd8163841,0xffff)/65535*2)-1 end
local function onHull(a,b,hull)
    if a>=#hull or b>=#hull then return false end
    local j=#hull
    for i=1,#hull do if a==hull[j] and b==hull[i] then return true end; j=i end
    return false
end
local function setTriFlags(tris,hull)
    for o=1,#tris,4 do
        local a,b,c=tris[o],tris[o+1],tris[o+2]; local flags=0
        if onHull(a,b,hull) then flags=bit32.bor(flags,DETAIL_EDGE_BOUNDARY) end
        if onHull(b,c,hull) then flags=bit32.bor(flags,bit32.lshift(DETAIL_EDGE_BOUNDARY,2)) end
        if onHull(c,a,hull) then flags=bit32.bor(flags,bit32.lshift(DETAIL_EDGE_BOUNDARY,4)) end
        tris[o+3]=flags
    end
end

local function buildPolyDetail(input,sampleDist,sampleMaxError,heightSearchRadius,chf,hp)
    local nin=#input; local verts=table.clone(input); local tris={}; local hull={}
    local cs,ics=chf.cs,1/chf.cs
    local minExtent=polyMinExtent(verts)
    if sampleDist>0 then
        local j=nin
        for i=1,nin do
            local vj,vi=input[j],input[i]; local swapped=false
            if math.abs(vj.X-vi.X)<1e-6 then if vj.Z>vi.Z then vj,vi=vi,vj; swapped=true end
            elseif vj.X>vi.X then vj,vi=vi,vj; swapped=true end
            local delta=vi-vj; local d=math.sqrt(delta.X*delta.X+delta.Z*delta.Z)
            local nn=1+math.floor(d/sampleDist); if nn>=MAX_VERTS_PER_EDGE then nn=MAX_VERTS_PER_EDGE-1 end
            if #verts+nn>=MAX_VERTS then nn=MAX_VERTS-1-#verts end
            local edge=table.create(nn+1)
            for k=0,nn do
                local u=k/nn; local pos=vj+delta*u
                local hh=getHeight(pos.X,pos.Y,pos.Z,ics,chf.ch,heightSearchRadius,hp)
                edge[k+1]=vector.create(pos.X,hh*chf.ch,pos.Z)
            end
            local idx={0,nn}; local k=1
            while k<#idx do
                local a,b=idx[k],idx[k+1]; local maxd,maxi=0,-1
                for m=a+1,b-1 do local dev=distancePtSeg(edge[m+1],edge[a+1],edge[b+1]); if dev>maxd then maxd,maxi=dev,m end end
                if maxi~=-1 and maxd>sampleMaxError*sampleMaxError then table.insert(idx,k+1,maxi) else k+=1 end
            end
            hull[#hull+1]=j-1
            if swapped then for q=#idx-1,2,-1 do verts[#verts+1]=edge[idx[q]+1]; hull[#hull+1]=#verts-1 end
            else for q=2,#idx-1 do verts[#verts+1]=edge[idx[q]+1]; hull[#hull+1]=#verts-1 end end
            j=i
        end
    else
        for i=0,nin-1 do hull[#hull+1]=i end
    end
    if minExtent<sampleDist*2 then tris=triangulateHull(verts,hull,nin); setTriFlags(tris,hull); return verts,tris end
    tris=triangulateHull(verts,hull,nin)
    if #tris==0 then return verts,tris end
    if sampleDist>0 then
        local bmin,bmax=input[1],input[1]
        for i=2,nin do local p=input[i]; bmin=vector.create(math.min(bmin.X,p.X),math.min(bmin.Y,p.Y),math.min(bmin.Z,p.Z)); bmax=vector.create(math.max(bmax.X,p.X),math.max(bmax.Y,p.Y),math.max(bmax.Z,p.Z)) end
        local x0,x1=math.floor(bmin.X/sampleDist),math.ceil(bmax.X/sampleDist)
        local z0,z1=math.floor(bmin.Z/sampleDist),math.ceil(bmax.Z/sampleDist)
        -- A sample's position never changes, so it is jittered once here, not on every pass
        -- over the samples.
        local positions: { vector }, added: { boolean } = {}, {}

        for row = z0, z1 - 1 do
          for column = x0, x1 - 1 do
            local probe = vector.create(column * sampleDist, (bmax.Y + bmin.Y) * 0.5, row * sampleDist)

            -- Written as the test a sample has to pass, so one whose distance is not a number
            -- is left out as upstream leaves it out.
            if not (distToPoly(input, probe) <= -sampleDist / 2) then continue end

            local height = getHeight(probe.X, probe.Y, probe.Z, ics, chf.ch, heightSearchRadius, hp)
            local sample = #positions

            table.insert(added, false)
            table.insert(positions, vector.create(
              column * sampleDist + jitterX(sample) * cs * 0.1,
              height * chf.ch,
              row * sampleDist + jitterZ(sample) * cs * 0.1
            ))
          end
        end

        for _ = 1, #positions do
          if #verts >= MAX_VERTS then break end

          local origins, terms = trianglePlanes(verts, tris)
          local farthest, chosen = 0, -1

          for sample, position in positions do
            if added[sample] then continue end

            local distance = distanceToMesh(position, origins, terms)

            if distance >= 0 and distance > farthest then
              farthest, chosen = distance, sample
            end
          end

          if farthest <= sampleMaxError or chosen == -1 then break end

          added[chosen] = true
          table.insert(verts, positions[chosen])
          tris = delaunayHull(verts, hull)
        end
    end
    if #tris//4>MAX_TRIS then while #tris>MAX_TRIS*4 do table.remove(tris) end end
    setTriFlags(tris,hull)
    return verts,tris
end

local CENTER_OFFSETS={{0,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0}}
local function dirForOffset(dx,dz)
    for dir0=0,3 do if DIR_X[dir0+1]==dx and DIR_Z[dir0+1]==dz then return dir0 end end
    return 0
end

local function seedArrayWithPolyCenter(chf,neighbors,polyIndices,npoly,meshVerts,borderSize,hp,queue)
    local startX,startZ,startSpan,dmin=0,0,-1,RC_UNSET_HEIGHT
    for j=0,npoly-1 do
        if dmin<=0 then break end
        local vi=polyIndices[j+1]; local vo=vi*6
        local vx,vy,vz=buffer.readu16(meshVerts,vo),buffer.readu16(meshVerts,vo+2),buffer.readu16(meshVerts,vo+4)
        for _,off in CENTER_OFFSETS do
            local ax,az=vx+off[1],vz+off[2]
            if ax<hp.xmin or ax>=hp.xmin+hp.width or az<hp.ymin or az>=hp.ymin+hp.height then continue end
            local cell=(ax+borderSize)+(az+borderSize)*chf.width
            local first=buffer.readu32(chf.cellIndex,cell*4); local count=buffer.readu8(chf.cellCount,cell)
            for li=0,count-1 do
                local span=first+li; local d=math.abs(vy-buffer.readu16(chf.y,span*2))
                if d<dmin then startX,startZ,startSpan,dmin=ax,az,span,d end
            end
        end
    end
    if startSpan<0 then return false end
    local pcx,pcz=0,0
    for j=1,npoly do local o=polyIndices[j]*6; pcx+=buffer.readu16(meshVerts,o); pcz+=buffer.readu16(meshVerts,o+4) end
    pcx=pcx//npoly; pcz=pcz//npoly
    table.clear(queue); queue[1]={startX,startZ,startSpan}
    buffer.fill(hp.data,0,0,buffer.len(hp.data))
    local dirs={0,1,2,3}; local cx,cz,ci=-1,-1,-1
    while #queue>0 do
        local e=table.remove(queue); cx,cz,ci=e[1],e[2],e[3]
        if cx==pcx and cz==pcz then break end
        local direct=if cx==pcx then dirForOffset(0,if pcz>cz then 1 else -1) else dirForOffset(if pcx>cx then 1 else -1,0)
        local slot=direct+1; dirs[slot],dirs[4]=dirs[4],dirs[slot]
        for di=1,4 do
            local dir0=dirs[di]; local ni=neighbors[ci * 4 + dir0 + 1]; if ni<0 then continue end
            local nx,nz=cx+DIR_X[dir0+1],cz+DIR_Z[dir0+1]; local hx,hz=nx-hp.xmin,nz-hp.ymin
            if hx<0 or hz<0 or hx>=hp.width or hz>=hp.height or patchRead(hp,hx,hz)~=0 then continue end
            patchWrite(hp,hx,hz,1)
            queue[#queue+1]={nx,nz,ni}
        end
        dirs[slot],dirs[4]=dirs[4],dirs[slot]
    end
    table.clear(queue); queue[1]={cx+borderSize,cz+borderSize,ci}
    buffer.fill(hp.data,0,0xff,buffer.len(hp.data))
    patchWrite(hp,cx-hp.xmin,cz-hp.ymin,buffer.readu16(chf.y,ci*2))
    return true
end

local function getHeightData(chf,neighbors,polyIndices,npoly,mesh,borderSize,hp,region)
    local queue={}; buffer.fill(hp.data,0,0xff,buffer.len(hp.data)); local empty=true
    local regions=chf.regions
    if region~=RC_MULTIPLE_REGS and regions~=nil then
        for hz=0,hp.height-1 do
            local z=hp.ymin+hz+borderSize
            for hx=0,hp.width-1 do
                local x=hp.xmin+hx+borderSize
                local cell=x+z*chf.width; local first=buffer.readu32(chf.cellIndex,cell*4); local count=buffer.readu8(chf.cellCount,cell)
                for li=0,count-1 do
                    local span=first+li
                    if buffer.readu16(regions,span*2)==region then
                        patchWrite(hp,hx,hz,buffer.readu16(chf.y,span*2)); empty=false
                        local border=false
                        for dir0=0,3 do
                            local ni=neighbors[span * 4 + dir0 + 1]
                            if ni>=0 and buffer.readu16(regions,ni*2)~=region then border=true; break end
                        end
                        if border then queue[#queue+1]={x,z,span} end
                        break
                    end
                end
            end
        end
    end
    if empty then seedArrayWithPolyCenter(chf,neighbors,polyIndices,npoly,mesh.verts,borderSize,hp,queue) end
    local head=1
    while head<=#queue do
        local e=queue[head]; head+=1; local cx,cz,ci=e[1],e[2],e[3]
        for dir0=0,3 do
            local ni=neighbors[ci * 4 + dir0 + 1]; if ni<0 then continue end
            local nx,nz=cx+DIR_X[dir0+1],cz+DIR_Z[dir0+1]
            local hx,hz=nx-hp.xmin-borderSize,nz-hp.ymin-borderSize
            if hx<0 or hz<0 or hx>=hp.width or hz>=hp.height or patchRead(hp,hx,hz)~=RC_UNSET_HEIGHT then continue end
            patchWrite(hp,hx,hz,buffer.readu16(chf.y,ni*2)); queue[#queue+1]={nx,nz,ni}
        end
    end
end

function Detail.buildPolyMeshDetail(mesh,chf,sampleDist,sampleMaxError)
    sampleDist=sampleDist or 0
    sampleMaxError=sampleMaxError or 0
    if mesh.nverts==0 or mesh.npolys==0 then return {meshes=buffer.create(0),verts=buffer.create(0),tris=buffer.create(0),nmeshes=0,nverts=0,ntris=0} end
    local nvp,cs,ch,borderSize=mesh.nvp,mesh.cs,mesh.ch,mesh.borderSize
    local heightSearchRadius=math.max(1,math.ceil(mesh.maxEdgeError or 0))
    local bounds=table.create(mesh.npolys); local maxhw,maxhh=0,0
    for p0=0,mesh.npolys-1 do
        local xmin,xmax,zmin,zmax=chf.width,0,chf.height,0
        local base=p0*nvp*2
        for j0=0,nvp-1 do
            local vi=buffer.readu16(mesh.polys,(base+j0)*2); if vi==RC_MESH_NULL_IDX then break end
            local vo=vi*6; local x,z=buffer.readu16(mesh.verts,vo),buffer.readu16(mesh.verts,vo+4)
            xmin=math.min(xmin,x);xmax=math.max(xmax,x);zmin=math.min(zmin,z);zmax=math.max(zmax,z)
        end
        xmin=math.max(0,xmin-1);xmax=math.min(chf.width,xmax+1);zmin=math.max(0,zmin-1);zmax=math.min(chf.height,zmax+1)
        bounds[p0+1]={xmin,xmax,zmin,zmax}; if xmin<xmax and zmin<zmax then maxhw=math.max(maxhw,xmax-xmin);maxhh=math.max(maxhh,zmax-zmin) end
    end
    local neighbors = buildDetailNeighborIndex(chf)
    local meshRecords,allVerts,allTris={}, {}, {}
    for p0=0,mesh.npolys-1 do
        local polyIndices,input={},{}; local base=p0*nvp*2
        for j0=0,nvp-1 do
            local vi=buffer.readu16(mesh.polys,(base+j0)*2); if vi==RC_MESH_NULL_IDX then break end
            polyIndices[#polyIndices+1]=vi; local vo=vi*6
            input[#input+1]=vector.create(buffer.readu16(mesh.verts,vo)*cs,buffer.readu16(mesh.verts,vo+2)*ch,buffer.readu16(mesh.verts,vo+4)*cs)
        end
        local b=bounds[p0+1]; local hp={xmin=b[1],ymin=b[3],width=b[2]-b[1],height=b[4]-b[3]}
        hp.data=buffer.create(math.max(1,hp.width*hp.height)*2)
        getHeightData(chf,neighbors,polyIndices,#polyIndices,mesh,borderSize,hp,buffer.readu16(mesh.regs,p0*2))
        local verts,tris=buildPolyDetail(input,sampleDist,sampleMaxError,heightSearchRadius,chf,hp)
        local vertBase=#allVerts; local triBase=#allTris//4
        for _,v in verts do allVerts[#allVerts+1]=vector.create(v.X+mesh.bmin.X,v.Y+mesh.bmin.Y+chf.ch,v.Z+mesh.bmin.Z) end
        for o=1,#tris,4 do allTris[#allTris+1]=tris[o];allTris[#allTris+1]=tris[o+1];allTris[#allTris+1]=tris[o+2];allTris[#allTris+1]=tris[o+3] end
        meshRecords[#meshRecords+1]={vertBase,#verts,triBase,#tris//4}
    end
    local meshes=buffer.create(#meshRecords*16); local verts=buffer.create(#allVerts*12); local tris=buffer.create(#allTris)
    for i0=0,#meshRecords-1 do local r=meshRecords[i0+1]; local o=i0*16; for k=1,4 do buffer.writeu32(meshes,o+(k-1)*4,r[k]) end end
    for i0=0,#allVerts-1 do writeVec3f(verts,i0+1,allVerts[i0+1]) end
    for i=1,#allTris do buffer.writeu8(tris,i-1,allTris[i]) end
    return {meshes=meshes,verts=verts,tris=tris,nmeshes=#meshRecords,nverts=#allVerts,ntris=#allTris//4}
end

function Detail.mergePolyMeshDetails(detailMeshes)
    local nmeshes,nverts,ntris=0,0,0
    for _,dm in detailMeshes do if dm~=nil then nmeshes+=dm.nmeshes;nverts+=dm.nverts;ntris+=dm.ntris end end
    local meshes,verts,tris=buffer.create(nmeshes*16),buffer.create(nverts*12),buffer.create(ntris*4)
    local mo,vo,to=0,0,0
    for _,dm in detailMeshes do
        if dm==nil then continue end
        for m0=0,dm.nmeshes-1 do
            local so=m0*16; local doff=mo*16
            buffer.writeu32(meshes,doff,vo+buffer.readu32(dm.meshes,so))
            buffer.writeu32(meshes,doff+4,buffer.readu32(dm.meshes,so+4))
            buffer.writeu32(meshes,doff+8,to+buffer.readu32(dm.meshes,so+8))
            buffer.writeu32(meshes,doff+12,buffer.readu32(dm.meshes,so+12)); mo+=1
        end
        if dm.nverts>0 then buffer.copy(verts,vo*12,dm.verts,0,dm.nverts*12) end
        if dm.ntris>0 then buffer.copy(tris,to*4,dm.tris,0,dm.ntris*4) end
        vo+=dm.nverts;to+=dm.ntris
    end
    return {meshes=meshes,verts=verts,tris=tris,nmeshes=nmeshes,nverts=nverts,ntris=ntris}
end

return table.freeze(Detail)
