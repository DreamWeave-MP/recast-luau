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

local function distPtTri(p,a,b,c)
    local v0,v1,v2=c-a,b-a,p-a
    local dot00,dot01,dot02=vdot2(v0,v0),vdot2(v0,v1),vdot2(v0,v2)
    local dot11,dot12=vdot2(v1,v1),vdot2(v1,v2)
    local denom=dot00*dot11-dot01*dot01
    if math.abs(denom)<1e-12 then return math.huge end
    local inv=1/denom
    local u=(dot11*dot02-dot01*dot12)*inv
    local v=(dot00*dot12-dot01*dot02)*inv
    if u>=-1e-4 and v>=-1e-4 and u+v<=1.0001 then return math.abs((a.Y+v0.Y*u+v1.Y*v)-p.Y) end
    return math.huge
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

local function distToTriMesh(p,verts,tris)
    local best=math.huge
    for o=1,#tris,4 do
        local d=distPtTri(p,verts[tris[o]+1],verts[tris[o+1]+1],verts[tris[o+2]+1])
        if d<best then best=d end
    end
    return if best==math.huge then -1 else best
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

local function edgeAt(edges,e)
    local o=e*4+1; return edges[o],edges[o+1],edges[o+2],edges[o+3]
end
local function setEdge(edges,e,s,t,l,r)
    local o=e*4+1; edges[o],edges[o+1],edges[o+2],edges[o+3]=s,t,l,r
end
local function findEdge(edges,s,t)
    for e=0,#edges//4-1 do local a,b=edgeAt(edges,e); if (a==s and b==t) or (a==t and b==s) then return e end end
    return EV_UNDEF
end
local function addEdge(edges,maxEdges,s,t,l,r)
    if #edges//4>=maxEdges then return EV_UNDEF end
    if findEdge(edges,s,t)~=EV_UNDEF then return EV_UNDEF end
    local e=#edges//4; setEdge(edges,e,s,t,l,r); return e
end
local function updateLeftFace(edges,e,s,t,f)
    local a,b,l,r=edgeAt(edges,e); local o=e*4+1
    if a==s and b==t and l==EV_UNDEF then edges[o+2]=f elseif b==s and a==t and r==EV_UNDEF then edges[o+3]=f end
end
local function overlapSegSeg2d(a,b,c,d)
    local a1,a2=vcross2(a,b,d),vcross2(a,b,c)
    if a1*a2<0 then local a3=vcross2(c,d,a); local a4=a3+a2-a1; if a3*a4<0 then return true end end
    return false
end
local function overlapEdges(pts,edges,s1,t1)
    for e=0,#edges//4-1 do
        local s0,t0=edgeAt(edges,e)
        if s0~=s1 and s0~=t1 and t0~=s1 and t0~=t1 and overlapSegSeg2d(pts[s0+1],pts[t0+1],pts[s1+1],pts[t1+1]) then return true end
    end
    return false
end

local function completeFacet(pts,edges,maxEdges,nfaces,e)
    local a,b,l,r=edgeAt(edges,e); local s,t
    if l==EV_UNDEF then s,t=a,b elseif r==EV_UNDEF then s,t=b,a else return nfaces end
    local pt=#pts; local center=vector.zero; local radius=-1
    for u=0,#pts-1 do
        if u==s or u==t then continue end
        if vcross2(pts[s+1],pts[t+1],pts[u+1])>1e-5 then
            if radius<0 then
                pt=u; center,radius=circumCircle(pts[s+1],pts[t+1],pts[u+1]); continue
            end
            local d=vdist2(center,pts[u+1]); local tol=.001
            if d>radius*(1+tol) then continue
            elseif d<radius*(1-tol) then pt=u; center,radius=circumCircle(pts[s+1],pts[t+1],pts[u+1])
            else
                if overlapEdges(pts,edges,s,u) or overlapEdges(pts,edges,t,u) then continue end
                pt=u; center,radius=circumCircle(pts[s+1],pts[t+1],pts[u+1])
            end
        end
    end
    if pt<#pts then
        updateLeftFace(edges,e,s,t,nfaces)
        local e1=findEdge(edges,pt,s); if e1==EV_UNDEF then addEdge(edges,maxEdges,pt,s,nfaces,EV_UNDEF) else updateLeftFace(edges,e1,pt,s,nfaces) end
        local e2=findEdge(edges,t,pt); if e2==EV_UNDEF then addEdge(edges,maxEdges,t,pt,nfaces,EV_UNDEF) else updateLeftFace(edges,e2,t,pt,nfaces) end
        return nfaces+1
    end
    updateLeftFace(edges,e,s,t,EV_HULL)
    return nfaces
end

local function delaunayHull(pts,hull)
    local edges={}; local nfaces=0; local maxEdges=#pts*10
    local j=#hull
    for i=1,#hull do addEdge(edges,maxEdges,hull[j],hull[i],EV_HULL,EV_UNDEF); j=i end
    local current=0
    while current<#edges//4 do
        local _,_,l,r=edgeAt(edges,current)
        if l==EV_UNDEF then nfaces=completeFacet(pts,edges,maxEdges,nfaces,current) end
        _,_,l,r=edgeAt(edges,current)
        if r==EV_UNDEF then nfaces=completeFacet(pts,edges,maxEdges,nfaces,current) end
        current+=1
    end
    local tris=table.create(nfaces*4)
    for i=1,nfaces*4 do tris[i]=-1 end
    for e=0,#edges//4-1 do
        local s,t,l,r=edgeAt(edges,e)
        if r>=0 then
            local o=r*4+1
            if tris[o]==-1 then tris[o],tris[o+1]=s,t elseif tris[o]==t then tris[o+2]=s elseif tris[o+1]==s then tris[o+2]=t end
        end
        if l>=0 then
            local o=l*4+1
            if tris[o]==-1 then tris[o],tris[o+1]=t,s elseif tris[o]==s then tris[o+2]=t elseif tris[o+1]==t then tris[o+2]=s end
        end
    end
    local o=1
    while o<=#tris do
        if tris[o]==-1 or tris[o+1]==-1 or tris[o+2]==-1 then for k=1,4 do table.remove(tris,o) end else o+=4 end
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
        local samples={}
        for z=z0,z1-1 do for x=x0,x1-1 do
            local p=vector.create(x*sampleDist,(bmax.Y+bmin.Y)*.5,z*sampleDist)
            if distToPoly(input,p)<=-sampleDist/2 then samples[#samples+1]={x=x,h=getHeight(p.X,p.Y,p.Z,ics,chf.ch,heightSearchRadius,hp),z=z,added=false} end
        end end
        for iter=1,#samples do
            if #verts>=MAX_VERTS then break end
            local bestd,besti,bestp=0,-1,nil
            for si=1,#samples do
                local s=samples[si]; if s.added then continue end
                local p=vector.create(s.x*sampleDist+jitterX(si-1)*cs*.1,s.h*chf.ch,s.z*sampleDist+jitterZ(si-1)*cs*.1)
                local d=distToTriMesh(p,verts,tris)
                if d>=0 and d>bestd then bestd,besti,bestp=d,si,p end
            end
            if bestd<=sampleMaxError or besti==-1 then break end
            samples[besti].added=true; verts[#verts+1]=bestp
            tris=delaunayHull(verts,hull)
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
