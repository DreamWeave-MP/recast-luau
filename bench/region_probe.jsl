local Recast=require('@recast/core')
local function grid(n)
 local vs={}; for z=0,n do for x=0,n do vs[#vs+1]=vector.create(x,0,z) end end
 local ts={}; local stride=n+1; for z=0,n-1 do for x=0,n-1 do local a=z*stride+x+1; local b=a+1; local c=a+stride; local d=c+1; ts[#ts+1]=a;ts[#ts+1]=c;ts[#ts+1]=b;ts[#ts+1]=b;ts[#ts+1]=c;ts[#ts+1]=d end end
 return vs,ts
end
local v,t=grid(9)
local b=Recast.buildCompactFromTriangles(v,t,{cs=1,ch=.2,agentRadius=0,walkableHeight=1,walkableClimb=0})
Recast.buildDistanceField(b.compact)
local r,n,o=Recast.buildRegions(b.compact,0,0,0)
print('watershed',n,#o,b.compact.maxDistance)
local c=Recast.buildContours(b.compact,1.3,0,0)
print('contours',#c.conts)
local b2=Recast.buildCompactFromTriangles(v,t,{cs=1,ch=.2,agentRadius=0,walkableHeight=1,walkableClimb=0})
local r2,n2=Recast.buildRegionsMonotone(b2.compact,0,0,0)
print('monotone',n2)
local c2=Recast.buildContours(b2.compact,1.3,0,0)
print('contours2',#c2.conts)
