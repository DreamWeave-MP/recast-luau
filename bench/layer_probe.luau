local rc=require('@recast/core')
local hf=rc.createHeightfield(3,3,vector.zero,vector.create(3,20,3),1,1)
for z=0,2 do for x=0,2 do rc.addSpan(hf,x,z,0,1,rc.RC_WALKABLE_AREA,1); rc.addSpan(hf,x,z,10,11,rc.RC_WALKABLE_AREA,1) end end
local chf=rc.buildCompactHeightfield(2,1,hf)
print('spans',chf.spanCount)
local set=rc.buildHeightfieldLayers(chf,0,2)
print('layers',set.nlayers)
for i,l in set.layers do print(i,l.hmin,l.hmax,l.minx,l.maxx,l.miny,l.maxy,buffer.readu8(l.heights,0),buffer.readu8(l.areas,0)) end
