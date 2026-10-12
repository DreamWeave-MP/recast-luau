--!native
local rc=require('@recast/core')
local function go(n)
 local hf=rc.createHeightfield(1,1,vector.zero,vector.create(1,20,1),1,1)
 rc.addSpan(hf,0,0,0,1,rc.RC_WALKABLE_AREA,1)
 if n==2 then rc.addSpan(hf,0,0,10,11,rc.RC_WALKABLE_AREA,1) end
 local chf=rc.buildCompactHeightfield(2,1,hf)
 local r,c=rc.buildLayerRegions(chf,0,0)
 print('n',n,'count',c,'spanCount',chf.spanCount)
 for i=0,chf.spanCount-1 do print(i,buffer.readu16(r,i*2)) end
end
go(1); go(2)
