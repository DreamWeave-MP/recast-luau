local rc=require('@recast/core')
local hf=rc.createHeightfield(7,7,vector.zero,vector.create(7,10,7),1,1)
for z=0,6 do for x=0,6 do rc.addSpan(hf,x,z,0,1,rc.RC_WALKABLE_AREA,1) end end
local chf=rc.buildCompactHeightfield(2,4,hf)
rc.buildConnectedRegions(chf)
local c=rc.buildContours(chf,1.3,0)
local m=rc.buildPolyMesh(c,6)
for z=2,4 do for x=2,4 do local s=buffer.readu32(chf.cellIndex,(x+z*chf.width)*4); buffer.writeu16(chf.y,s*2,4) end end
local d=rc.buildPolyMeshDetail(m,chf,1,0.1)
print('mesh',m.nverts,m.npolys,'detail',d.nverts,d.ntris)
