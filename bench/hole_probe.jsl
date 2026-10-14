--!native
local rc=require('@recast/core')
local hf=rc.createHeightfield(7,7,vector.zero,vector.create(7,10,7),1,1)
for z=0,6 do for x=0,6 do if not (x>=2 and x<=4 and z>=2 and z<=4) then rc.addSpan(hf,x,z,0,1,rc.RC_WALKABLE_AREA,1) end end end
local chf=rc.buildCompactHeightfield(2,1,hf)
rc.buildConnectedRegions(chf)
local c=rc.buildContours(chf,0,0)
print('contours',c.nconts)
for i,co in c.conts do print('cont',i,'n',co.nverts,'area2?',co.reg) end
local m=rc.buildPolyMesh(c,6)
print('mesh',m.nverts,m.npolys)
for p=0,m.npolys-1 do local vals={}; for j=0,m.nvp-1 do vals[#vals+1]=buffer.readu16(m.polys,(p*m.nvp*2+j)*2) end; print('p',p,table.concat(vals,',')) end
