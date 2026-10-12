--!native
local rc=require('@recast/core')
local hf=rc.createHeightfield(5,5,vector.zero,vector.create(5,10,5),1,1)
for z=0,4 do for x=0,4 do rc.addSpan(hf,x,z,0,1,rc.RC_WALKABLE_AREA,1) end end
local chf=rc.buildCompactHeightfield(2,1,hf)
rc.buildConnectedRegions(chf)
rc.buildDistanceField(chf)
print('maxdist', chf.maxDistance, buffer.readu16(chf.dist,(2+2*5)*2))
local c=rc.buildContours(chf,1.3,0)
print('contours',c.nconts)
for i,co in c.conts do print('cont',i,'n',co.nverts,'reg',co.reg,'area',co.area); for j=0,co.nverts-1 do local o=j*4; print(co.verts[o+1],co.verts[o+2],co.verts[o+3],co.verts[o+4]) end end
local m=rc.buildPolyMesh(c,6)
print('mesh',m.nverts,m.npolys,m.nvp)
for i=0,m.nverts-1 do local o=i*6; print('v',i,buffer.readu16(m.verts,o),buffer.readu16(m.verts,o+2),buffer.readu16(m.verts,o+4)) end
for p=0,m.npolys-1 do local vals={}; for j=0,m.nvp*2-1 do vals[#vals+1]=buffer.readu16(m.polys,(p*m.nvp*2+j)*2) end; print('p',p,table.concat(vals,',')) end
