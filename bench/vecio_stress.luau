--!native
local Common=require('@recast/common')
local n=tonumber(...) or 262144
local reps=8
local b=buffer.create(n*12)
for i0=0,n-1 do local o=i0*12; buffer.writef32(b,o,i0%997); buffer.writef32(b,o+4,(i0*3)%991); buffer.writef32(b,o+8,(i0*7)%983) end
local function localVertexAt(buf,i0) local o=i0*12; return vector.create(buffer.readf32(buf,o),buffer.readf32(buf,o+4),buffer.readf32(buf,o+8)) end
local cachedVertexAt=Common.vertexAt
local sum=vector.zero
local t0=os.clock()
for _=1,reps do for i0=0,n-1 do sum += Common.vertexAt(b,i0) end end
local t1=os.clock()
local sumCached=vector.zero
for _=1,reps do for i0=0,n-1 do sumCached += cachedVertexAt(b,i0) end end
local tCached=os.clock()
local sumLocal=vector.zero
for _=1,reps do for i0=0,n-1 do sumLocal += localVertexAt(b,i0) end end
local tLocal=os.clock()
local sum2=vector.zero
for _=1,reps do for i0=0,n-1 do local o=i0*12; sum2 += vector.create(buffer.readf32(b,o),buffer.readf32(b,o+4),buffer.readf32(b,o+8)) end end
local t2=os.clock()
local sx,sy,sz=0,0,0
for _=1,reps do for i0=0,n-1 do local o=i0*12; sx+=buffer.readf32(b,o); sy+=buffer.readf32(b,o+4); sz+=buffer.readf32(b,o+8) end end
local t3=os.clock()
print(`n={n} reads={n*reps} module_helper_ms={(t1-t0)*1000} cached_external_ms={(tCached-t1)*1000} local_helper_ms={(tLocal-tCached)*1000} inline_vec_ms={(t2-tLocal)*1000} scalar_ms={(t3-t2)*1000} sums={sum.X}/{sumCached.X}/{sumLocal.X}/{sum2.X}/{sx}`)
