--!native
local oa=require('@recast/obstacle-avoidance')
local function run(name,kind)
    local q=oa.newQuery(6,8)
    local pos=vector.create(0,0,0); local vel=vector.create(0.2,0,0.1); local dvel=vector.create(1.5,0,0.25)
    if kind==0 then
        oa.addCircle(q,vector.create(1,0,0),0.35,vector.zero,vector.zero)
    elseif kind==1 then
        oa.addCircle(q,vector.create(0.2,0,0.1),0.45,vector.create(-0.2,0,0),vector.create(0.1,0,-0.1))
        oa.addSegment(q,vector.create(-0.5,0,0.8),vector.create(1.2,0,0.8))
    else
        oa.addCircle(q,vector.create(1,0,0.2),0.3,vector.create(-0.1,0,0.05),vector.create(-0.2,0,0.1))
        oa.addCircle(q,vector.create(0.3,0,-0.8),0.25,vector.create(0.05,0,0.15),vector.create(0.1,0,0.2))
        oa.addSegment(q,vector.create(-0.8,0,0.6),vector.create(0.8,0,0.6))
        oa.addSegment(q,vector.create(-0.7,0,-0.6),vector.create(-0.7,0,0.6))
    end
    local p=oa.defaultParams(); local dbg=oa.newDebugData(4096)
    local out,ng=oa.sampleVelocityGrid(q,pos,0.35,2,vel,dvel,p,dbg)
    print(name,'grid','ns',ng,'out',out.X,out.Y,out.Z,'samples',dbg.count)
    oa.normalizeDebugSamples(dbg)
    for i=1,math.min(3,dbg.count) do print(name,'gdbg',i-1,'vel',dbg.vel[i].X,dbg.vel[i].Z,'size',dbg.ssize[i],'pen',dbg.pen[i],'v',dbg.vpen[i],'vc',dbg.vcpen[i],'side',dbg.spen[i],'toi',dbg.tpen[i]) end
    out,ng=oa.sampleVelocityAdaptive(q,pos,0.35,2,vel,dvel,p,dbg)
    print(name,'adaptive','ns',ng,'out',out.X,out.Y,out.Z,'samples',dbg.count)
    oa.normalizeDebugSamples(dbg)
    for i=1,math.min(3,dbg.count) do print(name,'adbg',i-1,'vel',dbg.vel[i].X,dbg.vel[i].Z,'size',dbg.ssize[i],'pen',dbg.pen[i],'v',dbg.vpen[i],'vc',dbg.vcpen[i],'side',dbg.spen[i],'toi',dbg.tpen[i]) end
end
run('headon',0); run('overlapwall',1); run('mixed',2)
