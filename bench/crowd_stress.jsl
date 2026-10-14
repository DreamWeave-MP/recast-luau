--!native
local dt = require('@recast/detour')
local crowdlib = require('@recast/crowd')
local argv = {...}
local count = tonumber(argv[1]) or 256
local side = math.ceil(math.sqrt(count))
local spacing = 0.75
local extent = side * spacing + 4
local verts = {
    vector.create(0,0,0), vector.create(0,0,extent),
    vector.create(extent,0,0), vector.create(extent,0,extent),
}
local nav = dt.buildTriangleNavMesh(verts, {{1,2,3},{3,2,4}})
local crowd = crowdlib.new(nav, count, {separationWeight=2, separationRange=3, constrainToNavmesh=false, maxAgentRadius=0.25})
for i=0,count-1 do
    local x=(i%side)*spacing+2
    local z=math.floor(i/side)*spacing+2
    local id = assert(crowdlib.addAgent(crowd, vector.create(x,0,z), {
        radius=0.2,height=1.8,maxSpeed=2,maxAcceleration=8,collisionQueryRange=1.2,
        updateFlags=crowdlib.DT_CROWD_SEPARATION,
    }))
    assert(crowdlib.requestMoveVelocity(crowd,id,vector.create(1.5,0,0.15*((i%3)-1))))
end
for _=1,10 do crowdlib.update(crowd,1/60) end
local t0=os.clock()
for _=1,60 do crowdlib.update(crowd,1/60) end
local t1=os.clock()
local checksum=0
for i=1,count do local a=crowdlib.getAgent(crowd,i); checksum+=a.position.X+a.position.Z end
print(`agents={count} updates=60 elapsed_ms={(t1-t0)*1000} per_update_ms={(t1-t0)*1000/60} checksum={checksum}`)
