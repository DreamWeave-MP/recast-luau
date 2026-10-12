--!native

local jt = require('@jess/test')
local suite = jt.new()
local dt = require('@recast/detour')

local function expect(condition, message)
    if not condition then error(message or 'expectation failed', 2) end
end

local function tileAt(x0)
    return dt.buildTriangleNavMesh({
        vector.create(x0, 0, 0), vector.create(x0, 0, 1),
        vector.create(x0 + 1, 0, 0), vector.create(x0 + 1, 0, 1),
    }, {{1,2,3},{3,2,4}}, {walkableSlopeAngle = 45})
end

local function tiledPair()
    local nav = dt.newTiledNavMesh({
        orig = vector.zero, tileWidth = 1, tileHeight = 1,
        maxTiles = 4, maxPolys = 8,
    })
    local leftRef, leftStatus = dt.addTile(nav, tileAt(0), 0, 0, 0)
    local rightRef, rightStatus = dt.addTile(nav, tileAt(1), 1, 0, 0)
    expect(dt.statusSucceed(leftStatus) and dt.statusSucceed(rightStatus))
    return nav, leftRef, rightRef
end

suite.it('packs tiled refs with Detour salt/tile/poly bits', function()
    local nav, leftRef, rightRef = tiledPair()
    jt.expect(leftRef).toBe(32)
    jt.expect(rightRef).toBe(40)
    local salt, tile, poly = dt.decodePolyId(nav, rightRef + 1)
    jt.expect(salt).toBe(1)
    jt.expect(tile).toBe(1)
    jt.expect(poly).toBe(1)
    jt.expect(dt.getTileRefAt(nav, 0, 0, 0)).toBe(leftRef)
    jt.expect(dt.getTileRefAt(nav, 1, 0, 0)).toBe(rightRef)
end)

suite.it('matches Detour tile hash-chain layer and freelist ordering', function()
    local nav=dt.newTiledNavMesh({orig=vector.zero,tileWidth=1,tileHeight=1,maxTiles=8,maxPolys=8})
    local refs={}
    for layer=0,2 do
        local ref,status=dt.addTile(nav,tileAt(0),0,0,layer)
        expect(dt.statusSucceed(status)); refs[#refs+1]=ref
    end
    local layers=dt.getTilesAt(nav,0,0,2)
    jt.expect(#layers).toBe(2); jt.expect(layers[1].layer).toBe(2); jt.expect(layers[2].layer).toBe(1)

    local nav2=dt.newTiledNavMesh({orig=vector.zero,tileWidth=1,tileHeight=1,maxTiles=6,maxPolys=8})
    dt.addTile(nav2,tileAt(0),0,0,0); dt.addTile(nav2,tileAt(1),1,0,0)
    local slot4=dt.encodePolyId(nav2,1,4,0)
    local r4,s4=dt.addTile(nav2,tileAt(4),4,0,0,slot4); expect(dt.statusSucceed(s4)); jt.expect(dt.decodePolyIdTile(nav2,r4)).toBe(4)
    local r2=assert(dt.addTile(nav2,tileAt(2),2,0,0)); jt.expect(dt.decodePolyIdTile(nav2,r2)).toBe(2)
    local r3=assert(dt.addTile(nav2,tileAt(3),3,0,0)); jt.expect(dt.decodePolyIdTile(nav2,r3)).toBe(3)
    local r5=assert(dt.addTile(nav2,tileAt(5),5,0,0)); jt.expect(dt.decodePolyIdTile(nav2,r5)).toBe(5)
end)

suite.it('queries and pathfinds across an external tile seam with encoded refs', function()
    local nav, leftRef, rightRef = tiledPair()
    jt.expect(nav.polyCount).toBe(4)
    jt.expect(nav.extraLinkCount).toBe(2)

    local startRef = dt.findNearestPoly(nav, vector.create(0.1, 0, 0.5), vector.create(0.5, 1, 0.5))
    local endRef = dt.findNearestPoly(nav, vector.create(1.9, 0, 0.5), vector.create(0.5, 1, 0.5))
    expect(startRef ~= nil and endRef ~= nil)
    expect(startRef >= leftRef and startRef < leftRef + 8)
    expect(endRef >= rightRef and endRef < rightRef + 8)

    local path = dt.findPath(nav, startRef, endRef)
    expect(path ~= nil and #path == 4, 'expected four-poly corridor across two tiles')
    jt.expect(path[1]).toBe(startRef)
    jt.expect(path[#path]).toBe(endRef)

    local result, err = dt.query(nav, vector.create(0.1,0,0.5), vector.create(1.9,0,0.5), vector.create(0.5,1,0.5))
    expect(result ~= nil, err)
    jt.expect(#result.corridor).toBe(4)
    expect(#result.points >= 2)
end)

suite.it('invalidates removed tile refs by salt and reconnects after re-add', function()
    local nav, _, rightRef = tiledPair()
    local oldPolyRef = rightRef + 1
    expect(dt.isValidPolyRef(nav, oldPolyRef))

    local removed, status = dt.removeTile(nav, rightRef)
    expect(removed ~= nil and dt.statusSucceed(status))
    expect(not dt.isValidPolyRef(nav, oldPolyRef))
    jt.expect(dt.getTileByRef(nav, rightRef)).toBe(nil)

    local newRef, addStatus = dt.addTile(nav, removed, 1, 0, 0)
    expect(dt.statusSucceed(addStatus))
    jt.expect(newRef).toBe(72) -- salt 2, tile 1, poly 0 for the configured bit split.
    expect(newRef ~= rightRef)
    expect(not dt.isValidPolyRef(nav, oldPolyRef))

    local result = dt.query(nav, vector.create(0.1,0,0.5), vector.create(1.9,0,0.5), vector.create(0.5,1,0.5))
    expect(result ~= nil, 're-added tile should reconnect its seam')
end)

suite.it('restores a requested lastRef slot and salt like dtNavMesh::addTile', function()
    local nav, _, rightRef = tiledPair()
    local removed, status = dt.removeTile(nav, rightRef)
    expect(removed ~= nil and dt.statusSucceed(status))

    local restoredRef, addStatus = dt.addTile(nav, removed, 1, 0, 0, rightRef)
    expect(dt.statusSucceed(addStatus))
    jt.expect(restoredRef).toBe(rightRef)
    jt.expect(dt.getTileByRef(nav, rightRef) ~= nil).toBeTruthy()
    expect(dt.isValidPolyRef(nav, rightRef + 1))
end)

return suite
