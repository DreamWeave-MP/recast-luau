--!native

local Recast = require('@recast/core')
local Detour = require('@recast/detour')
local Crowd = require('@recast/crowd')
local PathQueue = require('@recast/path-queue')
local TileCache = require('@recast/tilecache')
local NavMesh = require('@recast/navmesh')
local NavMeshBuilder = require('@recast/navmesh-builder')

local module = {
    Recast = Recast,
    Detour = Detour,
    Crowd = Crowd,
    PathQueue = PathQueue,
    TileCache = TileCache,
    NavMesh = NavMesh,
    NavMeshBuilder = NavMeshBuilder,
}

-- Convenience entry point for immediate game use: build a walkable triangle navmesh and query it.
module.buildTriangle = Detour.buildTriangleNavMesh
module.build = Detour.buildFromTriangles
module.query = Detour.query

return table.freeze(module)
