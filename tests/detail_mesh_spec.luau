--!strict
--!native
-- rcBuildPolyMeshDetail adds height samples one at a time, each time the one farthest from the
-- mesh so far, and re-triangulates around it: which sample wins, and which of two near-equal
-- triangulations the Delaunay pass settles on, both come down to floating-point comparisons.
-- These cases hold whole detail meshes to the bytes those comparisons gave.
local bytes = require '@dream/bytes'
local detour = require '@recast/detour'
local test = require '@jess/test'
local expect = test.expect
local suite = test.new()

type Terrain = (column: number, row: number) -> number

--- A square of `size` cells, two triangles each, at the heights `terrain` gives its corners.
local function mesh(size: number, terrain: Terrain): ({ vector }, { { number } })
  local side = size + 1
  local vertices: { vector } = table.create(side * side)
  local triangles: { { number } } = table.create(size * size * 2)

  for row = 0, size do
    for column = 0, size do
      table.insert(vertices, vector.create(column, terrain(column, row), row))
    end
  end

  for row = 0, size - 1 do
    for column = 0, size - 1 do
      local corner = row * side + column + 1

      table.insert(triangles, { corner, corner + side, corner + 1 })
      table.insert(triangles, { corner + 1, corner + side, corner + side + 1 })
    end
  end

  return vertices, triangles
end

local function ridges(column: number, row: number): number
  return if column % 8 == 4 or row % 8 == 4 then 0.5 else 0
end

local function steps(column: number, row: number): number
  return (column * 3 + row * 5) % 7 * 0.1
end

local function bowl(column: number, row: number): number
  return ((column - 6) * (column - 6) + (row - 6) * (row - 6)) * 0.02
end

type Sampling = { cs: number, ch: number, detailSampleDist: number, detailSampleMaxError: number }

--- The digest of the detail mesh the standard pipeline builds: its submesh records, vertices and
--- triangles, and how many of each.
local function digest(size: number, terrain: Terrain, sampling: Sampling): string
  local vertices, triangles = mesh(size, terrain)
  local build = detour.buildFromTriangles(vertices, triangles, {
    cs = sampling.cs,
    ch = sampling.ch,
    walkableSlopeAngle = 50,
    walkableHeight = 4,
    walkableClimb = 2,
    walkableRadius = 0,
    maxSimplificationError = 1.3,
    detailSampleDist = sampling.detailSampleDist,
    detailSampleMaxError = sampling.detailSampleMaxError,
  })
  local detail = build.detailMesh

  expect(detail.nmeshes).toBe(build.polyMesh.npolys)

  local counts = `{detail.nmeshes} {detail.nverts} {detail.ntris}`

  return bytes.sha256(`{counts} {bytes.sha256(detail.meshes)} {bytes.sha256(detail.verts)} {bytes.sha256(detail.tris)}`)
end

suite.it('samples ridges the polygons span', function()
  expect(digest(16, ridges, { cs = 0.5, ch = 0.25, detailSampleDist = 2, detailSampleMaxError = 0.1 })).toBe('b6f4b4876faf0e4883b5fd002ed70f9973923a2f1c3fe2884fe784f73d804c3b')
  expect(digest(12, ridges, { cs = 0.5, ch = 0.25, detailSampleDist = 3, detailSampleMaxError = 0.5 })).toBe('79956f562646716bf8541daba3b8bdcf2d01960aef33ff08dea458d557a29978')
  expect(digest(12, ridges, { cs = 0.25, ch = 0.125, detailSampleDist = 4, detailSampleMaxError = 0.25 })).toBe('749481f0defc26de01dca86294afc17b052e9b37a677b536bace71f7fccce947')
end)

suite.it('samples stepped terrain, where many samples sit at one height', function()
  expect(digest(12, steps, { cs = 0.5, ch = 0.25, detailSampleDist = 2, detailSampleMaxError = 0.1 })).toBe('2fb260185b22bc634c9be7cb4d40d6c233c2992da43e7836a51f6d6b3fb6038f')
  expect(digest(10, steps, { cs = 0.5, ch = 0.1, detailSampleDist = 1, detailSampleMaxError = 0.05 })).toBe('3e8ced6dd5b4420f29300ea416da1f3ef5de3166a7ea2b45b1c299e4778ed60d')
end)

suite.it('samples a bowl, curved on both axes', function()
  expect(digest(12, bowl, { cs = 0.5, ch = 0.1, detailSampleDist = 2, detailSampleMaxError = 0.1 })).toBe('063df47b81870bb8e46e5c3dce77f994585977c5008562333ea61edd78dbb938')
  expect(digest(12, bowl, { cs = 0.5, ch = 0.25, detailSampleDist = 6, detailSampleMaxError = 1 })).toBe('63b05ed4cb00fafe3b92a24a71f6e4cb05fed746e861da96d37bc32fd7ebc922')
end)

suite.it('triangulates only the hull when sampling is off', function()
  expect(digest(12, ridges, { cs = 0.5, ch = 0.25, detailSampleDist = 0, detailSampleMaxError = 0.1 })).toBe('439f6a1765fa245c836a2b2ca91083a2e44c6fde764d7fa0495a4e38b64e93bd')
end)

return suite
