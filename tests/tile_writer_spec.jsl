--!strict
--!native
-- What dtCreateNavMeshData writes besides the BV tree's order: each polygon edge's link (a
-- neighbour, or the side of the tile a portal leaves by), the off-mesh connections that start
-- inside the tile, the detail records, and BV bounds quantized from detail vertices that may lie
-- outside the tile or nowhere finite. These cases hold whole tiles over all of that to the bytes
-- the writer gave when it read every parameter off its table inside its loops.
local builder = require '@recast/navmesh-builder'
local bytes = require '@dream/bytes'
local test = require '@jess/test'
local expect = test.expect
local suite = test.new()

type Mesh = {
  verts: buffer,
  vertCount: number,
  polys: buffer,
  polyFlags: buffer,
  polyAreas: buffer,
  polyCount: number,
  nvp: number,
  detailMeshes: buffer?,
  detailVerts: buffer?,
  detailVertsCount: number?,
  detailTris: buffer?,
  detailTriCount: number?,
  offMeshConVerts: buffer?,
  offMeshConRad: buffer?,
  offMeshConFlags: buffer?,
  offMeshConAreas: buffer?,
  offMeshConDir: buffer?,
  offMeshConUserID: buffer?,
  offMeshConCount: number?,
  bmin: vector,
  bmax: vector,
  walkableHeight: number,
  walkableRadius: number,
  walkableClimb: number,
  cs: number,
  ch: number,
  buildBvTree: boolean,
  userId: number,
  tileX: number,
  tileY: number,
  tileLayer: number,
}

local NoVertex = 0xffff
local Portal = 0x8000
-- The lattice a soup's vertices sit on, in cells.
local SpanX, SpanY, SpanZ = 9, 30, 7

--- A deterministic generator (xorshift32), so a soup is the same soup on every run.
local function generator(seed: number): (bound: number) -> number
  local state = seed

  return function(bound: number): number
    state = bit32.bxor(state, bit32.lshift(state, 13))
    state = bit32.bxor(state, bit32.rshift(state, 17))
    state = bit32.bxor(state, bit32.lshift(state, 5))

    return state % bound
  end
end

--- `polygonCount` polygons of three to `vertsPerPolygon` vertices in no spatial order. An edge
--- has no neighbour, a neighbour polygon, or a portal with any low nibble: only nibbles 0 to 3
--- name a side, and 15 is not a portal the link count sees. The writer reads the mesh, it does
--- not check it, so the polygons need not tile.
local function soup(seed: number, polygonCount: number, vertsPerPolygon: number, buildBvTree: boolean): Mesh
  local pick = generator(seed)
  local vertexCount = math.max(8, polygonCount)
  local verts = buffer.create(vertexCount * 6)
  local polys = buffer.create(polygonCount * vertsPerPolygon * 4)
  local flags, areas = buffer.create(polygonCount * 2), buffer.create(polygonCount)

  buffer.fill(polys, 0, 0xff)

  for vertex = 0, vertexCount - 1 do
    buffer.writeu16(verts, vertex * 6, pick(SpanX))
    buffer.writeu16(verts, vertex * 6 + 2, pick(SpanY))
    buffer.writeu16(verts, vertex * 6 + 4, pick(SpanZ))
  end

  for polygon = 0, polygonCount - 1 do
    local record = polygon * vertsPerPolygon * 4

    buffer.writeu16(flags, polygon * 2, pick(0x10000))
    buffer.writeu8(areas, polygon, pick(256))

    for corner = 0, 2 + pick(vertsPerPolygon - 2) do
      local edge = record + (vertsPerPolygon + corner) * 2
      local kind = pick(4)

      buffer.writeu16(polys, record + corner * 2, pick(vertexCount))

      if kind == 1 then
        buffer.writeu16(polys, edge, pick(polygonCount))
      elseif kind == 2 then
        buffer.writeu16(polys, edge, Portal + pick(16))
      end
    end
  end

  return {
    verts = verts,
    vertCount = vertexCount,
    polys = polys,
    polyFlags = flags,
    polyAreas = areas,
    polyCount = polygonCount,
    nvp = vertsPerPolygon,
    bmin = vector.create(-3, 1, 7),
    bmax = vector.create(SpanX * 0.3 - 3, SpanY * 0.5 + 1, SpanZ * 0.3 + 7),
    walkableHeight = 2,
    walkableRadius = 0.5,
    walkableClimb = 0.5,
    cs = 0.3,
    ch = 0.5,
    buildBvTree = buildBvTree,
    userId = seed,
    tileX = pick(100),
    tileY = pick(100),
    tileLayer = pick(4),
  }
end

--- Gives a soup a detail mesh: each polygon's own vertices, then up to three samples. With
--- `astray`, some samples sit below the tile's minimum, far past its maximum, exactly on its
--- minimum, or at an infinity, and some polygons lie outside the tile altogether: every one of
--- those quantizes to an end of the u16 range.
local function withDetail(mesh: Mesh, seed: number, astray: boolean): Mesh
  local pick = generator(seed)
  local meshes = buffer.create(mesh.polyCount * 16)
  local verts = buffer.create(mesh.polyCount * 9 * 12)
  local tris = buffer.create(mesh.polyCount * 7 * 4)
  local vertexCount, triangleCount = 0, 0

  for polygon = 0, mesh.polyCount - 1 do
    local record = polygon * mesh.nvp * 4
    local corners = 0

    while corners < mesh.nvp and buffer.readu16(mesh.polys, record + corners * 2) ~= NoVertex do
      corners += 1
    end

    local samples = pick(4)

    buffer.writeu32(meshes, polygon * 16, vertexCount)
    buffer.writeu32(meshes, polygon * 16 + 4, corners + samples)
    buffer.writeu32(meshes, polygon * 16 + 8, triangleCount)
    buffer.writeu32(meshes, polygon * 16 + 12, corners - 2 + samples)

    for corner = 0, corners - 1 do
      local vertex = buffer.readu16(mesh.polys, record + corner * 2) * 6
      local offset = (vertexCount + corner) * 12

      buffer.writef32(verts, offset, mesh.bmin.X + buffer.readu16(mesh.verts, vertex) * mesh.cs)
      buffer.writef32(verts, offset + 4, mesh.bmin.Y + buffer.readu16(mesh.verts, vertex + 2) * mesh.ch)
      buffer.writef32(verts, offset + 8, mesh.bmin.Z + buffer.readu16(mesh.verts, vertex + 4) * mesh.cs)
    end

    for sample = 0, samples - 1 do
      local offset = (vertexCount + corners + sample) * 12
      local stray = if astray then pick(12) else -1

      buffer.writef32(verts, offset, mesh.bmin.X + pick(SpanX * 4) * mesh.cs / 4)
      buffer.writef32(verts, offset + 4, mesh.bmin.Y + pick(SpanY * 4) * mesh.ch / 4)
      buffer.writef32(verts, offset + 8, mesh.bmin.Z + pick(SpanZ * 4) * mesh.cs / 4)

      if stray == 0 then
        buffer.writef32(verts, offset, mesh.bmin.X - pick(5) * mesh.cs)
      elseif stray == 1 then
        buffer.writef32(verts, offset + 4, mesh.bmin.Y - mesh.cs / 4)
      elseif stray == 2 then
        buffer.writef32(verts, offset + 8, 1e9)
      elseif stray == 3 then
        buffer.writef32(verts, offset, math.huge)
      elseif stray == 4 then
        buffer.writef32(verts, offset + 8, -math.huge)
      elseif stray == 5 then
        buffer.writef32(verts, offset + 4, mesh.bmin.Y)
      end
    end

    -- A whole polygon adrift: all of it past the tile's far side, or all of it short of the near
    -- one, so its minimum clamps at the top of the range or its maximum at the bottom.
    local adrift = if astray then pick(8) else -1

    if adrift == 0 or adrift == 1 then
      for vertex = vertexCount, vertexCount + corners + samples - 1 do
        local offset = vertex * 12 + (if adrift == 0 then 0 else 8)

        buffer.writef32(verts, offset, if adrift == 0 then 1e9 + vertex else mesh.bmin.Z - 2 - vertex % 3)
      end
    end

    for triangle = 0, corners - 3 + samples do
      buffer.writeu32(tris, (triangleCount + triangle) * 4, pick(0x01000000))
    end

    vertexCount += corners + samples
    triangleCount += corners - 2 + samples
  end

  mesh.detailMeshes, mesh.detailVerts, mesh.detailVertsCount = meshes, verts, vertexCount
  mesh.detailTris, mesh.detailTriCount = tris, triangleCount

  return mesh
end

--- Gives a mesh `count` off-mesh connections whose ends fall inside the tile, outside it on
--- every side, and above and below it: only one that starts inside is stored.
local function withConnections(mesh: Mesh, seed: number, count: number): Mesh
  local pick = generator(seed)
  local verts, radii = buffer.create(count * 24), buffer.create(count * 4)
  local flags, areas = buffer.create(count * 2), buffer.create(count)
  local directions, users = buffer.create(count), buffer.create(count * 4)

  for connection = 0, count - 1 do
    for point = 0, 1 do
      local offset = connection * 24 + point * 12

      buffer.writef32(verts, offset, mesh.bmin.X + (pick(SpanX * 2) - SpanX // 2) * mesh.cs)
      buffer.writef32(verts, offset + 4, mesh.bmin.Y + (pick(SpanY * 2) - SpanY // 2) * mesh.ch)
      buffer.writef32(verts, offset + 8, mesh.bmin.Z + (pick(SpanZ * 2) - SpanZ // 2) * mesh.cs)
    end

    buffer.writef32(radii, connection * 4, pick(8) / 4)
    buffer.writeu16(flags, connection * 2, pick(0x10000))
    buffer.writeu8(areas, connection, pick(256))
    buffer.writeu8(directions, connection, pick(3))
    buffer.writeu32(users, connection * 4, pick(0x7fffffff))
  end

  mesh.offMeshConVerts, mesh.offMeshConRad, mesh.offMeshConFlags = verts, radii, flags
  mesh.offMeshConAreas, mesh.offMeshConDir, mesh.offMeshConUserID = areas, directions, users
  mesh.offMeshConCount = count

  return mesh
end

local function digest(mesh: Mesh): string
  local tile, problem = builder.createNavMeshData(mesh)

  if not tile then error(problem, 2) end

  return bytes.sha256(tile)
end

suite.it('links each edge to its neighbour, or to the side its portal leaves by', function()
  local expected = {
    { seed = 0x3c6ef372, polygons = 1, corners = 3, tree = false, want = 'aeb7548f0056706026bf3775e6af60806246352a9bddfe546e169972744b61b9' },
    { seed = 0x3c6ef373, polygons = 2, corners = 4, tree = false, want = 'e501c6e397a4a1b8f996ed0a62a3099356d0205f0b5aff7ddc18225472cb24b0' },
    { seed = 0x3c6ef374, polygons = 40, corners = 3, tree = false, want = '46b814102cf0481a04d34e041b04b54e9fc30893ad6202d966bf82f2299964f5' },
    { seed = 0x3c6ef375, polygons = 40, corners = 4, tree = true, want = '91f0182ce7dd1fc39f0b338673b2dd6cdc814680d1baf3238f3f85f17bdb74a0' },
    { seed = 0x3c6ef376, polygons = 40, corners = 5, tree = false, want = '80ce890da9f486aadec8f5681fd5c5dc7aa7aec8ba9c459e55eb8135793632ae' },
    { seed = 0x3c6ef377, polygons = 300, corners = 6, tree = false, want = '3d508ca692826abd79e057cff1ef7d2cdd3b6f87a0c743a969ff32ca98e0aa8f' },
    { seed = 0x3c6ef378, polygons = 300, corners = 6, tree = true, want = '2ce773b3614ec60364b6a552bc6e9b68fd60b266b2a08823befce954fd8726d5' },
  }

  for _, case in expected do
    expect(digest(soup(case.seed, case.polygons, case.corners, case.tree))).toBe(case.want)
  end
end)

suite.it('keeps only the detail vertices a polygon does not already have', function()
  local expected = {
    { seed = 0x510e527f, polygons = 1, corners = 6, tree = false, want = '1c1119ab293f219bdaaa9f7ed147159d1a298ca8d57f8a3073447344ede8de56' },
    { seed = 0x510e5280, polygons = 17, corners = 3, tree = false, want = '0983af8c034727dfa6799d7d63da2fa36c05d80de6d20119c029c533cfa98153' },
    { seed = 0x510e5281, polygons = 200, corners = 5, tree = false, want = '168cfdeae3d692d9fbdc30a9e9815b39910fda5f71338e5bc7f228de06eda283' },
    { seed = 0x510e5282, polygons = 200, corners = 6, tree = true, want = '99a86c2f1e4852d5bc13b6459f156bcec52f13e9a7ececb3e5d335e36a092f98' },
  }

  for _, case in expected do
    local mesh = soup(case.seed, case.polygons, case.corners, case.tree)

    expect(digest(withDetail(mesh, case.seed + 1, false))).toBe(case.want)
  end
end)

suite.it('clamps BV bounds from detail vertices that leave the tile', function()
  local expected = {
    { seed = 0x1f83d9ab, polygons = 9, corners = 6, want = '244b8b4355fd273ac309ae512465e6526b2bdf88af66f3357306099f55b200c1' },
    { seed = 0x1f83d9ac, polygons = 17, corners = 4, want = 'bd22632f2bf5e6a7778a9ca2a29c31e9b456b145c5221fdd49cf1ace424065a2' },
    { seed = 0x1f83d9ad, polygons = 33, corners = 6, want = '436af7846a25aa420559ec1f1f51f69227a4422115cccddc75c4275cf95cd22c' },
    { seed = 0x1f83d9ae, polygons = 500, corners = 6, want = '94794c8364b8b00aba161650c4ea7bcc1b55cd57ce271d90997040beaae220aa' },
    { seed = 0x1f83d9af, polygons = 2000, corners = 5, want = '8bf4af57bb4b460c4c3927001c57c38a6572f1842491cd203705438d0bae4767' },
  }

  for _, case in expected do
    local mesh = soup(case.seed, case.polygons, case.corners, true)

    expect(digest(withDetail(mesh, case.seed + 1, true))).toBe(case.want)
  end
end)

suite.it('stores the off-mesh connections that start inside the tile', function()
  local expected = {
    { seed = 0x5be0cd19, polygons = 5, corners = 6, connections = 3, detail = false, stored = 0, want = '6f060ebad56600eb964f9063572d15ba4084fcced5a3de5c60e59ebae68d922d' },
    { seed = 0x5be0cd1a, polygons = 60, corners = 6, connections = 12, detail = false, stored = 2, want = '3c8e083fd40ea698a068c536807682daf00bfecad2f9295f65a50b63fe17def3' },
    { seed = 0x5be0cd1b, polygons = 60, corners = 4, connections = 12, detail = true, stored = 1, want = '50289fd78be28730996e824a7bff5d67b89b0b27031521c245efaaeea749a826' },
    { seed = 0x5be0cd1c, polygons = 200, corners = 6, connections = 40, detail = true, stored = 5, want = '525b9abf22c7689bc3ecb9bd0036f615d1ce7d7c9a25730b2e800dfea5a1d2e4' },
  }

  for _, case in expected do
    local mesh = soup(case.seed, case.polygons, case.corners, true)

    if case.detail then withDetail(mesh, case.seed + 1, false) end

    local tile = assert(builder.createNavMeshData(withConnections(mesh, case.seed + 2, case.connections)))

    expect(builder.navMeshHeader(tile).offMeshConCount).toBe(case.stored)
    expect(bytes.sha256(tile)).toBe(case.want)
  end
end)

return suite
