--!strict
--!native
-- The order of a serialized tile's BV nodes is part of its bytes: dtCreateNavMeshData sorts each
-- subtree's polygons by their quantized minimum on its longest axis, and upstream's qsort keeps
-- tied polygons in the order it found them. These cases hold whole tiles, built from meshes
-- whose minima tie on every axis, to the bytes the builder gave with the stable sort its order
-- was checked against upstream with.
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
  bmin: vector,
  bmax: vector,
  walkableHeight: number,
  walkableRadius: number,
  walkableClimb: number,
  cs: number,
  ch: number,
  buildBvTree: boolean,
}

local QuadVertices = 4
local NoVertex = 0xffff

--- `size` by `size` unit quads on a flat grid, each wired to its four neighbours: every minimum
--- ties across a whole row or column.
local function grid(size: number): Mesh
  local side = size + 1
  local polygonCount = size * size
  local verts = buffer.create(side * side * 6)
  local polys = buffer.create(polygonCount * QuadVertices * 4)
  local flags, areas = buffer.create(polygonCount * 2), buffer.create(polygonCount)

  buffer.fill(polys, 0, 0xff)
  buffer.fill(areas, 0, 1)

  for row = 0, size do
    for column = 0, size do
      local offset = (row * side + column) * 6

      buffer.writeu16(verts, offset, column)
      buffer.writeu16(verts, offset + 4, row)
    end
  end

  for row = 0, size - 1 do
    for column = 0, size - 1 do
      local polygon = row * size + column
      local offset = polygon * QuadVertices * 4
      local corner = row * side + column

      buffer.writeu16(flags, polygon * 2, 1)
      buffer.writeu16(polys, offset, corner)
      buffer.writeu16(polys, offset + 2, corner + 1)
      buffer.writeu16(polys, offset + 4, corner + side + 1)
      buffer.writeu16(polys, offset + 6, corner + side)

      if row > 0 then buffer.writeu16(polys, offset + 8, polygon - size) end

      if column < size - 1 then buffer.writeu16(polys, offset + 10, polygon + 1) end

      if row < size - 1 then buffer.writeu16(polys, offset + 12, polygon + size) end

      if column > 0 then buffer.writeu16(polys, offset + 14, polygon - 1) end
    end
  end

  return {
    verts = verts,
    vertCount = side * side,
    polys = polys,
    polyFlags = flags,
    polyAreas = areas,
    polyCount = polygonCount,
    nvp = QuadVertices,
    bmin = vector.zero,
    bmax = vector.create(size, 1, size),
    walkableHeight = 2,
    walkableRadius = 0.5,
    walkableClimb = 0.5,
    cs = 1,
    ch = 1,
    buildBvTree = true,
  }
end

--- Gives `grid(size)` a detail mesh: each quad's corners and a centre sample at one of five
--- heights, fanned into four triangles.
local function withGridDetail(mesh: Mesh, size: number): Mesh
  local meshes = buffer.create(mesh.polyCount * 16)
  local verts, tris = buffer.create(mesh.polyCount * 5 * 12), buffer.create(mesh.polyCount * 4 * 4)
  local cornerOffsets = { 0, 0, 1, 0, 1, 1, 0, 1 }

  for row = 0, size - 1 do
    for column = 0, size - 1 do
      local polygon = row * size + column
      local vertex, triangle = polygon * 5 * 12, polygon * 4 * 4

      buffer.writeu32(meshes, polygon * 16, polygon * 5)
      buffer.writeu32(meshes, polygon * 16 + 4, 5)
      buffer.writeu32(meshes, polygon * 16 + 8, polygon * 4)
      buffer.writeu32(meshes, polygon * 16 + 12, 4)

      for corner = 0, 3 do
        buffer.writef32(verts, vertex + corner * 12, column + cornerOffsets[corner * 2 + 1])
        buffer.writef32(verts, vertex + corner * 12 + 8, row + cornerOffsets[corner * 2 + 2])
        buffer.writeu8(tris, triangle + corner * 4, corner)
        buffer.writeu8(tris, triangle + corner * 4 + 1, (corner + 1) % 4)
        buffer.writeu8(tris, triangle + corner * 4 + 2, 4)
        buffer.writeu8(tris, triangle + corner * 4 + 3, 1)
      end

      buffer.writef32(verts, vertex + 48, column + 0.5)
      buffer.writef32(verts, vertex + 52, (column * 7 + row * 13) % 5 * 0.25)
      buffer.writef32(verts, vertex + 56, row + 0.5)
    end
  end

  mesh.detailMeshes, mesh.detailVerts, mesh.detailVertsCount = meshes, verts, mesh.polyCount * 5
  mesh.detailTris, mesh.detailTriCount = tris, mesh.polyCount * 4

  return mesh
end

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

--- `polygonCount` polygons of three to six vertices picked from a small lattice, in no spatial
--- order: minima tie on all three axes and the longest axis changes from subtree to subtree,
--- the tall one included. The builder reads only vertices here, so the polygons need not tile.
local function soup(seed: number, polygonCount: number, spanX: number, spanY: number, spanZ: number): Mesh
  local nvp = 6
  local pick = generator(seed)
  local vertexCount = math.max(8, polygonCount)
  local verts = buffer.create(vertexCount * 6)
  local polys = buffer.create(polygonCount * nvp * 4)
  local flags, areas = buffer.create(polygonCount * 2), buffer.create(polygonCount)

  buffer.fill(polys, 0, 0xff)

  for vertex = 0, vertexCount - 1 do
    buffer.writeu16(verts, vertex * 6, pick(spanX))
    buffer.writeu16(verts, vertex * 6 + 2, pick(spanY))
    buffer.writeu16(verts, vertex * 6 + 4, pick(spanZ))
  end

  for polygon = 0, polygonCount - 1 do
    buffer.writeu16(flags, polygon * 2, 1)
    buffer.writeu8(areas, polygon, pick(64))

    for corner = 0, 2 + pick(4) do
      buffer.writeu16(polys, polygon * nvp * 4 + corner * 2, pick(vertexCount))
    end
  end

  return {
    verts = verts,
    vertCount = vertexCount,
    polys = polys,
    polyFlags = flags,
    polyAreas = areas,
    polyCount = polygonCount,
    nvp = nvp,
    bmin = vector.create(-3, 1, 7),
    bmax = vector.create(spanX * 0.3 - 3, spanY * 0.5 + 1, spanZ * 0.3 + 7),
    walkableHeight = 2,
    walkableRadius = 0.5,
    walkableClimb = 0.5,
    cs = 0.3,
    ch = 0.5,
    buildBvTree = true,
  }
end

--- Gives a soup a detail mesh: each polygon's own vertices, then up to three samples on a
--- quarter-cell lattice, which is what the builder quantizes its BV bounds from when a detail
--- mesh is present.
local function withSoupDetail(mesh: Mesh, seed: number, spanX: number, spanY: number, spanZ: number): Mesh
  local pick = generator(seed)
  local meshes = buffer.create(mesh.polyCount * 16)
  local verts = buffer.create(mesh.polyCount * 9 * 12)
  local tris = buffer.create(mesh.polyCount * 7 * 4)
  local vertexCount, triangleCount = 0, 0

  for polygon = 0, mesh.polyCount - 1 do
    local corners = 0

    while corners < mesh.nvp and buffer.readu16(mesh.polys, polygon * mesh.nvp * 4 + corners * 2) ~= NoVertex do
      corners += 1
    end

    local samples = pick(4)

    buffer.writeu32(meshes, polygon * 16, vertexCount)
    buffer.writeu32(meshes, polygon * 16 + 4, corners + samples)
    buffer.writeu32(meshes, polygon * 16 + 8, triangleCount)
    buffer.writeu32(meshes, polygon * 16 + 12, corners - 2 + samples)

    for corner = 0, corners - 1 do
      local vertex = buffer.readu16(mesh.polys, polygon * mesh.nvp * 4 + corner * 2) * 6
      local offset = (vertexCount + corner) * 12

      buffer.writef32(verts, offset, mesh.bmin.X + buffer.readu16(mesh.verts, vertex) * mesh.cs)
      buffer.writef32(verts, offset + 4, mesh.bmin.Y + buffer.readu16(mesh.verts, vertex + 2) * mesh.ch)
      buffer.writef32(verts, offset + 8, mesh.bmin.Z + buffer.readu16(mesh.verts, vertex + 4) * mesh.cs)
    end

    for sample = 0, samples - 1 do
      local offset = (vertexCount + corners + sample) * 12

      buffer.writef32(verts, offset, mesh.bmin.X + pick(spanX * 4) * mesh.cs / 4)
      buffer.writef32(verts, offset + 4, mesh.bmin.Y + pick(spanY * 4) * mesh.ch / 4)
      buffer.writef32(verts, offset + 8, mesh.bmin.Z + pick(spanZ * 4) * mesh.cs / 4)
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

local function digest(mesh: Mesh): string
  local tile, problem = builder.createNavMeshData(mesh)

  if not tile then error(problem, 2) end

  expect(builder.navMeshHeader(tile).bvNodeCount).toBe(mesh.polyCount * 2)

  return bytes.sha256(tile)
end

suite.it('keeps tied polygons in order across a flat grid', function()
  local expected = {
    [1] = '07a888716b4da88d70feee28e4f02169c9317cd3142e0720b6ffdd9750aa5588',
    [2] = '3bf935591270d07d12145c207061d613566da0edb94ab4e19bd1735502dc7de8',
    [3] = '3040392fd09f28901aff19d7358953406a6d0b7e523113a13f2178f0cbcecb79',
    [5] = '35e1726c3442d778a22990a338265eee0bca8bc25adbb2d9a794567988d83c07',
    [8] = 'dc111fbaa0b6dce7489058643c7af6bc8879bf52e06b02ed20e671d3690c42e6',
    [9] = '2e754ca745b669c64e1a20f2f5d3f2098bc7b18792cadf1d3b6250fa978bddeb',
    [13] = 'ec9228b8dd074fc8cc83f24bf374d9b1df5cb64f95b4f09f9f1c46fd5ca9a75d',
    [16] = 'dae5529cc2984f6dd1c1e5505aed15144e87ba4ed6d32ede077354e7a5a9fdd9',
    [17] = '56c4e45c3491a1958966a5c522fc63ec70f56b40ed5d6b99062801ad82bf09c2',
    [33] = '1482d38d27cbdcb565ebd73d6c5e764fe3a7cb8054e01e0fbb6f2f5f49965601',
    [64] = 'a5190efa32983e81ea23ef11f81d07e1469f4195b74b6632696c27a370aff9a7',
    [100] = 'd1e6b82d0c08215444b9e3127477dd47b6dad09a63abce21298f506e6259f1b6',
  }

  for size, want in expected do
    expect(digest(grid(size))).toBe(want)
  end
end)

suite.it('keeps tied polygons in order when the bounds come from a detail mesh', function()
  local expected = {
    [1] = '63e534fa2003a1a0679c83d8cd18503faa5d7d35dc1b666f940ddee3f033c501',
    [3] = '84e0f20cf2b130e831fc2c684f5443efce4f4a6192617fc8b0b9b1894c8ea373',
    [8] = '687975bf4343f144f45ba9cc2b66116ccb5e4f6e63f174f302d15d859b8d299b',
    [17] = 'e5a050a05b24bc1b32263e81d9f4b7cf835e3653e619a7c3fdeff7d94705271c',
    [40] = 'c42430575352850c161748b65519d5ba1ca1e8c16042ef2783c0743a1549cb78',
  }

  for size, want in expected do
    expect(digest(withGridDetail(grid(size), size))).toBe(want)
  end
end)

suite.it('sorts each subtree on its own longest axis, the tall one included', function()
  local expected = {
    { seed = 0x25461380, polygons = 1, spanX = 4, spanY = 4, spanZ = 4, want = '9d47637d9d62d07115488dbcb1c7bde994f4e65d71acb658788282fe8ab27f57' },
    { seed = 0x2546326f, polygons = 2, spanX = 4, spanY = 4, spanZ = 4, want = '156c0714bae7f019b609a063da0b7514ed10ffea7d4a7c007e3be8936e4c6244' },
    { seed = 0x2546515e, polygons = 3, spanX = 4, spanY = 4, spanZ = 4, want = 'c155d7ec08c1e9e5ea99cf501acbfd02a585fa0f7d8a8933575ddb80886a30b2' },
    { seed = 0x2546704d, polygons = 8, spanX = 3, spanY = 3, spanZ = 3, want = '530a1cf1e590c89abcdfa122e9eeec82606cc620e130639e8305cf7959d66f54' },
    { seed = 0x25468f3c, polygons = 9, spanX = 3, spanY = 3, spanZ = 3, want = '8f5f8a4c5f58adbf98099eef479fdc28a09834373e95614b0aba62185eb4ebed' },
    { seed = 0x2546ae2b, polygons = 17, spanX = 6, spanY = 40, spanZ = 6, want = '8d91b4a231422f7370b877ec7303510407b0818e3b77cb2112eeb9b7f479b16c' },
    { seed = 0x2546cd1a, polygons = 100, spanX = 6, spanY = 40, spanZ = 6, want = 'a0bcbec652842cdb4a8cdaa6c8f998ef1c11205f6ee2affdbe9553600877360f' },
    { seed = 0x2546ec09, polygons = 100, spanX = 40, spanY = 3, spanZ = 5, want = '179b39b8a430471af5a59d77503faf6a5bdc1cbda09c58d7ba90d750aaf0a4a0' },
    { seed = 0x25470af8, polygons = 100, spanX = 5, spanY = 3, spanZ = 40, want = 'b914aac1a61f47ce2593394f51913c53b143fb15d089d04c3717db17c22b117e' },
    { seed = 0x254729e7, polygons = 1000, spanX = 3, spanY = 3, spanZ = 3, want = '840d769e5184b7844b5e3e138ebb691fa115f24878ad6de141f9c06cc6811e03' },
    { seed = 0x254748d6, polygons = 1000, spanX = 12, spanY = 60, spanZ = 9, want = '60f9fa9e20a157dbe3a207ca0e56b0d0f65e4c57c25e866ac6d8719e6ad9c056' },
    { seed = 0x254767c5, polygons = 1000, spanX = 300, spanY = 300, spanZ = 300, want = 'd659ac41310123fceae6715c6a2718978adb0bb8287f5298f7c05ac4adad1105' },
    { seed = 0x254786b4, polygons = 5000, spanX = 16, spanY = 16, spanZ = 16, want = 'e2231282fe137e5f6b3853bb2954ee5c2a813faba6a694544bfe6a9ce3503cbd' },
    { seed = 0x2547a5a3, polygons = 5000, spanX = 7, spanY = 90, spanZ = 31, want = 'ade3d809053979c5f386679b018d1d9369c152e869ee2db25274224dc05480eb' },
  }

  for _, case in expected do
    expect(digest(soup(case.seed, case.polygons, case.spanX, case.spanY, case.spanZ))).toBe(case.want)
  end
end)

suite.it('sorts a detail mesh on each axis by its quantized minimum', function()
  local expected = {
    { seed = 0x6b8cde80, polygons = 1, spanX = 4, spanY = 4, spanZ = 4, want = '1228c6c907b6f847d24124b36ffc632011241c84b99a1ef3561a94998f812df4' },
    { seed = 0x6b8e7799, polygons = 2, spanX = 4, spanY = 4, spanZ = 4, want = '84020624fd08a901efd99d41bf23122849a30ad106e460de88edb9eceb4784c6' },
    { seed = 0x6b9010b2, polygons = 3, spanX = 4, spanY = 4, spanZ = 4, want = 'edd778076a230ec99ad5b75e1b3d68fab4f15bc1da6bea9a620769f3ec0dba76' },
    { seed = 0x6b91a9cb, polygons = 8, spanX = 3, spanY = 3, spanZ = 3, want = '13c917be0ebe9e49d225fb08cd81dfa77d438115bd314b1dee053a4f35a1f1b4' },
    { seed = 0x6b9342e4, polygons = 9, spanX = 3, spanY = 3, spanZ = 3, want = '946861c0a6d4f23eff7fafc42d2ffdddd81f07be2310b80281f3baf9a8434589' },
    { seed = 0x6b94dbfd, polygons = 17, spanX = 6, spanY = 40, spanZ = 6, want = '9425b57debc9e26c16472daba9b2a260ecd3f45298a54bd40a58fc1faa6c4a39' },
    { seed = 0x6b967516, polygons = 100, spanX = 6, spanY = 40, spanZ = 6, want = '53a781f7e74fa57da5c50a46afb2bc03c49437e03acdcc35fe71b8f2c08f1a0d' },
    { seed = 0x6b980e2f, polygons = 100, spanX = 40, spanY = 3, spanZ = 5, want = 'b3b91c1f79eeb86a42d6dff94469628c2c1010c144b4085e61a93a27131c4ed0' },
    { seed = 0x6b99a748, polygons = 100, spanX = 5, spanY = 3, spanZ = 40, want = '6be394c6f7acdadc4f29b89d0a38e68d9af6895af71945c905a569d581ae1f3c' },
    { seed = 0x6b9b4061, polygons = 1000, spanX = 3, spanY = 3, spanZ = 3, want = '229cff3becf3584bcdc375b0b27aaae1432ad6736e8b66f9603d831da2a4ebd2' },
    { seed = 0x6b9cd97a, polygons = 1000, spanX = 12, spanY = 60, spanZ = 9, want = '0d14f4594d1e0e940e750bdb8cc972bdc6376d1c464eba8303cd03b82d078d80' },
    { seed = 0x6b9e7293, polygons = 1000, spanX = 300, spanY = 300, spanZ = 300, want = 'e62ee6eec9b1598e37a61c2c9c04297825c4fcf48215db90fa2cf7a1a8b7450f' },
    { seed = 0x6ba00bac, polygons = 5000, spanX = 16, spanY = 16, spanZ = 16, want = '398c89a75da225f79bb9d289ba7c6d66937b50e5805a0e3b4d9292dedc2d6ba5' },
    { seed = 0x6ba1a4c5, polygons = 5000, spanX = 7, spanY = 90, spanZ = 31, want = 'da7bdc8f4dfb675d5e583e4ba07e91385b28a127b21466239ddae88602b634b6' },
  }

  for _, case in expected do
    local mesh = soup(case.seed, case.polygons, case.spanX, case.spanY, case.spanZ)

    expect(digest(withSoupDetail(mesh, case.seed + 1, case.spanX, case.spanY, case.spanZ))).toBe(case.want)
  end
end)

return suite
