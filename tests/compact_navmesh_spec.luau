--!strict
--!native
-- The compact-cell navmesh makes two triangles of every walkable span of a compact heightfield
-- and links them through the cells' own connections. It has no BV tree, so a box query over it
-- tests every polygon. These cases hold the mesh and that query to what they gave when the
-- spans were counted and the link buffers filled one element at a time.
local bytes = require '@dream/bytes'
local detour = require '@recast/detour'
local test = require '@jess/test'
local expect = test.expect
local suite = test.new()

--- A square of `size` cells, two triangles each, with a wall of cells left out every eight
--- columns: eroding by the walkable radius then leaves spans beside each wall unwalkable.
local function walled(size: number): ({ vector }, { { number } })
  local side = size + 1
  local vertices: { vector } = table.create(side * side)
  local triangles: { { number } } = {}

  for row = 0, size do
    for column = 0, size do
      table.insert(vertices, vector.create(column, if row % 8 == 6 then 0.5 else 0, row))
    end
  end

  for row = 0, size - 1 do
    for column = 0, size - 1 do
      if column % 8 == 4 and row > 2 and row < size - 3 then continue end

      local corner = row * side + column + 1

      table.insert(triangles, { corner, corner + side, corner + 1 })
      table.insert(triangles, { corner + 1, corner + side, corner + side + 1 })
    end
  end

  return vertices, triangles
end

local function compactNavMesh(size: number, walkableRadius: number)
  local vertices, triangles = walled(size)

  return detour.buildFromTriangles(vertices, triangles, {
    cs = 1,
    ch = 0.25,
    walkableHeight = 2,
    walkableClimb = 1,
    walkableRadius = walkableRadius,
    compactCellNavmesh = true,
  }).navmesh
end

suite.it('makes two triangles of each walkable span and no more', function()
  local expected = {
    { size = 4, radius = 0, spans = 4, polygons = 8, want = '8b7405421ef006a5e97a31030af49c76aa454d3c883209d4244b09afb0c29a2b' },
    { size = 20, radius = 0, spans = 236, polygons = 472, want = '12a320617d10c27a5c469c8b64ee9abd1dffbd7a12cc7008f68fa999fb11c606' },
    { size = 20, radius = 1, spans = 236, polygons = 208, want = '9507460205210caf421420768250cbe05c63b0a5fd37084fc465b8387f309476' },
    { size = 33, radius = 2, spans = 629, polygons = 162, want = 'efbdf999f8aa515696f760d5a9757ce22b59a84a1482ed5355ab7dc5ee289924' },
  }

  for _, case in expected do
    local navmesh = compactNavMesh(case.size, case.radius)
    local buffers = {
      navmesh.vertices,
      navmesh.triangles,
      navmesh.areas,
      navmesh.flags,
      navmesh.polyValid,
      navmesh.neighbors,
      navmesh.centers,
      navmesh.sourceSpan,
      navmesh.spanToBase,
      navmesh.spanNeighbors,
    }

    expect(navmesh.compact.spanCount).toBe(case.spans)
    expect(navmesh.polyCount).toBe(case.polygons)
    expect(navmesh.vertexCount).toBe(case.polygons * 2)
    expect(bytes.sha256(table.concat([for part in buffers => bytes.sha256(part)], ' '))).toBe(case.want)
  end
end)

suite.it('answers a box query with every polygon the box touches, in polygon order', function()
  local navmesh = compactNavMesh(20, 1)
  local expected = {
    { center = vector.create(10, 0, 10), halfExtents = vector.create(30, 5, 30), found = 208, want = 'c614278541fa708f2015e2fdf8068cdfd595181d7f2e7c20032ff8c2662aabef' },
    { center = vector.create(2.5, 0, 2.5), halfExtents = vector.create(1.25, 1, 1.25), found = 2, want = 'f71998fe363b9c29116c80b5eecf33a2fedca3b6159724384485804b71651029' },
    { center = vector.create(12, 0.5, 6), halfExtents = vector.create(3, 0.2, 0.75), found = 12, want = '34a4a28b4f190ff5009d2be95de65ab10cceaec53fd210bc21df6404afce9952' },
    { center = vector.create(7.5, 0, 10), halfExtents = vector.create(0.4, 1, 4), found = 20, want = 'd67ff504e45a36da06c32a8fcce4dd71a2f997dd7e7d3b85278bb16adc7150ee' },
    { center = vector.create(10, 9, 10), halfExtents = vector.create(30, 1, 30), found = 0, want = 'e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855' },
  }

  for _, case in expected do
    local found = detour.queryPolygons(navmesh, case.center, case.halfExtents, nil)

    expect(#found).toBe(case.found)
    expect(bytes.sha256(table.concat(found, ' '))).toBe(case.want)
  end
end)

suite.it('answers a box query with nothing when the filter excludes every polygon', function()
  local navmesh = compactNavMesh(20, 1)
  local everything = vector.create(30, 5, 30)
  local found = detour.queryPolygons(navmesh, vector.create(10, 0, 10), everything, detour.newQueryFilter { excludeFlags = 1 })

  expect(#found).toBe(0)
  expect(#detour.queryPolygons(navmesh, vector.create(10, 0, 10), everything, detour.newQueryFilter())).toBe(navmesh.polyCount)
end)

return suite
