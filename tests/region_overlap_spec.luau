--!strict
--!native
-- A watershed region that floods up a ramp and over its own start has two spans in one column.
-- rcBuildRegions never merges such a region with another and reports it, by its final id, in the
-- list of overlapping regions it returns.
local core = require '@recast/core'
local test = require '@jess/test'
local expect = test.expect
local suite = test.new()

local Rise = 0.2

--- One stair per entry of `flights`, side by side: steps climbing round a square, three units
--- wide, that many times. Each turn after a stair's first lies over the one before it.
local function stairs(flights: { number }): ({ vector }, { { number } })
  local circuit: { { number } } = {}

  for column = 0, 8 do
    table.insert(circuit, { column, 0, column + 1, 3 })
  end

  table.insert(circuit, { 9, 0, 12, 3 })

  for row = 3, 8 do
    table.insert(circuit, { 9, row, 12, row + 1 })
  end

  table.insert(circuit, { 9, 9, 12, 12 })

  for column = 8, 3, -1 do
    table.insert(circuit, { column, 9, column + 1, 12 })
  end

  table.insert(circuit, { 0, 9, 3, 12 })

  for row = 8, 3, -1 do
    table.insert(circuit, { 0, row, 3, row + 1 })
  end

  local vertices: { vector }, triangles: { { number } } = {}, {}

  for flight, turns in flights do
    local offset, height = (flight - 1) * 16, 0

    for _ = 1, turns do
      for _, step in circuit do
        local base = #vertices

        table.insert(vertices, vector.create(offset + step[1], height, step[2]))
        table.insert(vertices, vector.create(offset + step[1], height, step[4]))
        table.insert(vertices, vector.create(offset + step[3], height, step[2]))
        table.insert(vertices, vector.create(offset + step[3], height, step[4]))
        table.insert(triangles, { base + 1, base + 2, base + 3 })
        table.insert(triangles, { base + 3, base + 2, base + 4 })
        height += Rise
      end
    end
  end

  return vertices, triangles
end

local function regions(flights: { number })
  local vertices, triangles = stairs(flights)

  return core.buildCompactFromTriangles(vertices, triangles, {
    cs = 0.5,
    ch = Rise,
    walkableHeight = 4,
    walkableClimb = 2,
    walkableRadius = 0,
  }).compact
end

suite.it('reports no overlap for a stair that never passes over itself', function()
  local compact = regions { 1 }

  expect(compact.maxRegions).toBe(2)
  expect(#compact.regionOverlaps).toBe(0)
end)

suite.it('reports the region that floods over its own start', function()
  local compact = regions { 2 }

  expect(compact.maxRegions).toBe(2)
  expect(table.concat(compact.regionOverlaps, ' ')).toBe('1')
end)

suite.it('reports only the stairs that overlap, in region order', function()
  local compact = regions { 2, 1, 3, 1 }

  expect(compact.maxRegions).toBe(5)
  expect(table.concat(compact.regionOverlaps, ' ')).toBe('1 3')
end)

return suite
