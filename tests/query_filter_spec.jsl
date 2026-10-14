--!strict
--!native
-- A query filter's area costs: a path's cost through an area is its length times that area's
-- cost, and an area nobody priced costs one.
local detour = require '@recast/detour'
local test = require '@jess/test'
local expect = test.expect
local suite = test.new()

suite.it('costs every area one until told otherwise', function()
  local filter = detour.newQueryFilter()

  expect(buffer.len(filter.areaCosts)).toBe(detour.DT_MAX_AREAS * 4)
  expect(filter.minAreaCost).toBe(1)

  for area = 0, detour.DT_MAX_AREAS - 1 do
    expect(buffer.readf32(filter.areaCosts, area * 4)).toBe(1)
  end
end)

suite.it('prices only the areas it is given, and remembers the cheapest', function()
  local filter = detour.newQueryFilter { areaCosts = { [0] = 4, [7] = 0.25, [63] = 2.5, [64] = 9 } }

  expect(filter.minAreaCost).toBe(0.25)

  for area = 0, detour.DT_MAX_AREAS - 1 do
    local want = if area == 0 then 4 elseif area == 7 then 0.25 elseif area == 63 then 2.5 else 1

    expect(buffer.readf32(filter.areaCosts, area * 4)).toBe(want)
  end
end)

return suite
