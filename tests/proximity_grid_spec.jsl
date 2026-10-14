--!native
local jt = require('@jess/test')
local suite = jt.new()
local gridlib = require('@recast/proximity-grid')

local function expect(v, msg) if not v then error(msg or 'expectation failed', 2) end end

suite.it('matches Detour proximity-grid insertion, dedupe, and cell counts', function()
    local grid = gridlib.new(32, 3)
    expect(gridlib.addItem(grid, 7, -0.5, -0.5, 0.5, 0.5))
    expect(gridlib.addItem(grid, 9, 2.5, -0.5, 3.5, 0.5))
    local ids = gridlib.queryItems(grid, -1, -1, 4, 1, 32)
    table.sort(ids)
    jt.expect(#ids).toBe(2)
    jt.expect(ids[1]).toBe(7)
    jt.expect(ids[2]).toBe(9)
    expect(gridlib.getItemCountAt(grid, 0, 0) >= 1)
    local minx, miny, maxx, maxy = gridlib.getBounds(grid)
    expect(minx <= -1 and miny <= -1 and maxx >= 1 and maxy >= 0)
    jt.expect(gridlib.getCellSize(grid)).toBe(3)
end)

suite.it('clears without reallocating the packed pool', function()
    local grid = gridlib.new(8, 1)
    local pool = grid.pool
    gridlib.addItem(grid, 1, 0, 0, 0, 0)
    jt.expect(#gridlib.queryItems(grid, -0.1, -0.1, 0.1, 0.1, 8)).toBe(1)
    gridlib.clear(grid)
    jt.expect(grid.pool).toBe(pool)
    jt.expect(#gridlib.queryItems(grid, -0.1, -0.1, 0.1, 0.1, 8)).toBe(0)
end)

return suite
