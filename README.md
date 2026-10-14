# recast-luau for Jess

A **pure-Luau RecastNavigation / Detour port** for Jess. The target is 1:1 observable behavior where the upstream behavior is meaningful in Luau, while keeping the runtime free of a Recast C/C++ ABI, Rust shim, FFI bridge, or privileged engine implementation.

Runtime source is Luau and leans heavily on Jess/L3i's `buffer`, native `vector`, bit operations, and `--!native` compilation, and on l3i's bulk-data plane (`@dream/data`) for the one sort whose order is part of a tile's bytes. `--!native` compiles Luau; it is not a native Recast dependency, and neither is the data plane: it knows numbers in buffers, not navmeshes.

Package: `org.dreamweave.recast`  
Primary import: `@recast`

## Quick start

```lua
local recast = require('@recast')

local build = recast.build(vertices, triangles, {
    cs = 0.3,
    ch = 0.2,
    agentHeight = 1.8,
    agentRadius = 0.6,
    agentMaxClimb = 0.9,
    walkableSlopeAngle = 45,
    partitionType = 'watershed',
    maxVertsPerPoly = 6,
})

local result, err = recast.query(
    build.navmesh,
    vector.create(0, 0, 0),
    vector.create(20, 0, 20),
    vector.create(2, 4, 2)
)
assert(result, err)

for _, point in result.points do print(point) end
```

Lua table triangle indices are 1-based. Packed triangle buffers use 0-based indices. Packed geometry is stored in native `buffer`s; position math uses Luau `vector`s.

## Modules

- `@recast` — convenience build/query surface.
- `@recast/core` — Recast build pipeline: heightfields through regions, contours, polymeshes, detail meshes, and layers.
- `@recast/detail` — detail-mesh helpers.
- `@recast/detour` — feature-complete Luau navmesh/query surface, tiled refs, binary tile data, BV trees, off-mesh links, filters, sliced queries.
- `@recast/navmesh` — low-level `dtNavMesh`-style tile/ref identity and tile-state module.
- `@recast/navmesh-builder` — `dtCreateNavMeshData`-compatible binary tile creation/decoding/endian operations.
- `@recast/crowd` — DetourCrowd-style multi-agent simulation.
- `@recast/path-corridor`, `@recast/path-queue`, `@recast/local-boundary` — Crowd support components.
- `@recast/proximity-grid`, `@recast/obstacle-avoidance` — Crowd spatial/avoidance components.
- `@recast/tilecache` — DetourTileCache-style compressed layers, dynamic obstacles, rebuilds.
- `@recast/common` — shared packed geometry/data structures.

## What is implemented

The current tree contains the full normal Recast build chain: rasterization and filters, compact fields, erosion/area editing, distance fields, watershed/monotone/layer regions, contours including holes, polygon meshes, detail meshes, and heightfield layers.

Detour includes tiled 32-bit salt/tile/poly refs, hashed tile lookup, clipped partial tile portals, BV acceleration, flags/areas/query filters, nearest-poly and polygon queries, normal/sliced/partial/any-angle A*, Dijkstra searches, funnel paths, raycast, local neighbourhood, movement-on-surface, wall queries, random points, off-mesh connections, and independent `dtNavMeshQuery`-style query contexts.

`@recast/navmesh-builder` creates and reads real Detour tile binary layouts, including detail data, BV nodes, off-mesh records, and endian operations. Binary/BV behavior is tested against upstream-generated fixtures, including tie-heavy BV ordering.

Crowd includes asynchronous PathQueue behavior, corridor repair/optimization, local boundaries, proximity grid, adaptive obstacle avoidance, separation, four-pass collision resolution, direct-velocity semantics, and off-mesh animation states. Crowd and PathQueue own independent query contexts like upstream.

TileCache includes Detour-style compressed-layer headers/lifecycle, regions/contours/polymesh, cylinder/AABB/oriented-box obstacles, mesh-process callbacks, and rebuild through the serialized Detour navmesh-data path.

For the detailed parity boundary, see `docs/PARITY_STATUS.md`.

## Query filters

```lua
local dt = require('@recast/detour')
local filter = dt.newQueryFilter({includeFlags = 0xffff, excludeFlags = 0x4})
dt.setAreaCost(filter, 3, 4.0)

local ref, nearest = dt.findNearestPoly(nav, position, extents, filter)
local path, status = dt.findPathPositioned(nav, ref, targetRef, nearest, target, filter)
```

Independent query objects are available when sliced or Dijkstra state must coexist:

```lua
local qa = assert(dt.newNavMeshQuery(nav, 2048))
local qb = assert(dt.newNavMeshQuery(nav, 2048))

qa:initSlicedFindPath(a0, a1, apos0, apos1, filter)
qb:initSlicedFindPath(b0, b1, bpos0, bpos1, filter)
-- qa and qb can now be advanced independently on the same navmesh.
```

## Performance notes

The port is intentionally profiler-driven. Several tempting “runtime primitive” ideas were rejected after better Luau layout/algorithm choices solved them.

`BENCHMARKS.md` has the current numbers, from `bench/bench.jsl`, and what moved them. The runs below are older, from the standalone stress scripts in `bench/`, and are kept for the shapes they show:

```text
Runtime BV query, 16,384 polys:
  10,000 box queries ~77.6 ms

Tiled lookup, 4096 tiles:
  100,000 hashed lookups ~12.6 ms
  100,000 linear lookups ~2639 ms

Crowd:
  256 agents, 60 updates ~77.8 ms total (~1.30 ms/update)

TileCache:
  64 tile updates ~0.586 ms total
```

Numbers are workload/machine dependent; use them as relative baselines, not promises.

One L3i optimization candidate **did** survive the full elimination process: a stable numeric-key permutation/argsort primitive for the exact serialized Detour BV builder. l3i 2.0 ships it as `@dream/data`'s `argsort`, and the builder sorts with it; the evidence that asked for it, the rejected alternatives, and the handoff are in `docs/L3I_OPTIMIZATION_REPORT.md`. Its workload is still here:

```sh
jess run --package . ./bench/navdata_bv_stress.jsl 254
jess run --package . --profile ./bench/navdata_bv_stress.jsl 254
```

## Testing

Run:

```sh
jess package test .
```

Jess executes manifest tests from both Package Source and a packed temporary JPK. The final release report shipped beside the artifact is authoritative for the exact passing count.

The suite covers Recast construction, Detour query/binary/tiled/off-mesh behavior, Crowd/PathQueue/avoidance, TileCache, endian/ref lifecycle, and differential upstream fixtures.

## Remaining parity boundary

The goal remains 1:1 behavior, but a few differences are structural rather than algorithmic:

- the package targets Detour's default **32-bit** poly refs, not `DT_POLYREF64`;
- native allocators/pointers and `rcContext` ownership are represented by Luau/GC objects rather than C++ memory interfaces;
- pointer/callback overloads are exposed in Luau-shaped forms;
- compressor byte streams depend on the supplied TileCache compressor implementation.

See `docs/PARITY_STATUS.md` for the maintained list.

## License / origin

This is an altered implementation based on RecastNavigation by Mikko Mononen and contributors. It is clearly marked as altered and is not represented as the original source. The upstream zlib-style license notice is retained in `RECAST-LICENSE.txt`.
