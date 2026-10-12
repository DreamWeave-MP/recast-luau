# recast-luau benchmarks

`bench/bench.luau` times the package through its public API: Detour tiles serialized with and
without their BV tree, the Recast build with and without a sampled detail mesh, rasterization,
the runtime navmesh and its box and nearest-polygon queries, paths, the crowd, the tile cache, and
tiled navmeshes. Its inputs are made in the file: grids of unit quads wired to their neighbours
(every quantized minimum ties across a row or a column, which is the case the serialized BV tree's
order has to get right), a strip of the same quads, ridged terrain, and a maze. Each workload is
warmed up, sized to a batch of about a tenth of a second, and reported as the median of seven
batches, with a full collection between them. Every workload's result is checked: a tile against
its SHA-256, everything else against a count or a checksum.

```sh
jess run --package jpk/recast-luau jpk/recast-luau/bench/bench.luau [filter]
```

## The move onto l3i 2.0, base against head

- Machine: Intel Core i7-10870H @ 2.20 GHz, 16 threads, shared with other work while this ran.
- Date: 2026-10-12.
- Binary: the verified release build of Jess 0.1.0 on l3i 2.0.1 (Luau 0.742), the same one for
  both sides.
- Base: `bba81fc`, the package as imported, with this bench and the tests that pin tile bytes.
  Head: `3a538d3`.
- Protocol: the whole bench run against base, then head, three times over, back to back under one
  lock; each cell is the median of its three runs. A difference under 5 percent is called no
  change.

| Workload | Base | Head | Base / head |
|---|---:|---:|---:|
| tile, 16129 polygons, no BV tree | 5.17 ms | 2.20 ms | 2.35x |
| tile, 16129 polygons, BV tree | 53.53 ms | 12.62 ms | 4.24x |
| tile, 16129 polygons with a detail mesh, BV tree | 54.64 ms | 13.55 ms | 4.03x |
| tile, 64516 polygons, no BV tree | 22.82 ms | 10.14 ms | 2.25x |
| tile, 64516 polygons, BV tree | 263.90 ms | 56.48 ms | 4.67x |
| tile, a strip of 15 by 4000 polygons, BV tree | 214.05 ms | 42.84 ms | 5.00x |
| recast build with detail, 512 triangles | 63.24 ms | 10.51 ms | 6.02x |
| tile from a recast build, BV tree | 10.5 µs | 6.4 µs | 1.64x |
| load a tile from a recast build | 7.2 µs | 7.2 µs | no change |
| recast build, maze of 7392 triangles | 17.44 ms | 14.08 ms | 1.24x |
| query a path through the maze | 66.0 µs | 44.6 µs | 1.48x |
| rasterize 32768 triangles | 34.83 ms | 21.16 ms | 1.65x |
| runtime navmesh from 16384 polygons | 17.59 ms | 13.61 ms | 1.29x |
| 1000 box queries over 16384 polygons | 6.72 ms | 6.60 ms | no change |
| 1000 nearest-polygon queries over 16384 polygons | 68.41 ms | 48.16 ms | 1.42x |
| find a path across 9216 polygons | 4.74 ms | 3.63 ms | 1.31x |
| find a positioned path across 9216 polygons | 257.2 µs | 248.5 µs | no change |
| straighten a path of 191 polygons | 138.9 µs | 70.9 µs | 1.96x |
| crowd of 256 agents, 10 updates | 11.03 ms | 10.35 ms | 1.07x |
| tile cache, 64 tiles under 8 obstacles | 580.4 µs | 603.8 µs | no change |
| tiled navmesh, add 64 tiles | 1.15 ms | 1.18 ms | no change |
| tiled navmesh, find a path across 64 tiles | 66.3 µs | 51.0 µs | 1.30x |
| tiled navmesh, 10000 tile lookups | 787.1 µs | 820.0 µs | no change |

## What moved, and why

**The serialized BV tree, from l3i 2.0.** `dtCreateNavMeshData` sorts each subtree's polygons by
their minimum on the subtree's longest axis, and the order of ties is part of the tile's bytes.
The builder used to do that with a stable merge sort written in Luau, three copies of it, one per
axis. It now keeps the minima as f32 key columns in a buffer and the tree order as a u32 index
vector, and sorts a subtree with `@dream/data`: a `gather` of the keys into the subtree's order,
a stable `argsort` over them into caller scratch, and a `gather` of the order through the
permutation. Subtrees of sixteen polygons or fewer are sorted by insertion, which costs less than
three native calls. A subtree whose parent sorted on the same axis is not sorted at all (the strip
row, where one axis stays longest for eight levels, measured 1.21x for that against the commit
before it, three rounds each). On the 64516-polygon tile the tree alone went from about 241 ms
to about 46 ms. In an instrumented run the argsort form with caller scratch was the faster one
here (24 ms of sorting against 29 ms for the allocating form). What is left is the sorts
themselves, the bounds scan every subtree needs before it can pick an axis (sixteen levels over
64516 polygons in tree order, a random walk over the bounds at about 17 ns a polygon), and
quantizing the bounds.

**The rest of the tile writer** read every parameter off the normalized table inside its loops
and walked each polygon's vertices three times to count them. Reading the parameters into locals
and counting once is the 2.2x to 2.4x on the rows without a BV tree. Nothing there fits the data
plane: it is one pass of field-by-field writes.

**The detail mesh** is where a Recast build with sampling spends its time: the sampling loop
measures every remaining sample against every triangle on every pass and then re-triangulates.
Two things changed, neither of them the dialect. The part of each height test that depends only
on the triangle is worked out once per triangulation and kept in a buffer, so a sample pays for
one vector subtraction and a few multiplies; and the Delaunay pass finds the edge between two
points with one read of a `count * count` table of u16s where it used to scan its edge list. That
is about 4.1x of the detail row's 6.02x, and took a 32-cell terrain's build from 1.17 s to
0.16 s.

**Vector annotations.** Under `--!native`, reading `.X` of a value the compiler can't type goes
through the generic index path: three such reads in a loop measured 23 ns here against 4 ns for
a known vector. Almost nothing in this package was annotated. A parameter a function only ever
treats as a vector (or a list of them) is now annotated as one, 120 of them, and so are 60 locals
in the Recast and Detour modules. That is the rasterizer's 1.65x, straightening's 1.96x,
nearest-polygon's 1.42x, the path rows' 1.24x to 1.48x, the runtime navmesh's 1.29x, the crowd's
1.07x and the last 1.5x of the detail row. The box queries and the tile cache did not move. The
same annotations on the locals of the crowd, the tile cache and the detail builder measured as no
change and were not kept.

**The dialect itself** moved nothing that can be measured. This package is numeric loops over
buffers and graph searches with early exits; it builds few lists. Three list builds became two
comprehensions and a buffer sink, nine fill and count loops became `buffer.fill`, `data.fill` and
`data.count`, and none of those is on a path the bench can see.

## The workload l3i's argsort was added for

`docs/L3I_OPTIMIZATION_REPORT.md` measured `bench/navdata_bv_stress.luau`, and its numbers are the
reason l3i 2.0 has a stable `argsort` with caller scratch. The same script, same binary, base
against head, best of five:

| Polygons | | Base | Head |
|---:|---|---:|---:|
| 16129 | no BV tree | 5.22 ms | 2.20 ms |
| 16129 | with BV tree | 52.02 ms | 11.46 ms |
| 16129 | the tree alone | 46.81 ms | 9.26 ms |
| 64516 | no BV tree | 19.89 ms | 8.58 ms |
| 64516 | with BV tree | 267.35 ms | 57.47 ms |
| 64516 | the tree alone | 247.46 ms | 48.89 ms |

`jess run --profile` on the 64516-polygon run counted 94 million native blocks a build at base
and 21 million with the argsort in; the three merge loops that led the profile are gone from it,
not moved.
