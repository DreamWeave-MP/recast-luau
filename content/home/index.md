+++
title = "recast-luau"
description = "A pure-Luau port of RecastNavigation and Detour for Jess: navmesh building and pathfinding with no C or C++ ABI."
date = 2026-10-21

[taxonomies]
tags = ["Jess", "Luau"]

[extra]
sections = ["overview", "releases", "credits"]
+++

A pure-Luau port of RecastNavigation and Detour for Jess: navmesh building and pathfinding with no C or C++ ABI.

<!-- more -->

A [Jess](https://github.com/DreamWeave-MP/jess) package, `org.dreamweave.recast`. Add it to a `jess.toml` and
`jess workspace sync` pins it from this site:

```toml
[dependencies]
"org.dreamweave.recast" = "0.2"
```

Every release below is the package's own archive, with its content digest beside the archive's
SHA-256; `jess.lock` pins the first and checks both.
