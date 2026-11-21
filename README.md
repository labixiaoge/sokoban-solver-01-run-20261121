# Sokoban solver — push-optimal, room by room

[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/labixiaoge/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/labixiaoge/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

## About

Entry for make.fun's ["Build the best Sokoban solver, 21 November 2026"](https://make.fun/bounty/sokoban-solver-01-run-20261121) bounty ($100 over the top 10). One program, `main.ts`, answers a whole set of 5 Sokoban levels in a single run.

## How it works

Every level the generator builds is one hallway row plus one to four rooms off it, each room holding 3–4 boxes and as many goals behind a single door. A box pushed out through a door can never come back, so every box stays in its room and the level's push-optimal count is the sum of its rooms' optima. The solver therefore:

1. splits the level into hallway and rooms (connected floor components off the hallway row, one door each);
2. solves each room push-optimally on its own with A\* over pushes — a state is the sorted box squares plus the player's reachable region, the heuristic is the cheapest box-to-goal assignment under shortest-path distances, and dead squares (reverse pull-reachability from the goals) plus frozen 2×2 blocks prune the search; a solved room must also let the player walk back out through its door;
3. stitches the room solutions together with plain walks through the hallway.

If a level ever looks different from the generator's shape, the solver falls back to one push-optimal search over the whole level, and to an empty (valid, zero-point) answer rather than crashing or writing anything invalid.

The search is bounded by work, never by the clock, and every tie-break is fixed, so two runs write the same bytes. No dependencies, no network, no clock reads.

## Results

On the kit's practice set the solver hits the known push-optimal count on every instance — 13, 16, 29, 47 and 60 pushes — for a score of **1,000,000 / 1,000,000** in about 0.9 s for the whole set. The house baseline (plain-astar 1.0.0) scores 200,000 on the same set: it solves only the first level and leaves the other four empty. The solver also scores 1,000,000 on five further sets freshly drawn from the same generator (different seeds), each in under a second.

## Run

```sh
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

Reads `/input/<id>/level.txt` (or `MAKEFUN_INPUT_DIR`) and writes `/output/<id>/solution.txt` (or `MAKEFUN_OUTPUT_DIR`), one run for the whole set. Node 22; plain JavaScript in a `.ts` file, no TypeScript-only syntax.
