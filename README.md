# sokoban-solver-01

[![sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/ukyo9k/sokoban-solver-01/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/ukyo9k/sokoban-solver-01/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver, built for the [make.fun bounty](https://make.fun/bounty/sokoban-solver-01-run-20261121). One run answers all 5 instances of a set, writing each level's moves to `/output/<id>/solution.txt`.

## Design

- **Room decomposition.** Each level is a 1-wide hallway with 1–4 rooms off it. A box pushed out of a room into the hallway can never come back, so every room is an independent problem and the level optimum is the sum of the rooms' optima. Rooms are found by cutting the hallway row's dead squares (squares from which a box can never be pulled back onto a goal) out of the floor and taking the connected components that hold boxes.
- **Push-optimal A\* per room.** A state is the sorted box squares plus the player's reachable region (named by its lowest square). The heuristic is the cheapest assignment of boxes to goals by walking distance (ignoring other boxes), which never overestimates the pushes still needed, so the first solved state popped is push-optimal. Dead squares and frozen 2×2 blocks prune dead states; a transposition table keeps the best g per state.
- **Stitching.** Rooms are solved in turn with the other rooms' boxes standing as walls; the player's walk between pushes is a shortest BFS walk. If A\* runs out of its work budget on a room, the same search falls back to breadth-first order, then to greedy best-first, so an instance is always answered with a valid solution.
- **Deterministic.** All budgets are counts of expanded states, never the clock, and every tie is broken by a fixed order — two runs write identical bytes.

Practice set score: **1,000,000** (every instance solved in exactly P\* pushes), in ~0.4 s for the whole set — far inside the 60 s budget. The house baseline (plain A\*) scores 200,000 on the same set.

## Run locally

```sh
# Node 22+ (type-stripping, no dependencies)
MAKEFUN_INPUT_DIR=./practice/input MAKEFUN_OUTPUT_DIR=./out node main.ts
```

`main.ts` reads `/input/<id>/level.txt` for `i001`–`i005` (or the `MAKEFUN_INPUT_DIR` / `MAKEFUN_OUTPUT_DIR` overrides) and writes `/output/<id>/solution.txt` for each, creating the directories itself.

## License

MIT — see [LICENSE](LICENSE).
