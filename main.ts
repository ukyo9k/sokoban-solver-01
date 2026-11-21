// Sokoban solver for make.fun sokoban-solver-01-run-20261121.
//
// Design (written from scratch for this entry):
//   * A level is a 1-wide hallway with 1-4 rooms off it. A box pushed out of
//     a room into the hallway can never come back, so each room is an
//     independent problem and the level optimum is the sum of the rooms'
//     optima. Rooms are found by cutting the hallway row's dead squares
//     (squares from which a box can never be pulled back onto a goal) out of
//     the floor and taking the connected components that hold boxes.
//   * Each room is solved push-optimally with A* over pushes. A state is the
//     sorted box squares plus the player's reachable region (named by its
//     lowest square). The heuristic is the cheapest assignment of boxes to
//     goals by walking distance (ignoring other boxes), which never
//     overestimates the pushes still needed, so the first solved state popped
//     is push-optimal. Dead squares (a box can never leave them) and frozen
//     2x2 blocks prune dead states; a transposition table keeps the best g.
//   * Rooms are solved in turn with the other rooms' boxes standing as
//     walls; the player's walk between pushes is a shortest BFS walk. If A*
//     runs out of its work budget on a room, the same search falls back to
//     breadth-first order, then to greedy best-first (which still finds a
//     valid, if longer, solution), so an instance is always answered.
//   * All budgets are counts of expanded states, never the clock, and every
//     tie is broken by a fixed order, so two runs write identical bytes.

import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
const IDS = ['i001', 'i002', 'i003', 'i004', 'i005'];

// Work budgets (expanded states per room search). No clock is ever read.
const ASTAR_BUDGET = 250000;
const BFS_BUDGET = 250000;
const GREEDY_BUDGET = 250000;

// Directions in fixed order: up, down, left, right.
const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const LETTER = ['u', 'd', 'l', 'r'];

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

interface Room {
  boxes: number[];
  goals: number[];
  done: boolean;
}

/** Parse XSB text. Short rows pad with walls, like the checker. */
function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

/**
 * The squares from which a box can be pushed onto some goal, by pulling a
 * box back from the goals over open floor: a square y is live when some live
 * square x = y + d has the square y - d open for the player to stand on.
 */
function liveSquares(wall: Uint8Array, goals: number[], w: number, h: number): Uint8Array {
  const size = w * h;
  const live = new Uint8Array(size);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi];
    for (let k = 0; k < 4; k++) {
      const d = DY[k] * w + DX[k];
      const y = x - d;
      if (y < 0 || y >= size) continue;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

/** Walking distances from `from` over open floor (-1 where unreachable). */
function bfsDist(wall: Uint8Array, from: number, w: number, h: number): Int32Array {
  const size = w * h;
  const dist = new Int32Array(size).fill(-1);
  const queue = [from];
  dist[from] = 0;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    for (let k = 0; k < 4; k++) {
      const q = p + DY[k] * w + DX[k];
      if (q < 0 || q >= size) continue;
      if (!wall[q] && dist[q] < 0) {
        dist[q] = dist[p] + 1;
        queue.push(q);
      }
    }
  }
  return dist;
}

/** All permutations of 0..n-1, in lexicographic order (n <= 4 here). */
function permutations(n: number): number[][] {
  const out: number[][] = [];
  const cur: number[] = [];
  const used = new Uint8Array(n);
  const rec = (): void => {
    if (cur.length === n) {
      out.push(cur.slice());
      return;
    }
    for (let i = 0; i < n; i++) {
      if (used[i]) continue;
      used[i] = 1;
      cur.push(i);
      rec();
      cur.pop();
      used[i] = 0;
    }
  };
  rec();
  return out;
}

/**
 * The level's rooms: cut the dead squares of the player's row (the hallway:
 * a box there reaches no goal) out of the floor; each remaining part holding
 * boxes is a room. Null when the level is no set of rooms.
 */
function roomsOf(level: Level): Room[] | null {
  const { w, h, wall, goals, boxes, player } = level;
  const size = w * h;
  const live = liveSquares(wall, goals, w, h);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(size);
  for (let i = row * w; i < (row + 1) * w; i++) {
    if (!wall[i] && !live[i]) cut[i] = 1;
  }
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(size).fill(-1);
  const rooms: Room[] = [];
  for (let s = 0; s < size; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (let k = 0; k < 4; k++) {
        const q = squares[i] + DY[k] * w + DX[k];
        if (q < 0 || q >= size) continue;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ boxes: [], goals: [], done: false });
  }
  for (const b of boxes) rooms[part[b]].boxes.push(b);
  for (const g of goals) {
    if (part[g] < 0) return null;
    rooms[part[g]].goals.push(g);
  }
  if (rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

/**
 * Whether the box just pushed to `t` sits in a frozen 2x2 block: four
 * squares of walls and boxes holding a box off its goal can never come apart.
 */
function frozenBlock(wall: Uint8Array, occ: Uint8Array, goal: Uint8Array, t: number, w: number): boolean {
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occ[a]) || !(wall[b] || occ[b]) || !(wall[c] || occ[c]) || !(wall[d] || occ[d])) continue;
    if ((occ[a] && !goal[a]) || (occ[b] && !goal[b]) || (occ[c] && !goal[c]) || (occ[d] && !goal[d])) return true;
  }
  return false;
}

/**
 * The shortest walk from `from` to `to` around walls and boxes, in lower
 * LURD letters, or null when unreachable. Directions tried u, d, l, r.
 */
function walkTo(wall: Uint8Array, occ: Uint8Array, from: number, to: number, w: number, h: number): string | null {
  if (from === to) return '';
  const size = w * h;
  const prev = new Int32Array(size).fill(-1);
  const how = new Int8Array(size).fill(-1);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    if (p === to) break;
    for (let k = 0; k < 4; k++) {
      const q = p + DY[k] * w + DX[k];
      if (q < 0 || q >= size) continue;
      if (wall[q] || occ[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = k;
      queue.push(q);
    }
  }
  if (prev[to] === -1) return null;
  let s = '';
  for (let p = to; p !== from; p = prev[p]) s = LETTER[how[p]] + s;
  return s;
}

/**
 * A flood-fill region finder over open floor with `occ` in the way. Each
 * call returns its own marker so two finders never share marks: one names
 * the region a state expands from, the other the region a successor's key
 * is built on.
 */
function makeRegion(wall: Uint8Array, w: number, h: number, occ: Uint8Array): (from: number) => number {
  const size = w * h;
  const seen = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;
  return (from: number): number => {
    stamp++;
    let low = from;
    let sp = 0;
    stack[sp++] = from;
    seen[from] = stamp;
    while (sp > 0) {
      const p = stack[--sp];
      if (p < low) low = p;
      for (let k = 0; k < 4; k++) {
        const q = p + DY[k] * w + DX[k];
        if (q < 0 || q >= size) continue;
        if (!wall[q] && !occ[q] && seen[q] !== stamp) {
          seen[q] = stamp;
          stack[sp++] = q;
        }
      }
    }
    return low;
  };
}

/**
 * The player's reachable region from `from` with `occ` in the way, as a
 * boolean mask in `out` (indexed by square), plus its lowest square.
 */
function regionMask(wall: Uint8Array, w: number, h: number, occ: Uint8Array, from: number, out: Uint8Array): number {
  const size = w * h;
  const stack = new Int32Array(size);
  out.fill(0);
  let low = from;
  let sp = 0;
  stack[sp++] = from;
  out[from] = 1;
  while (sp > 0) {
    const p = stack[--sp];
    if (p < low) low = p;
    for (let k = 0; k < 4; k++) {
      const q = p + DY[k] * w + DX[k];
      if (q < 0 || q >= size) continue;
      if (!wall[q] && !occ[q] && !out[q]) {
        out[q] = 1;
        stack[sp++] = q;
      }
    }
  }
  return low;
}

/** The square the player stands on after `moves` from `from`. */
function endOf(w: number, from: number, moves: string): number {
  let p = from;
  for (const ch of moves) p += ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
  return p;
}

interface SearchResult {
  pushes: number;
  moves: string;
}

/**
 * Push-optimal (astar) / breadth-first (bfs) / greedy (greedy) search of one
 * room, from `player`, over at most `budget` expanded states. Returns the
 * push sequence as moves, or null when the budget runs out or no solution
 * exists. A state is the sorted box squares plus the player's reachable
 * region, named by its lowest square; the heuristic is the cheapest
 * box-to-goal assignment by walking distance, which never overestimates.
 */
function searchRoom(
  w: number,
  h: number,
  wall: Uint8Array,
  goals: number[],
  boxes0: number[],
  player0: number,
  budget: number,
  mode: 'astar' | 'bfs' | 'greedy',
): SearchResult | null {
  const size = w * h;
  const n = boxes0.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(wall, goals, w, h);
  for (const b of boxes0) if (!live[b]) return null;
  const dists = goals.map((g) => bfsDist(wall, g, w, h));
  const perms = permutations(n);
  const heuristic = (bs: number[]): number => {
    let best = Infinity;
    for (const p of perms) {
      let s = 0;
      for (let i = 0; i < n; i++) {
        const d = dists[p[i]][bs[i]];
        if (d < 0) {
          s = Infinity;
          break;
        }
        s += d;
      }
      if (s < best) best = s;
    }
    return best;
  };
  const solved = (bs: number[]): boolean => {
    for (const b of bs) if (!goal[b]) return false;
    return true;
  };
  const keyOf = (bs: number[], low: number): number => {
    let k = low;
    for (const b of bs) k = k * size + b;
    return k;
  };

  // Flood-fill scratch. Two region finders with separate marks: one names
  // the region a state expands from, the other the region a successor's key
  // is built on, so keying a successor never disturbs the expansion.
  const occ = new Uint8Array(size);
  const regionKey = makeRegion(wall, w, h, occ);
  const inRegion = new Uint8Array(size);

  const startBoxes = boxes0.slice().sort((a, b) => a - b);
  const startH = heuristic(startBoxes);
  if (startH === Infinity) return null;
  if (solved(startBoxes)) return { pushes: 0, moves: '' };

  // States, in creation order: boxes, player square, parent, push that made it.
  const sBoxes: number[][] = [startBoxes];
  const sPlayer: number[] = [player0];
  const sParent: number[] = [-1];
  const sPushFrom: number[] = [-1];
  const sPushTo: number[] = [-1];
  const sPushDir: number[] = [-1];
  const sG: number[] = [0];
  const sH: number[] = [startH];
  const closed = new Map<number, number>();
  for (const b of startBoxes) occ[b] = 1;
  closed.set(keyOf(startBoxes, regionKey(player0)), 0);
  for (const b of startBoxes) occ[b] = 0;

  // Binary heap of state indices by (priority, h, creation order).
  const prio = (s: number): number => (mode === 'astar' ? sG[s] + sH[s] : mode === 'bfs' ? sG[s] : sH[s]);
  const heap: number[] = [0];
  const less = (a: number, b: number): boolean => {
    const pa = prio(a);
    const pb = prio(b);
    if (pa !== pb) return pa < pb;
    if (sH[a] !== sH[b]) return sH[a] < sH[b];
    return a < b;
  };
  const heapPush = (s: number): void => {
    heap.push(s);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      const t = heap[i];
      heap[i] = heap[p];
      heap[p] = t;
      i = p;
    }
  };
  const heapPop = (): number => {
    const top = heap[0];
    const last = heap.pop() as number;
    if (heap.length > 0) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1;
        const r = l + 1;
        let m = i;
        if (l < heap.length && less(heap[l], heap[m])) m = l;
        if (r < heap.length && less(heap[r], heap[m])) m = r;
        if (m === i) break;
        const t = heap[i];
        heap[i] = heap[m];
        heap[m] = t;
        i = m;
      }
    }
    return top;
  };

  let expanded = 0;
  let found = -1;
  while (heap.length > 0) {
    const s = heapPop();
    if (solved(sBoxes[s])) {
      found = s;
      break;
    }
    if (expanded >= budget) break;
    expanded++;
    const bs = sBoxes[s];
    for (const b of bs) occ[b] = 1;
    regionMask(wall, w, h, occ, sPlayer[s], inRegion);
    for (let bi = 0; bi < n; bi++) {
      const b = bs[bi];
      for (let k = 0; k < 4; k++) {
        const d = DY[k] * w + DX[k];
        const t = b + d;
        if (t < 0 || t >= size) continue;
        if (inRegion[b - d] !== 1 || wall[t] || occ[t] || !live[t]) continue;
        occ[b] = 0;
        occ[t] = 1;
        const frozen = frozenBlock(wall, occ, goal, t, w);
        let key = -1;
        let next: number[] | null = null;
        let nextH = 0;
        if (!frozen) {
          next = bs.slice();
          next[bi] = t;
          next.sort((a, b2) => a - b2);
          nextH = heuristic(next);
          if (nextH !== Infinity) key = keyOf(next, regionKey(b));
        }
        occ[t] = 0;
        occ[b] = 1;
        if (frozen || key < 0) continue;
        const prevG = closed.get(key);
        if (prevG !== undefined && prevG <= sG[s] + 1) continue;
        closed.set(key, sG[s] + 1);
        sBoxes.push(next);
        sPlayer.push(b);
        sParent.push(s);
        sPushFrom.push(b);
        sPushTo.push(t);
        sPushDir.push(k);
        sG.push(sG[s] + 1);
        sH.push(nextH);
        heapPush(sBoxes.length - 1);
      }
    }
    for (const b of bs) occ[b] = 0;
  }
  if (found < 0) return null;

  // Rebuild the pushes, then expand each into a walk plus the push.
  const pushes: { from: number; to: number; dir: number }[] = [];
  for (let s = found; s > 0; s = sParent[s]) {
    pushes.push({ from: sPushFrom[s], to: sPushTo[s], dir: sPushDir[s] });
  }
  pushes.reverse();
  let moves = '';
  let cur = player0;
  const curBoxes = startBoxes.slice();
  for (const p of pushes) {
    const d = DY[p.dir] * w + DX[p.dir];
    const stand = p.from - d;
    for (const b of curBoxes) occ[b] = 1;
    const walk = walkTo(wall, occ, cur, stand, w, h);
    for (const b of curBoxes) occ[b] = 0;
    if (walk === null) return null;
    moves += walk + LETTER[p.dir];
    cur = p.from;
    curBoxes[curBoxes.indexOf(p.from)] = p.to;
  }
  return { pushes: pushes.length, moves };
}

/** Solve one level room by room, or null when any room resists all searches. */
function solveLevel(level: Level): SearchResult | null {
  const rooms = roomsOf(level);
  if (rooms === null) return null;
  let player = level.player;
  let moves = '';
  let pushes = 0;
  for (const room of rooms) {
    const wall = level.wall.slice();
    for (const other of rooms) {
      if (other === room) continue;
      for (const b of other.done ? other.goals : other.boxes) wall[b] = 1;
    }
    const subGoals = room.goals.slice().sort((a, b) => a - b);
    const subBoxes = room.boxes.slice().sort((a, b) => a - b);
    const solved =
      searchRoom(level.w, level.h, wall, subGoals, subBoxes, player, ASTAR_BUDGET, 'astar') ??
      searchRoom(level.w, level.h, wall, subGoals, subBoxes, player, BFS_BUDGET, 'bfs') ??
      searchRoom(level.w, level.h, wall, subGoals, subBoxes, player, GREEDY_BUDGET, 'greedy');
    if (solved === null) return null;
    moves += solved.moves;
    pushes += solved.pushes;
    player = endOf(level.w, player, solved.moves);
    room.done = true;
  }
  return { pushes, moves };
}

/**
 * Last resort for a level that is no set of rooms: a greedy whole-level
 * search (sum of nearest-goal distances as the heuristic), so the instance
 * still gets a valid answer when one is found.
 */
function solveWholeGreedy(level: Level): SearchResult | null {
  const { w, h, wall, goals, boxes, player } = level;
  const size = w * h;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(wall, goals, w, h);
  for (const b of boxes) if (!live[b]) return null;
  const dists = goals.map((g) => bfsDist(wall, g, w, h));
  const heuristic = (bs: number[]): number => {
    let s = 0;
    for (const b of bs) {
      let best = Infinity;
      for (let gi = 0; gi < goals.length; gi++) {
        const d = dists[gi][b];
        if (d >= 0 && d < best) best = d;
      }
      if (best === Infinity) return Infinity;
      s += best;
    }
    return s;
  };
  // Reuse the room search with the whole level as one room by temporarily
  // calling searchRoom with all goals/boxes: it uses the assignment
  // heuristic, which needs permutations of up to 16 boxes -- not feasible.
  // So run a small dedicated greedy search here instead.
  const solved = (bs: number[]): boolean => {
    for (const b of bs) if (!goal[b]) return false;
    return true;
  };
  const startBoxes = boxes.slice().sort((a, b) => a - b);
  if (solved(startBoxes)) return { pushes: 0, moves: '' };
  // String keys: a numeric base-size key overflows 2^53 with 16 boxes.
  const keyOf = (bs: number[], low: number): string => low + '|' + bs.join(',');
  const occ = new Uint8Array(size);
  const regionKey = makeRegion(wall, w, h, occ);
  const inRegion = new Uint8Array(size);
  const sBoxes: number[][] = [startBoxes];
  const sPlayer: number[] = [player];
  const sParent: number[] = [-1];
  const sPushFrom: number[] = [-1];
  const sPushTo: number[] = [-1];
  const sPushDir: number[] = [-1];
  const sH: number[] = [heuristic(startBoxes)];
  const closed = new Map<string, number>();
  for (const b of startBoxes) occ[b] = 1;
  closed.set(keyOf(startBoxes, regionKey(player)), 0);
  for (const b of startBoxes) occ[b] = 0;
  const heap: number[] = [0];
  const less = (a: number, b: number): boolean => (sH[a] !== sH[b] ? sH[a] < sH[b] : a < b);
  const heapPush = (s: number): void => {
    heap.push(s);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      const t = heap[i];
      heap[i] = heap[p];
      heap[p] = t;
      i = p;
    }
  };
  const heapPop = (): number => {
    const top = heap[0];
    const last = heap.pop() as number;
    if (heap.length > 0) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1;
        const r = l + 1;
        let m = i;
        if (l < heap.length && less(heap[l], heap[m])) m = l;
        if (r < heap.length && less(heap[r], heap[m])) m = r;
        if (m === i) break;
        const t = heap[i];
        heap[i] = heap[m];
        heap[m] = t;
        i = m;
      }
    }
    return top;
  };
  let expanded = 0;
  let found = -1;
  while (heap.length > 0) {
    const s = heapPop();
    if (solved(sBoxes[s])) {
      found = s;
      break;
    }
    if (expanded >= GREEDY_BUDGET) break;
    expanded++;
    const bs = sBoxes[s];
    for (const b of bs) occ[b] = 1;
    regionMask(wall, w, h, occ, sPlayer[s], inRegion);
    for (let bi = 0; bi < bs.length; bi++) {
      const b = bs[bi];
      for (let k = 0; k < 4; k++) {
        const d = DY[k] * w + DX[k];
        const t = b + d;
        if (t < 0 || t >= size) continue;
        if (inRegion[b - d] !== 1 || wall[t] || occ[t] || !live[t]) continue;
        occ[b] = 0;
        occ[t] = 1;
        const frozen = frozenBlock(wall, occ, goal, t, w);
        let key = '';
        let next: number[] | null = null;
        let nextH = 0;
        if (!frozen) {
          next = bs.slice();
          next[bi] = t;
          next.sort((x, y) => x - y);
          nextH = heuristic(next);
          if (nextH !== Infinity) key = keyOf(next, regionKey(b));
        }
        occ[t] = 0;
        occ[b] = 1;
        if (frozen || key === '') continue;
        if (closed.has(key)) continue;
        closed.set(key, 1);
        sBoxes.push(next);
        sPlayer.push(b);
        sParent.push(s);
        sPushFrom.push(b);
        sPushTo.push(t);
        sPushDir.push(k);
        sH.push(nextH);
        heapPush(sBoxes.length - 1);
      }
    }
    for (const b of bs) occ[b] = 0;
  }
  if (found < 0) return null;
  const pushes: { from: number; to: number; dir: number }[] = [];
  for (let s = found; s > 0; s = sParent[s]) {
    pushes.push({ from: sPushFrom[s], to: sPushTo[s], dir: sPushDir[s] });
  }
  pushes.reverse();
  let moves = '';
  let cur = player;
  const curBoxes = startBoxes.slice();
  for (const p of pushes) {
    const d = DY[p.dir] * w + DX[p.dir];
    const stand = p.from - d;
    for (const b of curBoxes) occ[b] = 1;
    const walk = walkTo(wall, occ, cur, stand, w, h);
    for (const b of curBoxes) occ[b] = 0;
    if (walk === null) return null;
    moves += walk + LETTER[p.dir];
    cur = p.from;
    curBoxes[curBoxes.indexOf(p.from)] = p.to;
  }
  return { pushes: pushes.length, moves };
}

function main(): void {
  for (const id of IDS) {
    const text = readFileSync(join(INPUT_DIR, id, 'level.txt'), 'utf8');
    const level = parseLevel(text);
    const solved = solveLevel(level) ?? solveWholeGreedy(level);
    const moves = solved === null ? '' : solved.moves;
    mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
    writeFileSync(join(OUTPUT_DIR, id, 'solution.txt'), moves + '\n', 'utf8');
  }
}

main();
