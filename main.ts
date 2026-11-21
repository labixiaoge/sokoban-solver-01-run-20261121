// Sokoban solver: push-optimal, room by room.
//
// Idea: in every level the generator builds, the board is one hallway row
// (where the player starts) plus one to four rooms off it, each room holding
// 3-4 boxes and as many goals, opening onto the hallway through a single
// door. A box pushed out through a door can never be pushed back, so every
// box stays in its room and the level's push-optimal count is the sum of its
// rooms' optima. This solver therefore splits the level into rooms, solves
// each room push-optimally on its own, and stitches the room solutions
// together with plain walks through the hallway.
//
// Each room is solved with A* over pushes (not moves): a state is the sorted
// box squares plus the player's reachable region, a step is one push, and
// the heuristic is the cheapest assignment of boxes to goals under
// shortest-path distances inside the room. Dead squares (found by reverse
// pull-reachability from the goals) and frozen 2x2 blocks prune the search.
// The search is bounded by work, never by the clock, and every tie-break
// is fixed, so two runs write the same bytes.
//
// Plain JavaScript in a .ts file (no TypeScript-only syntax), Node 22,
// no dependencies, no network, no clock reads.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Work bounds: deterministic, counted in expansions, never in seconds.
const ROOM_EXPANSION_CAP = 2000000;
const GLOBAL_EXPANSION_CAP = 2000000;

const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const LETTER = ['u', 'd', 'l', 'r'];

/** Parse XSB text into walls, goals, boxes and the player square. */
function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const wall = new Uint8Array(w * h).fill(1);
  const goals = new Set();
  const boxes = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.add(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

/**
 * Breadth-first walk from `start` over squares where `open(i)` is true.
 * Returns parent pointers (or null when the target is unreachable) so a
 * path can be rebuilt. Neighbour order is fixed: u, d, l, r.
 */
function bfsWalk(start, target, open, w, h) {
  if (start === target) return '';
  const prev = new Int32Array(w * h).fill(-2);
  const prevDir = new Int8Array(w * h).fill(-1);
  prev[start] = -1;
  const queue = [start];
  let head = 0;
  while (head < queue.length) {
    const cur = queue[head++];
    const cx = cur % w;
    const cy = (cur / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d];
      const ny = cy + DY[d];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const ni = ny * w + nx;
      if (prev[ni] !== -2 || !open(ni)) continue;
      prev[ni] = cur;
      prevDir[ni] = d;
      if (ni === target) {
        let moves = '';
        for (let t = ni; t !== start; t = prev[t]) moves = LETTER[prevDir[t]] + moves;
        return moves;
      }
      queue.push(ni);
    }
  }
  return null;
}

/** All squares reachable from `start` without crossing `blocked` squares. */
function reachableRegion(start, blocked, open, w, h) {
  const seen = new Set([start]);
  const queue = [start];
  let head = 0;
  while (head < queue.length) {
    const cur = queue[head++];
    const cx = cur % w;
    const cy = (cur / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d];
      const ny = cy + DY[d];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const ni = ny * w + nx;
      if (seen.has(ni) || blocked.has(ni) || !open(ni)) continue;
      seen.add(ni);
      queue.push(ni);
    }
  }
  return seen;
}

/** A binary heap with a fully deterministic order: (f, h, sequence). */
function makeHeap() {
  const a = [];
  const less = (x, y) =>
    x[0] < y[0] || (x[0] === y[0] && (x[1] < y[1] || (x[1] === y[1] && x[2] < y[2])));
  return {
    get size() {
      return a.length;
    },
    push(e) {
      let i = a.length;
      a.push(e);
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (!less(a[i], a[p])) break;
        const t = a[i];
        a[i] = a[p];
        a[p] = t;
        i = p;
      }
    },
    pop() {
      const top = a[0];
      const last = a.pop();
      if (a.length > 0) {
        a[0] = last;
        for (let i = 0; ; ) {
          let c = 2 * i + 1;
          if (c >= a.length) break;
          if (c + 1 < a.length && less(a[c + 1], a[c])) c++;
          if (!less(a[c], a[i])) break;
          const t = a[i];
          a[i] = a[c];
          a[c] = t;
          i = c;
        }
      }
      return top;
    },
  };
}

/**
 * Push-optimal A* for one closed area.
 *
 * boxCells: squares a box may ever stand on. walkCells: squares the player
 * may ever stand on (a superset: it also holds the door the player enters
 * through, which no box may enter). goals: the goal squares inside the
 * area. startBoxes: sorted starting box squares. startPlayer: the square
 * the player starts on. exitCell: when not null, a solved state must also
 * let the player walk back to this square (the door out).
 *
 * Returns the push list [{from, to, dir}] or null when no solution is
 * found inside the expansion cap.
 */
function pushOptimal(boxCells, walkCells, goals, startBoxes, startPlayer, exitCell, w, h, cap) {
  const boxSet = new Set(boxCells);
  const walkSet = new Set(walkCells);
  const goalSet = new Set(goals);
  const cellIndex = new Map(boxCells.map((c, i) => [c, i]));
  const n = boxCells.length;

  // Shortest box-travel distances inside the area, ignoring other boxes:
  // a lower bound on what any box still has to travel.
  const dist = [];
  for (const s of boxCells) {
    const row = new Int32Array(n).fill(-1);
    const si = cellIndex.get(s);
    row[si] = 0;
    const queue = [s];
    let head = 0;
    while (head < queue.length) {
      const cur = queue[head++];
      const ci = cellIndex.get(cur);
      const cx = cur % w;
      const cy = (cur / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = cx + DX[d];
        const ny = cy + DY[d];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const ni = ny * w + nx;
        if (!boxSet.has(ni)) continue;
        const niIdx = cellIndex.get(ni);
        if (row[niIdx] !== -1) continue;
        row[niIdx] = row[ci] + 1;
        queue.push(ni);
      }
    }
    dist.push(row);
  }

  // Dead squares: a box can be pulled from a square to a neighbour only
  // when the square behind it is open for the player, so the squares from
  // which some goal is reachable by such pulls are exactly the live ones.
  // Anything else can never hold a box that still gets solved.
  const live = new Set(goals);
  {
    const queue = [...goals];
    let head = 0;
    while (head < queue.length) {
      const cur = queue[head++];
      const cx = cur % w;
      const cy = (cur / w) | 0;
      for (let d = 0; d < 4; d++) {
        // A box on `prev` can be pushed to `cur` when the player can
        // stand on the square behind `prev`.
        const px = cx - DX[d];
        const py = cy - DY[d];
        const bx = cx - 2 * DX[d];
        const by = cy - 2 * DY[d];
        if (px < 0 || py < 0 || px >= w || py >= h) continue;
        if (bx < 0 || by < 0 || bx >= w || by >= h) continue;
        const prev = py * w + px;
        const stance = by * w + bx;
        if (!walkSet.has(stance) || !boxSet.has(prev) || live.has(prev)) continue;
        live.add(prev);
        queue.push(prev);
      }
    }
  }

  // Frozen 2x2 blocks: four squares, each wall or box, holding a box
  // that is not on a goal, can never move again.
  const blocks = [];
  for (let y = 0; y + 1 < h; y++) {
    for (let x = 0; x + 1 < w; x++) {
      const cells = [y * w + x, y * w + x + 1, (y + 1) * w + x, (y + 1) * w + x + 1];
      if (cells.every((c) => boxSet.has(c) || !walkSet.has(c))) blocks.push(cells);
    }
  }
  const frozen = (boxes) => {
    const boxSetLocal = new Set(boxes);
    for (const cells of blocks) {
      let boxCount = 0;
      let offGoal = false;
      for (const c of cells) {
        if (boxSetLocal.has(c)) {
          boxCount++;
          if (!goalSet.has(c)) offGoal = true;
        } else if (walkSet.has(c)) {
          return false; // an open square: not a frozen block
        }
      }
      if (boxCount > 0 && offGoal) return true;
    }
    return false;
  };

  // Cheapest box-to-goal assignment, by trying every bijection
  // (at most 4 boxes, so at most 24). Infinity when some box can never
  // reach its goal: the state is dead.
  const goalIdx = goals.map((g) => cellIndex.get(g));
  const heuristicCache = new Map();
  const heuristic = (boxes) => {
    const key = boxes.join(',');
    const cached = heuristicCache.get(key);
    if (cached !== undefined) return cached;
    let best = Infinity;
    const k = boxes.length;
    const used = new Array(k).fill(false);
    const assign = (i, sum) => {
      if (sum >= best) return;
      if (i === k) {
        if (sum < best) best = sum;
        return;
      }
      const bi = cellIndex.get(boxes[i]);
      for (let j = 0; j < k; j++) {
        if (used[j]) continue;
        const d = dist[bi][goalIdx[j]];
        if (d < 0) continue;
        used[j] = true;
        assign(i + 1, sum + d);
        used[j] = false;
      }
    };
    assign(0, 0);
    heuristicCache.set(key, best);
    return best;
  };

  const solved = (boxes) => boxes.every((b) => goalSet.has(b));
  const regionOf = (player, boxes) =>
    reachableRegion(player, new Set(boxes), (i) => walkSet.has(i), w, h);
  const regionRep = (region) => {
    let m = Infinity;
    for (const c of region) if (c < m) m = c;
    return m;
  };

  const startRegion = regionOf(startPlayer, startBoxes);
  if (solved(startBoxes) && (exitCell === null || startRegion.has(exitCell))) return [];
  if (startBoxes.some((b) => !live.has(b)) || frozen(startBoxes)) return null;

  const stateBoxes = [startBoxes.slice()];
  const statePlayer = [startPlayer];
  const stateG = [0];
  const stateParent = [-1];
  const statePush = [null];
  const closed = new Map();
  const startKey = startBoxes.join(',') + '|' + regionRep(startRegion);
  closed.set(startKey, 0);
  const heap = makeHeap();
  const h0 = heuristic(startBoxes);
  if (h0 === Infinity) return null;
  let seq = 0;
  heap.push([h0, h0, seq++, 0]);

  let expanded = 0;
  while (heap.size > 0) {
    const [, , , s] = heap.pop();
    const boxes = stateBoxes[s];
    const g = stateG[s];
    const region = regionOf(statePlayer[s], boxes);
    const key = boxes.join(',') + '|' + regionRep(region);
    if (closed.get(key) !== g) continue;
    if (solved(boxes)) {
      if (exitCell === null || region.has(exitCell)) {
        const pushes = [];
        for (let t = s; t > 0; t = stateParent[t]) pushes.push(statePush[t]);
        pushes.reverse();
        return pushes;
      }
    }
    if (++expanded > cap) return null;
    const boxSetLocal = new Set(boxes);
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi];
      const bx = b % w;
      const by = (b / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = bx + DX[d];
        const ny = by + DY[d];
        const sx = bx - DX[d];
        const sy = by - DY[d];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (sx < 0 || sy < 0 || sx >= w || sy >= h) continue;
        const to = ny * w + nx;
        const stance = sy * w + sx;
        if (!boxSet.has(to) || boxSetLocal.has(to)) continue;
        if (!region.has(stance)) continue;
        const nb = boxes.slice();
        nb[bi] = to;
        nb.sort((a, c) => a - c);
        if (nb.some((x) => !live.has(x)) || frozen(nb)) continue;
        const hh = heuristic(nb);
        if (hh === Infinity) continue;
        const nRegion = regionOf(b, nb);
        const nKey = nb.join(',') + '|' + regionRep(nRegion);
        const seen = closed.get(nKey);
        if (seen !== undefined && seen <= g + 1) continue;
        closed.set(nKey, g + 1);
        const t = stateBoxes.length;
        stateBoxes.push(nb);
        statePlayer.push(b);
        stateG.push(g + 1);
        stateParent.push(s);
        statePush.push({ from: b, to, dir: d });
        heap.push([g + 1 + hh, hh, seq++, t]);
      }
    }
  }
  return null;
}

/** Turn a push list into LURD moves, walking the player into place. */
function pushesToMoves(pushes, startPlayer, startBoxes, walkCells, w, h) {
  let moves = '';
  let player = startPlayer;
  let boxes = startBoxes.slice();
  const walkSet = new Set(walkCells);
  for (const p of pushes) {
    const stance = p.from - (p.to - p.from);
    const walk = bfsWalk(player, stance, (i) => walkSet.has(i) && !boxes.includes(i), w, h);
    if (walk === null) return null;
    moves += walk;
    moves += LETTER[p.dir];
    boxes = boxes.map((b) => (b === p.from ? p.to : b));
    boxes.sort((a, b) => a - b);
    player = p.from;
  }
  return { moves, player, boxes };
}

/** Split a level into its hallway and its rooms. Null when unexpected. */
function decompose(level) {
  const { w, h, wall, goals, boxes, player } = level;
  const hallRow = (player / w) | 0;
  const floor = (i) => !wall[i];
  // Every box and goal must sit in a room, never in the hallway.
  for (const b of boxes) if (((b / w) | 0) === hallRow) return null;
  for (const g of goals) if (((g / w) | 0) === hallRow) return null;

  const rest = new Set();
  for (let i = 0; i < w * h; i++) {
    if (floor(i) && ((i / w) | 0) !== hallRow) rest.add(i);
  }
  const seen = new Set();
  const rooms = [];
  for (const s of rest) {
    if (seen.has(s)) continue;
    const comp = [];
    const queue = [s];
    seen.add(s);
    while (queue.length) {
      const cur = queue.pop();
      comp.push(cur);
      const cx = cur % w;
      const cy = (cur / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = cx + DX[d];
        const ny = cy + DY[d];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const ni = ny * w + nx;
        if (rest.has(ni) && !seen.has(ni)) {
          seen.add(ni);
          queue.push(ni);
        }
      }
    }
    comp.sort((a, b) => a - b);
    const roomBoxes = boxes.filter((b) => comp.includes(b));
    const roomGoals = [...goals].filter((g) => comp.includes(g));
    if (roomBoxes.length === 0) continue;
    if (roomBoxes.length !== roomGoals.length) return null;
    if (roomBoxes.length > 4) return null;
    // The door: a hallway-row floor square touching the room.
    const compSet = new Set(comp);
    const doors = new Set();
    for (const c of comp) {
      const cx = c % w;
      const cy = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = cx + DX[d];
        const ny = cy + DY[d];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const ni = ny * w + nx;
        if (((ni / w) | 0) === hallRow && floor(ni) && !compSet.has(ni)) doors.add(ni);
      }
    }
    if (doors.size === 0) return null;
    rooms.push({ cells: comp, boxes: roomBoxes, goals: roomGoals, door: Math.min(...doors) });
  }
  // Every box must belong to exactly one room.
  if (rooms.reduce((a, r) => a + r.boxes.length, 0) !== boxes.length) return null;
  rooms.sort((a, b) => a.cells[0] - b.cells[0]);
  return { rooms, hallRow };
}

/** Solve one level; always returns a moves string (possibly empty). */
function solve(text) {
  const level = parseLevel(text);
  const { w, h, wall, goals, boxes, player } = level;
  const floor = (i) => !wall[i];

  const rooms = decompose(level);
  if (rooms !== null) {
    let moves = '';
    let curPlayer = player;
    let curBoxes = boxes.slice();
    let ok = true;
    for (const room of rooms.rooms) {
      // Walk through the hallway to this room's door.
      const toDoor = bfsWalk(curPlayer, room.door, (i) => floor(i) && !curBoxes.includes(i), w, h);
      if (toDoor === null) {
        ok = false;
        break;
      }
      moves += toDoor;
      curPlayer = room.door;
      const walkCells = [...room.cells, room.door];
      const pushes = pushOptimal(
        room.cells,
        walkCells,
        room.goals,
        room.boxes,
        room.door,
        room.door,
        w,
        h,
        ROOM_EXPANSION_CAP,
      );
      if (pushes === null) {
        ok = false;
        break;
      }
      const conv = pushesToMoves(pushes, room.door, room.boxes, walkCells, w, h);
      if (conv === null) {
        ok = false;
        break;
      }
      moves += conv.moves;
      // Walk back out through the door into the hallway.
      const out = bfsWalk(conv.player, room.door, (i) => walkCells.includes(i) && !conv.boxes.includes(i), w, h);
      if (out === null) {
        ok = false;
        break;
      }
      moves += out;
      curPlayer = room.door;
      curBoxes = curBoxes.filter((b) => !room.boxes.includes(b)).concat(room.goals);
      curBoxes.sort((a, b) => a - b);
    }
    if (ok) return moves;
  }

  // Fallback: one push-optimal search over the whole level.
  const pushes = pushOptimal(
    [...Array(w * h).keys()].filter((i) => floor(i)),
    [...Array(w * h).keys()].filter((i) => floor(i)),
    [...goals],
    boxes,
    player,
    null,
    w,
    h,
    GLOBAL_EXPANSION_CAP,
  );
  if (pushes === null) return '';
  const allFloor = [...Array(w * h).keys()].filter((i) => floor(i));
  const conv = pushesToMoves(pushes, player, boxes, allFloor, w, h);
  return conv === null ? '' : conv.moves;
}

for (const id of readdirSync(I).sort()) {
  let moves = '';
  try {
    moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  } catch {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
