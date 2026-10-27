// Fleece model and physics.
//
// The fleece is NOT a texture or a displaced skin. It is a separate volumetric
// body: a thick felted mat whose fibres (staples) stand on the skin. It is
// discretised as two particle layers sharing one triangulation:
//
//   tip layer   o-----o-----o      outer surface (weathered staple tips)
//               |\   /|\   /|      staple + crossed shear fibres
//               | \ / | \ / |      (keep the mat thick and resist shear)
//   base layer  o-----o-----o      the cut face / skin side (felted, cohesive)
//               ^     ^     ^
//              skin attachment (wool follicles) until the comb passes
//
// While attached, the columns are rigidly rooted to the (animated) skin. When
// the cutter passes a column it is released and becomes a dynamic body coupled
// to its neighbours by elastic in-plane links. The mat therefore keeps its bulk,
// hinges at the cut line, peels back under gravity and the push of the
// handpiece, drapes over the unshorn wool and the floor, opens its staple
// cracks when stretched over a fold, and can tear if it is overloaded.
//
// Solver: small-step XPBD (Macklin et al. 2019) with distance constraints,
// collision against a precomputed sheep SDF (plus the surface of the still
// unshorn fleece), the floor, the handpiece and itself.

import { sampleGrid, surfaceNets, projectToSurface, relax, compact, dropSmallComponents, adjacency } from './mesher.js';
import { sheepSDF, sdfGrad, stapleLength, BOUNDS } from './anatomy.js';
import { computeWeights, computeBreath } from './rig.js';
import { classify } from './director.js';

const T_INNER = 0, T_OUTER = 1, T_STAPLE = 2, T_SHEAR = 3, T_TETHER = 4;
const SELF_R = 0.02;       // self-contact distance between folds (closes the gaps in a 2.6 cm lattice)
const SELF_INNER = 2;      // virtual contact points along each staple between root and tip
// forward half of a 3x3x3 cell neighbourhood (entry 0 is the cell itself)
const HALF_NBR = [0, 0, 0];
for (let u = -1; u <= 1; u++) for (let v = -1; v <= 1; v++) for (let w = -1; w <= 1; w++) {
  if (u > 0 || (u === 0 && (v > 0 || (v === 0 && w > 0)))) HALF_NBR.push(u, v, w);
}
const SELF_MARGIN = 0.012; // pair search reach beyond SELF_R (motion within one frame)

export const DEFAULT_PHYSICS = {
  substeps: 8,
  gravity: 9.81,
  density: 50,          // kg/m^3 of greasy fleece in situ (gives a ~4.5 kg Merino fleece)
  innerCompliance: 1e-3, // felted base layer (cut face) in stretch - m/N (soft enough to drape, no shape memory)
  baseBuckle: 40,        // base layer is this much softer in compression (it bunches and crumples)
  outerCompliance: 2e-3, // tips crowding together (compression only)
  tipCohesion: 5e-2,     // tips pulling apart: barely resisted, staples part
  stapleCompliance: 2e-5,
  tetherCompliance: 1e-3, // felting tethers (second-ring, stretch only)
  shearCompliance: 2e-1, // staple uprightness: tip-to-neighbour-root links sized from the CURRENT root spacing
  baseCreep: 0.03,       // felted base slowly takes the shape it is held in
  edgeSplay: 10,         // shear-fibre compliance multiplier along free edges (edge locks flop outward)
  hingeSlip: 30,         // compliance multiplier for links from cut to rooted wool
  mould: 0.6,            // handpiece rolls the lifted slice back over its hinge (mouldboard)
  tilt: 0.7,             // handpiece tilt: open-side edge of the slice is lifted higher
  damping: 3.0,          // 1/s - fibre friction makes the mat very dead
  friction: 0.45,
  floorFriction: 0.8,
  tearRatio: 2.3,        // stretch ratio at which the felted base tears (sustained)
  wedgeLength: 0.3,      // comb + handpiece + hand, m
  wedgeWidth: 0.1,
  wedgeHeight: 0.1,
  wedgeSlope: 0.4,
  wedgeDrag: 0.3,
  yieldStrain: 0.08,     // fibre mat creeps (stays folded) beyond this strain
  creep: 0.1,            // fraction of excess strain made permanent per frame (tips)
  selfCollision: true,
  selfFriction: 0.5,     // friction between folds of wool in contact (position-based, static up to mu x push)
  holdFleece: false,
};

function hash3(x, y, z) {
  let h = Math.sin(x * 127.1 + y * 311.7 + z * 74.7) * 43758.5453;
  return h - Math.floor(h);
}
function vnoise(x, y, z) {
  const ix = Math.floor(x), iy = Math.floor(y), iz = Math.floor(z);
  const fx = x - ix, fy = y - iy, fz = z - iz;
  const sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy), sz = fz * fz * (3 - 2 * fz);
  let r = 0;
  for (let c = 0; c < 8; c++) {
    const dx = c & 1, dy = (c >> 1) & 1, dz = (c >> 2) & 1;
    r += hash3(ix + dx, iy + dy, iz + dz) * (dx ? sx : 1 - sx) * (dy ? sy : 1 - sy) * (dz ? sz : 1 - sz);
  }
  return r;
}

/** Regular grid storing the sheep SDF (for collisions) and nearest fleece column. */
class CollisionGrid {
  constructor(h) {
    const pad = 0.2;
    const g = sampleGrid(sheepSDF, BOUNDS.min.map((v) => v - pad), BOUNDS.max.map((v) => v + pad), h, true);
    Object.assign(this, g);
    this.nearest = new Int32Array(g.nx * g.ny * g.nz).fill(-1);
  }
  buildNearest(basePos, N, maxDist) {
    const { nx, ny, nz, min, h } = this;
    const cs = maxDist;
    this.nearest.fill(-1);
    const hash = new Map();
    const key = (a, b, c) => (a * 73856093) ^ (b * 19349663) ^ (c * 83492791);
    for (let i = 0; i < N; i++) {
      const k = key(Math.floor(basePos[3 * i] / cs), Math.floor(basePos[3 * i + 1] / cs), Math.floor(basePos[3 * i + 2] / cs));
      let arr = hash.get(k); if (!arr) hash.set(k, arr = []); arr.push(i);
    }
    for (let k = 0; k < nz; k++) for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
      const id = i + nx * (j + ny * k);
      const d = this.data[id];
      if (d < -0.02 || d > 0.16) continue;
      const x = min[0] + i * h, y = min[1] + j * h, z = min[2] + k * h;
      const cx = Math.floor(x / cs), cy = Math.floor(y / cs), cz = Math.floor(z / cs);
      let best = -1, bd = 1e9;
      for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) for (let c = -1; c <= 1; c++) {
        const arr = hash.get(key(cx + a, cy + b, cz + c));
        if (!arr) continue;
        for (const q of arr) {
          const dx = basePos[3 * q] - x, dy = basePos[3 * q + 1] - y, dz = basePos[3 * q + 2] - z;
          const dd = dx * dx + dy * dy + dz * dz;
          if (dd < bd) { bd = dd; best = q; }
        }
      }
      if (bd < maxDist * maxDist * 4) this.nearest[id] = best;
    }
  }
}

export class Fleece {
  /**
   * @param {object} opts { resolution (m), physics, rig }
   */
  constructor(opts) {
    this.params = Object.assign({}, DEFAULT_PHYSICS, opts.physics || {});
    this.rig = opts.rig;
    this.floorY = opts.floorY ?? -0.95;
    // simulate:false builds a lightweight mirror (topology + attachment state only)
    this.simulated = opts.simulate !== false;
    this._killLog = [];
    this._wakeVer = 0;          // bumped whenever wool wakes or is cut free (self-contact pairs go stale)
    this._buildMesh(opts.resolution || 0.026, opts.seed || 1);
    this._buildParticles();
    this._buildConstraints();
    this._buildPartings();
    if (this.simulated) {
      this.collisionGrid = opts.collisionGrid || new CollisionGrid(0.018);
      this.collisionGrid.buildNearest(this.restLocal, this.N, 0.05);
    }
    this.clipper = null;
    this.stats = { cut: 0, tears: 0, recuts: 0 };
    this.topologyVersion = 0;
    this.activeDirty = true;
  }

  // ------------------------------------------------------------------ build
  _buildMesh(h, seed) {
    const g = sampleGrid(sheepSDF, BOUNDS.min.map((v) => v - h), BOUNDS.max.map((v) => v + h), h, true);
    let m = surfaceNets(g);
    projectToSurface(m.positions, sheepSDF, 3);
    m = compact(m.positions, m.indices);
    relax(m.positions, m.indices, sheepSDF, 4, 0.5);
    // wool growth map
    const n = m.positions.length / 3;
    const keep = new Uint8Array(n), Lv = new Float32Array(n), nrm = new Float32Array(3 * n), g3 = [0, 0, 0];
    for (let i = 0; i < n; i++) {
      const x = m.positions[3 * i], y = m.positions[3 * i + 1], z = m.positions[3 * i + 2];
      sdfGrad(x, y, z, g3);
      nrm.set(g3, 3 * i);
      let L = stapleLength(x, y, z, g3[0], g3[1], g3[2]);
      // natural variation between staples / locks
      L *= 0.9 + 0.2 * vnoise(x * 18 + seed * 7.1, y * 18, z * 18);
      Lv[i] = L;
      keep[i] = L > 0.014 ? 1 : 0;
    }
    let c = compact(m.positions, m.indices, keep);
    const remapA = c.remap;
    c = dropSmallComponents(c.positions, c.indices, 25);
    // carry L and normals through both remaps
    const N = c.positions.length / 3;
    this.N = N;
    this.L = new Float32Array(N);
    this.restN = new Float32Array(3 * N);
    for (let i = 0; i < n; i++) {
      const a = remapA[i]; if (a < 0) continue;
      const b = c.remap[a]; if (b < 0) continue;
      this.L[b] = Lv[i];
      this.restN.set(nrm.subarray(3 * i, 3 * i + 3), 3 * b);
    }
    this.basePos = c.positions;
    this.tris = new Int32Array(c.indices);
    this.T = this.tris.length / 3;
    // smooth L a little over the mesh so the mat has no steps
    const adj = adjacency(N, this.tris);
    this.adj = adj;
    for (let it = 0; it < 2; it++) {
      const L2 = new Float32Array(N);
      for (let i = 0; i < N; i++) {
        let s = this.L[i], k = 1;
        for (let q = adj.start[i]; q < adj.start[i + 1]; q++) { s += this.L[adj.list[q]]; k++; }
        L2[i] = s / k;
      }
      this.L = L2;
    }
    // boundary columns taper (fleece edge thins out naturally)
    const boundary = this._boundaryVertices();
    for (let i = 0; i < N; i++) if (boundary[i]) this.L[i] *= 0.75;
  }

  _boundaryVertices() {
    const count = new Map();
    const T = this.tris;
    for (let t = 0; t < T.length; t += 3) for (let e = 0; e < 3; e++) {
      const a = T[t + e], b = T[t + (e + 1) % 3];
      const k = a < b ? a * 1e6 + b : b * 1e6 + a;
      count.set(k, (count.get(k) || 0) + 1);
    }
    const out = new Uint8Array(this.N);
    for (const [k, c] of count) if (c === 1) { out[Math.floor(k / 1e6)] = 1; out[k % 1e6] = 1; }
    return out;
  }

  _buildParticles() {
    const N = this.N, P = 2 * N;
    this.P = P;
    this.restLocal = new Float32Array(3 * P);
    this.restLocal.set(this.basePos, 0);
    // tips stand along the (outward, slightly gravity-combed) skin normal
    for (let i = 0; i < N; i++) {
      const L = this.L[i];
      let nx = this.restN[3 * i], ny = this.restN[3 * i + 1], nz = this.restN[3 * i + 2];
      // staples lean slightly downward and backward (wool "lies" on the body)
      nx -= 0.08; ny -= 0.06;
      const l = Math.hypot(nx, ny, nz); nx /= l; ny /= l; nz /= l;
      this.restLocal[3 * (N + i)] = this.basePos[3 * i] + nx * L;
      this.restLocal[3 * (N + i) + 1] = this.basePos[3 * i + 1] + ny * L;
      this.restLocal[3 * (N + i) + 2] = this.basePos[3 * i + 2] + nz * L;
    }
    // mass from column volume (1/3 of incident triangle areas x staple length)
    const area = new Float32Array(N);
    const p = this.basePos, T = this.tris;
    for (let t = 0; t < T.length; t += 3) {
      const a = 3 * T[t], b = 3 * T[t + 1], c = 3 * T[t + 2];
      const ux = p[b] - p[a], uy = p[b + 1] - p[a + 1], uz = p[b + 2] - p[a + 2];
      const vx = p[c] - p[a], vy = p[c + 1] - p[a + 1], vz = p[c + 2] - p[a + 2];
      const A = 0.5 * Math.hypot(uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
      area[T[t]] += A / 3; area[T[t + 1]] += A / 3; area[T[t + 2]] += A / 3;
    }
    this.mass = new Float32Array(P);
    let total = 0;
    for (let i = 0; i < N; i++) {
      const m = Math.max(2e-4, area[i] * this.L[i] * this.params.density);
      this.mass[i] = 0.55 * m; this.mass[N + i] = 0.45 * m;
      total += m;
    }
    this.totalMass = total;
    this.invMass = new Float32Array(P);  // 0 for kinematic particles
    this.dyn = new Uint8Array(P);
    this.attached = new Uint8Array(N).fill(1);
    this.pos = new Float32Array(3 * P);
    this.prev = new Float32Array(3 * P);
    this.vel = new Float32Array(3 * P);
    this.target = new Float32Array(3 * P);   // world-space anchors for kinematic particles
    this.anchorLocal = new Float32Array(3 * P);
    this.weights = computeWeights(this.basePos);
    // tips use the weights of their roots
    const w2 = { idx: new Uint8Array(3 * P), w: new Float32Array(3 * P) };
    w2.idx.set(this.weights.idx); w2.idx.set(this.weights.idx, 3 * N);
    w2.w.set(this.weights.w); w2.w.set(this.weights.w, 3 * N);
    this.weights = w2;
    const br = computeBreath(this.basePos);
    this.breathVec = new Float32Array(3 * P);
    this.breathVec.set(br); this.breathVec.set(br, 3 * N);
    this.strain = new Float32Array(P).fill(1);
    this.dynList = new Int32Array(P);   // awake dynamic particles
    this.nDyn = 0;
    this.kinList = new Int32Array(P);   // rooted particles (follow the skin)
    this.nKin = P;
    for (let i = 0; i < P; i++) this.kinList[i] = i;
    // sleeping: wool at rest (on the board, draped) stops being simulated until disturbed
    this.asleep = new Uint8Array(P);
    this.still = new Uint8Array(P);
    this.nAsleep = 0;
  }

  _buildConstraints() {
    const N = this.N, T = this.tris;
    const edgeMap = new Map();
    const eA = [], eB = [], eT0 = [], eT1 = [];
    const triEdges = new Int32Array(T.length);
    for (let t = 0; t < T.length / 3; t++) for (let e = 0; e < 3; e++) {
      const a = T[3 * t + e], b = T[3 * t + (e + 1) % 3];
      const k = a < b ? a * 1e6 + b : b * 1e6 + a;
      let id = edgeMap.get(k);
      if (id === undefined) { id = eA.length; edgeMap.set(k, id); eA.push(a); eB.push(b); eT0.push(t); eT1.push(-1); }
      else eT1[id] = t;
      triEdges[3 * t + e] = id;
    }
    const E = eA.length;
    this.E = E;
    this.eA = Int32Array.from(eA); this.eB = Int32Array.from(eB);
    this.eTri = new Int32Array(2 * E);
    for (let e = 0; e < E; e++) { this.eTri[2 * e] = eT0[e]; this.eTri[2 * e + 1] = eT1[e]; }
    this.triEdges = triEdges;
    this.triAlive = new Uint8Array(T.length / 3).fill(1);
    this.eLive = new Uint8Array(E);
    for (let e = 0; e < E; e++) this.eLive[e] = eT1[e] >= 0 ? 2 : 1;
    this.eOver = new Uint8Array(E);

    // Felting: fibres (~9 cm) are much longer than the lattice spacing, so each
    // column is entangled with its second ring of neighbours. Stretch-only
    // tethers (also what keeps a large hanging sheet from going rubbery).
    const tethers = [];
    const edgeId = (a, b) => edgeMap.get(a < b ? a * 1e6 + b : b * 1e6 + a);
    const adj = this.adj;
    for (let i = 0; i < N; i++) {
      const seen = new Map();
      for (let q = adj.start[i]; q < adj.start[i + 1]; q++) {
        const k = adj.list[q];
        for (let r = adj.start[k]; r < adj.start[k + 1]; r++) {
          const j = adj.list[r];
          if (j <= i || seen.has(j)) continue;
          let direct = false;
          for (let z = adj.start[i]; z < adj.start[i + 1]; z++) if (adj.list[z] === j) { direct = true; break; }
          if (direct) continue;
          seen.set(j, k);
        }
      }
      for (const [j, k] of seen) tethers.push(i, j, edgeId(i, k), edgeId(k, j));
    }
    const nTe = tethers.length / 4;
    this.nTethers = nTe;

    // constraints: per edge inner, outer, 2 shear; per column one staple; tethers
    const M = 4 * E + N + nTe;
    this.M = M;
    this.cA = new Int32Array(M); this.cB = new Int32Array(M);
    this.cRest = new Float32Array(M); this.cType = new Uint8Array(M); this.cEdge = new Int32Array(M);
    this.cAlive = new Uint8Array(M).fill(1);
    let c = 0;
    const R = this.restLocal;
    const add = (a, b, type, edge) => {
      this.cA[c] = a; this.cB[c] = b; this.cType[c] = type; this.cEdge[c] = edge;
      this.cRest[c] = Math.hypot(R[3 * a] - R[3 * b], R[3 * a + 1] - R[3 * b + 1], R[3 * a + 2] - R[3 * b + 2]);
      c++;
    };
    for (let e = 0; e < E; e++) {
      const a = this.eA[e], b = this.eB[e];
      add(a, b, T_INNER, e);
      add(N + a, N + b, T_OUTER, e);
      // staples are about as wide as their root spacing: tips crowd only when
      // packed closer than that (no memory of the splay they had on the sheep)
      this.cRest[c - 1] = this.cRest[c - 2] * 0.92;
      add(a, N + b, T_SHEAR, e);
      add(b, N + a, T_SHEAR, e);
    }
    for (let i = 0; i < N; i++) add(i, N + i, T_STAPLE, -1);
    this.tetherStart = c;
    // edge -> tethers that run through it (a tether dies with either of its edges)
    const cnt = new Int32Array(E + 1);
    for (let t = 0; t < nTe; t++) { cnt[tethers[4 * t + 2] + 1]++; cnt[tethers[4 * t + 3] + 1]++; }
    for (let e = 0; e < E; e++) cnt[e + 1] += cnt[e];
    this.eTetherStart = cnt.slice();
    this.eTethers = new Int32Array(2 * nTe);
    const fillp = cnt.slice();
    for (let t = 0; t < nTe; t++) {
      add(tethers[4 * t], tethers[4 * t + 1], T_TETHER, -1);
      this.eTethers[fillp[tethers[4 * t + 2]]++] = this.tetherStart + t;
      this.eTethers[fillp[tethers[4 * t + 3]]++] = this.tetherStart + t;
    }
    this.cRest0 = this.cRest.slice();
    this.eWeak = new Float32Array(E).fill(1);
    this.active = new Int32Array(M);
    this.nActive = 0;
    this.edgeConstraintStart = 0; // constraint 4e..4e+3 belong to edge e
  }

  // Belly wool (with the inside of the legs) is shorter and finer than the main
  // fleece. It is only classified here: nothing is pre-cut, the fleece starts as a
  // closed sleeve and comes apart only where the clippers or a tear open it.
  _buildPartings() {
    const N = this.N, p = this.basePos, n = this.restN;
    this.bellyPiece = new Uint8Array(N);
    for (let i = 0; i < N; i++) {
      const x = p[3 * i], y = p[3 * i + 1], z = p[3 * i + 2];
      const r = classify(x, y, z, n[3 * i], n[3 * i + 1], n[3 * i + 2]);
      const medial = n[3 * i + 2] * Math.sign(z) < -0.25;
      const leg = r.includes('leg');
      this.bellyPiece[i] = r === 'Belly' || (leg && medial && y < -0.1) ? 1 : 0;
    }
    // blow that released each column (-1 while rooted): the cutter only cuts through
    // wool shorn by an earlier blow, never the slice it is lifting right now
    this.releaseBlow = new Int32Array(N).fill(-1);
    // mesh edges around each vertex (CSR), for finding the edges the cutter crosses
    const cnt = new Int32Array(N + 1);
    for (let e = 0; e < this.E; e++) { cnt[this.eA[e] + 1]++; cnt[this.eB[e] + 1]++; }
    for (let i = 0; i < N; i++) cnt[i + 1] += cnt[i];
    this.vEdgeStart = cnt.slice();
    this.vEdges = new Int32Array(2 * this.E);
    const fill = cnt.slice();
    for (let e = 0; e < this.E; e++) { this.vEdges[fill[this.eA[e]]++] = e; this.vEdges[fill[this.eB[e]]++] = e; }
  }

  /** The cutter went through the mat across edge e: part the fleece there. */
  cutEdge(e) {
    if (!this.eLive[e]) return false;
    for (const t of [this.eTri[2 * e], this.eTri[2 * e + 1]]) if (t >= 0) this._killTri(t);
    this.stats.recuts++;
    this.activeDirty = true;
    return true;
  }

  _killTri(t) {
    if (!this.triAlive[t]) return;
    this.triAlive[t] = 0;
    this._killLog.push(t);
    this.topologyVersion++;
    for (let k = 0; k < 3; k++) {
      const ed = this.triEdges[3 * t + k];
      if (--this.eLive[ed] === 0) {
        for (let j = 0; j < 4; j++) this.cAlive[4 * ed + j] = 0;
        for (let q = this.eTetherStart[ed]; q < this.eTetherStart[ed + 1]; q++) this.cAlive[this.eTethers[q]] = 0;
      }
    }
  }

  // ------------------------------------------------------------- per frame
  /** Compute anchor targets for every particle from the rig + sheep transform. */
  updateAnchors(R, t) {
    this.rig.skin(this.restLocal, this.weights, this.breathVec, this.anchorLocal);
    const a = this.anchorLocal, out = this.target;
    for (let i = 0; i < this.P; i++) {
      const x = a[3 * i], y = a[3 * i + 1], z = a[3 * i + 2];
      out[3 * i] = R[0] * x + R[1] * y + R[2] * z + t[0];
      out[3 * i + 1] = R[3] * x + R[4] * y + R[5] * z + t[1];
      out[3 * i + 2] = R[6] * x + R[7] * y + R[8] * z + t[2];
    }
    this.R = R; this.t = t;
  }

  /** Snap everything to anchors (initial state). */
  reset() {
    this.pos.set(this.target);
    this.prev.set(this.target);
    this.vel.fill(0);
  }

  /** Release the column rooted at base vertex i (the cutter severed its fibres). */
  release(i, vx = 0, vy = 0, vz = 0, blow = 0) {
    if (!this.attached[i]) return false;
    this.attached[i] = 0;
    this.releaseBlow[i] = blow;
    const N = this.N;
    for (const p of [i, N + i]) {
      this.dyn[p] = 1;
      this.asleep[p] = 0; this.still[p] = 0;
      this.invMass[p] = 1 / this.mass[p];
      this.vel[3 * p] = vx; this.vel[3 * p + 1] = vy; this.vel[3 * p + 2] = vz;
    }
    this.stats.cut++;
    this._wakeVer = (this._wakeVer || 0) + 1;
    this.activeDirty = true;
    if (this.nAsleep) this.wakeNear(this.pos[3 * i], this.pos[3 * i + 1], this.pos[3 * i + 2], 0.3);
    return true;
  }

  _wake(p) {
    if (!this.asleep[p]) return;
    this.stats.wakes = (this.stats.wakes || 0) + 1;
    this._wakeVer = (this._wakeVer || 0) + 1;
    this.asleep[p] = 0; this.still[p] = 0; this.nAsleep--;
    this.invMass[p] = 1 / this.mass[p];
    this.vel[3 * p] = this.vel[3 * p + 1] = this.vel[3 * p + 2] = 0;
    this.activeDirty = true;
  }

  wakeNear(x, y, z, r) {
    if (!this.nAsleep) return;
    const r2 = r * r, pos = this.pos;
    for (let p = 0; p < this.P; p++) {
      if (!this.asleep[p]) continue;
      const dx = pos[3 * p] - x, dy = pos[3 * p + 1] - y, dz = pos[3 * p + 2] - z;
      if (dx * dx + dy * dy + dz * dz < r2) this._wake(p);
    }
  }

  wakeAll() { if (this.nAsleep) for (let p = 0; p < this.P; p++) this._wake(p); }

  /** Update tunable parameters; mass scale is applied relative to the build density. */
  applyParams(p) {
    const oldDensity = this.params.density;
    Object.assign(this.params, p);
    if (p.density && p.density !== oldDensity) {
      const k = p.density / oldDensity;
      this.totalMass *= k;
      for (let i = 0; i < this.P; i++) {
        this.mass[i] *= k;
        if (this.dyn[i] && !this.asleep[i]) this.invMass[i] = 1 / this.mass[i];
      }
    }
  }

  /** Release every column (fleece fully off). */
  releaseAll() { for (let i = 0; i < this.N; i++) this.release(i); }

  _rebuildActive() {
    let n = 0;
    const { cA, cB, cAlive, dyn, asleep } = this;
    const noShear = !(this.params.shearCompliance > 0);
    const cType = this.cType;
    for (let c = 0; c < this.M; c++) {
      if (!cAlive[c]) continue;
      if (noShear && cType[c] === T_SHEAR) continue;
      const a = cA[c], b = cB[c];
      if ((dyn[a] && !asleep[a]) || (dyn[b] && !asleep[b])) this.active[n++] = c;
    }
    this.nActive = n;
    let d = 0, k = 0;
    for (let p = 0; p < this.P; p++) {
      if (!dyn[p]) this.kinList[k++] = p;
      else if (!asleep[p]) this.dynList[d++] = p;
    }
    this.nDyn = d; this.nKin = k;
    this.activeDirty = false;
    this._activeVer = (this._activeVer || 0) + 1;
  }

  step(dt) {
    // a moving sheep (repositioning) disturbs everything; the handpiece disturbs its surroundings
    if (this.nAsleep) {
      const R = this.R, t = this.t, L = this._lastRT;
      if (L) {
        let d = 0;
        for (let k = 0; k < 9; k++) d = Math.max(d, Math.abs(R[k] - L[k]));
        for (let k = 0; k < 3; k++) d = Math.max(d, Math.abs(t[k] - L[9 + k]) * 0.5);
        if (d > 2e-4) this.wakeAll();
      }
      const cl = this.clipper;
      if (cl && cl.active) this.wakeNear(cl.px, cl.py, cl.pz, 0.35);
    }
    this._openSide();
    this._lastRT = [...this.R, ...this.t];
    if (this.activeDirty) this._rebuildActive();
    const P = this.P, pos = this.pos, prev = this.prev, vel = this.vel, im = this.invMass, dyn = this.dyn, tgt = this.target;
    const ns = this.params.substeps, h = dt / ns, g = this.params.gravity;
    // kinematic particles: interpolate from current to target across substeps
    const start = this._kinStart || (this._kinStart = new Float32Array(3 * P));
    const nDyn = this.nDyn, dl = this.dynList, nKin = this.nKin, kl = this.kinList;
    for (let q = 0; q < nKin; q++) { const i3 = 3 * kl[q]; start[i3] = pos[i3]; start[i3 + 1] = pos[i3 + 1]; start[i3 + 2] = pos[i3 + 2]; }
    for (let q = 0; q < nDyn; q++) { const i3 = 3 * dl[q]; start[i3] = pos[i3]; start[i3 + 1] = pos[i3 + 1]; start[i3 + 2] = pos[i3 + 2]; }
    const alpha = [
      this.params.innerCompliance / (h * h),
      this.params.outerCompliance / (h * h),
      this.params.stapleCompliance / (h * h),
      this.params.shearCompliance / (h * h),
      this.params.tetherCompliance / (h * h),
      this.params.tipCohesion / (h * h),
    ];
    const damp = Math.max(0, 1 - this.params.damping * h);
    // the shearer holding the fleece on the board (end of shear): no horizontal drift
    const hold = this.params.holdFleece ? 0.6 : 0;
    const maxV = 6;
    const selfCol = this.params.selfCollision && nDyn > 0;
    if (selfCol && this._selfPairsStale()) this._findSelfPairs();
    for (let s = 1; s <= ns; s++) {
      const f = s / ns;
      // kinematic
      for (let q = 0; q < nKin; q++) {
        const i3 = 3 * kl[q];
        pos[i3] = start[i3] + (tgt[i3] - start[i3]) * f;
        pos[i3 + 1] = start[i3 + 1] + (tgt[i3 + 1] - start[i3 + 1]) * f;
        pos[i3 + 2] = start[i3 + 2] + (tgt[i3 + 2] - start[i3 + 2]) * f;
      }
      // predict
      for (let q = 0; q < nDyn; q++) {
        const p = dl[q], i3 = 3 * p;
        if (s === 1) { pos[i3] = start[i3]; pos[i3 + 1] = start[i3 + 1]; pos[i3 + 2] = start[i3 + 2]; }
        else { pos[i3] = prev[i3]; pos[i3 + 1] = prev[i3 + 1]; pos[i3 + 2] = prev[i3 + 2]; }
        prev[i3] = pos[i3]; prev[i3 + 1] = pos[i3 + 1]; prev[i3 + 2] = pos[i3 + 2];
        vel[i3 + 1] -= g * h;
        pos[i3] += vel[i3] * h; pos[i3 + 1] += vel[i3 + 1] * h; pos[i3 + 2] += vel[i3 + 2] * h;
      }
      this._solveConstraints(alpha);
      if (selfCol) this._solveSelfPairs();
      this._collide(h);
      // velocities (prev holds the substep start; store end positions back into prev for next substep)
      for (let q = 0; q < nDyn; q++) {
        const p = dl[q], i3 = 3 * p;
        let vx = (pos[i3] - prev[i3]) / h * damp, vy = (pos[i3 + 1] - prev[i3 + 1]) / h * damp, vz = (pos[i3 + 2] - prev[i3 + 2]) / h * damp;
        if (hold) { vx *= hold; vz *= hold; }
        const sp = Math.hypot(vx, vy, vz);
        if (sp > maxV) { vx *= maxV / sp; vy *= maxV / sp; vz *= maxV / sp; }
        vel[i3] = vx; vel[i3 + 1] = vy; vel[i3 + 2] = vz;
        prev[i3] = pos[i3]; prev[i3 + 1] = pos[i3 + 1]; prev[i3 + 2] = pos[i3 + 2];
      }
    }
    this._checkTears();
    this._creep();
    this._sleep();
  }

  // Which side of the comb is already open (fewer rooted columns)? The slice is
  // turned over toward that side, like a plough's mouldboard throws the furrow.
  _openSide() {
    const cl = this.clipper;
    if (!cl || !cl.active) return;
    const tg = this.target, att = this.attached;
    let left = 0, right = 0;
    for (let i = 0; i < this.N; i++) {
      if (!att[i]) continue;
      const dx = tg[3 * i] - cl.px, dy = tg[3 * i + 1] - cl.py, dz = tg[3 * i + 2] - cl.pz;
      const u = dx * cl.Zx + dy * cl.Zy + dz * cl.Zz;
      if (u > 0.04 || u < -0.2) continue;
      const w = dx * cl.Yx + dy * cl.Yy + dz * cl.Yz;
      if (w > 0.05 || w < -0.05) continue;
      const v = dx * cl.Xx + dy * cl.Xy + dz * cl.Xz;
      if (v > 0.045 && v < 0.15) right++;
      else if (v < -0.045 && v > -0.15) left++;
    }
    // smooth so the throw direction doesn't flicker
    const target = left === right ? 0 : (left > right ? 1 : -1);
    this._side = (this._side || 0) * 0.8 + target * 0.2;
  }

  // Put resting wool to sleep; wake sleepers that an awake neighbour is pulling on.
  // "Pulling" is measured against the link's length when it first joined a sleeper
  // to an awake particle, not its rest length: soft wool lying still is stretched
  // by its own weight, and must not keep waking itself.
  _sleep() {
    const { dynList, nDyn, vel, still, asleep, active, nActive, cA, cB, pos } = this;
    const ref = this._sleepRef || (this._sleepRef = new Float32Array(this.M));
    for (let q = 0; q < nActive; q++) {
      const c = active[q], a = cA[c], b = cB[c];
      if (asleep[a] === asleep[b]) { ref[c] = 0; continue; }
      const a3 = 3 * a, b3 = 3 * b;
      const len = Math.hypot(pos[b3] - pos[a3], pos[b3 + 1] - pos[a3 + 1], pos[b3 + 2] - pos[a3 + 2]);
      if (!ref[c]) { ref[c] = len; continue; }
      if (Math.abs(len - ref[c]) > 0.006) { ref[c] = 0; this._wake(asleep[a] ? a : b); }
    }
    for (let q = 0; q < nDyn; q++) {
      const p = dynList[q], i3 = 3 * p;
      if (asleep[p]) continue;
      const v2 = vel[i3] * vel[i3] + vel[i3 + 1] * vel[i3 + 1] + vel[i3 + 2] * vel[i3 + 2];
      if (v2 > 0.0004) { still[p] = 0; continue; }
      if (++still[p] > 40) {
        asleep[p] = 1; this.nAsleep++;
        this.invMass[p] = 0;
        vel[i3] = vel[i3 + 1] = vel[i3 + 2] = 0;
        this.activeDirty = true;
      }
    }
  }

  // Viscoplastic fibre mat: bending (outer layer) and shear fibres take a set
  // when strained past yield, so a fleece rolled back stays rolled back.
  _creep() {
    const { active, nActive, cA, cB, cRest, cRest0, cType, pos, dyn } = this;
    const y = this.params.yieldStrain, kTip = this.params.creep, kBase = this.params.baseCreep;
    for (let q = 0; q < nActive; q++) {
      const c = active[q];
      const type = cType[c];
      if (type !== T_OUTER && type !== T_INNER) continue;
      const k = type === T_INNER ? kBase : kTip;
      const a = cA[c], b = cB[c];
      if (!dyn[a] || !dyn[b]) continue;
      const a3 = 3 * a, b3 = 3 * b;
      const len = Math.hypot(pos[b3] - pos[a3], pos[b3 + 1] - pos[a3 + 1], pos[b3 + 2] - pos[a3 + 2]);
      const r = cRest[c], hi = r * (1 + y), lo = r * (1 - y);
      let nr = r;
      if (len > hi) nr = r + (len - hi) * k;
      else if (len < lo) nr = r + (len - lo) * k;
      const r0 = cRest0[c];
      cRest[c] = type === T_INNER ? Math.min(r0 * 1.15, Math.max(r0 * 0.6, nr)) : Math.min(r0 * 1.8, Math.max(r0 * 0.55, nr));
    }
  }

  /** Columns on a free edge of the mat (fleece border, cut-out or torn edge); cached per topology. */
  _edgeColumns() {
    if (this._edgeColsVer === this.topologyVersion && this._edgeCols) return this._edgeCols;
    const on = this._edgeCols || (this._edgeCols = new Uint8Array(this.N));
    on.fill(0);
    for (let e = 0; e < this.E; e++) if (this.eLive[e] === 1) { on[this.eA[e]] = 1; on[this.eB[e]] = 1; }
    this._edgeColsVer = this.topologyVersion;
    return on;
  }

  _solveConstraints(alpha) {
    const { active, nActive, cA, cB, cRest, cType, pos, invMass, N } = this;
    const Ls = this.L;
    const buckle = this.params.baseBuckle, hinge = this.params.hingeSlip, splay = this.params.edgeSplay, dyn = this.dyn;
    const onEdge = this._edgeColumns();
    for (let q = 0; q < nActive; q++) {
      const c = active[q];
      const a = cA[c], b = cB[c];
      const wa = invMass[a], wb = invMass[b];
      const w = wa + wb;
      if (w === 0) continue;
      const a3 = 3 * a, b3 = 3 * b;
      const dx = pos[b3] - pos[a3], dy = pos[b3 + 1] - pos[a3 + 1], dz = pos[b3 + 2] - pos[a3 + 2];
      const len = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (len < 1e-7) continue;
      const type = cType[c];
      let rest = cRest[c];
      if (type === T_SHEAR) {
        // tip of column j should sit straight above root j: |root_i - tip_j|^2 = d_ij^2 + L_j^2,
        // with d_ij the current root spacing - keeps staples upright without remembering any shape
        const j = b - N, j3 = 3 * j;
        const ex = pos[j3] - pos[a3], ey = pos[j3 + 1] - pos[a3 + 1], ez = pos[j3 + 2] - pos[a3 + 2];
        const Lj = Ls[j];
        rest = Math.sqrt(ex * ex + ey * ey + ez * ez + Lj * Lj);
      }
      let C = len - rest;
      if (type === T_TETHER && C < 0) continue;
      // staples resist being stretched far more than being compressed (crimped fibre)
      let al = alpha[type];
      // at the cut line a lifted staple slides along its still-rooted neighbour
      // (they are entangled over the whole staple length, not just at the root)
      if (dyn[a] !== dyn[b] && (type === T_INNER || type === T_TETHER)) al = al * hinge + alpha[1];
      if (type === T_STAPLE) { if (C < 0) al *= 6; }
      else if (type === T_SHEAR) { if (onEdge[b - N]) al *= splay; }   // nothing outside a free edge holds its staples up
      else if (type === T_OUTER) { if (C > 0) al = alpha[5]; }        // tips part easily
      else if (type === T_INNER) { if (C < 0) al *= buckle; }        // base bunches up
      const dlam = -C / (w + al);
      const s = dlam / len;
      pos[a3] -= wa * s * dx; pos[a3 + 1] -= wa * s * dy; pos[a3 + 2] -= wa * s * dz;
      pos[b3] += wb * s * dx; pos[b3 + 1] += wb * s * dy; pos[b3 + 2] += wb * s * dz;
    }
  }

  _collide(h) {
    const G = this.collisionGrid, { nx, ny, nz, min, data, nearest } = G, gh = G.h;
    const pos = this.pos, prev = this.prev, dl = this.dynList, nDyn = this.nDyn, N = this.N;
    const R = this.R, t = this.t, L = this.L, attached = this.attached, rest = this.restLocal;
    const mu = this.params.friction, floorY = this.floorY, muF = this.params.floorFriction;
    const cl = this.clipper && this.clipper.active ? this.clipper : null;
    const wedgeLen = this.params.wedgeLength, wedgeHalfW = this.params.wedgeWidth / 2;
    const wedgeMax = this.params.wedgeHeight, wedgeSlope = this.params.wedgeSlope, wedgeDrag = this.params.wedgeDrag;
    const mould = this.params.mould * 4, side = this._side || 0, tilt = this.params.tilt;
    const sx = nx - 1, sy = ny - 1, sz = nz - 1;
    for (let q = 0; q < nDyn; q++) {
      const p = dl[q], i3 = 3 * p;
      const isTip = p >= N, col = isTip ? p - N : p;
      let x = pos[i3], y = pos[i3 + 1], z = pos[i3 + 2];
      // --- sheep body (+ unshorn fleece) in sheep-local space
      const wx = x - t[0], wy = y - t[1], wz = z - t[2];
      const lx = R[0] * wx + R[3] * wy + R[6] * wz;
      const ly = R[1] * wx + R[4] * wy + R[7] * wz;
      const lz = R[2] * wx + R[5] * wy + R[8] * wz;
      const gx = (lx - min[0]) / gh, gy = (ly - min[1]) / gh, gz = (lz - min[2]) / gh;
      if (gx >= 0 && gy >= 0 && gz >= 0 && gx < sx && gy < sy && gz < sz) {
        const ix = gx | 0, iy = gy | 0, iz = gz | 0;
        const fx = gx - ix, fy = gy - iy, fz = gz - iz;
        const o = ix + nx * (iy + ny * iz);
        const c000 = data[o], c100 = data[o + 1], c010 = data[o + nx], c110 = data[o + nx + 1];
        const o2 = o + nx * ny;
        const c001 = data[o2], c101 = data[o2 + 1], c011 = data[o2 + nx], c111 = data[o2 + nx + 1];
        const x00 = c000 + (c100 - c000) * fx, x10 = c010 + (c110 - c010) * fx;
        const x01 = c001 + (c101 - c001) * fx, x11 = c011 + (c111 - c011) * fx;
        const y0 = x00 + (x10 - x00) * fy, y1 = x01 + (x11 - x01) * fy;
        const d = y0 + (y1 - y0) * fz;
        let clearance = isTip ? 0.012 : 0.005;
        if (d < 0.16) {
          const nb = nearest[ix + (fx > 0.5 ? 1 : 0) + nx * (iy + (fy > 0.5 ? 1 : 0) + ny * (iz + (fz > 0.5 ? 1 : 0)))];
          if (nb >= 0 && nb !== col && attached[nb]) {
            // lie on top of unshorn wool - but near the cut line the uncut mat
            // bends over, so ramp the clearance in with distance along the skin
            const rx = rest[3 * nb] - rest[3 * col], ry = rest[3 * nb + 1] - rest[3 * col + 1], rz = rest[3 * nb + 2] - rest[3 * col + 2];
            const rd = Math.sqrt(rx * rx + ry * ry + rz * rz);
            if (rd > 0.05) clearance += 0.9 * L[nb] * Math.min(1, (rd - 0.05) / 0.06);
          }
        }
        if (d < clearance) {
          // gradient of the trilinear field
          const dgx = ((c100 - c000) * (1 - fy) + (c110 - c010) * fy) * (1 - fz) + ((c101 - c001) * (1 - fy) + (c111 - c011) * fy) * fz;
          const dgy = ((x10 - x00)) * (1 - fz) + (x11 - x01) * fz;
          const dgz = y1 - y0;
          const gl = Math.hypot(dgx, dgy, dgz) || 1;
          const nlx = dgx / gl, nly = dgy / gl, nlz = dgz / gl;
          // world normal
          const nwx = R[0] * nlx + R[1] * nly + R[2] * nlz;
          const nwy = R[3] * nlx + R[4] * nly + R[5] * nlz;
          const nwz = R[6] * nlx + R[7] * nly + R[8] * nlz;
          const push = Math.min(clearance - d, 0.03);
          x += nwx * push; y += nwy * push; z += nwz * push;
          // friction: remove part of the tangential motion this substep
          const mx = x - prev[i3], my = y - prev[i3 + 1], mz = z - prev[i3 + 2];
          const mn = mx * nwx + my * nwy + mz * nwz;
          const tx = mx - mn * nwx, ty = my - mn * nwy, tz = mz - mn * nwz;
          x -= tx * mu; y -= ty * mu; z -= tz * mu;
        }
      }
      // --- floor
      const fr = isTip ? 0.01 : 0.004;
      if (y < floorY + fr) {
        y = floorY + fr;
        x -= (x - prev[i3]) * muF; z -= (z - prev[i3 + 2]) * muF;
      }
      // --- comb & handpiece: a wedge that runs under the wool. Severed wool
      // rides up the comb and over the raised handpiece (and the shearer's hand
      // on it), which is what rolls each strip back off the skin.
      if (cl) {
        const dx = x - cl.px, dy = y - cl.py, dz = z - cl.pz;
        const u = dx * cl.Zx + dy * cl.Zy + dz * cl.Zz;
        if (u < 0.045 && u > -wedgeLen) {
          const v = dx * cl.Xx + dy * cl.Xy + dz * cl.Xz;
          if (v < wedgeHalfW && v > -wedgeHalfW) {
            const w = dx * cl.Yx + dy * cl.Yy + dz * cl.Yz;
            if (w > -0.02) {
              let top = u > -0.012 ? 0.007 : Math.min(wedgeMax, 0.007 + (-0.012 - u) * wedgeSlope);
              const lat = Math.abs(v) / wedgeHalfW;
              top *= 1 - lat * lat;
              // the handpiece is tilted: the free (open-side) edge of the slice rides
              // higher than the edge still entangled with the uncut wool
              top *= 1 + tilt * side * v / wedgeHalfW;
              top += isTip ? 0.006 : 0.002;
              if (w < top) {
                const push = (top - w) * 0.5;
                x += cl.Yx * push; y += cl.Yy * push; z += cl.Yz * push;
                // mouldboard: the higher it rides, the more the slice is rolled back
                // over its hinge onto the uncut wool (cut face turning up)
                const turn = -push * mould * side * Math.min(1, w / 0.04 + 0.3);
                x += cl.Xx * turn; y += cl.Xy * turn; z += cl.Xz * turn;
                // the wool slides back along the handpiece as it advances
                x -= cl.Zx * push * wedgeDrag; y -= cl.Zy * push * wedgeDrag; z -= cl.Zz * push * wedgeDrag;
              }
            }
          }
        }
      }
      pos[i3] = x; pos[i3 + 1] = y; pos[i3 + 2] = z;
    }
  }

  // Self-collision (fleece folding onto itself) is solved inside the substeps,
  // like every other constraint, so it never fights the fibres. The mat is solid,
  // not a cage of two particle layers: each staple also carries SELF_INNER virtual
  // contact points between root and tip, whose pushes go to its two particles.
  // Candidate pairs (close in space, far apart in the fleece) are found once per
  // frame with a margin for the motion within the frame.
  //
  // Contact points: q < P is particle q; q >= P is virtual point k of column i,
  // with q = P + i * SELF_INNER + k, at fraction (k + 1) / (SELF_INNER + 1) up the staple.
  _contactPoint(q, out) {
    const pos = this.pos, P = this.P;
    if (q < P) { out[0] = pos[3 * q]; out[1] = pos[3 * q + 1]; out[2] = pos[3 * q + 2]; return; }
    const i = Math.floor((q - P) / SELF_INNER), t = ((q - P) % SELF_INNER + 1) / (SELF_INNER + 1), j = this.N + i;
    out[0] = pos[3 * i] + (pos[3 * j] - pos[3 * i]) * t;
    out[1] = pos[3 * i + 1] + (pos[3 * j + 1] - pos[3 * i + 1]) * t;
    out[2] = pos[3 * i + 2] + (pos[3 * j + 2] - pos[3 * i + 2]) * t;
  }

  // The pair list stays valid until a contact point has moved more than half the
  // search margin, or wool has woken up or been cut free (new contact points).
  _selfPairsStale() {
    if (!this._selfPts || this._selfWakeVer !== this._wakeVer) return true;
    const pts = this._selfPts, ppos = this._selfPos, m = this._nSelfPts, c = [0, 0, 0];
    const lim2 = (SELF_MARGIN / 2) ** 2;
    for (let a = 0; a < m; a++) {
      this._contactPoint(pts[a], c);
      const dx = c[0] - ppos[3 * a], dy = c[1] - ppos[3 * a + 1], dz = c[2] - ppos[3 * a + 2];
      if (dx * dx + dy * dy + dz * dz > lim2) return true;
    }
    return false;
  }

  _findSelfPairs() {
    this._selfWakeVer = this._wakeVer;
    const r = SELF_R, reach = r + SELF_MARGIN, reach2 = reach * reach, inv = 1 / reach;
    const P = this.P, N = this.N, rest = this.restLocal, dl = this.dynList, n = this.nDyn, asleep = this.asleep;
    // contact points: every loose particle (sleeping wool is an obstacle too), plus the
    // inner points of every loose staple; pairs where both sides sleep are skipped
    const pts = this._selfPts || (this._selfPts = new Int32Array(P + SELF_INNER * N));
    const dyn = this.dyn;
    const sleeps = (q) => (q < P ? asleep[q] : asleep[Math.floor((q - P) / SELF_INNER)] && asleep[N + Math.floor((q - P) / SELF_INNER)]);
    const ppos = this._selfPos || (this._selfPos = new Float32Array(3 * (P + SELF_INNER * N)));
    const prest = this._selfRest || (this._selfRest = new Float32Array(3 * (P + SELF_INNER * N)));
    const c = [0, 0, 0];
    let m = 0;
    for (let q = 0; q < P; q++) if (dyn[q]) pts[m++] = q;
    for (let i = 0; i < N; i++) {
      if (!dyn[i]) continue;
      for (let k = 0; k < SELF_INNER; k++) pts[m++] = P + i * SELF_INNER + k;
    }
    this._nSelfPts = m;
    for (let a = 0; a < m; a++) {
      const q = pts[a];
      this._contactPoint(q, c);
      ppos[3 * a] = c[0]; ppos[3 * a + 1] = c[1]; ppos[3 * a + 2] = c[2];
      if (q < P) { prest[3 * a] = rest[3 * q]; prest[3 * a + 1] = rest[3 * q + 1]; prest[3 * a + 2] = rest[3 * q + 2]; }
      else {
        const i = Math.floor((q - P) / SELF_INNER), t = ((q - P) % SELF_INNER + 1) / (SELF_INNER + 1), j = N + i;
        for (let d = 0; d < 3; d++) prest[3 * a + d] = rest[3 * i + d] + (rest[3 * j + d] - rest[3 * i + d]) * t;
      }
    }
    const TABLE = 16384;
    if (!this._hashHead) { this._hashHead = new Int32Array(TABLE); this._hashNext = new Int32Array(P + SELF_INNER * N); }
    const head = this._hashHead, next = this._hashNext;
    head.fill(-1);
    const hcell = (x, y, z) => (((x * 92837111) ^ (y * 689287499) ^ (z * 283923481)) >>> 0) % TABLE;
    for (let a = 0; a < m; a++) {
      const hh = hcell(Math.floor(ppos[3 * a] * inv), Math.floor(ppos[3 * a + 1] * inv), Math.floor(ppos[3 * a + 2] * inv));
      next[a] = head[hh]; head[hh] = a;
    }
    const minRest2 = (2.4 * r) * (2.4 * r);
    let pairs = this._selfPairs || (this._selfPairs = new Int32Array(8 * P));
    let np = 0;
    const wake = [];
    for (let a = 0; a < m; a++) {
      const a3 = 3 * a;
      const cx = Math.floor(ppos[a3] * inv), cy = Math.floor(ppos[a3 + 1] * inv), cz = Math.floor(ppos[a3 + 2] * inv);
      const home = hcell(cx, cy, cz);
      // own cell (each pair once) plus the forward half of the 26 neighbours
      for (let h = 0; h < 14; h++) {
        const cell = h === 0 ? home : hcell(cx + HALF_NBR[3 * h], cy + HALF_NBR[3 * h + 1], cz + HALF_NBR[3 * h + 2]);
        if (h > 0 && cell === home) continue;
        let b = head[cell];
        while (b >= 0) {
          if (h > 0 || b > a) {
            const b3 = 3 * b;
            const dx = ppos[b3] - ppos[a3], dy = ppos[b3 + 1] - ppos[a3 + 1], dz = ppos[b3 + 2] - ppos[a3 + 2];
            if (dx * dx + dy * dy + dz * dz < reach2) {
              const rx = prest[b3] - prest[a3], ry = prest[b3 + 1] - prest[a3 + 1], rz = prest[b3 + 2] - prest[a3 + 2];
              if (rx * rx + ry * ry + rz * rz > minRest2) {
                const sa = sleeps(pts[a]), sb = sleeps(pts[b]);
                if (sa && sb) { b = next[b]; continue; }
                // wool landing hard on a sleeping pile wakes it, so the pile gives a little
                if ((sa || sb) && dx * dx + dy * dy + dz * dz < (0.6 * r) ** 2) wake.push(sa ? pts[a] : pts[b]);
                if (np + 2 > pairs.length) {
                  const grown = new Int32Array(2 * pairs.length);
                  grown.set(pairs); pairs = this._selfPairs = grown;
                }
                pairs[np++] = pts[a]; pairs[np++] = pts[b];
              }
            }
          }
          b = next[b];
        }
      }
    }
    this._nSelfPairs = np;
    for (const q of wake) {
      if (q < P) this._wake(q);
      else { const i = Math.floor((q - P) / SELF_INNER); this._wake(i); this._wake(N + i); }
    }
    // resolve each contact point to its particle(s) and weights once per frame:
    // slots 0,1 = point A (root, tip), 2,3 = point B; a real particle uses one slot
    const npairs = np / 2;
    if (!this._pairIdx || this._pairIdx.length < 4 * npairs) {
      this._pairIdx = new Int32Array(4 * Math.max(npairs, 1024) * 2);
      this._pairWt = new Float32Array(this._pairIdx.length);
    }
    const pi = this._pairIdx, pw = this._pairWt;
    for (let k = 0; k < npairs; k++) {
      for (let e = 0; e < 2; e++) {
        const q = pairs[2 * k + e], o = 4 * k + 2 * e;
        if (q < P) { pi[o] = q; pi[o + 1] = q; pw[o] = 1; pw[o + 1] = 0; continue; }
        const i = Math.floor((q - P) / SELF_INNER), t = ((q - P) % SELF_INNER + 1) / (SELF_INNER + 1);
        pi[o] = i; pi[o + 1] = N + i; pw[o] = 1 - t; pw[o + 1] = t;
      }
    }
  }

  _solveSelfPairs() {
    const r = SELF_R, r2 = r * r, pos = this.pos, prev = this.prev, im = this.invMass;
    const pi = this._pairIdx, pw = this._pairWt, npairs = this._nSelfPairs / 2, mu = this.params.selfFriction;
    for (let k = 0; k < npairs; k++) {
      const o = 4 * k;
      const a0 = pi[o], a1 = pi[o + 1], b0 = pi[o + 2], b1 = pi[o + 3];
      const ua = pw[o], va = pw[o + 1], ub = pw[o + 2], vb = pw[o + 3];
      const A0 = 3 * a0, A1 = 3 * a1, B0 = 3 * b0, B1 = 3 * b1;
      // contact points (a real particle has weights 1, 0)
      let dx = pos[B0] * ub + pos[B1] * vb - pos[A0] * ua - pos[A1] * va;
      let dy = pos[B0 + 1] * ub + pos[B1 + 1] * vb - pos[A0 + 1] * ua - pos[A1 + 1] * va;
      let dz = pos[B0 + 2] * ub + pos[B1 + 2] * vb - pos[A0 + 2] * ua - pos[A1 + 2] * va;
      const d2 = dx * dx + dy * dy + dz * dz;
      if (d2 >= r2 || d2 < 1e-12) continue;
      // generalised inverse masses of the two contact points
      const ga0 = ua * im[a0], ga1 = va * im[a1], gb0 = ub * im[b0], gb1 = vb * im[b1];
      const w = ua * ga0 + va * ga1 + ub * gb0 + vb * gb1;
      if (w === 0) continue;
      const d = Math.sqrt(d2), pen = r - d, s = pen / (d * w);
      let x = dx * s, y = dy * s, z = dz * s;
      pos[A0] -= x * ga0; pos[A0 + 1] -= y * ga0; pos[A0 + 2] -= z * ga0;
      pos[A1] -= x * ga1; pos[A1 + 1] -= y * ga1; pos[A1 + 2] -= z * ga1;
      pos[B0] += x * gb0; pos[B0 + 1] += y * gb0; pos[B0 + 2] += z * gb0;
      pos[B1] += x * gb1; pos[B1 + 1] += y * gb1; pos[B1 + 2] += z * gb1;
      if (!mu) continue;
      // fibre-on-fibre friction: wool touching wool barely slides (static up to mu * push)
      dx /= d; dy /= d; dz /= d;
      let rx = (pos[B0] - prev[B0]) * ub + (pos[B1] - prev[B1]) * vb - (pos[A0] - prev[A0]) * ua - (pos[A1] - prev[A1]) * va;
      let ry = (pos[B0 + 1] - prev[B0 + 1]) * ub + (pos[B1 + 1] - prev[B1 + 1]) * vb - (pos[A0 + 1] - prev[A0 + 1]) * ua - (pos[A1 + 1] - prev[A1 + 1]) * va;
      let rz = (pos[B0 + 2] - prev[B0 + 2]) * ub + (pos[B1 + 2] - prev[B1 + 2]) * vb - (pos[A0 + 2] - prev[A0 + 2]) * ua - (pos[A1 + 2] - prev[A1 + 2]) * va;
      const rn = rx * dx + ry * dy + rz * dz;
      rx -= rn * dx; ry -= rn * dy; rz -= rn * dz;
      const rt = Math.hypot(rx, ry, rz);
      if (rt < 1e-9) continue;
      const f = Math.min(1, mu * pen / rt) / w;
      x = rx * f; y = ry * f; z = rz * f;
      pos[A0] += x * ga0; pos[A0 + 1] += y * ga0; pos[A0 + 2] += z * ga0;
      pos[A1] += x * ga1; pos[A1 + 1] += y * ga1; pos[A1 + 2] += z * ga1;
      pos[B0] -= x * gb0; pos[B0 + 1] -= y * gb0; pos[B0 + 2] -= z * gb0;
      pos[B1] -= x * gb1; pos[B1 + 1] -= y * gb1; pos[B1 + 2] -= z * gb1;
    }
  }

  _checkTears() {
    const { active, nActive, cA, cB, cRest, cType, cEdge, pos, eOver } = this;
    const limit = this.params.tearRatio;
    let tornThisFrame = 0;
    for (let q = 0; q < nActive; q++) {
      const c = active[q];
      if (cType[c] !== T_INNER) continue;
      if (!this.dyn[cA[c]] || !this.dyn[cB[c]]) continue; // the cut line parts, it doesn't tear
      const a3 = 3 * cA[c], b3 = 3 * cB[c];
      const len = Math.hypot(pos[b3] - pos[a3], pos[b3 + 1] - pos[a3 + 1], pos[b3 + 2] - pos[a3 + 2]);
      const e = cEdge[c];
      if (len > (1 + (limit - 1) * this.eWeak[e]) * this.cRest0[c]) {
        if (++eOver[e] > 12 && tornThisFrame < 4) { this._tearEdge(e); tornThisFrame++; }
      } else eOver[e] = 0;
    }
  }

  _tearEdge(e) {
    for (const t of [this.eTri[2 * e], this.eTri[2 * e + 1]]) {
      if (t < 0 || !this.triAlive[t]) continue;
      this._killTri(t);
    }
    this.stats.tears++;
    this.topologyVersion++;
    this.activeDirty = true;
  }

  /** Per-particle stretch of the outer layer (drives the opening of staple cracks). */
  computeStrain() {
    const { E, eA, eB, eLive, pos, restLocal, N, strain } = this;
    const acc = this._strainAcc || (this._strainAcc = new Float32Array(this.P));
    const cnt = this._strainCnt || (this._strainCnt = new Float32Array(this.P));
    acc.fill(0); cnt.fill(0);
    for (let e = 0; e < E; e++) {
      if (!eLive[e]) continue;
      const a = N + eA[e], b = N + eB[e];
      const a3 = 3 * a, b3 = 3 * b;
      const len = Math.hypot(pos[b3] - pos[a3], pos[b3 + 1] - pos[a3 + 1], pos[b3 + 2] - pos[a3 + 2]);
      const rl = Math.hypot(restLocal[b3] - restLocal[a3], restLocal[b3 + 1] - restLocal[a3 + 1], restLocal[b3 + 2] - restLocal[a3 + 2]);
      const s = len / rl;
      acc[a] += s; acc[b] += s; cnt[a]++; cnt[b]++;
      // base layer too (compression shows as bunching)
      const c3 = 3 * eA[e], d3 = 3 * eB[e];
      const l2 = Math.hypot(pos[d3] - pos[c3], pos[d3 + 1] - pos[c3 + 1], pos[d3 + 2] - pos[c3 + 2]);
      const r2 = Math.hypot(restLocal[d3] - restLocal[c3], restLocal[d3 + 1] - restLocal[c3 + 1], restLocal[d3 + 2] - restLocal[c3 + 2]);
      acc[eA[e]] += l2 / r2; acc[eB[e]] += l2 / r2; cnt[eA[e]]++; cnt[eB[e]]++;
    }
    for (let p = 0; p < this.P; p++) strain[p] = cnt[p] ? acc[p] / cnt[p] : 1;
  }

  /** Number of separate fleece pieces (connected live triangles that are free or attached). */
  countPieces() {
    const n = this.N, parent = new Int32Array(n).map((_, i) => i);
    const find = (a) => { while (parent[a] !== a) { parent[a] = parent[parent[a]]; a = parent[a]; } return a; };
    const used = new Uint8Array(n);
    for (let t = 0; t < this.T; t++) {
      if (!this.triAlive[t]) continue;
      const a = this.tris[3 * t], b = this.tris[3 * t + 1], c = this.tris[3 * t + 2];
      used[a] = used[b] = used[c] = 1;
      parent[find(b)] = find(a); parent[find(c)] = find(a);
    }
    // loose locks (small bits that pull away at thin edges) don't count as pieces
    const size = new Map();
    for (let i = 0; i < n; i++) if (used[i]) { const r = find(i); size.set(r, (size.get(r) || 0) + 1); }
    let pieces = 0;
    for (const c of size.values()) if (c >= 40) pieces++;
    return Math.max(1, pieces);
  }

  get fractionShorn() { return this.stats.cut / this.N; }
}

export { CollisionGrid };
