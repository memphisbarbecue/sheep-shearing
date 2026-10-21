// Skeletal rig for the sheep: a small bone hierarchy driven procedurally
// (breathing, head and neck movement, tail flicks, occasional leg kicks) and a
// CPU linear-blend skinner shared by the skin mesh and the fleece anchors, so
// wool stays rooted to exactly the same moving skin.

import { SKELETON as S } from './anatomy.js';

// bone: name, parent index, pivot (joint centre), influence segment [a, b, radius]
const BONES = [
  { name: 'body', parent: -1, pivot: [0, 0, 0], seg: [[-0.40, 0.05, 0], [0.22, 0.04, 0], 0.15] },
  { name: 'neck', parent: 0, pivot: S.c7, seg: [[0.29, 0.10, 0], [0.49, 0.25, 0], 0.075] },
  { name: 'head', parent: 1, pivot: S.atlas, seg: [[0.53, 0.27, 0], [0.71, 0.15, 0], 0.045] },
  { name: 'tail', parent: 0, pivot: S.tailRoot, seg: [[-0.41, 0.11, 0], [-0.47, 0.02, 0], 0.03] },
  { name: 'foreL', parent: 0, pivot: S.elbow, seg: [[0.215, -0.17, 0.095], [0.255, -0.51, 0.085], 0.025] },
  { name: 'foreR', parent: 0, pivot: mirror(S.elbow), seg: [[0.215, -0.17, -0.095], [0.255, -0.51, -0.085], 0.025] },
  { name: 'hindL', parent: 0, pivot: S.stifle, seg: [[-0.225, -0.17, 0.095], [-0.30, -0.51, 0.075], 0.03] },
  { name: 'hindR', parent: 0, pivot: mirror(S.stifle), seg: [[-0.225, -0.17, -0.095], [-0.30, -0.51, -0.075], 0.03] },
];
export const BONE_INDEX = Object.fromEntries(BONES.map((b, i) => [b.name, i]));

function mirror(p) { return [p[0], p[1], -p[2]]; }

function segDist(p, a, b) {
  const bax = b[0] - a[0], bay = b[1] - a[1], baz = b[2] - a[2];
  const pax = p[0] - a[0], pay = p[1] - a[1], paz = p[2] - a[2];
  const t = Math.max(0, Math.min(1, (pax * bax + pay * bay + paz * baz) / (bax * bax + bay * bay + baz * baz)));
  return Math.hypot(pax - bax * t, pay - bay * t, paz - baz * t);
}

/** Top-3 skin weights per point. */
export function computeWeights(positions) {
  const n = positions.length / 3;
  const idx = new Uint8Array(n * 3), w = new Float32Array(n * 3);
  const p = [0, 0, 0], ws = new Float32Array(BONES.length);
  for (let i = 0; i < n; i++) {
    p[0] = positions[3 * i]; p[1] = positions[3 * i + 1]; p[2] = positions[3 * i + 2];
    for (let b = 0; b < BONES.length; b++) {
      const [a, c, r] = BONES[b].seg;
      ws[b] = Math.exp(-(segDist(p, a, c) - r) / 0.018);
    }
    const order = [...ws.keys()].sort((a, b) => ws[b] - ws[a]).slice(0, 3);
    let sum = 0;
    for (const o of order) sum += ws[o];
    for (let k = 0; k < 3; k++) { idx[3 * i + k] = order[k]; w[3 * i + k] = ws[order[k]] / sum; }
  }
  return { idx, w };
}

/** Breathing displacement direction × magnitude (radial expansion of thorax & flank). */
export function computeBreath(positions) {
  const n = positions.length / 3;
  const out = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    const x = positions[3 * i], y = positions[3 * i + 1], z = positions[3 * i + 2];
    // radial from the trunk axis
    const ry = y - 0.0, rz = z;
    const r = Math.hypot(ry, rz) || 1;
    const along = Math.exp(-Math.pow((x - 0.0) / 0.26, 4));
    const m = along * Math.min(1, r / 0.12) * (y < -0.18 ? Math.max(0, 1 + (y + 0.18) / 0.05) : 1);
    // ribs move laterally more than vertically; belly drops a little
    out[3 * i] = 0;
    out[3 * i + 1] = (ry / r) * m * 0.6;
    out[3 * i + 2] = (rz / r) * m;
  }
  return out;
}

// --- tiny 3x4 matrix helpers (row-major [r00 r01 r02 tx r10 ...]) ---
function rotXYZ(rx, ry, rz) {
  const cx = Math.cos(rx), sx = Math.sin(rx), cy = Math.cos(ry), sy = Math.sin(ry), cz = Math.cos(rz), sz = Math.sin(rz);
  return [
    cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx, 0,
    sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx, 0,
    -sy, cy * sx, cy * cx, 0,
  ];
}
function mul(a, b, out) {
  for (let r = 0; r < 3; r++) {
    const a0 = a[4 * r], a1 = a[4 * r + 1], a2 = a[4 * r + 2], a3 = a[4 * r + 3];
    out[4 * r] = a0 * b[0] + a1 * b[4] + a2 * b[8];
    out[4 * r + 1] = a0 * b[1] + a1 * b[5] + a2 * b[9];
    out[4 * r + 2] = a0 * b[2] + a1 * b[6] + a2 * b[10];
    out[4 * r + 3] = a0 * b[3] + a1 * b[7] + a2 * b[11] + a3;
  }
  return out;
}
function aboutPivot(R, p) {
  // T(p) R T(-p)
  const m = R.slice();
  m[3] = p[0] - (R[0] * p[0] + R[1] * p[1] + R[2] * p[2]);
  m[7] = p[1] - (R[4] * p[0] + R[5] * p[1] + R[6] * p[2]);
  m[11] = p[2] - (R[8] * p[0] + R[9] * p[1] + R[10] * p[2]);
  return m;
}

export class Rig {
  constructor() {
    this.nb = BONES.length;
    this.mats = new Float32Array(12 * this.nb);
    this.pose = Object.fromEntries(BONES.map((b) => [b.name, [0, 0, 0]]));
    this.breath = 0;
    this.update();
  }
  /** Recompute bone matrices from this.pose (Euler angles per bone). */
  update() {
    const world = [];
    for (let b = 0; b < this.nb; b++) {
      const B = BONES[b];
      const local = aboutPivot(rotXYZ(...this.pose[B.name]), B.pivot);
      const m = B.parent < 0 ? local : mul(world[B.parent], local, new Array(12));
      world.push(m);
      this.mats.set(m, 12 * b);
    }
  }
  boneMatrix(name) { const b = BONE_INDEX[name]; return this.mats.subarray(12 * b, 12 * b + 12); }

  /**
   * Skin `rest` (xyz triples) into `out`. Optional normals. `breathVec` adds a
   * breathing offset scaled by this.breath before skinning.
   */
  skin(rest, weights, breathVec, out, restN, outN, count = rest.length / 3) {
    const M = this.mats, wi = weights.idx, ww = weights.w, br = this.breath;
    for (let i = 0; i < count; i++) {
      const i3 = 3 * i;
      let x = rest[i3], y = rest[i3 + 1], z = rest[i3 + 2];
      if (breathVec) { x += breathVec[i3] * br; y += breathVec[i3 + 1] * br; z += breathVec[i3 + 2] * br; }
      let ox = 0, oy = 0, oz = 0, nx = 0, ny = 0, nz = 0;
      for (let k = 0; k < 3; k++) {
        const w = ww[i3 + k];
        if (w < 1e-4) continue;
        const o = 12 * wi[i3 + k];
        ox += w * (M[o] * x + M[o + 1] * y + M[o + 2] * z + M[o + 3]);
        oy += w * (M[o + 4] * x + M[o + 5] * y + M[o + 6] * z + M[o + 7]);
        oz += w * (M[o + 8] * x + M[o + 9] * y + M[o + 10] * z + M[o + 11]);
        if (restN) {
          const a = restN[i3], b = restN[i3 + 1], c = restN[i3 + 2];
          nx += w * (M[o] * a + M[o + 1] * b + M[o + 2] * c);
          ny += w * (M[o + 4] * a + M[o + 5] * b + M[o + 6] * c);
          nz += w * (M[o + 8] * a + M[o + 9] * b + M[o + 10] * c);
        }
      }
      out[i3] = ox; out[i3 + 1] = oy; out[i3 + 2] = oz;
      if (restN) { outN[i3] = nx; outN[i3 + 1] = ny; outN[i3 + 2] = nz; }
    }
  }
}

/**
 * Procedural behaviour of a calm sheep being handled: breathing, slow head
 * movements, ear and tail flicks, the occasional hind-leg kick.
 */
export class SheepBehaviour {
  constructor(rig) {
    this.rig = rig;
    this.t = 0;
    this.kick = { side: 'hindL', t: -10 };
    this.nextKick = 6 + Math.random() * 10;
    this.ear = { L: 0, R: 0, tL: -10, tR: -10 };
  }
  step(dt, agitation = 0) {
    this.t += dt;
    const t = this.t, P = this.rig.pose;
    // breathing ~ 30/min at rest, faster when agitated
    const rate = 0.5 + 0.4 * agitation;
    this.phase = (this.phase || 0) + dt * rate * Math.PI * 2;
    this.rig.breath = 0.006 * (0.5 + 0.5 * Math.sin(this.phase)) * (1 + 0.5 * agitation);
    const n = (f, s) => Math.sin(t * f + s) * 0.6 + Math.sin(t * f * 2.31 + s * 1.7) * 0.4;
    P.neck = [n(0.37, 1) * 0.05, n(0.23, 4) * 0.08, n(0.31, 2) * 0.05];
    P.head = [n(0.41, 7) * 0.06, n(0.29, 3) * 0.1, n(0.53, 5) * 0.08];
    P.tail = [0, Math.sin(t * 7) * 0.15 * Math.max(0, Math.sin(t * 0.7)) , n(0.6, 9) * 0.1];
    // leg kick
    if (t > this.nextKick) {
      this.kick = { side: Math.random() < 0.5 ? 'hindL' : 'hindR', t };
      this.nextKick = t + 7 + Math.random() * 12 / (1 + agitation);
    }
    for (const s of ['hindL', 'hindR', 'foreL', 'foreR']) P[s] = [0, 0, 0];
    const kt = t - this.kick.t;
    if (kt >= 0 && kt < 0.9) {
      const a = Math.sin(Math.PI * kt / 0.9) * (kt < 0.35 ? 1 : 0.85);
      P[this.kick.side] = [0, 0, 0.35 * a];
    }
    P.foreL = [0, 0, n(0.2, 11) * 0.04];
    P.foreR = [0, 0, n(0.21, 13) * 0.04];
    // ears
    for (const s of ['L', 'R']) {
      if (Math.random() < dt * 0.15) this.ear['t' + s] = t;
      const et = t - this.ear['t' + s];
      this.ear[s] = et < 0.5 ? Math.sin(Math.PI * et / 0.5) * 0.5 : 0;
    }
    this.rig.update();
  }
}
