// What the comb and cutter do to the wool as the handpiece moves from `from` to
// `to` (world space) with comb basis X (across the comb), Y (skin normal) and Z
// (direction of travel):
//
//  * rooted staples in the comb are severed at the skin and lift off the comb;
//  * going back over skin already shorn by an EARLIER blow, the wool that grew
//    there and still lies in the cutter's mouth is cut through: the mat parts
//    along the middle of the stroke. That is how a new opening is made in the
//    fleece, and how going back over opened wool chops a fleece up. Wool draped
//    over the path from elsewhere rides up over the comb and is not cut.
//
// The slice the current blow is lifting is never cut. Shared by the game and the
// test harness.
import { COMB_WIDTH } from './clipper.js';

const STEP = 0.008;                       // m between samples along the stroke
const ROOT_UP = 0.03, ROOT_BACK = -0.022, ROOT_FWD = 0.036;
const MOUTH_LO = -0.015, MOUTH_HI = 0.025; // cutter mouth height around the comb plane
const CUT_LANE = COMB_WIDTH / 4;          // re-cut only wool that grew in the middle half of the comb's path

/**
 * @returns {{ released: number[][], cutEdges: number[] }} released: [col, vx, vy, vz]
 */
export function cutterPass(fleece, from, to, basis, blow) {
  const { X, Y, Z } = basis;
  const released = [], cutEdges = [];
  const tgt = fleece.target, pos = fleece.pos, att = fleece.attached, N = fleece.N, half = COMB_WIDTH / 2;
  const rb = fleece.releaseBlow;
  const dist = Math.hypot(to.x - from.x, to.y - from.y, to.z - from.z);
  const steps = Math.max(1, Math.min(40, Math.ceil(dist / STEP)));
  const mouth = fleece._mouth || (fleece._mouth = new Float32Array(N));   // lateral offset of loose columns in the mouth
  const inMouth = fleece._inMouth || (fleece._inMouth = new Uint8Array(N));
  for (let s = 1; s <= steps; s++) {
    const f = s / steps;
    const vx = from.x + (to.x - from.x) * f, vy = from.y + (to.y - from.y) * f, vz = from.z + (to.z - from.z) * f;
    const marked = [];
    for (let i = 0; i < N; i++) {
      if (att[i]) {
        const dx = tgt[3 * i] - vx, dy = tgt[3 * i + 1] - vy, dz = tgt[3 * i + 2] - vz;
        const up = dx * Y.x + dy * Y.y + dz * Y.z;
        if (up > ROOT_UP || up < -ROOT_UP) continue;
        const lat = dx * X.x + dy * X.y + dz * X.z;
        if (lat > half || lat < -half) continue;
        const fw = dx * Z.x + dy * Z.y + dz * Z.z;
        if (fw < ROOT_BACK || fw > ROOT_FWD) continue;
        // severed fibres: the column lifts off the comb and rolls back over the handpiece
        const r = 0.05 * (Math.random() - 0.5);
        const ux = Y.x * 0.35 - Z.x * 0.12 + r, uy = Y.y * 0.35 - Z.y * 0.12 + r, uz = Y.z * 0.35 - Z.z * 0.12 + r;
        if (fleece.release(i, ux, uy, uz, blow)) released.push([i, ux, uy, uz]);
        continue;
      }
      if (rb[i] < 0 || rb[i] >= blow) continue;
      // loose wool from an earlier blow, going back over the skin it grew on (wool
      // draped over the path from elsewhere rides up over the comb instead)
      const rx = tgt[3 * i] - vx, ry = tgt[3 * i + 1] - vy, rz = tgt[3 * i + 2] - vz;
      const rUp = rx * Y.x + ry * Y.y + rz * Y.z;
      if (rUp > ROOT_UP || rUp < -ROOT_UP) continue;
      const rLat = rx * X.x + ry * X.y + rz * X.z;
      if (rLat > CUT_LANE || rLat < -CUT_LANE) continue;
      const rFw = rx * Z.x + ry * Z.y + rz * Z.z;
      if (rFw < ROOT_BACK || rFw > ROOT_FWD) continue;
      // ... and still lying there: is its base or its tip in the cutter's mouth?
      for (const p of [i, N + i]) {
        const dx = pos[3 * p] - vx, dy = pos[3 * p + 1] - vy, dz = pos[3 * p + 2] - vz;
        const up = dx * Y.x + dy * Y.y + dz * Y.z;
        if (up > MOUTH_HI || up < MOUTH_LO) continue;
        const lat = dx * X.x + dy * X.y + dz * X.z;
        if (lat > half || lat < -half) continue;
        const fw = dx * Z.x + dy * Z.y + dz * Z.z;
        if (fw < ROOT_BACK || fw > ROOT_FWD) continue;
        mouth[i] = rLat; inMouth[i] = 1; marked.push(i);
        break;
      }
    }
    // the cutter parts the mat along the middle of the stroke: cut the live edges
    // between two loose columns in the mouth that lie on opposite sides of it
    const { vEdgeStart, vEdges, eA, eB, eLive } = fleece;
    for (const i of marked) {
      for (let q = vEdgeStart[i]; q < vEdgeStart[i + 1]; q++) {
        const e = vEdges[q];
        if (!eLive[e]) continue;
        const j = eA[e] === i ? eB[e] : eA[e];
        if (!inMouth[j] || (mouth[i] > 0) === (mouth[j] > 0)) continue;
        if (fleece.cutEdge(e)) cutEdges.push(e);
      }
    }
    for (const i of marked) inMouth[i] = 0;
  }
  return { released, cutEdges };
}
