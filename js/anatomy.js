// Anatomical description of a Merino-type wool ewe, expressed as a signed distance
// field built from the skeleton (bone landmarks) and the major muscle masses that
// sit on it. Units are metres. Local frame: +x toward the head, +y up, +z the
// sheep's left side. The ground the sheep stands on is at y = GROUND_Y.
//
// The animal is bilaterally symmetric, so every primitive is evaluated at
// (x, y, |z|): lateral parts are defined once on the left (+z) side.

export const GROUND_Y = -0.52;

// ---------------------------------------------------------------------------
// Skeleton landmarks (approximate adult Merino ewe, ~55 kg, 0.68 m at withers)
// ---------------------------------------------------------------------------
export const SKELETON = {
  // axial
  sacrum: [-0.36, 0.12, 0],
  lumbar: [-0.18, 0.14, 0],
  thoracic: [0.02, 0.15, 0],
  withers: [0.20, 0.175, 0],
  c7: [0.29, 0.08, 0],
  atlas: [0.50, 0.27, 0],
  poll: [0.535, 0.305, 0],
  noseTip: [0.715, 0.15, 0],
  tailRoot: [-0.40, 0.115, 0],
  tailTip: [-0.47, 0.02, 0],
  // pelvis / hind limb (left)
  tuberCoxae: [-0.22, 0.12, 0.11],
  pinBone: [-0.40, 0.075, 0.055],
  hip: [-0.30, 0.02, 0.10],
  stifle: [-0.205, -0.15, 0.10],
  hock: [-0.335, -0.31, 0.075],
  hindFetlock: [-0.315, -0.458, 0.075],
  hindHoof: [-0.295, -0.505, 0.075],
  // thoracic limb (left)
  scapulaTop: [0.18, 0.15, 0.10],
  shoulder: [0.30, -0.04, 0.10],
  elbow: [0.215, -0.15, 0.095],
  carpus: [0.235, -0.35, 0.085],
  foreFetlock: [0.24, -0.458, 0.085],
  foreHoof: [0.258, -0.505, 0.085],
};

// ---------------------------------------------------------------------------
// Primitive helpers
// ---------------------------------------------------------------------------
const CONE = 0, ELL = 1;

function rotMatrix(rx, ry, rz) {
  // returns rows of R^T (world->local) for Euler XYZ rotation (radians)
  const cx = Math.cos(rx), sx = Math.sin(rx);
  const cy = Math.cos(ry), sy = Math.sin(ry);
  const cz = Math.cos(rz), sz = Math.sin(rz);
  // R = Rz * Ry * Rx
  const r00 = cz * cy, r01 = cz * sy * sx - sz * cx, r02 = cz * sy * cx + sz * sx;
  const r10 = sz * cy, r11 = sz * sy * sx + cz * cx, r12 = sz * sy * cx - cz * sx;
  const r20 = -sy, r21 = cy * sx, r22 = cy * cx;
  // transpose
  return [r00, r10, r20, r01, r11, r21, r02, r12, r22];
}

class PrimList {
  constructor() { this.items = []; }
  // round cone (tapered capsule) between a and b with radii ra, rb
  cone(a, b, ra, rb, k, tag = '') { this.items.push({ t: CONE, a, b, ra, rb, k, tag }); return this; }
  // oriented ellipsoid; rot = [rx, ry, rz] Euler radians
  ell(c, r, rot = [0, 0, 0], k = 0.04, tag = '') { this.items.push({ t: ELL, c, r, rot, k, tag }); return this; }
}

// ---------------------------------------------------------------------------
// Body plan
// ---------------------------------------------------------------------------
function buildBody() {
  const S = SKELETON;
  const P = new PrimList();
  const D = Math.PI / 180;

  // --- trunk: thorax (ribcage + wall muscles), abdomen, loin
  P.ell([-0.02, 0.0, 0], [0.225, 0.168, 0.152], [0, 0, 0], 0.06, 'trunk');        // mid barrel (ribs 6-13)
  P.ell([0.17, 0.005, 0], [0.16, 0.158, 0.112], [0, 0, 4 * D], 0.06, 'trunk');    // cranial thorax, narrower between the forelegs
  P.ell([-0.165, -0.025, 0], [0.205, 0.152, 0.150], [0, 0, 0], 0.06, 'trunk');   // abdomen (rumen side)
  P.ell([-0.29, 0.065, 0], [0.13, 0.095, 0.122], [0, 0, -6 * D], 0.06, 'trunk');  // loin & croup
  // epaxial muscles (longissimus) along the topline, spinous processes
  P.cone([-0.37, 0.115, 0.035], [0.18, 0.155, 0.035], 0.04, 0.04, 0.05, 'trunk');
  P.cone([0.08, 0.15, 0], [0.23, 0.165, 0], 0.035, 0.03, 0.05, 'trunk');           // withers

  // --- pelvis landmarks
  P.ell(S.tuberCoxae, [0.035, 0.03, 0.03], [0, 0, 0], 0.045, 'pelvis');
  P.ell(S.pinBone, [0.035, 0.035, 0.03], [0, 0, 0], 0.045, 'pelvis');

  // --- hindquarter: gluteals, biceps femoris, semitendinosus/membranosus (the "leg")
  P.ell([-0.30, 0.04, 0.085], [0.11, 0.075, 0.06], [0, 0, -10 * D], 0.05, 'thigh');   // gluteus medius
  P.ell([-0.295, -0.07, 0.09], [0.105, 0.115, 0.062], [0, 0, -18 * D], 0.05, 'thigh'); // biceps femoris
  P.cone([-0.39, 0.05, 0.06], [-0.305, -0.15, 0.08], 0.06, 0.04, 0.05, 'thigh');      // hamstrings / breech
  P.cone([-0.28, 0.02, 0.105], [-0.205, -0.135, 0.10], 0.055, 0.04, 0.05, 'thigh');   // quadriceps
  P.ell(S.stifle, [0.035, 0.035, 0.032], [0, 0, 0], 0.03, 'thigh');

  // --- crus (gaskin) and hock
  P.cone([-0.225, -0.15, 0.095], [-0.33, -0.305, 0.076], 0.045, 0.02, 0.035, 'hindleg');  // tibia + flexors
  P.cone([-0.26, -0.13, 0.088], [-0.355, -0.298, 0.075], 0.045, 0.012, 0.03, 'hindleg');  // gastrocnemius -> Achilles
  P.ell(S.hock, [0.024, 0.026, 0.022], [0, 0, 0], 0.015, 'hindleg');
  P.cone(S.hock, S.hindFetlock, 0.017, 0.0145, 0.012, 'hindleg');                           // metatarsus (cannon)
  P.ell(S.hindFetlock, [0.019, 0.019, 0.018], [0, 0, 0], 0.01, 'hindleg');
  P.cone(S.hindFetlock, [-0.305, -0.488, 0.075], 0.0145, 0.014, 0.008, 'hindleg');          // pastern

  // --- shoulder: scapula with supraspinatus / infraspinatus / deltoid
  P.ell([0.235, 0.06, 0.10], [0.065, 0.125, 0.038], [0, 0, 30 * D], 0.05, 'shoulder');
  P.ell(S.shoulder, [0.045, 0.045, 0.04], [0, 0, 0], 0.04, 'shoulder');                  // point of shoulder
  P.ell([0.235, -0.075, 0.095], [0.07, 0.075, 0.05], [0, 0, -15 * D], 0.045, 'shoulder'); // triceps
  P.ell([0.30, -0.10, 0.0], [0.085, 0.07, 0.085], [0, 0, 0], 0.05, 'brisket');            // pectorals / brisket
  P.ell([0.20, -0.15, 0.09], [0.03, 0.03, 0.028], [0, 0, 0], 0.03, 'foreleg');            // olecranon

  // --- forearm, carpus, cannon
  P.cone([0.22, -0.15, 0.095], S.carpus, 0.035, 0.019, 0.03, 'foreleg');
  P.ell(S.carpus, [0.022, 0.025, 0.021], [0, 0, 0], 0.012, 'foreleg');
  P.cone(S.carpus, S.foreFetlock, 0.0165, 0.0145, 0.01, 'foreleg');
  P.ell(S.foreFetlock, [0.019, 0.019, 0.018], [0, 0, 0], 0.01, 'foreleg');
  P.cone(S.foreFetlock, [0.25, -0.488, 0.085], 0.0145, 0.014, 0.008, 'foreleg');

  // --- cloven hooves: two claws per foot
  for (const [hx, hz] of [[0.262, 0.085], [-0.298, 0.075]]) {
    P.ell([hx, -0.505, hz + 0.0115], [0.03, 0.02, 0.011], [0, 0, -18 * D], 0.006, 'hoof');
    P.ell([hx, -0.505, hz - 0.0115], [0.03, 0.02, 0.011], [0, 0, -18 * D], 0.006, 'hoof');
  }

  // --- neck (brachiocephalicus, sternocephalicus, splenius over the cervical vertebrae)
  P.cone([0.24, 0.05, 0], [0.47, 0.235, 0], 0.105, 0.062, 0.07, 'neck');
  P.cone([0.20, 0.12, 0], [0.48, 0.285, 0], 0.05, 0.04, 0.06, 'neck');   // crest
  P.cone([0.30, -0.02, 0], [0.50, 0.19, 0], 0.07, 0.045, 0.06, 'neck');  // throat / jugular groove

  // --- head (Merino: broad poll, slightly Roman nose, deep jaw, short muzzle)
  P.ell([0.545, 0.268, 0], [0.07, 0.062, 0.052], [0, 0, -40 * D], 0.03, 'head');           // cranium
  P.cone([0.56, 0.255, 0], [0.682, 0.162, 0], 0.05, 0.031, 0.03, 'head');                  // nasal bones / face
  P.ell([0.622, 0.228, 0], [0.055, 0.022, 0.03], [0, 0, -38 * D], 0.025, 'head');          // Roman nose
  P.cone([0.53, 0.205, 0.034], [0.66, 0.138, 0.016], 0.034, 0.019, 0.025, 'head');         // mandible
  P.ell([0.565, 0.212, 0.034], [0.044, 0.036, 0.024], [0, 0, -30 * D], 0.02, 'head');      // masseter
  P.ell([0.69, 0.152, 0], [0.026, 0.03, 0.03], [0, 0, -20 * D], 0.02, 'head');             // nose pad / lips
  P.ell([0.582, 0.284, 0.034], [0.022, 0.016, 0.018], [0, 0, -30 * D], 0.015, 'head');     // brow / orbit rim

  // --- docked tail
  P.cone(S.tailRoot, S.tailTip, 0.035, 0.028, 0.03, 'tail');

  // subtractive details
  const SUB = new PrimList();
  SUB.ell([0.708, 0.165, 0.014], [0.009, 0.005, 0.005], [0, 0, -45 * D], 0.004, 'nostril');
  SUB.ell([0.698, 0.126, 0.0], [0.03, 0.003, 0.03], [0, 0, -20 * D], 0.003, 'mouth');
  return { add: P.items, sub: SUB.items };
}

// ---------------------------------------------------------------------------
// Compiled evaluator
// ---------------------------------------------------------------------------
function compile(items) {
  const out = [];
  for (const it of items) {
    if (it.t === CONE) {
      const [ax, ay, az] = it.a, [bx, by, bz] = it.b;
      const bax = bx - ax, bay = by - ay, baz = bz - az;
      const l2 = bax * bax + bay * bay + baz * baz;
      const rr = it.ra - it.rb;
      out.push({
        t: CONE, ax, ay, az, bax, bay, baz, l2, rr, a2: l2 - rr * rr, il2: 1 / l2, ra: it.ra, rb: it.rb, k: it.k,
        cx: (ax + bx) / 2, cy: (ay + by) / 2, cz: (az + bz) / 2,
        bound: Math.sqrt(l2) / 2 + Math.max(it.ra, it.rb), tag: it.tag,
      });
    } else {
      const m = rotMatrix(...it.rot);
      out.push({
        t: ELL, cx: it.c[0], cy: it.c[1], cz: it.c[2], m, rx: it.r[0], ry: it.r[1], rz: it.r[2], k: it.k,
        bound: Math.max(...it.r), tag: it.tag,
      });
    }
  }
  return out;
}

function evalPrim(q, x, y, z) {
  if (q.t === CONE) {
    // Inigo Quilez round cone
    const pax = x - q.ax, pay = y - q.ay, paz = z - q.az;
    const yy = pax * q.bax + pay * q.bay + paz * q.baz;
    const zz = yy - q.l2;
    const xvx = pax * q.l2 - q.bax * yy, xvy = pay * q.l2 - q.bay * yy, xvz = paz * q.l2 - q.baz * yy;
    const x2 = xvx * xvx + xvy * xvy + xvz * xvz;
    const y2 = yy * yy * q.l2, z2 = zz * zz * q.l2;
    const k = Math.sign(q.rr) * q.rr * q.rr * x2;
    if (Math.sign(zz) * q.a2 * z2 > k) return Math.sqrt(x2 + z2) * q.il2 - q.rb;
    if (Math.sign(yy) * q.a2 * y2 < k) return Math.sqrt(x2 + y2) * q.il2 - q.ra;
    return (Math.sqrt(x2 * q.a2 * q.il2) + yy * q.rr) * q.il2 - q.ra;
  }
  const dx = x - q.cx, dy = y - q.cy, dz = z - q.cz, m = q.m;
  const lx = m[0] * dx + m[1] * dy + m[2] * dz;
  const ly = m[3] * dx + m[4] * dy + m[5] * dz;
  const lz = m[6] * dx + m[7] * dy + m[8] * dz;
  const ax = lx / q.rx, ay = ly / q.ry, az = lz / q.rz;
  const k0 = Math.sqrt(ax * ax + ay * ay + az * az);
  const bx = ax / q.rx, by = ay / q.ry, bz = az / q.rz;
  const k1 = Math.sqrt(bx * bx + by * by + bz * bz);
  if (k1 < 1e-9) return -Math.min(q.rx, q.ry, q.rz);
  return k0 * (k0 - 1) / k1;
}

function smin(a, b, k) {
  const h = Math.max(k - Math.abs(a - b), 0) / k;
  return Math.min(a, b) - h * h * k * 0.25;
}

const BODY = buildBody();
const ADD = compile(BODY.add);
const SUB = compile(BODY.sub);

// neck axis for Merino skin folds
const NECK_A = [0.27, 0.07, 0], NECK_B = [0.47, 0.235, 0];
const NECK_AX = [NECK_B[0] - NECK_A[0], NECK_B[1] - NECK_A[1], 0];
const NECK_LEN = Math.hypot(NECK_AX[0], NECK_AX[1]);
NECK_AX[0] /= NECK_LEN; NECK_AX[1] /= NECK_LEN;

/** Signed distance (m) from the skin surface; negative inside. */
export function sheepSDF(x, y, z) {
  z = Math.abs(z);
  let d = 1e9;
  for (let i = 0; i < ADD.length; i++) {
    const q = ADD[i];
    const dx = x - q.cx, dy = y - q.cy, dz = z - q.cz;
    const lb = Math.sqrt(dx * dx + dy * dy + dz * dz) - q.bound;
    if (lb > d + q.k) continue; // cannot influence the blend
    d = smin(d, evalPrim(q, x, y, z), q.k);
  }
  for (let i = 0; i < SUB.length; i++) {
    const q = SUB[i];
    const dx = x - q.cx, dy = y - q.cy, dz = z - q.cz;
    if (Math.sqrt(dx * dx + dy * dy + dz * dz) - q.bound > 0.02) continue;
    d = -smin(-d, evalPrim(q, x, y, z), q.k);
  }
  // Merino neck folds: transverse skin wrinkles around the neck
  const px = x - NECK_A[0], py = y - NECK_A[1];
  const t = px * NECK_AX[0] + py * NECK_AX[1];
  if (t > 0 && t < NECK_LEN) {
    const rx = px - NECK_AX[0] * t, ry = py - NECK_AX[1] * t;
    const radial = Math.sqrt(rx * rx + ry * ry + z * z);
    if (radial < 0.14) {
      const u = t / NECK_LEN;
      const mask = smooth(0.08, 0.25, u) * (1 - smooth(0.7, 0.92, u)) * (1 - smooth(0.09, 0.14, radial));
      // folds are deeper on the underside/sides of the neck
      const under = 0.6 + 0.4 * Math.max(0, -ry / (radial + 1e-6));
      d += 0.0045 * mask * under * Math.sin(u * NECK_LEN / 0.034 * Math.PI * 2 + z * 8);
    }
  }
  // flat soles on the ground
  d = Math.max(d, GROUND_Y - y);
  return d;
}

export function sdfGrad(x, y, z, out, e = 2e-4) {
  const gx = sheepSDF(x + e, y, z) - sheepSDF(x - e, y, z);
  const gy = sheepSDF(x, y + e, z) - sheepSDF(x, y - e, z);
  const gz = sheepSDF(x, y, z + e) - sheepSDF(x, y, z - e);
  const l = Math.hypot(gx, gy, gz) || 1;
  out[0] = gx / l; out[1] = gy / l; out[2] = gz / l;
  return out;
}

export function smooth(e0, e1, x) {
  const t = Math.min(1, Math.max(0, (x - e0) / (e1 - e0)));
  return t * t * (3 - 2 * t);
}

/** Which anatomical primitive group dominates near p (used for colouring). */
export function nearestTag(x, y, z) {
  z = Math.abs(z);
  let best = 1e9, tag = '';
  for (const q of ADD) {
    const d = evalPrim(q, x, y, z);
    if (d < best) { best = d; tag = q.tag; }
  }
  return tag;
}

// ---------------------------------------------------------------------------
// Wool growth: staple length (m) after ~11 months of growth, by body region.
// Merino: long dense staple over back and sides, shorter belly wool, wool on the
// legs fading out above knee and hock, wool over the poll but an open face.
// ---------------------------------------------------------------------------
export function stapleLength(x, y, z, nx, ny, nz) {
  const az = Math.abs(z);
  let L = 0.088;
  // belly wool is shorter and finer
  const belly = smooth(-0.25, -0.75, ny) * smooth(0.36, 0.22, x) * smooth(-0.36, -0.22, x);
  L = L * (1 - belly) + 0.055 * belly;
  // britch (hind thigh) wool is slightly longer and coarser
  L += 0.008 * smooth(-0.26, -0.36, x) * smooth(-0.15, 0.0, y);
  // neck wool shorter toward the head
  const neckT = Math.max(0, ((x - 0.27) * NECK_AX[0] + (y - 0.07) * NECK_AX[1]) / NECK_LEN);
  if (x > 0.25) L *= 1 - 0.25 * Math.min(1, neckT);
  // poll/topknot: wool stops behind the eyes, face is open
  const headFade = 1 - smooth(0.505, 0.55, x + 0.35 * (y - 0.27));
  L *= headFade;
  if (x > 0.48 && y < 0.24) L *= 1 - smooth(0.46, 0.52, x); // jowls/under-jaw bare
  // forelegs: wool fades out above the knee
  if (x > 0.1 && y < -0.05) L *= smooth(-0.27, -0.17, y);
  // hind legs: wool down to just above the hock (leg wool)
  if (x < -0.12 && y < -0.08) L *= smooth(-0.29, -0.2, y);
  // armpit and inner-thigh skin is bare-ish
  if (y < -0.1 && az < 0.08 && ny < -0.3) {
    const fore = smooth(0.16, 0.22, x), hind = smooth(-0.16, -0.22, x);
    L *= 1 - 0.55 * Math.max(fore, hind) * smooth(-0.12, -0.18, y);
  }
  // bare patch under the tail (crutch)
  const cx = x + 0.44, cy = y - 0.02;
  const cr = Math.sqrt(cx * cx + cy * cy + az * az);
  L *= smooth(0.03, 0.06, cr);
  // pizzle/udder region: skirted belly wool fades between the hind legs
  L *= 1 - 0.7 * smooth(-0.14, -0.2, y) * smooth(-0.1, -0.22, x) * smooth(0.07, 0.02, az);
  return Math.max(0, L);
}

export const BOUNDS = { min: [-0.53, -0.53, -0.19], max: [0.76, 0.37, 0.19] };
