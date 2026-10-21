// Shearing pattern and automatic sheep handling. The wool is divided into
// anatomical areas; when the area being worked is (nearly) finished the sheep
// is turned to present the next area, roughly following the Bowen pattern.

export const REGION_ORDER = [
  'Belly', 'Right hind leg', 'Left hind leg', 'Crutch & tail', 'Left foreleg', 'Left neck',
  'Left side', 'Back', 'Right neck', 'Right foreleg', 'Right side',
];

// Which way is "up" for the sheep when each area is presented (sheep-local).
// Mirrors real shearing positions: belly and hind legs with the sheep sat on
// its rump (head up), neck with the head held up, sides with the back up.
const PRESENT_UP = {
  'Belly': [1, 0.3, 0], 'Right hind leg': [1, 0.2, 0], 'Left hind leg': [1, 0.2, 0],
  'Crutch & tail': [0, 1, 0], 'Left foreleg': [0.6, 1, 0], 'Right foreleg': [0.6, 1, 0],
  'Left neck': [1, 0.5, 0], 'Right neck': [1, 0.5, 0],
  'Left side': [0.3, 1, 0], 'Right side': [0.3, 1, 0], 'Back': [1, 0, 0],
};

export function classify(x, y, z, nx, ny, nz) {
  const left = z >= 0;
  if (x < -0.37 || (x < -0.3 && y > -0.05 && Math.abs(z) < 0.07)) return 'Crutch & tail';
  if (x < -0.13 && y < -0.08 && Math.abs(z) > 0.035) return left ? 'Left hind leg' : 'Right hind leg';
  if (x > 0.13 && x < 0.32 && y < -0.1 && Math.abs(z) > 0.05) return left ? 'Left foreleg' : 'Right foreleg';
  if (x > 0.27 && y > -0.06) return left ? 'Left neck' : 'Right neck';
  if (ny < -0.45) return 'Belly';
  if (ny > 0.62) return 'Back';
  return left ? 'Left side' : 'Right side';
}

export class Director {
  constructor(fleece) {
    this.setFleece(fleece);
    this.anim = null;
    this.current = null;
  }

  setFleece(f) {
    this.f = f;
    const N = f.N;
    this.regionOf = new Array(N);
    this.regions = {};
    for (const name of REGION_ORDER) this.regions[name] = { name, total: 0, cut: 0, c: [0, 0, 0], n: [0, 0, 0] };
    for (let i = 0; i < N; i++) {
      const p = f.basePos, n = f.restN;
      const r = classify(p[3 * i], p[3 * i + 1], p[3 * i + 2], n[3 * i], n[3 * i + 1], n[3 * i + 2]);
      this.regionOf[i] = r;
      const R = this.regions[r];
      R.total++;
      for (let k = 0; k < 3; k++) { R.c[k] += p[3 * i + k]; R.n[k] += n[3 * i + k]; }
    }
    for (const R of Object.values(this.regions)) {
      if (!R.total) continue;
      for (let k = 0; k < 3; k++) R.c[k] /= R.total;
      const l = Math.hypot(...R.n) || 1;
      for (let k = 0; k < 3; k++) R.n[k] /= l;
    }
    this.recent = [];
    this.current = null;
  }

  onCut(i) {
    const r = this.regions[this.regionOf[i]];
    r.cut++;
    this.recent.push(r.name);
    if (this.recent.length > 40) this.recent.shift();
  }

  fraction(name) { const r = this.regions[name]; return r.total ? r.cut / r.total : 1; }

  /** The area the shearer is working on, judged from recent cuts. */
  workingRegion() {
    if (!this.recent.length) return this.current;
    const count = {};
    for (const r of this.recent) count[r] = (count[r] || 0) + 1;
    return Object.entries(count).sort((a, b) => b[1] - a[1])[0][0];
  }

  nextRegion(threshold, after = null) {
    const list = REGION_ORDER.filter((n) => this.regions[n].total > 0);
    let start = after ? list.indexOf(after) + 1 : 0;
    for (let k = 0; k < list.length; k++) {
      const n = list[(start + k) % list.length];
      if (this.fraction(n) < threshold) return n;
    }
    return null;
  }

  /**
   * Orientation (row-major 3x3, local->world) presenting `name` to a camera
   * looking down -Z, keeping the sheep upright when practical.
   */
  targetRotation(name, faceDir = [0, -0.35, 1]) {
    // faceDir: where the area's surface should point. Slightly downward so the
    // severed fleece falls away from the skin under its own weight.
    const R = this.regions[name];
    const n = R.n;
    const v = normalize(faceDir);
    let ref = PRESENT_UP[name] || [0, 1, 0];
    if (Math.abs(dot(normalize(ref), n)) > 0.9) ref = [0, 1, 0];
    // local orthonormal frame {a, b, n}: b = ref projected
    let b = sub(ref, scale(n, dot(ref, n))); b = normalize(b);
    const a = cross(b, n);
    // world frame {A, B, V}: B = world up projected perpendicular to v
    let B = sub([0, 1, 0], scale(v, v[1])); B = normalize(B);
    const A = cross(B, v);
    // R maps a->A, b->B, n->v : R = [A B V] [a b n]^T
    const M = new Array(9);
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) M[3 * r + c] = A[r] * a[c] + B[r] * b[c] + v[r] * n[c];
    return M;
  }
}

function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
function scale(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function normalize(a) { const l = Math.hypot(...a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; }
