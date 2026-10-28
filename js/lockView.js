// Renders the fleece as locks: one tapered, crimped, lumpy tuft per simulated
// column, drawn in a single instanced call. The GPU builds each tuft from the
// column's root (base particle) and tip particle, read straight from the
// simulation's position array, so the CPU does almost nothing per frame.
//
// A tuft's root cap is the creamy cut face; its sides carry fibre streaks and
// crimp; its tip end is dusty and ragged (alpha-tested). Partings are simply the
// gaps between neighbouring tufts, so stretched wool opens up between locks.
import * as THREE from 'three';
import { NOISE } from './materials.js';

const SIDES = 6;      // around the tuft
const RINGS = 4;      // along it (root to tip)

// Per-instance: iRoot/iTip (live), iInfo = (radius, seed, dust, length), iRest (noise space)
const LOCK_VERTEX = /* glsl */`
attribute vec3 iRoot; attribute vec3 iTip; attribute vec4 iInfo; attribute vec3 iRest; attribute float aCap;
varying float vU; varying float vSide; varying float vCap; varying float vSeed; varying float vDust; varying vec3 vRest; varying vec2 vLoc;
float lh(float n){ return fract(sin(n) * 43758.5453); }
// position (and outward normal) of a tuft vertex; position.xyz = (cos a, sin a, u) scaled by the
// distance from the axis; aCap = 1 on the root cap (cut face), 2 on the domed crown (weathered tip)
void lockPoint(out vec3 p, out vec3 n){
  vec3 axis = iTip - iRoot;
  float L = max(length(axis), 1e-4);
  vec3 d = axis / L;
  vec3 ref = abs(d.y) < 0.9 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
  vec3 e1 = normalize(cross(d, ref)), e2 = cross(d, e1);
  float u = position.z, cap = aCap, seed = iInfo.y;
  float a = atan(position.y, position.x);
  // lumpy cross-section, a slight bulge, and a taper to a ragged point
  float k = floor((a + 3.14159) / 6.28318 * ${SIDES}.0 + 0.5);
  float lump = 0.82 + 0.36 * lh(seed * 17.0 + k * 3.1 + floor(u * ${RINGS - 1}.0) * 0.7);
  // blunt Merino staples: nearly as wide at the tip as at the root
  float rr = length(position.xy);                 // 0 at the cap / crown centre
  float r = iInfo.x * lump * (0.92 + 0.15 * sin(3.14159 * u)) * (1.0 - 0.22 * smoothstep(0.75, 1.0, u)) * rr;
  // crimp: the whole lock waves gently along its length
  float waves = 2.0 + 1.5 * lh(seed * 5.3);
  float crimp = sin(u * waves * 6.28318 + seed * 6.28318) * iInfo.x * 0.35 * smoothstep(0.0, 0.25, u);
  // each lock is turned a random way so the facets of neighbours never line up
  float roll = seed * 6.28318, cr = cos(roll), sr = sin(roll);
  vec3 side = e1 * (position.x * cr - position.y * sr) + e2 * (position.x * sr + position.y * cr);
  // the crown domes up a little over the tip ring
  float dome = cap > 1.5 ? (1.0 - rr * rr) * iInfo.x * 0.3 : 0.0;
  p = iRoot + d * (u * L + dome) + side * r + e1 * crimp;
  n = cap > 1.5 ? normalize(d * (1.2 - rr) + side * rr * 0.9) : cap > 0.5 ? -d : normalize(side);
  vU = u; vSide = a; vCap = cap; vSeed = seed; vDust = iInfo.z; vRest = iRest; vLoc = position.xy * iInfo.x;
}
`;

const LOCK_FRAGMENT = /* glsl */`
varying float vU; varying float vSide; varying float vCap; varying float vSeed; varying float vDust; varying vec3 vRest; varying vec2 vLoc;
const vec3 CREAM = vec3(0.9, 0.83, 0.68);
vec3 lockColour(out float hgt){
  // fibre streaks run root -> tip; crimp shows as faint bands across them
  float strand = vn(vec3(vSide * 9.0, vU * 2.0, vSeed * 37.0)) * 0.6 + vn(vec3(vSide * 31.0, vU * 5.0, vSeed * 11.0)) * 0.4;
  float bands = sin(vU * 60.0 + strand * 6.0);
  // suint and dust: patchy over the fleece, heaviest along the back
  float dirt = clamp(0.3 + vDust * 0.6 + (vn(vRest * 6.0) - 0.5) * 0.8 + (vn(vRest * 23.0) - 0.5) * 0.3, 0.0, 1.0);
  vec3 tip = mix(vec3(0.78, 0.66, 0.42), vec3(0.45, 0.34, 0.2), dirt);
  float tipZone = smoothstep(0.6, 1.0, vU);
  vec3 col = mix(CREAM * (0.82 + 0.25 * strand), tip * (0.85 + 0.3 * strand), tipZone);
  col *= 0.94 + 0.06 * bands;
  // deep in the fleece the sides of a lock are shaded by its neighbours
  col *= mix(0.32, 1.0, smoothstep(0.05, 0.85, vU));
  hgt = strand * 0.7 + bands * 0.1;
  if (vCap > 1.5) {
    // weathered crown: crusted, dusty fibre tips
    // fibre ends (fine stipple), crimp waves across the lock, crusted clumps
    // (vLoc: position across the lock in metres)
    vec3 q = vec3(vLoc, vSeed * 7.0);
    vec3 ql = vec3(vLoc, 0.0);
    float fine = mix(0.5, vn(q * vec3(1400.0, 1400.0, 1.0)), aaFade(ql, 1400.0));
    float mid = mix(0.5, vn(q * vec3(420.0, 420.0, 1.0) + 3.0), aaFade(ql, 420.0));
    float tips = fine * 0.45 + mid * 0.35 + vn(q * vec3(130.0, 130.0, 1.0)) * 0.2;
    float crimpBands = sin(vLoc.x * 450.0 + vSeed * 20.0 + tips * 4.0) * aaFade(ql, 450.0 / 6.28);
    float crust = smoothstep(0.45, 0.8, vn(q * vec3(90.0, 90.0, 1.0) + 9.0));
    col = tip * (0.7 + 0.45 * tips) * (0.92 + 0.08 * crimpBands);
    col = mix(col, tip * 0.72, crust * 0.5);
    hgt = tips * 1.0 + crimpBands * 0.25 - crust * 0.2;
  } else if (vCap > 0.5) {
    // cut face: dense creamy fibre ends
    vec3 q = vec3(vLoc, vSeed * 7.0);
    vec3 ql = vec3(vLoc, 0.0);
    float ends = mix(0.5, vn(q * vec3(1600.0, 1600.0, 1.0)), aaFade(ql, 1600.0)) * 0.5
               + mix(0.5, vn(q * vec3(500.0, 500.0, 1.0) + 5.0), aaFade(ql, 500.0)) * 0.5;
    col = CREAM * (0.88 + 0.16 * ends);
    hgt = ends * 0.5;
  }
  return col;
}
`;

function patchVertex(shader, withNormal) {
  shader.vertexShader = shader.vertexShader
    .replace('#include <common>', `#include <common>\n${LOCK_VERTEX}`)
    .replace('#include <beginnormal_vertex>', withNormal
      ? 'vec3 lockP, lockN; lockPoint(lockP, lockN); vec3 objectNormal = lockN;'
      : '#include <beginnormal_vertex>')
    .replace('#include <begin_vertex>', withNormal
      ? 'vec3 transformed = lockP;'
      : 'vec3 lockP, lockN; lockPoint(lockP, lockN); vec3 transformed = lockP;');
}

function makeLockMaterial() {
  const m = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 1, metalness: 0 });
  m.onBeforeCompile = (shader) => {
    patchVertex(shader, true);
    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', `#include <common>\n${NOISE}\n${LOCK_FRAGMENT}`)
      .replace('#include <color_fragment>', `#include <color_fragment>
        // ragged tip ends: fibres stop at different lengths
        float lockH;
        diffuseColor.rgb = lockColour(lockH);`)
      .replace('#include <normal_fragment_maps>', `#include <normal_fragment_maps>
        normal = bumpNormal(-vViewPosition, normal, lockH, 0.007);`);
  };
  m.customProgramCacheKey = () => 'wool-locks';
  return m;
}

function makeLockDepthMaterial() {
  const m = new THREE.MeshDepthMaterial({ depthPacking: THREE.RGBADepthPacking });
  m.onBeforeCompile = (shader) => patchVertex(shader, false);
  m.customProgramCacheKey = () => 'wool-locks-depth';
  return m;
}

/** Unit tuft: rings of SIDES vertices from root (u = 0) to tip (u = 1), a root cap and a tip point. */
function tuftGeometry() {
  const pos = [], cap = [], idx = [];
  for (let k = 0; k < RINGS; k++) {
    const u = k / (RINGS - 1);
    for (let s = 0; s < SIDES; s++) {
      const a = (s / SIDES) * Math.PI * 2;
      pos.push(Math.cos(a), Math.sin(a), u); cap.push(0);
    }
  }
  for (let k = 0; k < RINGS - 1; k++) for (let s = 0; s < SIDES; s++) {
    const a = k * SIDES + s, b = k * SIDES + (s + 1) % SIDES, c = a + SIDES, d = b + SIDES;
    idx.push(a, c, b, b, c, d);
  }
  // root cap (cut face): its own vertices so it can shade differently
  const c0 = pos.length / 3;
  pos.push(0, 0, 0); cap.push(1);
  for (let s = 0; s < SIDES; s++) {
    const a = (s / SIDES) * Math.PI * 2;
    pos.push(Math.cos(a), Math.sin(a), 0); cap.push(1);
  }
  for (let s = 0; s < SIDES; s++) idx.push(c0, c0 + 1 + (s + 1) % SIDES, c0 + 1 + s);
  // domed crown over the tip: centre and rim
  const t0 = pos.length / 3;
  pos.push(0, 0, 1); cap.push(2);
  for (const rr of [1]) for (let s = 0; s < SIDES; s++) {
    const a = (s / SIDES) * Math.PI * 2;
    pos.push(Math.cos(a) * rr, Math.sin(a) * rr, 1); cap.push(2);
  }
  for (let s = 0; s < SIDES; s++) {
    idx.push(t0, t0 + 1 + s, t0 + 1 + (s + 1) % SIDES);
  }
  const g = new THREE.InstancedBufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('aCap', new THREE.Float32BufferAttribute(cap, 1));
  g.setIndex(idx);
  return g;
}

export class LockView {
  constructor(fleece, { shadows = true } = {}) {
    this.f = fleece;
    const N = fleece.N;
    const g = tuftGeometry();
    // live root / tip positions: views straight into the simulation's position array
    this.rootAttr = new THREE.InstancedBufferAttribute(fleece.pos.subarray(0, 3 * N), 3).setUsage(THREE.DynamicDrawUsage);
    this.tipAttr = new THREE.InstancedBufferAttribute(fleece.pos.subarray(3 * N, 6 * N), 3).setUsage(THREE.DynamicDrawUsage);
    g.setAttribute('iRoot', this.rootAttr);
    g.setAttribute('iTip', this.tipAttr);
    // static per lock: radius from the local lattice spacing, seed, dust load, rest position
    const info = new Float32Array(4 * N), rest = new Float32Array(3 * N);
    const bp = fleece.basePos, spacing = new Float32Array(N), cnt = new Float32Array(N);
    for (let e = 0; e < fleece.E; e++) {
      const a = fleece.eA[e], b = fleece.eB[e];
      const l = Math.hypot(bp[3 * b] - bp[3 * a], bp[3 * b + 1] - bp[3 * a + 1], bp[3 * b + 2] - bp[3 * a + 2]);
      spacing[a] += l; spacing[b] += l; cnt[a]++; cnt[b]++;
    }
    for (let i = 0; i < N; i++) {
      const s = cnt[i] ? spacing[i] / cnt[i] : 0.025;
      const ny = fleece.restN[3 * i + 1];
      info[4 * i] = 0.85 * s;                                           // neighbouring locks press together
      info[4 * i + 1] = (Math.sin(i * 12.9898) * 43758.5453) % 1;       // seed
      info[4 * i + 2] = Math.min(1, Math.max(0, (ny + 0.3) / 1.1));      // dust: most along the back
      info[4 * i + 3] = fleece.L[i];
      rest[3 * i] = bp[3 * i]; rest[3 * i + 1] = bp[3 * i + 1]; rest[3 * i + 2] = bp[3 * i + 2];
    }
    g.setAttribute('iInfo', new THREE.InstancedBufferAttribute(info, 4));
    g.setAttribute('iRest', new THREE.InstancedBufferAttribute(rest, 3));
    g.instanceCount = N;
    this.geometry = g;
    this.mesh = new THREE.Mesh(g, makeLockMaterial());
    this.mesh.customDepthMaterial = makeLockDepthMaterial();
    this.mesh.frustumCulled = false;
    this.mesh.castShadow = shadows; this.mesh.receiveShadow = shadows;
    this.group = new THREE.Group();
    this.group.add(this.mesh);
  }

  setShadows(on) { this.mesh.castShadow = on; this.mesh.receiveShadow = on; }
  setShellsVisible() {}

  update() {
    this.rootAttr.needsUpdate = true;
    this.tipAttr.needsUpdate = true;
  }

  dispose() {
    this.geometry.dispose();
    this.mesh.material.dispose();
    this.mesh.customDepthMaterial.dispose();
  }
}
