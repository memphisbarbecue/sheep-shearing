import * as THREE from 'three';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { Rig, SheepBehaviour } from './rig.js';
import { SheepBody } from './sheep.js';
import { Fleece, DEFAULT_PHYSICS } from './fleece.js';
import { FleeceView } from './fleeceView.js';
import { Clipper, COMB_WIDTH } from './clipper.js';
import { cutterPass } from './cutter.js';
import { Director, REGION_ORDER } from './director.js';
import { ClipperAudio } from './audio.js';
import { sdfGrad, sheepSDF } from './anatomy.js';
import { loadSettings, buildSettingsPanel, bindingLabel, RESOLUTIONS, REFERENCE_RESOLUTION } from './settings.js';

const FLOOR_Y = -0.95;
const VIEW_DIR = new THREE.Vector3(0, 0.3, 1).normalize();   // camera direction from its target
const FACE_DIR = [0, -0.32, 1];                                 // presented area faces the shearer, slightly down

const settings = loadSettings();
const $ = (id) => document.getElementById(id);

// ---------------------------------------------------------------- renderer
const canvas = $('scene');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, settings.pixelRatio));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 0.95;
renderer.shadowMap.enabled = settings.shadows;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x2a2520);
scene.fog = new THREE.Fog(0x2a2520, 4, 9);
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
scene.environmentIntensity = 0.45;

const camera = new THREE.PerspectiveCamera(38, window.innerWidth / window.innerHeight, 0.05, 30);
let camDist = 2.3;
const camTarget = new THREE.Vector3(0, -0.45, 0);
const camTargetGoal = camTarget.clone();

// lights: shed skylight + a key light over the board
scene.add(new THREE.HemisphereLight(0xfff4e0, 0x5a4632, 0.75));
const key = new THREE.DirectionalLight(0xfff0dc, 2.0);
key.position.set(-1.2, 2.6, 1.8);
key.castShadow = true;
key.shadow.mapSize.set(2048, 2048);
key.shadow.camera.left = -1.4; key.shadow.camera.right = 1.4;
key.shadow.camera.top = 1.4; key.shadow.camera.bottom = -1.4;
key.shadow.camera.near = 0.5; key.shadow.camera.far = 7;
key.shadow.bias = -0.0004; key.shadow.normalBias = 0.01;
scene.add(key);
const fill = new THREE.DirectionalLight(0xdfe8ff, 0.55);
fill.position.set(2, 0.8, 1.2);
scene.add(fill);
const rim = new THREE.DirectionalLight(0xffe2b8, 1.0);
rim.position.set(0.5, 1.5, -2.5);
scene.add(rim);

// shearing board
scene.add(makeBoard());

// ---------------------------------------------------------------- sheep
const rig = new Rig();
const behaviour = new SheepBehaviour(rig);
const sheepRoot = new THREE.Group();
scene.add(sheepRoot);
let body, fleece, fleeceView, director;

// Physics runs in a worker; `fleece` on this thread is a non-simulating mirror.
const sim = {
  worker: new Worker(new URL('./fleeceWorker.js', import.meta.url), { type: 'module' }),
  id: 0, ready: false, inFlight: false, spare: null, releases: [], cuts: [],
  tears: 0, nDyn: 0, ms: 0,
};
sim.worker.onmessage = (ev) => {
  const m = ev.data;
  if (m.id !== sim.id) return; // from a previous sheep
  if (m.type === 'ready') { sim.ready = true; return; }
  if (m.type === 'state') {
    sim.inFlight = false;
    const pos = fleece.pos, dyn = fleece.dyn, src = m.pos;
    for (let p = 0, n = fleece.P; p < n; p++) {
      if (!dyn[p]) continue;
      const i3 = 3 * p;
      pos[i3] = src[i3]; pos[i3 + 1] = src[i3 + 1]; pos[i3 + 2] = src[i3 + 2];
    }
    sim.spare = src;
    for (const t of m.killed) fleece._killTri(t);
    fleece._killLog.length = 0; // the mirror doesn't need its own log
    fleece.stats.tears = m.tears;
    sim.nDyn = m.nDyn; sim.ms = m.ms;
  }
};
sim.worker.onerror = (e) => console.error('fleece worker:', e.message || e);
const clipper = new Clipper(scene);
clipper.setGhostVisible(settings.xray);
const audio = new ClipperAudio();
audio.volume = settings.volume;

const sheepQ = new THREE.Quaternion();
let anim = null;          // { from, to, t, dur }
let presented = null;     // region the sheep is currently turned to
let started = false;
let startTime = 0, finishTime = 0;
let piecesTimer = 0, pieces = 1;
let exit = null;          // end of shear: sheep walks out of the fleece

function physicsParams() {
  const e = settings.elasticity;
  return {
    // a finer lattice converges more slowly per substep: keep it as stiff as the reference
    substeps: Math.round(settings.substeps * (REFERENCE_RESOLUTION / (RESOLUTIONS[settings.resolution] || RESOLUTIONS.medium)) ** 2),
    innerCompliance: DEFAULT_PHYSICS.innerCompliance * e,
    outerCompliance: DEFAULT_PHYSICS.outerCompliance * e,
    shearCompliance: DEFAULT_PHYSICS.shearCompliance * e,
    stapleCompliance: DEFAULT_PHYSICS.stapleCompliance * e,
    tetherCompliance: DEFAULT_PHYSICS.tetherCompliance * e,
    tearRatio: 1 + (DEFAULT_PHYSICS.tearRatio - 1) * settings.fibreStrength,
    density: DEFAULT_PHYSICS.density * settings.fleeceWeight,
    selfCollision: settings.selfCollision,
  };
}

// The sheep rests on the board in whatever position it is turned to (sitting on
// its rump, lying on its side, ...): lift it so its lowest point - skin or
// still-attached wool - touches the floor.
const _m3 = new THREE.Matrix3();
function restOnBoard(dt) {
  _m3.setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(sheepQ));
  const e = _m3.elements; // column-major; world y row = e[1], e[4], e[7]
  const r0 = e[1], r1 = e[4], r2 = e[7];
  let minY = 1e9;
  const sk = body.rest;
  for (let i = 0; i < sk.length; i += 3 * 5) minY = Math.min(minY, r0 * sk[i] + r1 * sk[i + 1] + r2 * sk[i + 2]);
  const tl = fleece.restLocal, N = fleece.N, att = fleece.attached;
  for (let i = 0; i < N; i += 2) {
    if (!att[i]) continue;
    const o = 3 * (N + i);
    minY = Math.min(minY, r0 * tl[o] + r1 * tl[o + 1] + r2 * tl[o + 2]);
  }
  const goal = FLOOR_Y + 0.004 - minY;
  const y = sheepRoot.position.y;
  // settle down gently (as wool comes off) but never sink into the board
  sheepRoot.position.y = Math.max(goal, dt ? y + (goal - y) * Math.min(1, dt * 4) : goal);
}

function sheepMatrices() {
  sheepRoot.quaternion.copy(sheepQ);
  sheepRoot.updateMatrixWorld(true);
  const e = sheepRoot.matrixWorld.elements; // column-major
  return {
    R: [e[0], e[4], e[8], e[1], e[5], e[9], e[2], e[6], e[10]],
    t: [e[12], e[13], e[14]],
  };
}

function newFleece() {
  if (fleeceView) { scene.remove(fleeceView.group); fleeceView.dispose(); }
  const res = RESOLUTIONS[settings.resolution] || RESOLUTIONS.medium;
  const seed = Math.random() * 100;
  fleece = new Fleece({ rig, resolution: res, physics: physicsParams(), floorY: FLOOR_Y, seed, simulate: false });
  if (body) restOnBoard(0);
  const { R, t } = sheepMatrices();
  fleece.updateAnchors(R, t);
  fleece.reset();
  sim.id++; sim.ready = false; sim.inFlight = false; sim.releases = []; sim.cuts = []; sim.spare = null; blow = 0; inBlow = false;
  sim.worker.postMessage({ type: 'init', id: sim.id, resolution: res, physics: physicsParams(), floorY: FLOOR_Y, seed, R, t });
  fleeceView = new FleeceView(fleece, { shells: 3, shadows: settings.shadows });
  fleeceView.setShellsVisible(settings.shells);
  scene.add(fleeceView.group);
  if (director) director.setFleece(fleece); else director = new Director(fleece);
  presented = null; anim = null; exit = null; autopilot.opened.clear();
  sheepRoot.position.set(0, sheepRoot.position.y, 0);
  startTime = 0; finishTime = 0; pieces = 1;
  $('done').classList.add('hidden');
  $('weight').textContent = fleece.totalMass.toFixed(1) + ' kg';
  updateHud(true);
}

async function init() {
  const status = (t) => { $('loading-text').textContent = t; return new Promise((r) => setTimeout(r, 30)); };
  await status('Building skeleton and musculature…');
  body = new SheepBody(rig, { shadows: settings.shadows });
  sheepRoot.add(body.group);
  await status('Growing eleven months of wool…');
  sheepQ.setFromEuler(new THREE.Euler(0, -0.55, 0));
  sheepRoot.position.y = FLOOR_Y + 0.52;
  newFleece();
  $('loading').classList.add('hidden');
  showIntro();
  requestAnimationFrame(loop);
}

// ---------------------------------------------------------------- input
const mouse = { x: 0, y: 0, ndc: new THREE.Vector2(), inside: false };
const held = new Set();      // 'mouse:N' / 'key:Code'
let shearing = false;
let rotating = false;
const isHeld = (b) => b && held.has(b.type === 'mouse' ? `mouse:${b.button}` : `key:${b.code}`);
const matches = (b, ev) => b && (ev.type.startsWith('mouse') ? b.type === 'mouse' && b.button === ev.button : b.type === 'key' && b.code === ev.code);
const uiOpen = () => !$('settings').classList.contains('hidden') || !$('intro').classList.contains('hidden');

function onPress(ev) {
  if (matches(settings.shearBinding, ev) && settings.shearMode === 'toggle') shearing = !shearing;
  if (matches(settings.nextRegionKey, ev)) gotoNextRegion(true);
}

window.addEventListener('mousedown', (e) => {
  if (e.target !== canvas) return;
  if (e.button === 1) e.preventDefault(); // no autoscroll
  held.add(`mouse:${e.button}`);
  onPress(e);
});
window.addEventListener('mouseup', (e) => held.delete(`mouse:${e.button}`));
canvas.addEventListener('contextmenu', (e) => e.preventDefault());
canvas.addEventListener('auxclick', (e) => e.preventDefault());
window.addEventListener('mousemove', (e) => {
  rotating = isHeld(settings.rotateBinding);
  if (rotating && !uiOpen()) {
    const s = 0.006 * settings.rotateSensitivity;
    rotateSheep(e.movementX * s * (settings.invertX ? -1 : 1), e.movementY * s * (settings.invertY ? -1 : 1));
    return; // clippers stay put while the sheep is turned
  }
  if (settings.clipperControl === 'drag' && !isHeld(settings.dragBinding)) return;
  mouse.x = e.clientX; mouse.y = e.clientY;
  mouse.ndc.set((e.clientX / window.innerWidth) * 2 - 1, -(e.clientY / window.innerHeight) * 2 + 1);
  mouse.inside = true;
});
window.addEventListener('keydown', (e) => {
  if (e.target.closest && e.target.closest('input, select, textarea')) return;
  if (e.code === 'Escape') { toggleSettings(); return; }
  if (uiOpen()) return;
  const bound = [settings.shearBinding, settings.rotateBinding, settings.nextRegionKey, settings.dragBinding].some((b) => matches(b, e));
  if (bound || e.code.startsWith('Arrow') || e.code === 'Space') e.preventDefault();
  if (e.repeat) return;
  held.add(`key:${e.code}`);
  onPress(e);
});
window.addEventListener('keyup', (e) => held.delete(`key:${e.code}`));
window.addEventListener('blur', () => { held.clear(); });
canvas.addEventListener('wheel', (e) => {
  if (!settings.wheelZoom) return;
  e.preventDefault();
  camDist = THREE.MathUtils.clamp(camDist * Math.exp(e.deltaY * 0.001), 1.1, 4.0);
}, { passive: false });
window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

const _qa = new THREE.Quaternion(), _qb = new THREE.Quaternion();
function rotateSheep(yaw, pitch) {
  anim = null;
  _qa.setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw);
  _qb.setFromAxisAngle(new THREE.Vector3(1, 0, 0), pitch);
  sheepQ.premultiply(_qb).premultiply(_qa);
}

function turnTo(region) {
  if (!region) return;
  const M = director.targetRotation(region, FACE_DIR);
  const m4 = new THREE.Matrix4().set(M[0], M[1], M[2], 0, M[3], M[4], M[5], 0, M[6], M[7], M[8], 0, 0, 0, 0, 1);
  const to = new THREE.Quaternion().setFromRotationMatrix(m4);
  const angle = 2 * Math.acos(Math.min(1, Math.abs(sheepQ.dot(to))));
  anim = { from: sheepQ.clone(), to, t: 0, dur: (0.5 + angle * 0.45) / settings.repositionSpeed };
  presented = region;
  autopilot.passes = 0;
  director.recent = [];
}

// re-count region progress from the fleece (columns may be released by the simulation itself)
function syncDirector() {
  for (const R of Object.values(director.regions)) R.cut = 0;
  for (let i = 0; i < fleece.N; i++) if (!fleece.attached[i]) director.regions[director.regionOf[i]].cut++;
}

function gotoNextRegion(manual) {
  const next = director.nextRegion(manual ? 1.0 : settings.regionThreshold, presented);
  if (next) turnTo(next);
}

// ---------------------------------------------------------------- clipper
const raycaster = new THREE.Raycaster();
const clip = {
  onSheep: false, pos: new THREE.Vector3(), lastPos: new THREE.Vector3(), lastValid: false,
  fwdLocal: new THREE.Vector3(1, 0, 0), normal: new THREE.Vector3(0, 1, 0), load: 0,
};
const _v = new THREE.Vector3(), _n = new THREE.Vector3(), _f = new THREE.Vector3(), _inv = new THREE.Quaternion();
const g3 = [0, 0, 0];
let lastCutTime = 0;
let blow = 0, inBlow = false;   // blow number: the cutter only cuts through wool shorn by earlier blows

function updateClipper(dt, now) {
  raycaster.setFromCamera(mouse.ndc, camera);
  const hits = mouse.inside ? raycaster.intersectObject(body.pickMesh, false) : [];
  _inv.copy(sheepQ).invert();
  if (hits.length) {
    const h = hits[0];
    // snap the coarse hit onto the exact skin along the SDF gradient
    _v.copy(h.point).sub(sheepRoot.position).applyQuaternion(_inv);   // sheep-local
    for (let k = 0; k < 2; k++) {
      const d = sheepSDF(_v.x, _v.y, _v.z);
      sdfGrad(_v.x, _v.y, _v.z, g3);
      _v.x -= g3[0] * d; _v.y -= g3[1] * d; _v.z -= g3[2] * d;
    }
    h.point.copy(_v).applyQuaternion(sheepQ).add(sheepRoot.position);
    _n.set(g3[0], g3[1], g3[2]).applyQuaternion(sheepQ);
    clip.normal.lerp(_n, clip.onSheep ? 0.5 : 1).normalize();
    // steer the comb along the direction of travel (kept in sheep-local space)
    if (clip.onSheep) {
      const d = _v.copy(h.point).sub(clip.pos);
      d.addScaledVector(clip.normal, -d.dot(clip.normal));
      if (d.length() > 0.0015) {
        _f.copy(clip.fwdLocal).applyQuaternion(sheepQ).lerp(d.normalize(), 0.35).normalize();
        clip.fwdLocal.copy(_f).applyQuaternion(_inv);
      }
    }
    clip.pos.copy(h.point);
    clip.onSheep = true;
  } else {
    // hover in front of the sheep
    const plane = new THREE.Plane(VIEW_DIR.clone(), -0.35 - VIEW_DIR.dot(sheepRoot.position));
    raycaster.ray.intersectPlane(plane, clip.pos);
    clip.onSheep = false;
  }
  _f.copy(clip.fwdLocal).applyQuaternion(sheepQ);
  clipper.setPose(clip.pos, clip.normal, _f);

  const running = held.size >= 0 && (settings.shearMode === 'hold' ? isHeld(settings.shearBinding) : shearing) && !uiOpen();
  clipper.running = running;
  let cuts = 0;
  // a blow starts each time the comb goes into the wool with the clippers running
  if (running && clip.onSheep && !inBlow) blow++;
  inBlow = running && clip.onSheep;
  if (inBlow) {
    if (!started) { started = true; }
    const from = clip.lastValid ? clip.lastPos : clip.pos;
    const { released, cutEdges } = cutterPass(fleece, from, clip.pos, clipper.basis, blow);
    for (const r of released) {
      sim.releases.push(r);
      director.onCut(r[0]);
    }
    for (const e of cutEdges) sim.cuts.push(e);
    cuts = released.length + cutEdges.length;
    if (cuts) {
      lastCutTime = now;
      if (!startTime) startTime = now;
    }
  }
  // the comb/handpiece wedge acts on the wool only while a blow is being made
  // (between blows the shearer lifts the handpiece clear of the wool)
  const { X, Y, Z } = clipper.basis;
  const c = sim.clipper || (sim.clipper = {});
  c.active = clip.onSheep && running;
  c.px = clip.pos.x; c.py = clip.pos.y; c.pz = clip.pos.z;
  c.Xx = X.x; c.Xy = X.y; c.Xz = X.z; c.Yx = Y.x; c.Yy = Y.y; c.Yz = Y.z; c.Zx = Z.x; c.Zy = Z.y; c.Zz = Z.z;

  clip.lastPos.copy(clip.pos);
  clip.lastValid = clip.onSheep && running;
  clip.load += ((cuts ? Math.min(1, cuts / 4) : 0) - clip.load) * Math.min(1, dt * 10);
  audio.update(running, clip.load);
  clipper.update(dt);
}

// ---------------------------------------------------------------- autopilot
// Demonstration/testing mode (?autopilot): shears the presented area with
// long parallel blows, starting from the open (lower) edge and working up,
// driving the same pointer -> comb -> cutting path as a player.
const autopilot = { on: new URLSearchParams(location.search).has('autopilot'), queue: [], wait: 0, opened: new Set(), passes: 0, lastFrac: 0 };
// Opening passes: after shearing these areas the autopilot goes back along the
// ventral midline, which cuts the shorn wool there and opens the fleece sleeve.
const OPENERS = {
  'Belly': (x, y, z, ny) => Math.abs(z) < 0.02 && ny < -0.2 && x > -0.3 && x < 0.3,
  'Crutch & tail': (x, y, z, ny, nx) => Math.abs(z) < 0.02 && x < -0.34 && y < 0.075 && nx < -0.3,
  'Left neck': (x, y, z, ny) => Math.abs(z) < 0.02 && x > 0.25 && ny < -0.2,
};
function planOpening(area) {
  const sel = OPENERS[area], pts = [];
  const p = fleece.basePos, n = fleece.restN, tgt = fleece.target;
  for (let i = 0; i < fleece.N; i++) {
    if (!sel(p[3 * i], p[3 * i + 1], p[3 * i + 2], n[3 * i + 1], n[3 * i])) continue;
    _p.set(tgt[3 * i], tgt[3 * i + 1], tgt[3 * i + 2]).project(camera);
    if (Math.abs(_p.x) > 1 || Math.abs(_p.y) > 1) continue;
    pts.push([p[3 * i], _p.x, _p.y]);
  }
  if (pts.length < 3) return;
  pts.sort((a, b) => a[0] - b[0]);   // along the body
  autopilot.queue.push({ x: pts[0][1], y: pts[0][2], cut: false });
  for (const [, x, y] of pts) autopilot.queue.push({ x, y, cut: true });
  autopilot.queue.push({ x: pts[pts.length - 1][1], y: pts[pts.length - 1][2], cut: false });
}
const _p = new THREE.Vector3();
function planStrokes() {
  const area = presented;
  if (!area) return;
  const pts = [];
  const tgt = fleece.target;
  for (let i = 0; i < fleece.N; i++) {
    if (!fleece.attached[i] || director.regionOf[i] !== area) continue;
    _p.set(tgt[3 * i], tgt[3 * i + 1], tgt[3 * i + 2]).project(camera);
    if (Math.abs(_p.x) > 1 || Math.abs(_p.y) > 1) continue;
    pts.push([_p.x, _p.y]);
  }
  if (!pts.length) return;
  // one blow per comb width (screen units at the sheep's distance): wider rows leave
  // stripes for the tidy-up, narrower ones go back over shorn wool and cut the fleece
  const band = 0.8 * COMB_WIDTH / (camDist * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)));
  // each tidy-up pass shifts the rows by half a band so it runs along the stripes left between blows
  const shift = (autopilot.passes % 2) * 0.5 * band;
  const rows = new Map();
  for (const [x, y] of pts) {
    const k = Math.floor((y - shift) / band);
    if (!rows.has(k)) rows.set(k, []);
    rows.get(k).push(x);
  }
  const keys = [...rows.keys()].sort((a, b) => a - b); // bottom (open edge) first
  for (const k of keys) {
    const y = (k + 0.5) * band + shift;
    // one blow per run of unshorn wool: lift over shorn gaps rather than go back over them
    const xs = rows.get(k).sort((a, b) => a - b);
    let start = xs[0];
    for (let q = 1; q <= xs.length; q++) {
      if (q < xs.length && xs[q] - xs[q - 1] < 0.04) continue;
      const a = start - 0.03, b = xs[q - 1] + 0.03;
      const n = Math.max(2, Math.ceil((b - a) / 0.012));
      autopilot.queue.push({ x: a, y, cut: false });
      for (let j = 0; j <= n; j++) autopilot.queue.push({ x: a + (b - a) * j / n, y, cut: true });   // long blows all run tail to head on screen
      autopilot.queue.push({ x: b, y, cut: false });
      if (q < xs.length) start = xs[q];
    }
  }
}
function runAutopilot(dt) {
  if (!autopilot.on || !started || finishTime) return;
  if (anim) { autopilot.queue.length = 0; held.delete('key:' + settings.shearBinding.code); return; }
  if (!autopilot.queue.length) {
    held.delete('key:' + settings.shearBinding.code);
    autopilot.wait += dt;
    if (autopilot.wait < 0.6) return;
    autopilot.wait = 0;
    // tidy up: once every area passes the threshold, go back for the leftovers
    const thr = director.nextRegion(settings.regionThreshold) ? settings.regionThreshold : 0.999;
    // re-plan the presented area until it is done or a pass makes no progress
    const frac = presented ? director.fraction(presented) : 1;
    const progressing = autopilot.passes === 0 || frac > autopilot.lastFrac + 0.002;
    if (presented && frac < thr && progressing && autopilot.passes < 6) {
      autopilot.lastFrac = frac;
      planStrokes();
      autopilot.passes++;
    } else if (presented && OPENERS[presented] && !autopilot.opened.has(presented)) {
      autopilot.opened.add(presented);
      planOpening(presented);
    }
    if (!autopilot.queue.length) {
      const next = director.nextRegion(thr, presented);
      if (next) turnTo(next);
    }
    return;
  }
  const s = autopilot.queue.shift();
  mouse.ndc.set(s.x, s.y); mouse.inside = true;
  if (s.cut) held.add('key:' + settings.shearBinding.code); else held.delete('key:' + settings.shearBinding.code);
}
// debugging/testing handle
window.__sheep = {
  get fleece() { return fleece; }, get director() { return director; }, autopilot,
  shearAll() {
    started = true; startTime = startTime || performance.now() / 1000;
    for (let i = 0; i < fleece.N; i++) if (fleece.release(i)) sim.releases.push([i, 0, 0, 0]);
    syncDirector();
  },
  /** Shear every column whose rest (sheep-local) position satisfies pred(x, y, z). */
  shearWhere(pred) {
    started = true; startTime = startTime || performance.now() / 1000;
    const p = fleece.basePos;
    for (let i = 0; i < fleece.N; i++) {
      if (pred(p[3 * i], p[3 * i + 1], p[3 * i + 2]) && fleece.release(i, 0, 0, 0, ++blow)) sim.releases.push([i, 0, 0, 0]);
    }
    syncDirector();
  },
  freeze: false,   // stop the sheep's idle movements (breathing, head, ears, kicks)
  get sim() { return sim; },
};

// ---------------------------------------------------------------- loop
let last = performance.now(), acc = 0, fpsT = 0, fpsN = 0;
const perf = { phys: 0, view: 0, skin: 0, clip: 0, render: 0 };
const STEP = 1 / 60;

function loop(nowMs) {
  requestAnimationFrame(loop);
  const now = nowMs / 1000;
  const dt = Math.min(0.1, (nowMs - last) / 1000);
  last = nowMs;

  // keyboard rotation
  if (settings.keyboardRotate && !uiOpen()) {
    const k = 1.6 * dt * settings.rotateSensitivity;
    const yaw = (held.has('key:ArrowRight') ? 1 : 0) - (held.has('key:ArrowLeft') ? 1 : 0);
    const pitch = (held.has('key:ArrowDown') ? 1 : 0) - (held.has('key:ArrowUp') ? 1 : 0);
    if (yaw || pitch) rotateSheep(yaw * k * (settings.invertX ? -1 : 1), pitch * k * (settings.invertY ? -1 : 1));
  }

  // automatic handling
  if (anim) {
    anim.t += dt / anim.dur;
    const t = Math.min(1, anim.t), e = t * t * (3 - 2 * t);
    sheepQ.slerpQuaternions(anim.from, anim.to, e);
    if (t >= 1) anim = null;
  } else if (!exit && settings.autoReposition && started && fleece.stats.cut < fleece.N && now - lastCutTime > 0.35) {
    // the presented area is finished (or, before any turn, the one being worked)
    const w = presented || director.workingRegion();
    if (w && director.fraction(w) >= settings.regionThreshold) {
      const next = director.nextRegion(settings.regionThreshold, w);
      if (next && next !== presented) turnTo(next);
    }
  }
  // frame the presented area (or the fleece on the board at the end)
  if (exit) {
    const p = fleece.pos; let x = 0, y = 0, z = 0;
    for (let i = 0; i < fleece.N; i++) { x += p[3 * i]; y += p[3 * i + 1]; z += p[3 * i + 2]; }
    camTargetGoal.set(x / fleece.N, y / fleece.N, z / fleece.N);
  } else if (presented) {
    const c = director.regions[presented].c;
    camTargetGoal.set(c[0], c[1], c[2]).applyQuaternion(sheepQ).multiplyScalar(0.55).add(sheepRoot.position);
  } else camTargetGoal.copy(sheepRoot.position);
  camTarget.lerp(camTargetGoal, Math.min(1, dt * 2.5));
  camera.position.copy(camTarget).addScaledVector(VIEW_DIR, camDist);
  camera.lookAt(camTarget);

  // animate the sheep (more agitated while the clippers run)
  let t0 = performance.now();
  if (!window.__sheep.freeze) behaviour.step(dt, clipper.running ? 0.6 : 0);
  body.update(behaviour.ear);
  perf.skin += performance.now() - t0;
  restOnBoard(dt);
  const { R, t } = sheepMatrices();

  runAutopilot(dt);
  t0 = performance.now();
  updateClipper(dt, now);
  perf.clip += performance.now() - t0;

  // physics at a fixed rate
  t0 = performance.now();
  acc += dt;
  let n = 0;
  fleece.updateAnchors(R, t);
  if (sim.ready && !sim.inFlight) {
    while (acc >= STEP && n < 2) { acc -= STEP; n++; }
    if (acc > STEP * 2) acc = 0;
    if (n || sim.releases.length || sim.cuts.length) {
      const msg = {
        type: 'step', id: sim.id, steps: n, dt: STEP, R, t, mats: rig.mats, breath: rig.breath,
        clipper: { ...sim.clipper }, releases: sim.releases, cuts: sim.cuts, buffer: sim.spare,
      };
      sim.worker.postMessage(msg, sim.spare ? [sim.spare.buffer] : []);
      sim.inFlight = true; sim.spare = null; sim.releases = []; sim.cuts = [];
    }
  } else if (acc > STEP * 4) acc = STEP * 4;
  // rooted (unshorn) columns follow the skin with no latency
  const fp = fleece.pos, ft = fleece.target, fd = fleece.dyn;
  for (let p = 0, np = fleece.P; p < np; p++) {
    if (fd[p]) continue;
    const i3 = 3 * p;
    fp[i3] = ft[i3]; fp[i3 + 1] = ft[i3 + 1]; fp[i3 + 2] = ft[i3 + 2];
  }
  perf.phys += performance.now() - t0;
  t0 = performance.now();
  fleeceView.update();
  perf.view += performance.now() - t0;

  // completion: once nearly everything is off, the shearer pulls the fleece free
  if (!finishTime && fleece.stats.cut >= fleece.N * 0.985 && fleece.stats.cut < fleece.N && now - lastCutTime > 1.5) {
    for (let i = 0; i < fleece.N; i++) if (fleece.release(i)) sim.releases.push([i, 0, 0, 0]);
    syncDirector();
  }
  if (!finishTime && fleece.stats.cut >= fleece.N) {
    finishTime = now;
    // the sheep is let up and dragged out backwards while the fleece is held on the board
    exit = { phase: 'turn', t: 0 };
    anim = { from: sheepQ.clone(), to: new THREE.Quaternion().setFromEuler(new THREE.Euler(0, -0.35, 0)), t: 0, dur: 1.6 };
    presented = null;
    sim.worker.postMessage({ type: 'params', physics: { friction: 0.02, holdFleece: true } });
  }
  if (exit) {
    exit.t += dt;
    if (exit.phase === 'turn' && !anim && exit.t > 1.8) { exit.phase = 'walk'; exit.t = 0; }
    if (exit.phase === 'walk') {
      const back = new THREE.Vector3(-1, 0, 0).applyQuaternion(sheepQ);
      const v = Math.min(1, exit.t / 0.8) * 0.45;
      sheepRoot.position.addScaledVector(back, v * dt);
      if (exit.t > 4) { exit.phase = 'done'; showDone(finishTime - startTime); }
    }
  }
  piecesTimer -= dt;
  if (piecesTimer < 0) { pieces = fleece.countPieces(); piecesTimer = 1; }
  updateHud(false, now);

  t0 = performance.now();
  renderer.render(scene, camera);
  perf.render += performance.now() - t0;

  if (settings.showFps) {
    fpsN++; fpsT += dt;
    if (fpsT > 0.5) {
      const ms = (v) => (v / fpsN).toFixed(1);
      $('fps').textContent = `${Math.round(fpsN / fpsT)} fps · physics ${sim.ms.toFixed(1)} ms (worker) · fleece mesh ${ms(perf.view)} ms · skin ${ms(perf.skin)} ms · clippers ${ms(perf.clip)} ms · render ${ms(perf.render)} ms · ${sim.nDyn} moving particles`;
      fpsN = 0; fpsT = 0; for (const k in perf) perf[k] = 0;
    }
  }
}

// ---------------------------------------------------------------- UI
function updateHud(force, now = 0) {
  const pct = fleece.stats.cut / fleece.N;
  $('progress-bar').style.width = (pct * 100).toFixed(1) + '%';
  $('progress-text').textContent = Math.floor(pct * 100) + '% shorn';
  const area = presented || director.workingRegion();
  $('region').textContent = exit ? 'Fleece off — well done' : area ? `${area} · ${Math.round(director.fraction(area) * 100)}%` : 'Choose where to start';
  const el = startTime ? Math.max(0, (finishTime || now) - startTime) : 0;
  $('timer').textContent = `${Math.floor(el / 60)}:${String(Math.floor(el % 60)).padStart(2, '0')}`;
  $('pieces').textContent = pieces <= 1 ? 'in one piece' : `in ${pieces} pieces`;
  renderAreas(area);
  $('fps').classList.toggle('hidden', !settings.showFps);
  $('running').classList.toggle('on', clipper.running);
}

const areaRows = new Map();
function renderAreas(current) {
  const list = $('areas');
  for (const name of REGION_ORDER) {
    const R = director.regions[name];
    if (!R || !R.total) continue;
    let row = areaRows.get(name);
    if (!row) {
      row = document.createElement('li');
      row.innerHTML = '<span></span><i><b></b></i>';
      row.firstChild.textContent = name;
      list.appendChild(row);
      areaRows.set(name, row);
    }
    const f = director.fraction(name);
    row.querySelector('b').style.width = (f * 100).toFixed(0) + '%';
    row.classList.toggle('done', f >= settings.regionThreshold);
    row.classList.toggle('current', name === current);
  }
}

function showDone(sec) {
  sec = Math.max(0, sec);
  $('done-time').textContent = `${Math.floor(sec / 60)} min ${Math.round(sec % 60)} s`;
  $('done-weight').textContent = fleece.totalMass.toFixed(1) + ' kg';
  $('done-pieces').textContent = pieces <= 1 ? 'one piece — beautiful' : `in ${pieces} pieces`;
  $('done').classList.remove('hidden');
}

function controlsSummary() {
  const clipperMove = settings.clipperControl === 'pointer' ? 'Move the mouse' : `Drag with ${bindingLabel(settings.dragBinding)}`;
  return [
    [clipperMove, 'steer the handpiece over the skin'],
    [`${settings.shearMode === 'hold' ? 'Hold' : 'Press'} ${bindingLabel(settings.shearBinding)}`, 'run the clippers'],
    [`${bindingLabel(settings.rotateBinding)} + move`, 'turn the sheep'],
    [bindingLabel(settings.nextRegionKey), 'turn to the next area'],
    ['Esc', 'settings'],
  ];
}

function renderControls(el) {
  el.innerHTML = '';
  for (const [k, v] of controlsSummary()) {
    const dt = document.createElement('dt'); dt.textContent = k;
    const dd = document.createElement('dd'); dd.textContent = v;
    el.append(dt, dd);
  }
}

function showIntro() {
  renderControls($('intro-controls'));
  $('intro').classList.remove('hidden');
}
$('start').addEventListener('click', () => {
  $('intro').classList.add('hidden');
  started = true;
  turnTo(director.nextRegion(settings.regionThreshold));
});

function toggleSettings(force) {
  const el = $('settings');
  const open = force ?? el.classList.contains('hidden');
  el.classList.toggle('hidden', !open);
  if (open) buildSettingsPanel($('settings-body'), settings, onSettingChange);
  else renderControls($('hud-controls'));
}
$('btn-settings').addEventListener('click', () => toggleSettings(true));
$('settings-close').addEventListener('click', () => toggleSettings(false));
$('btn-next').addEventListener('click', () => gotoNextRegion(true));
$('btn-new').addEventListener('click', () => { sheepQ.setFromEuler(new THREE.Euler(0, -0.55, 0)); newFleece(); started = true; turnTo(director.nextRegion(settings.regionThreshold)); });
$('done-new').addEventListener('click', () => $('btn-new').click());
$('btn-help').addEventListener('click', showIntro);
$('btn-demo').addEventListener('click', () => {
  autopilot.on = !autopilot.on;
  autopilot.queue.length = 0;
  held.delete('key:' + settings.shearBinding.code);
  $('btn-demo').textContent = autopilot.on ? 'Stop demo' : 'Demo';
  if (autopilot.on && !presented) { started = true; turnTo(director.nextRegion(settings.regionThreshold)); }
});
if (autopilot.on) $('btn-demo').textContent = 'Stop demo';

function onSettingChange(k) {
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, settings.pixelRatio));
  renderer.shadowMap.enabled = settings.shadows;
  body.setShadows(settings.shadows);
  fleeceView.setShadows(settings.shadows);
  fleeceView.setShellsVisible(settings.shells);
  clipper.setGhostVisible(settings.xray);
  audio.volume = settings.volume;
  sim.worker.postMessage({ type: 'params', physics: physicsParams() });
  fleece.applyParams(physicsParams()); // keeps the mirror's mass (HUD weight) in step
  $('weight').textContent = fleece.totalMass.toFixed(1) + ' kg';
  if (k === 'shadows') scene.traverse((o) => { if (o.material) o.material.needsUpdate = true; });
  shearing = false;
}
renderControls($('hud-controls'));

init().catch((e) => {
  console.error(e);
  $('loading-text').textContent = 'Something went wrong: ' + e.message;
});

// ---------------------------------------------------------------- board
function makeBoard() {
  const c = document.createElement('canvas');
  c.width = 1024; c.height = 1024;
  const x = c.getContext('2d');
  const planks = 8;
  for (let i = 0; i < planks; i++) {
    const hue = 28 + Math.random() * 6, l = 30 + Math.random() * 8;
    x.fillStyle = `hsl(${hue} 32% ${l}%)`;
    x.fillRect(0, i * 128, 1024, 128);
    // grain and lanolin polish
    for (let k = 0; k < 90; k++) {
      x.strokeStyle = `hsla(${hue} 30% ${l - 8 + Math.random() * 16}% / 0.35)`;
      x.lineWidth = 1 + Math.random() * 2;
      const y = i * 128 + Math.random() * 128;
      x.beginPath(); x.moveTo(0, y);
      for (let s = 0; s <= 8; s++) x.lineTo(s * 128, y + Math.sin(s * 0.9 + k) * 3 + (Math.random() - 0.5) * 2);
      x.stroke();
    }
    x.fillStyle = 'rgba(15,8,4,0.8)';
    x.fillRect(0, i * 128, 1024, 3);
  }
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.repeat.set(3, 3);
  tex.anisotropy = 8;
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(10, 10),
    new THREE.MeshStandardMaterial({ map: tex, roughness: 0.55, metalness: 0 }),
  );
  floor.rotation.x = -Math.PI / 2;
  floor.position.y = FLOOR_Y;
  floor.receiveShadow = true;
  return floor;
}
