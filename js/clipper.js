// Shearing handpiece: 77 mm 13-tooth comb with a 4-point cutter, motor-driven
// handpiece on an overhead down-tube. Local frame: +Z = forward (the way the
// comb teeth point / the stroke direction), +Y = skin normal, +X = Y x Z.
import * as THREE from 'three';

export const COMB_WIDTH = 0.077;

export class Clipper {
  constructor(scene) {
    this.group = new THREE.Group();
    this.model = new THREE.Group();
    this.group.add(this.model);
    scene.add(this.group);
    this.running = false;
    this.time = 0;

    const steel = new THREE.MeshStandardMaterial({ color: 0xc9ccd0, metalness: 0.9, roughness: 0.28 });
    const brass = new THREE.MeshStandardMaterial({ color: 0xb8a070, metalness: 0.85, roughness: 0.35 });
    const body = new THREE.MeshStandardMaterial({ color: 0x2c2f33, metalness: 0.4, roughness: 0.45 });
    const accent = new THREE.MeshStandardMaterial({ color: 0xc8501e, metalness: 0.3, roughness: 0.5 });
    const grip = new THREE.MeshStandardMaterial({ color: 0x1a1b1d, metalness: 0.1, roughness: 0.85 });

    // comb
    const comb = new THREE.Mesh(extrudeXZ(combShape(), 0.0028), steel);
    comb.castShadow = true;
    this.model.add(comb);
    // cutter (sits on the comb, oscillates sideways when running)
    this.cutter = new THREE.Mesh(extrudeXZ(cutterShape(), 0.0026), brass);
    this.cutter.position.y = 0.0029;
    this.cutter.castShadow = true;
    this.model.add(this.cutter);

    // head + handpiece body, tilted up away from the skin
    const handle = new THREE.Group();
    handle.position.set(0, 0.022, -0.03);
    handle.rotation.x = 0.2; // rises toward the shearer's hand
    this.model.add(handle);
    const head = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.03, 0.045, 2, 2, 2), body);
    head.position.set(0, -0.004, 0.0);
    handle.add(head);
    const prof = [
      [0.0, 0.0], [0.03, 0.0], [0.031, 0.02], [0.029, 0.06], [0.024, 0.1], [0.026, 0.14],
      [0.028, 0.19], [0.026, 0.23], [0.02, 0.25], [0.0, 0.25],
    ].map(([r, y]) => new THREE.Vector2(r, y));
    const lathe = new THREE.LatheGeometry(prof, 28);
    lathe.rotateX(-Math.PI / 2); // lathe +Y -> -Z
    const bodyMesh = new THREE.Mesh(lathe, body);
    bodyMesh.position.z = -0.01;
    bodyMesh.castShadow = true;
    handle.add(bodyMesh);
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.0285, 0.003, 8, 28), accent);
    ring.position.z = -0.075; handle.add(ring);
    const gripMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.0285, 0.0285, 0.07, 24, 1, true), grip);
    gripMesh.rotation.x = Math.PI / 2; gripMesh.position.z = -0.17; handle.add(gripMesh);
    // tension nut and fork
    const nut = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.009, 0.03, 12), steel);
    nut.position.set(0, 0.022, 0.0); handle.add(nut);
    const fork = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.006, 0.03), steel);
    fork.position.set(0, -0.016, 0.022); handle.add(fork);
    this.handle = handle;

    // overhead flexible down-tube (rebuilt every frame)
    this.tubeMat = new THREE.MeshStandardMaterial({ color: 0x3a3a3a, roughness: 0.7, metalness: 0.2 });
    this.tube = new THREE.Mesh(new THREE.BufferGeometry(), this.tubeMat);
    this.tube.castShadow = true;
    scene.add(this.tube);
    this.hangPoint = new THREE.Vector3(-1.1, 1.9, 0.5);

    // x-ray ghost so the comb can be seen when it is buried in the wool
    this.ghost = this.model.clone(true);
    // drawn only where something (the wool) is in front of the clipper
    const ghostMat = new THREE.MeshBasicMaterial({ color: 0xffb060, transparent: true, opacity: 0.38, depthFunc: THREE.GreaterDepth, depthWrite: false });
    this.ghost.traverse((o) => { if (o.isMesh) { o.material = ghostMat; o.renderOrder = 999; o.castShadow = false; } });
    this.group.add(this.ghost);
    this.ghostCutter = this.ghost.children[1];

    this._fwd = new THREE.Vector3(0, 0, 1);
    this._nrm = new THREE.Vector3(0, 1, 0);
    this._tmpM = new THREE.Matrix4();
    this.colliders = [
      { ax: 0, ay: 0, az: 0, dx: 0, dy: 0, dz: 0, il2: 1, r: 0.024 },
      { ax: 0, ay: 0, az: 0, dx: 0, dy: 0, dz: 0, il2: 1, r: 0.032 },
    ];
  }

  setGhostVisible(v) { this.ghost.visible = v; }

  /** Pose the comb at `pos` lying on a surface with normal `n`, teeth toward `f`. */
  setPose(pos, n, f) {
    const Y = this._nrm.copy(n).normalize();
    const Z = this._fwd.copy(f).addScaledVector(Y, -f.dot(Y));
    if (Z.lengthSq() < 1e-8) Z.set(1, 0, 0).addScaledVector(Y, -Y.x);
    Z.normalize();
    const X = new THREE.Vector3().crossVectors(Y, Z);
    this._tmpM.makeBasis(X, Y, Z);
    this.group.quaternion.setFromRotationMatrix(this._tmpM);
    this.group.position.copy(pos);
    this.basis = { X, Y: Y.clone(), Z: Z.clone() };
  }

  update(dt) {
    this.time += dt;
    const j = this.running ? 1 : 0;
    // cutter traverses the comb ~ 3000 strokes/min; rendered as a blur of offsets
    const off = j * Math.sin(this.time * 377) * 0.004;
    this.cutter.position.x = off;
    this.ghostCutter.position.x = off;
    this.model.position.set(j * (Math.random() - 0.5) * 0.0006, j * (Math.random() - 0.5) * 0.0004, 0);
    this.group.updateMatrixWorld(true);

    // colliders (world space): head over the comb and the handpiece body
    const m = this.group.matrixWorld;
    const a = new THREE.Vector3(0, 0.02, 0.012).applyMatrix4(m);
    const b = new THREE.Vector3(0, 0.03, -0.05).applyMatrix4(m);
    setCapsule(this.colliders[0], a, b, 0.022);
    const c = new THREE.Vector3(0, 0.035, -0.06).applyMatrix4(m);
    const d = new THREE.Vector3(0, 0.08, -0.27).applyMatrix4(m);
    setCapsule(this.colliders[1], c, d, 0.03);

    // down-tube from the back of the handpiece to the overhead plant
    const rear = new THREE.Vector3(0, 0.075, -0.27).applyMatrix4(m);
    const back = new THREE.Vector3(0, 0.09, -0.4).applyMatrix4(m);
    const mid = rear.clone().lerp(this.hangPoint, 0.55).add(new THREE.Vector3(0, -0.25, 0));
    const curve = new THREE.CatmullRomCurve3([rear, back, mid, this.hangPoint]);
    this.tube.geometry.dispose();
    this.tube.geometry = new THREE.TubeGeometry(curve, 36, 0.011, 7, false);
  }
}

function setCapsule(C, a, b, r) {
  C.ax = a.x; C.ay = a.y; C.az = a.z;
  C.dx = b.x - a.x; C.dy = b.y - a.y; C.dz = b.z - a.z;
  C.il2 = 1 / (C.dx * C.dx + C.dy * C.dy + C.dz * C.dz);
  C.r = r;
}

// 2D shapes are drawn in (x, z) with z forward; extruded along +y.
function combShape() {
  const s = new THREE.Shape();
  const w = COMB_WIDTH / 2, back = -0.034, root = -0.002, tip = 0.032, nT = 13;
  s.moveTo(-w + 0.004, back);
  s.lineTo(w - 0.004, back);
  s.quadraticCurveTo(w, back, w, back + 0.006);
  s.lineTo(w, root);
  const pitch = COMB_WIDTH / nT;
  for (let i = nT - 1; i >= 0; i--) {
    const cx = -w + pitch * (i + 0.5);
    const hw = pitch * 0.36, tw = pitch * 0.17;
    s.lineTo(cx + hw, root);
    s.lineTo(cx + tw, tip - 0.003);
    s.quadraticCurveTo(cx, tip + 0.0015, cx - tw, tip - 0.003);
    s.lineTo(cx - hw, root);
  }
  s.lineTo(-w, root);
  s.lineTo(-w, back + 0.006);
  s.quadraticCurveTo(-w, back, -w + 0.004, back);
  return s;
}

function cutterShape() {
  const s = new THREE.Shape();
  const w = 0.031, back = -0.026, root = -0.004, tip = 0.012;
  s.moveTo(-w, back); s.lineTo(w, back); s.lineTo(w, root);
  const nP = 4, pitch = 2 * w / nP;
  for (let i = nP - 1; i >= 0; i--) {
    const cx = -w + pitch * (i + 0.5);
    s.lineTo(cx + pitch * 0.42, root);
    s.lineTo(cx + 0.002, tip);
    s.lineTo(cx - 0.002, tip);
    s.lineTo(cx - pitch * 0.42, root);
  }
  s.lineTo(-w, root); s.lineTo(-w, back);
  return s;
}

function extrudeXZ(shape, depth) {
  const g = new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: true, bevelThickness: 0.0006, bevelSize: 0.0005, bevelSegments: 1, curveSegments: 4 });
  // shape (x, y) -> (x, z); extrusion -> +y
  g.rotateX(Math.PI / 2);
  g.translate(0, depth, 0);
  return g;
}
