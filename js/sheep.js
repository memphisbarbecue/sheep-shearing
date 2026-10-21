// The sheep's body: skin mesh polygonised from the anatomical SDF, coloured by
// anatomy (open face, legs, hooves, nose, shorn body skin), skinned on the CPU
// with the shared rig, plus separate ears and eyes attached to the head bone.
import * as THREE from 'three';
import { sheepSDF, sdfGrad, BOUNDS, stapleLength, nearestTag } from './anatomy.js';
import { sampleGrid, surfaceNets, projectToSurface, compact } from './mesher.js';
import { computeWeights, computeBreath } from './rig.js';
import { makeSkinMaterial } from './materials.js';

export class SheepBody {
  constructor(rig, { resolution = 0.011, shadows = true } = {}) {
    this.rig = rig;
    this.group = new THREE.Group();
    const pad = resolution * 2;
    const grid = sampleGrid(sheepSDF, BOUNDS.min.map((v) => v - pad), BOUNDS.max.map((v) => v + pad), resolution, true);
    let m = surfaceNets(grid);
    m = compact(m.positions, m.indices);
    projectToSurface(m.positions, sheepSDF, 3);
    const n = m.positions.length / 3;
    this.rest = m.positions;
    this.restN = new Float32Array(3 * n);
    const colors = new Float32Array(3 * n), stubble = new Float32Array(n), hoof = new Float32Array(n);
    const g3 = [0, 0, 0];
    const col = new THREE.Color();
    for (let i = 0; i < n; i++) {
      const x = m.positions[3 * i], y = m.positions[3 * i + 1], z = m.positions[3 * i + 2];
      sdfGrad(x, y, z, g3);
      this.restN.set(g3, 3 * i);
      const L = stapleLength(x, y, z, g3[0], g3[1], g3[2]);
      stubble[i] = Math.min(1, L / 0.03);
      const tag = nearestTag(x, y, z);
      // short white hair of face & legs
      col.setRGB(0.88, 0.84, 0.78);
      if (tag === 'hoof' || y < -0.49) {
        col.setRGB(0.16, 0.13, 0.12); hoof[i] = 1;
      } else if (tag === 'head') {
        // pink-grey nose & lips
        const dn = Math.hypot(x - 0.70, y - 0.152, z * 1.2);
        const t = Math.max(0, 1 - dn / 0.035);
        col.lerp(new THREE.Color(0.62, 0.48, 0.47), t * t);
      }
      colors[3 * i] = col.r; colors[3 * i + 1] = col.g; colors[3 * i + 2] = col.b;
    }
    const geo = new THREE.BufferGeometry();
    this.pos = new Float32Array(this.rest);
    this.nrm = new Float32Array(this.restN);
    this.posAttr = new THREE.BufferAttribute(this.pos, 3).setUsage(THREE.DynamicDrawUsage);
    this.nrmAttr = new THREE.BufferAttribute(this.nrm, 3).setUsage(THREE.DynamicDrawUsage);
    geo.setAttribute('position', this.posAttr);
    geo.setAttribute('normal', this.nrmAttr);
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geo.setAttribute('aRest', new THREE.BufferAttribute(this.rest, 3));
    geo.setAttribute('aStubble', new THREE.BufferAttribute(stubble, 1));
    geo.setAttribute('aHoof', new THREE.BufferAttribute(hoof, 1));
    geo.setIndex(new THREE.BufferAttribute(m.indices, 1));
    geo.computeBoundingSphere();
    geo.boundingSphere.radius *= 1.15;
    this.geometry = geo;
    this.mesh = new THREE.Mesh(geo, makeSkinMaterial());
    this.mesh.castShadow = shadows; this.mesh.receiveShadow = shadows;
    this.group.add(this.mesh);

    this.weights = computeWeights(this.rest);
    this.breath = computeBreath(this.rest);

    this._buildHeadParts(shadows);
    this._buildPickProxy();
  }

  // Coarse, invisible copy of the skin for fast pointer ray casts (the hit is
  // then snapped to the exact skin surface with the SDF).
  _buildPickProxy() {
    const h = 0.02;
    const grid = sampleGrid(sheepSDF, BOUNDS.min.map((v) => v - h), BOUNDS.max.map((v) => v + h), h, true);
    let m = surfaceNets(grid);
    m = compact(m.positions, m.indices);
    projectToSurface(m.positions, sheepSDF, 2);
    this.pickRest = m.positions;
    this.pickPos = new Float32Array(m.positions);
    this.pickWeights = computeWeights(this.pickRest);
    const g = new THREE.BufferGeometry();
    this.pickAttr = new THREE.BufferAttribute(this.pickPos, 3);
    g.setAttribute('position', this.pickAttr);
    g.setIndex(new THREE.BufferAttribute(m.indices, 1));
    g.computeBoundingSphere();
    g.boundingSphere.radius *= 1.15;
    this.pickMesh = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ visible: false, side: THREE.DoubleSide }));
    this.group.add(this.pickMesh);
  }

  _surfacePoint(origin, dir) {
    // march from inside along dir to the skin
    let t = 0;
    for (let i = 0; i < 200; i++) {
      const p = [origin[0] + dir[0] * t, origin[1] + dir[1] * t, origin[2] + dir[2] * t];
      if (sheepSDF(...p) > 0) return p;
      t += 0.001;
    }
    return origin;
  }

  _buildHeadParts(shadows) {
    // eyes: dark glossy globes with a horizontal-pupil iris texture
    const eyeTex = makeEyeTexture();
    const eyeMat = new THREE.MeshPhysicalMaterial({ map: eyeTex, roughness: 0.08, clearcoat: 1, clearcoatRoughness: 0.05 });
    this.headParts = new THREE.Group();
    this.headParts.matrixAutoUpdate = false;
    this.group.add(this.headParts);
    for (const side of [1, -1]) {
      const sp = this._surfacePoint([0.598, 0.268, 0], [0.25, 0.1, side]);
      const n = sdfGrad(sp[0], sp[1], sp[2], [0, 0, 0]);
      const eye = new THREE.Mesh(new THREE.SphereGeometry(0.0135, 24, 16), eyeMat);
      eye.position.set(sp[0] - n[0] * 0.007, sp[1] - n[1] * 0.007, sp[2] - n[2] * 0.007);
      eye.lookAt(eye.position.clone().add(new THREE.Vector3(n[0], n[1], n[2])));
      this.headParts.add(eye);

      // ear: cupped leaf shape, pink inside, short white hair outside
      const ear = makeEar(side, shadows);
      const ep = this._surfacePoint([0.535, 0.29, 0], [0, 0.15, side]);
      ear.position.set(ep[0] - 0.004, ep[1] - 0.004, ep[2] - side * 0.006);
      ear.userData.side = side;
      this.headParts.add(ear);
      if (side > 0) this.earL = ear; else this.earR = ear;
    }
  }

  update(earFlick = { L: 0, R: 0 }) {
    this.rig.skin(this.rest, this.weights, this.breath, this.pos, this.restN, this.nrm);
    this.posAttr.needsUpdate = true;
    this.nrmAttr.needsUpdate = true;
    this.rig.skin(this.pickRest, this.pickWeights, null, this.pickPos);
    this.pickAttr.needsUpdate = true;
    // head-mounted parts follow the head bone
    const m = this.rig.boneMatrix('head');
    this.headParts.matrix.set(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], 0, 0, 0, 1);
    this.headParts.matrixWorldNeedsUpdate = true;
    this.earL.rotation.set(-0.5 * earFlick.L, 0, 0);
    this.earR.rotation.set(0.5 * earFlick.R, 0, 0);
  }

  setShadows(on) {
    this.mesh.castShadow = this.mesh.receiveShadow = on;
  }
}

function makeEar(side, shadows) {
  // parametric cupped ear: u along length, v across
  const segU = 14, segV = 8, len = 0.095;
  const pos = [], uv = [], idx = [];
  for (let i = 0; i <= segU; i++) {
    const u = i / segU;
    const w = 0.026 * Math.sin(Math.PI * Math.pow(u, 0.75)) + 0.008 * (1 - u);
    for (let j = 0; j <= segV; j++) {
      const v = j / segV * 2 - 1;
      const cup = 0.009 * (1 - v * v) * Math.sin(Math.PI * Math.min(1, u * 1.2));
      // ear points laterally (local +z*side), droops downward and back
      const along = u * len;
      // cup opens forward/down; droop increases toward the tip
      pos.push(-along * 0.3 + v * w * 0.95 - cup * 0.3, -along * 0.22 - 0.25 * along * u - cup, side * along * 0.92);
      uv.push(u, (v + 1) / 2);
    }
  }
  for (let i = 0; i < segU; i++) for (let j = 0; j < segV; j++) {
    const a = i * (segV + 1) + j, b = a + segV + 1;
    if (side > 0) idx.push(a, b, a + 1, b, b + 1, a + 1); else idx.push(a, a + 1, b, b, a + 1, b + 1);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  g.setIndex(idx);
  g.computeVertexNormals();
  const tex = makeEarTexture();
  const outer = new THREE.MeshStandardMaterial({ color: 0xe2dbcf, roughness: 0.85, side: THREE.FrontSide });
  const inner = new THREE.MeshStandardMaterial({ map: tex, roughness: 0.6, side: THREE.BackSide });
  const grp = new THREE.Group();
  const m1 = new THREE.Mesh(g, outer), m2 = new THREE.Mesh(g, inner);
  m1.castShadow = shadows;
  grp.add(m1, m2);
  return grp;
}

function makeEyeTexture() {
  const c = document.createElement('canvas');
  c.width = 128; c.height = 64;
  const x = c.getContext('2d');
  x.fillStyle = '#2a1a10'; x.fillRect(0, 0, 128, 64);
  // amber-brown iris around the front of the sphere (sphere +z, which lookAt aims outward, is at u=0.25)
  const g = x.createRadialGradient(32, 32, 2, 32, 32, 26);
  g.addColorStop(0, '#0a0604'); g.addColorStop(0.35, '#0a0604'); g.addColorStop(0.4, '#6b4a22'); g.addColorStop(0.9, '#3d2a14'); g.addColorStop(1, '#1d140c');
  x.fillStyle = g; x.beginPath(); x.ellipse(32, 32, 26, 22, 0, 0, Math.PI * 2); x.fill();
  x.fillStyle = '#050302'; x.beginPath(); x.ellipse(32, 32, 13, 4, 0, 0, Math.PI * 2); x.fill(); // horizontal pupil
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function makeEarTexture() {
  const c = document.createElement('canvas');
  c.width = 64; c.height = 32;
  const x = c.getContext('2d');
  const g = x.createLinearGradient(0, 0, 0, 32);
  g.addColorStop(0, '#d9c8bd'); g.addColorStop(0.5, '#e8b3a8'); g.addColorStop(1, '#d9c8bd');
  x.fillStyle = g; x.fillRect(0, 0, 64, 32);
  // veins
  x.strokeStyle = 'rgba(170,80,80,0.35)'; x.lineWidth = 1;
  for (let i = 0; i < 4; i++) { x.beginPath(); x.moveTo(2, 16); x.quadraticCurveTo(30, 8 + i * 5, 62, 12 + i * 3); x.stroke(); }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}
