// Renders the simulated fleece as a closed thick shell: tip surface, cut face,
// side walls along every free edge (where the mat's fibrous cross-section is
// exposed), plus a few alpha-tested fibre shells for a fuzzy silhouette.
import * as THREE from 'three';
import { makeWoolMaterial } from './materials.js';

export class FleeceView {
  constructor(fleece, { shells = 3, shadows = true } = {}) {
    this.f = fleece;
    const N = fleece.N, P = fleece.P;
    this.group = new THREE.Group();

    // --- main surfaces (base layer = cut face, tip layer = outer surface)
    const g = new THREE.BufferGeometry();
    this.posAttr = new THREE.BufferAttribute(fleece.pos, 3).setUsage(THREE.DynamicDrawUsage);
    this.normals = new Float32Array(3 * P);
    this.nrmAttr = new THREE.BufferAttribute(this.normals, 3).setUsage(THREE.DynamicDrawUsage);
    this.strainAttr = new THREE.BufferAttribute(fleece.strain, 1).setUsage(THREE.DynamicDrawUsage);
    const rest = new Float32Array(3 * P);
    rest.set(fleece.basePos); rest.set(fleece.basePos, 3 * N);
    const layer = new Float32Array(P); layer.fill(1, N);
    // dust load on the tips: highest along the back, least on the belly
    this.dirt = new Float32Array(P);
    for (let i = 0; i < N; i++) {
      const ny = fleece.restN[3 * i + 1];
      this.dirt[i] = this.dirt[N + i] = Math.min(1, Math.max(0, (ny + 0.3) / 1.1));
    }
    g.setAttribute('position', this.posAttr);
    g.setAttribute('normal', this.nrmAttr);
    g.setAttribute('aRest', new THREE.BufferAttribute(rest, 3));
    g.setAttribute('aLayer', new THREE.BufferAttribute(layer, 1));
    g.setAttribute('aStrain', this.strainAttr);
    g.setAttribute('aDirt', new THREE.BufferAttribute(this.dirt, 1));
    this.index = new THREE.BufferAttribute(new Uint32Array(fleece.T * 6), 1);
    g.setIndex(this.index);
    this.geometry = g;
    this.mesh = new THREE.Mesh(g, makeWoolMaterial());
    this.mesh.frustumCulled = false;
    this.mesh.castShadow = shadows; this.mesh.receiveShadow = shadows;
    this.group.add(this.mesh);

    // --- fibre shells over the outer surface
    const sg = new THREE.BufferGeometry();
    for (const k of ['position', 'normal', 'aRest', 'aLayer', 'aStrain', 'aDirt']) sg.setAttribute(k, g.getAttribute(k));
    this.shellIndex = new THREE.BufferAttribute(new Uint32Array(fleece.T * 3), 1);
    sg.setIndex(this.shellIndex);
    this.shellGeometry = sg;
    this.shells = [];
    for (let s = 1; s <= shells; s++) {
      const m = new THREE.Mesh(sg, makeWoolMaterial({ shellT: s / shells, shellLen: 0.011 }));
      m.frustumCulled = false;
      m.receiveShadow = shadows;
      this.shells.push(m);
      this.group.add(m);
    }

    // --- walls along free edges
    const E = fleece.E;
    const wg = new THREE.BufferGeometry();
    this.wPos = new Float32Array(E * 12);
    this.wNrm = new Float32Array(E * 12);
    this.wRest = new Float32Array(E * 12);
    this.wLayer = new Float32Array(E * 4);
    this.wStrain = new Float32Array(E * 4).fill(1);
    this.wDirt = new Float32Array(E * 4);
    this.wPosAttr = new THREE.BufferAttribute(this.wPos, 3).setUsage(THREE.DynamicDrawUsage);
    this.wNrmAttr = new THREE.BufferAttribute(this.wNrm, 3).setUsage(THREE.DynamicDrawUsage);
    wg.setAttribute('position', this.wPosAttr);
    wg.setAttribute('normal', this.wNrmAttr);
    this.wRestAttr = new THREE.BufferAttribute(this.wRest, 3);
    this.wLayerAttr = new THREE.BufferAttribute(this.wLayer, 1);
    wg.setAttribute('aRest', this.wRestAttr);
    wg.setAttribute('aLayer', this.wLayerAttr);
    wg.setAttribute('aStrain', new THREE.BufferAttribute(this.wStrain, 1));
    this.wDirtAttr = new THREE.BufferAttribute(this.wDirt, 1);
    wg.setAttribute('aDirt', this.wDirtAttr);
    this.wIndex = new THREE.BufferAttribute(new Uint32Array(E * 6), 1);
    wg.setIndex(this.wIndex);
    this.wallGeometry = wg;
    this.wallMesh = new THREE.Mesh(wg, makeWoolMaterial({ wall: true }));
    this.wallMesh.frustumCulled = false;
    this.wallMesh.castShadow = shadows; this.wallMesh.receiveShadow = shadows;
    this.group.add(this.wallMesh);
    // fibre fringe on the walls (the mat's edge is ragged, not a cut block)
    for (let k = 1; k <= 2; k++) {
      const m = new THREE.Mesh(wg, makeWoolMaterial({ wall: true, shellT: k / 2, shellLen: 0.009 }));
      m.frustumCulled = false;
      this.shells.push(m);
      this.group.add(m);
    }

    this.wallEdges = new Int32Array(E * 2); // per wall quad: a, b (base indices in winding order)
    this.nWalls = 0;
    this.topologyVersion = -1;
    this._rebuildTopology();
  }

  setShadows(on) {
    this.mesh.castShadow = this.mesh.receiveShadow = on;
    this.wallMesh.castShadow = this.wallMesh.receiveShadow = on;
    for (const s of this.shells) s.receiveShadow = on;
  }
  setShellsVisible(on) { for (const s of this.shells) s.visible = on; }

  _rebuildTopology() {
    const f = this.f, N = f.N, T = f.tris;
    const idx = this.index.array, sidx = this.shellIndex.array;
    let n = 0, s = 0;
    for (let t = 0; t < f.T; t++) {
      if (!f.triAlive[t]) continue;
      const a = T[3 * t], b = T[3 * t + 1], c = T[3 * t + 2];
      idx[n++] = a; idx[n++] = c; idx[n++] = b;             // cut face looks toward the skin
      idx[n++] = N + a; idx[n++] = N + b; idx[n++] = N + c;  // outer surface
      sidx[s++] = N + a; sidx[s++] = N + b; sidx[s++] = N + c;
    }
    this.geometry.setDrawRange(0, n);
    this.index.needsUpdate = true;
    this.shellGeometry.setDrawRange(0, s);
    this.shellIndex.needsUpdate = true;
    this.liveIndexCount = n;

    // free edges: edges with exactly one live triangle; keep that triangle's winding
    let w = 0;
    const wi = this.wIndex.array;
    for (let e = 0; e < f.E; e++) {
      if (f.eLive[e] !== 1) continue;
      let t = f.eTri[2 * e];
      if (t < 0 || !f.triAlive[t]) t = f.eTri[2 * e + 1];
      if (t < 0 || !f.triAlive[t]) continue;
      // find directed edge within triangle t
      let a = f.eA[e], b = f.eB[e];
      for (let k = 0; k < 3; k++) {
        if (T[3 * t + k] === b && T[3 * t + (k + 1) % 3] === a) { const tmp = a; a = b; b = tmp; break; }
      }
      this.wallEdges[2 * w] = a; this.wallEdges[2 * w + 1] = b;
      const v = 4 * w;
      // vertices: base a, base b, tip b, tip a
      const ra = f.basePos.subarray(3 * a, 3 * a + 3), rb = f.basePos.subarray(3 * b, 3 * b + 3);
      this.wRest.set(ra, 3 * v); this.wRest.set(rb, 3 * v + 3); this.wRest.set(rb, 3 * v + 6); this.wRest.set(ra, 3 * v + 9);
      this.wLayer[v] = 0; this.wLayer[v + 1] = 0; this.wLayer[v + 2] = 1; this.wLayer[v + 3] = 1;
      this.wDirt[v] = this.wDirt[v + 3] = this.dirt[a]; this.wDirt[v + 1] = this.wDirt[v + 2] = this.dirt[b];
      wi[6 * w] = v; wi[6 * w + 1] = v + 1; wi[6 * w + 2] = v + 2;
      wi[6 * w + 3] = v; wi[6 * w + 4] = v + 2; wi[6 * w + 5] = v + 3;
      w++;
    }
    this.nWalls = w;
    this.wallGeometry.setDrawRange(0, 6 * w);
    this.wIndex.needsUpdate = true;
    this.wRestAttr.needsUpdate = true;
    this.wLayerAttr.needsUpdate = true;
    this.wDirtAttr.needsUpdate = true;
    this.topologyVersion = f.topologyVersion;
  }

  update() {
    const f = this.f;
    if (f.topologyVersion !== this.topologyVersion) this._rebuildTopology();
    f.computeStrain();
    // vertex normals
    const nr = this.normals, p = f.pos, idx = this.index.array, n = this.liveIndexCount;
    nr.fill(0);
    for (let i = 0; i < n; i += 3) {
      const a = 3 * idx[i], b = 3 * idx[i + 1], c = 3 * idx[i + 2];
      const ux = p[b] - p[a], uy = p[b + 1] - p[a + 1], uz = p[b + 2] - p[a + 2];
      const vx = p[c] - p[a], vy = p[c + 1] - p[a + 1], vz = p[c + 2] - p[a + 2];
      const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
      nr[a] += nx; nr[a + 1] += ny; nr[a + 2] += nz;
      nr[b] += nx; nr[b + 1] += ny; nr[b + 2] += nz;
      nr[c] += nx; nr[c + 1] += ny; nr[c + 2] += nz;
    }
    for (let i = 0; i < nr.length; i += 3) {
      const l = Math.hypot(nr[i], nr[i + 1], nr[i + 2]) || 1;
      nr[i] /= l; nr[i + 1] /= l; nr[i + 2] /= l;
    }
    this.posAttr.needsUpdate = true;
    this.nrmAttr.needsUpdate = true;
    this.strainAttr.needsUpdate = true;

    // walls
    const N = f.N, wp = this.wPos, wn = this.wNrm, we = this.wallEdges;
    for (let w = 0; w < this.nWalls; w++) {
      const a = we[2 * w], b = we[2 * w + 1];
      const o = 12 * w;
      const A = 3 * a, B = 3 * b, TA = 3 * (N + a), TB = 3 * (N + b);
      wp[o] = p[A]; wp[o + 1] = p[A + 1]; wp[o + 2] = p[A + 2];
      wp[o + 3] = p[B]; wp[o + 4] = p[B + 1]; wp[o + 5] = p[B + 2];
      wp[o + 6] = p[TB]; wp[o + 7] = p[TB + 1]; wp[o + 8] = p[TB + 2];
      wp[o + 9] = p[TA]; wp[o + 10] = p[TA + 1]; wp[o + 11] = p[TA + 2];
      // normal from the quad diagonals
      const d1x = p[TB] - p[A], d1y = p[TB + 1] - p[A + 1], d1z = p[TB + 2] - p[A + 2];
      const d2x = p[TA] - p[B], d2y = p[TA + 1] - p[B + 1], d2z = p[TA + 2] - p[B + 2];
      let nx = d2y * d1z - d2z * d1y, ny = d2z * d1x - d2x * d1z, nz = d2x * d1y - d2y * d1x;
      // orientation: (b-a) x (tipA - a) points outward
      const ex = p[B] - p[A], ey = p[B + 1] - p[A + 1], ez = p[B + 2] - p[A + 2];
      const sx = p[TA] - p[A], sy = p[TA + 1] - p[A + 1], sz = p[TA + 2] - p[A + 2];
      const ox = ey * sz - ez * sy, oy = ez * sx - ex * sz, oz = ex * sy - ey * sx;
      if (nx * ox + ny * oy + nz * oz < 0) { nx = -nx; ny = -ny; nz = -nz; }
      const l = Math.hypot(nx, ny, nz) || 1;
      nx /= l; ny /= l; nz /= l;
      for (let k = 0; k < 4; k++) { wn[o + 3 * k] = nx; wn[o + 3 * k + 1] = ny; wn[o + 3 * k + 2] = nz; }
    }
    this.wPosAttr.needsUpdate = true;
    this.wNrmAttr.needsUpdate = true;
  }

  dispose() {
    this.geometry.dispose(); this.shellGeometry.dispose(); this.wallGeometry.dispose();
    this.mesh.material.dispose(); this.wallMesh.material.dispose();
    for (const s of this.shells) s.material.dispose();
  }
}
