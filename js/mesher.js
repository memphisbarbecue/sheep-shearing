// Field sampling and Naive Surface Nets polygonisation.

/** Sample f on a regular grid. With `mirrorZ` the grid is made symmetric about z=0
 *  and only half of it is evaluated (the sheep is bilaterally symmetric). */
export function sampleGrid(f, min, max, h, mirrorZ = true) {
  min = min.slice();
  const nx = Math.ceil((max[0] - min[0]) / h) + 1;
  const ny = Math.ceil((max[1] - min[1]) / h) + 1;
  let nz = Math.ceil((max[2] - min[2]) / h) + 1;
  if (mirrorZ) {
    const half = Math.ceil(Math.max(-min[2], max[2]) / h);
    nz = 2 * half + 1;
    min[2] = -half * h;
  }
  const data = new Float32Array(nx * ny * nz);
  for (let k = 0; k < nz; k++) {
    if (mirrorZ && k < (nz - 1) / 2) continue;
    const z = min[2] + k * h;
    for (let j = 0; j < ny; j++) {
      const y = min[1] + j * h;
      let idx = nx * (j + ny * k);
      for (let i = 0; i < nx; i++, idx++) data[idx] = f(min[0] + i * h, y, z);
    }
  }
  if (mirrorZ) {
    for (let k = 0; k < (nz - 1) / 2; k++) {
      const km = nz - 1 - k;
      data.copyWithin(nx * ny * k, nx * ny * km, nx * ny * (km + 1));
    }
  }
  return { data, nx, ny, nz, min, h };
}

/** Naive surface nets. Returns { positions: Float32Array, indices: Uint32Array }. */
export function surfaceNets(grid) {
  const { data, nx, ny, nz, min, h } = grid;
  const cellIndex = new Int32Array(nx * ny * nz).fill(-1);
  const pos = [];
  const cornerOff = [];
  for (let c = 0; c < 8; c++) cornerOff.push([c & 1, (c >> 1) & 1, (c >> 2) & 1]);
  const edges = [[0, 1], [2, 3], [4, 5], [6, 7], [0, 2], [1, 3], [4, 6], [5, 7], [0, 4], [1, 5], [2, 6], [3, 7]];
  const v = new Float32Array(8);
  let count = 0;
  for (let k = 0; k < nz - 1; k++) for (let j = 0; j < ny - 1; j++) for (let i = 0; i < nx - 1; i++) {
    let mask = 0;
    for (let c = 0; c < 8; c++) {
      const o = cornerOff[c];
      v[c] = data[(i + o[0]) + nx * ((j + o[1]) + ny * (k + o[2]))];
      if (v[c] < 0) mask |= 1 << c;
    }
    if (mask === 0 || mask === 255) continue;
    let sx = 0, sy = 0, sz = 0, n = 0;
    for (const [a, b] of edges) {
      const ia = v[a] < 0, ib = v[b] < 0;
      if (ia === ib) continue;
      const t = v[a] / (v[a] - v[b]);
      const oa = cornerOff[a], ob = cornerOff[b];
      sx += oa[0] + (ob[0] - oa[0]) * t;
      sy += oa[1] + (ob[1] - oa[1]) * t;
      sz += oa[2] + (ob[2] - oa[2]) * t;
      n++;
    }
    pos.push(min[0] + (i + sx / n) * h, min[1] + (j + sy / n) * h, min[2] + (k + sz / n) * h);
    cellIndex[i + nx * (j + ny * k)] = count++;
  }
  const idx = [];
  const C = (i, j, k) => cellIndex[i + nx * (j + ny * k)];
  const quad = (a, b, c, d, flip) => {
    if (a < 0 || b < 0 || c < 0 || d < 0) return;
    if (flip) { const t = b; b = d; d = t; }
    // split along the shorter diagonal
    const d1 = dist2(pos, a, c), d2 = dist2(pos, b, d);
    if (d1 <= d2) idx.push(a, b, c, a, c, d);
    else idx.push(a, b, d, b, c, d);
  };
  for (let k = 1; k < nz - 1; k++) for (let j = 1; j < ny - 1; j++) for (let i = 1; i < nx - 1; i++) {
    const v0 = data[i + nx * (j + ny * k)];
    const in0 = v0 < 0;
    // x edge
    if ((data[i + 1 + nx * (j + ny * k)] < 0) !== in0)
      quad(C(i, j - 1, k - 1), C(i, j, k - 1), C(i, j, k), C(i, j - 1, k), !in0);
    // y edge
    if ((data[i + nx * (j + 1 + ny * k)] < 0) !== in0)
      quad(C(i - 1, j, k - 1), C(i - 1, j, k), C(i, j, k), C(i, j, k - 1), !in0);
    // z edge
    if ((data[i + nx * (j + ny * (k + 1))] < 0) !== in0)
      quad(C(i - 1, j - 1, k), C(i, j - 1, k), C(i, j, k), C(i - 1, j, k), !in0);
  }
  return { positions: new Float32Array(pos), indices: new Uint32Array(idx) };
}

function dist2(p, a, b) {
  const dx = p[3 * a] - p[3 * b], dy = p[3 * a + 1] - p[3 * b + 1], dz = p[3 * a + 2] - p[3 * b + 2];
  return dx * dx + dy * dy + dz * dz;
}

/** Newton-project vertices onto the zero level set of an analytic SDF. */
export function projectToSurface(positions, sdf, iters = 3) {
  const e = 2e-4;
  for (let v = 0; v < positions.length; v += 3) {
    let x = positions[v], y = positions[v + 1], z = positions[v + 2];
    for (let it = 0; it < iters; it++) {
      const d = sdf(x, y, z);
      const gx = (sdf(x + e, y, z) - sdf(x - e, y, z)) / (2 * e);
      const gy = (sdf(x, y + e, z) - sdf(x, y - e, z)) / (2 * e);
      const gz = (sdf(x, y, z + e) - sdf(x, y, z - e)) / (2 * e);
      const g2 = gx * gx + gy * gy + gz * gz;
      if (g2 < 1e-8) break;
      const s = Math.max(-0.01, Math.min(0.01, d / g2));
      x -= gx * s; y -= gy * s; z -= gz * s;
      if (Math.abs(d) < 1e-5) break;
    }
    positions[v] = x; positions[v + 1] = y; positions[v + 2] = z;
  }
}

/** Build vertex adjacency (CSR) from triangle indices. */
export function adjacency(nVerts, indices) {
  const sets = Array.from({ length: nVerts }, () => new Set());
  for (let t = 0; t < indices.length; t += 3) {
    const a = indices[t], b = indices[t + 1], c = indices[t + 2];
    sets[a].add(b); sets[a].add(c); sets[b].add(a); sets[b].add(c); sets[c].add(a); sets[c].add(b);
  }
  const start = new Int32Array(nVerts + 1);
  for (let i = 0; i < nVerts; i++) start[i + 1] = start[i] + sets[i].size;
  const list = new Int32Array(start[nVerts]);
  for (let i = 0; i < nVerts; i++) { let o = start[i]; for (const n of sets[i]) list[o++] = n; }
  return { start, list };
}

/** Tangential Laplacian relaxation followed by re-projection (improves triangle quality). */
export function relax(positions, indices, sdf, iters = 4, lambda = 0.5) {
  const n = positions.length / 3;
  const adj = adjacency(n, indices);
  const tmp = new Float32Array(positions.length);
  for (let it = 0; it < iters; it++) {
    for (let i = 0; i < n; i++) {
      const s = adj.start[i], e = adj.start[i + 1];
      if (e === s) { tmp[3 * i] = positions[3 * i]; tmp[3 * i + 1] = positions[3 * i + 1]; tmp[3 * i + 2] = positions[3 * i + 2]; continue; }
      let ax = 0, ay = 0, az = 0;
      for (let q = s; q < e; q++) { const j = adj.list[q]; ax += positions[3 * j]; ay += positions[3 * j + 1]; az += positions[3 * j + 2]; }
      const c = 1 / (e - s);
      tmp[3 * i] = positions[3 * i] + (ax * c - positions[3 * i]) * lambda;
      tmp[3 * i + 1] = positions[3 * i + 1] + (ay * c - positions[3 * i + 1]) * lambda;
      tmp[3 * i + 2] = positions[3 * i + 2] + (az * c - positions[3 * i + 2]) * lambda;
    }
    positions.set(tmp);
    projectToSurface(positions, sdf, 2);
  }
}

/** Remove degenerate triangles and unreferenced vertices; returns remap. */
export function compact(positions, indices, keepVertex) {
  const n = positions.length / 3;
  const keepTri = [];
  for (let t = 0; t < indices.length; t += 3) {
    const a = indices[t], b = indices[t + 1], c = indices[t + 2];
    if (a === b || b === c || a === c) continue;
    if (keepVertex && (!keepVertex[a] || !keepVertex[b] || !keepVertex[c])) continue;
    keepTri.push(a, b, c);
  }
  const remap = new Int32Array(n).fill(-1);
  let m = 0;
  for (const v of keepTri) if (remap[v] < 0) remap[v] = m++;
  const pos = new Float32Array(m * 3);
  for (let i = 0; i < n; i++) if (remap[i] >= 0) pos.set(positions.subarray(3 * i, 3 * i + 3), 3 * remap[i]);
  const idx = new Uint32Array(keepTri.length);
  for (let i = 0; i < keepTri.length; i++) idx[i] = remap[keepTri[i]];
  return { positions: pos, indices: idx, remap };
}

/** Keep only connected components with at least `minSize` vertices. */
export function dropSmallComponents(positions, indices, minSize) {
  const n = positions.length / 3;
  const parent = new Int32Array(n).map((_, i) => i);
  const find = (a) => { while (parent[a] !== a) { parent[a] = parent[parent[a]]; a = parent[a]; } return a; };
  for (let t = 0; t < indices.length; t += 3) {
    const a = find(indices[t]), b = find(indices[t + 1]), c = find(indices[t + 2]);
    parent[b] = a; parent[find(c)] = a;
  }
  const size = new Int32Array(n);
  for (let i = 0; i < n; i++) size[find(i)]++;
  const keep = new Uint8Array(n);
  for (let i = 0; i < n; i++) keep[i] = size[find(i)] >= minSize ? 1 : 0;
  return compact(positions, indices, keep);
}
