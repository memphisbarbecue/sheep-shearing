// Runs the fleece simulation off the main thread. The main thread keeps a
// non-simulating mirror of the fleece (topology, attachment state) and sends
// the rig pose, sheep transform, clipper pose and newly severed columns each
// frame; the worker steps the physics and returns particle positions.
import { Fleece } from './fleece.js';
import { Rig } from './rig.js';

let fleece = null;
const rig = new Rig();

self.onmessage = (ev) => {
  const m = ev.data;
  if (m.type === 'init') {
    fleece = new Fleece({ rig, resolution: m.resolution, physics: m.physics, floorY: m.floorY, seed: m.seed });
    fleece.updateAnchors(m.R, m.t);
    fleece.reset();
    self.postMessage({ type: 'ready', id: m.id });
    return;
  }
  if (!fleece) return;
  if (m.type === 'params') { fleece.applyParams(m.physics); return; }
  if (m.type === 'step') {
    const t0 = performance.now();
    for (const [i, vx, vy, vz] of m.releases) fleece.release(i, vx, vy, vz);
    for (const e of m.cuts) fleece.cutEdge(e);
    fleece.clipper = m.clipper;
    rig.mats.set(m.mats);
    rig.breath = m.breath;
    for (let k = 0; k < m.steps; k++) {
      fleece.updateAnchors(m.R, m.t);
      fleece.step(m.dt);
    }
    const buf = m.buffer && m.buffer.length === fleece.pos.length ? m.buffer : new Float32Array(fleece.pos.length);
    buf.set(fleece.pos);
    const killed = fleece._killLog.slice();
    fleece._killLog.length = 0;
    self.postMessage({
      type: 'state', id: m.id, pos: buf, killed,
      tears: fleece.stats.tears, nDyn: fleece.nDyn, ms: performance.now() - t0,
    }, [buf.buffer]);
  }
};
