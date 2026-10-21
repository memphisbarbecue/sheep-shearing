// User settings: defaults, persistence and the settings panel.

export const DEFAULTS = {
  // controls
  rotateBinding: { type: 'mouse', button: 1, label: 'Middle mouse' },
  shearBinding: { type: 'key', code: 'Space', label: 'Space' },
  shearMode: 'hold',             // 'hold' | 'toggle'
  clipperControl: 'pointer',     // 'pointer' (follows mouse) | 'drag' (follows while dragBinding held)
  dragBinding: { type: 'mouse', button: 0, label: 'Left mouse' },
  rotateSensitivity: 1.0,
  invertX: false,
  invertY: false,
  keyboardRotate: true,          // arrow keys rotate the sheep
  wheelZoom: true,
  nextRegionKey: { type: 'key', code: 'KeyN', label: 'N' },
  // handling
  autoReposition: true,
  regionThreshold: 0.9,
  repositionSpeed: 1.0,
  // physics
  resolution: 'medium',          // fleece lattice: low | medium | high
  substeps: 8,
  elasticity: 1.0,               // multiplies compliances
  fibreStrength: 1.0,            // multiplies tear ratio margin
  fleeceWeight: 1.0,
  selfCollision: true,
  // graphics
  shells: true,
  shadows: true,
  xray: true,
  pixelRatio: 1.5,
  showFps: false,
  // audio
  volume: 0.5,
};

const KEY = 'sheep-shearer-settings-v1';

export function loadSettings() {
  let saved = {};
  try { saved = JSON.parse(localStorage.getItem(KEY) || '{}'); } catch { saved = {}; }
  return Object.assign(structuredClone(DEFAULTS), saved);
}

export function saveSettings(s) {
  try { localStorage.setItem(KEY, JSON.stringify(s)); } catch { /* storage unavailable */ }
}

export const RESOLUTIONS = { low: 0.032, medium: 0.026, high: 0.021 };

export function bindingLabel(b) {
  if (!b) return '—';
  if (b.type === 'mouse') return ['Left mouse', 'Middle mouse', 'Right mouse', 'Mouse 4', 'Mouse 5'][b.button] || `Mouse ${b.button}`;
  return prettyKey(b.code);
}

export function prettyKey(code) {
  if (code === 'Space') return 'Space';
  if (code.startsWith('Key')) return code.slice(3);
  if (code.startsWith('Digit')) return code.slice(5);
  return code.replace(/(Left|Right)$/, ' ($1)').replace('Arrow', 'Arrow ');
}

/**
 * Builds the settings panel. `onChange(key, value, needsRebuild)` is called on edits.
 */
export function buildSettingsPanel(root, settings, onChange) {
  root.innerHTML = '';
  const section = (title) => {
    const s = document.createElement('section');
    const h = document.createElement('h3'); h.textContent = title; s.appendChild(h);
    root.appendChild(s); return s;
  };
  const row = (parent, label, control, hint) => {
    const r = document.createElement('div'); r.className = 'row';
    const l = document.createElement('label'); l.textContent = label;
    if (hint) { const s = document.createElement('small'); s.textContent = hint; l.appendChild(s); }
    r.append(l, control); parent.appendChild(r); return r;
  };
  const set = (k, v, rebuild = false) => { settings[k] = v; saveSettings(settings); onChange(k, v, rebuild); };

  const bindingButton = (key, allowMouse = true) => {
    const b = document.createElement('button');
    b.className = 'bind';
    b.textContent = bindingLabel(settings[key]);
    b.addEventListener('click', (ev) => {
      ev.stopPropagation();
      b.textContent = allowMouse ? 'Press a key or mouse button…' : 'Press a key…';
      b.classList.add('listening');
      const finish = (binding) => {
        window.removeEventListener('keydown', onKey, true);
        window.removeEventListener('mousedown', onMouse, true);
        b.classList.remove('listening');
        if (binding) set(key, binding);
        b.textContent = bindingLabel(settings[key]);
      };
      const onKey = (e) => {
        e.preventDefault(); e.stopPropagation();
        if (e.code === 'Escape') return finish(null);
        finish({ type: 'key', code: e.code });
      };
      const onMouse = (e) => {
        if (!allowMouse) return;
        e.preventDefault(); e.stopPropagation();
        finish({ type: 'mouse', button: e.button });
      };
      // defer so this click doesn't bind itself
      setTimeout(() => {
        window.addEventListener('keydown', onKey, true);
        window.addEventListener('mousedown', onMouse, true);
      }, 0);
    });
    return b;
  };
  const select = (key, options, rebuild = false) => {
    const s = document.createElement('select');
    for (const [v, label] of options) {
      const o = document.createElement('option'); o.value = v; o.textContent = label; s.appendChild(o);
    }
    s.value = settings[key];
    s.addEventListener('change', () => set(key, s.value, rebuild));
    return s;
  };
  const check = (key, rebuild = false) => {
    const c = document.createElement('input'); c.type = 'checkbox'; c.checked = !!settings[key];
    c.addEventListener('change', () => set(key, c.checked, rebuild));
    return c;
  };
  const slider = (key, min, max, step, fmt = (v) => v, rebuild = false) => {
    const w = document.createElement('div'); w.className = 'slider';
    const r = document.createElement('input'); r.type = 'range'; r.min = min; r.max = max; r.step = step; r.value = settings[key];
    const o = document.createElement('output'); o.textContent = fmt(settings[key]);
    r.addEventListener('input', () => { o.textContent = fmt(+r.value); set(key, +r.value, rebuild); });
    w.append(r, o); return w;
  };

  const c = section('Controls');
  row(c, 'Rotate sheep', bindingButton('rotateBinding'), 'hold and move the mouse');
  row(c, 'Run clippers', bindingButton('shearBinding'));
  row(c, 'Clipper trigger mode', select('shearMode', [['hold', 'Hold to cut'], ['toggle', 'Press to toggle']]));
  row(c, 'Move clippers with', select('clipperControl', [['pointer', 'Mouse pointer'], ['drag', 'Mouse drag (hold button)']]));
  row(c, 'Drag button', bindingButton('dragBinding'), 'used when moving clippers by drag');
  row(c, 'Next area', bindingButton('nextRegionKey', false));
  row(c, 'Rotation sensitivity', slider('rotateSensitivity', 0.2, 3, 0.1, (v) => v.toFixed(1) + '×'));
  row(c, 'Invert horizontal', check('invertX'));
  row(c, 'Invert vertical', check('invertY'));
  row(c, 'Arrow keys rotate sheep', check('keyboardRotate'));
  row(c, 'Mouse wheel zoom', check('wheelZoom'));

  const h = section('Sheep handling');
  row(h, 'Auto-reposition sheep', check('autoReposition'), 'turn the sheep to the next area when one is finished');
  row(h, 'Area finished at', slider('regionThreshold', 0.6, 1, 0.01, (v) => Math.round(v * 100) + '%'));
  row(h, 'Repositioning speed', slider('repositionSpeed', 0.3, 3, 0.1, (v) => v.toFixed(1) + '×'));

  const p = section('Fleece physics');
  row(p, 'Fleece lattice', select('resolution', [['low', 'Low (fast)'], ['medium', 'Medium'], ['high', 'High (detailed)']], true), 'applies to the next sheep');
  row(p, 'Solver substeps', slider('substeps', 4, 20, 1, (v) => v));
  row(p, 'Fleece elasticity', slider('elasticity', 0.3, 4, 0.1, (v) => v.toFixed(1) + '×'), 'crimp stretch of the felted mat');
  row(p, 'Fibre strength', slider('fibreStrength', 0.5, 3, 0.1, (v) => v.toFixed(1) + '×'), 'how hard it is to tear the fleece');
  row(p, 'Fleece weight', slider('fleeceWeight', 0.5, 2, 0.1, (v) => v.toFixed(1) + '×'));
  row(p, 'Fleece self-collision', check('selfCollision'));

  const g = section('Graphics');
  row(g, 'Fibre shells (fuzzy wool edges)', check('shells'));
  row(g, 'Shadows', check('shadows'));
  row(g, 'Show clippers through wool', check('xray'));
  row(g, 'Render resolution', slider('pixelRatio', 0.5, 2, 0.25, (v) => v.toFixed(2) + '×'));
  row(g, 'Show FPS', check('showFps'));

  const a = section('Audio');
  row(a, 'Clipper volume', slider('volume', 0, 1, 0.05, (v) => Math.round(v * 100) + '%'));

  const reset = document.createElement('button');
  reset.className = 'reset'; reset.textContent = 'Restore defaults';
  reset.addEventListener('click', () => {
    for (const k of Object.keys(DEFAULTS)) settings[k] = structuredClone(DEFAULTS[k]);
    saveSettings(settings);
    onChange('*', null, false);
    buildSettingsPanel(root, settings, onChange);
  });
  root.appendChild(reset);
}
