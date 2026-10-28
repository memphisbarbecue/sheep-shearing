# Sheep Shearer (v2)

v2 of the app; v1 is the copy one folder up. v2 is the start of a lock-based fleece model.

A browser simulation of shearing a Merino ewe. No build step: plain ES modules + three.js (from a CDN).

## Run

Any static file server works (ES modules don't load from `file://`):

```sh
cd sheep-shearer
python3 -m http.server 8000
# open http://localhost:8000
```

Add `?autopilot` to the URL to watch the app shear a sheep by itself (Bowen-style long blows).

## Controls (defaults — all rebindable in Settings, `Esc`)

| Action | Default |
| --- | --- |
| Steer the handpiece | move the mouse (or: drag with a chosen button) |
| Run the clippers | hold `Space` (or toggle mode; any key or mouse button) |
| Turn the sheep | hold middle mouse + move (any key/button), arrow keys |
| Next area | `N` |
| Zoom | mouse wheel |

The sheep is turned automatically to present the next area when the current one is finished
(threshold and speed are configurable, and it can be switched off). The HUD's *Areas* list shows
progress per area. Once ~98.5% is off and the clippers pause, the remaining wisps are pulled free.

## How it works

### Anatomy (`js/anatomy.js`)
The skin is the zero level set of a signed distance field built from an anatomical description of
a ~55 kg Merino ewe: skeleton landmarks (spine, scapula, pelvis, limb joints, skull) and the major
muscle masses on them (longissimus, gluteals, biceps femoris, hamstrings, quadriceps, triceps,
shoulder muscles, pectorals/brisket, neck muscles, masseter), blended with smooth unions. Merino
neck folds are added as skin displacement. It is polygonised with surface nets (`js/mesher.js`)
and skinned on the CPU (`js/rig.js`) for breathing, head and ear movement, tail flicks and the
odd hind-leg kick.

Wool growth (`stapleLength`) follows a Merino with ~11 months of growth: ~9 cm on back and sides,
longer britch, shorter belly wool, leg wool fading above knee and hock, wool over the poll and an
open face.

### Fleece (`js/fleece.js`)
The fleece is its own volumetric body, not a texture or a displaced skin:

* **Two particle layers** over a shared triangulation: the base layer (skin side / cut face) and
  the tip layer (weathered outer surface), joined per column by a **staple** constraint and by
  crossed **shear fibres**. The mat therefore keeps its thickness and bulk when it comes off.
* **No shape memory.** Nothing in the mat remembers the shape it had on the sheep: a cut piece
  drapes, folds and lies flat on the board. Tips are spaced by their *root* spacing; the links that
  keep staples upright are sized from the *current* root spacing every substep.
* **Felted base**: stiff in stretch, soft in compression (it bunches and crumples), slowly taking a
  set; second-ring stretch-only *felting tethers* because fibres are longer than the lattice
  spacing. **Loose tips**: they part almost freely (lock partings open up on the outside of a fold)
  but resist being crowded together. **Crimp**: staples are compliant in compression.
* At the **cut line** a lifted staple slides along its still-rooted neighbour (they are entangled
  along the whole staple), so the fleece parts there instead of tearing.
* The **handpiece** acts like a plough: the comb runs under the slice, the tilted handpiece lifts
  its free edge and rolls it back over the uncut wool, cut face up.
* **Viscoplastic creep** of bending/shear fibres: a fleece rolled back stays rolled back.
* **Tearing** when the felted base is overstretched for a sustained time.
* **Nothing is pre-cut.** The fleece starts as a closed sleeve round the sheep. The cutter
  (`js/cutter.js`) severs rooted staples at the skin, and when the comb goes back along skin an
  earlier blow already shore, it cuts through the wool that grew there and still lies in its mouth:
  the mat parts along the middle of the stroke. That is how the shearer opens the fleece (the
  autopilot goes back along the belly, under the tail and up the throat), and how careless,
  overlapping blows chop a fleece into pieces. Wool draped over the path from elsewhere rides up
  over the comb and is not cut, so neat adjacent blows keep the fleece whole.
* Columns are rooted to the animated skin until the cutter passes, then become dynamic.
* Small-step **XPBD** solver running in a **Web Worker** (`js/fleeceWorker.js`); the main thread
  keeps a lightweight mirror for cutting and topology, and draws rooted wool with zero latency. Collisions against a precomputed sheep SDF, the still-unshorn wool,
  the board, the **comb/handpiece wedge** (cut wool rides up over the comb and handpiece), and
  itself (spatial hash; solved inside the substeps with the fibres so folds don't jitter).

### Rendering, v2 (`js/lockView.js`)
v2 draws the fleece as **locks**: one tapered, crimped, lumpy tuft per simulated column in a single
instanced draw call, built on the GPU from the column's root and tip particles (read straight from the
simulation's position array, so the CPU does nothing per frame). Each tuft has a creamy cut-face cap
at the root, fibre streaks and crimp down its sides, shaded crevices low down, and a domed, crusted,
dusty crown. Partings are the gaps between tufts. Settings → Graphics → Fleece look switches back to the
v1 shell (lighter on the GPU); `?view=shell` / `?view=locks` override it.

### Rendering, v1 (`js/fleeceView.js`, `js/materials.js`)
The fleece is drawn as a closed thick shell: outer tip surface (blocky staples and lock partings
that open up when the mat is stretched over a fold, dust-stained tips), the creamy cut face, and
**side walls** along every free or torn edge showing fibre striations and crimp. Alpha-tested fibre
shells give fuzzy silhouettes.

## Settings
Controls, handling (auto-reposition, area threshold, speed), physics (lattice resolution,
substeps, elasticity, fibre strength, fleece weight, self-collision), graphics (fibre shells,
shadows, x-ray clippers, render resolution, FPS) and audio. Settings persist in `localStorage`.

## Testing

Headless tools in `tools/` (Playwright + Chromium in `~/.venvs/shear`):

* `run_scenarios.py`: physics scenarios run on the real solver (patch drop, whole-fleece slump,
  hang by the neck, re-cut along / beside a blow) with metrics and renders.
* `sweep.py`: compare physics overrides side by side.
* `autopilot_snap.py`: run the app with `?autopilot` and take screenshots.
