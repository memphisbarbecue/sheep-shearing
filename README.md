# Sheep Shearer

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
* **Felted base** (stiff in-plane links plus second-ring stretch-only *felting tethers*, because
  fibres are longer than the lattice spacing), **loose tips** (staples part easily, resist being
  packed together), **crimp elasticity** (compliant staples in compression).
* **Viscoplastic creep** of bending/shear fibres: a fleece rolled back stays rolled back.
* **Tearing** when the felted base is overstretched for a sustained time; natural partings are
  weaker (throat), and the belly wool is skirted off separately as shearers do — this opens the
  sleeve-shaped fleece into a sheet so it can come off in one piece.
* Columns are rooted to the animated skin until the cutter passes, then become dynamic.
* Small-step **XPBD** solver running in a **Web Worker** (`js/fleeceWorker.js`); the main thread
  keeps a lightweight mirror for cutting and topology, and draws rooted wool with zero latency. Collisions against a precomputed sheep SDF, the still-unshorn wool,
  the board, the **comb/handpiece wedge** (cut wool rides up over the comb and handpiece), and
  itself (spatial hash).

### Rendering (`js/fleeceView.js`, `js/materials.js`)
The fleece is drawn as a closed thick shell: outer tip surface (blocky staples and lock partings
that open up when the mat is stretched over a fold, dust-stained tips), the creamy cut face, and
**side walls** along every free or torn edge showing fibre striations and crimp. Alpha-tested fibre
shells give fuzzy silhouettes.

## Settings
Controls, handling (auto-reposition, area threshold, speed), physics (lattice resolution,
substeps, elasticity, fibre strength, fleece weight, self-collision), graphics (fibre shells,
shadows, x-ray clippers, render resolution, FPS) and audio. Settings persist in `localStorage`.
