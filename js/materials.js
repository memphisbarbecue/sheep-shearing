// Procedural materials: greasy Merino fleece (staple tips, cut face, cut walls,
// fibre shells) and the sheep's skin / short hair.
import * as THREE from 'three';

export const NOISE = /* glsl */`
float h13(vec3 p){ p = fract(p*0.1031); p += dot(p, p.zyx+31.32); return fract((p.x+p.y)*p.z); }
vec3 h33(vec3 p){ p = fract(p*vec3(.1031,.1030,.0973)); p += dot(p, p.yxz+33.33); return fract((p.xxy+p.yxx)*p.zyx); }
float vn(vec3 x){
  vec3 i = floor(x), f = fract(x); f = f*f*(3.0-2.0*f);
  return mix(mix(mix(h13(i), h13(i+vec3(1,0,0)), f.x), mix(h13(i+vec3(0,1,0)), h13(i+vec3(1,1,0)), f.x), f.y),
             mix(mix(h13(i+vec3(0,0,1)), h13(i+vec3(1,0,1)), f.x), mix(h13(i+vec3(0,1,1)), h13(i+vec3(1,1,1)), f.x), f.y), f.z);
}
vec2 vor(vec3 x){
  vec3 p = floor(x), f = fract(x); float d1 = 8.0, d2 = 8.0;
  for(int k=-1;k<=1;k++) for(int j=-1;j<=1;j++) for(int i=-1;i<=1;i++){
    vec3 b = vec3(float(i),float(j),float(k));
    vec3 r = b + h33(p+b)*0.85 - f; float d = dot(r,r);
    if(d < d1){ d2 = d1; d1 = d; } else if(d < d2) d2 = d;
  }
  return sqrt(vec2(d1, d2));
}
// fade out detail finer than ~a pixel (prevents sparkle on fine fibre noise)
float aaFade(vec3 p, float freq){ float w = length(fwidth(p)) * freq; return 1.0 - smoothstep(0.35, 1.2, w); }
vec3 bumpNormal(vec3 pos, vec3 n, float hgt, float scale){
  vec3 sx = dFdx(pos), sy = dFdy(pos);
  vec3 r1 = cross(sy, n), r2 = cross(n, sx);
  float det = dot(sx, r1);
  vec2 dh = vec2(dFdx(hgt), dFdy(hgt)) * scale;
  vec3 g = sign(det) * (dh.x * r1 + dh.y * r2);
  return normalize(abs(det) * n - g);
}
`;

const WOOL_FN = /* glsl */`
uniform float uShell;
varying vec3 vRest; varying float vLayer; varying float vStrain; varying float vDirt;
const vec3 CREAM = vec3(0.9, 0.83, 0.68);
vec3 tipColour(float fib){
  // dust and suint collect on the weathered tips, most along the back
  float dirt = clamp(0.15 + vDirt * 0.7 + (vn(vRest*7.0) * 0.7 + vn(vRest*23.0) * 0.3 - 0.5) * 0.7, 0.0, 1.0);
  vec3 tip = mix(vec3(0.76, 0.67, 0.5), vec3(0.52, 0.4, 0.25), dirt);
  return tip * (0.88 + 0.24*fib);
}
// returns colour, writes height for bump mapping
vec3 woolSurface(out float hgt){
  vec3 warp = vec3(vn(vRest*18.0), vn(vRest*18.0+7.3), vn(vRest*18.0+13.1)) - 0.5;
  vec3 warp2 = vec3(vn(vRest*7.0+2.1), vn(vRest*7.0+5.7), vn(vRest*7.0+9.4)) - 0.5;
  vec3 P = vRest*52.0 + warp*1.3;
  float fib = mix(0.5, vn(vRest*vec3(430.0)), aaFade(vRest, 430.0));
  float fib2 = mix(0.5, vn(vRest*vec3(1100.0) + 3.0), aaFade(vRest, 1100.0));
#ifdef WALL
  // cut/torn face of the mat: fibres run root->tip, crimp waves cross them
  float strand = vn(vec3(vRest*360.0)) * 0.65 + fib2 * 0.35;
  float crimp = sin(vLayer * 46.0 + strand * 9.0);
  float tipZone = smoothstep(0.72, 1.0, vLayer);
  vec3 col = mix(CREAM * (0.78 + 0.3*strand), tipColour(fib), tipZone);
  col *= 0.93 + 0.07*crimp;
  // lock boundaries show as darker vertical partings
  vec2 F = vor(P);
  float part = 1.0 - smoothstep(0.0, 0.12, F.y - F.x);
  col *= 1.0 - 0.35*part*(1.0 - vLayer*0.5);
  hgt = strand*0.6 + crimp*0.15 - part*0.6;
  return col;
#else
  if (vLayer > 0.5) {
    // weathered outer surface: blocky staples grouped into locks; partings open when stretched
    float open = clamp((vStrain - 1.0) * 2.8, 0.0, 1.0);
    vec2 B = vor(vRest*21.0 + warp*0.9 + warp2*1.6); // locks / blocks (~5 cm), wavy partings
    vec2 F = vor(P);                               // staples (~2 cm)
    float bEdge = B.y - B.x, edge = F.y - F.x;
    float pw = 0.05 + 0.08*vn(vRest*13.0) + 0.5*open;   // parting width varies along its length
    // at rest the locks press together (only faint seams); stretch opens them
    float parting = (1.0 - smoothstep(0.0, pw, bEdge)) * (0.45 + 0.55*vn(vRest*31.0)) * (0.28 + 0.72*open);
    float crack = (1.0 - smoothstep(0.0, 0.06 + 0.35*open, edge)) * 0.22 * aaFade(vRest, 52.0) * (0.3 + 0.7*open);
    float gap = max(parting, crack);
    vec3 tip = tipColour(fib);
    vec3 inside = mix(CREAM * 0.9, CREAM * 0.42, 0.35 + 0.65*max(open, parting*0.6));
    vec3 col = mix(tip, inside, gap * (0.5 + 0.5*open));
    col *= 0.9 + 0.1*(1.0 - B.x);                   // block crowns catch a little more light
    hgt = (1.0 - B.x)*0.5 + (1.0 - F.x)*0.25 - gap*0.6 + fib*0.12 + fib2*0.05;
    return col;
  } else {
    // freshly cut face against the skin: clean, creamy, dense fibre ends; locks visible faintly
    vec2 F = vor(P*0.9);
    float part = 1.0 - smoothstep(0.0, 0.08 + 0.25*clamp(vStrain-1.0,0.0,1.0), F.y - F.x);
    vec3 col = CREAM * (0.9 + 0.12*fib2 + 0.05*fib) * vec3(1.0, 0.985, 0.95);
    col *= 1.0 - 0.25*part;
    hgt = fib2*0.35 + fib*0.2 - part*0.4;
    return col;
  }
#endif
}
`;

function patchWool(shader, defines) {
  shader.uniforms.uShell = { value: defines.shellT || 0 };
  shader.vertexShader = shader.vertexShader
    .replace('#include <common>', `#include <common>
      attribute vec3 aRest; attribute float aLayer; attribute float aStrain; attribute float aDirt;
      uniform float uShell; uniform float uShellLen;
      varying vec3 vRest; varying float vLayer; varying float vStrain; varying float vDirt;`)
    .replace('#include <begin_vertex>', `#include <begin_vertex>
      vRest = aRest; vLayer = aLayer; vStrain = aStrain; vDirt = aDirt;
      #ifdef SHELL
        transformed += normalize(objectNormal) * uShellLen * uShell;
      #endif`);
  shader.uniforms.uShellLen = { value: defines.shellLen || 0.012 };
  shader.fragmentShader = shader.fragmentShader
    .replace('#include <common>', `#include <common>\n${NOISE}\n${WOOL_FN}`)
    .replace('#include <color_fragment>', `#include <color_fragment>
      float woolH;
      vec3 woolCol = woolSurface(woolH);
      #if defined(SHELL) && defined(WALL)
        // ragged fibre fringe along cut/torn edges: strands run root -> tip
        float strandF = mix(0.5, vn(vec3(vRest*520.0)), aaFade(vRest, 520.0))*0.6 + vn(vRest*140.0)*0.4;
        float reach = strandF * (0.55 + 0.45*vLayer);
        if (reach < 0.25 + uShell*0.5) discard;
        woolCol = mix(CREAM*0.95, tipColour(strandF), smoothstep(0.6, 1.0, vLayer));
      #elif defined(SHELL)
        float fibres = mix(0.5, vn(vRest*vec3(820.0)), aaFade(vRest, 820.0))*0.55 + vn(vRest*vec3(300.0)+1.7)*0.45;
        vec2 Fs = vor(vRest*52.0 + (vec3(vn(vRest*18.0), vn(vRest*18.0+7.3), vn(vRest*18.0+13.1)) - 0.5)*1.3);
        float cover = (1.0 - Fs.x) * fibres * 1.25 - clamp(vStrain-1.0, 0.0, 1.0)*0.5;
        // fibre tips only read against the light at grazing angles: keep the fuzz on silhouettes
        float facing = abs(dot(normalize(vNormal), normalize(vViewPosition)));
        if (cover < 0.22 + uShell*0.55 + smoothstep(0.25, 0.75, facing)*0.9) discard;
        woolCol = tipColour(fibres) * (1.0 + 0.12*uShell);
      #endif
      diffuseColor.rgb = woolCol;`)
    .replace('#include <normal_fragment_maps>', `#include <normal_fragment_maps>
      #ifndef SHELL
        normal = bumpNormal(-vViewPosition, normal, woolH, ${defines.wall ? '0.0045' : '0.006'});
      #endif`);
}

export function makeWoolMaterial({ wall = false, shellT = 0, shellLen = 0.012 } = {}) {
  const m = new THREE.MeshPhysicalMaterial({
    color: 0xffffff,
    roughness: 0.92,
    metalness: 0,
    sheen: 0.7,
    sheenRoughness: 0.55,
    sheenColor: new THREE.Color(0.95, 0.9, 0.78),
    side: THREE.FrontSide,
  });
  m.defines = {};
  if (wall) m.defines.WALL = '';
  if (shellT > 0) m.defines.SHELL = '';
  m.onBeforeCompile = (shader) => patchWool(shader, { wall, shellT, shellLen });
  m.customProgramCacheKey = () => `wool-${wall}-${shellT > 0}`;
  if (wall && shellT > 0) m.side = THREE.DoubleSide;
  return m;
}

export function makeSkinMaterial() {
  const m = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.82, metalness: 0 });
  m.onBeforeCompile = (shader) => {
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', `#include <common>
        attribute vec3 aRest; attribute float aStubble; attribute float aHoof;
        varying vec3 vRest; varying float vStubble; varying float vHoof;`)
      .replace('#include <begin_vertex>', `#include <begin_vertex>
        vRest = aRest; vStubble = aStubble; vHoof = aHoof;`);
    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', `#include <common>\n${NOISE}
        varying vec3 vRest; varying float vStubble; varying float vHoof;`)
      .replace('#include <color_fragment>', `#include <color_fragment>
        float fineHair = mix(0.5, vn(vRest*vec3(900.0)), aaFade(vRest, 900.0)) * 0.6 + mix(0.5, vn(vRest*vec3(2400.0)), aaFade(vRest, 2400.0)) * 0.4;
        float blotch = vn(vRest*30.0);
        // shorn skin: a few mm of creamy stubble over pink skin; follicle pattern
        vec3 stubble = vec3(0.9, 0.82, 0.7) * (0.8 + 0.3*fineHair);
        vec3 pinkSkin = vec3(0.92, 0.68, 0.62);
        // follicle-level mottling: pink skin shows between the stubble
        float cover = 0.38 + 0.3*blotch + 0.25*smoothstep(0.35, 0.75, fineHair);
        vec3 shorn = mix(pinkSkin, stubble, cover);
        diffuseColor.rgb = mix(diffuseColor.rgb * (0.9 + 0.18*fineHair), shorn, vStubble);
        // hoof horn growth rings
        diffuseColor.rgb *= mix(1.0, 0.85 + 0.25*sin(vRest.y*900.0 + vn(vRest*80.0)*3.0), vHoof);
        float skinH = fineHair * (0.4 + 0.6*vStubble) + vHoof*sin(vRest.y*900.0)*0.3;`)
      .replace('#include <roughnessmap_fragment>', `#include <roughnessmap_fragment>
        roughnessFactor = mix(roughnessFactor, 0.45, vHoof);`)
      .replace('#include <normal_fragment_maps>', `#include <normal_fragment_maps>
        normal = bumpNormal(-vViewPosition, normal, skinH, 0.0012);`);
  };
  m.customProgramCacheKey = () => 'sheep-skin';
  return m;
}
