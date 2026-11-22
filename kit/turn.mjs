import {
  T3_EYE,
  T3_LEVELS,
  T3_Q,
  T3_SHINE,
  T3_TAU,
  t3BoxOf,
  t3Camera,
  t3EmitBall,
  t3EmitDots,
  t3EmitFixed,
  t3EmitLathe,
  t3EmitPlanes,
  t3EmitPrism,
  t3EmitRound,
  t3Faded,
  t3Frame,
  t3Hull,
  t3Lines,
  t3Mul,
  t3Path,
  t3PathOf,
  t3Poses,
  t3PosesMap,
  t3Prepare,
  t3Quat,
  t3Rot,
  t3State,
  t3Step,
  t3Unit,
} from "./turn-core.mjs";
function t3Sides(part, cam, r) {
  const m = cam.m;
  const k = cam.k;
  const n = part.count;
  const out = { n, flat: new Float64Array(2 * n), left: new Float64Array(2 * n), right: new Float64Array(2 * n), shineA: new Float64Array(2 * n), shineB: new Float64Array(2 * n), shadeA: new Float64Array(2 * n), shadeB: new Float64Array(2 * n), frames: [] };
  const half = r * k;
  for (let i = 0; i < n; i++) {
    const x = part.P3[3 * i];
    const y = part.P3[3 * i + 1];
    const z = part.P3[3 * i + 2];
    const px = cam.ox + m[0] * x + m[1] * y + m[2] * z;
    const py = cam.oy + m[3] * x + m[4] * y + m[5] * z;
    const dx = m[0] * part.D3[3 * i] + m[1] * part.D3[3 * i + 1] + m[2] * part.D3[3 * i + 2];
    const dy = m[3] * part.D3[3 * i] + m[4] * part.D3[3 * i + 1] + m[5] * part.D3[3 * i + 2];
    const l = Math.hypot(dx, dy) || 1;
    const nx = -dy / l;
    const ny = dx / l;
    out.flat[2 * i] = px;
    out.flat[2 * i + 1] = py;
    out.frames.push({ p: [px, py], nx, ny, half });
    out.left[2 * i] = px + nx * half;
    out.left[2 * i + 1] = py + ny * half;
    out.right[2 * i] = px - nx * half;
    out.right[2 * i + 1] = py - ny * half;
    const toward = nx * T3_SHINE[0] + ny * T3_SHINE[1];
    const shineAt = half * 0.44 * toward;
    const shineHalf = half * 0.24;
    out.shineA[2 * i] = px + nx * (shineAt + shineHalf);
    out.shineA[2 * i + 1] = py + ny * (shineAt + shineHalf);
    out.shineB[2 * i] = px + nx * (shineAt - shineHalf);
    out.shineB[2 * i + 1] = py + ny * (shineAt - shineHalf);
    const shadeAt = -half * 0.62 * toward;
    const shadeHalf = half * 0.2;
    out.shadeA[2 * i] = px + nx * (shadeAt + shadeHalf);
    out.shadeA[2 * i + 1] = py + ny * (shadeAt + shadeHalf);
    out.shadeB[2 * i] = px + nx * (shadeAt - shadeHalf);
    out.shadeB[2 * i + 1] = py + ny * (shadeAt - shadeHalf);
  }
  return out;
}

const t3Pts = (buf, n) => {
  const out = new Array(n);
  for (let i = 0; i < n; i++) out[i] = [buf[2 * i], buf[2 * i + 1]];
  return out;
};

function t3TubeCap(cam, centre, tangent, radius, inward) {
  const V = cam.V;
  const m = cam.m;
  let n1 = [tangent[1] * V[2] - tangent[2] * V[1], tangent[2] * V[0] - tangent[0] * V[2], tangent[0] * V[1] - tangent[1] * V[0]];
  if (Math.hypot(n1[0], n1[1], n1[2]) < 1e-6) {
    const helper = Math.abs(tangent[2]) > 0.9 ? [1, 0, 0] : [0, 0, 1];
    n1 = [helper[1] * tangent[2] - helper[2] * tangent[1], helper[2] * tangent[0] - helper[0] * tangent[2], helper[0] * tangent[1] - helper[1] * tangent[0]];
  }
  n1 = t3Unit(n1);
  const n2 = t3Unit([tangent[1] * n1[2] - tangent[2] * n1[1], tangent[2] * n1[0] - tangent[0] * n1[2], tangent[0] * n1[1] - tangent[1] * n1[0]]);
  const steps = 28;
  const proj = (p) => [cam.ox + m[0] * p[0] + m[1] * p[1] + m[2] * p[2], cam.oy + m[3] * p[0] + m[4] * p[1] + m[5] * p[2]];
  const ring = new Array(steps);
  for (let index = 0; index < steps; index++) {
    const angle = (index / steps) * T3_TAU;
    const cs = radius * Math.cos(angle);
    const sn = radius * Math.sin(angle);
    ring[index] = proj([centre[0] + n1[0] * cs + n2[0] * sn, centre[1] + n1[1] * cs + n2[1] * sn, centre[2] + n1[2] * cs + n2[2] * sn]);
  }
  const C = proj(centre);
  const probe = proj([centre[0] + n2[0], centre[1] + n2[1], centre[2] + n2[2]]);
  const along = (probe[0] - C[0]) * inward[0] + (probe[1] - C[1]) * inward[1];
  const from = along > 0 ? steps / 2 : 0;
  const half = (start) => Array.from({ length: steps / 2 + 1 }, (_, index) => ring[(start + index) % steps]);
  return { ring, arc: half(from), inner: half((from + steps / 2) % steps) };
}

function t3StripeEnd(half, centre, nx, ny, from, to) {
  const offset = (p) => (p[0] - centre[0]) * nx + (p[1] - centre[1]) * ny;
  const values = half.map(offset);
  const lo = Math.min(from, to);
  const hi = Math.max(from, to);
  const points = [];
  const cross = (k, level) => {
    const t = (level - values[k]) / (values[k + 1] - values[k] || 1e-9);
    return [half[k][0] + (half[k + 1][0] - half[k][0]) * t, half[k][1] + (half[k + 1][1] - half[k][1]) * t];
  };
  for (let k = 0; k < half.length; k++) {
    if (values[k] >= lo && values[k] <= hi) points.push(half[k]);
    if (k < half.length - 1) for (const level of [lo, hi]) if ((values[k] - level) * (values[k + 1] - level) < 0) points.push(cross(k, level));
  }
  return points
    .map((p) => [p, offset(p)])
    .sort((x, y) => (from > to ? y[1] - x[1] : x[1] - y[1]))
    .map(([p]) => p);
}

function t3EmitTube(run, cam, f) {
  const r = run.r;
  const sides = t3Sides(run.body, cam, r);
  const bands = run.stripe === run.body ? sides : t3Sides(run.stripe, cam, r);
  const n = sides.n;
  const left = t3Pts(sides.left, n);
  const right = t3Pts(sides.right, n);
  const loop = left.concat(right.slice().reverse());
  let area = 0;
  for (let i = 0; i < loop.length; i++) {
    const a = loop[i];
    const b = loop[(i + 1) % loop.length];
    area += a[0] * b[1] - b[0] * a[1];
  }
  const sign = Math.sign(area);
  let body = t3PathOf(loop, true);
  let edges = t3PathOf(left, false) + t3PathOf(right, false);
  const tips = [null, null];
  const last = n - 1;
  const quads = [];
  for (let i = 0; i < last; i++) quads.push([left[i], left[i + 1], right[i + 1], right[i]]);
  for (const side of [0, 1]) {
    if (!run.capped[side]) continue;
    const at = side ? last : 0;
    const other = side ? last - 1 : 1;
    const tangent = run.tips[side];
    const outward = side ? tangent : [-tangent[0], -tangent[1], -tangent[2]];
    const inward = [sides.flat[2 * other] - sides.flat[2 * at], sides.flat[2 * other + 1] - sides.flat[2 * at + 1]];
    const centre = [run.body.P3[3 * at], run.body.P3[3 * at + 1], run.body.P3[3 * at + 2]];
    const { ring, arc, inner } = t3TubeCap(cam, centre, tangent, r, inward);
    let ringArea = 0;
    for (let i = 0; i < ring.length; i++) {
      const a = ring[i];
      const b = ring[(i + 1) % ring.length];
      ringArea += a[0] * b[1] - b[0] * a[1];
    }
    body += t3PathOf(Math.sign(ringArea) === sign ? ring : ring.slice().reverse(), true);
    edges += t3PathOf(arc, false);
    const V = cam.V;
    const open = outward[0] * V[0] + outward[1] * V[1] + outward[2] * V[2] > 0;
    if (open) edges += t3PathOf(inner, false);
    tips[side] = { half: open ? inner : arc, frame: sides.frames[at] };
    quads.push(ring);
  }
  f.d[run.role.body] = body;
  f.d[run.role.edges] = edges;
  if (run.wide) {
    const toward = (frame) => frame.nx * T3_SHINE[0] + frame.ny * T3_SHINE[1];
    const shineOffsets = (frame) => [frame.half * 0.44 * toward(frame) + frame.half * 0.24, frame.half * 0.44 * toward(frame) - frame.half * 0.24];
    const shadeOffsets = (frame) => [-frame.half * 0.62 * toward(frame) + frame.half * 0.2, -frame.half * 0.62 * toward(frame) - frame.half * 0.2];
    const bn = bands.n;
    const band = (A, B, offsets) => {
      const head = tips[1] ? t3StripeEnd(tips[1].half, tips[1].frame.p, tips[1].frame.nx, tips[1].frame.ny, offsets(tips[1].frame)[0], offsets(tips[1].frame)[1]) : [];
      const tail = tips[0] ? t3StripeEnd(tips[0].half, tips[0].frame.p, tips[0].frame.nx, tips[0].frame.ny, offsets(tips[0].frame)[1], offsets(tips[0].frame)[0]) : [];
      return t3PathOf([...t3Pts(A, bn), ...head, ...t3Pts(B, bn).reverse(), ...tail], true);
    };
    f.d[run.role.shine] = band(bands.shineA, bands.shineB, shineOffsets);
    f.d[run.role.shade] = band(bands.shadeA, bands.shadeB, shadeOffsets);
  }
  if (run.rings) {
    const lines = t3Lines;
    lines.d.length = 0;
    lines.alpha.length = 0;
  lines.key.length = 0;
    lines.key.length = 0;
    const V = cam.V;
    const m = cam.m;
    const seg = new Float64Array(4);
    const surface = (sample, out, index) => {
      const T = sample.T;
      let n1 = [T[1] * V[2] - T[2] * V[1], T[2] * V[0] - T[0] * V[2], T[0] * V[1] - T[1] * V[0]];
      const l1 = Math.hypot(n1[0], n1[1], n1[2]);
      if (l1 < 1e-6) return null;
      n1 = [n1[0] / l1, n1[1] / l1, n1[2] / l1];
      let n2 = t3Unit([T[1] * n1[2] - T[2] * n1[1], T[2] * n1[0] - T[0] * n1[2], T[0] * n1[1] - T[1] * n1[0]]);
      if (n2[0] * V[0] + n2[1] * V[1] + n2[2] * V[2] < 0) n2 = [-n2[0], -n2[1], -n2[2]];
      const cs = Math.cos(sample.theta);
      const sn = Math.sin(sample.theta);
      const normal = [n1[0] * cs + n2[0] * sn, n1[1] * cs + n2[1] * sn, n1[2] * cs + n2[2] * sn];
      const p = [sample.c[0] + normal[0] * r, sample.c[1] + normal[1] * r, sample.c[2] + normal[2] * r];
      out[index] = cam.ox + m[0] * p[0] + m[1] * p[1] + m[2] * p[2];
      out[index + 1] = cam.oy + m[3] * p[0] + m[4] * p[1] + m[5] * p[2];
      return normal[0] * V[0] + normal[1] * V[1] + normal[2] * V[2];
    };
    const fade = run.rings.fade;
    let index = -1;
    for (const [a, b] of run.rings.segments) {
      index++;
      const fa = surface(a, seg, 0);
      const fb = surface(b, seg, 2);
      if (fa === null || fb === null) continue;
      const alpha = fade ? t3Step(fade[0], fade[1], (fa + fb) / 2) : 1;
      if (alpha * T3_LEVELS < 0.5) continue;
      lines.d.push(t3Path(seg, 2, false));
      lines.alpha.push(alpha);
      lines.key.push(index);
    }
    f.levels = t3Faded(f, run, "r", lines, lines.d.length);
  }
  const all = left.concat(right);
  t3BoxOf(all, f.box);
  for (const q of quads) for (const p of q) {
    if (p[0] < f.box[0]) f.box[0] = p[0];
    if (p[1] < f.box[1]) f.box[1] = p[1];
    if (p[0] > f.box[2]) f.box[2] = p[0];
    if (p[1] > f.box[3]) f.box[3] = p[1];
  }
  const list = f.quadList && f.quadList.length === quads.length ? f.quadList : (f.quadList = quads.map(() => [new Float64Array(0), 0]));
  for (let qi = 0; qi < quads.length; qi++) {
    const q = quads[qi];
    let entry = list[qi];
    if (entry[0].length < 2 * q.length) entry[0] = new Float64Array(2 * q.length);
    const buf = entry[0];
    for (let i = 0; i < q.length; i++) {
      buf[2 * i] = q[i][0];
      buf[2 * i + 1] = q[i][1];
    }
    entry[1] = q.length;
  }
  f.quadN = quads.length;
  f.quads = f.quadList;
  f.hullN = 0;
}

function t3Emit(run, cam, out) {
  const f = out ?? t3Frame(run);
  if (run.kind === "prism") t3EmitPrism(run, cam, f);
  else if (run.kind === "round") t3EmitRound(run, cam, f);
  else if (run.kind === "ball") t3EmitBall(run, cam, f);
  else if (run.kind === "lathe") t3EmitLathe(run, cam, f);
  else if (run.kind === "tube") {
    t3EmitTube(run, cam, f);
    return f;
  } else if (run.kind === "fixed") {
    t3EmitFixed(run, cam, f);
    return f;
  }
  t3EmitPlanes(run, cam, f);
  t3EmitDots(run, cam, f);
  return f;
}

function t3Overlap(a, b, pad) {
  return a[0] < b[2] - pad && b[0] < a[2] - pad && a[1] < b[3] - pad && b[1] < a[3] - pad;
}

function t3Apart(A, an, B, bn, pad) {
  for (let pass = 0; pass < 2; pass++) {
    const P = pass ? B : A;
    const pn = pass ? bn : an;
    for (let i = 0; i < pn; i++) {
      const j = (i + 1) % pn;
      const nx = P[2 * j + 1] - P[2 * i + 1];
      const ny = P[2 * i] - P[2 * j];
      const l = Math.hypot(nx, ny);
      if (l < 1e-9) continue;
      let a0 = Infinity;
      let a1 = -Infinity;
      for (let q = 0; q < an; q++) {
        const v = (A[2 * q] * nx + A[2 * q + 1] * ny) / l;
        if (v < a0) a0 = v;
        if (v > a1) a1 = v;
      }
      let b0 = Infinity;
      let b1 = -Infinity;
      for (let q = 0; q < bn; q++) {
        const v = (B[2 * q] * nx + B[2 * q + 1] * ny) / l;
        if (v < b0) b0 = v;
        if (v > b1) b1 = v;
      }
      if (a1 < b0 + pad || b1 < a0 + pad) return true;
    }
  }
  return false;
}

function t3Disjoint(fa, fb, pad) {
  const qa = fa.quadN ? fa.quads : null;
  const qb = fb.quadN ? fb.quads : null;
  if (!qa && !qb) return t3Apart(fa.hull, fa.hullN, fb.hull, fb.hullN, pad);
  const listA = qa ? fa.quadList : [[fa.hull, fa.hullN]];
  const listB = qb ? fb.quadList : [[fb.hull, fb.hullN]];
  for (const [A, an] of listA) for (const [B, bn] of listB) if (!t3Apart(A, an, B, bn, pad)) return false;
  return true;
}

function t3Support(cloud, R, t, dir, world) {
  let best = -Infinity;
  let at = 0;
  const n = cloud.length / 3;
  const dx = R[0] * dir[0] + R[3] * dir[1] + R[6] * dir[2];
  const dy = R[1] * dir[0] + R[4] * dir[1] + R[7] * dir[2];
  const dz = R[2] * dir[0] + R[5] * dir[1] + R[8] * dir[2];
  for (let i = 0; i < n; i++) {
    const v = cloud[3 * i] * dx + cloud[3 * i + 1] * dy + cloud[3 * i + 2] * dz;
    if (v > best) {
      best = v;
      at = i;
    }
  }
  const x = cloud[3 * at];
  const y = cloud[3 * at + 1];
  const z = cloud[3 * at + 2];
  world[0] = R[0] * x + R[1] * y + R[2] * z + t[0];
  world[1] = R[3] * x + R[4] * y + R[5] * z + t[1];
  world[2] = R[6] * x + R[7] * y + R[8] * z + t[2];
  return best + dir[0] * t[0] + dir[1] * t[1] + dir[2] * t[2];
}

function t3Gjk(cloudA, camA, cloudB, camB, warm, rounds = 8, hints = null) {
  let w = warm && (warm[0] || warm[1] || warm[2]) ? [warm[0], warm[1], warm[2]] : null;
  const a = [0, 0, 0];
  const b = [0, 0, 0];
  if (!w) {
    t3Support(cloudA, camA.R, camA.t, [1, 0, 0], a);
    t3Support(cloudB, camB.R, camB.t, [-1, 0, 0], b);
    w = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
  }
  let margin = -Infinity;
  let best = w;
  const step = (limit) => {
    for (let it = 0; it < limit; it++) {
      const l = Math.hypot(w[0], w[1], w[2]);
      if (l < 1e-12) break;
      const n = [w[0] / l, w[1] / l, w[2] / l];
      const hiA = t3Support(cloudA, camA.R, camA.t, n, a);
      const loB = -t3Support(cloudB, camB.R, camB.t, [-n[0], -n[1], -n[2]], b);
      const value = loB - hiA;
      if (value > margin) {
        margin = value;
        best = n;
      }
      const s = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
      const ds = [w[0] - s[0], w[1] - s[1], w[2] - s[2]];
      const dd = ds[0] * ds[0] + ds[1] * ds[1] + ds[2] * ds[2];
      if (dd < 1e-14) break;
      const tt = Math.max(0, Math.min(1, (w[0] * ds[0] + w[1] * ds[1] + w[2] * ds[2]) / dd));
      w = [w[0] - ds[0] * tt, w[1] - ds[1] * tt, w[2] - ds[2] * tt];
    }
  };
  step(rounds);
  if (margin < 0) step(24);
  if (margin < 0.05 && hints) {
    if (hints.centroids) {
      const [ca, cb] = hints.centroids;
      const wa = [camA.R[0] * ca[0] + camA.R[1] * ca[1] + camA.R[2] * ca[2] + camA.t[0], camA.R[3] * ca[0] + camA.R[4] * ca[1] + camA.R[5] * ca[2] + camA.t[1], camA.R[6] * ca[0] + camA.R[7] * ca[1] + camA.R[8] * ca[2] + camA.t[2]];
      const wb = [camB.R[0] * cb[0] + camB.R[1] * cb[1] + camB.R[2] * cb[2] + camB.t[0], camB.R[3] * cb[0] + camB.R[4] * cb[1] + camB.R[5] * cb[2] + camB.t[1], camB.R[6] * cb[0] + camB.R[7] * cb[1] + camB.R[8] * cb[2] + camB.t[2]];
      const n = t3Unit([wb[0] - wa[0], wb[1] - wa[1], wb[2] - wa[2]]);
      const hiA = t3Support(cloudA, camA.R, camA.t, n, a);
      const loB = -t3Support(cloudB, camB.R, camB.t, [-n[0], -n[1], -n[2]], b);
      if (loB - hiA > margin) {
        margin = loB - hiA;
        best = n;
      }
    }
    for (const [axis, cam] of hints) {
      const R = cam.R;
      const w0 = [R[0] * axis[0] + R[1] * axis[1] + R[2] * axis[2], R[3] * axis[0] + R[4] * axis[1] + R[5] * axis[2], R[6] * axis[0] + R[7] * axis[1] + R[8] * axis[2]];
      for (const sign of [1, -1]) {
        const n = [w0[0] * sign, w0[1] * sign, w0[2] * sign];
        const hiA = t3Support(cloudA, camA.R, camA.t, n, a);
        const loB = -t3Support(cloudB, camB.R, camB.t, [-n[0], -n[1], -n[2]], b);
        if (loB - hiA > margin) {
          margin = loB - hiA;
          best = n;
        }
      }
    }
  }
  const l = Math.hypot(best[0], best[1], best[2]) || 1;
  return { n: [best[0] / l, best[1] / l, best[2] / l], margin };
}

function t3Order(layer, frames, cams, previous, stats, scene) {
  const parts = layer.parts;
  const count = parts.length;
  const local = layer.local;
  const rank = layer._rank ?? (layer._rank = new Int32Array(count));
  if (previous) for (let at = 0; at < count; at++) rank[local.get(previous[at])] = at;
  else for (let at = 0; at < count; at++) rank[local.get(layer.rest[at])] = at;
  let groupEdges = 0;
  if (layer.groupPairs) for (const gp of layer.groupPairs) groupEdges += gp.a.length * gp.b.length;
  const maxEdges = layer.planar.length / 5 + (layer.framed ? layer.framed.length / 6 : 0) + groupEdges + layer.dynamic.length / 2 + (layer.free ? layer.free.length * count : 0) + 1;
  const head = layer._head ?? (layer._head = new Int32Array(count));
  const next = layer._next && layer._next.length >= maxEdges ? layer._next : (layer._next = new Int32Array(maxEdges));
  const to = layer._to && layer._to.length >= maxEdges ? layer._to : (layer._to = new Int32Array(maxEdges));
  const live = layer._live && layer._live.length >= maxEdges ? layer._live : (layer._live = new Uint8Array(maxEdges));
  const indeg = layer._indeg ?? (layer._indeg = new Int32Array(count));
  head.fill(-1);
  indeg.fill(0);
  let edges = 0;
  const addEdge = (back, front) => {
    to[edges] = front;
    next[edges] = head[back];
    live[edges] = 1;
    head[back] = edges;
    indeg[front]++;
    edges++;
  };
  const runs = scene ? scene.runs : null;
  const groupOf = (global) => (runs ? runs[global].group : -1);
  const planar = layer.planar;
  for (let p = 0; p < planar.length; p += 5) {
    const i = planar[p];
    const j = planar[p + 1];
    const fi = frames[i];
    const fj = frames[j];
    if (!t3Overlap(fi.box, fj.box, -0.05)) continue;
    const g = groupOf(i);
    const V = g >= 0 ? cams[g].V : cams.world.V;
    const s = planar[p + 2] * V[0] + planar[p + 3] * V[1] + planar[p + 4] * V[2];
    if (s < 1e-7 && s > -1e-7) continue;
    const li = local.get(i);
    const lj = local.get(j);
    if (s > 0) addEdge(li, lj);
    else addEdge(lj, li);
  }
  const framed = layer.framed;
  if (framed)
    for (let p = 0; p < framed.length; p += 6) {
      const i = framed[p];
      const j = framed[p + 1];
      if (!t3Overlap(frames[i].box, frames[j].box, -0.05)) continue;
      const g = framed[p + 2];
      const V = g >= 0 ? cams[g].V : cams.world.V;
      const s = framed[p + 3] * V[0] + framed[p + 4] * V[1] + framed[p + 5] * V[2];
      if (s < 1e-7 && s > -1e-7) continue;
      const li = local.get(i);
      const lj = local.get(j);
      if (s > 0) addEdge(li, lj);
      else addEdge(lj, li);
    }
  if (layer.groupPairs)
    for (const gp of layer.groupPairs) {
      const union = (list, box) => {
        box[0] = Infinity;
        box[1] = Infinity;
        box[2] = -Infinity;
        box[3] = -Infinity;
        for (let q = 0; q < list.length; q++) {
          const b = frames[list[q]].box;
          if (b[0] < box[0]) box[0] = b[0];
          if (b[1] < box[1]) box[1] = b[1];
          if (b[2] > box[2]) box[2] = b[2];
          if (b[3] > box[3]) box[3] = b[3];
        }
      };
      union(gp.a, gp.boxA);
      union(gp.b, gp.boxB);
      if (!t3Overlap(gp.boxA, gp.boxB, -0.05)) continue;
      const V = gp.frame >= 0 ? cams[gp.frame].V : cams.world.V;
      const s = gp.n[0] * V[0] + gp.n[1] * V[1] + gp.n[2] * V[2];
      if (s < 1e-7 && s > -1e-7) continue;
      for (let x = 0; x < gp.a.length; x++) {
        const i = gp.a[x];
        const bi = frames[i].box;
        if (!t3Overlap(bi, gp.boxB, -0.05)) continue;
        for (let y = 0; y < gp.b.length; y++) {
          const j = gp.b[y];
          if (!t3Overlap(bi, frames[j].box, -0.05)) continue;
          if (s > 0) addEdge(local.get(i), local.get(j));
          else addEdge(local.get(j), local.get(i));
        }
      }
    }
  const dynamic = layer.dynamic;
  let unseparated = 0;
  let solved = 0;
  const rel = layer.rel ?? (layer.rel = new Map());
  const cross = (i, j) => {
    const ri = runs[i];
    const rj = runs[j];
    const ci = ri.group >= 0 ? cams[ri.group] : cams.world;
    const cj = rj.group >= 0 ? cams[rj.group] : cams.world;
    const A = ci.R;
    const B = cj.R;
    const dx = cj.t[0] - ci.t[0];
    const dy = cj.t[1] - ci.t[1];
    const dz = cj.t[2] - ci.t[2];
    const key = i * 65536 + j;
    let memo = rel.get(key);
    if (!memo) {
      memo = { pose: new Float64Array(12), n: [0, 0, 0], margin: -Infinity, fresh: false };
      rel.set(key, memo);
    }
    const pose = memo.pose;
    let same = memo.fresh;
    let at = 0;
    for (let c = 0; c < 3; c++)
      for (let r = 0; r < 3; r++) {
        const v = A[c] * B[r] + A[3 + c] * B[3 + r] + A[6 + c] * B[6 + r];
        if (same && Math.abs(pose[at] - v) > 1e-12) same = false;
        pose[at++] = v;
      }
    for (let c = 0; c < 3; c++) {
      const v = A[c] * dx + A[3 + c] * dy + A[6 + c] * dz;
      if (same && Math.abs(pose[at] - v) > 1e-9) same = false;
      pose[at++] = v;
    }
    let nx;
    let ny;
    let nz;
    if (same) {
      const n = memo.n;
      nx = A[0] * n[0] + A[1] * n[1] + A[2] * n[2];
      ny = A[3] * n[0] + A[4] * n[1] + A[5] * n[2];
      nz = A[6] * n[0] + A[7] * n[1] + A[8] * n[2];
    } else {
      const n = memo.n;
      const warm = memo.fresh ? [A[0] * n[0] + A[1] * n[1] + A[2] * n[2], A[3] * n[0] + A[4] * n[1] + A[5] * n[2], A[6] * n[0] + A[7] * n[1] + A[8] * n[2]] : null;
      const hints = [...ri.axes.map((axis) => [axis, ci]), ...rj.axes.map((axis) => [axis, cj]), [[0, 0, 1], cams.world]];
      hints.centroids = [ri.centroid, rj.centroid];
      const hit = t3Gjk(ri.cloud, ci, rj.cloud, cj, warm, 8, hints);
      solved++;
      nx = hit.n[0];
      ny = hit.n[1];
      nz = hit.n[2];
      n[0] = A[0] * nx + A[3] * ny + A[6] * nz;
      n[1] = A[1] * nx + A[4] * ny + A[7] * nz;
      n[2] = A[2] * nx + A[5] * ny + A[8] * nz;
      memo.margin = hit.margin;
      memo.fresh = true;
    }
    if (memo.margin < -0.08) unseparated++;
    const V = cams.world.V;
    return nx * V[0] + ny * V[1] + nz * V[2];
  };
  for (let p = 0; p < dynamic.length; p += 2) {
    const i = dynamic[p];
    const j = dynamic[p + 1];
    if (!t3Overlap(frames[i].box, frames[j].box, -0.05)) continue;
    const s = cross(i, j);
    if (s < 1e-7 && s > -1e-7) continue;
    const li = local.get(i);
    const lj = local.get(j);
    if (s > 0) addEdge(li, lj);
    else addEdge(lj, li);
  }
  const loose = layer.free;
  if (loose && loose.length) {
    const isLoose = layer.looseAt;
    for (let a = 0; a < loose.length; a++) {
      const i = loose[a];
      const gi = runs[i].group;
      const bi = frames[i].box;
      for (let b = 0; b < count; b++) {
        const j = parts[b];
        if (j === i || runs[j].group === gi) continue;
        if (isLoose[b] && j < i) continue;
        if (!t3Overlap(bi, frames[j].box, -0.05)) continue;
        const s = cross(i, j);
        if (s < 1e-7 && s > -1e-7) continue;
        const li = local.get(i);
        if (s > 0) addEdge(li, b);
        else addEdge(b, li);
      }
    }
  }
  const heap = layer._heap ?? (layer._heap = new Int32Array(count));
  const done = layer._done ?? (layer._done = new Uint8Array(count));
  done.fill(0);
  let size = 0;
  const less = (a, b) => rank[a] < rank[b];
  const push = (v) => {
    let i = size++;
    heap[i] = v;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      const tmp = heap[i];
      heap[i] = heap[p];
      heap[p] = tmp;
      i = p;
    }
  };
  const pop = () => {
    const top = heap[0];
    heap[0] = heap[--size];
    let i = 0;
    for (;;) {
      const l = 2 * i + 1;
      const r = l + 1;
      let s = i;
      if (l < size && less(heap[l], heap[s])) s = l;
      if (r < size && less(heap[r], heap[s])) s = r;
      if (s === i) break;
      const tmp = heap[i];
      heap[i] = heap[s];
      heap[s] = tmp;
      i = s;
    }
    return top;
  };
  for (let v = 0; v < count; v++) if (indeg[v] === 0) push(v);
  const out = new Int32Array(count);
  let placed = 0;
  let forced = 0;
  let refined = 0;
  let tested = false;
  const mark = layer._mark && layer._mark.length >= count ? layer._mark : (layer._mark = new Uint8Array(count));
  const via = layer._via && layer._via.length >= count ? layer._via : (layer._via = new Int32Array(count));
  const iter = layer._iter && layer._iter.length >= count ? layer._iter : (layer._iter = new Int32Array(count));
  const stack = layer._stack && layer._stack.length >= count ? layer._stack : (layer._stack = new Int32Array(count));
  const checked = layer._checked && layer._checked.length >= maxEdges ? layer._checked : (layer._checked = new Uint8Array(maxEdges));
  checked.fill(0, 0, edges);
  const breakCycle = () => {
    mark.fill(0, 0, count);
    for (let root = 0; root < count; root++) {
      if (done[root] || mark[root]) continue;
      let top = 0;
      stack[top++] = root;
      mark[root] = 1;
      iter[root] = head[root];
      while (top) {
        const v = stack[top - 1];
        let e = iter[v];
        while (e >= 0 && (!live[e] || done[to[e]] || mark[to[e]] === 2)) e = next[e];
        if (e < 0) {
          mark[v] = 2;
          top--;
          continue;
        }
        iter[v] = next[e];
        const w = to[e];
        if (mark[w] === 1) {
          let dropped = false;
          let x = v;
          let edge = e;
          for (;;) {
            if (!checked[edge]) {
              checked[edge] = 1;
              const a = x;
              const b = to[edge];
              if (t3Disjoint(frames[parts[a]], frames[parts[b]], 0.1)) {
                live[edge] = 0;
                indeg[b]--;
                refined++;
                dropped = true;
                if (indeg[b] === 0) push(b);
              }
            }
            if (x === w) break;
            edge = via[x];
            let from = -1;
            for (let s = 0; s < top; s++) if (stack[s] === x) from = s > 0 ? stack[s - 1] : -1;
            if (from < 0) break;
            x = from;
          }
          return dropped ? 1 : 0;
        }
        via[w] = e;
        mark[w] = 1;
        iter[w] = head[w];
        stack[top++] = w;
      }
    }
    return -1;
  };
  while (placed < count) {
    if (size === 0) {
      let broken = 0;
      for (let tries = 0; tries < 64 && size === 0; tries++) {
        const result = breakCycle();
        if (result <= 0) break;
        broken++;
      }
      if (size) continue;
      void broken;
      void tested;
      let pick = -1;
      for (let v = 0; v < count; v++) if (!done[v] && (pick < 0 || indeg[v] < indeg[pick] || (indeg[v] === indeg[pick] && rank[v] < rank[pick]))) pick = v;
      forced++;
      indeg[pick] = 0;
      push(pick);
    }
    const v = pop();
    if (done[v]) continue;
    done[v] = 1;
    out[placed++] = parts[v];
    for (let e = head[v]; e >= 0; e = next[e]) {
      if (!live[e]) continue;
      const w = to[e];
      if (done[w]) continue;
      if (--indeg[w] === 0) push(w);
    }
  }
  if (stats) {
    stats.edges = (stats.edges ?? 0) + edges;
    stats.forced = (stats.forced ?? 0) + forced;
    stats.refined = (stats.refined ?? 0) + refined;
    stats.unseparated = (stats.unseparated ?? 0) + unseparated;
    stats.solved = (stats.solved ?? 0) + solved;
  }
  return out;
}

function t3Keep(values) {
  const n = values.length;
  const tails = [];
  const tailAt = [];
  const back = new Int32Array(n).fill(-1);
  for (let i = 0; i < n; i++) {
    let lo = 0;
    let hi = tails.length;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (tails[mid] < values[i]) lo = mid + 1;
      else hi = mid;
    }
    tails[lo] = values[i];
    tailAt[lo] = i;
    back[i] = lo > 0 ? tailAt[lo - 1] : -1;
  }
  const keep = new Uint8Array(n);
  let k = tails.length ? tailAt[tails.length - 1] : -1;
  while (k >= 0) {
    keep[k] = 1;
    k = back[k];
  }
  return keep;
}

function t3Cams(scene, values) {
  const poses = t3Poses(scene.groups, values);
  const cams = poses.map((pose) => t3Camera(scene.P, pose));
  cams.world = t3Camera(scene.P, null);
  cams.poses = poses;
  return cams;
}

function t3Mount(stage, data, { onHold = null, onFrame = null, follow = null, painter = "svg" } = {}) {
  void painter;
  const layerEls = new Map([...stage.querySelectorAll("svg[data-live]")].map((svg) => [svg.dataset.live, svg]));
  const first = layerEls.values().next().value;
  const viewWidth = first && first.viewBox && first.viewBox.baseVal ? first.viewBox.baseVal.width : 0;
  const unitOf = () => {
    const width = first ? first.getBoundingClientRect().width : 0;
    return viewWidth && width ? Math.round((width / viewWidth) * 100) / 100 : 1;
  };
  let unit = unitOf();
  let scene = t3Prepare(data, { unit });
  const runs = () => scene.runs;
  const count = data.parts.length;
  const nodes = new Array(count);
  const bound = new Array(count);
  const cache = new Array(count);
  const containers = new Map();
  for (const [name, svg] of layerEls) containers.set(name, svg.querySelector(".iso-parts") ?? svg);
  for (const run of scene.runs) {
    const layer = data.layers[run.layer];
    const container = containers.get(layer.name);
    const node = container ? container.querySelector(`[data-p="${run.index}"]`) : null;
    nodes[run.index] = node;
    const map = {};
    if (node) {
      map.move = node;
      node.querySelectorAll("[data-r]").forEach((el) => (map[el.getAttribute("data-r")] = el));
    }
    bound[run.index] = run.roles.map((role) => map[role.name] ?? null);
    cache[run.index] = run.roles.map(() => ({ d: null, q: null, b: null, f: null, w: null, t: null, o: null, cx: null, cy: null }));
  }
  const frames = scene.runs.map((run) => t3Frame(run));
  const values = Object.fromEntries(data.groups.map((g) => [g.name, data.values ? data.values[g.name] ?? 0 : 0]));
  let lastPoses = null;
  let cams = null;
  const orders = scene.layers.map((layer) => Int32Array.from(layer.rest));
  const changes = [];
  const stats = { ms: 0, writes: 0, moved: 0, forced: 0, refined: 0, unseparated: 0, emitted: 0 };
  let writes = 0;
  const write = (index) => {
    const run = scene.runs[index];
    const f = frames[index];
    const els = bound[index];
    const memo = cache[index];
    for (let r = 0; r < run.roles.length; r++) {
      const el = els[r];
      if (!el) continue;
      const role = run.roles[r];
      const c = memo[r];
      const type = role.type;
      if (type === "path" || type === "face" || type === "slot") {
        const d = f.d[r];
        if (c.d !== d) {
          el.setAttribute("d", d);
          c.d = d;
          writes++;
        }
        if (type === "face") {
          const q = f.q[r];
          if (c.q !== q) {
            const band = Math.min(3, Math.floor(q / T3_Q));
            const mix = (q - T3_Q * band) / T3_Q;
            if (c.b !== band) {
              el.setAttribute("data-b", String(band));
              c.b = band;
              writes++;
            }
            if (c.f !== mix) {
              if (mix) el.style.setProperty("--f", String(mix));
              else el.style.removeProperty("--f");
              c.f = mix;
              writes++;
            }
            c.q = q;
          }
          const w = f.w[r];
          if (c.w !== w) {
            if (w < 0) el.style.removeProperty("stroke-width");
            else el.style.strokeWidth = String(w);
            c.w = w;
            writes++;
          }
        } else if (type === "slot") {
          const o = f.o[r];
          if (c.o !== o) {
            el.setAttribute("opacity", String(Math.round(o * 1000) / 1000));
            c.o = o;
            writes++;
          }
        }
      } else if (type === "xform" || type === "self" || type === "plane") {
        const t = f.t[r];
        if (c.t !== t) {
          if (t) el.setAttribute("transform", t);
          else el.removeAttribute("transform");
          c.t = t;
          writes++;
        }
        if (type === "plane") {
          const o = f.o[r];
          if (c.o !== o) {
            el.setAttribute("opacity", String(o));
            c.o = o;
            writes++;
          }
        }
      } else if (type === "dot") {
        const x = Math.round(f.cx[r] * 100) / 100;
        const y = Math.round(f.cy[r] * 100) / 100;
        const o = f.o[r];
        if (c.cx !== x) {
          el.setAttribute("cx", String(x));
          c.cx = x;
          writes++;
        }
        if (c.cy !== y) {
          el.setAttribute("cy", String(y));
          c.cy = y;
          writes++;
        }
        if (c.o !== o) {
          el.setAttribute("fill-opacity", String(o));
          c.o = o;
          writes++;
        }
      }
    }
  };
  const samePose = (a, b) => {
    if (!a || !b) return false;
    for (let i = 0; i < 9; i++) if (a.R[i] !== b.R[i]) return false;
    return a.t[0] === b.t[0] && a.t[1] === b.t[1] && a.t[2] === b.t[2];
  };
  let lastUnit = unit;
  let moved = 0;
  const render = () => {
    const t0 = performance.now();
    writes = 0;
    moved = 0;
    cams = t3Cams(scene, values);
    t3State.detail = detail;
    const poses = cams.poses;
    const fresh = lastUnit !== unit || !lastPoses;
    let emitted = 0;
    for (const run of scene.runs) {
      const g = run.group;
      const changed = fresh || (g >= 0 ? !samePose(poses[g], lastPoses[g]) : false);
      if (!changed) continue;
      t3Emit(run, g >= 0 ? cams[g] : cams.world, frames[run.index]);
      write(run.index);
      emitted++;
    }
    lastPoses = poses;
    lastUnit = unit;
    const orderStats = { forced: 0, refined: 0, unseparated: 0, edges: 0 };
    scene.layers.forEach((layer, index) => {
      const previous = orders[index];
      const order = t3Order(layer, frames, cams, previous, orderStats, scene);
      let same = true;
      for (let j = 0; j < order.length; j++)
        if (order[j] !== previous[j]) {
          same = false;
          break;
        }
      if (same) return;
      const container = containers.get(layer.name);
      const position = new Map();
      previous.forEach((global, at) => position.set(global, at));
      const keep = t3Keep(Array.from(order, (global) => position.get(global)));
      let after = null;
      const shifted = [];
      for (let j = order.length - 1; j >= 0; j--) {
        const node = nodes[order[j]];
        if (!keep[j] && node && container) {
          container.insertBefore(node, after);
          moved++;
          shifted.push(scene.runs[order[j]].name);
        }
        if (node) after = node;
      }
      orders[index] = order;
      if (shifted.length) changes.push({ layer: layer.name, moved: shifted });
    });
    stats.ms = performance.now() - t0;
    stats.writes = writes;
    stats.moved = moved;
    stats.forced = orderStats.forced;
    stats.refined = orderStats.refined;
    stats.unseparated = orderStats.unseparated;
    stats.emitted = emitted;
    t3State.detail = 1;
    if (onFrame) onFrame(controller);
    return { ...stats };
  };
  let detail = 1;
  const set = (next, options) => {
    if (options && options.detail !== undefined) {
      const value = Math.max(0, Math.min(1, options.detail));
      if (value !== detail) {
        detail = value;
        lastPoses = null;
      }
    }
    if (typeof next === "number") {
      const g = data.groups.find((group) => group.kind === "turn");
      if (g) values[g.name] = next;
    } else if (next) for (const [name, value] of Object.entries(next)) values[name] = value;
    if (follow) Object.assign(values, follow({ ...values }));
    return render();
  };
  let unwrap = new Map();
  const angleAt = (clientX, clientY, z = 0, groupName) => {
    const svg = first;
    const box = svg.getBoundingClientRect();
    const vb = svg.viewBox.baseVal;
    const x = vb.x + ((clientX - box.left) / box.width) * vb.width;
    const y = vb.y + ((clientY - box.top) / box.height) * vb.height;
    const c = t3Camera(scene.P, null);
    const p = (x - c.ox) / c.k;
    const q = (y - c.oy + z * c.ce * c.k) / (c.se * c.k);
    const wx = p * c.sa + q * c.ca;
    const wy = -p * c.ca + q * c.sa;
    const group = data.groups.find((g) => g.name === groupName) ?? data.groups.find((g) => g.kind === "turn");
    const pivot = group ? group.origin : [0, 0, 0];
    const raw = (Math.atan2(wy - pivot[1], wx - pivot[0]) * 180) / Math.PI;
    const key = group ? group.name : "";
    const last = unwrap.get(key);
    let angle = raw;
    if (last !== undefined) angle = last + ((((raw - last) % 360) + 540) % 360) - 180;
    unwrap.set(key, angle);
    return angle;
  };
  const observer = typeof ResizeObserver === "function" && first
    ? new ResizeObserver(() => {
        const next = unitOf();
        if (Math.abs(next - unit) < 0.05) return;
        unit = next;
        scene = t3Prepare(data, { unit });
        render();
      })
    : null;
  if (observer) observer.observe(first);
  if (follow) Object.assign(values, follow({ ...values }));
  const controller = {
    set,
    render,
    angleAt,
    get values() {
      return { ...values };
    },
    cover: (names, options) => t3Cover(controller, names, options),
    order: (name) => {
      const index = name === undefined ? 0 : scene.layers.findIndex((layer) => layer.name === name);
      return index < 0 ? [] : Array.from(orders[index], (global) => scene.runs[global].name);
    },
    orderIndex: (name) => {
      const index = name === undefined ? 0 : scene.layers.findIndex((layer) => layer.name === name);
      return index < 0 ? null : orders[index];
    },
    events: () => changes.splice(0, changes.length),
    stats: () => ({ ...stats }),
    frames: () => frames,
    cams: () => cams,
    scene: () => scene,
    pose: (name) => t3PoseOf(scene, cams, name),
    detach: (name) => {
      const index = scene.groups.findIndex((g) => g.name === name);
      const pose = t3PoseOf(scene, cams, name);
      const o = scene.groups[index].origin;
      return { world: true, R: pose.R, t: [pose.t[0] - o[0], pose.t[1] - o[1], pose.t[2] - o[2]] };
    },
    anchor: (name, point, direction) => t3Anchor(scene, cams, name, point, direction),
    destroy: () => {
      if (observer) observer.disconnect();
      if (typeof window !== "undefined" && window.__isoTurn && window.__isoTurn.controller === controller) delete window.__isoTurn;
    },
  };
  const hold = () => {
    if (onHold) onHold();
  };
  if (typeof window !== "undefined") {
    window.__isoTurn = {
      controller,
      set(next) {
        hold();
        return set(next);
      },
      get() {
        hold();
        return { ...values };
      },
      order(name) {
        hold();
        return controller.order(name);
      },
      events() {
        hold();
        return controller.events();
      },
      stats() {
        hold();
        return controller.stats();
      },
      layers() {
        hold();
        return scene.layers.map((layer) => layer.name);
      },
    };
  }
  render();
  return controller;
}

function t3Enclose(points, edges) {
  const hull = t3Hull(points);
  const n = hull.length;
  if (n < 3) return [];
  let area = 0;
  for (let i = 0; i < n; i++) area += hull[i][0] * hull[(i + 1) % n][1] - hull[(i + 1) % n][0] * hull[i][1];
  const sign = area >= 0 ? 1 : -1;
  let lines = [];
  for (let i = 0; i < n; i++) {
    const p = hull[i];
    const q = hull[(i + 1) % n];
    const dx = q[0] - p[0];
    const dy = q[1] - p[1];
    const l = Math.hypot(dx, dy);
    if (l < 1e-9) continue;
    const nx = (sign * dy) / l;
    const ny = (-sign * dx) / l;
    lines.push([nx, ny, -(nx * p[0] + ny * p[1])]);
  }
  const meet = (A, B) => {
    const det = A[0] * B[1] - A[1] * B[0];
    if (Math.abs(det) < 1e-12) return null;
    return [(-A[2] * B[1] + B[2] * A[1]) / det, (-A[0] * B[2] + B[0] * A[2]) / det];
  };
  while (lines.length > edges) {
    let best = -1;
    let cost = Infinity;
    const m = lines.length;
    for (let i = 0; i < m; i++) {
      const prev = lines[(i - 1 + m) % m];
      const self = lines[i];
      const next = lines[(i + 1) % m];
      const turn = prev[0] * next[1] - prev[1] * next[0];
      if (sign * turn <= 1e-9) continue;
      const apex = meet(prev, next);
      const a = meet(prev, self);
      const b = meet(self, next);
      if (!apex || !a || !b) continue;
      const added = Math.abs((a[0] - apex[0]) * (b[1] - apex[1]) - (a[1] - apex[1]) * (b[0] - apex[0])) / 2;
      if (added < cost) {
        cost = added;
        best = i;
      }
    }
    if (best < 0) break;
    lines.splice(best, 1);
  }
  return lines;
}

function t3Cover(controller, names, { polygons = 6, edges = 10, pad = 0.3, layers = null } = {}) {
  const uCoverEdge = new Float32Array(polygons * edges * 3);
  const uCoverCount = new Float32Array(polygons);
  const scene = controller.scene();
  const frames = controller.frames();
  const sets = Array.isArray(names) ? (names.length && Array.isArray(names[0]) ? names : [names]) : [[names]];
  const candidates = [];
  const taken = new Set();
  for (const list of sets) {
    const targets = scene.runs.filter((run) => list.includes(run.name));
    if (!targets.length) continue;
    const box = [Infinity, Infinity, -Infinity, -Infinity];
    for (const run of targets) {
      const b = frames[run.index].box;
      box[0] = Math.min(box[0], b[0]);
      box[1] = Math.min(box[1], b[1]);
      box[2] = Math.max(box[2], b[2]);
      box[3] = Math.max(box[3], b[3]);
    }
    const home = targets[0].layer;
    scene.layers.forEach((layer, index) => {
      const order = controller.orderIndex(layer.name);
      if (!order) return;
      let from = 0;
      if (index === home) {
        for (let at = 0; at < order.length; at++) if (targets.some((run) => run.index === order[at])) from = at + 1;
      } else if (!(layers && layers.includes(layer.name)) || index < home) return;
      for (let at = from; at < order.length; at++) {
        const run = scene.runs[order[at]];
        if (list.includes(run.name) || taken.has(run.index)) continue;
        const f = frames[run.index];
        const b = f.box;
        if (!(b[0] < box[2] + pad && box[0] < b[2] + pad && b[1] < box[3] + pad && box[1] < b[3] + pad)) continue;
        if (!targets.some((target) => !t3Disjoint(frames[target.index], f, -pad))) continue;
        taken.add(run.index);
        const share = (Math.min(b[2], box[2]) - Math.max(b[0], box[0])) * (Math.min(b[3], box[3]) - Math.max(b[1], box[1]));
        candidates.push({ run, f, share });
      }
    });
  }
  candidates.sort((a, b) => b.share - a.share);
  const parts = [];
  let slot = 0;
  for (const { run, f } of candidates) {
    if (slot >= polygons) break;
    const points = [];
    if (f.quadN) for (const [buf, count] of f.quadList) for (let i = 0; i < count; i++) points.push([buf[2 * i], buf[2 * i + 1]]);
    else for (let i = 0; i < f.hullN; i++) points.push([f.hull[2 * i], f.hull[2 * i + 1]]);
    const lines = t3Enclose(points, edges);
    if (!lines.length) continue;
    lines.forEach((line, e) => {
      const at = (slot * edges + e) * 3;
      uCoverEdge[at] = line[0];
      uCoverEdge[at + 1] = line[1];
      uCoverEdge[at + 2] = line[2] - pad;
    });
    uCoverCount[slot] = lines.length;
    parts.push(run.name);
    slot++;
  }
  return { uCoverEdge, uCoverCount, dropped: Math.max(0, candidates.length - slot), parts };
}

function t3PoseOf(scene, cams, name) {
  const index = scene.groups.findIndex((g) => g.name === name);
  const cam = index >= 0 && cams ? cams[index] : null;
  return { R: cam ? Array.from(cam.R) : Array.from(T3_EYE), t: cam ? [cam.t[0], cam.t[1], cam.t[2]] : [0, 0, 0] };
}

function t3Anchor(scene, cams, name, point, direction) {
  const index = name === null || name === undefined ? -1 : scene.groups.findIndex((g) => g.name === name);
  const cam = index >= 0 ? cams[index] : cams.world;
  const o = index >= 0 ? scene.groups[index].origin : [0, 0, 0];
  const R = cam.R;
  const q = [point[0] - o[0], point[1] - o[1], point[2] - o[2]];
  const world = [R[0] * q[0] + R[1] * q[1] + R[2] * q[2] + cam.t[0], R[3] * q[0] + R[4] * q[1] + R[5] * q[2] + cam.t[1], R[6] * q[0] + R[7] * q[1] + R[8] * q[2] + cam.t[2]];
  const w = cams.world;
  const screen = [w.ox + w.m[0] * world[0] + w.m[1] * world[1] + w.m[2] * world[2], w.oy + w.m[3] * world[0] + w.m[4] * world[1] + w.m[5] * world[2]];
  const out = { world, screen };
  if (direction) out.direction = [R[0] * direction[0] + R[1] * direction[1] + R[2] * direction[2], R[3] * direction[0] + R[4] * direction[1] + R[5] * direction[2], R[6] * direction[0] + R[7] * direction[1] + R[8] * direction[2]];
  return out;
}

function t3FloorOf(scene, name, R = T3_EYE) {
  const index = scene.groups.findIndex((g) => g.name === name);
  let low = Infinity;
  for (const run of scene.runs) {
    if (run.group !== index) continue;
    const c = run.cloud;
    for (let i = 0; i < c.length; i += 3) {
      const z = R[6] * c[i] + R[7] * c[i + 1] + R[8] * c[i + 2];
      if (z < low) low = z;
    }
  }
  return low;
}

function t3Compose(A, B) {
  return Array.from(t3Mul(A, B));
}

function t3PoseUniforms(controller, group, prefix = "uGroup") {
  const scene = controller.scene();
  const index = scene.groups.findIndex((g) => g.name === group);
  const cams = controller.cams();
  const cam = index >= 0 && cams ? cams[index] : null;
  const R = cam ? cam.R : T3_EYE;
  const t = cam ? cam.t : [0, 0, 0];
  const o = index >= 0 ? scene.groups[index].origin : [0, 0, 0];
  const T = [t[0] - (R[0] * o[0] + R[1] * o[1] + R[2] * o[2]), t[1] - (R[3] * o[0] + R[4] * o[1] + R[5] * o[2]), t[2] - (R[6] * o[0] + R[7] * o[1] + R[8] * o[2])];
  return { [`${prefix}X`]: [R[0], R[3], R[6]], [`${prefix}Y`]: [R[1], R[4], R[7]], [`${prefix}Z`]: [R[2], R[5], R[8]], [`${prefix}T`]: T };
}

const T3_MAT_KEYS = ["paper", "face", "top", "shade-0", "shade-1", "shade-2", "shade-3", "lit-top", "lit-shade", "hi", "mid", "lo", "faint", "dot", "lit"];
const T3_MAT_TUBE = ["body", "shine", "shade", "edge"];

const T3_MATERIALS = {
  gold: {
    light: ["#d3bb88", "#e4d2a8", "#f1e5c6", "#a1844f", "#bc9f69", "#d3bb88", "#e4d2a8", "#f8efd8", "#dfcb9c", "#4b3920", "#86693f", "#a98c5b", "#c1a676", "#4a381f", "#2a1d0c", "#cdb07a", "#f3e8cc", "#8e7044", "#4b3920"],
    dark: ["#3f3521", "#4b3f28", "#5a4c31", "#2b2416", "#352c1b", "#3f3521", "#4b3f28", "#6a5a3b", "#4f4329", "#e6d2a3", "#8a7550", "#5e4e33", "#463a26", "#e6d2a3", "#fff3d6", "#4a3e27", "#8c7853", "#251f13", "#e6d2a3"],
  },
  chrome: {
    light: ["#8e959c", "#c3c8cd", "#f7f8f9", "#a2a9b0", "#6f767d", "#454b52", "#cacfd4", "#ffffff", "#bcc2c8", "#202428", "#585e65", "#878d94", "#a9afb5", "#2a2e33", "#000000", "#8b9299", "#fbfcfd", "#3c4248", "#202428"],
    dark: ["#2a2e32", "#4a5056", "#6e757c", "#2e3236", "#24272b", "#1a1d20", "#4a5056", "#848b92", "#3a3f44", "#dfe3e7", "#7c838a", "#4d5359", "#363b40", "#e4e7ea", "#ffffff", "#34393e", "#9aa1a8", "#16191b", "#dfe3e7"],
  },
  steel: {
    light: ["#b8bdc1", "#cdd1d4", "#e5e8ea", "#8f959b", "#a4aaaf", "#b8bdc1", "#cdd1d4", "#f1f3f4", "#c6cacd", "#2b2f33", "#6b7177", "#959ba1", "#aeb3b8", "#33373b", "#000000", "#b3b8bd", "#eef0f2", "#7d8389", "#2b2f33"],
    dark: ["#2c3034", "#34393d", "#3e4348", "#1f2225", "#25292c", "#2c3034", "#34393d", "#4a5056", "#33383c", "#d0d4d8", "#727980", "#4a5056", "#33383d", "#d8dce0", "#ffffff", "#30353a", "#5c6369", "#1a1d20", "#d0d4d8"],
  },
  gunmetal: {
    light: ["#444a51", "#50575e", "#5f666e", "#2f3439", "#393f45", "#444a51", "#50575e", "#6d757d", "#4b5258", "#0f1113", "#1d2125", "#272b30", "#30353a", "#b0b5ba", "#000000", "#3d4349", "#6e757c", "#24282c", "#0f1113"],
    dark: ["#1d2125", "#22272b", "#2a2f34", "#15181b", "#191c20", "#1d2125", "#22272b", "#323840", "#20252a", "#a9b0b7", "#5c636a", "#3a4046", "#2a2f34", "#b8bec4", "#ffffff", "#1f2327", "#3a4046", "#111316", "#a9b0b7"],
  },
  rubber: {
    light: ["#2a2b2e", "#34363a", "#46484d", "#1b1c1e", "#222326", "#2a2b2e", "#34363a", "#55575c", "#2f3134", "#070708", "#111214", "#18191b", "#1f2023", "#6a6c70", "#000000", "#27282b", "#5b5d62", "#121314", "#050506"],
    dark: ["#121213", "#161618", "#1c1c1f", "#0c0c0d", "#0f0f10", "#121213", "#161618", "#232327", "#141416", "#8c8e92", "#4a4c50", "#2c2d30", "#1f2023", "#9a9ca0", "#ffffff", "#131315", "#2b2c30", "#0a0a0b", "#8c8e92"],
  },
  brass: {
    light: ["#bd9646", "#d2ad5b", "#e6c87e", "#8a6a2c", "#a47f38", "#bd9646", "#d2ad5b", "#f0d796", "#c8a252", "#3f2e0e", "#765a24", "#9a7a3c", "#b08c46", "#3a2a0c", "#1f1606", "#b8913f", "#efd795", "#7d5f26", "#3f2e0e"],
    dark: ["#3d3017", "#49391c", "#574523", "#2a210f", "#332813", "#3d3017", "#49391c", "#66522b", "#45371b", "#e9cf8a", "#8e7438", "#5e4c25", "#44371c", "#e9cf8a", "#fff0c8", "#433419", "#8a6f36", "#21190b", "#e9cf8a"],
  },
  copper: {
    light: ["#b3633e", "#c9784f", "#df9670", "#7c4128", "#9a5132", "#b3633e", "#c9784f", "#eaa985", "#be6d46", "#3a1a0c", "#6e3820", "#925037", "#a65d3f", "#3a1a0c", "#1c0b04", "#ad5f3b", "#f0b896", "#6e3720", "#3a1a0c"],
    dark: ["#3d1f13", "#492617", "#582e1c", "#2a160d", "#331a10", "#3d1f13", "#492617", "#673722", "#432215", "#eaa985", "#8f5236", "#5f3624", "#45271a", "#eaa985", "#ffe0cc", "#42221a", "#8a4c31", "#1f0f08", "#eaa985"],
  },
};

function t3MaterialCss() {
  const rules = [];
  const block = (values) => T3_MAT_KEYS.map((key, i) => `--anatomy-${key}:${values[i]}`).concat(T3_MAT_TUBE.map((key, i) => `--mat-${key}:${values[T3_MAT_KEYS.length + i]}`)).join(";");
  for (const [name, { light, dark }] of Object.entries(T3_MATERIALS)) {
    rules.push(`[data-mat="${name}"]{${block(dark)}}`);
    rules.push(`.iso[data-theme="light"] [data-mat="${name}"]{${block(light)}}`);
    rules.push(`.iso[data-theme="dark"] [data-mat="${name}"]{${block(dark)}}`);
  }
  rules.push("[data-mat] .tb-body{fill:var(--mat-body)}");
  rules.push("[data-mat] .tb-shine{fill:var(--mat-shine)}");
  rules.push("[data-mat] .tb-shade{fill:var(--mat-shade)}");
  rules.push("[data-mat] .tb-edge{stroke:var(--mat-edge)}");
  return rules.join("\n");
}

function t3Css() {
  const rules = [];
  const ramp = "--tn-0:var(--anatomy-shade-0);--tn-1:var(--anatomy-shade-1);--tn-2:var(--anatomy-shade-2);--tn-3:var(--anatomy-shade-3);--tn-4:var(--anatomy-top)";
  rules.push(`.iso-live{${ramp}}`);
  rules.push(`.iso-live [data-mat]{${ramp}}`);
  rules.push(".iso-live [data-lit]{--tn-0:var(--anatomy-lit-shade);--tn-1:var(--anatomy-lit-shade);--tn-2:var(--anatomy-lit-shade);--tn-3:var(--anatomy-lit-shade);--tn-4:var(--anatomy-lit-top)}");
  const colour = "color-mix(in srgb,var(--ta) calc(100% - var(--f,0) * 100%),var(--tb))";
  rules.push(`.iso-tn{stroke-width:.4;stroke-linejoin:round;--ta:var(--tn-0);--tb:var(--tn-1);fill:${colour};stroke:${colour}}`);
  for (let i = 1; i < 4; i++) rules.push(`.iso-tn[data-b="${i}"]{--ta:var(--tn-${i});--tb:var(--tn-${i + 1})}`);
  rules.push(t3MaterialCss());
  for (const hue of ["red", "green", "blue"]) rules.push(`.tb[data-hue="${hue}"]{--tb-ink:var(--anatomy-${hue})}`);
  rules.push(".tb[data-hue] .tb-body{fill:color-mix(in srgb,var(--tb-ink) 54%,var(--anatomy-shade-2))}");
  rules.push(".tb[data-hue] .tb-shine{fill:color-mix(in srgb,var(--tb-ink) 16%,var(--anatomy-top))}");
  rules.push(".tb[data-hue] .tb-shade{fill:color-mix(in srgb,var(--tb-ink) 30%,var(--anatomy-hi))}");
  rules.push(".tb[data-hue] .tb-edge{stroke:color-mix(in srgb,var(--tb-ink) 38%,var(--anatomy-hi))}");
  rules.push(".tb[data-hue][data-mat] .tb-body{fill:color-mix(in srgb,var(--tb-ink) 62%,var(--mat-body))}");
  rules.push(".tb[data-hue][data-mat] .tb-shine{fill:color-mix(in srgb,var(--tb-ink) 22%,var(--mat-shine))}");
  rules.push(".tb[data-hue][data-mat] .tb-shade{fill:color-mix(in srgb,var(--tb-ink) 46%,var(--mat-shade))}");
  rules.push(".tb[data-hue][data-mat] .tb-edge{stroke:color-mix(in srgb,var(--tb-ink) 30%,var(--mat-edge))}");
  rules.push(".iso-turn{position:relative}");
  rules.push(".iso-turn>.iso-layer{position:absolute;inset:0;width:100%;height:100%;pointer-events:none}");
  return rules.join("\n");
}

export const TURN = { materials: Object.keys(T3_MATERIALS), materialCss: t3MaterialCss, rotation: (axis, degrees) => Array.from(t3Rot(axis, degrees)), quat: (q) => Array.from(t3Quat(q)), compose: t3Compose, floorOf: t3FloorOf, anchor: t3Anchor, camera: t3Camera, poses: t3PosesMap, prepare: t3Prepare, emit: t3Emit, order: t3Order, mount: t3Mount, cover: t3Cover, poseUniforms: t3PoseUniforms, css: t3Css, frame: t3Frame, cams: t3Cams, gjk: t3Gjk, hintsOf: (ri, ci, rj, cj, world) => Object.assign([...ri.axes.map((axis) => [axis, ci]), ...rj.axes.map((axis) => [axis, cj]), [[0, 0, 1], world]], { centroids: [ri.centroid, rj.centroid] }), disjoint: t3Disjoint, levels: T3_LEVELS, q: T3_Q, exact: (on) => { t3State.exact = Boolean(on); } };
export const TURN_CSS = t3Css();
