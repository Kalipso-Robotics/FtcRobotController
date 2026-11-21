// Pure model for the robot editor (no DOM, no three.js): robot.json <-> editable model, plus the origin-click maths. See ../../robot.schema.md.
// The model always has arrays/nulls so the editor never null-checks; modelToRobot drops the empty ones so missing keys stay missing.

const clone = (v) => JSON.parse(JSON.stringify(v));

export const BEAM_NAMES = ['beamBreakAOuter', 'beamBreakAInner', 'beamBreakBInner', 'beamBreakBOuter'];
export const UNIT_SCALE = { mm: 1, cm: 10, in: 25.4 };

export function robotToModel(robot) {
  const r = clone(robot || {});
  return {
    ...r,   // unknown keys survive
    version: 1,
    meshes: r.meshes || [],
    intakeZones: r.intakeZones || [],
    transferPath: r.transferPath || [],
    turret: r.turret || null,
    shooter: r.shooter || null,
    beams: r.beams || [],
    noodles: r.noodles || [],
  };
}

export function modelToRobot(model, footprint) {
  const m = clone(model);
  const out = { version: 1 };
  const fp = footprint || m.footprint;
  if (fp) out.footprint = fp;
  for (const [k, v] of Object.entries(m)) {
    if (k === 'version' || k === 'footprint') continue;
    if (v === null || v === undefined || (Array.isArray(v) && v.length === 0)) continue;
    out[k] = v;
  }
  return out;
}

/** Rotation matrix applied to v: Euler XYZ in degrees (three.js order: Rx * Ry * Rz). */
export function rotate(v, deg) {
  const [ax, ay, az] = deg.map((d) => (d * Math.PI) / 180);
  let [x, y, z] = v;
  [x, y] = [x * Math.cos(az) - y * Math.sin(az), x * Math.sin(az) + y * Math.cos(az)];
  [x, z] = [x * Math.cos(ay) + z * Math.sin(ay), -x * Math.sin(ay) + z * Math.cos(ay)];
  [y, z] = [y * Math.cos(ax) - z * Math.sin(ax), y * Math.sin(ax) + z * Math.cos(ax)];
  return [x, y, z];
}

/** Offset that makes the model-space point p (in the file's own units) the robot origin: -(R * (scale * p)). */
export function originOffset(p, scale, rotationDeg) {
  return rotate(p.map((c) => c * scale), rotationDeg).map((c) => (c === 0 ? 0 : -c));
}
