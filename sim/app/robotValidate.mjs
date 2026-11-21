// Checks a robot.json (sim/robot.schema.md) before the server saves it.

/** Rejects a robot.json that would break the world or the 3D view. Returns an error text, or null. */
const BEAM_NAMES = ['beamBreakAOuter', 'beamBreakAInner', 'beamBreakBInner', 'beamBreakBOuter'];   // the hardware-map keys (SimHardware.java)

export function validateRobot(r) {
  if (!r || typeof r !== 'object' || Array.isArray(r)) return 'robot.json must be an object.';
  const vec = (v) => Array.isArray(v) && v.length === 3 && v.every(Number.isFinite);
  for (const m of r.meshes || []) {
    if (typeof m.file !== 'string' || !/^[\w.-]+\.stl$/i.test(m.file)) return `Bad mesh file name: ${m.file}`;
    if (!['body', 'turret', 'hood'].includes(m.part)) return `Bad mesh part: ${m.part}`;
    if (!vec(m.offset) || !vec(m.rotationDeg) || !(m.scale > 0)) return `Mesh ${m.file} needs offset [x,y,z], rotationDeg [x,y,z] and scale > 0.`;
    if (m.color !== undefined && !/^#[0-9a-f]{6}$/i.test(m.color)) return `Mesh ${m.file} color must be "#rrggbb".`;
  }
  for (const z of r.intakeZones || []) if (!['front', 'back'].includes(z.side) || !vec(z.min) || !vec(z.max)) return 'An intake zone needs side front|back, min and max.';
  for (const p of r.transferPath || []) if (!vec(p)) return 'transferPath points must be [x,y,z].';
  if (r.turret && !vec(r.turret.pivot)) return 'turret.pivot must be [x,y,z].';
  if (r.shooter) {
    if (!vec(r.shooter.exit) || !vec(r.shooter.hoodPivot)) return 'shooter.exit and shooter.hoodPivot must be [x,y,z].';
    if (!Number.isFinite(r.shooter.hoodMinDeg) || !Number.isFinite(r.shooter.hoodMaxDeg)) return 'shooter.hoodMinDeg and hoodMaxDeg must be numbers.';
  }
  for (const b of r.beams || []) {
    if (!BEAM_NAMES.includes(b.name)) return `Bad beam name "${b.name}": use one of ${BEAM_NAMES.join(', ')}.`;
    if (!vec(b.emitter) || !vec(b.receiver)) return 'A beam needs emitter and receiver [x,y,z].';
  }
  for (const n of r.noodles || []) if (!vec(n.center) || !vec(n.axis) || !(n.radius > 0) || !(n.length > 0) || !(n.blades >= 1)) return 'A noodle needs center, axis, radius > 0, length > 0 and blades >= 1.';
  return null;
}

/** Warnings (not errors) for mesh files that robot.json names but sim/robot/ does not have. `files` is the folder listing. */
export function missingMeshes(r, files) {
  return ((r && r.meshes) || []).filter((m) => !files.includes(m.file)).map((m) => `Mesh ${m.file} is in robot.json but not in sim/robot/ (upload it, or delete the mesh).`);
}
