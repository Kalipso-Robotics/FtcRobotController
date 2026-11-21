// Checks an R1 path file (sim/README.md "R1 paths") before the server saves it.

export const FIELD_MM = 3657.6;
const STEP_KEYS = ['drive', 'intake', 'shoot', 'wait'];

/** Returns an error text, or null if the path file is fine. */
export function validatePath(p) {
  if (!p || typeof p !== 'object' || Array.isArray(p)) return 'A path file must be an object.';
  if (typeof p.name !== 'string' || !/^[\w-]{1,40}$/.test(p.name)) return 'name must be 1-40 letters, digits, _ or -.';
  if (p.alliance !== undefined && !['red', 'blue'].includes(p.alliance)) return 'alliance must be red or blue.';
  const pose = (v) => Array.isArray(v) && v.length === 3 && v.every(Number.isFinite) && v[0] >= 0 && v[0] <= FIELD_MM && v[1] >= 0 && v[1] <= FIELD_MM;
  if (!pose(p.start)) return `start must be [x, y, deg] with x and y inside the field (0 to ${FIELD_MM} mm).`;
  if (!Array.isArray(p.steps) || p.steps.length === 0) return 'steps must be a non-empty list.';
  for (const [i, s] of p.steps.entries()) {
    const at = `Step ${i + 1}`;
    if (!s || typeof s !== 'object') return `${at} must be an object.`;
    const keys = Object.keys(s).filter((k) => STEP_KEYS.includes(k));
    if (keys.length !== 1) return `${at} needs exactly one of drive, intake, shoot, wait.`;
    const k = keys[0];
    if (k === 'drive' && !pose(s.drive)) return `${at}: drive must be [x, y, deg] inside the field.`;
    if (k === 'intake' && typeof s.intake !== 'boolean') return `${at}: intake must be true or false.`;
    if (k === 'shoot' && !(Number.isInteger(s.shoot) && s.shoot >= 1 && s.shoot <= 10)) return `${at}: shoot must be a whole number 1 to 10.`;
    if (k === 'wait' && !(Number.isFinite(s.wait) && s.wait > 0 && s.wait <= 30)) return `${at}: wait must be seconds, above 0 up to 30.`;
    if (s.speedMmS !== undefined && !(Number.isFinite(s.speedMmS) && s.speedMmS > 0)) return `${at}: speedMmS must be above 0.`;
  }
  return null;
}
