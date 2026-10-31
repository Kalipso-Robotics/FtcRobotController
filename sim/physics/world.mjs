// The robot, the field and the balls, in Rapier 3D. Pure physics: no input/output here (server.mjs does that).
//
// Units: Rapier works best in metres, so everything is metres inside and millimetres at the edges.
// Frame: the same as the robot code's Odometry. Field x, y; z is up; theta counter-clockwise about z.
// The robot frame has x forward and y left.
//
// THE ROBOT. The four wheel powers the code set drive four wheels that speed up toward (power / kV) with
// a lag, then the code's own wheel mix is run backwards to get the robot's velocity
//   FL = vx + vy + w,  BL = vx - vy + w,  FR = vx - vy - w,  BR = vx + vy - w
// That velocity is given to the Rapier body (it can only move in the floor plane and turn about z), and
// Rapier resolves walls, obstacles and balls. The pods (dead wheels) read the REAL motion of the body.
//
// THE BALLS (only when init.balls is given). Where the manual puts them (Figure 10-2): 4 pollen in each
// Flower, 4 in each Garden, 3 nectar in each upward Cell. Each Hive is a body on a pivot with two Cells,
// held in one of two stable states by a spring (bi-stable). Enough weight in the up Cell swings it over
// and the balls spill out. All of it is Rapier: nothing is scripted.

import RAPIER from '@dimforge/rapier3d-compat';

const FL = 0, FR = 1, BL = 2, BR = 3;
const STEP_NANOS = 1_000_000; // physics step: 1 ms
const MM = 0.001;
const GRAVITY = 9.81;
const CONTACT_FREQUENCY_HZ = 300;
const TOUCH_DISTANCE_M = 0.002;       // closer than 2 mm counts as touching
const ROBOT_HEIGHT_M = 0.457;         // UNMEASURED: 18 in tall, the start-size limit

// Who touches whom. A pair collides when each one's membership is in the other's filter.
const G_STATIC = 1, G_ROBOT = 2, G_BALL = 4, G_ROBOT_ONLY = 8, G_BALL_ONLY = 16, G_PARTNER = 32;
const groups = (membership, filter) => (membership << 16) | filter;
const GROUPS = {
  floor: groups(G_STATIC, 0xffff),
  robot: groups(G_ROBOT, G_STATIC | G_BALL | G_ROBOT_ONLY | G_PARTNER),
  ball: groups(G_BALL, G_STATIC | G_ROBOT | G_BALL | G_BALL_ONLY | G_PARTNER),
  partner: groups(G_PARTNER, G_ROBOT | G_BALL),  // R2: kinematic, follows its path file, solid to R1 and the balls
  robotOnly: groups(G_ROBOT_ONLY, G_ROBOT),     // Flower bases and Hive legs: the robot cannot pass, balls fly through
  ballOnly: groups(G_BALL_ONLY, G_BALL),        // Flower pipes and Hive Cell panels: balls cannot pass, the robot is not stopped
};

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const angleOf = (q) => 2 * Math.atan2(q.z, q.w);          // rotation about z
const aboutX = (q) => 2 * Math.atan2(q.x, q.w);           // rotation about x

/** Where the partner is at tSec on its path: [xMm, yMm, thetaRad]. Linear, theta the short way round, ends held. */
export function partnerPoseAt(poses, tSec) {
  if (tSec <= poses[0][0]) return poses[0].slice(1);
  const last = poses[poses.length - 1];
  if (tSec >= last[0]) return last.slice(1);
  let i = 1;
  while (poses[i][0] < tSec) i++;
  const [t0, x0, y0, th0] = poses[i - 1], [t1, x1, y1, th1] = poses[i];
  const f = (tSec - t0) / (t1 - t0);
  return [x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, th0 + wrap(th1 - th0) * f];
}

let rapierReady = false;
async function ensureRapier() {
  if (!rapierReady) {
    await RAPIER.init();
    rapierReady = true;
  }
}

/**
 * init = {
 *   pose: [xMm, yMm, thetaRad],
 *   robot: { frontMm, backMm, widthMm, massKg },        // centre of the robot (the odometry point) to its front / back
 *   field: { sizeMm, boxes: [{ name, cx, cy, hw, hh }] } // obstacles the ROBOT cannot pass, mm
 *   drive: { kV, kOmegaMmPerRad, responseSeconds, brakeResponseSeconds, trackWidthMm, backDistanceMm },
 *   balls: {                                             // optional: leave it out for a field with no balls
 *     flowers: [{ name, x, y, wallDirX, wallDirY }],     // centre of each Flower (mm) and which way its wall is
 *     gardens: [{ name, x0, y0, dx, dy }],               // first ball of each Garden line and the step to the next
 *     hives: [{ name, x, y, upSide, nectarType }],       // pivot position; upSide +1 = the +y Cell starts up
 *     params: { ... see BALL_DEFAULTS }
 *   }
 * }
 */
import ballDefaults from './ball-defaults.json' with { type: 'json' };
const BALL_DEFAULTS = ballDefaults; // sim/physics/ball-defaults.json: every number is explained there, and the 3D view reads the same file

export async function createWorld(init) {
  await ensureRapier();

  const world = new RAPIER.World({ x: 0, y: 0, z: -GRAVITY });
  // Rapier's default contacts are soft springs (30 Hz): a robot driving at 2.7 m/s sinks ~75 mm into a wall.
  // A real robot and wall do not give like that, so contacts are made stiff.
  world.integrationParameters.contact_natural_frequency = CONTACT_FREQUENCY_HZ;
  world.integrationParameters.numSolverIterations = 8;

  // Floor and walls.
  const size = init.field.sizeMm * MM;
  const slab = 1; // half thickness, metres
  const half = size / 2;
  world.createCollider(RAPIER.ColliderDesc.cuboid(half + slab, half + slab, 0.5).setTranslation(half, half, -0.5)
    .setFriction(0.6).setRestitution(0.3).setCollisionGroups(GROUPS.floor));
  const obstacles = []; // { name, collider } : what the robot can touch
  for (const [name, cx, cy, hw, hh] of [
    ['wall-y0', half, -slab, half + slab, slab],
    ['wall-yMax', half, size + slab, half + slab, slab],
    ['wall-x0', -slab, half, slab, half + slab],
    ['wall-xMax', size + slab, half, slab, half + slab],
  ]) {
    obstacles.push({ name, collider: world.createCollider(
      RAPIER.ColliderDesc.cuboid(hw, hh, 1.5).setTranslation(cx, cy, 1.5).setFriction(0.3).setRestitution(0.3).setCollisionGroups(GROUPS.floor)) });
  }
  // Flower bases and Hive legs: the robot cannot pass them. (Balls fly through; the Flower pipes and Hive Cells below are for balls.)
  for (const box of init.field.boxes) {
    obstacles.push({
      name: box.name,
      collider: world.createCollider(
        RAPIER.ColliderDesc.cuboid(box.hw * MM, box.hh * MM, 0.3).setTranslation(box.cx * MM, box.cy * MM, 0.3)
          .setFriction(0.3).setCollisionGroups(GROUPS.robotOnly)),
    });
  }

  // The robot. The body's origin is the odometry point, and the code's wheel mix describes the motion of
  // that point, so the mass sits at the origin. The footprint is offset from it.
  const [x0, y0, theta0] = init.pose;
  const r = init.robot;
  const lengthM = (r.frontMm + r.backMm) * MM;
  const widthM = r.widthMm * MM;
  const centreOffsetM = (r.frontMm - r.backMm) / 2 * MM;
  const inertia = r.massKg * (lengthM ** 2 + widthM ** 2) / 12 + r.massKg * centreOffsetM ** 2;
  const body = world.createRigidBody(
    RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(x0 * MM, y0 * MM, ROBOT_HEIGHT_M / 2)
      .setRotation({ x: 0, y: 0, z: Math.sin(theta0 / 2), w: Math.cos(theta0 / 2) })
      .enabledTranslations(true, true, false)
      .enabledRotations(false, false, true)
      .setLinearDamping(0)
      .setAngularDamping(0)
      .setCanSleep(false)
      .setCcdEnabled(true)
      .setAdditionalMassProperties(r.massKg, { x: 0, y: 0, z: 0 }, { x: inertia, y: inertia, z: inertia }, { x: 0, y: 0, z: 0, w: 1 }));
  const robotCollider = world.createCollider(
    RAPIER.ColliderDesc.cuboid(lengthM / 2, widthM / 2, ROBOT_HEIGHT_M / 2)
      .setTranslation(centreOffsetM, 0, 0)
      .setDensity(0)
      .setFriction(0.3)
      .setRestitution(0.1)
      .setCollisionGroups(GROUPS.robot),
    body);

  // The partner R2: a kinematic body that follows its path file. R1 and the balls bump into it; it never moves for them.
  const partner = init.partner || null;
  let partnerStartNanos = null;
  let partnerPose = partner ? partner.poses[0].slice(1) : null;
  let partnerBody = null;
  if (partner) {
    const pLen = (partner.front + partner.back) * MM, pCentre = (partner.front - partner.back) / 2 * MM;
    partnerBody = world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased()
      .setTranslation(partnerPose[0] * MM, partnerPose[1] * MM, ROBOT_HEIGHT_M / 2)
      .setRotation({ x: 0, y: 0, z: Math.sin(partnerPose[2] / 2), w: Math.cos(partnerPose[2] / 2) }));
    const partnerCollider = world.createCollider(
      RAPIER.ColliderDesc.cuboid(pLen / 2, partner.width * MM / 2, ROBOT_HEIGHT_M / 2)
        .setTranslation(pCentre, 0, 0).setFriction(0.3).setCollisionGroups(GROUPS.partner), partnerBody);
    obstacles.push({ name: 'partner', collider: partnerCollider });
  }
  function setPartnerTo(pose, next) {
    partnerPose = pose;
    const t = { x: pose[0] * MM, y: pose[1] * MM, z: ROBOT_HEIGHT_M / 2 };
    const q = { x: 0, y: 0, z: Math.sin(pose[2] / 2), w: Math.cos(pose[2] / 2) };
    if (next) { partnerBody.setNextKinematicTranslation(t); partnerBody.setNextKinematicRotation(q); }
    else { partnerBody.setTranslation(t, true); partnerBody.setRotation(q, true); }
  }
  function movePartner(toNanos) {
    if (!partner || partnerStartNanos === null) return;
    setPartnerTo(partnerPoseAt(partner.poses, (toNanos - partnerStartNanos) / 1e9 * partner.speed), true);
  }

  // ---- balls, Flowers, Gardens, Hives ----
  const P = { ...BALL_DEFAULTS, ...((init.balls && init.balls.params) || {}) };
  const balls = [];        // { id, type: 0 pollen / 1 red nectar / 2 blue nectar, radius, body }
  const hives = [];        // { name, body, joint, state, tilt, stiffness }
  const worldEvents = [];  // contacts and Hive tips since the last state() call

  /** Gives a ball a physical body at a place with a velocity (a ball that is held inside the robot has none). */
  function giveBody(ball, xM, yM, zM, vx = 0, vy = 0, vz = 0) {
    const mass = ball.type === 0 ? P.pollenMassKg : P.nectarMassKg;
    ball.body = world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic().setTranslation(xM, yM, zM).setLinvel(vx, vy, vz).setCcdEnabled(true)
        .setLinearDamping(0.02).setAngularDamping(P.ballRollingDamping));
    world.createCollider(
      RAPIER.ColliderDesc.ball(ball.radius).setDensity(mass / ((4 / 3) * Math.PI * ball.radius ** 3))
        .setFriction(P.ballFriction).setRestitution(P.ballBounce).setCollisionGroups(GROUPS.ball),
      ball.body);
    ball.held = false;
    ball.slot = -1;
  }

  function addBall(type, xM, yM, zM) {
    const radius = (type === 0 ? P.pollenRadiusMm : P.nectarRadiusMm) * MM;
    const ball = { id: balls.length, type, radius, body: null, held: false, slot: -1, last: null };
    balls.push(ball);
    giveBody(ball, xM, yM, zM);
    return ball;
  }

  function buildFlower(f) {
    // The Flower (Figures 9-12, 9-13): a lower ring on the floor with a 70.9 mm hole, and four pipes above a 90 mm open gap.
    const cx = f.x * MM, cy = f.y * MM;
    const ringInner = 0.03545, ringOuter = 0.062, ringHeight = 0.011;
    for (let i = 0; i < 12; i++) { // the ring is 12 short blocks around a circle
      const a = (i / 12) * 2 * Math.PI;
      const mid = (ringInner + ringOuter) / 2;
      world.createCollider(RAPIER.ColliderDesc.cuboid((ringOuter - ringInner) / 2, 0.015, ringHeight / 2)
        .setTranslation(cx + mid * Math.cos(a), cy + mid * Math.sin(a), ringHeight / 2)
        .setRotation({ x: 0, y: 0, z: Math.sin(a / 2), w: Math.cos(a / 2) })
        .setFriction(0.4).setCollisionGroups(GROUPS.ballOnly));
    }
    for (let i = 0; i < 4; i++) { // four pipes from the open gap (z = 101 mm) up to the top ring at 546 mm
      const a = Math.PI / 4 + (i * Math.PI) / 2;
      world.createCollider(RAPIER.ColliderDesc.cylinder((0.546 - 0.101) / 2, 0.008)
        .setRotation({ x: Math.sin(Math.PI / 4), y: 0, z: 0, w: Math.cos(Math.PI / 4) }) // cylinders stand along y by default
        .setTranslation(cx + 0.0575 * Math.cos(a), cy + 0.0575 * Math.sin(a), (0.546 + 0.101) / 2)
        .setFriction(0.3).setCollisionGroups(GROUPS.ballOnly));
    }
    for (let i = 0; i < 4; i++) addBall(0, cx, cy, P.pollenRadiusMm * MM + i * 2 * P.pollenRadiusMm * MM + 0.0005);
  }

  function buildHive(h) {
    const pivot = { x: h.x * MM, y: h.y * MM, z: P.hivePivotZMm * MM };
    const tilt = (P.hiveTiltDeg * Math.PI) / 180;
    const fixed = world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(pivot.x, pivot.y, pivot.z));
    const state = h.upSide; // +1: the +y Cell is up
    const hiveBody = world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic().setTranslation(pivot.x, pivot.y, pivot.z)
        .setRotation({ x: Math.sin((state * tilt) / 2), y: 0, z: 0, w: Math.cos((state * tilt) / 2) })
        .setCanSleep(false)
        .setAngularDamping(P.hiveDamping) // the dampers on the real Hive
        .setAdditionalMassProperties(3, { x: 0, y: 0, z: 0 }, { x: 0.3, y: 0.3, z: 0.3 }, { x: 0, y: 0, z: 0, w: 1 }));

    // Two Cells, one on each side of the pivot: open bins with the opening facing outward (along +-y).
    const t = 0.003;                                  // panel half thickness
    const hw = (P.hiveCellWidthMm * MM) / 2, hz = (P.hiveCellHeightMm * MM) / 2;
    const inner = P.hiveCellInnerMm * MM, depth = P.hiveCellDepthMm * MM;
    const zMid = P.hiveCellBottomMm * MM + hz;
    for (const side of [1, -1]) {
      const yMid = side * (inner + depth / 2);
      const panel = (hx, hy, hzz, px, py, pz) => world.createCollider(
        RAPIER.ColliderDesc.cuboid(hx, hy, hzz).setTranslation(px, py, pz).setDensity(0).setFriction(0.35).setRestitution(0.3)
          .setCollisionGroups(GROUPS.ballOnly), hiveBody);
      panel(hw, t, hz, 0, side * (inner + t), zMid);                       // back wall (the end nearest the pivot)
      panel(hw, depth / 2, t, 0, yMid, zMid - hz + t);                     // floor
      panel(hw, depth / 2, t, 0, yMid, zMid + hz - t);                     // roof
      panel(t, depth / 2, hz, -hw + t, yMid, zMid);                        // side walls
      panel(t, depth / 2, hz, hw - t, yMid, zMid);
    }
    const joint = world.createImpulseJoint(
      RAPIER.JointData.revolute({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }, { x: 1, y: 0, z: 0 }), fixed, hiveBody, true);
    joint.setLimits(-tilt, tilt); // the two stable states are hard stops against the frame
    // Bi-stable (over-center): a torque proportional to the tilt pushes the Hive toward the nearer stop. It is largest
    // at the stop, so weight in the up Cell has to beat it right there; once past the dead centre (angle 0) it helps.
    // It is set so hiveTipBalls pollen tip the Cell. Balls rest against the back wall, so that is the arm.
    const arm = (inner + P.pollenRadiusMm * MM) * Math.cos(tilt); // the horizontal lever arm at the stop
    const holdTorque = (P.hiveTipBalls - 0.5) * P.pollenMassKg * GRAVITY * arm;
    const hive = { name: h.name, body: hiveBody, joint, state, tilt, overCenter: holdTorque / tilt };
    hives.push(hive);

    // 3 nectar in the up Cell: against the back wall, in a line from the side nearest the Alliance Area (x low for the red Hive).
    const upY = state * (inner + 0.0457 + 0.004);
    const xs = h.nectarType === 1 ? -1 : 1;                           // red Hive: from the -x side; blue Hive: from the +x side
    const upAngle = state * tilt;
    for (let i = 0; i < 3; i++) {
      const lx = xs * (-hw + 0.0457 + 0.006 + i * 0.0914), ly = upY, lz = P.hiveCellBottomMm * MM + 0.0457 + 0.006;
      // rotate the local point (ly, lz) by the Hive's tilt about x, then move it to the pivot
      const wy = ly * Math.cos(upAngle) - lz * Math.sin(upAngle), wz = ly * Math.sin(upAngle) + lz * Math.cos(upAngle);
      addBall(h.nectarType, pivot.x + lx, pivot.y + wy, pivot.z + wz);
    }
  }

  if (init.balls) {
    for (const f of init.balls.flowers) buildFlower(f);
    for (const g of init.balls.gardens) {
      for (let i = 0; i < 4; i++) addBall(0, (g.x0 + g.dx * i) * MM, (g.y0 + g.dy * i) * MM, P.pollenRadiusMm * MM);
    }
    for (const h of init.balls.hives) buildHive(h);
  }

  // ---- the robot's intake: balls it holds sit in four slots along the ball path (A_OUTER, A_INNER, B_INNER, B_OUTER) ----
  const SLOT_X_M = [0.12, 0.04, -0.04, -0.12];     // robot frame, along the robot's length. UNMEASURED
  const SLOT_Z_M = 0.15;
  const slots = [null, null, null, null];           // the ball in each slot
  const heldCount = () => slots.filter(Boolean).length;
  const intakeMouthM = { front: P.intakeFrontReachMm, back: P.intakeBackReachMm };
  const act = { intake: 0, stopper: 0.47, rps: 0, hood: 0, turret: 0 };   // what the code is doing with the ball hardware now

  function putInSlot(ball, fromFront) {
    // balls stack from the outside in: a ball entering at the front goes to the first free slot from A_OUTER, at the back from B_OUTER
    const order = fromFront ? [0, 1, 2, 3] : [3, 2, 1, 0];
    const free = order.find((i) => !slots[i]);
    if (free === undefined) return false;
    slots[free] = ball;
    ball.held = true;
    ball.slot = free;
    if (ball.body) { world.removeRigidBody(ball.body); ball.body = null; }
    return true;
  }

  function runIntake() {
    if (act.intake <= 0.2 || heldCount() >= 4) return;
    const p = body.translation();
    const rot = angleOf(body.rotation());
    const c = Math.cos(rot), s = Math.sin(rot);
    const halfW = widthM / 2 - 0.01;
    for (const ball of balls) {
      if (ball.held || !ball.body) continue;
      const q = ball.body.translation();
      if (q.z > ball.radius + 0.06) continue;              // only balls on the floor
      const dx = q.x - p.x, dy = q.y - p.y;
      const lx = dx * c + dy * s, ly = -dx * s + dy * c;
      if (Math.abs(ly) > halfW) continue;
      const front = lx > r.frontMm * MM - 0.06 && lx < r.frontMm * MM + intakeMouthM.front * MM;
      const back = lx < -r.backMm * MM + 0.06 && lx > -r.backMm * MM - intakeMouthM.back * MM;
      if ((front || back) && putInSlot(ball, front)) {
        worldEvents.push({ name: `*picked up a ${ball.type === 0 ? 'pollen' : ball.type === 1 ? 'red nectar' : 'blue nectar'} ball (${heldCount()} held)`, nanos: simNanos });
        if (heldCount() >= 4) return;
      }
    }
  }

  // ---- shots: a ball leaves when the stopper is open, the intake is pushing and the flywheel is up to speed ----
  let lastShotNanos = -1e18;

  /** Each Hive's up-Cell opening centre in the world (metres), for the auto-aimed shot and for "did it go in". */
  function upCellOpenings() {
    const out = [];
    const reach = (P.hiveCellInnerMm + P.hiveCellDepthMm) * MM;
    const s = (P.hiveCellBottomMm + P.hiveCellHeightMm / 2) * MM;
    for (const hive of hives) {
      const phi = aboutX(hive.body.rotation());
      const origin = hive.body.translation();
      for (const side of [1, -1]) {
        const y = side * reach * Math.cos(phi) - s * Math.sin(phi);
        const z = side * reach * Math.sin(phi) + s * Math.cos(phi);
        if (z > 0.15) out.push({ hive: hive.name, x: origin.x, y: origin.y + y, z: origin.z + z });
      }
    }
    return out;
  }

  /** The launch velocity (m/s) for a shot from (x, y, z) toward a point, at a fixed angle. */
  function solveLaunch(from, to, angleRad) {
    const dx = to.x - from.x, dy = to.y - from.y;
    const dist = Math.hypot(dx, dy), dz = to.z - from.z;
    let angle = angleRad;
    for (let i = 0; i < 20; i++) {
      const denom = 2 * Math.cos(angle) ** 2 * (dist * Math.tan(angle) - dz);
      if (denom > 1e-6) {
        const v = Math.sqrt((GRAVITY * dist * dist) / denom);
        return { v, angle, azimuth: Math.atan2(dy, dx) };
      }
      angle += 0.05; // the target is too high for this angle: lob it steeper
    }
    return { v: 8, angle: 1.2, azimuth: Math.atan2(dy, dx) };
  }

  function runShooter() {
    if (!(act.stopper < P.stopperOpenBelow && act.intake > 0.2 && act.rps > P.minFlywheelRps)) return;
    if (simNanos - lastShotNanos < P.shotIntervalMs * 1e6) return;
    const slot = slots.findIndex(Boolean);
    if (slot < 0) return;
    const ball = slots[slot];
    const p = body.translation();
    const heading = angleOf(body.rotation());
    const exit = { x: p.x, y: p.y, z: P.exitHeightMm * MM };
    let v, angle, azimuth;
    if (P.shotMode === 1) {
      const openings = upCellOpenings();
      if (!openings.length) return;
      openings.sort((a, b) => Math.hypot(a.x - exit.x, a.y - exit.y) - Math.hypot(b.x - exit.x, b.y - exit.y));
      ({ v, angle, azimuth } = solveLaunch(exit, openings[0], (P.autoAimAngleDeg * Math.PI) / 180));
    } else {
      const k = Math.min(1, Math.max(0, (act.hood - P.hoodMin) / (P.hoodMax - P.hoodMin)));
      angle = ((P.hoodMinAngleDeg + k * (P.hoodMaxAngleDeg - P.hoodMinAngleDeg)) * Math.PI) / 180;
      v = P.shotSpeedPerRps * act.rps;
      azimuth = heading + act.turret;
    }
    slots[slot] = null;
    exit.x += (P.exitForwardMm * MM) * Math.cos(azimuth);
    exit.y += (P.exitForwardMm * MM) * Math.sin(azimuth);
    giveBody(ball, exit.x, exit.y, exit.z, v * Math.cos(angle) * Math.cos(azimuth), v * Math.cos(angle) * Math.sin(azimuth), v * Math.sin(angle));
    ball.shot = true;
    lastShotNanos = simNanos;
    worldEvents.push({ name: `*shot a ${ball.type === 0 ? 'pollen' : 'nectar'} ball (${act.rps.toFixed(1)} RPS, ${(angle * 180 / Math.PI).toFixed(0)}°, ${v.toFixed(1)} m/s)`, nanos: simNanos });
  }

  /** A shot ball that is now inside a Cell: say so once. */
  function checkShotsInCells() {
    for (const ball of balls) {
      if (!ball.shot || ball.inCell || !ball.body) continue;
      const q = ball.body.translation();
      if (q.z < 0.5) continue;
      for (const hive of hives) {
        const o = hive.body.translation();
        const phi = aboutX(hive.body.rotation());
        const dy = q.y - o.y, dz = q.z - o.z;
        const ly = dy * Math.cos(phi) + dz * Math.sin(phi), lz = -dy * Math.sin(phi) + dz * Math.cos(phi);   // into the Hive's own frame
        const reach0 = P.hiveCellInnerMm * MM, reach1 = (P.hiveCellInnerMm + P.hiveCellDepthMm) * MM;
        const inside = Math.abs(q.x - o.x) < (P.hiveCellWidthMm * MM) / 2 && Math.abs(ly) > reach0 && Math.abs(ly) < reach1
          && lz > P.hiveCellBottomMm * MM && lz < (P.hiveCellBottomMm + P.hiveCellHeightMm) * MM;
        if (inside) {
          ball.inCell = true;
          worldEvents.push({ name: `*a ${ball.type === 0 ? 'pollen' : 'nectar'} ball went into the ${hive.name.replace('-hive', '')} Hive's ${Math.sign(ly) * Math.sign(hive.state) > 0 ? 'up' : 'down'} Cell`, nanos: simNanos });
        }
      }
    }
  }

  /** Where a held ball is: inside the robot, at its slot. */
  function heldPosition(ball) {
    const p = body.translation();
    const rot = angleOf(body.rotation());
    const x = SLOT_X_M[ball.slot];
    return { x: p.x + x * Math.cos(rot), y: p.y + x * Math.sin(rot), z: SLOT_Z_M };
  }

  // 4 pollen preloaded in the robot (manual 10.3.4), if asked for
  for (let i = 0; i < (init.balls?.preload ?? 0); i++) {
    const ball = addBall(0, 0, 0, 5);
    putInSlot(ball, i % 2 === 0);
  }

  // ---- stepping ----
  const d = init.drive;
  const wheelMmPerS = [0, 0, 0, 0];
  let simNanos = 0;
  let theta = theta0;          // unwrapped
  let lastRotation = angleOf(body.rotation());
  let leftPodMm = 0, rightPodMm = 0, backPodMm = 0, fLeftWheelMm = 0;
  const touching = new Set();
  let touchEvents = [];

  /** Before each physics step: the over-center torque on each Hive. */
  function pushHives() {
    for (const hive of hives) {
      hive.body.resetTorques(true);
      hive.body.addTorque({ x: hive.overCenter * aboutX(hive.body.rotation()), y: 0, z: 0 }, true);
    }
  }

  /** After each physics step: a Hive that swung through the dead centre is now in the other stable state. */
  function stepHives() {
    for (const hive of hives) {
      if (hive.state * aboutX(hive.body.rotation()) < 0) {
        hive.state = -hive.state;
        worldEvents.push({ name: `${hive.name} tipped`, nanos: simNanos });
      }
    }
  }

  function step(dt, power, brake) {
    const maxWheelMmPerS = 1 / d.kV;
    for (let i = 0; i < 4; i++) {
      const p = Math.max(-1, Math.min(1, power[i]));
      const response = (p === 0 && brake[i]) ? d.brakeResponseSeconds : d.responseSeconds;
      wheelMmPerS[i] += (p * maxWheelMmPerS - wheelMmPerS[i]) * (1 - Math.exp(-dt / response));
    }
    fLeftWheelMm += wheelMmPerS[FL] * dt;

    const vx = (wheelMmPerS[FL] + wheelMmPerS[BL] + wheelMmPerS[FR] + wheelMmPerS[BR]) / 4;
    const vy = (wheelMmPerS[FL] - wheelMmPerS[BL] - wheelMmPerS[FR] + wheelMmPerS[BR]) / 4;
    const w = ((wheelMmPerS[FL] + wheelMmPerS[BL] - wheelMmPerS[FR] - wheelMmPerS[BR]) / 4) / d.kOmegaMmPerRad;

    const rot = angleOf(body.rotation());
    const c = Math.cos(rot), s = Math.sin(rot);
    body.setLinvel({ x: (vx * c - vy * s) * MM, y: (vy * c + vx * s) * MM, z: 0 }, true);
    body.setAngvel({ x: 0, y: 0, z: w }, true);

    const before = body.translation();
    world.timestep = dt;
    pushHives();
    movePartner(simNanos + Math.round(dt * 1e9));
    world.step();
    const after = body.translation();

    const newRot = angleOf(body.rotation());
    const dTheta = wrap(newRot - lastRotation);
    lastRotation = newRot;
    theta += dTheta;
    const mid = newRot - dTheta / 2;
    const dxw = (after.x - before.x) / MM, dyw = (after.y - before.y) / MM;
    const dx = dxw * Math.cos(mid) + dyw * Math.sin(mid);
    const dy = -dxw * Math.sin(mid) + dyw * Math.cos(mid);
    leftPodMm += dx + d.trackWidthMm / 2 * dTheta;
    rightPodMm += dx - d.trackWidthMm / 2 * dTheta;
    backPodMm += dy + d.backDistanceMm * dTheta;

    for (const { name, collider } of obstacles) {
      let touchingNow = false;
      world.contactPair(robotCollider, collider, (manifold) => {
        for (let i = 0; i < manifold.numContacts(); i++) {
          if (manifold.contactDist(i) <= TOUCH_DISTANCE_M) touchingNow = true;
        }
      });
      if (touchingNow && !touching.has(name)) {
        touching.add(name);
        touchEvents.push({ name, nanos: simNanos + Math.round(dt * 1e9) });
      } else if (!touchingNow) {
        touching.delete(name);
      }
    }
    stepHives();
    runIntake();
    runShooter();
    checkShotsInCells();
  }

  // Let the balls settle into the Flowers, Cells and Garden before the match starts. Nothing here is on the clock.
  if (init.balls) {
    world.timestep = 0.004;
    for (let i = 0; i < Math.round(P.settleSeconds / 0.004); i++) { pushHives(); world.step(); stepHives(); }
    worldEvents.length = 0;
    for (const b of balls) { if (b.body) { b.body.setLinvel({ x: 0, y: 0, z: 0 }, true); b.body.setAngvel({ x: 0, y: 0, z: 0 }, true); } }
    // the robot may have been nudged by the settle (a start pose inside a wall gets pushed out): put it back exactly
    body.setTranslation({ x: x0 * MM, y: y0 * MM, z: ROBOT_HEIGHT_M / 2 }, true);
    body.setRotation({ x: 0, y: 0, z: Math.sin(theta0 / 2), w: Math.cos(theta0 / 2) }, true);
    body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    lastRotation = angleOf(body.rotation());
    if (partner) setPartnerTo(partner.poses[0].slice(1), false);
  }

  return {
    /** Moves the world to toNanos with the given wheel powers (constant over the interval). Returns the state. */
    advanceTo(toNanos, power, brake, actuators) {
      if (actuators) Object.assign(act, actuators);
      while (simNanos < toNanos) {
        const stepNanos = Math.min(STEP_NANOS, toNanos - simNanos);
        step(stepNanos / 1e9, power, brake);
        simNanos += stepNanos;
      }
      return this.state();
    },
    /** The partner's clock starts here (the match START). Before this it holds the first pose of its path. */
    partnerStart(startNanos) { partnerStartNanos = startNanos; },
    /** The partner's pose now: [xMm, yMm, thetaRad], or null when there is none. */
    partnerPose() { return partnerPose; },
    /** The state now, plus the contacts and Hive tips that began since the last call. */
    state() {
      const p = body.translation();
      const events = touchEvents.concat(worldEvents);
      touchEvents = [];
      worldEvents.length = 0;
      return { x: p.x / MM, y: p.y / MM, theta, leftPodMm, rightPodMm, backPodMm, fLeftWheelMm, events, held: heldCount(), slots: slots.map((b) => (b ? 1 : 0)).join(''), ...(partner ? { partner: partnerPose } : {}) };
    },
    /** What each ball is: [id, type, radiusMm]. Sent once. */
    ballInfo() {
      return balls.map((b) => [b.id, b.type, b.radius / MM]);
    },
    /** Balls that moved since the last call (and every ball the first time): [id, xMm, yMm, zMm]. */
    ballsMoved() {
      const moved = [];
      for (const b of balls) {
        const p = b.held ? heldPosition(b) : b.body.translation();
        const now = [Math.round(p.x / MM), Math.round(p.y / MM), Math.round(p.z / MM)];
        if (!b.last || now[0] !== b.last[0] || now[1] !== b.last[1] || now[2] !== b.last[2]) {
          b.last = now;
          moved.push([b.id, ...now]);
        }
      }
      return moved;
    },
    /** Every ball right now: [id, type, xMm, yMm, zMm, held (1 or 0)]. For the sim's fake camera. */
    ballPositions() {
      return balls.map((b) => {
        const p = b.held ? heldPosition(b) : b.body.translation();
        return [b.id, b.type, Math.round(p.x / MM), Math.round(p.y / MM), Math.round(p.z / MM), b.held ? 1 : 0];
      });
    },
    /** Each Hive's tilt about its pivot, in radians: positive lifts the +y Cell. */
    hiveAngles() {
      return hives.map((h) => aboutX(h.body.rotation()));
    },
    /** For tests: put one more ball in the world. */
    debugAddBall(type, xMm, yMm, zMm) {
      return addBall(type, xMm * MM, yMm * MM, zMm * MM).id;
    },
    debugBall(id) {
      const p = balls[id].held ? heldPosition(balls[id]) : balls[id].body.translation();
      return { x: p.x / MM, y: p.y / MM, z: p.z / MM };
    },
    debugStep(seconds, power = [0, 0, 0, 0], actuators) {
      return this.advanceTo(simNanos + Math.round(seconds * 1e9), power, [true, true, true, true], actuators);
    },
    free() {
      world.free();
    },
  };
}
