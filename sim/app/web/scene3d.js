// The 3D view: the same run as the top view, with height. Plain Three.js meshes built from the recorded run:
// floor, glass walls, tape, the Flowers (green pipes, yellow rings), the Hive frame and its two Hives with pentagon
// Cells that pivot, balls as flat-shaded spheres, and the robot as a box. The look follows the design mockup
// ("Kalipso Auto Sim UI"); positions and sizes still come from the run. Units are mm. Scene axes: x = field x,
// y = -field y (so it matches the top view), z = up. The camera orbits with the mouse.
//
// Mockup inches (fx, fy; origin at the field centre, +y toward the far wall) map to the scene as
// x = (fx + 72) * IN, y = (fy - 72) * IN. Checked against the run's flower-A..D obstacles.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const IN = 25.4;
const ROBOT_HEIGHT_MM = 457;         // UNMEASURED: the start-size limit, 18 in

// Real-world colours of the field, as in the mockup. The theme tokens in `colors` only style the robot overlay.
const COL = {
  ground: 0x15171b, floor: 0x3a3d43, seam: 0x2a2d32, wall: 0x9aa3ad, wallEdge: 0x5d636c,
  red: 0xd23c3c, redDeep: 0x7a2222, redArea: 0x2a1517, blue: 0x3a63d6, blueDeep: 0x1f3473, blueArea: 0x141b2e,
  pollen: 0xf2c94c, frame: 0x8b929c, green: 0x3fbf6a, ring: 0xe0b53a, black: 0x16181b,
};

export function build3D(run, colors, canvas) {
  const scene = new THREE.Scene();
  const S = run.field.size;
  const params = run.balls.params;

  scene.add(new THREE.HemisphereLight(0xffffff, 0x30343a, 1.5));
  const sun = new THREE.DirectionalLight(0xffffff, 1.2);
  sun.position.set(-1500, -4200, 5200);
  scene.add(sun);

  const lambert = (color, extra = {}) => new THREE.MeshLambertMaterial({ color, ...extra });
  const glass = (color, opacity) => lambert(color, { transparent: true, opacity, depthWrite: false, side: THREE.DoubleSide });
  const material = (color, opacity = 1) => new THREE.MeshStandardMaterial({ color, transparent: opacity < 1, opacity, roughness: 0.75, metalness: 0.05, depthWrite: opacity >= 1 });
  const at = (mesh, x, y, z) => { mesh.position.set(x, y, z); return mesh; };
  const box = (w, d, h, mat) => new THREE.Mesh(new THREE.BoxGeometry(w, d, h), mat);
  const F = (fx, fy, z = 0) => new THREE.Vector3((fx + 72) * IN, (fy - 72) * IN, z * IN);   // mockup inches -> scene

  // a square beam between two points (the mockup's `member`)
  const member = (a, b, t, mat) => {
    const m = new THREE.Mesh(new THREE.BoxGeometry(t, t, a.distanceTo(b)), mat);
    m.position.copy(a).add(b).multiplyScalar(0.5);
    m.up.set(...(Math.abs(b.z - a.z) > 0.99 * a.distanceTo(b) ? [1, 0, 0] : [0, 0, 1]));   // lookAt breaks when parallel to `up`
    m.lookAt(b);
    return m;
  };

  // ground, floor and seams
  const ground = at(new THREE.Mesh(new THREE.PlaneGeometry(900 * IN, 900 * IN), lambert(COL.ground)), S / 2, -S / 2, -30);
  scene.add(ground);
  for (const [sign, color] of [[-1, COL.redArea], [1, COL.blueArea]]) {   // the alliance areas beside the field
    scene.add(at(new THREE.Mesh(new THREE.PlaneGeometry(54 * IN, 97 * IN), lambert(color)), S / 2 + sign * 99 * IN, -S / 2, -28));
  }
  scene.add(at(box(S, S, IN, lambert(COL.floor)), S / 2, -S / 2, -IN / 2));
  const seams = [];
  for (let k = 1; k <= 5; k++) {
    seams.push(new THREE.Vector3(k * 24 * IN, 0, 1), new THREE.Vector3(k * 24 * IN, -S, 1), new THREE.Vector3(0, -k * 24 * IN, 1), new THREE.Vector3(S, -k * 24 * IN, 1));
  }
  scene.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(seams), new THREE.LineBasicMaterial({ color: COL.seam })));

  // glass walls with outlined edges
  const wallMat = glass(COL.wall, 0.1), edgeMat = new THREE.LineBasicMaterial({ color: COL.wallEdge });
  for (const [cx, cy, w, d] of [[S / 2, IN / 2, S + 2 * IN, IN], [S / 2, -S - IN / 2, S + 2 * IN, IN], [-IN / 2, -S / 2, IN, S + 2 * IN], [S + IN / 2, -S / 2, IN, S + 2 * IN]]) {
    const geo = new THREE.BoxGeometry(w, d, 12 * IN);
    scene.add(at(new THREE.Mesh(geo, wallMat), cx, cy, 6 * IN), at(new THREE.LineSegments(new THREE.EdgesGeometry(geo), edgeMat), cx, cy, 6 * IN));
  }

  // loading-zone and garden tape (mockup rectangles [x0, y0, x1, y1] in inches; blue mirrors red)
  const tape = ([x0, y0, x1, y1], color) => {
    const c = F((x0 + x1) / 2, (y0 + y1) / 2);
    scene.add(at(box((x1 - x0) * IN, (y1 - y0) * IN, 3, lambert(color)), c.x, c.y, 2));
  };
  for (const r of [[-61, 24, -60, 47], [-72, 46, -60, 47], [-72, 24, -60, 25]]) {
    tape(r, COL.red);
    tape([-r[2], -r[3], -r[0], -r[1]], COL.blue);
  }
  tape([-72, -72, -49, -70], COL.red);
  tape([49, 70, 72, 72], COL.blue);

  // Flowers: the obstacle list has each Flower's base box; its centre is the Flower's centre. The wall is the nearest edge.
  const ringMat = lambert(COL.ring), pipeMat = lambert(COL.green), blackMat = lambert(COL.black);
  for (const o of run.field.obstacles.filter((o) => o.name.startsWith('flower-'))) {
    const cx = (o.minX + o.maxX) / 2, cy = -((o.minY + o.maxY) / 2);       // scene coordinates
    const gaps = [[cx, -1, 0], [S - cx, 1, 0], [-cy, 0, 1], [S + cy, 0, -1]].sort((a, b) => a[0] - b[0]);
    const [gap, nx, ny] = [gaps[0][0], -gaps[0][1], -gaps[0][2]];          // normal points from the wall into the field
    const wall = new THREE.Vector3(cx - nx * gap, cy - ny * gap, 0);
    for (const [r, tube, z] of [[2.6, 0.45, 0.45], [2.6, 0.45, 4.6], [2.3, 0.55, 21.5]]) {
      scene.add(at(new THREE.Mesh(new THREE.TorusGeometry(r * IN, tube * IN, 6, 18), ringMat), cx, cy, z * IN));
    }
    for (let i = 0; i < 4; i++) {
      const a = Math.PI / 4 + (i * Math.PI) / 2;
      const pipe = new THREE.Mesh(new THREE.CylinderGeometry(0.4 * IN, 0.4 * IN, 16.9 * IN, 8), pipeMat);
      pipe.rotation.x = Math.PI / 2;
      scene.add(at(pipe, cx + Math.cos(a) * 2.5 * IN, cy + Math.sin(a) * 2.5 * IN, 13.05 * IN));
    }
    const stop = at(box(6 * IN, 1.25 * IN, 0.6 * IN, blackMat), cx - nx * 2.6 * IN, cy - ny * 2.6 * IN, 22.6 * IN);
    stop.rotation.z = Math.atan2(ny, nx) + Math.PI / 2;                    // its long side lies along the wall
    scene.add(stop, at(box(IN, IN, 4.6 * IN, blackMat), cx - nx * 2.9 * IN, cy - ny * 2.9 * IN, 2.3 * IN));
    for (const z of [4.6, 21.5]) {
      scene.add(member(new THREE.Vector3(wall.x, wall.y, z * IN), new THREE.Vector3(cx - nx * 2.3 * IN, cy - ny * 2.3 * IN, z * IN), 0.9 * IN, blackMat));
    }
  }

  // Hive frame: two A-frames (legs up to the pivot height, base beam, brace) and the crossbar between them
  const hive = run.field.hive;
  const hiveCentreX = (hive.minX + hive.maxX) / 2, hiveCentreY = (hive.minY + hive.maxY) / 2;
  const frameMat = lambert(COL.frame), pivotZ = params.hivePivotZMm;
  const P = (dx, dy, z) => new THREE.Vector3(hiveCentreX + dx * IN, -hiveCentreY + dy * IN, z);
  for (const sx of [-24.7, 24.7]) {
    scene.add(member(P(sx, -19.5, 0.8 * IN), P(sx, 0, pivotZ), 1.6 * IN, frameMat), member(P(sx, 19.5, 0.8 * IN), P(sx, 0, pivotZ), 1.6 * IN, frameMat),
      member(P(sx, -19.5, 0.8 * IN), P(sx, 19.5, 0.8 * IN), 1.6 * IN, frameMat), member(P(sx, -10.6, 20 * IN), P(sx, 10.6, 20 * IN), 1.2 * IN, frameMat));
  }
  scene.add(member(P(-24.7, 0, pivotZ), P(24.7, 0, pivotZ), 2 * IN, frameMat));

  // Hives: each is a group that pivots about the x axis. A Cell is a pentagon prism that opens outward along y.
  const hives = [];
  const hiveOffset = 25.5 * IN / 2;
  const W = params.hiveCellWidthMm, H = params.hiveCellHeightMm, r0 = params.hiveCellInnerMm, r1 = r0 + params.hiveCellDepthMm, z0 = params.hiveCellBottomMm;
  const pent = [[-W / 2, 0], [W / 2, 0], [W / 2, 0.544 * H], [0, H], [-W / 2, 0.544 * H]].map(([u, w]) => [u, z0 + w]);
  const tagMat = new THREE.MeshBasicMaterial({ color: 0xe8eaed }), tagDark = new THREE.MeshBasicMaterial({ color: 0x15171b });
  [[hiveCentreX - hiveOffset, COL.red, COL.redDeep], [hiveCentreX + hiveOffset, COL.blue, COL.blueDeep]].forEach(([x, color, deep]) => {
    const group = new THREE.Group();
    group.position.set(x, -hiveCentreY, pivotZ);
    const cellMat = lambert(color), thinMat = lambert(deep), panelMat = glass(color, 0.3);
    for (const sy of [1, -1]) {                         // the two Cells, one each side of the pivot (sy is the local y direction)
      const outer = pent.map(([u, w]) => new THREE.Vector3(u, sy * r1, w));
      const back = pent.map(([u, w]) => new THREE.Vector3(u, sy * r0, w));
      const pos = [];
      const tri = (a, b, c) => pos.push(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
      for (let i = 0; i < 5; i++) { const j = (i + 1) % 5; tri(outer[i], outer[j], back[j]); tri(outer[i], back[j], back[i]); }
      for (let i = 1; i < 4; i++) tri(back[0], back[i], back[i + 1]);
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
      geo.computeVertexNormals();
      group.add(new THREE.Mesh(geo, panelMat));
      for (let i = 0; i < 5; i++) {
        const j = (i + 1) % 5;
        group.add(member(outer[i], outer[j], 1.2 * IN, cellMat), member(back[i], back[j], 0.8 * IN, thinMat), member(outer[i], back[i], 0.8 * IN, thinMat));
      }
      for (let t = -1.5; t <= 1.5; t += 1) {            // AprilTag cluster under the Cell
        const tagY = sy * (r0 + r1) / 2;
        group.add(at(box(2.6 * IN, 2.6 * IN, 3, tagMat), t * 3.4 * IN, tagY, z0 - 3), at(box(1.6 * IN, 1.6 * IN, 4, tagDark), t * 3.4 * IN, tagY, z0 - 3));
      }
    }
    group.add(member(new THREE.Vector3(0, -21.5 * IN, -2.4 * IN), new THREE.Vector3(0, 21.5 * IN, -2.4 * IN), 1.8 * IN, lambert(0xa6adb6)));   // beam
    const hub = new THREE.Mesh(new THREE.CylinderGeometry(1.7 * IN, 1.7 * IN, 5 * IN, 12), lambert(0x6b727c));
    hub.rotation.z = Math.PI / 2;
    group.add(hub);
    for (const d of [-1, 1]) group.add(at(box(1.2 * IN, 1.2 * IN, 1.2 * IN, tagDark), 0, 4.5 * IN * d, -3.6 * IN));   // dampers
    scene.add(group);
    hives.push(group);
  });

  // balls: flat-shaded, pollen yellow and nectar in the alliance colours
  const ballColor = (type) => (type === 0 ? COL.pollen : type === 1 ? COL.red : COL.blue);
  const ballMeshes = run.balls.info.map((b) => {
    const mesh = new THREE.Mesh(new THREE.IcosahedronGeometry(b.radius, 1), new THREE.MeshPhongMaterial({ color: ballColor(b.type), flatShading: true, shininess: 8 }));
    scene.add(mesh);
    return mesh;
  });


  // robot (a box) and the odometry ghost (its outline)
  const r = run.field.robot;
  const boxGeo = new THREE.BoxGeometry(r.front + r.back, r.width, ROBOT_HEIGHT_MM);
  const robotMesh = new THREE.Mesh(boxGeo, material(colors.accent, 0.62));
  const robot = new THREE.Group();
  robotMesh.position.set((r.front - r.back) / 2, 0, ROBOT_HEIGHT_MM / 2);
  const nose = new THREE.Mesh(new THREE.ConeGeometry(40, 90, 4), material(colors.accent, 1));
  nose.rotation.z = -Math.PI / 2; nose.position.set(r.front + 30, 0, ROBOT_HEIGHT_MM + 20);
  robot.add(robotMesh, nose, new THREE.LineSegments(new THREE.EdgesGeometry(boxGeo), new THREE.LineBasicMaterial({ color: colors.accent })));
  robot.children[2].position.copy(robotMesh.position);
  scene.add(robot);
  // the partner R2: a solid box in a darker shade of the alliance colour
  let partnerGroup = null;
  if (run.partner) {
    const pr = run.partner.robot;
    const pColor = new THREE.Color(run.alliance === 'blue' ? colors.blue : colors.red).multiplyScalar(0.55);
    const pGeo = new THREE.BoxGeometry(pr.front + pr.back, pr.width, ROBOT_HEIGHT_MM);
    const pMesh = new THREE.Mesh(pGeo, material(pColor, 0.85));
    const pEdges = new THREE.LineSegments(new THREE.EdgesGeometry(pGeo), new THREE.LineBasicMaterial({ color: run.alliance === 'blue' ? colors.blue : colors.red }));
    pMesh.position.set((pr.front - pr.back) / 2, 0, ROBOT_HEIGHT_MM / 2);
    pEdges.position.copy(pMesh.position);
    partnerGroup = new THREE.Group();
    partnerGroup.add(pMesh, pEdges);
    scene.add(partnerGroup);
  }
  // odometry is a dashed outline drawn on top (no depth test) so it stays visible where it overlaps the true body
  const ghost = new THREE.LineSegments(new THREE.EdgesGeometry(boxGeo), new THREE.LineDashedMaterial({ color: colors.ghost, dashSize: 40, gapSize: 30, transparent: true, opacity: 0.95, depthTest: false }));
  ghost.computeLineDistances();
  ghost.renderOrder = 10;
  const ghostGroup = new THREE.Group();
  ghost.position.set((r.front - r.back) / 2, 0, ROBOT_HEIGHT_MM / 2);
  ghostGroup.add(ghost);
  scene.add(ghostGroup);

  // trails on the floor
  const trail = (index, color, opacity) => {
    const positions = new Float32Array(run.frames.length * 3);
    run.frames.forEach((fr, i) => positions.set([fr[index], -fr[index + 1], 3], i * 3));
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const line = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color, transparent: true, opacity }));
    line.frustumCulled = false;
    scene.add(line);
    return line;
  };
  const trails = [trail(1, colors.accent, 0.8), trail(4, colors.ghost, 0.6)];

  // camera and controls
  const camera = new THREE.PerspectiveCamera(38, 1, 50, 30000);
  camera.up.set(0, 0, 1);
  const home = { position: new THREE.Vector3(S * 0.5, -S * 2.05, S * 1.2), target: new THREE.Vector3(S / 2, -S / 2, 300) };
  camera.position.copy(home.position);
  const controls = new OrbitControls(camera, canvas);
  controls.target.copy(home.target);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.maxPolarAngle = Math.PI / 2 - 0.03;
  controls.minDistance = 600;
  controls.maxDistance = 9000;
  controls.update();
  controls.enabled = false;           // only while the 3D view is showing
  let following = false;              // F: keep the camera's target on the robot

  return {
    scene, camera, controls,
    resize(w, h) { camera.aspect = w / h; camera.updateProjectionMatrix(); },
    setCamera(px, py, pz, tx, ty, tz) { camera.position.set(px, py, pz); controls.target.set(tx, ty, tz); controls.update(); },
    resetCamera() { camera.position.copy(home.position); controls.target.copy(home.target); controls.update(); },
    /** [px, py, pz, tx, ty, tz] so a rebuilt scene can keep where the camera was. */
    getCamera() { return [camera.position.x, camera.position.y, camera.position.z, controls.target.x, controls.target.y, controls.target.z]; },
    follow(on) { following = on; },
    isFollowing() { return following; },
    /** pose: the robot at this moment; balls: [[x, y, z] per ball]; tilts: [red, blue] in radians; index: frame index for the trails. */
    update(pose, balls, tilts, index, partnerPose) {
      robot.position.set(pose.x, -pose.y, 0);
      robot.rotation.z = -pose.theta;
      if (following) {               // slide the camera and its target together so the view keeps its angle
        const dx = pose.x - controls.target.x, dy = -pose.y - controls.target.y;
        camera.position.x += dx; camera.position.y += dy; controls.target.x += dx; controls.target.y += dy;
      }
      if (partnerGroup && partnerPose) { partnerGroup.position.set(partnerPose.x, -partnerPose.y, 0); partnerGroup.rotation.z = -partnerPose.theta; }
      ghostGroup.visible = Number.isFinite(pose.ox);
      if (ghostGroup.visible) { ghostGroup.position.set(pose.ox, -pose.oy, 0); ghostGroup.rotation.z = -pose.otheta; }
      balls.forEach((p, i) => ballMeshes[i].position.set(p[0], -p[1], p[2]));
      hives.forEach((g, i) => { g.rotation.x = -tilts[i]; }); // scene y is flipped, so the tilt about x flips too
      for (const t of trails) t.geometry.setDrawRange(0, index + 1);
      controls.update();
    },
    dispose() {
      controls.dispose();
      scene.traverse((o) => { o.geometry?.dispose(); o.material?.dispose(); });
    },
  };
}
