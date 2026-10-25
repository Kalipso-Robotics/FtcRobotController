// The 3D view: the same run as the top view, with height. Plain Three.js meshes built from the recorded run:
// floor, walls, the Flowers (lower ring, four pipes, top ring), the Hive frame and its two Hives with Cells that pivot,
// balls as spheres, and the robot as a box. Units are mm. Scene axes: x = field x, y = -field y (so it matches the top
// view), z = up. The camera orbits with the mouse.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const FLOWER_PIPE_TOP_MM = 546;      // manual 9.7: the top ring is 21.5 in above the floor
const FLOWER_GAP_MM = 101;           // the pipes start above the 90 mm open gap
const ROBOT_HEIGHT_MM = 457;         // UNMEASURED: the start-size limit, 18 in

export function build3D(run, colors, canvas) {
  const scene = new THREE.Scene();
  const S = run.field.size;
  const params = run.balls.params;
  const scale = (v) => v; // mm everywhere

  scene.add(new THREE.AmbientLight(0xffffff, 1.35));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(-1500, -4200, 5200);
  scene.add(sun);

  const material = (color, opacity = 1, extra = {}) => new THREE.MeshStandardMaterial({
    color, transparent: opacity < 1, opacity, roughness: 0.75, metalness: 0.05, depthWrite: opacity >= 1, ...extra });

  // floor and tile lines
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(S, S), material(colors.field));
  floor.position.set(S / 2, -S / 2, 0);
  scene.add(floor);
  const lines = [];
  for (let k = 0; k <= 6; k++) {
    lines.push(new THREE.Vector3(k * 609.6, 0, 1), new THREE.Vector3(k * 609.6, -S, 1), new THREE.Vector3(0, -k * 609.6, 1), new THREE.Vector3(S, -k * 609.6, 1));
  }
  scene.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(lines), new THREE.LineBasicMaterial({ color: colors.tile })));

  // walls (a low rim so the field is easy to see into)
  const wallMat = material(colors.line, 0.55);
  for (const [cx, cy, w, h] of [[S / 2, 0, S + 40, 20], [S / 2, -S, S + 40, 20], [0, -S / 2, 20, S], [S, -S / 2, 20, S]]) {
    const wall = new THREE.Mesh(new THREE.BoxGeometry(w, h, 300), wallMat);
    wall.position.set(cx, cy, 150);
    scene.add(wall);
  }

  // Flowers: the obstacle list has each Flower's base box; its centre is the Flower's centre
  const pipeMat = material(colors.obstacle, 1);
  for (const o of run.field.obstacles.filter((o) => o.name.startsWith('flower-'))) {
    const cx = (o.minX + o.maxX) / 2, cy = -((o.minY + o.maxY) / 2);
    const ring = new THREE.Mesh(new THREE.TorusGeometry(48.5, 13, 8, 28), pipeMat);
    ring.scale.z = 0.42; ring.position.set(cx, cy, 5.5);
    scene.add(ring);
    const top = new THREE.Mesh(new THREE.TorusGeometry(57, 6, 8, 28), pipeMat);
    top.position.set(cx, cy, FLOWER_PIPE_TOP_MM);
    scene.add(top);
    for (let i = 0; i < 4; i++) {
      const a = Math.PI / 4 + (i * Math.PI) / 2;
      const pipe = new THREE.Mesh(new THREE.CylinderGeometry(8, 8, FLOWER_PIPE_TOP_MM - FLOWER_GAP_MM, 10), pipeMat);
      pipe.rotation.x = Math.PI / 2;
      pipe.position.set(cx + 57.5 * Math.cos(a), cy - 57.5 * Math.sin(a), (FLOWER_PIPE_TOP_MM + FLOWER_GAP_MM) / 2);
      scene.add(pipe);
    }
  }

  // Hive frame: two legs (the obstacle boxes, standing up to the pivot) and a crossbar
  const hive = run.field.hive;
  const hiveCentreX = (hive.minX + hive.maxX) / 2, hiveCentreY = (hive.minY + hive.maxY) / 2;
  for (const o of run.field.obstacles.filter((o) => o.name.startsWith('hive-leg'))) {
    const leg = new THREE.Mesh(new THREE.BoxGeometry(o.maxX - o.minX, o.maxY - o.minY, params.hivePivotZMm), material(colors.obstacle, 0.28));
    leg.position.set((o.minX + o.maxX) / 2, -((o.minY + o.maxY) / 2), params.hivePivotZMm / 2);
    scene.add(leg);
  }
  const bar = new THREE.Mesh(new THREE.BoxGeometry(hive.maxX - hive.minX, 50, 50), material(colors.line, 1));
  bar.position.set(hiveCentreX, -hiveCentreY, params.hivePivotZMm);
  scene.add(bar);

  // Hives: each is a group that pivots about the x axis; its two Cells are open bins
  const hives = [];
  const hiveOffset = 25.5 * 25.4 / 2;
  const cellWidth = params.hiveCellWidthMm, cellHeight = params.hiveCellHeightMm, inner = params.hiveCellInnerMm, depth = params.hiveCellDepthMm;
  [[hiveCentreX - hiveOffset, colors.red], [hiveCentreX + hiveOffset, colors.blue]].forEach(([x, color]) => {
    const group = new THREE.Group();
    group.position.set(x, -hiveCentreY, params.hivePivotZMm);
    const panelMat = material(color, 0.3);
    const zMid = params.hiveCellBottomMm + cellHeight / 2;
    for (const side of [1, -1]) {                       // side +1 is the +y Cell. Scene y is flipped, so +y is -y here.
      const yMid = -side * (inner + depth / 2);
      const panel = (w, h, d, px, py, pz) => { const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), panelMat); m.position.set(px, py, pz); group.add(m); };
      panel(cellWidth, 6, cellHeight, 0, -side * (inner + 3), zMid);                                      // back wall
      panel(cellWidth, depth, 6, 0, yMid, zMid - cellHeight / 2 + 3);                                     // floor
      panel(cellWidth, depth, 6, 0, yMid, zMid + cellHeight / 2 - 3);                                     // roof
      panel(6, depth, cellHeight, -cellWidth / 2 + 3, yMid, zMid);                                        // sides
      panel(6, depth, cellHeight, cellWidth / 2 - 3, yMid, zMid);
    }
    scene.add(group);
    hives.push(group);
  });

  // balls
  const ballColor = (type) => (type === 0 ? 0xe8b923 : type === 1 ? colors.red : colors.blue);
  const ballMeshes = run.balls.info.map((b) => {
    const mesh = new THREE.Mesh(new THREE.SphereGeometry(b.radius, 20, 14), material(ballColor(b.type), 1, { roughness: 0.5 }));
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
