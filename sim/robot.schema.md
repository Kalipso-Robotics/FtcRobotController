# sim/robot.json

The robot's geometry, written by the `#/robot` editor, read by the physics (`sim/physics/world.mjs`) and the 3D view.
All numbers are **millimetres in the robot frame**: origin = the odometry point, x forward, y left, z up (floor = 0).
Angles are degrees. Every key is optional; a missing one means "use the old built-in value".
`RobotFootprint.java` stays the source of truth for the outline: `footprint` here is only checked against it (a warning, never an overwrite).

```json
{
  "version": 1,
  "footprint": { "front": 252, "back": 252, "width": 337 },
  "meshes": [
    { "file": "body.stl", "part": "body", "scale": 1, "rotationDeg": [0, 0, 0], "offset": [0, 0, 0] }
  ],
  "intakeZones": [
    { "name": "front", "side": "front", "min": [252, -150, 0], "max": [330, 150, 60] }
  ],
  "transferPath": [[250, 0, 90], [120, 0, 150], [-120, 0, 150], [-250, 0, 90]],
  "turret": { "pivot": [0, 0, 300] },
  "shooter": { "exit": [60, 0, 500], "hoodPivot": [40, 0, 420], "hoodMinDeg": 68.5, "hoodMaxDeg": 40 },
  "beams": [
    { "name": "beamBreakAOuter", "emitter": [120, -40, 150], "receiver": [120, 40, 150] }
  ],
  "noodles": [
    { "name": "front", "center": [200, 0, 80], "axis": [0, 1, 0], "radius": 40, "length": 300, "blades": 6 }
  ]
}
```

- `meshes[].part`: `body` | `turret` | `hood`. `file` is in `sim/robot/`. The mesh is placed as: scale, then rotate (x, y, z degrees, three.js `Euler` order `XYZ`), then offset.
  (Editor workflow: load STL, pick units/rotation, click the odometry point; those become `scale`, `rotationDeg`, `offset`.)
- `meshes[].color` (optional): `"#rrggbb"`. Overrides the per-part shade in both 3D views.
- `beams[].name` must be one of the hardware-map names `beamBreakAOuter`, `beamBreakAInner`, `beamBreakBInner`, `beamBreakBOuter`.
- Physics uses today (P1): `beams` x/z -> where a held ball sits in each slot (all four needed, else the old slots); `shooter.exit` (+ `turret.pivot`)
  -> where a shot leaves: the pivot rides on the robot, the exit swings about it with the turret; `intakeZones` -> how far past the robot's
  front/back a ball is picked up (zone `max[0]` / `min[0]`, length only). The 3D view draws `meshes`. `transferPath` and `noodles` are stored for the
  beam work (P5); `shooter.hoodPivot` and the hood angles for P4.
- `sim/physics/server.mjs` reads this file on every `init` (no Java involved), so saving it and re-running is enough.
