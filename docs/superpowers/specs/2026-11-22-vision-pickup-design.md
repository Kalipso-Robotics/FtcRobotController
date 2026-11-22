# Vision pickup: V1 design (built) and V2 plan

## Goal
Any auto can call one action with simple parameters. It looks at the balls the Arducam sees, drives to them with the intake on, keeps going until the intake holds 4 balls, and stops wherever it ends up. The auto then drives to its shoot point and shoots. The ball source is either the TFLite model or HSV color blobs.

## Using it (V1)

```java
// init
ballVision = BallVision.start(hardwareMap, BallVision.Detector.TFLITE, false); // or COLOR_BLOB
beamBreakCountAction = new BeamBreakCountAction(opModeUtilities);             // update it every loop: the ball count

// graph
VisionPickupAction pickup = new VisionPickupAction(driveTrain, stopper, intake, allianceColor)
        .setTimeoutMs(6000);           // optional: give up in time to shoot and park
pickup.setName("pickup");
pickup.setDependentActions(driveToLookPoint);  // somewhere the camera can see the balls
auto.addAction(pickup);
goToShoot.setDependentActions(pickup);         // shoot as usual

// end
ballVision.close();
```

Optional setters: `setTargetBalls(n)` (default 4), `setMaxStops(n)` (default `MAX_STOPS` = 3), `setTimeoutMs(ms)` (default none).
Results: `getEndReason()` (FULL, NOTHING_TO_PICK, MAX_STOPS, TIMEOUT, DROPPED), `getBallsPickedUp()`, `getStops()`, `getFailed()`.
RedThreeTipSoloAuto trip 4 uses it.

## How V1 works

```
LOOK ── wait LOOK_FRAMES new frames (or LOOK_TIMEOUT_MS) ── every sighting goes into BallMemory
  │
CHOOSE ── ClusterDetectionAction.chooseBest(memory, robot, alliance, room = target − held)
  │         null ─────────────────────────────────────────────────────────► DONE (NOTHING_TO_PICK)
PICKUP ── AutoToBallsPathingAction(aim).setIntakeFromStart(true).setOvershootMm(OVERSHOOT_MM)
  │         forget that group's balls; stops ≥ max ──────────────────────► DONE (MAX_STOPS)
  └──► LOOK again
Every loop: ball count ≥ target ──► DONE (FULL), even mid-drive.  Timeout ──► DONE (TIMEOUT).
Done or dropped: the running pickup is stopped, which stops the intake. The robot stays where it is.
```

| Piece | File | What changed |
|---|---|---|
| Action | `actions/autoActionsPath/VisionPickupAction.java` | Renamed from `BallDetectionPathing`, which had no callers. Now loops look → choose → pickup. |
| Choosing | `biobuzz/ClusterDetectionAction.java` | Fixed 24″ tiles replaced by RedDecideAuto's 250 mm chaining. Score = points per second, capped by free room, with a one-group lookahead. Aims at the nearest ball when the middle of a group is empty floor. Skips the other half of the field. Runs in a single pass, not one comparison per loop. |
| Memory | `biobuzz/BallMemory.java` | New. Same rules as RedDecideAuto: merge within 100 mm, forget after `MEMORY_MS`. |
| Overshoot | `navigation/PathPlanning.overshoot` | Moves the stop point further along the drive-in direction, but only if the body still fits (walls, legs, midline). |
| Intake on the way | `AutoToBallsPathingAction.setIntakeFromStart` | The leading side runs for the whole drive, not only for the last 600 mm. |
| Detector choice | `vision/BallVision.Detector`, `vision/colorblobbing/BallColorBlobProcessor.java` | Color blob covers yellow, red and blue. Its labels match what `Raytracer.typeFor` expects, so the raytracer needs no change. |

### Knobs to tune on the robot (all `TODO-measure`)
- `VisionPickupAction.OVERSHOOT_MM` (80): the smallest value at which the last ball always breaks the beam.
- `VisionPickupAction.LOOK_FRAMES` (3) and `LOOK_TIMEOUT_MS` (500).
- `BallMemory.MEMORY_MS` (5000).
- `ClusterDetectionAction.SPEED_MM_PER_SECOND` (1000) and `STOP_SECONDS` (1.0), both guesses shared with RedDecideAuto.
- `BallColorBlobProcessor` red and blue HSV ranges: these are textbook values. Tune them before using COLOR_BLOB.

### Known limits of V1
- A group containing an opponent or UNKNOWN ball is skipped entirely. One misread can hide a good pile.
- Scoring uses straight-line distance, not the planner's route around the Hive.
- Memory forgets by age only.
- The camera only adds to memory during LOOK, not while driving.
- No look-point selection: the auto chooses where the robot stands to look.

## V2 plan (not built)

Ordered by value per effort.

1. **Remember while driving.** Add frames to `BallMemory` in every state, not only during LOOK. The Raytracer already stamps each frame with the pose at capture time, so sightings taken while moving land in the right place. This removes most re-looks.
2. **Field-of-view forgetting (negative evidence).** Forget a remembered ball only when its spot is inside the camera's view (using `VisionConfig.ARDUCAM` intrinsics and the robot pose) and no detection lands there for N frames. A ball that drifts out of view because the robot drove closer stays remembered. This is the "balls fade out of view" case.
3. **Tracks instead of sightings (multi-object tracking).** Each ball becomes a track with a hit count. Nearest-neighbor association is enough at FTC ball counts. Trust a track after 2 or more hits, and forget it after K misses while in view. This filters single-frame false positives, which matters most for color blob.
4. **Whole-route planning (capacitated orienteering).** Choose the ordered set of groups that maximizes points per second within `room`, then re-plan after every stop (receding horizon). With 6 or fewer groups, trying every order is instant. `chooseBest` already does the two-group version. Generalize it, and use `PathPlanning.plan` lengths instead of straight lines.
5. **Multi-trip awareness.** When the groups in memory hold more than 4 balls, return the plan for the *next* trip too. The auto shoots, then calls the action again, and it starts from memory without re-looking. This needs the memory to outlive one action, for example one `BallMemory` owned by the OpMode and passed in.
6. **Commitment hysteresis.** Keep the chosen target unless a new option scores clearly better (RedDecideAuto's `COMMIT_SWITCH_SCORE` of 1.2×). This only matters once the robot re-plans mid-drive.
7. **Share with RedDecideAuto.** Move its memory and grouping onto `BallMemory` and `ClusterDetectionAction` so both autos run the same tested code.

Real-world counterparts: object permanence and track management (multi-object tracking: association, hit/miss counts), evidence grids and negative evidence (occupancy mapping), the capacitated vehicle routing and orienteering problems (which stops, in what order, under a capacity limit), receding-horizon / MPC-style replanning, and active perception (choosing where to look from).
