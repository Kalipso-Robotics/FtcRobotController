# Vision pipeline handoff

Snapshot of vision work in this session: what changed, what's still broken, what's next.

## What was done

### 1. Raytracing test OpMode
Created `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/kalipsorobotics/test/cameraVision/CameraIntrinsicsDistanceTest.java`.

- Live validation of `CameraIntrinsics.calculateWorldPos` / `getDistanceFromRobot`.
- Two modes:
  - **Probe pixel** — set `PROBE_PX` / `PROBE_PY` on FtcDashboard and the OpMode raytraces that pixel. Detector-independent.
  - **Largest TFLite detection** — pulls `artifacts.getLatestResult()`, picks the largest by `getArea()`, and raytraces its bottom-middle pixel.
- Streams the camera to FtcDashboard via `FtcDashboard.startCameraStream(portal, 30)`.
- Dashboard-tunable: probe pixel, mount angle, camera offsets (X/Y/Z), exposure, gain, resolution, simulated robot position.
- Per-frame raytrace dump logged to logcat with tag `RAYTRACE_DBG` — every intermediate value (norm, dir, world, t, floor, fromRobot, plus the real function output).
- Initially used the color-blob processor; switched to `TFLiteArtifactDetector` per user request.

### 2. TFLite model conversion
- User's `best.pt` (YOLO Ultralytics, single class `Artifact-Detector`, 640×640) was sitting in `~/Downloads/`. Moved to `/Users/ethan/Kalipso-Robotics/best.pt` to work around macOS TCC restrictions on `~/Downloads`.
- Exported via Ultralytics in a Python 3.12 venv (TF doesn't support 3.14): `YOLO('best.pt').export(format='tflite', imgsz=640)`.
- Verified output tensor shape `[1, 5, 8400]` matches what `TFLiteArtifactDetector` decodes (`[1, 4+nc, anchors]`, single class).
- Copied to `TeamCode/src/main/assets/best_float32.tflite` (10.6 MB). Filename matches the existing `MODEL_FILE` constant — no Java edits needed.
- Cleaned up `/tmp/yolo_export_env/` and `/Users/ethan/Kalipso-Robotics/best_saved_model/`. `best.pt` remains at repo root and should be moved out or `.gitignore`d.

### 3. Intrinsics resolution bug (root cause of "raytracing is way off")
- `CameraIntrinsics.ARDUCAM` was calibrated at **1280×800** via `calibrate_camera.py` (lines 129-130 set the capture resolution explicitly).
- `VisionManager.Builder` defaults to **640×480** streaming.
- Pixel-based intrinsics (`fx, fy, cx, cy`) only make sense at the resolution they were captured at. `cx=700` literally cannot exist in a 640-wide frame — that was the smoking gun.
- Applied **Option B**: scaled the constants and baked them into `CameraIntrinsics.ARDUCAM` (`CameraIntrinsics.java` lines 14-23) for 640×480 streaming. Scale factors: `fx, cx *= 0.5`, `fy, cy *= 0.6`. Distortion coeffs stay the same (dimensionless).
- Also updated the hardcoded `fx`/`fy` in `dumpRayTrace` so the manual recomputation matches `calculateWorldPos`.

### 4. Sign-flip fix (was already applied by user before this session)
`CameraIntrinsics.calculateWorldPos` had its image-Y inverted: `double norm_y = this.cy - pixelY;` (line 58). This is consistent with the Y-UP world convention used by the rest of the math (camera height stored as positive `cameraOffset.Y`, ray going down requires `world_y < 0`). The dump in the test OpMode was updated to match.

## Current problems

### A. TFLite model fails to load at runtime
`TFLiteArtifactDetector.java:90` throws `Failed to load TFLite model: best_float32.tflite`. The file IS in `TeamCode/src/main/assets/` (verified), but it's not in the merged-assets directory of the last build (`build/intermediates/assets/debug/...` only contains the older `robotv2_model.tflite` and `mecanum_inverse.tflite`). Gradle cached the asset merge step from before the file was copied in.

**Fix (untried by user as of this handoff):**
```bash
./gradlew :TeamCode:clean :TeamCode:installDebug
```

Or minimal:
```bash
rm -rf TeamCode/build/intermediates/assets TeamCode/build/intermediates/merged_assets
./gradlew :TeamCode:installDebug
```

Verify with:
```bash
find TeamCode/build/intermediates -name "best_float32.tflite"
```

### B. Raytracing not yet end-to-end validated with scaled intrinsics
- Hand-check predicts ~756 mm for the green-ball pixel `(343, 408)` with the new scaled intrinsics. User estimated ~1000 mm in person. The ~25% gap is probably distance estimation error and the bounding-box bottom not being exactly the floor contact (color thresholding clips edges).
- Once the TFLite asset loads (problem A), the user should run the OpMode, place an object at a measured distance, and confirm the reported distance agrees within ~5%.

### C. Single-class TFLite model
The trained model has one class `Artifact-Detector`. `TFLiteArtifactDetector.java:53` has `LABEL = "Artifact"`. Every detection — purple or green ball — gets the same label. The test OpMode picks the *largest* detection rather than purple vs. green.

If purple/green distinction is needed later: retrain with two classes and refactor the detector:
- Replace `LABEL` constant with `String[] LABELS = {"Purple", "Green"}` (or whatever order the model uses).
- In `detect()`, track the winning class index alongside the score: `if (score > bestScore) { bestScore = score; bestClassIndex = classIndex; }`.
- Pass `LABELS[bestClassIndex]` to the `VisionRecognition` constructor.

## What still needs to be done

1. **Clean rebuild and verify TFLite asset loads** (problem A above).
2. **Re-run `Test: Intrinsics Raytracing` OpMode** with the scaled intrinsics:
   - Confirm the probe pixel at a known floor location reports the expected distance.
   - Confirm the largest detection's distance matches a tape-measured ground truth within ~5%.
3. **Move `best.pt` out of the repo** (currently sitting at `/Users/ethan/Kalipso-Robotics/best.pt`) or add it to `.gitignore`. Otherwise it'll get committed.
4. **Optional polish** (not blocking):
   - Add `forResolution(int w, int h)` factory to `CameraIntrinsics` that auto-scales from the calibration resolution stored alongside the constants. This would make the whole "wrong resolution" bug class impossible to repeat.
   - Add public getters for `fx`, `fy`, `focalLength` to `CameraIntrinsics` so `dumpRayTrace` in the test OpMode doesn't need hardcoded constants.
   - In `calibrate_camera.py`, change capture resolution (lines 129-130) to match the robot's streaming resolution (640×480), or save the capture resolution into the JSON output so future imports know to scale.
   - The `RedAutoDepotVision.java` (currently open in IDE) consumes `CameraIntrinsics` — verify it works with the now-scaled values. Math should be identical, but worth a glance.

## Reference: files touched this session

| File | What changed |
|---|---|
| `TeamCode/.../test/cameraVision/CameraIntrinsicsDistanceTest.java` | Created. Probe-pixel + TFLite raytrace test with `RAYTRACE_DBG` logcat dump. |
| `TeamCode/.../vision/CameraIntrinsics.java` | Lines 14-23: scaled `ARDUCAM` constants from 1280×800 calibration to 640×480. Line 58: `norm_y = cy - pixelY` (sign-flip, was applied prior). |
| `TeamCode/src/main/assets/best_float32.tflite` | Replaced with user's trained model exported from `best.pt`. |

## Reference: how to inspect what got built

```bash
# What's the current merged-asset state of the TeamCode APK?
find TeamCode/build/intermediates -name "*.tflite"

# Raw raytrace dump from the robot:
adb logcat -d -s RAYTRACE_DBG > raytrace.log

# Verify the model's I/O shapes (run on laptop, in a Python venv with tensorflow):
python -c "
import tensorflow as tf
i = tf.lite.Interpreter('TeamCode/src/main/assets/best_float32.tflite')
i.allocate_tensors()
print('input:',  i.get_input_details()[0]['shape'])
print('output:', i.get_output_details()[0]['shape'])
"
```
