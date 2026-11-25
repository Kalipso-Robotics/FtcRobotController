# Shooter lookup table (LUT): how to fill it

The shooter picks its flywheel speed (RPS) and hood position from the distance to the Hive.
`ShooterInterpolationDataLookup` reads the rows in `ShooterInterpolationConfig.shooterData`.
Its rows are still the old DECODE robot's numbers, carried over, not BIOBUZZ data. Replace all of them with a BIOBUZZ sweep before trusting shot distance.

## The three OpModes

| OpMode | What it is for | Output file |
|---|---|---|
| `MotorPowerRPSDataCollector` | finds kF (A) and kP (B) | `MotorPowerRPSData_*.csv`, `MotorKpTuning_*.csv` |
| `ShooterDataCollector` | tunes kF then kP (RPS % error), kF samples for voltage comp | `ShooterKfSamples_*.csv` |
| `RPSPredictionDataCollector` | **fills the LUT**: RPS, target RPS (closed loop with kF/kP), power, voltage, hood and distance of each saved hit, plus target point and turret offset | `RPSPredictionData_*.csv` |

All files go to `/sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/RobotLogs/`.

## Steps

1. **kF and kP.** Run `ShooterDataCollector` on a full battery. GP1 X = flywheel on/off, GP1 Y = next step.
   - Step 1, kF: GP1 dpad sets the power; at STEADY, GP1 RB takes a sample. Sample 2-3 powers around your shooting RPS.
   - Step 2, kP: kP starts at 0, so the % error shown is kF's alone. Raise kP (GP2 dpad) until it says OSCILLATING and keeps
     swinging (an RPS step on GP1 dpad or a ball on GP1 LB kicks it). GP1 RB takes that kP as Ku and sets kP = 0.5 Ku.
   - Step 3, shots: GP2 RT loads, GP1 LB runs the real `ShooterReady` + `PushBall`. Watch error % at the feed, drop % and recovery; trim kP.
   - Copy kP and kF into `ShooterConfig`: they are lost when the app restarts.
   - (`MotorPowerRPSDataCollector` does both automatically, no balls: A = kF sequence, then B = kP sequence, X = cancel. Copy the two values into `ShooterConfig`.)
   **Voltage comp** (`ShooterConfig.VOLTAGE_COMP_EXPONENT`, 0 = off, 1 = full): take step 1 samples at one power on two batteries
   at least 0.7 V apart (robot still, intake off). Exponent = ln(RPS1 / RPS2) / ln(V1 / V2). Then redo kF.
2. **Rows.** Run `RPSPredictionDataCollector` at several distances, near to far (controls are on its screen). Save hits only (Y).
3. **Get the files:**
   `adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/RobotLogs/`
4. **Copy** `DistanceToTargetMM, CurrentRPS, HoodPosition` of each row into `shooterData` in `ShooterInterpolationConfig`.
   - Keep them **sorted by distance** (smallest first).
   - Remove near-duplicates: one row per distance; if you have several hits at one spot, keep the middle one.
   - Units: distance in mm, RPS, and the **raw** hood servo value (0.1284-0.3, `MIN_HOOD`..`MAX_HOOD`, with `hoodFlipDirection=true`).
5. **Rebuild** and deploy.

## How the lookup behaves

- Empty table: `DEFAULT_RPS` and `MIN_HOOD` (and an error in the log).
- Closer than the first row: the first row. Farther than the last row: the last row.
- Between two rows: straight-line interpolation of RPS and hood.
- `rpsOffset` / `hoodOffset` are added on top at match time (quick trim from Dashboard).
  **Never bake them into the rows**: measure with both at 0.
