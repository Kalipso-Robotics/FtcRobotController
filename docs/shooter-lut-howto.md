# Shooter lookup table (LUT): how to fill it

The shooter picks its flywheel speed (RPS) and hood position from the distance to the Hive.
`ShooterInterpolationDataLookup` reads the rows in `ShooterInterpolationConfig.shooterData`.
Its rows are still the old DECODE robot's numbers, carried over, not BIOBUZZ data. Replace all of them with a BIOBUZZ sweep before trusting shot distance.

## The three OpModes

| OpMode | What it is for | Output file |
|---|---|---|
| `MotorPowerRPSDataCollector` | finds kF | `MotorPowerRPSData_*.csv` |
| `ShooterDataCollector` | **fills the LUT** | `ShooterDataRows_*.csv` |
| `RPSPredictionDataCollector` | open-loop check: power -> RPS at a voltage, hood and distance, plus target point and turret offset | `RPSPredictionData_*.csv` |

All files go to `/sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/RobotLogs/`.

## Steps

1. **kF.** Put in a full battery and run `MotorPowerRPSDataCollector`.
   It steps the power from 0.3 to 1.0 in 0.1 steps, waits 6 s at each step, and samples for 1 s.
   Take the `kf` column at about 0.8 power and put it in `ShooterConfig.kf`. kP stays hand-tuned.
2. **Rows.** Run `ShooterDataCollector` at several distances, near to far.
   - Set `targetRPS` and `hoodPosition` live in Dashboard (the `ShooterDataCollector` config block).
   - Load balls with GP2 RT, then fire with GP1 LB.
   - Mark the shot: GP2 A = hit, GP2 B = miss. Each mark writes one row; A/B do nothing before the first shot.
   - The screen shows the last row saved and the hit/miss counts.
   - Each row looks like `2400, 52.0, 0.2100, // hit measuredRps=51.6` (distance mm, target RPS, hood, result, flywheel RPS at the shot).
3. **Get the files:**
   `adb pull /sdcard/Android/data/com.qualcomm.ftcrobotcontroller/files/RobotLogs/`
4. **Paste** only the `// hit` rows into `shooterData` in `ShooterInterpolationConfig`.
   - Keep them **sorted by distance** (smallest first).
   - Remove near-duplicates: one row per distance; if you have several hits at one spot, keep the middle one.
   - Units: distance in mm, RPS, and the **raw** hood servo value (0.1284-0.3, `MIN_HOOD`..`MAX_HOOD`, with `hoodFlipDirection=true`).
   - If `measuredRps` is far below the target RPS, the flywheel had not reached speed; re-shoot that spot.
5. **Rebuild** and deploy.

## How the lookup behaves

- Empty table: `DEFAULT_RPS` and `MIN_HOOD` (and an error in the log).
- Closer than the first row: the first row. Farther than the last row: the last row.
- Between two rows: straight-line interpolation of RPS and hood.
- `rpsOffset` / `hoodOffset` are added on top at match time (quick trim from Dashboard).
  **Never bake them into the rows**: measure with both at 0.
