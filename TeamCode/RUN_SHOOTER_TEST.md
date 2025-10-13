# ShooterLutPredictor Unit Test

A unit test for the ShooterLutPredictor that runs locally on your machine (no FTC controller needed).

## Quick Start - Run with Gradle (Recommended)

```bash
cd /Users/ethan/Kalipso-Robotics
./gradlew :TeamCode:testDebugUnitTest
```

## Alternative - Run the Standalone Test

```bash
cd /Users/ethan/Kalipso-Robotics/TeamCode

# Compile and run
javac src/test/java/com/kalipsorobotics/modules/shooter/StandaloneShooterLutPredictorTest.java
java -cp src/test/java com.kalipsorobotics.modules.shooter.StandaloneShooterLutPredictorTest
```

## What the Test Does

The test verifies the ShooterLutPredictor using real data from `Shooter LUT.json`:

1. **Known Data Point Test** - Verifies exact match when predicting at a known point (200, 200)
2. **Interpolation Test** - Tests KNN interpolation between nearby points
3. **Hood Clamping Test** - Ensures hood values stay within [0.23, 0.8] range
4. **Consistency Test** - Verifies predictions are deterministic
5. **Multiple Positions Test** - Tests various (x, y) pixel positions

## Files Created

1. **ShooterLutPredictorTest.java** - JUnit test
   - Location: `src/test/java/com/kalipsorobotics/modules/shooter/`
   - Run with: `./gradlew :TeamCode:testDebugUnitTest`
   - Uses data directly from: `src/main/assets/ShooterLUT.json`

2. **StandaloneShooterLutPredictorTest.java** - Standalone test (no dependencies)
   - Location: `src/test/java/com/kalipsorobotics/modules/shooter/`
   - Can run with just `java` and `javac` commands
   - Uses data from: `src/test/resources/ShooterLUT.json`

## Implementation Details

The predictor uses:
- **K-Nearest Neighbors (K=8)** for finding nearby data points
- **Inverse Distance Weighting (Shepard's method)** for interpolation
- **Hood clamping** to keep values in valid servo range [0.23, 0.8]

## Example Output

```
Loaded 1025 data points from LUT

=== Running Shooter LUT Predictor Tests ===

Test 1: Prediction at known data point (200, 200)
  Expected: rps=38.812, hood=0.581
  Actual:   rps=38.812, hood=0.581
  ✓ PASS

Test 2: Prediction with interpolation (225, 225)
  Result: rps=36.007, hood=0.431
  ✓ PASS

...

=== All Tests Passed! ===
```
