# Shooter Manual Data Lookup - Linear Interpolation

A simple linear interpolation lookup table based on your manually tuned shooter data from `ShooterTestedManualData.csv`.

## Files Created

### 1. `ShooterManualDataLookup.java`
**Location:** `TeamCode/src/main/java/com/kalipsorobotics/modules/shooter/ShooterManualDataLookup.java`

**Features:**
- ✅ Loads data from CSV file
- ✅ Linear interpolation between data points
- ✅ Handles edge cases (distances outside data range)
- ✅ Simple, efficient lookup

### 2. `ShooterManualDataLookupTest.java`
**Location:** `TeamCode/src/test/java/com/kalipsorobotics/modules/shooter/ShooterManualDataLookupTest.java`

**Tests:**
- ✅ Data loading verification
- ✅ Exact data point matching
- ✅ Linear interpolation accuracy
- ✅ Edge case handling
- ✅ All tests passing!

## Usage

### In Your Robot Code

```java
// Initialize (do this once in your robot init)
ShooterManualDataLookup lookup = new ShooterManualDataLookup(
    hardwareMap.appContext,
    "ShooterTestedManualData.csv"
);

// Use in teleop/auto (calculate distance to target first)
double distanceMM = /* your distance calculation */;

// Get shooter parameters
ShooterManualDataLookup.ShooterParams params = lookup.getParams(distanceMM);

// Use the values
double targetRPS = params.rps;
double targetHoodPosition = params.hoodPosition;

// Set your shooter
shooter.setRPS(targetRPS);
shooter.setHoodPosition(targetHoodPosition);
```

### Example with Distance Calculation

```java
// Calculate distance to target
Point robotPosition = odometry.getPosition();
Point targetPosition = Shooter.RED_TARGET_FROM_NEAR;
double distancePixels = Point.distance(
    robotPosition.getX(),
    robotPosition.getY(),
    targetPosition.getX(),
    targetPosition.getY()
);
double distanceMM = Shooter.pixelToMM(distancePixels);

// Look up shooter parameters
ShooterManualDataLookup.ShooterParams params = lookup.getParams(distanceMM);

// Apply to shooter
shooter.setTargetRPS(params.rps);
shooter.setHoodPosition(params.hoodPosition);
```

## How Linear Interpolation Works

When you request parameters for a distance that's between two data points:

1. **Find surrounding points:**
   - Example: You want 1000mm, data has 913mm and 1108mm

2. **Calculate interpolation factor (t):**
   ```
   t = (distance - lower) / (upper - lower)
   t = (1000 - 913) / (1108 - 913)
   t = 87 / 195 = 0.446
   ```

3. **Interpolate RPS:**
   ```
   RPS = lower_rps + t * (upper_rps - lower_rps)
   RPS = 35 + 0.446 * (36 - 35)
   RPS = 35.446
   ```

4. **Interpolate Hood:**
   ```
   Hood = lower_hood + t * (upper_hood - lower_hood)
   Hood = 0.38 + 0.446 * (0.55 - 0.38)
   Hood = 0.456
   ```

## Your Data Summary

From `ShooterTestedManualData.csv`:
- **20 data points** from 540mm to 3547mm
- **RPS range:** 34 to 52
- **Hood range:** 0.1 to 0.55

### Data Points (Sample):

| Distance (mm) | RPS  | Hood Position |
|---------------|------|---------------|
| 540           | 34.0 | 0.10          |
| 682           | 34.0 | 0.31          |
| 913           | 35.0 | 0.38          |
| 1108          | 36.0 | 0.55          |
| 1323          | 37.0 | 0.55          |
| ...           | ...  | ...           |
| 2825          | 49.0 | 0.55          |
| 3076          | 50.5 | 0.55          |
| 3547          | 52.0 | 0.55          |

## Edge Cases

### Distance Below Minimum (< 540mm)
Returns the first data point (RPS=34, Hood=0.1)

### Distance Above Maximum (> 3547mm)
Returns the last data point (RPS=52, Hood=0.55)

### Exact Match
Returns the exact values from your manual tuning

## Testing

Run the tests:
```bash
./gradlew :TeamCode:test --tests "ShooterManualDataLookupTest"
```

All 6 tests should pass:
- ✅ testDataLoaded
- ✅ testExactDataPoints
- ✅ testLinearInterpolation
- ✅ testEdgeCases
- ✅ testMultipleDistances
- ✅ testInterpolationAccuracy

## Integration with Existing Code

### Option 1: Replace ShooterLutPredictor

If you want to use this instead of the LUT predictor:

```java
// Old code
ShooterLutPredictor predictor = new ShooterLutPredictor(ctx, "ShooterLUT2.bin");
ShooterLutPredictor.Prediction pred = predictor.predict(xPixel, yPixel);

// New code
ShooterManualDataLookup lookup = new ShooterManualDataLookup(ctx, "ShooterTestedManualData.csv");
ShooterManualDataLookup.ShooterParams params = lookup.getParams(distanceMM);
```

### Option 2: Use Alongside

Keep both available and switch based on preference or testing:

```java
// In Shooter class
private ShooterManualDataLookup manualLookup;
private ShooterLutPredictor lutPredictor;

public void init(Context ctx) {
    manualLookup = new ShooterManualDataLookup(ctx, "ShooterTestedManualData.csv");
    lutPredictor = new ShooterLutPredictor(ctx, "ShooterLUT2.bin");
}

public void setTargetFromDistance(double distanceMM) {
    // Use manual tuned data
    ShooterManualDataLookup.ShooterParams params = manualLookup.getParams(distanceMM);
    setTargetRPS(params.rps);
    setHoodPosition(params.hoodPosition);
}
```

## Advantages of This Approach

1. **Simple and Transparent:** Easy to understand and debug
2. **Based on Real Testing:** Uses your actual manually tuned values
3. **Fast:** Just O(log n) binary search + simple math
4. **Predictable:** Linear interpolation is smooth and predictable
5. **Easy to Update:** Just edit the CSV file and reload

## Updating the Data

To add or modify shooter parameters:

1. Edit `TeamCode/src/main/assets/ShooterTestedManualData.csv`
2. Add new rows or update existing ones
3. Keep the format: `DistanceMM,RPS,Hood Position Relative,Optional`
4. Data can be in any order (will be sorted automatically)
5. Rebuild and run!

## Comparison with Other Methods

| Method                    | Pros                          | Cons                        |
|---------------------------|-------------------------------|-----------------------------|
| **Manual Data Lookup**    | Simple, tested values, fast   | Limited to tested distances |
| **ML Model (KNN)**        | Can generalize, many points   | Complex, needs export       |
| **LUT Binary**            | Very fast, compact            | Requires preprocessing      |
| **Regression Model**      | Smooth curve, mathematical    | May not match reality       |

## Next Steps

1. ✅ Load and test with your robot
2. ✅ Verify the interpolated values make sense
3. ✅ Test shooting at various distances
4. ✅ Add more data points if needed
5. ✅ Fine-tune based on actual performance

Enjoy your accurately tuned shooter! 🎯
