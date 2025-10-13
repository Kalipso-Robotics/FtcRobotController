# ShooterLutPredictor Unit Test

This unit test allows you to test the ShooterLutPredictor locally on your machine without needing the FTC controller.

## Running the Test

### Option 1: Using Gradle (Recommended)

First, you need to fix the compilation errors in the project (RevolverTeleop.java). Once fixed, run:

```bash
cd /Users/ethan/Kalipso-Robotics
./gradlew :TeamCode:testDebugUnitTest
```

### Option 2: Using IntelliJ IDEA or Android Studio

1. Open the project in your IDE
2. Navigate to `TeamCode/src/test/java/com/kalipsorobotics/modules/shooter/ShooterLutPredictorTest.java`
3. Right-click on the test class
4. Select "Run 'ShooterLutPredictorTest'"

## Test Coverage

The test includes:

1. **testPredictionAtKnownDataPoint** - Verifies exact match when predicting at a known data point
2. **testPredictionInterpolation** - Tests interpolation between known points
3. **testHoodClamping** - Ensures hood values are clamped to valid range [0.23, 0.8]
4. **testPredictionConsistency** - Verifies same input produces same output
5. **testMultiplePositions** - Tests predictions across various positions

## Data File

The test uses the real data from: `TeamCode/src/test/resources/Shooter LUT.json`

This is a copy of the production data file from `TeamCode/src/main/assets/Shooter LUT.json`
