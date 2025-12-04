# Shooter ML Predictor - Implementation Summary

## What Was Created

I've created a complete Java implementation to use machine learning models (from `.pkl` files) in your FTC robot code.

### 1. Java Implementation
**File:** `TeamCode/src/main/java/com/kalipsorobotics/modules/shooter/ShooterMLPredictor.java`

**Features:**
- Loads ML model parameters from binary files
- Supports StandardScaler (feature normalization)
- Supports KNeighborsRegressor (KNN with inverse distance weighting)
- Supports LinearRegression
- Same API as existing ShooterLutPredictor
- Compatible with Android/FTC environment

**Usage:**
```java
// In your OpMode or robot code
ShooterMLPredictor predictor = new ShooterMLPredictor(hardwareMap.appContext, "ShooterModelParams.bin");

// Make predictions
double xPixel = Shooter.mmToPixel(distanceToTarget);
double yPixel = Shooter.GOAL_HEIGHT_PIXELS;
ShooterMLPredictor.Prediction pred = predictor.predict(xPixel, yPixel);

// Use the predictions
shooterRPS = pred.rps;
hoodPosition = pred.hood;
```

### 2. Python Export Scripts

**File:** `load_and_export_model.py`

This script converts your `ShooterModel.pkl` to a Java-compatible binary format:
- Handles Pipeline models with StandardScaler
- Handles MultiOutputRegressor (for RPS and Hood predictions)
- Exports KNN and Linear Regression models
- Creates `ShooterModelParams.bin` for Java

### 3. Documentation

**File:** `SHOOTER_MODEL_README.md`

Complete guide explaining:
- Why we need to convert pickle files
- How to export the model
- Troubleshooting steps
- Model types supported

## Current Status

### ✅ Complete
- Java implementation for ML predictor
- Support for StandardScaler, KNN, and Linear Regression
- Python export scripts
- Code compiles successfully

### ⚠️ Blocked
- The `ShooterModel.pkl` file has compatibility issues
- Error: "STACK_GLOBAL requires str"
- This prevents automatic export

## Next Steps

### Option A: Fix the Pickle File (Recommended)

If you have the original Python training code:

1. Re-save the model with compatibility:
   ```python
   import pickle

   # ... your model training code ...

   # Save with explicit protocol
   with open('ShooterModel.pkl', 'wb') as f:
       pickle.dump(model, f, protocol=4)
   ```

2. Run the export script:
   ```bash
   python3 load_and_export_model.py
   ```

3. This creates `ShooterModelParams.bin` which Java can load

### Option B: Manual Export

If you can load the model in any Python environment:

```python
import pickle
import struct

# Load the model
with open('ShooterModel.pkl', 'rb') as f:
    model = pickle.load(f)

# Check what type of model it is
print(f"Model type: {type(model)}")
print(f"Model: {model}")

# Test it
import numpy as np
test = model.predict(np.array([[200, 200]]))
print(f"Test prediction: {test}")
```

Then use the export script to convert it.

### Option C: Continue Using LUT Files

Your existing LUT-based system works perfectly:
- `ShooterLutPredictor` with `CappedShooterLUT.bin` or `ShooterLUT2.bin`
- Already tested and working
- No conversion needed

## Model Architecture (from pickle inspection)

Based on the hexdump of `ShooterModel.pkl`, the model appears to be:
```
Pipeline([
    ('scaler', StandardScaler()),
    ('regressor', MultiOutputRegressor or similar)
])
```

The model has:
- Feature names (likely 'x_pixel', 'y_pixel')
- StandardScaler with mean and scale parameters
- A regressor that predicts RPS and Hood position

## Testing

Once you have a working `ShooterModelParams.bin`:

```java
@Test
public void testMLPredictorVsLUT() throws Exception {
    // Load ML predictor
    ShooterMLPredictor mlPredictor = new ShooterMLPredictor(
        new FileInputStream("src/main/assets/ShooterModelParams.bin"));

    // Load LUT predictor for comparison
    ShooterLutPredictor lutPredictor = new ShooterLutPredictor(
        new FileInputStream("src/main/assets/ShooterLUT2.bin"));

    // Compare predictions
    double[][] testPoints = {
        {200, 200},
        {500, 200},
        {750, 200}
    };

    for (double[] point : testPoints) {
        var mlPred = mlPredictor.predict(point[0], point[1]);
        var lutPred = lutPredictor.predict(point[0], point[1]);

        System.out.printf("Point [%.0f, %.0f]:%n", point[0], point[1]);
        System.out.printf("  ML:  RPS=%.2f, Hood=%.4f%n", mlPred.rps, mlPred.hood);
        System.out.printf("  LUT: RPS=%.2f, Hood=%.4f%n", lutPred.rps, lutPred.hood);
        System.out.printf("  Diff: ΔRPS=%.2f, ΔHood=%.4f%n%n",
            mlPred.rps - lutPred.rps,
            mlPred.hood - lutPred.hood);
    }
}
```

## Questions to Help Resolve the Issue

1. **Do you have the original Python code that created `ShooterModel.pkl`?**
   - If yes, we can re-export it cleanly

2. **What Python version was used to create the pickle file?**
   - Python 2 vs Python 3 incompatibility could cause this

3. **What type of model is it?**
   - KNN? Linear? RandomForest? GradientBoosting?

4. **Can you load it in any Python environment?**
   - Even if just to inspect it and manually export parameters

5. **Do you need to use the ML model, or would the LUT approach work?**
   - The LUT files you already have are working fine

## Files in This Project

- `ShooterMLPredictor.java` - Java ML model loader (✅ Complete, compiles)
- `load_and_export_model.py` - Python export script (✅ Complete)
- `export_shooter_model.py` - Alternative export script (✅ Complete)
- `SHOOTER_MODEL_README.md` - Detailed documentation (✅ Complete)
- `SHOOTER_ML_SUMMARY.md` - This file (✅ Complete)

## Summary

You now have everything needed to use ML models in Java/FTC:
1. ✅ Complete Java implementation
2. ✅ Export scripts ready
3. ⚠️ Just need to fix the pickle file to complete the pipeline

Let me know if you have the original Python training code or can load the model, and I can help you complete the export!
