# ShooterModel - Using ML Models in Java/FTC

## Problem

The `ShooterModel.pkl` file is a Python pickle file containing a scikit-learn machine learning model. Java (and Android/FTC) cannot directly load pickle files, so we need to convert it to a format Java can use.

## Current Issue

The `ShooterModel.pkl` file appears to have compatibility issues. The error `STACK_GLOBAL requires str` suggests:
1. The file may have been created with an older Python version
2. The file might be corrupted
3. There may be a version mismatch between pickle protocols

## Solution

### Option 1: Re-export the Model (Recommended)

If you have access to the original Python code that created `ShooterModel.pkl`:

1. **Re-save the model with compatibility mode:**
   ```python
   import pickle
   from sklearn.pipeline import Pipeline
   # ... your model training code ...

   # Save with protocol 4 (Python 3.4+) with explicit encoding
   with open('ShooterModel.pkl', 'wb') as f:
       pickle.dump(model, f, protocol=4)
   ```

2. **Then run the export script:**
   ```bash
   python3 load_and_export_model.py
   ```

   This will create `TeamCode/src/main/assets/ShooterModelParams.bin`

3. **Use in Java:**
   ```java
   ShooterMLPredictor predictor = new ShooterMLPredictor(context, "ShooterModelParams.bin");
   ShooterMLPredictor.Prediction pred = predictor.predict(xPixel, yPixel);
   System.out.println("RPS: " + pred.rps + ", Hood: " + pred.hood);
   ```

### Option 2: Export Model Parameters Manually

If you can load the model in Python, export just the parameters:

```python
import pickle
import json
import numpy as np

# Load model
with open('ShooterModel.pkl', 'rb') as f:
    model = pickle.load(f)

# Test what kind of model it is
print(f"Model type: {type(model)}")

# If it's a KNN model, export training data
if hasattr(model, '_fit_X'):
    X = model._fit_X
    y = model._y
    k = model.n_neighbors if hasattr(model, 'n_neighbors') else 5

    # Save as JSON for inspection
    data = {
        'model_type': 'KNN',
        'k': int(k),
        'training_data': [
            {
                'x_pixel': float(X[i, 0]),
                'y_pixel': float(X[i, 1]),
                'rps': float(y[i, 0]) if y.ndim > 1 else float(y[i]),
                'hood': float(y[i, 1]) if y.ndim > 1 and y.shape[1] > 1 else 0.0
            }
            for i in range(len(X))
        ]
    }

    with open('model_params.json', 'w') as f:
        json.dump(data, f, indent=2)

    print(f"Exported {len(X)} training samples to model_params.json")
```

### Option 3: Use Existing LUT Files

The codebase already has working LUT files:
- `CappedShooterLUT.bin`
- `ShooterLUT2.bin`

These use the `ShooterLutPredictor` class which is already working. You can continue using those until the model export is fixed.

## Files Created

1. **Java Implementation:**
   - `TeamCode/src/main/java/com/kalipsorobotics/modules/shooter/ShooterMLPredictor.java`
     - Loads ML model parameters from binary file
     - Supports KNN and Linear Regression
     - Includes StandardScaler support for feature normalization
     - Same API as ShooterLutPredictor

2. **Python Export Scripts:**
   - `export_shooter_model.py` - Basic export script
   - `load_and_export_model.py` - Enhanced export with better error handling and scaler support

## Model Types Supported

### Currently Implemented:
- **KNeighborsRegressor** - K-Nearest Neighbors with inverse distance weighting
- **LinearRegression** - Linear regression with optional feature scaling
- **MultiOutputRegressor** - Wrapper for models that predict multiple outputs (RPS and Hood)
- **Pipeline** - Supports StandardScaler preprocessing

### Not Yet Implemented:
- RandomForestRegressor
- GradientBoostingRegressor
- Other complex ensemble methods

## Testing

Once you have a working `ShooterModelParams.bin` file, you can test it with:

```java
// In your test file
@Test
public void testMLPredictor() throws Exception {
    String assetsPath = "src/main/assets/ShooterModelParams.bin";
    java.io.File file = new java.io.File(assetsPath);
    java.io.InputStream is = new java.io.FileInputStream(file);
    ShooterMLPredictor predictor = new ShooterMLPredictor(is);
    is.close();

    ShooterMLPredictor.Prediction pred = predictor.predict(200, 200);
    System.out.println("Prediction: " + pred);

    assertTrue("RPS should be positive", pred.rps > 0);
    assertTrue("Hood should be in range", pred.hood >= 0.23 && pred.hood <= 0.8);
}
```

## Next Steps

1. **Fix the pickle file** - Re-export from the original Python training code
2. **Run the export script** - `python3 load_and_export_model.py`
3. **Test in Java** - Use ShooterMLPredictor with the new binary file
4. **Compare results** - Make sure predictions match the original model

## Questions?

- What kind of model is in ShooterModel.pkl? (KNN, Linear, RandomForest, etc.)
- Do you have the original Python code that created it?
- What version of Python and scikit-learn was used to create it?
