#!/usr/bin/env python3
"""
Alternative approach: Load and export ShooterModel.pkl with better error handling.
"""
import pickle
import pickletools
import struct
import sys

try:
    import numpy as np
    from sklearn.pipeline import Pipeline
    from sklearn.preprocessing import StandardScaler
    from sklearn.neighbors import KNeighborsRegressor
    from sklearn.linear_model import LinearRegression
    from sklearn.multioutput import MultiOutputRegressor
except ImportError as e:
    print(f"ERROR: {e}")
    print("Install with: pip3 install --user --upgrade scikit-learn numpy")
    sys.exit(1)

def export_to_binary(model, output_path):
    """Export model to binary format for Java."""

    # Unwrap pipeline if needed
    if isinstance(model, Pipeline):
        print("Model is a Pipeline:")
        for name, step in model.steps:
            print(f"  - {name}: {type(step).__name__}")
        final_model = model.steps[-1][1]
    else:
        final_model = model

    print(f"\nFinal model type: {type(final_model).__name__}")

    # Handle MultiOutputRegressor
    if isinstance(final_model, MultiOutputRegressor):
        print(f"MultiOutputRegressor with {len(final_model.estimators_)} outputs")

        with open(output_path, 'wb') as f:
            # Write header
            f.write(struct.pack('i', 2))  # 2 outputs (RPS and Hood)
            f.write(struct.pack('i', len(final_model.estimators_)))

            for i, estimator in enumerate(final_model.estimators_):
                print(f"\nExporting output {i}: {type(estimator).__name__}")
                export_single_estimator(f, estimator, model if isinstance(model, Pipeline) else None)

    else:
        # Single output model
        with open(output_path, 'wb') as f:
            f.write(struct.pack('i', 1))
            export_single_estimator(f, final_model, model if isinstance(model, Pipeline) else None)

    print(f"\n✓ Model exported to: {output_path}")

def export_single_estimator(f, estimator, pipeline=None):
    """Export a single estimator to file."""

    # Get the scaler if part of pipeline
    scaler = None
    if pipeline and isinstance(pipeline, Pipeline):
        for name, step in pipeline.steps:
            if isinstance(step, StandardScaler):
                scaler = step
                break

    # Determine model type
    model_types = {
        KNeighborsRegressor: 1,
        LinearRegression: 2,
    }

    model_type_id = 0
    for cls, type_id in model_types.items():
        if isinstance(estimator, cls):
            model_type_id = type_id
            break

    f.write(struct.pack('i', model_type_id))

    if isinstance(estimator, KNeighborsRegressor):
        # Export KNN with scaler info
        export_knn_with_scaler(f, estimator, scaler)
    elif isinstance(estimator, LinearRegression):
        export_linear_with_scaler(f, estimator, scaler)
    else:
        raise ValueError(f"Unsupported estimator type: {type(estimator)}")

def export_knn_with_scaler(f, knn, scaler):
    """Export KNN model, optionally with scaler parameters."""

    # Write whether we have a scaler
    has_scaler = scaler is not None
    f.write(struct.pack('?', has_scaler))

    if has_scaler:
        # Write scaler parameters
        mean = scaler.mean_
        scale = scaler.scale_

        f.write(struct.pack('i', len(mean)))
        for val in mean:
            f.write(struct.pack('d', float(val)))

        f.write(struct.pack('i', len(scale)))
        for val in scale:
            f.write(struct.pack('d', float(val)))

        print(f"  Scaler: mean={mean}, scale={scale}")

    # Write KNN parameters
    k = knn.n_neighbors
    X = knn._fit_X
    y = knn._y

    f.write(struct.pack('i', k))
    f.write(struct.pack('i', X.shape[0]))
    f.write(struct.pack('i', X.shape[1]))

    print(f"  KNN: k={k}, samples={X.shape[0]}, features={X.shape[1]}")

    # Write training data
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            f.write(struct.pack('d', float(X[i, j])))
        f.write(struct.pack('d', float(y[i])))

def export_linear_with_scaler(f, linear, scaler):
    """Export Linear Regression model, optionally with scaler."""

    # Write whether we have a scaler
    has_scaler = scaler is not None
    f.write(struct.pack('?', has_scaler))

    if has_scaler:
        mean = scaler.mean_
        scale = scaler.scale_

        f.write(struct.pack('i', len(mean)))
        for val in mean:
            f.write(struct.pack('d', float(val)))

        f.write(struct.pack('i', len(scale)))
        for val in scale:
            f.write(struct.pack('d', float(val)))

    # Write linear regression parameters
    coef = linear.coef_
    intercept = linear.intercept_

    f.write(struct.pack('i', len(coef)))
    for val in coef:
        f.write(struct.pack('d', float(val)))

    f.write(struct.pack('d', float(intercept)))

    print(f"  Linear: coef={coef}, intercept={intercept}")

def main():
    model_path = 'TeamCode/src/main/assets/ShooterModel.pkl'
    output_path = 'TeamCode/src/main/assets/ShooterModelParams.bin'

    print("Loading ShooterModel.pkl...")

    try:
        with open(model_path, 'rb') as f:
            model = pickle.load(f)
    except Exception as e:
        print(f"ERROR loading model: {e}")
        print("\nTrying alternative loading methods...")

        try:
            import sys
            sys.modules['sklearn.externals.joblib'] = __import__('joblib')
            with open(model_path, 'rb') as f:
                model = pickle.load(f)
        except Exception as e2:
            print(f"Alternative loading also failed: {e2}")
            sys.exit(1)

    print(f"✓ Model loaded: {type(model).__name__}")

    # Test the model
    print("\nTesting model predictions...")
    test_points = [
        [200, 200],
        [500, 200],
        [750, 200],
    ]

    for point in test_points:
        test_input = np.array([point])
        prediction = model.predict(test_input)
        print(f"  Input {point}: Output {prediction[0]}")

    # Export
    print("\nExporting model to binary format...")
    export_to_binary(model, output_path)

    print(f"\n✓ Complete! Use ShooterMLPredictor.java to load '{output_path}'")

if __name__ == "__main__":
    main()
