#!/usr/bin/env python3
"""
Export ShooterModel.pkl parameters to a binary format that Java can read.
This script handles various scikit-learn model types and exports their parameters.
"""
import pickle
import struct
import sys

try:
    import numpy as np
except ImportError:
    print("ERROR: numpy is required")
    print("Install with: pip3 install numpy scikit-learn")
    sys.exit(1)

def export_model_params(model, output_path):
    """
    Export model parameters based on model type.
    Supports: KNeighborsRegressor, LinearRegression, Pipeline, etc.
    """
    model_type = type(model).__name__
    print(f"Exporting {model_type} model...")

    # Handle Pipeline
    if model_type == 'Pipeline':
        print(f"Pipeline detected with {len(model.steps)} steps:")
        for name, step in model.steps:
            print(f"  - {name}: {type(step).__name__}")
        # Export the final estimator in the pipeline
        final_estimator = model.steps[-1][1]
        return export_model_params(final_estimator, output_path)

    # Handle MultiOutputRegressor
    elif model_type == 'MultiOutputRegressor':
        print(f"MultiOutputRegressor with {len(model.estimators_)} outputs")
        base_model = model.estimators_[0]
        print(f"  Base estimator: {type(base_model).__name__}")

        # For each output, export its model
        with open(output_path, 'wb') as f:
            # Write header
            f.write(struct.pack('i', 2))  # Number of outputs (RPS and Hood)
            f.write(struct.pack('i', len(model.estimators_)))  # Verify it's 2

            for i, estimator in enumerate(model.estimators_):
                print(f"  Exporting output {i}: {type(estimator).__name__}")
                export_single_model(f, estimator)

        print(f"✓ Exported to {output_path}")
        return True

    # Handle single models
    else:
        with open(output_path, 'wb') as f:
            f.write(struct.pack('i', 1))  # Single output
            export_single_model(f, model)
        print(f"✓ Exported to {output_path}")
        return True

def export_single_model(f, model):
    """Export a single model's parameters to file."""
    model_type = type(model).__name__

    # Write model type ID
    type_ids = {
        'KNeighborsRegressor': 1,
        'LinearRegression': 2,
        'RandomForestRegressor': 3,
        'GradientBoostingRegressor': 4,
    }

    type_id = type_ids.get(model_type, 0)
    f.write(struct.pack('i', type_id))

    if model_type == 'KNeighborsRegressor':
        export_knn(f, model)
    elif model_type == 'LinearRegression':
        export_linear(f, model)
    elif model_type == 'RandomForestRegressor':
        export_random_forest(f, model)
    elif model_type == 'GradientBoostingRegressor':
        export_gradient_boosting(f, model)
    else:
        print(f"WARNING: Unsupported model type {model_type}")
        print(f"Using fallback: will extract training data if available")
        # Try to extract training data (X_fit_, y_fit_)
        if hasattr(model, '_fit_X'):
            export_knn_data(f, model._fit_X, model._y)
        else:
            raise ValueError(f"Cannot export model type: {model_type}")

def export_knn(f, model):
    """Export KNeighborsRegressor parameters."""
    print(f"    KNN with k={model.n_neighbors}, metric={model.metric}")

    # Get training data
    X = model._fit_X
    y = model._y

    # Write parameters
    f.write(struct.pack('i', model.n_neighbors))  # k
    f.write(struct.pack('i', X.shape[0]))  # number of training samples
    f.write(struct.pack('i', X.shape[1]))  # number of features

    # Write training data
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            f.write(struct.pack('d', float(X[i, j])))
        f.write(struct.pack('d', float(y[i])))

    print(f"    Exported {X.shape[0]} training samples with {X.shape[1]} features")

def export_knn_data(f, X, y):
    """Export KNN-style data."""
    f.write(struct.pack('i', 5))  # default k=5
    f.write(struct.pack('i', X.shape[0]))
    f.write(struct.pack('i', X.shape[1]))

    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            f.write(struct.pack('d', float(X[i, j])))
        f.write(struct.pack('d', float(y[i])))

def export_linear(f, model):
    """Export LinearRegression parameters."""
    print(f"    Linear model with {len(model.coef_)} coefficients")

    # Write coefficients
    f.write(struct.pack('i', len(model.coef_)))
    for coef in model.coef_:
        f.write(struct.pack('d', float(coef)))

    # Write intercept
    f.write(struct.pack('d', float(model.intercept_)))

def export_random_forest(f, model):
    """Export RandomForestRegressor (simplified - exports as training data for KNN fallback)."""
    print(f"    WARNING: RandomForest export not fully supported, using training data fallback")
    # This would require exporting the entire tree structure, which is complex
    # Instead, we'll use the training data with KNN as a fallback
    raise NotImplementedError("RandomForest export requires tree serialization")

def export_gradient_boosting(f, model):
    """Export GradientBoostingRegressor (simplified)."""
    print(f"    WARNING: GradientBoosting export not fully supported")
    raise NotImplementedError("GradientBoosting export requires tree serialization")

def main():
    # Load the model
    model_path = 'TeamCode/src/main/assets/ShooterModel.pkl'
    output_path = 'TeamCode/src/main/assets/ShooterModelParams.bin'

    print("Loading model...")
    try:
        with open(model_path, 'rb') as f:
            model = pickle.load(f, encoding='latin1')
    except FileNotFoundError:
        print(f"ERROR: {model_path} not found!")
        sys.exit(1)
    except ModuleNotFoundError as e:
        print(f"ERROR: Missing required module: {e}")
        print("Install with: pip3 install scikit-learn")
        sys.exit(1)

    print(f"Model type: {type(model).__name__}")

    # Test the model
    print("\nTesting model...")
    try:
        test_input = np.array([[200, 200]])
        prediction = model.predict(test_input)
        print(f"Test prediction [200, 200]: {prediction}")
    except Exception as e:
        print(f"Test prediction failed: {e}")

    # Export the model
    print(f"\nExporting model to {output_path}...")
    export_model_params(model, output_path)

    print("\n✓ Model export complete!")
    print(f"\nNow you can use ShooterMLPredictor.java to load {output_path}")

if __name__ == "__main__":
    main()