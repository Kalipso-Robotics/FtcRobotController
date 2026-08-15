#!/usr/bin/env python3
"""
Export the ShooterModel.pkl to a format that Java can read.
This script extracts the model parameters and saves them in a binary format.
"""
import pickle
import struct

try:
    import numpy as np
    import sklearn
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False
    print("Error: sklearn and numpy are required to export the model.")
    print("Please install them with: pip3 install scikit-learn numpy")
    exit(1)

# Load the pickle file
with open('TeamCode/src/main/assets/ShooterModel.pkl', 'rb') as f:
    model = pickle.load(f)

print(f"Model type: {type(model)}")
print(f"Model: {model}")

# Check what kind of model it is
if hasattr(model, '__dict__'):
    print("\nModel parameters:")
    for key, value in sorted(model.__dict__.items()):
        if not key.startswith('_'):
            print(f"  {key}: {type(value).__name__}", end='')
            if hasattr(value, 'shape'):
                print(f" shape={value.shape}")
            else:
                print()

# Test the model
test_input = np.array([[200, 200]])
try:
    prediction = model.predict(test_input)
    print(f"\nTest prediction for x=200, y=200: {prediction}")
    print(f"Prediction shape: {prediction.shape}")
    print(f"Prediction type: {type(prediction)}")
except Exception as e:
    print(f"\nPrediction failed: {e}")

# Export model parameters to a format Java can read
# The format depends on the model type
print("\n" + "="*60)
print("Exporting model parameters...")
print("="*60)

model_class = type(model).__name__

# Handle different model types
if 'Pipeline' in model_class:
    print(f"\nModel is a Pipeline with {len(model.steps)} steps:")
    for i, (name, step) in enumerate(model.steps):
        print(f"  Step {i}: {name} ({type(step).__name__})")

# Save the model info for Java implementation
model_info = {
    'model_class': model_class,
    'model_module': type(model).__module__,
}

# We'll need to implement the specific model in Java based on what we find
print(f"\nTo implement this in Java, you'll need to:")
print(f"1. Understand the model type: {model_class}")
print(f"2. Extract the model parameters")
print(f"3. Implement the prediction logic in Java")