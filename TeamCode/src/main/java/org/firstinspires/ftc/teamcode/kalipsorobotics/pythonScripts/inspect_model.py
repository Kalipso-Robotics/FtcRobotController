import pickle

# Load the pickle file
with open('TeamCode/src/main/assets/ShooterModel.pkl', 'rb') as f:
    model = pickle.load(f)

# Inspect the model
print("Model type:", type(model))
print("Model class name:", type(model).__name__)
print("Model module:", type(model).__module__)

# Check if it's a dict or has attributes
if hasattr(model, '__dict__'):
    print("\nModel attributes:")
    for key, value in model.__dict__.items():
        print(f"  {key}: {type(value)}")
        value_type = type(value).__name__
        if 'array' in value_type.lower() or 'ndarray' in value_type.lower():
            try:
                print(f"    Shape: {value.shape}")
                print(f"    First few values: {value.flat[:5]}")
            except:
                print(f"    Value: {value}")

# If it's a dict
if isinstance(model, dict):
    print("\nModel is a dictionary with keys:")
    for key, value in model.items():
        print(f"  {key}: {type(value)}")
        value_type = type(value).__name__
        if 'array' in value_type.lower() or 'ndarray' in value_type.lower():
            try:
                print(f"    Shape: {value.shape}")
            except:
                print(f"    Value: {value}")

# Try to get model methods/functions
print("\nModel methods:")
for attr in dir(model):
    if not attr.startswith('_'):
        print(f"  {attr}")

if hasattr(model, 'predict'):
    print("\n✓ Model has predict method")