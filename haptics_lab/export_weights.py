import torch
import numpy as np
from haptic_net import HapticNetV0

def export_to_header(model, filename="weights.h"):
    """
    Extracts weights/biases from PyTorch and writes a C++ header file.
    """
    print(f"Exporting model to {filename}...")
    
    # 1. Get the state dictionary (the raw numbers)
    state_dict = model.state_dict()
    
    with open(filename, "w") as f:
        f.write("#ifndef WEIGHTS_H\n#define WEIGHTS_H\n\n")
        f.write("// Neural Network Weights exported from PyTorch\n")
        f.write("// Architecture: 9 -> 32 -> 32 -> 5\n\n")

        # 2. Iterate through layers and write arrays
        for name, param in state_dict.items():
            # Flatten the tensor to a 1D array
            data = param.detach().numpy().flatten()
            
            # Clean up name (e.g., "fc1.weight" -> "fc1_weight")
            c_name = name.replace(".", "_")
            
            f.write(f"const float {c_name}[] = {{\n")
            
            # Write numbers comma-separated
            for i, val in enumerate(data):
                f.write(f"    {val:.6f}f")
                if i < len(data) - 1:
                    f.write(", ")
                if (i + 1) % 8 == 0: # Newline every 8 numbers for readability
                    f.write("\n")
            
            f.write("\n};\n\n")
            
            # Write the dimension so C++ knows loop size
            f.write(f"const int {c_name}_len = {len(data)};\n\n")

        f.write("#endif // WEIGHTS_H\n")
    
    print("Success! C++ header generated.")

if __name__ == "__main__":
    # Create an untrained model (random weights) just to test the exporter
    model = HapticNetV0()
    model.eval()
    export_to_header(model)