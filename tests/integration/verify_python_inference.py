import sys
import os
sys.path.append(os.getcwd() + "/src")

import torch
import numpy as np
from models.nn_v0 import NN_v0

def verify_python_model():
    print("Running Twin Check (Python Side)...")
    
    # Load the EXACT model used for export
    model = NN_v0()
    checkpoint = torch.load("tests/test_model.pt")
    model.load_state_dict(checkpoint)
    model.eval()
    
    # Create deterministic input (13 dims)
    # [0.1, 0.2, ..., 1.3]
    test_input_np = np.array([0.1 * (i+1) for i in range(13)], dtype=np.float32)
    test_input = torch.from_numpy(test_input_np).unsqueeze(0) # Batch size 1
    
    with torch.no_grad():
        prediction = model(test_input)
        
    # We'll compare specific outputs
    # Impact A, Ring f_Hz[0], Shear A
    impact_A = prediction['impact']['A'].item()
    ring_f0 = prediction['ring']['f_Hz'][0, 0].item()
    shear_A = prediction['shear']['A'].item()
    
    print(f"PYTHON_IMPACT_A: {impact_A:.6f}")
    print(f"PYTHON_RING_F0: {ring_f0:.6f}")
    print(f"PYTHON_SHEAR_A: {shear_A:.6f}")
    
    # Save to file for C++ comparison script to read (optional, or just read stdout)
    with open("tests/python_truth.txt", "w") as f:
        f.write(f"{impact_A:.6f}\n")
        f.write(f"{ring_f0:.6f}\n")
        f.write(f"{shear_A:.6f}\n")

if __name__ == "__main__":
    verify_python_model()
