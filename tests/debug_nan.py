import sys
sys.path.append('src')

import torch
import numpy as np
from data_generator.phase2_dataset import Phase2DatasetLoader
from models.nn_v0 import NN_v0

def debug_nan():
    """Diagnose NaN issue in training"""
    
    print("="*60)
    print("DEBUGGING NaN LOSS")
    print("="*60)
    
    # 1. Check dataset for NaN/Inf
    print("\n1. Checking dataset...")
    dataset = Phase2DatasetLoader("data/phase2_processed/phase2_deltas_v1.h5")
    
    print(f"  X shape: {dataset.X.shape}")
    print(f"  X has NaN: {np.isnan(dataset.X).any()}")
    print(f"  X has Inf: {np.isinf(dataset.X).any()}")
    
    # Check each delta field
    for key in dataset.Y_delta:
        for subkey in dataset.Y_delta[key]:
            data = dataset.Y_delta[key][subkey]
            has_nan = np.isnan(data).any()
            has_inf = np.isinf(data).any()
            print(f"  Y_delta[{key}][{subkey}]: NaN={has_nan}, Inf={has_inf}")
            if has_nan or has_inf:
                print(f"    Min: {np.nanmin(data)}, Max: {np.nanmax(data)}")
    
    # 2. Test model forward pass
    print("\n2. Testing model forward pass...")
    model = NN_v0(input_dim=9)
    
    # Get one batch
    indices = np.array([0])
    X_batch, Y_batch = dataset.get_batch(indices)
    
    print(f"  Input X: {X_batch[0]}")
    print(f"  Input has NaN: {torch.isnan(X_batch).any()}")
    
    # Forward pass
    with torch.no_grad():
        pred = model(X_batch)
    
    print(f"\n  Predictions:")
    for key in pred:
        for subkey in pred[key]:
            val = pred[key][subkey]
            print(f"    {key}.{subkey}: {val}, has_nan={torch.isnan(val).any()}")
    
    # 3. Test loss computation
    print("\n3. Testing loss computation...")
    
    # Simple MAE loss
    try:
        loss = torch.nn.L1Loss()(pred['weight']['A'], Y_batch['weight']['A'])
        print(f"  Weight A loss: {loss.item()}")
    except Exception as e:
        print(f"  ERROR in loss: {e}")
    
    # 4. Check delta statistics
    print("\n4. Delta statistics:")
    weight_deltas = dataset.Y_delta['weight']['A']
    print(f"  Weight deltas - Min: {weight_deltas.min():.4f}, Max: {weight_deltas.max():.4f}")
    print(f"  Weight deltas - Mean: {weight_deltas.mean():.4f}, Std: {weight_deltas.std():.4f}")
    
    impact_fall_deltas = dataset.Y_delta['impact']['fall_ms']
    print(f"  Impact fall deltas - Min: {impact_fall_deltas.min():.4f}, Max: {impact_fall_deltas.max():.4f}")
    
    print("\n" + "="*60)

if __name__ == "__main__":
    debug_nan()