"""
Training script for NN_v1 (Phase 2 delta model)
"""

import sys
sys.path.append('src')

import torch
import torch.nn as nn
import torch.optim as optim
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from models.nn_v0 import NN_v0  # Reuse same architecture!
from data_generator.phase2_dataset import Phase2DatasetLoader

class DeltaTrainer:
    """Trainer for NN_v1 delta prediction"""
    
    def __init__(self, model, dataset_loader, device='cpu'):
        # The 'model' argument is now ignored, as the model is initialized internally
        from models.nn_v0 import NN_v1_Delta
        self.model = NN_v1_Delta(input_dim=13).to(device)
        print(f"Initialized NN_v1_Delta on {device}")
        self.dataset = dataset_loader
        self.device = device
        
        # Training config
        self.batch_size = 16
        self.learning_rate = 1e-3
        self.optimizer = optim.Adam(self.model.parameters(), lr=self.learning_rate)
        
        # Loss tracking
        self.train_losses = []
        self.val_losses = []
        
    def compute_loss(self, pred_delta, target_delta):
        """
        Compute MAE loss for delta predictions.
        Deltas can be positive or negative, so MAE works well.
        """
        losses = {}
        
        # Impact delta loss
        impact_loss = (
            nn.L1Loss()(pred_delta['impact']['A'], target_delta['impact']['A']) +
            nn.L1Loss()(pred_delta['impact']['rise_ms'], target_delta['impact']['rise_ms']) +
            nn.L1Loss()(pred_delta['impact']['fall_ms'], target_delta['impact']['fall_ms']) +
            nn.L1Loss()(pred_delta['impact']['hf_weight'], target_delta['impact']['hf_weight'])
        ) / 4
        losses['impact'] = impact_loss
        
        # Weight delta loss (IMPORTANT - this has the biggest deltas)
        weight_loss = (
            nn.L1Loss()(pred_delta['weight']['A'], target_delta['weight']['A']) * 3.0 +  # Extra weight!
            nn.L1Loss()(pred_delta['weight']['rate_ms'], target_delta['weight']['rate_ms'])
        ) / 4
        losses['weight'] = weight_loss
        
        # Shear delta loss
        shear_loss = nn.L1Loss()(pred_delta['shear']['A'], target_delta['shear']['A'])
        losses['shear'] = shear_loss
        
        # Texture delta loss
        texture_loss = nn.L1Loss()(pred_delta['texture']['A'], target_delta['texture']['A'])
        losses['texture'] = texture_loss
        
        # Total loss (weight the important ones)
        total_loss = (
            losses['impact'] * 2.0 +
            losses['weight'] * 3.0 +    # Weight is critical!
            losses['shear'] * 0.5 +
            losses['texture'] * 0.5
        ) / 6.0
        
        losses['total'] = total_loss
        return losses
    
    def train_epoch(self, train_indices):
        """Train for one epoch"""
        self.model.train()
        
        # Shuffle indices
        np.random.shuffle(train_indices)
        
        epoch_losses = []
        n_batches = len(train_indices) // self.batch_size
        
        for i in range(n_batches):
            # Get batch
            batch_indices = train_indices[i*self.batch_size:(i+1)*self.batch_size]
            X_batch, Y_delta_batch = self.dataset.get_batch(batch_indices)
            X_batch = X_batch.to(self.device)
            
            # Move targets to device
            for key in Y_delta_batch:
                for subkey in Y_delta_batch[key]:
                    Y_delta_batch[key][subkey] = Y_delta_batch[key][subkey].to(self.device)
            
            # Forward pass (predicting deltas)
            self.optimizer.zero_grad()
            pred_delta = self.model(X_batch)
            
            # Compute loss
            losses = self.compute_loss(pred_delta, Y_delta_batch)
            
            # Backward pass
            losses['total'].backward()
            self.optimizer.step()
            
            epoch_losses.append(losses['total'].item())
        
        return np.mean(epoch_losses)
    
    def validate(self, val_indices):
        """Validate on validation set"""
        self.model.eval()
        
        val_losses = []
        n_batches = len(val_indices) // self.batch_size
        
        with torch.no_grad():
            for i in range(n_batches):
                batch_indices = val_indices[i*self.batch_size:(i+1)*self.batch_size]
                X_batch, Y_delta_batch = self.dataset.get_batch(batch_indices)
                X_batch = X_batch.to(self.device)
                
                for key in Y_delta_batch:
                    for subkey in Y_delta_batch[key]:
                        Y_delta_batch[key][subkey] = Y_delta_batch[key][subkey].to(self.device)
                
                pred_delta = self.model(X_batch)
                losses = self.compute_loss(pred_delta, Y_delta_batch)
                val_losses.append(losses['total'].item())
        
        return np.mean(val_losses)
    
    def train(self, n_epochs: int, save_path: str = "models/checkpoints"):
        """Full training loop"""
        print(f"\n{'='*60}")
        print(f"TRAINING NN_v1 (Phase 2 Delta Model)")
        print(f"{'='*60}")
        print(f"Model parameters: {self.model.count_parameters():,}")
        print(f"Training samples: {len(self.dataset.X)}")
        print(f"Batch size: {self.batch_size}")
        print(f"Learning rate: {self.learning_rate}")
        print(f"Epochs: {n_epochs}")
        print(f"{'='*60}\n")
        
        # Split data
        train_indices, val_indices = self.dataset.get_train_val_split(val_split=0.2)
        print(f"Train: {len(train_indices)} samples")
        print(f"Val: {len(val_indices)} samples\n")
        
        # Training loop
        best_val_loss = float('inf')
        
        for epoch in range(n_epochs):
            # Train
            train_loss = self.train_epoch(train_indices)
            self.train_losses.append(train_loss)
            
            # Validate
            val_loss = self.validate(val_indices)
            self.val_losses.append(val_loss)
            
            # Print progress
            print(f"Epoch {epoch+1}/{n_epochs}: "
                  f"Train Loss={train_loss:.4f}, Val Loss={val_loss:.4f}")
            
            # Save best model
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                Path(save_path).mkdir(parents=True, exist_ok=True)
                torch.save(self.model.state_dict(), f"{save_path}/nn_v1_best.pt")
                print(f"  → Saved best model (val_loss={val_loss:.4f})")
        
        print(f"\n{'='*60}")
        print(f"TRAINING COMPLETE!")
        print(f"Best validation loss: {best_val_loss:.4f}")
        print(f"{'='*60}\n")
        
        # Plot training curves
        self.plot_training_curves(save_path)
    
    def plot_training_curves(self, save_path):
        """Plot and save training curves"""
        plt.figure(figsize=(10, 5))
        plt.plot(self.train_losses, label='Train Loss', linewidth=2)
        plt.plot(self.val_losses, label='Val Loss', linewidth=2)
        plt.xlabel('Epoch')
        plt.ylabel('Loss (MAE)')
        plt.title('NN_v1 Delta Training Curves')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.savefig(f"{save_path}/training_curves_v1.png", dpi=150)
        print(f"✓ Saved training curves to: {save_path}/training_curves_v1.png")
        plt.close()


def main():
    """Main training script"""
    # Load Phase 2 dataset
    print("Loading Phase 2 delta dataset...")
    dataset = Phase2DatasetLoader("data/phase2_processed/phase2_deltas_v1.h5")
    print(f"✓ Loaded {dataset.n_samples} delta samples\n")
    
    # Create NN_v1 (same architecture as NN_v0, but will predict deltas)
    from models.nn_v0 import NN_v1_Delta
    model_v1 = NN_v1_Delta(input_dim=9)
    
    # Create trainer
    trainer = DeltaTrainer(model_v1, dataset, device='cpu')
    
    # Train (deltas are easier to learn, so fewer epochs needed)
    trainer.train(n_epochs=30, save_path="models/checkpoints")

if __name__ == "__main__":
    main()