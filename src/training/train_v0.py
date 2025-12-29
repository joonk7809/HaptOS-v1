"""
Training script for NN_v0 (Phase 1 baseline model)
"""

import sys
sys.path.append('src')

import torch
import torch.nn as nn
import torch.optim as optim
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

from models.nn_v0 import NN_v0, DatasetLoader

class Trainer:
    """Trainer for NN_v0"""
    
    def __init__(self, model, dataset_loader, device='cpu'):
        self.model = model.to(device)
        self.dataset = dataset_loader
        self.device = device
        
        # Training config
        self.batch_size = 32
        self.learning_rate = 1e-3
        self.optimizer = optim.Adam(model.parameters(), lr=self.learning_rate)
        
        # Loss tracking
        self.train_losses = []
        self.val_losses = []
        
    def compute_loss(self, pred, target):
        """
        Compute MAE loss for all cue components.
        Using L1 (MAE) as specified in your design doc.
        """
        losses = {}
        
        # Impact loss
        impact_loss = (
            nn.L1Loss()(pred['impact']['A'], target['impact']['A']) +
            nn.L1Loss()(pred['impact']['rise_ms'], target['impact']['rise_ms']) +
            nn.L1Loss()(pred['impact']['fall_ms'], target['impact']['fall_ms']) +
            nn.L1Loss()(pred['impact']['hf_weight'], target['impact']['hf_weight'])
        ) / 4
        losses['impact'] = impact_loss
        
        # Ring loss (only for valid modes)
        ring_loss = (
            nn.L1Loss()(pred['ring']['f_Hz'], target['ring']['f_Hz']) +
            nn.L1Loss()(pred['ring']['tau_ms'], target['ring']['tau_ms']) +
            nn.L1Loss()(pred['ring']['a'], target['ring']['a'])
        ) / 3
        losses['ring'] = ring_loss
        
        # Shear loss
        shear_loss = (
            nn.L1Loss()(pred['shear']['A'], target['shear']['A']) +
            nn.L1Loss()(pred['shear']['band_Hz'], target['shear']['band_Hz'])
        ) / 2
        losses['shear'] = shear_loss
        
        # Weight loss
        weight_loss = (
            nn.L1Loss()(pred['weight']['A'], target['weight']['A']) +
            nn.L1Loss()(pred['weight']['rate_ms'], target['weight']['rate_ms'])
        ) / 2
        losses['weight'] = weight_loss
        
        # Texture loss
        texture_loss = nn.L1Loss()(pred['texture']['A'], target['texture']['A'])
        # Color is categorical - use cross entropy
        color_target_indices = target['texture']['color']
        color_loss = nn.CrossEntropyLoss()(
            pred['texture']['color'], 
            color_target_indices
        )
        losses['texture'] = (texture_loss + color_loss) / 2
        
        # Total loss (weighted average)
        total_loss = (
            losses['impact'] * 2.0 +    # Impact is important
            losses['ring'] * 1.0 +
            losses['shear'] * 0.5 +
            losses['weight'] * 2.0 +    # Weight is physics-critical
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
            X_batch, Y_batch = self.dataset.get_batch(batch_indices)
            X_batch = X_batch.to(self.device)
            
            # Move targets to device
            for key in Y_batch:
                for subkey in Y_batch[key]:
                    Y_batch[key][subkey] = Y_batch[key][subkey].to(self.device)
            
            # Forward pass
            self.optimizer.zero_grad()
            pred = self.model(X_batch)
            
            # Compute loss
            losses = self.compute_loss(pred, Y_batch)
            
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
                X_batch, Y_batch = self.dataset.get_batch(batch_indices)
                X_batch = X_batch.to(self.device)
                
                for key in Y_batch:
                    for subkey in Y_batch[key]:
                        Y_batch[key][subkey] = Y_batch[key][subkey].to(self.device)
                
                pred = self.model(X_batch)
                losses = self.compute_loss(pred, Y_batch)
                val_losses.append(losses['total'].item())
        
        return np.mean(val_losses)
    
    def train(self, n_epochs: int, save_path: str = "models/checkpoints"):
        """Full training loop"""
        print(f"\n{'='*60}")
        print(f"TRAINING NN_v0 (Phase 1 Baseline)")
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
                torch.save(self.model.state_dict(), f"{save_path}/nn_v0_best.pt")
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
        plt.title('NN_v0 Training Curves')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.savefig(f"{save_path}/training_curves.png", dpi=150)
        print(f"✓ Saved training curves to: {save_path}/training_curves.png")
        plt.close()


def main():
    """Main training script"""
    # Load dataset
    print("Loading dataset...")
    dataset = DatasetLoader("data/processed/phase1_test.h5")
    print(f"✓ Loaded {dataset.n_samples} samples\n")
    
    # Create model
    model = NN_v0(input_dim=13)  # 13 features (One-Hot + Log Forces)
    
    # Create trainer
    trainer = Trainer(model, dataset, device='cpu')
    
    # Train
    trainer.train(n_epochs=50, save_path="models/checkpoints")

if __name__ == "__main__":
    main()