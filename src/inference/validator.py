"""
Validation metrics and evaluation for trained models.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple
import sys
sys.path.append('src')

from models.nn_v0 import DatasetLoader
from inference.predictor import HapticPredictor

class ModelValidator:
    """
    Evaluates model performance on test data.
    Computes metrics and generates visualizations.
    """
    
    def __init__(self, predictor: HapticPredictor, dataset: DatasetLoader):
        self.predictor = predictor
        self.dataset = dataset
    
    def validate(self, n_samples: int = None) -> Dict:
        """
        Run validation on dataset.
        
        Args:
            n_samples: Number of samples to validate (None = all)
            
        Returns:
            Dict of metrics
        """
        if n_samples is None:
            n_samples = min(1000, self.dataset.n_samples)
        
        # Get random validation samples
        indices = np.random.choice(self.dataset.n_samples, n_samples, replace=False)
        
        print(f"\n{'='*60}")
        print(f"VALIDATING MODEL ON {n_samples} SAMPLES")
        print(f"{'='*60}\n")
        
        # Collect predictions and ground truth
        predictions = {
            'impact_A': [], 'impact_rise': [], 'impact_fall': [], 'impact_hf': [],
            'weight_A': [], 'weight_rate': [],
            'shear_A': [], 'shear_band': [],
            'texture_A': []
        }
        
        ground_truth = {
            'impact_A': [], 'impact_rise': [], 'impact_fall': [], 'impact_hf': [],
            'weight_A': [], 'weight_rate': [],
            'shear_A': [], 'shear_band': [],
            'texture_A': []
        }
        
        # Run predictions
        for i, idx in enumerate(indices):
            if i % 100 == 0:
                print(f"  Processing: {i}/{n_samples}", end='\r')
            
            # Get input features
            feature_vec = self._extract_feature_vec(idx)
            
            # Predict
            pred = self.predictor.predict(feature_vec)
            
            # Store predictions
            predictions['impact_A'].append(pred['impact']['A'])
            predictions['impact_rise'].append(pred['impact']['rise_ms'])
            predictions['impact_fall'].append(pred['impact']['fall_ms'])
            predictions['impact_hf'].append(pred['impact']['hf_weight'])
            predictions['weight_A'].append(pred['weight']['A'])
            predictions['weight_rate'].append(pred['weight']['rate_ms'])
            predictions['shear_A'].append(pred['shear']['A'])
            predictions['shear_band'].append(pred['shear']['band_Hz'][0])
            predictions['texture_A'].append(pred['texture']['A'])
            
            # Store ground truth
            ground_truth['impact_A'].append(float(self.dataset.Y['impact']['A'][idx]))
            ground_truth['impact_rise'].append(float(self.dataset.Y['impact']['rise_ms'][idx]))
            ground_truth['impact_fall'].append(float(self.dataset.Y['impact']['fall_ms'][idx]))
            ground_truth['impact_hf'].append(float(self.dataset.Y['impact']['hf_weight'][idx]))
            ground_truth['weight_A'].append(float(self.dataset.Y['weight']['A'][idx]))
            ground_truth['weight_rate'].append(float(self.dataset.Y['weight']['rate_ms'][idx]))
            ground_truth['shear_A'].append(float(self.dataset.Y['shear']['A'][idx]))
            ground_truth['shear_band'].append(float(self.dataset.Y['shear']['band_Hz'][idx]))
            ground_truth['texture_A'].append(float(self.dataset.Y['texture']['A'][idx]))
        
        print(f"  Processing: {n_samples}/{n_samples} ✓")
        
        # Compute metrics
        metrics = self._compute_metrics(predictions, ground_truth)
        
        # Print results
        self._print_metrics(metrics)
        
        return {
            'metrics': metrics,
            'predictions': predictions,
            'ground_truth': ground_truth
        }
    
    def _extract_feature_vec(self, idx: int) -> dict:
        """Extract FeatureVec from dataset at index"""
        phase_map = {0: "PHASE_NO_CONTACT", 1: "PHASE_IMPACT", 
                     2: "PHASE_HOLD", 3: "PHASE_SLIP", 4: "PHASE_RELEASE"}
        
        phase_val = int(self.dataset.X[idx, 0] * 4)  # Denormalize
        
        return {
            'phase': phase_map.get(phase_val, "PHASE_NO_CONTACT"),
            'phase_confidence_01': float(self.dataset.X[idx, 1]),
            'normal_force_N': float(self.dataset.X[idx, 2]),
            'shear_force_N': float(self.dataset.X[idx, 3]),
            'slip_speed_mms': float(self.dataset.X[idx, 4]),
            'material_features': {
                'hardness_01': float(self.dataset.X[idx, 5]),
                'mu_01': float(self.dataset.X[idx, 6]),
                'roughness_rms_um': float(self.dataset.X[idx, 7])
            },
            'uncertainty_pct': float(self.dataset.X[idx, 8])
        }
    
    def _compute_metrics(self, pred: Dict, gt: Dict) -> Dict:
        """Compute MAE and RMSE for each cue parameter"""
        metrics = {}
        
        for key in pred.keys():
            pred_arr = np.array(pred[key])
            gt_arr = np.array(gt[key])
            
            mae = np.mean(np.abs(pred_arr - gt_arr))
            rmse = np.sqrt(np.mean((pred_arr - gt_arr)**2))
            mape = np.mean(np.abs((pred_arr - gt_arr) / (gt_arr + 1e-6))) * 100
            
            metrics[key] = {
                'MAE': mae,
                'RMSE': rmse,
                'MAPE': mape
            }
        
        return metrics
    
    def _print_metrics(self, metrics: Dict):
        """Print metrics in formatted table"""
        print(f"\n{'='*60}")
        print(f"VALIDATION METRICS")
        print(f"{'='*60}")
        print(f"{'Parameter':<20} {'MAE':>12} {'RMSE':>12} {'MAPE':>12}")
        print(f"{'-'*60}")
        
        for key, vals in metrics.items():
            print(f"{key:<20} {vals['MAE']:>12.4f} {vals['RMSE']:>12.4f} {vals['MAPE']:>11.2f}%")
        
        print(f"{'='*60}\n")
    
    def plot_predictions(self, predictions: Dict, ground_truth: Dict, 
                        save_path: str = "logs/validation"):
        """
        Create scatter plots comparing predictions vs ground truth.
        """
        from pathlib import Path
        Path(save_path).mkdir(parents=True, exist_ok=True)
        
        # Select key parameters to visualize
        params_to_plot = [
            ('impact_A', 'Impact Amplitude'),
            ('weight_A', 'Weight Amplitude'),
            ('impact_rise', 'Impact Rise Time (ms)'),
            ('impact_fall', 'Impact Fall Time (ms)'),
            ('shear_A', 'Shear Amplitude'),
            ('texture_A', 'Texture Amplitude')
        ]
        
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        axes = axes.flatten()
        
        for idx, (key, label) in enumerate(params_to_plot):
            ax = axes[idx]
            
            pred = np.array(predictions[key])
            gt = np.array(ground_truth[key])
            
            # Scatter plot
            ax.scatter(gt, pred, alpha=0.5, s=20, edgecolors='k', linewidth=0.5)
            
            # Perfect prediction line
            min_val = min(gt.min(), pred.min())
            max_val = max(gt.max(), pred.max())
            ax.plot([min_val, max_val], [min_val, max_val], 'r--', linewidth=2, label='Perfect')
            
            # Labels
            ax.set_xlabel('Ground Truth')
            ax.set_ylabel('Prediction')
            ax.set_title(label)
            ax.grid(True, alpha=0.3)
            ax.legend()
            
            # Add R² score
            correlation = np.corrcoef(gt, pred)[0, 1]
            r2 = correlation ** 2
            ax.text(0.05, 0.95, f'R²={r2:.3f}', 
                   transform=ax.transAxes, 
                   verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        plt.tight_layout()
        plt.savefig(f"{save_path}/prediction_scatter.png", dpi=150)
        print(f"✓ Saved scatter plots to: {save_path}/prediction_scatter.png")
        plt.close()
    
    def plot_error_distribution(self, predictions: Dict, ground_truth: Dict,
                               save_path: str = "logs/validation"):
        """
        Plot error distributions for each parameter.
        """
        from pathlib import Path
        Path(save_path).mkdir(parents=True, exist_ok=True)
        
        params = ['impact_A', 'weight_A', 'impact_rise', 'shear_A']
        labels = ['Impact Amplitude', 'Weight Amplitude', 'Impact Rise (ms)', 'Shear Amplitude']
        
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        axes = axes.flatten()
        
        for idx, (key, label) in enumerate(zip(params, labels)):
            pred = np.array(predictions[key])
            gt = np.array(ground_truth[key])
            errors = pred - gt
            
            ax = axes[idx]
            ax.hist(errors, bins=50, alpha=0.7, edgecolor='black')
            ax.axvline(0, color='r', linestyle='--', linewidth=2, label='Zero Error')
            ax.set_xlabel('Error (Pred - GT)')
            ax.set_ylabel('Frequency')
            ax.set_title(f'{label} Error Distribution')
            ax.grid(True, alpha=0.3)
            ax.legend()
            
            # Add stats
            mean_err = np.mean(errors)
            std_err = np.std(errors)
            ax.text(0.05, 0.95, f'μ={mean_err:.4f}\nσ={std_err:.4f}', 
                   transform=ax.transAxes,
                   verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        plt.tight_layout()
        plt.savefig(f"{save_path}/error_distribution.png", dpi=150)
        print(f"✓ Saved error distributions to: {save_path}/error_distribution.png")
        plt.close()