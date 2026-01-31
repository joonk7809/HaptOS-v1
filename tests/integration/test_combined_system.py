import sys
sys.path.append('src')

import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from inference.combined_predictor import CombinedPredictor

def test_combined_system():
    """
    Test the complete Phase 2 system:
    Compare NN_v0 alone vs. (NN_v0 + NN_v1) vs. expert gold standard
    """
    
    print("\n" + "="*60)
    print("PHASE 2 COMBINED SYSTEM VALIDATION")
    print("="*60 + "\n")
    
    # Load combined predictor
    predictor = CombinedPredictor()
    
    # Load expert tunings for comparison
    print("\nLoading expert gold standards...")
    with open("data/phase2_tunings/synthetic_expert_001_tunings.json", 'r') as f:
        tunings = json.load(f)
    
    print(f"Loaded {len(tunings)} expert tunings\n")
    
    # Test on subset
    n_test = min(50, len(tunings))
    test_samples = np.random.choice(len(tunings), n_test, replace=False)
    
    # Collect results
    results = {
        'baseline_impact_A': [],
        'delta_impact_A': [],
        'final_impact_A': [],
        'gold_impact_A': [],
        
        'baseline_weight_A': [],
        'delta_weight_A': [],
        'final_weight_A': [],
        'gold_weight_A': [],
        
        'baseline_fall_ms': [],
        'delta_fall_ms': [],
        'final_fall_ms': [],
        'gold_fall_ms': []
    }
    
    print("Running predictions...")
    for idx in test_samples:
        tuning = tunings[idx]
        feature_vec = tuning['feature_vec']
        
        # Get predictions with components
        baseline, delta, final = predictor.predict(
            feature_vec, 
            return_components=True
        )
        
        # Gold standard
        gold = tuning['gold_cues']
        
        # Store results
        results['baseline_impact_A'].append(baseline['impact']['A'])
        results['delta_impact_A'].append(delta['impact']['A'])
        results['final_impact_A'].append(final['impact']['A'])
        results['gold_impact_A'].append(gold['impact']['A'])
        
        results['baseline_weight_A'].append(baseline['weight']['A'])
        results['delta_weight_A'].append(delta['weight']['A'])
        results['final_weight_A'].append(final['weight']['A'])
        results['gold_weight_A'].append(gold['weight']['A'])
        
        results['baseline_fall_ms'].append(baseline['impact']['fall_ms'])
        results['delta_fall_ms'].append(delta['impact']['fall_ms'])
        results['final_fall_ms'].append(final['impact']['fall_ms'])
        results['gold_fall_ms'].append(gold['impact']['fall_ms'])
    
    # Compute errors
    print("\n" + "="*60)
    print("COMPARISON: NN_v0 vs (NN_v0 + NN_v1) vs Expert Gold")
    print("="*60)
    
    # Impact amplitude
    baseline_error_impact = np.mean(np.abs(
        np.array(results['baseline_impact_A']) - np.array(results['gold_impact_A'])
    ))
    final_error_impact = np.mean(np.abs(
        np.array(results['final_impact_A']) - np.array(results['gold_impact_A'])
    ))
    
    print(f"\nImpact Amplitude MAE:")
    print(f"  NN_v0 alone:      {baseline_error_impact:.4f}")
    print(f"  NN_v0 + NN_v1:    {final_error_impact:.4f}")
    improvement_impact = ((baseline_error_impact - final_error_impact) / baseline_error_impact * 100)
    if improvement_impact > 0:
        print(f"  Improvement:      {improvement_impact:.1f}% ✓")
    else:
        print(f"  Change:           {improvement_impact:.1f}%")
    
    # Weight amplitude (this should show biggest improvement)
    baseline_error_weight = np.mean(np.abs(
        np.array(results['baseline_weight_A']) - np.array(results['gold_weight_A'])
    ))
    final_error_weight = np.mean(np.abs(
        np.array(results['final_weight_A']) - np.array(results['gold_weight_A'])
    ))
    
    print(f"\nWeight Amplitude MAE (Weber's Law):")
    print(f"  NN_v0 alone:      {baseline_error_weight:.4f}")
    print(f"  NN_v0 + NN_v1:    {final_error_weight:.4f}")
    improvement_weight = ((baseline_error_weight - final_error_weight) / baseline_error_weight * 100)
    if improvement_weight > 0:
        print(f"  Improvement:      {improvement_weight:.1f}% ✓")
    else:
        print(f"  Change:           {improvement_weight:.1f}%")
    
    # Fall time
    baseline_error_fall = np.mean(np.abs(
        np.array(results['baseline_fall_ms']) - np.array(results['gold_fall_ms'])
    ))
    final_error_fall = np.mean(np.abs(
        np.array(results['final_fall_ms']) - np.array(results['gold_fall_ms'])
    ))
    
    print(f"\nImpact Fall Time MAE:")
    print(f"  NN_v0 alone:      {baseline_error_fall:.4f} ms")
    print(f"  NN_v0 + NN_v1:    {final_error_fall:.4f} ms")
    improvement_fall = ((baseline_error_fall - final_error_fall) / baseline_error_fall * 100)
    if improvement_fall > 0:
        print(f"  Improvement:      {improvement_fall:.1f}% ✓")
    else:
        print(f"  Change:           {improvement_fall:.1f}%")
    
    print("="*60 + "\n")
    
    # Visualize
    visualize_improvements(results)
    
    # Summary
    avg_improvement = (improvement_impact + improvement_weight + improvement_fall) / 3
    
    print("\n" + "="*60)
    print("PHASE 2 SUMMARY")
    print("="*60)
    print(f"Average improvement: {avg_improvement:.1f}%")
    
    if avg_improvement > 20:
        print("Status: ✓ Phase 2 delta learning is WORKING!")
        print("        Expert refinements successfully learned.")
    elif avg_improvement > 0:
        print("Status: ⚠ Phase 2 shows modest improvement")
        print("        May need more training data or tuning.")
    else:
        print("Status: ✗ Phase 2 not improving predictions")
        print("        Need to debug delta learning.")
    
    print("="*60 + "\n")
    
    print("✓ Check logs/phase2_validation/ for detailed plots")

def visualize_improvements(results):
    """Create visualization comparing baseline vs final"""
    
    Path("logs/phase2_validation").mkdir(parents=True, exist_ok=True)
    
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    # Impact amplitude
    ax = axes[0]
    gold = np.array(results['gold_impact_A'])
    baseline = np.array(results['baseline_impact_A'])
    final = np.array(results['final_impact_A'])
    
    ax.scatter(gold, baseline, alpha=0.6, label='NN_v0 alone', s=50, color='blue')
    ax.scatter(gold, final, alpha=0.6, label='NN_v0 + NN_v1', s=50, color='green')
    ax.plot([0, 1], [0, 1], 'r--', label='Perfect', linewidth=2)
    ax.set_xlabel('Expert Gold Standard')
    ax.set_ylabel('Prediction')
    ax.set_title('Impact Amplitude')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Weight amplitude
    ax = axes[1]
    gold = np.array(results['gold_weight_A'])
    baseline = np.array(results['baseline_weight_A'])
    final = np.array(results['final_weight_A'])
    
    ax.scatter(gold, baseline, alpha=0.6, label='NN_v0 alone', s=50, color='blue')
    ax.scatter(gold, final, alpha=0.6, label='NN_v0 + NN_v1', s=50, color='green')
    ax.plot([0, 1], [0, 1], 'r--', label='Perfect', linewidth=2)
    ax.set_xlabel('Expert Gold Standard')
    ax.set_ylabel('Prediction')
    ax.set_title('Weight Amplitude (Weber\'s Law)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Fall time
    ax = axes[2]
    gold = np.array(results['gold_fall_ms'])
    baseline = np.array(results['baseline_fall_ms'])
    final = np.array(results['final_fall_ms'])
    
    ax.scatter(gold, baseline, alpha=0.6, label='NN_v0 alone', s=50, color='blue')
    ax.scatter(gold, final, alpha=0.6, label='NN_v0 + NN_v1', s=50, color='green')
    min_val = min(gold.min(), baseline.min(), final.min())
    max_val = max(gold.max(), baseline.max(), final.max())
    ax.plot([min_val, max_val], [min_val, max_val], 'r--', label='Perfect', linewidth=2)
    ax.set_xlabel('Expert Gold Standard (ms)')
    ax.set_ylabel('Prediction (ms)')
    ax.set_title('Impact Fall Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('logs/phase2_validation/combined_comparison.png', dpi=150)
    print(f"\n✓ Saved comparison plot: logs/phase2_validation/combined_comparison.png")
    plt.close()

if __name__ == "__main__":
    test_combined_system()