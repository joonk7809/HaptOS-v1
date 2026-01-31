import sys
sys.path.append('src')

from tuning.synthetic_expert import SyntheticExpert, PerceptualMetrics
from models.nn_v0 import DatasetLoader
from inference.predictor import HapticPredictor
import json
from pathlib import Path
import numpy as np

def test_synthetic_expert():
    """
    Test synthetic expert tuning on dataset samples.
    Generates Phase 2 training data automatically.
    """
    
    print("\n" + "="*60)
    print("SYNTHETIC EXPERT TUNING - Phase 2 Data Generation")
    print("="*60 + "\n")
    
    # Initialize
    predictor = HapticPredictor("models/checkpoints/nn_v0_best.pt")
    expert = SyntheticExpert(expert_id="synthetic_001")
    dataset = DatasetLoader("data/processed/phase1_test.h5")
    
    print(expert.get_tuning_style_description())
    
    # Generate tunings
    n_samples = 500
    print(f"\nGenerating {n_samples} synthetic expert tunings...")
    
    indices = np.random.choice(dataset.n_samples, n_samples, replace=False)
    
    tunings = []
    
    for i, idx in enumerate(indices):
        if i % 20 == 0:
            print(f"  Progress: {i}/{n_samples}", end='\r')
        
        # Extract features
        feature_vec = extract_feature_vec(dataset, idx)
        
        # Get baseline prediction
        baseline = predictor.predict(feature_vec)
        
        # Apply expert tuning
        gold = expert.tune(baseline, feature_vec)
        
        # Compute delta
        delta = compute_delta(baseline, gold)
        
        # Save tuning
        tuning = {
            'scenario_id': f"scenario_{idx}",
            'expert_id': expert.expert_id,
            'feature_vec': feature_vec,
            'baseline_cues': baseline,
            'gold_cues': gold,
            'delta_cues': delta
        }
        tunings.append(tuning)
    
    print(f"  Progress: {n_samples}/{n_samples} ✓")
    
    # Analyze tuning patterns
    print("\nAnalyzing tuning patterns...")
    analysis = PerceptualMetrics.analyze_tuning_patterns(tunings)
    
    print(f"\nTuning Statistics:")
    print(f"  Avg Impact Rise Change: {analysis['avg_impact_rise_change']:.1f}%")
    print(f"  Avg Weight Change: {analysis['avg_weight_change']:.1f}%")
    
    # Save to JSON
    output_dir = Path("data/phase2_tunings")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Custom JSON encoder for numpy types
    class NumpyEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            if isinstance(obj, np.float32):
                return float(obj)
            if isinstance(obj, np.int64):
                return int(obj)
            return super(NumpyEncoder, self).default(obj)
            
    output_file = output_dir / "synthetic_expert_001_tunings.json"
    with open(output_file, 'w') as f:
        json.dump(tunings, f, indent=2, cls=NumpyEncoder)
    
    print(f"\n✓ Saved {len(tunings)} tunings to: {output_file}")
    
    # Show examples
    print("\n" + "="*60)
    print("EXAMPLE TUNINGS")
    print("="*60)
    
    for i in range(min(3, len(tunings))):
        t = tunings[i]
        print(f"\nExample {i+1}:")
        print(f"  Force: {t['feature_vec']['normal_force_N']:.2f}N")
        print(f"  Baseline impact.rise: {t['baseline_cues']['impact']['rise_ms']:.2f}ms")
        print(f"  Gold impact.rise: {t['gold_cues']['impact']['rise_ms']:.2f}ms")
        print(f"  Delta: {t['delta_cues']['impact']['rise_ms']:.2f}ms")
    
    print("\n" + "="*60)
    print("READY FOR PHASE 2 TRAINING!")
    print("="*60)
    print("\nNext steps:")
    print("1. Convert JSON tunings to HDF5 dataset")
    print("2. Train NN_v1 on delta predictions")
    print("3. Validate combined NN_v0 + NN_v1 system")

def extract_feature_vec(dataset, idx):
    """Helper to extract feature vec from dataset"""
    # dataset.X is now 13-dim
    # 0-4: phase_one_hot
    # 5: phase_confidence
    # 6: normal_force_log
    # 7: shear_force_log
    # 8: slip_speed_mms
    # 9: hardness
    # 10: mu
    # 11: roughness
    # 12: uncertainty
    
    phase_one_hot = dataset.X[idx, 0:5]
    phase_idx = np.argmax(phase_one_hot)
    phase_map = ["PHASE_NO_CONTACT", "PHASE_IMPACT", "PHASE_HOLD", "PHASE_SLIP", "PHASE_RELEASE"]
    
    # Un-log forces
    normal_force_log = dataset.X[idx, 6]
    shear_force_log = dataset.X[idx, 7]
    normal_force_linear = (10.0 ** normal_force_log) - 1.0
    shear_force_linear = (10.0 ** shear_force_log) - 1.0
    
    return {
        'phase': phase_map[phase_idx],
        'phase_confidence_01': float(dataset.X[idx, 5]),
        'normal_force_N': float(normal_force_linear),
        'shear_force_N': float(shear_force_linear),
        'slip_speed_mms': float(dataset.X[idx, 8]),
        'material_features': {
            'hardness_01': float(dataset.X[idx, 9]),
            'mu_01': float(dataset.X[idx, 10]),
            'roughness_rms_um': float(dataset.X[idx, 11])
        },
        'uncertainty_pct': float(dataset.X[idx, 12])
    }

def compute_delta(baseline, gold):
    """Compute delta = gold - baseline"""
    # Ensure numpy arrays for math
    def to_arr(x): return np.array(x, dtype=np.float32) if isinstance(x, list) else x
    
    return {
        'impact': {
            'A': gold['impact']['A'] - baseline['impact']['A'],
            'rise_ms': gold['impact']['rise_ms'] - baseline['impact']['rise_ms'],
            'fall_ms': gold['impact']['fall_ms'] - baseline['impact']['fall_ms'],
            'hf_weight': gold['impact']['hf_weight'] - baseline['impact']['hf_weight']
        },
        'ring': {
            'tau_ms': to_arr(gold['ring']['tau_ms']) - to_arr(baseline['ring']['tau_ms']),
            'a': to_arr(gold['ring']['a']) - to_arr(baseline['ring']['a'])
        },
        'weight': {
            'A': gold['weight']['A'] - baseline['weight']['A'],
            'rate_ms': gold['weight']['rate_ms'] - baseline['weight']['rate_ms']
        },
        'shear': {
            'A': gold['shear']['A'] - baseline['shear']['A'],
            'band_Hz': gold['shear']['band_Hz']
        },
        'texture': {
            'A': gold['texture']['A'] - baseline['texture']['A'],
            'color': gold['texture']['color']
        }
    }

if __name__ == "__main__":
    test_synthetic_expert()