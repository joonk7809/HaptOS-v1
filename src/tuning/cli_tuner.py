"""
CLI-based haptic tuning interface for Phase 2.
Allows expert to adjust NN_v0 predictions and save gold standards.
"""

import sys
sys.path.append('src')

import json
from pathlib import Path
from typing import Dict, List
from dataclasses import dataclass, asdict

from sync_runner import SynchronizedRunner
from converter.converter import Converter
from inference.predictor import HapticPredictor
from models.nn_v0 import DatasetLoader

@dataclass
class ExpertTuning:
    """Records an expert's tuning session"""
    scenario_name: str
    expert_id: str
    timestamp_us: int
    
    # Input features (X)
    feature_vec: dict
    
    # NN_v0 baseline prediction
    baseline_cues: dict
    
    # Expert's gold standard
    gold_cues: dict
    
    # Computed delta
    delta_cues: dict

class CLITuner:
    """
    Command-line interface for expert haptic tuning.
    """
    
    def __init__(self, 
                 model_path: str = "models/checkpoints/nn_v0_best.pt",
                 expert_id: str = "expert_001"):
        self.predictor = HapticPredictor(model_path)
        self.expert_id = expert_id
        self.tunings = []
        
        # Output directory
        self.output_dir = Path("data/phase2_tunings")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        print(f"\n{'='*60}")
        print(f"HAPTIC TUNING STUDIO (CLI Mode)")
        print(f"{'='*60}")
        print(f"Expert ID: {self.expert_id}")
        print(f"Model: {model_path}")
        print(f"Output: {self.output_dir}")
        print(f"{'='*60}\n")
    
    def tune_from_dataset(self, 
                         dataset_path: str = "data/processed/phase1_test.h5",
                         n_samples: int = 20):
        """
        Load samples from Phase 1 dataset and tune them.
        """
        print(f"Loading dataset: {dataset_path}")
        dataset = DatasetLoader(dataset_path)
        
        # Select random samples to tune
        import numpy as np
        indices = np.random.choice(dataset.n_samples, n_samples, replace=False)
        
        print(f"\nYou will tune {n_samples} scenarios")
        print(f"Commands:")
        print(f"  - Enter new value to change parameter")
        print(f"  - Press ENTER to keep baseline value")
        print(f"  - Type 'skip' to skip this scenario")
        print(f"  - Type 'quit' to finish and save\n")
        
        for i, idx in enumerate(indices):
            print(f"\n{'='*60}")
            print(f"SCENARIO {i+1}/{n_samples} (Index: {idx})")
            print(f"{'='*60}")
            
            # Extract features
            feature_vec = self._extract_feature_vec_from_dataset(dataset, idx)
            
            # Get baseline prediction
            baseline = self.predictor.predict(feature_vec)
            
            # Show context
            self._print_context(feature_vec)
            
            # Show baseline
            print(f"\nNN_v0 Baseline Prediction:")
            self._print_cues(baseline)
            
            # Get expert tuning
            result = self._tune_interactive(feature_vec, baseline, f"scenario_{idx}")
            
            if result == "skip":
                print("  Skipped.")
                continue
            elif result == "quit":
                print("\nQuitting early...")
                break
            
            print("  ✓ Tuning saved")
        
        # Save all tunings
        self._save_tunings()
        
        print(f"\n{'='*60}")
        print(f"TUNING SESSION COMPLETE")
        print(f"{'='*60}")
        print(f"Total tunings: {len(self.tunings)}")
        print(f"Saved to: {self.output_dir}")
        print(f"{'='*60}\n")
    
    def _extract_feature_vec_from_dataset(self, dataset, idx) -> dict:
        """Extract FeatureVec from dataset"""
        phase_map = {0: "PHASE_NO_CONTACT", 1: "PHASE_IMPACT", 
                     2: "PHASE_HOLD", 3: "PHASE_SLIP", 4: "PHASE_RELEASE"}
        
        phase_val = int(dataset.X[idx, 0] * 4)
        
        return {
            'phase': phase_map.get(phase_val, "PHASE_NO_CONTACT"),
            'phase_confidence_01': float(dataset.X[idx, 1]),
            'normal_force_N': float(dataset.X[idx, 2]),
            'shear_force_N': float(dataset.X[idx, 3]),
            'slip_speed_mms': float(dataset.X[idx, 4]),
            'material_features': {
                'hardness_01': float(dataset.X[idx, 5]),
                'mu_01': float(dataset.X[idx, 6]),
                'roughness_rms_um': float(dataset.X[idx, 7])
            },
            'uncertainty_pct': float(dataset.X[idx, 8])
        }
    
    def _print_context(self, feature_vec: dict):
        """Print scenario context"""
        print(f"\nScenario Context:")
        print(f"  Phase: {feature_vec['phase']}")
        print(f"  Normal Force: {feature_vec['normal_force_N']:.3f} N")
        print(f"  Shear Force: {feature_vec['shear_force_N']:.3f} N")
        print(f"  Slip Speed: {feature_vec['slip_speed_mms']:.2f} mm/s")
        print(f"  Material: hardness={feature_vec['material_features']['hardness_01']:.2f}, "
              f"μ={feature_vec['material_features']['mu_01']:.2f}")
    
    def _print_cues(self, cues: dict):
        """Print cue parameters in readable format"""
        print(f"  Impact:")
        print(f"    A = {cues['impact']['A']:.3f}")
        print(f"    rise_ms = {cues['impact']['rise_ms']:.2f}")
        print(f"    fall_ms = {cues['impact']['fall_ms']:.2f}")
        print(f"    hf_weight = {cues['impact']['hf_weight']:.3f}")
        print(f"  Weight:")
        print(f"    A = {cues['weight']['A']:.3f}")
        print(f"    rate_ms = {cues['weight']['rate_ms']:.2f}")
        print(f"  Shear:")
        print(f"    A = {cues['shear']['A']:.3f}")
        print(f"  Texture:")
        print(f"    A = {cues['texture']['A']:.3f}")
        print(f"    color = {cues['texture']['color']}")
    
    def _tune_interactive(self, feature_vec: dict, baseline: dict, scenario_name: str):
        """Interactive tuning session"""
        import copy
        gold = copy.deepcopy(baseline)
        
        print(f"\nAdjust parameters (or press ENTER to keep):")
        
        # Impact parameters
        gold['impact']['A'] = self._get_float_input(
            "  impact.A", baseline['impact']['A'], 0.0, 1.0
        )
        if gold['impact']['A'] == "skip":
            return "skip"
        if gold['impact']['A'] == "quit":
            return "quit"
            
        gold['impact']['rise_ms'] = self._get_float_input(
            "  impact.rise_ms", baseline['impact']['rise_ms'], 0.5, 10.0
        )
        if gold['impact']['rise_ms'] in ["skip", "quit"]:
            return gold['impact']['rise_ms']
            
        gold['impact']['fall_ms'] = self._get_float_input(
            "  impact.fall_ms", baseline['impact']['fall_ms'], 2.0, 50.0
        )
        if gold['impact']['fall_ms'] in ["skip", "quit"]:
            return gold['impact']['fall_ms']
            
        gold['impact']['hf_weight'] = self._get_float_input(
            "  impact.hf_weight", baseline['impact']['hf_weight'], 0.0, 1.0
        )
        if gold['impact']['hf_weight'] in ["skip", "quit"]:
            return gold['impact']['hf_weight']
        
        # Weight parameters
        gold['weight']['A'] = self._get_float_input(
            "  weight.A", baseline['weight']['A'], 0.0, 1.0
        )
        if gold['weight']['A'] in ["skip", "quit"]:
            return gold['weight']['A']
            
        gold['weight']['rate_ms'] = self._get_float_input(
            "  weight.rate_ms", baseline['weight']['rate_ms'], 10.0, 200.0
        )
        if gold['weight']['rate_ms'] in ["skip", "quit"]:
            return gold['weight']['rate_ms']
        
        # Compute deltas
        delta = self._compute_delta(baseline, gold)
        
        # Save tuning
        tuning = ExpertTuning(
            scenario_name=scenario_name,
            expert_id=self.expert_id,
            timestamp_us=feature_vec.get('timestamp_us', 0),
            feature_vec=feature_vec,
            baseline_cues=baseline,
            gold_cues=gold,
            delta_cues=delta
        )
        
        self.tunings.append(tuning)
        return "success"
    
    def _get_float_input(self, param_name: str, current_val, min_val, max_val):
        """Get float input with validation"""
        while True:
            try:
                user_input = input(f"{param_name} [{current_val:.3f}]: ").strip()
                
                if user_input == "":
                    return current_val
                if user_input.lower() == "skip":
                    return "skip"
                if user_input.lower() == "quit":
                    return "quit"
                
                val = float(user_input)
                if min_val <= val <= max_val:
                    return val
                else:
                    print(f"    Error: Value must be between {min_val} and {max_val}")
            except ValueError:
                print(f"    Error: Invalid number. Try again.")
    
    def _compute_delta(self, baseline: dict, gold: dict) -> dict:
        """Compute delta = gold - baseline"""
        delta = {
            'impact': {
                'A': gold['impact']['A'] - baseline['impact']['A'],
                'rise_ms': gold['impact']['rise_ms'] - baseline['impact']['rise_ms'],
                'fall_ms': gold['impact']['fall_ms'] - baseline['impact']['fall_ms'],
                'hf_weight': gold['impact']['hf_weight'] - baseline['impact']['hf_weight']
            },
            'weight': {
                'A': gold['weight']['A'] - baseline['weight']['A'],
                'rate_ms': gold['weight']['rate_ms'] - baseline['weight']['rate_ms']
            },
            'shear': {
                'A': gold['shear']['A'] - baseline['shear']['A'],
                'band_Hz': gold['shear']['band_Hz']  # Keep as-is for now
            },
            'texture': {
                'A': gold['texture']['A'] - baseline['texture']['A'],
                'color': gold['texture']['color']  # Categorical, no delta
            }
        }
        return delta
    
    def _save_tunings(self):
        """Save all tunings to JSON"""
        output_file = self.output_dir / f"{self.expert_id}_tunings.json"
        
        with open(output_file, 'w') as f:
            json.dump([asdict(t) for t in self.tunings], f, indent=2)
        
        print(f"\n✓ Saved {len(self.tunings)} tunings to: {output_file}")


def main():
    """Run CLI tuner"""
    tuner = CLITuner(expert_id="expert_001")
    tuner.tune_from_dataset(n_samples=20)

if __name__ == "__main__":
    main()