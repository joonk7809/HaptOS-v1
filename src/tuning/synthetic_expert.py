"""
Synthetic Expert: Rule-based haptic tuning for Phase 2 PoC.
Simulates expert preferences using hand-crafted perceptual rules.
"""

import numpy as np
from typing import Dict
import copy

class SyntheticExpert:
    """
    Generates "expert" tunings using perceptual heuristics.
    
    Design principles:
    1. Sharper impacts at higher forces (faster rise)
    2. Material properties affect decay (friction → longer ring)
    3. Weight saturates (Weber's law)
    4. Texture increases with roughness
    """
    
    def __init__(self, expert_id: str = "synthetic_expert_001"):
        self.expert_id = expert_id
        
        # Tuning parameters (adjust these to change expert "style")
        self.impact_sharpness_factor = 0.7  # Lower = sharper impacts
        self.weight_saturation = 0.9        # Max weight amplitude
        self.texture_boost = 1.5            # Amplify texture
        
    def tune(self, baseline_cues: Dict, feature_vec: Dict) -> Dict:
        """
        Apply expert tuning rules to baseline prediction.
        
        Args:
            baseline_cues: NN_v0 prediction
            feature_vec: Input features (X)
            
        Returns:
            Gold standard cues with expert adjustments
        """
        # Create a numpy-friendly copy of baseline for calculations
        baseline_np = copy.deepcopy(baseline_cues)
        for key in ['impact', 'ring', 'shear', 'weight', 'texture']:
            for subkey in baseline_np[key]:
                if isinstance(baseline_np[key][subkey], list):
                    baseline_np[key][subkey] = np.array(baseline_np[key][subkey], dtype=np.float32)
        
        # Gold starts as copy of baseline
        gold = copy.deepcopy(baseline_np)
        
        # Extract relevant features
        force = feature_vec['normal_force_N']
        shear = feature_vec['shear_force_N']
        slip = feature_vec['slip_speed_mms']
        hardness = feature_vec['material_features']['hardness_01']
        mu = feature_vec['material_features']['mu_01']
        roughness = feature_vec['material_features']['roughness_rms_um']
        phase = feature_vec['phase']
        
        # Only tune during actual contact phases
        if phase == "PHASE_NO_CONTACT":
            return gold  # No changes for non-contact
        
        # === UNIVERSAL TUNING (applies to all phases) ===
        
        # Rule 1: Weight follows Weber's law (logarithmic perception)
        # Apply to ALL contact phases, not just impact
        if force > 0.01:
            # Logarithmic mapping instead of linear
            perceived_weight = np.log10(force + 1.0) / np.log10(20.0)  # Normalize to max 20N
            gold['weight']['A'] = np.clip(perceived_weight, 0.0, self.weight_saturation)
            
            # Adjust rate based on force
            if force < 1.0:
                # Light forces → slower ramp (more gentle)
                gold['weight']['rate_ms'] = np.clip(baseline_np['weight']['rate_ms'] * 1.5, 10.0, 200.0)
            elif force > 5.0:
                # Heavy forces → faster ramp (more sudden)
                gold['weight']['rate_ms'] = np.clip(baseline_np['weight']['rate_ms'] * 0.6, 10.0, 200.0)
        
        # Rule 2: Texture based on roughness (applies to all phases)
        if roughness > 3.0:
            texture_factor = 1.0 + (roughness / 10.0) * 0.5  # Up to 1.5x
            gold['texture']['A'] = np.clip(
                baseline_np['texture']['A'] * texture_factor,
                0.0, 1.0
            )
        else:
            # Reduce texture for very smooth surfaces
            gold['texture']['A'] = np.clip(
                baseline_np['texture']['A'] * 0.7,
                0.0, 1.0
            )
        
        # Rule 3: Texture color based on hardness
        if hardness > 0.7:
            gold['texture']['color'] = "COLOR_WHITE"
        elif hardness < 0.4:
            gold['texture']['color'] = "COLOR_BROWN"
        else:
            gold['texture']['color'] = "COLOR_PINK"
        
        # === PHASE-SPECIFIC TUNING ===
        
        if phase == "PHASE_IMPACT":
            # Rule 4: Sharper rise for harder/faster impacts
            if force > 3.0:
                rise_factor = np.clip(0.5 * (10.0 / (force + 1.0)), 0.3, 1.0)
                gold['impact']['rise_ms'] = baseline_np['impact']['rise_ms'] * rise_factor
            
            # Rule 5: Softer materials → longer fall time
            if hardness < 0.5:
                fall_factor = 1.0 + (0.5 - hardness)  # Up to 1.5x longer
                gold['impact']['fall_ms'] = baseline_np['impact']['fall_ms'] * fall_factor
            
            # Rule 6: Hard impacts have more high-frequency content
            if force > 5.0 and hardness > 0.6:
                gold['impact']['hf_weight'] = np.clip(
                    baseline_np['impact']['hf_weight'] + 0.2, 
                    0.0, 1.0
                )
        
        elif phase == "PHASE_HOLD":
            # Rule 7: During hold, impact cues should decay faster
            # Shorten fall time to avoid lingering impact sensation
            gold['impact']['fall_ms'] = np.clip(
                baseline_np['impact']['fall_ms'] * 0.7,  # 30% shorter
                2.0, 50.0
            )
            
            # Rule 8: Reduce impact amplitude during hold
            gold['impact']['A'] = np.clip(
                baseline_np['impact']['A'] * 0.5,  # 50% reduction
                0.0, 1.0
            )
            
            # Rule 9: Increase texture during steady contact
            # This gives more "feel" to the surface
            if baseline_np['texture']['A'] < 0.3:
                gold['texture']['A'] = np.clip(
                    baseline_np['texture']['A'] + 0.2,  # Boost quiet textures
                    0.0, 1.0
                )
            
            # Rule 14 (NEW): Dampen ring during hold (hand damping)
            # Reduce decay time (tau) and amplitude (a)
            gold['ring']['tau_ms'] = np.clip(
                baseline_np['ring']['tau_ms'] * 0.8,  # 20% faster decay
                20.0, 1000.0
            )
            gold['ring']['a'] = np.clip(
                baseline_np['ring']['a'] * 0.9,       # Slight amplitude reduction
                0.0, 1.0
            )
        
        elif phase == "PHASE_SLIP":
            # Rule 10: Amplify shear during slip
            if slip > 5.0:
                shear_boost = np.clip(slip / 20.0, 0.0, 0.5)
                gold['shear']['A'] = np.clip(
                    baseline_np['shear']['A'] + shear_boost,
                    0.0, 1.0
                )
            
            # Rule 11: Increase texture during slip (dynamic friction)
            gold['texture']['A'] = np.clip(
                baseline_np['texture']['A'] * 1.3,
                0.0, 1.0
            )
        
        elif phase == "PHASE_RELEASE":
            # Rule 12: Quick fade during release
            gold['impact']['fall_ms'] = np.clip(
                baseline_np['impact']['fall_ms'] * 0.5,  # Fast decay
                2.0, 20.0
            )
            
            # Rule 13: Reduce all amplitudes
            gold['impact']['A'] = np.clip(baseline_np['impact']['A'] * 0.3, 0.0, 1.0)
            gold['weight']['A'] = np.clip(baseline_np['weight']['A'] * 0.5, 0.0, 1.0)
            
            # Fast ring decay on release
            gold['ring']['tau_ms'] = np.clip(
                baseline_np['ring']['tau_ms'] * 0.5,
                20.0, 1000.0
            )
        
        return gold
    
    def get_tuning_style_description(self) -> str:
        """Describe this expert's tuning preferences"""
        return f"""
Synthetic Expert Profile: {self.expert_id}
==========================================
Preferences:
- Impact Sharpness: {self.impact_sharpness_factor:.1f} (lower = sharper)
- Weight Saturation: {self.weight_saturation:.1f}
- Texture Boost: {self.texture_boost:.1f}x

Universal Tuning Rules (All Phases):
1. Logarithmic weight perception (Weber's law)
2. Weight ramp speed based on force magnitude
3. Texture amplification based on roughness
4. Texture color matches material hardness

Phase-Specific Rules:
IMPACT:
  - Sharper rise for hard/fast impacts
  - Longer decay for soft materials
  - More HF content for hard impacts

HOLD:
  - Faster impact decay (avoid lingering)
  - Reduced impact amplitude
  - Boosted texture for surface feel
  - Ring damping (NEW): Faster decay due to hand contact

SLIP:
  - Amplified shear with slip speed
  - Increased texture (dynamic friction)

RELEASE:
  - Quick fade on all cues
  - Reduced amplitudes
  - Fast ring decay
"""


class PerceptualMetrics:
    """
    Evaluate how much expert tuning changed the baseline.
    Useful for analyzing tuning patterns.
    """
    
    @staticmethod
    def compute_tuning_magnitude(baseline: Dict, gold: Dict) -> Dict:
        """
        Compute relative change between baseline and gold standard.
        
        Returns:
            Dict of percentage changes for each parameter
        """
        changes = {}
        
        # Impact changes
        changes['impact_A_pct'] = ((gold['impact']['A'] - baseline['impact']['A']) / 
                                   (baseline['impact']['A'] + 0.001)) * 100
        changes['impact_rise_pct'] = ((gold['impact']['rise_ms'] - baseline['impact']['rise_ms']) / 
                                      (baseline['impact']['rise_ms'] + 0.001)) * 100
        changes['impact_fall_pct'] = ((gold['impact']['fall_ms'] - baseline['impact']['fall_ms']) / 
                                      (baseline['impact']['fall_ms'] + 0.001)) * 100
        
        # Weight changes
        changes['weight_A_pct'] = ((gold['weight']['A'] - baseline['weight']['A']) / 
                                   (baseline['weight']['A'] + 0.001)) * 100
        
        # Texture changes  
        changes['texture_A_pct'] = ((gold['texture']['A'] - baseline['texture']['A']) / 
                                    (baseline['texture']['A'] + 0.001)) * 100
        
        return changes
    
    @staticmethod
    def analyze_tuning_patterns(tunings: list) -> Dict:
        """
        Analyze trends across multiple tunings.
        """
        if not tunings:
            return {}
        
        changes_list = [
            PerceptualMetrics.compute_tuning_magnitude(t['baseline_cues'], t['gold_cues'])
            for t in tunings
        ]
        
        # Count non-zero deltas
        n_changed = sum(1 for c in changes_list if abs(c['impact_A_pct']) > 1.0)
        
        analysis = {
            'avg_impact_A_change': np.mean([c['impact_A_pct'] for c in changes_list]),
            'avg_impact_rise_change': np.mean([c['impact_rise_pct'] for c in changes_list]),
            'avg_impact_fall_change': np.mean([c['impact_fall_pct'] for c in changes_list]),
            'avg_weight_change': np.mean([c['weight_A_pct'] for c in changes_list]),
            'avg_texture_change': np.mean([c['texture_A_pct'] for c in changes_list]),
            'pct_changed': (n_changed / len(changes_list)) * 100,
            'n_samples': len(tunings)
        }
        
        return analysis