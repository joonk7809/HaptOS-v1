"""
Extracts FeatureVec (X) from physics ContactPatch data.
This is what the NN will receive at runtime.
"""

import numpy as np
from dataclasses import dataclass
from typing import List
from enum import Enum

class PhaseType(Enum):
    """Contact phase states"""
    PHASE_NO_CONTACT = "PHASE_NO_CONTACT"
    PHASE_IMPACT = "PHASE_IMPACT"
    PHASE_HOLD = "PHASE_HOLD"
    PHASE_SLIP = "PHASE_SLIP"
    PHASE_RELEASE = "PHASE_RELEASE"

@dataclass
class MaterialFeatures:
    """Material properties aggregated over contact window"""
    hardness_01: float  # 0=soft, 1=hard (derived from solref damping)
    mu_01: float        # Coefficient of friction (0-1)
    roughness_rms_um: float  # RMS roughness in micrometers

@dataclass
class FeatureVec:
    """
    Input feature vector (X) for the neural network.
    Represents a 10ms window of physics state at 100Hz.
    """
    timestamp_us: int
    phase_one_hot: List[float]  # [NO_CONTACT, IMPACT, HOLD, SLIP, RELEASE]
    phase_confidence_01: float
    normal_force_log: float     # log10(force + 1)
    shear_force_log: float      # log10(force + 1)
    slip_speed_mms: float
    material_features: MaterialFeatures
    uncertainty_pct: float

class FeatureExtractor:
    """
    Converts 10ms window of ContactPatch data (10 samples @ 1kHz)
    into a single FeatureVec for training.
    """
    
    def __init__(self):
        # State tracking for phase detection
        self.prev_phase = PhaseType.PHASE_NO_CONTACT
        self.phase_start_time_us = 0
        self.impact_detected = False
        
        # Thresholds for phase detection
        self.IMPACT_FORCE_THRESHOLD = 0.5  # N
        self.SLIP_SPEED_THRESHOLD = 5.0     # mm/s
        self.HOLD_TIME_MS = 20              # ms
        
    def extract(self, contact_window: List, window_start_us: int) -> FeatureVec:
        """
        Extract features from a 10ms window of ContactPatch data.
        
        Args:
            contact_window: List of 10 ContactPatch objects (1kHz data)
            window_start_us: Timestamp of window start
            
        Returns:
            FeatureVec ready for NN input
        """
        # Aggregate forces over window
        forces_normal = [p.normal_force_N for p in contact_window]
        forces_shear = [p.shear_force_N for p in contact_window]
        slip_speeds = [p.slip_speed_mms for p in contact_window]
        in_contact = any(p.in_contact for p in contact_window)
        
        # Mean values
        normal_force = np.mean(forces_normal)
        shear_force = np.mean(forces_shear)
        slip_speed = np.mean(slip_speeds)
        
        # Detect phase
        phase, confidence = self._detect_phase(
            normal_force, shear_force, slip_speed, in_contact, window_start_us
        )
        
        # One-Hot Encode Phase
        phase_map = {
            PhaseType.PHASE_NO_CONTACT: 0,
            PhaseType.PHASE_IMPACT: 1,
            PhaseType.PHASE_HOLD: 2,
            PhaseType.PHASE_SLIP: 3,
            PhaseType.PHASE_RELEASE: 4
        }
        phase_idx = phase_map[phase]
        phase_one_hot = [0.0] * 5
        phase_one_hot[phase_idx] = 1.0
        
        # Log Transform Forces
        normal_force_log = np.log10(normal_force + 1.0)
        shear_force_log = np.log10(shear_force + 1.0)
        
        # Extract material properties (from first contact in window)
        material = self._extract_material_features(contact_window)
        
        # Calculate uncertainty (based on variance in the window)
        force_variance = np.var(forces_normal) if len(forces_normal) > 1 else 0
        uncertainty = min(100, (force_variance / (normal_force + 0.001)) * 100)
        
        return FeatureVec(
            timestamp_us=window_start_us,
            phase_one_hot=phase_one_hot,
            phase_confidence_01=confidence,
            normal_force_log=normal_force_log,
            shear_force_log=shear_force_log,
            slip_speed_mms=slip_speed,
            material_features=material,
            uncertainty_pct=uncertainty
        )
    
    def _detect_phase(self, normal_force: float, shear_force: float, 
                      slip_speed: float, in_contact: bool,
                      current_time_us: int) -> tuple[PhaseType, float]:
        """
        Detect current contact phase using simple FSM.
        
        Returns:
            (phase, confidence) tuple
        """
        confidence = 0.95  # Default high confidence
        
        # NO CONTACT
        if not in_contact or normal_force < 0.01:
            self.prev_phase = PhaseType.PHASE_NO_CONTACT
            self.impact_detected = False
            return PhaseType.PHASE_NO_CONTACT, 1.0
        
        # IMPACT: Rising force edge
        if normal_force > self.IMPACT_FORCE_THRESHOLD:
            if self.prev_phase == PhaseType.PHASE_NO_CONTACT:
                self.prev_phase = PhaseType.PHASE_IMPACT
                self.phase_start_time_us = current_time_us
                self.impact_detected = True
                return PhaseType.PHASE_IMPACT, 0.98
        
        # SLIP: High slip speed during contact
        if self.impact_detected and slip_speed > self.SLIP_SPEED_THRESHOLD:
            self.prev_phase = PhaseType.PHASE_SLIP
            return PhaseType.PHASE_SLIP, 0.90
        
        # HOLD: Steady force after impact
        time_since_phase_start = (current_time_us - self.phase_start_time_us) / 1000  # ms
        if self.impact_detected and time_since_phase_start > self.HOLD_TIME_MS:
            if slip_speed < self.SLIP_SPEED_THRESHOLD:
                self.prev_phase = PhaseType.PHASE_HOLD
                return PhaseType.PHASE_HOLD, 0.85
        
        # RELEASE: Falling force edge
        if self.prev_phase in [PhaseType.PHASE_HOLD, PhaseType.PHASE_SLIP]:
            if normal_force < self.IMPACT_FORCE_THRESHOLD * 0.5:
                self.prev_phase = PhaseType.PHASE_RELEASE
                return PhaseType.PHASE_RELEASE, 0.80
        
        # Default: stay in previous phase with lower confidence
        return self.prev_phase, 0.70
    
    def _extract_material_features(self, contact_window: List) -> MaterialFeatures:
        """
        Extract material properties from contact window.
        """
        # Find first contact in window
        contact_patch = None
        for p in contact_window:
            if p.in_contact:
                contact_patch = p
                break
        
        if contact_patch is None:
            # No contact - return defaults
            return MaterialFeatures(
                hardness_01=0.5,
                mu_01=0.5,
                roughness_rms_um=1.0
            )
        
        # Hardness from solref damping (higher damping = softer)
        # Typical range: 0.5-2.0, normalize to 0-1
        hardness = 1.0 - np.clip((contact_patch.solref_damping - 0.5) / 1.5, 0, 1)
        
        # Friction coefficient (already 0-1 range)
        mu = contact_patch.mu_static
        
        # Roughness placeholder (would come from texture in real system)
        # For now, derive from friction: high friction suggests rough surface
        roughness = mu * 10.0  # Scale to micrometers
        
        return MaterialFeatures(
            hardness_01=hardness,
            mu_01=mu,
            roughness_rms_um=roughness
        )
    
    def reset(self):
        """Reset phase tracking state"""
        self.prev_phase = PhaseType.PHASE_NO_CONTACT
        self.phase_start_time_us = 0
        self.impact_detected = False