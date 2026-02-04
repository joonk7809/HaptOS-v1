"""
14-Dimensional Feature Extractor for SensationParams Architecture (v2.0)

Extends feature extraction from 13 to 14 dimensions for NeuralRenderer_v2.

New features:
- force_delta: F_t - F_{t-10} for impact detection and hysteresis robustness
- contact_area: Estimated from force and material stiffness
- contact_duration: Time since phase became non-NO_CONTACT

Changes from v1:
- Raw force values instead of log-transformed
- 4-dim phase one-hot (NO_CONTACT removed, implicit when all 0)
- No phase_confidence (embedded in one-hot)
- No uncertainty (not needed for perceptual model)
"""

import numpy as np
from collections import deque, defaultdict
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional
from src.core.schemas import ContactPatch


class Phase(Enum):
    """Contact phase states for FSM."""
    NO_CONTACT = 0
    IMPACT = 1
    HOLD = 2
    SLIP = 3
    RELEASE = 4


class FeatureExtractorV2:
    """
    Extracts 14-dimensional feature vectors from ContactPatch for NeuralRenderer_v2.

    Feature dimensions:
    [0] normal_force: Current normal force [N]
    [1] shear_magnitude: Current shear magnitude [N]
    [2] force_delta: F_t - F_{t-10} [N] (10ms ago)
    [3] slip_speed: Tangential velocity [mm/s]
    [4] acceleration: d(slip_speed)/dt [mm/s²] (or phase if needed)
    [5] hardness: Material hardness [0-1]
    [6] friction: Friction coefficient [0-1]
    [7] roughness: Surface roughness [0-1]
    [8] phase_impact: One-hot phase indicator
    [9] phase_hold: One-hot phase indicator
    [10] phase_slip: One-hot phase indicator
    [11] phase_release: One-hot phase indicator
    [12] contact_duration: Time in contact [ms]
    [13] contact_area: Estimated contact area [m²]
    """

    def __init__(self):
        # Force history: 10-frame buffer per body_part_id
        self.force_history: Dict[int, deque] = defaultdict(lambda: deque([0.0] * 10, maxlen=10))

        # Contact timing: timestamp when contact started per body_part_id
        self.contact_start_time: Dict[int, int] = {}

        # Phase tracking for FSM
        self.prev_phase: Dict[int, Phase] = defaultdict(lambda: Phase.NO_CONTACT)
        self.phase_start_time: Dict[int, int] = {}

        # Material stiffness database (material_hint → stiffness [N/m])
        # Used for contact_area estimation
        self.material_stiffness = {
            0: 1e4,   # Unknown/default
            1: 5e4,   # Metal (steel)
            2: 1e4,   # Wood
            3: 1e5,   # Glass
            4: 5e3,   # Rubber
            5: 2e4,   # Plastic
        }

        # Phase detection thresholds
        self.IMPACT_FORCE_THRESHOLD = 0.5  # N
        self.SLIP_SPEED_THRESHOLD = 10.0   # mm/s
        self.HOLD_TIME_MS = 20             # ms

        # Store last extracted features for onset detection
        self.last_features: Optional[np.ndarray] = None

    def extract(self, contact: ContactPatch) -> np.ndarray:
        """
        Extract 14-dim feature vector from a single ContactPatch.

        Args:
            contact: ContactPatch from physics simulation

        Returns:
            np.ndarray: 14-dimensional feature vector
        """
        bid = contact.body_part_id

        # === Feature [0-2]: Force features ===
        normal_force = contact.force_normal
        shear_magnitude = np.linalg.norm(contact.force_shear)

        # Calculate force_delta (F_t - F_{t-10})
        force_10ms_ago = self.force_history[bid][0]
        force_delta = normal_force - force_10ms_ago
        self.force_history[bid].append(normal_force)

        # === Feature [3]: Slip speed ===
        # Calculate from velocity magnitude in contact plane
        slip_speed = np.linalg.norm(contact.velocity[:2]) * 1000.0  # m/s → mm/s

        # === Feature [4]: Acceleration (placeholder - could compute d(slip_speed)/dt) ===
        # For now, use 0.0 - can add slip_speed history if needed
        acceleration = 0.0

        # === Features [5-7]: Material properties ===
        hardness, friction, roughness = self._get_material_properties(contact.material_hint)

        # === Features [8-11]: Phase one-hot (4 dims) ===
        current_phase = self._detect_phase(
            normal_force, shear_magnitude, slip_speed,
            contact.timestamp_us, bid
        )
        phase_one_hot = self._phase_to_onehot(current_phase)

        # === Feature [12]: Contact duration ===
        if current_phase == Phase.NO_CONTACT:
            self.contact_start_time[bid] = contact.timestamp_us
            contact_duration = 0.0
        else:
            if bid not in self.contact_start_time:
                self.contact_start_time[bid] = contact.timestamp_us
            contact_duration = (contact.timestamp_us - self.contact_start_time[bid]) / 1000.0  # μs → ms

        # === Feature [13]: Contact area ===
        contact_area = self._estimate_contact_area(normal_force, contact.material_hint)

        # Assemble 14-dim feature vector
        features = np.array([
            normal_force,           # [0]
            shear_magnitude,        # [1]
            force_delta,            # [2]
            slip_speed,             # [3]
            acceleration,           # [4]
            hardness,               # [5]
            friction,               # [6]
            roughness,              # [7]
            phase_one_hot[0],       # [8] phase_impact
            phase_one_hot[1],       # [9] phase_hold
            phase_one_hot[2],       # [10] phase_slip
            phase_one_hot[3],       # [11] phase_release
            contact_duration,       # [12]
            contact_area            # [13]
        ], dtype=np.float32)

        # Store for onset detection
        self.last_features = features

        return features

    def _detect_phase(self, normal_force: float, shear_magnitude: float,
                     slip_speed: float, timestamp_us: int, body_part_id: int) -> Phase:
        """
        Detect current contact phase using FSM.

        Args:
            normal_force: Normal force [N]
            shear_magnitude: Shear force magnitude [N]
            slip_speed: Slip velocity [mm/s]
            timestamp_us: Current timestamp
            body_part_id: Body part ID for state tracking

        Returns:
            Phase: Current phase
        """
        bid = body_part_id
        prev_phase = self.prev_phase[bid]

        # NO CONTACT: No force
        if normal_force < 0.01:
            self.prev_phase[bid] = Phase.NO_CONTACT
            return Phase.NO_CONTACT

        # IMPACT: Rising force edge (transition from NO_CONTACT)
        if normal_force > self.IMPACT_FORCE_THRESHOLD:
            if prev_phase == Phase.NO_CONTACT:
                self.prev_phase[bid] = Phase.IMPACT
                self.phase_start_time[bid] = timestamp_us
                return Phase.IMPACT

        # SLIP: High slip speed during contact
        if slip_speed > self.SLIP_SPEED_THRESHOLD and prev_phase != Phase.NO_CONTACT:
            self.prev_phase[bid] = Phase.SLIP
            return Phase.SLIP

        # HOLD: Steady force after impact
        if bid in self.phase_start_time:
            time_since_phase_start = (timestamp_us - self.phase_start_time[bid]) / 1000.0  # μs → ms
            if time_since_phase_start > self.HOLD_TIME_MS:
                if slip_speed < self.SLIP_SPEED_THRESHOLD:
                    self.prev_phase[bid] = Phase.HOLD
                    return Phase.HOLD

        # RELEASE: Falling force edge
        if prev_phase in [Phase.HOLD, Phase.SLIP, Phase.IMPACT]:
            if normal_force < self.IMPACT_FORCE_THRESHOLD * 0.5:
                self.prev_phase[bid] = Phase.RELEASE
                return Phase.RELEASE

        # Default: stay in previous phase
        return prev_phase

    def _phase_to_onehot(self, phase: Phase) -> np.ndarray:
        """
        Convert phase to 4-dim one-hot encoding.

        NO_CONTACT is not included (implicit when all zeros).

        Args:
            phase: Current phase

        Returns:
            np.ndarray: [phase_impact, phase_hold, phase_slip, phase_release]
        """
        if phase == Phase.IMPACT:
            return np.array([1.0, 0.0, 0.0, 0.0])
        elif phase == Phase.HOLD:
            return np.array([0.0, 1.0, 0.0, 0.0])
        elif phase == Phase.SLIP:
            return np.array([0.0, 0.0, 1.0, 0.0])
        elif phase == Phase.RELEASE:
            return np.array([0.0, 0.0, 0.0, 1.0])
        else:  # NO_CONTACT
            return np.array([0.0, 0.0, 0.0, 0.0])

    def _get_material_properties(self, material_hint: int) -> tuple:
        """
        Get material properties from material hint.

        Args:
            material_hint: Material ID (0 = unknown)

        Returns:
            (hardness, friction, roughness) tuple, all in [0, 1]
        """
        # Material property database
        # Format: {material_id: (hardness, friction, roughness)}
        material_db = {
            0: (0.5, 0.5, 0.3),   # Unknown/default
            1: (0.9, 0.2, 0.2),   # Metal (hard, smooth, low friction)
            2: (0.6, 0.6, 0.5),   # Wood (medium hardness, medium friction, medium rough)
            3: (0.95, 0.1, 0.1),  # Glass (very hard, very smooth, very low friction)
            4: (0.2, 0.8, 0.4),   # Rubber (soft, high friction, medium rough)
            5: (0.7, 0.4, 0.3),   # Plastic (hard, low-med friction, smooth)
        }

        return material_db.get(material_hint, material_db[0])

    def _estimate_contact_area(self, normal_force: float, material_hint: int) -> float:
        """
        Estimate contact area from Hertzian contact theory.

        Simplified formula: A ≈ π × (sqrt(F / k))²
        where k is material stiffness [N/m]

        Args:
            normal_force: Normal force [N]
            material_hint: Material ID

        Returns:
            float: Estimated contact area [m²]
        """
        stiffness = self.material_stiffness.get(material_hint, 1e4)

        # Avoid division by zero
        if normal_force < 1e-6:
            return 0.0

        # Hertzian contact: radius ~ sqrt(F/k)
        contact_radius = np.sqrt(normal_force / stiffness)
        contact_area = np.pi * contact_radius**2

        return float(contact_area)

    def reset(self, body_part_id: Optional[int] = None):
        """
        Reset state tracking.

        Args:
            body_part_id: If provided, reset only this body part. Otherwise reset all.
        """
        if body_part_id is None:
            # Reset all
            self.force_history.clear()
            self.contact_start_time.clear()
            self.prev_phase.clear()
            self.phase_start_time.clear()
            self.last_features = None
        else:
            # Reset specific body part
            if body_part_id in self.force_history:
                self.force_history[body_part_id] = deque([0.0] * 10, maxlen=10)
            if body_part_id in self.contact_start_time:
                del self.contact_start_time[body_part_id]
            if body_part_id in self.prev_phase:
                self.prev_phase[body_part_id] = Phase.NO_CONTACT
            if body_part_id in self.phase_start_time:
                del self.phase_start_time[body_part_id]

    def get_feature_names(self) -> list:
        """
        Get human-readable feature names for debugging.

        Returns:
            list: 14 feature names
        """
        return [
            'normal_force',
            'shear_magnitude',
            'force_delta',
            'slip_speed',
            'acceleration',
            'hardness',
            'friction',
            'roughness',
            'phase_impact',
            'phase_hold',
            'phase_slip',
            'phase_release',
            'contact_duration',
            'contact_area'
        ]
