"""
Neural Renderer - Layer 2 of HAPTOS Platform

Converts filtered contacts to haptic cue parameters via ML inference.
Wraps existing ML pipeline (FeatureExtractor + CombinedPredictor) with adapters.

Architecture:
    FilteredContact[] → Schema Adapter → OLD ContactPatch[]
                     ↓
              FeatureExtractor → FeatureVec (13-dim)
                     ↓
              CombinedPredictor (NN_v0 + NN_v1)
                     ↓
              Legacy CueParams dict
                     ↓
              Output Adapter → NEW CueParams dataclass
                     ↓
              Cue Masking → Apply cue_mask filter
                     ↓
              Dict[body_part_id, CueParams]
"""

from typing import List, Dict, Optional
from collections import deque, defaultdict
import time

from src.core.schemas import FilteredContact, CueParams
from src.converter.feature_extractor import FeatureExtractor, PhaseType
from src.inference.combined_predictor import CombinedPredictor
from src.inference.adapters import (
    new_contact_to_old,
    legacy_cues_to_new,
    apply_cue_mask,
    OldContactPatch
)


class ContactBuffer:
    """
    Manages 10ms rolling windows for each body part.

    FeatureExtractor expects 10 samples @ 1kHz (10ms window).
    Router provides 1ms snapshots, so we buffer them.
    """

    def __init__(self, window_size: int = 10):
        """
        Initialize buffer.

        Args:
            window_size: Number of samples to buffer (default 10 for 10ms @ 1kHz)
        """
        self.window_size = window_size
        self.buffers: Dict[int, deque] = {}  # body_part_id → deque of OldContactPatch

    def add(self, body_part_id: int, patch: OldContactPatch):
        """
        Add contact to rolling window.

        Args:
            body_part_id: Body part identifier
            patch: OLD ContactPatch to buffer
        """
        if body_part_id not in self.buffers:
            self.buffers[body_part_id] = deque(maxlen=self.window_size)
        self.buffers[body_part_id].append(patch)

    def get_window(self, body_part_id: int) -> Optional[List[OldContactPatch]]:
        """
        Get full 10ms window for a body part.

        Args:
            body_part_id: Body part identifier

        Returns:
            List of ContactPatch (10 samples) or None if buffer not full yet
        """
        if body_part_id not in self.buffers:
            return None

        window = list(self.buffers[body_part_id])
        return window if len(window) == self.window_size else None

    def clear(self, body_part_id: Optional[int] = None):
        """
        Clear buffer for specific body part or all.

        Args:
            body_part_id: Body part to clear, or None for all
        """
        if body_part_id is None:
            self.buffers.clear()
        elif body_part_id in self.buffers:
            self.buffers[body_part_id].clear()


class NeuralRenderer:
    """
    Layer 2: Neural Renderer

    Consumes FilteredContact[] from Router, emits CueParams for Hardware Driver.
    Wraps existing ML pipeline (FeatureExtractor + CombinedPredictor) with adapters.

    Key Features:
    - Window buffering: Accumulate 10ms windows per body part
    - FSM state tracking: Detect phase transitions for trigger_impulse
    - Schema adaptation: NEW → OLD → ML → NEW
    - Cue masking: Selective rendering based on cue_mask
    """

    def __init__(
        self,
        nn_v0_path: str = "models/checkpoints/nn_v0_best.pt",
        nn_v1_path: str = "models/checkpoints/nn_v1_best.pt",
        device: str = 'cpu'
    ):
        """
        Initialize Neural Renderer.

        Args:
            nn_v0_path: Path to NN_v0 baseline model weights
            nn_v1_path: Path to NN_v1 delta refinement weights
            device: Torch device ('cpu' or 'cuda')
        """
        # ML pipeline components
        self.predictor = CombinedPredictor(nn_v0_path, nn_v1_path, device)
        self.feature_extractors: Dict[int, FeatureExtractor] = {}  # Per body part

        # Buffering and state tracking
        self.buffer = ContactBuffer(window_size=10)
        self.prev_phases: Dict[int, PhaseType] = {}  # For impulse detection

        # Statistics
        self.render_count = 0
        self.total_inference_time_ms = 0.0
        self.active_contacts = set()

        print("✓ Neural Renderer initialized")
        print(f"  NN_v0: {nn_v0_path}")
        print(f"  NN_v1: {nn_v1_path}")
        print(f"  Device: {device}")

    def render(
        self,
        filtered_contacts: List[FilteredContact]
    ) -> Dict[int, CueParams]:
        """
        Generate haptic cue parameters for all contacts.

        Pipeline:
        1. Convert FilteredContact → OLD ContactPatch (schema adapter)
        2. Add to 10ms rolling windows (buffer management)
        3. Extract features → FeatureVec (when window full)
        4. Run ML inference → Legacy CueParams dict
        5. Detect phase transitions → trigger_impulse flag
        6. Convert legacy dict → NEW CueParams (output adapter)
        7. Apply cue masking (selective rendering)
        8. Return Dict[body_part_id, CueParams]

        Args:
            filtered_contacts: List of biologically-filtered contacts from Router

        Returns:
            Dict[body_part_id, CueParams]: Haptic parameters per contact
        """
        start_time = time.time()
        cue_dict = {}

        # Track active contacts
        current_contacts = {fc.patch.body_part_id for fc in filtered_contacts}
        self.active_contacts = current_contacts

        for filtered_contact in filtered_contacts:
            body_part_id = filtered_contact.patch.body_part_id
            contact_patch = filtered_contact.patch
            cue_mask = filtered_contact.cue_mask

            # Step 1: Convert NEW → OLD ContactPatch
            old_patch = new_contact_to_old(contact_patch)

            # Step 2: Add to buffer
            self.buffer.add(body_part_id, old_patch)

            # Step 3: Check if we have a full window
            window = self.buffer.get_window(body_part_id)
            if window is None:
                # Not enough samples yet, skip inference
                continue

            # Get or create FeatureExtractor for this body part
            if body_part_id not in self.feature_extractors:
                self.feature_extractors[body_part_id] = FeatureExtractor()

            extractor = self.feature_extractors[body_part_id]

            # Step 4: Extract features
            # Use first timestamp in window as window_start_us
            window_start_us = window[0].timestamp_us
            feature_vec = extractor.extract(window, window_start_us)

            # Step 5: Run ML inference
            # Convert FeatureVec dataclass to dict for predictor
            feature_dict = self._feature_vec_to_dict(feature_vec)
            legacy_cues = self.predictor.predict(feature_dict)

            # Step 6: Detect phase transitions for trigger_impulse
            # phase_one_hot is a list [0,0,1,0,0] - find index of 1.0
            current_phase_idx = feature_vec.phase_one_hot.index(1.0) if 1.0 in feature_vec.phase_one_hot else 0
            trigger_impulse = self._detect_impulse(body_part_id, current_phase_idx)

            # Step 7: Convert legacy dict → NEW CueParams
            cue_params = legacy_cues_to_new(
                legacy_dict=legacy_cues,
                contact=contact_patch,
                timestamp_us=contact_patch.timestamp_us,
                trigger_impulse=trigger_impulse
            )

            # Step 8: Apply cue masking
            masked_cues = apply_cue_mask(cue_params, cue_mask)

            cue_dict[body_part_id] = masked_cues

        # Update statistics
        inference_time_ms = (time.time() - start_time) * 1000
        self.render_count += 1
        self.total_inference_time_ms += inference_time_ms

        return cue_dict

    def _feature_vec_to_dict(self, feature_vec) -> dict:
        """
        Convert FeatureVec dataclass to dict for CombinedPredictor.

        Args:
            feature_vec: FeatureVec from FeatureExtractor

        Returns:
            dict compatible with CombinedPredictor.predict()
        """
        return {
            'timestamp_us': feature_vec.timestamp_us,
            'phase_one_hot': feature_vec.phase_one_hot,
            'phase_confidence_01': feature_vec.phase_confidence_01,
            'normal_force_log': feature_vec.normal_force_log,
            'shear_force_log': feature_vec.shear_force_log,
            'slip_speed_mms': feature_vec.slip_speed_mms,
            'material_features': {
                'hardness_01': feature_vec.material_features.hardness_01,
                'mu_01': feature_vec.material_features.mu_01,
                'roughness_rms_um': feature_vec.material_features.roughness_rms_um
            },
            'uncertainty_pct': feature_vec.uncertainty_pct
        }

    def _detect_impulse(
        self,
        body_part_id: int,
        current_phase_idx: int
    ) -> bool:
        """
        Detect transient event from phase FSM state transition.

        Trigger impulse on:
        - NO_CONTACT (0) → IMPACT (1): initial contact
        - HOLD (2) → SLIP (3): slip initiation
        - Any → RELEASE (4): contact break

        Phase Index Mapping:
        - 0 = PHASE_NO_CONTACT
        - 1 = PHASE_IMPACT
        - 2 = PHASE_HOLD
        - 3 = PHASE_SLIP
        - 4 = PHASE_RELEASE

        Args:
            body_part_id: Body part identifier
            current_phase_idx: Current FSM phase index (0-4)

        Returns:
            True if impulse should fire this frame
        """
        prev_phase_idx = self.prev_phases.get(body_part_id, 0)  # Default: NO_CONTACT

        # Define trigger conditions
        trigger = False

        # NO_CONTACT (0) → IMPACT (1): initial contact
        if prev_phase_idx == 0 and current_phase_idx == 1:
            trigger = True

        # HOLD (2) → SLIP (3): slip initiation
        elif prev_phase_idx == 2 and current_phase_idx == 3:
            trigger = True

        # Any → RELEASE (4): contact break
        elif current_phase_idx == 4:
            trigger = True

        # Update state
        self.prev_phases[body_part_id] = current_phase_idx

        return trigger

    def reset(self):
        """Reset all state (FSM, buffers, statistics)."""
        self.buffer.clear()
        self.prev_phases.clear()
        self.feature_extractors.clear()
        self.render_count = 0
        self.total_inference_time_ms = 0.0
        self.active_contacts.clear()
        print("✓ Neural Renderer reset")

    def get_stats(self) -> Dict:
        """
        Get rendering statistics.

        Returns:
            Dict with:
            - total_renders: Number of render() calls
            - avg_inference_time_ms: Average ML inference time
            - active_contacts: Current number of contacts being rendered
        """
        avg_time = (
            self.total_inference_time_ms / self.render_count
            if self.render_count > 0 else 0.0
        )

        return {
            'total_renders': self.render_count,
            'avg_inference_time_ms': avg_time,
            'active_contacts': len(self.active_contacts)
        }
