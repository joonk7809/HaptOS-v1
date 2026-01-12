#!/usr/bin/env python3
"""
Somatotopic Router - Biological Haptic Filtering.

Routes ContactPatches to FilteredContacts based on Homunculus table.
The Homunculus encodes biological sensor distribution (resolution, sensitivity,
frequency range) for different body parts, mimicking the somatosensory cortex.
"""

from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
import logging

from src.core.schemas import ContactPatch, FilteredContact, make_cue_mask

logger = logging.getLogger(__name__)


@dataclass
class BodyPartProperties:
    """
    Perceptual properties for a specific body part.

    Based on biological mechanoreceptor distribution and sensitivity.
    """
    spatial_res_mm: float       # Two-point discrimination threshold [mm]
    freq_range_hz: Tuple[float, float]  # Sensitive frequency range [Hz]
    sensitivity: float          # Force sensitivity multiplier (1.0 = baseline)
    rendering_tier: int         # Hardware tier requirement (1=VCA, 2=LRA, 3=ERM)
    cue_mask: int              # Enabled haptic cues for this body part


class Homunculus:
    """
    Biological sensor distribution model.

    Maps body parts to their perceptual properties based on mechanoreceptor
    density and characteristics. Inspired by Penfield's cortical homunculus.

    Phase 1: Hardcoded table for index fingertip
    Phase 2+: Configurable, user-calibrated
    """

    def __init__(self, config: Optional[Dict] = None):
        """
        Initialize Homunculus with perceptual properties.

        Args:
            config: Optional configuration dict. If None, uses default table.
        """
        if config:
            self.table = self._load_config(config)
        else:
            self.table = self._default_table()

    def _default_table(self) -> Dict[str, BodyPartProperties]:
        """
        Default Homunculus table based on biological data.

        References:
        - Johansson & Vallbo (1979): Mechanoreceptor properties
        - Weinstein (1968): Two-point discrimination thresholds
        - Bolanowski et al. (1988): Frequency sensitivity curves
        """
        return {
            # HAND - High resolution, full cue support
            'index_tip': BodyPartProperties(
                spatial_res_mm=2.0,         # Fingertips: 2-3mm discrimination
                freq_range_hz=(20, 500),    # RA + PC mechanoreceptors
                sensitivity=1.0,             # Baseline sensitivity
                rendering_tier=1,            # Requires full VCA synthesis
                cue_mask=FilteredContact.CUE_ALL  # All 5 cue types
            ),
            'thumb_tip': BodyPartProperties(
                spatial_res_mm=2.5,
                freq_range_hz=(20, 500),
                sensitivity=1.0,
                rendering_tier=1,
                cue_mask=FilteredContact.CUE_ALL
            ),
            'middle_tip': BodyPartProperties(
                spatial_res_mm=2.5,
                freq_range_hz=(20, 500),
                sensitivity=0.95,
                rendering_tier=1,
                cue_mask=FilteredContact.CUE_ALL
            ),
            'ring_tip': BodyPartProperties(
                spatial_res_mm=3.0,
                freq_range_hz=(20, 500),
                sensitivity=0.9,
                rendering_tier=1,
                cue_mask=FilteredContact.CUE_ALL
            ),
            'pinky_tip': BodyPartProperties(
                spatial_res_mm=3.5,
                freq_range_hz=(20, 500),
                sensitivity=0.85,
                rendering_tier=1,
                cue_mask=FilteredContact.CUE_ALL
            ),
            'palm_center': BodyPartProperties(
                spatial_res_mm=10.0,        # Palm: 8-12mm discrimination
                freq_range_hz=(30, 300),    # Lower resolution
                sensitivity=0.5,             # Less sensitive than fingertips
                rendering_tier=2,            # LRA acceptable
                cue_mask=make_cue_mask(FilteredContact.CUE_IMPACT,
                                      FilteredContact.CUE_TEXTURE,
                                      FilteredContact.CUE_WEIGHT)  # No shear/ring
            ),

            # ARM - Medium resolution, simplified cues
            'forearm': BodyPartProperties(
                spatial_res_mm=30.0,        # Forearm: 25-35mm discrimination
                freq_range_hz=(50, 200),
                sensitivity=0.3,
                rendering_tier=2,
                cue_mask=make_cue_mask(FilteredContact.CUE_IMPACT,
                                      FilteredContact.CUE_WEIGHT)
            ),
            'upper_arm': BodyPartProperties(
                spatial_res_mm=40.0,
                freq_range_hz=(50, 150),
                sensitivity=0.25,
                rendering_tier=3,            # ERM sufficient
                cue_mask=FilteredContact.CUE_IMPACT | FilteredContact.CUE_WEIGHT
            ),

            # TORSO - Low resolution, impact/pressure only
            'chest': BodyPartProperties(
                spatial_res_mm=45.0,        # Trunk: 40-50mm discrimination
                freq_range_hz=(50, 100),
                sensitivity=0.2,
                rendering_tier=3,
                cue_mask=FilteredContact.CUE_IMPACT | FilteredContact.CUE_WEIGHT
            ),
            'back': BodyPartProperties(
                spatial_res_mm=50.0,        # Back: poorest discrimination
                freq_range_hz=(50, 100),
                sensitivity=0.15,
                rendering_tier=3,
                cue_mask=FilteredContact.CUE_IMPACT | FilteredContact.CUE_WEIGHT
            ),

            # FEET - High pressure sensitivity, lower texture resolution
            'foot_sole': BodyPartProperties(
                spatial_res_mm=8.0,         # Foot sole: 8-10mm (better than palm)
                freq_range_hz=(30, 300),
                sensitivity=0.7,             # High pressure sensitivity
                rendering_tier=2,
                cue_mask=make_cue_mask(FilteredContact.CUE_IMPACT,
                                      FilteredContact.CUE_TEXTURE,
                                      FilteredContact.CUE_WEIGHT)
            ),
            'toes': BodyPartProperties(
                spatial_res_mm=5.0,
                freq_range_hz=(20, 400),
                sensitivity=0.8,
                rendering_tier=2,
                cue_mask=FilteredContact.CUE_ALL
            ),
        }

    def _load_config(self, config: Dict) -> Dict[str, BodyPartProperties]:
        """Load Homunculus from configuration dict."""
        table = {}
        for body_part, props in config.items():
            table[body_part] = BodyPartProperties(**props)
        return table

    def lookup(self, body_part_id: int) -> BodyPartProperties:
        """
        Get perceptual properties for a body part.

        Args:
            body_part_id: Numeric body part identifier from physics engine

        Returns:
            BodyPartProperties: Perceptual properties for this body part
        """
        # Map numeric ID to string key
        body_part_name = self._id_to_name(body_part_id)

        # Return properties or fallback
        return self.table.get(body_part_name, self._fallback_properties())

    def _id_to_name(self, body_part_id: int) -> str:
        """
        Map numeric body part ID to string name.

        TODO: Phase 2 - Load mapping from MuJoCo model metadata
        For now, use hardcoded mapping from existing hand model.
        """
        # Mapping from existing hand model geom IDs
        # These correspond to the 18-sensor detailed hand model
        id_map = {
            # Fingertips (distal segments)
            7: 'thumb_tip',      # thumb_geom
            10: 'index_tip',     # index_geom
            13: 'middle_tip',    # middle_geom
            16: 'ring_tip',      # ring_geom
            19: 'pinky_tip',     # pinky_geom

            # Middle segments
            6: 'thumb_mid',      # thumb_mid_geom
            9: 'index_mid',      # index_mid_geom
            12: 'middle_mid',    # middle_mid_geom
            15: 'ring_mid',      # ring_mid_geom
            18: 'pinky_mid',     # pinky_mid_geom

            # Proximal segments
            5: 'thumb_prox',     # thumb_prox_geom
            8: 'index_prox',     # index_prox_geom
            11: 'middle_prox',   # middle_prox_geom
            14: 'ring_prox',     # ring_prox_geom
            17: 'pinky_prox',    # pinky_prox_geom

            # Palm
            3: 'palm_center',    # palm_center_geom
        }

        name = id_map.get(body_part_id, f'unknown_{body_part_id}')

        # Map segment names to tip names for lookup
        # (Default table only has tip entries for now)
        if '_mid' in name or '_prox' in name:
            # Use tip properties for finger segments
            base_finger = name.split('_')[0]
            return f'{base_finger}_tip'

        return name

    def _fallback_properties(self) -> BodyPartProperties:
        """
        Fallback properties for unmapped body parts.

        Conservative settings: low sensitivity, minimal cues.
        """
        return BodyPartProperties(
            spatial_res_mm=40.0,         # Poor discrimination
            freq_range_hz=(50, 150),     # Limited frequency range
            sensitivity=0.2,              # Low sensitivity
            rendering_tier=3,             # ERM tier (simplest)
            cue_mask=FilteredContact.CUE_IMPACT | FilteredContact.CUE_WEIGHT  # Only basic cues
        )

    def save(self, filepath: str):
        """Save Homunculus configuration to file."""
        import json
        config = {
            name: {
                'spatial_res_mm': props.spatial_res_mm,
                'freq_range_hz': props.freq_range_hz,
                'sensitivity': props.sensitivity,
                'rendering_tier': props.rendering_tier,
                'cue_mask': props.cue_mask
            }
            for name, props in self.table.items()
        }
        with open(filepath, 'w') as f:
            json.dump(config, f, indent=2)
        logger.info(f"Saved Homunculus config to {filepath}")

    @staticmethod
    def load(filepath: str) -> 'Homunculus':
        """Load Homunculus configuration from file."""
        import json
        with open(filepath, 'r') as f:
            config = json.load(f)
        return Homunculus(config)


class SomatotopicRouter:
    """
    Routes ContactPatches based on biological perception model.

    Applies perceptual filtering:
    1. Sensitivity gating (force threshold based on body part)
    2. Rendering tier assignment (VCA/LRA/ERM capability)
    3. Cue mask application (enable/disable cue types)
    """

    def __init__(self, homunculus: Optional[Homunculus] = None):
        """
        Initialize router with Homunculus model.

        Args:
            homunculus: Optional custom Homunculus. If None, uses default.
        """
        self.homunculus = homunculus or Homunculus()
        self.contact_count = 0  # Statistics

    def route(self, patches: List[ContactPatch]) -> List[FilteredContact]:
        """
        Filter and tag contacts for neural rendering.

        Applies biological filtering based on Homunculus table:
        - Rejects contacts below perceptual threshold
        - Assigns rendering tier and cue mask
        - Preserves original ContactPatch for feature extraction

        Args:
            patches: List of ContactPatches from physics simulation

        Returns:
            List[FilteredContact]: Biologically-filtered contacts
        """
        filtered = []

        for patch in patches:
            # Lookup perceptual properties
            props = self.homunculus.lookup(patch.body_part_id)

            # Sensitivity gating: Reject contacts below perceptual threshold
            # Threshold scales inversely with sensitivity (higher sensitivity = lower threshold)
            force_threshold = 0.01 / props.sensitivity  # Base threshold: 0.01N

            if patch.force_normal < force_threshold:
                continue  # Below perceptual threshold for this body part

            # Create filtered contact
            filtered_contact = FilteredContact(
                patch=patch,
                rendering_tier=props.rendering_tier,
                cue_mask=props.cue_mask
            )

            filtered.append(filtered_contact)
            self.contact_count += 1

        if filtered:
            logger.debug(f"Routed {len(filtered)}/{len(patches)} contacts "
                        f"(total: {self.contact_count})")

        return filtered

    def reset_stats(self):
        """Reset statistics counters."""
        self.contact_count = 0

    def get_stats(self) -> Dict:
        """Get routing statistics."""
        return {
            'total_contacts_routed': self.contact_count
        }


# Convenience function for testing
def create_default_router() -> SomatotopicRouter:
    """Create router with default Homunculus."""
    return SomatotopicRouter(Homunculus())
