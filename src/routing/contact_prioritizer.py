#!/usr/bin/env python3
"""
Contact Prioritizer - Scalable Contact Processing for Full Body Haptics.

Assigns priority scores and processing tiers to contacts based on:
- Body part sensitivity
- Contact force
- Recency (new vs ongoing contacts)

Enables graceful degradation when contact count exceeds processing budget.
"""

from typing import Dict, List
from dataclasses import dataclass, field
import logging

from src.core.schemas import FilteredContact
from src.routing.somatotopic_router import Homunculus

logger = logging.getLogger(__name__)


@dataclass
class PrioritizedContact:
    """
    Contact with assigned priority score and processing tier.

    Processing tiers:
        1: Full neural inference (VCA body parts, high priority)
        2: Batched neural inference (LRA body parts, medium priority)
        3: Lookup table only (ERM body parts, low priority)
    """
    contact: FilteredContact
    priority_score: float
    processing_tier: int


@dataclass
class PrioritizerConfig:
    """Configuration for contact prioritization."""
    tier2_threshold: float = 0.3   # Priority above this gets batched inference
    max_tier1_contacts: int = 10   # Max contacts for full inference
    max_tier2_contacts: int = 20   # Max contacts for batched inference
    recency_bonus: float = 1.5     # Multiplier for new contacts (impact detection)
    force_normalization: float = 10.0  # Force [N] that maps to priority=1.0
    stale_timeout_sec: float = 0.1     # Remove contacts not seen for >100ms


class ContactPrioritizer:
    """
    Assigns priority scores and processing tiers to contacts.

    Priority Score Calculation:
        priority = sensitivity × force_normalized × recency_bonus

    Where:
        sensitivity: From homunculus (0.15 - 1.0)
        force_normalized: contact_force / force_normalization (clamped to [0,1])
        recency_bonus: 1.5 if new contact, 1.0 if ongoing

    Tier Assignment:
        - VCA body parts (rendering_tier=1) → always tier 1
        - High priority (> tier2_threshold) → tier 2
        - Low priority → tier 3

    Example priorities:
        - Fingertip, 5N, new: 1.0 × 0.5 × 1.5 = 0.75 → Tier 1
        - Palm, 3N, ongoing: 0.5 × 0.3 × 1.0 = 0.15 → Tier 3
        - Chest, 8N, new: 0.2 × 0.8 × 1.5 = 0.24 → Tier 3
    """

    def __init__(self, homunculus: Homunculus, config: PrioritizerConfig = None):
        """
        Initialize prioritizer.

        Args:
            homunculus: Body part properties lookup table
            config: Prioritization configuration
        """
        self.homunculus = homunculus
        self.config = config or PrioritizerConfig()

        # Track active contacts for recency detection
        # Maps body_part_id → timestamp when first seen
        self.active_contacts: Dict[int, float] = {}

        logger.info(f"ContactPrioritizer initialized: "
                   f"tier2_threshold={self.config.tier2_threshold}, "
                   f"max_tier1={self.config.max_tier1_contacts}, "
                   f"max_tier2={self.config.max_tier2_contacts}")

    def prioritize(self,
                   contacts: List[FilteredContact],
                   timestamp: float) -> List[PrioritizedContact]:
        """
        Assign priorities and processing tiers to all contacts.

        Args:
            contacts: Raw filtered contacts from router
            timestamp: Current simulation time [seconds]

        Returns:
            Contacts sorted by priority (highest first), with processing tier assigned
        """
        if not contacts:
            return []

        prioritized = []

        for contact in contacts:
            # Get body part properties
            body_part_id = contact.patch.body_part_id
            props = self.homunculus.lookup(body_part_id)

            # Calculate priority score
            sensitivity = props.sensitivity if props else 0.5
            force_normalized = min(
                contact.patch.force_normal / self.config.force_normalization,
                1.0
            )

            # Recency bonus for new contacts (helps with impact detection)
            if body_part_id not in self.active_contacts:
                self.active_contacts[body_part_id] = timestamp
                recency = self.config.recency_bonus
            else:
                recency = 1.0

            priority = sensitivity * force_normalized * recency

            # Assign processing tier
            # Rule 1: VCA body parts always get full inference
            if props and props.rendering_tier == 1:
                processing_tier = 1
            # Rule 2: High priority contacts get batched inference
            elif priority > self.config.tier2_threshold:
                processing_tier = 2
            # Rule 3: Low priority contacts use lookup table
            else:
                processing_tier = 3

            prioritized.append(PrioritizedContact(
                contact=contact,
                priority_score=priority,
                processing_tier=processing_tier
            ))

        # Clean up stale contacts (not seen for > timeout)
        self._cleanup_stale(contacts, timestamp)

        # Sort by priority (highest first)
        prioritized.sort(key=lambda x: x.priority_score, reverse=True)

        # Log tier distribution
        tier1_count = sum(1 for p in prioritized if p.processing_tier == 1)
        tier2_count = sum(1 for p in prioritized if p.processing_tier == 2)
        tier3_count = sum(1 for p in prioritized if p.processing_tier == 3)

        logger.debug(f"Prioritized {len(contacts)} contacts: "
                    f"tier1={tier1_count}, tier2={tier2_count}, tier3={tier3_count}")

        return prioritized

    def _cleanup_stale(self, contacts: List[FilteredContact], timestamp: float):
        """
        Remove contacts not seen for > timeout.

        Prevents active_contacts dict from growing unbounded.
        """
        current_ids = {c.patch.body_part_id for c in contacts}
        stale = [
            bid for bid, first_seen in self.active_contacts.items()
            if bid not in current_ids and
            timestamp - first_seen > self.config.stale_timeout_sec
        ]

        for bid in stale:
            del self.active_contacts[bid]

        if stale:
            logger.debug(f"Cleaned up {len(stale)} stale contacts")

    def reset(self):
        """Reset prioritizer state. Useful for testing or scene changes."""
        self.active_contacts.clear()
        logger.debug("Prioritizer state reset")
