"""
Multi-contact physics engine for tracking multiple simultaneous contacts.
Extends the single-contact MuJoCoEngine to handle hand interactions.
"""

import mujoco
import numpy as np
from typing import Dict, List, Optional
from dataclasses import dataclass
from src.physics.mujoco_engine import ContactPatch as OldContactPatch, MuJoCoEngine
from src.core.schemas import ContactPatch


class MultiContactEngine(MuJoCoEngine):
    """
    Extended physics engine that tracks multiple contacts simultaneously.
    Each contact is identified by body part (e.g., "thumb", "index", "palm").
    """

    def __init__(self, model_path: str, max_contacts: int = 10):
        """
        Initialize multi-contact engine.

        Args:
            model_path: Path to MuJoCo XML model
            max_contacts: Maximum number of simultaneous contacts to track
        """
        super().__init__(model_path)

        self.max_contacts = max_contacts

        # Pre-allocate contact force arrays for all possible contacts
        self.contact_forces = [np.zeros(6) for _ in range(self.max_contacts)]

        # Build mapping from geom ID to body part name
        self.geom_to_body = self._build_geom_map()

        # Track floor geom ID to exclude from body part identification
        self.floor_geom_id = self._find_floor_geom()

        print(f"✓ Multi-contact engine initialized:")
        print(f"  Max contacts: {self.max_contacts}")
        print(f"  Body parts tracked: {len(self.geom_to_body)}")
        print(f"  Geom map: {self.geom_to_body}")

    def _build_geom_map(self) -> Dict[int, str]:
        """
        Build mapping from geom ID to body part name.

        Returns:
            Dict mapping geom_id -> body_part_name
        """
        geom_map = {}

        for i in range(self.model.ngeom):
            geom_name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, i
            )

            if geom_name is None:
                continue

            # Extract body part from geom name
            # Expected naming: "thumb_tip", "index_tip", "palm", etc.
            body_part = self._extract_body_part(geom_name)

            if body_part and body_part != "floor":
                geom_map[i] = body_part

        return geom_map

    def _extract_body_part(self, geom_name: str) -> Optional[str]:
        """
        Extract body part name from geom name.

        Examples:
            "thumb_tip" -> "thumb"
            "index_tip" -> "index"
            "palm" -> "palm"
            "floor" -> None (excluded)

        Args:
            geom_name: Name of geometry

        Returns:
            Body part name or None if should be excluded
        """
        if not geom_name:
            return None

        # Exclude floor/ground geometries
        if "floor" in geom_name.lower() or "ground" in geom_name.lower():
            return None

        # For fingertips, extract finger name
        if "_tip" in geom_name:
            return geom_name.replace("_tip", "")

        # For other parts (palm, etc.), use name as-is
        if geom_name in ["palm", "thumb", "index", "middle", "ring", "pinky"]:
            return geom_name

        # For "finger" geometry (single-contact fallback), return "fingertip"
        if geom_name == "finger":
            return "fingertip"

        return geom_name

    def _find_floor_geom(self) -> int:
        """Find the floor geom ID to exclude from contact identification."""
        for i in range(self.model.ngeom):
            geom_name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, i
            )
            if geom_name and ("floor" in geom_name.lower() or "ground" in geom_name.lower()):
                return i
        return 0  # Default to first geom if not found

    def step_multi(self) -> Dict[str, OldContactPatch]:
        """
        Run one physics step (1ms) and extract ALL contacts.

        Returns:
            Dict mapping body part name to OldContactPatch (legacy format).
            Empty dict if no contacts.
        """
        # Step simulation
        mujoco.mj_step(self.model, self.data)
        self.time_us += int(self.dt * 1e6)

        contact_dict = {}

        if self.data.ncon == 0:
            return contact_dict  # No contacts

        # Process each contact
        n_contacts = min(self.data.ncon, self.max_contacts)

        for i in range(n_contacts):
            contact = self.data.contact[i]

            # Calculate contact force
            mujoco.mj_contactForce(
                self.model, self.data, i, self.contact_forces[i]
            )

            # Extract normal and tangential forces
            normal_force = abs(self.contact_forces[i][0])
            shear_force = np.linalg.norm(self.contact_forces[i][1:3])

            # Identify which body part is in contact
            geom1_id = contact.geom1
            geom2_id = contact.geom2

            # Determine which geom is the hand (not floor)
            if geom1_id == self.floor_geom_id:
                hand_geom_id = geom2_id
            elif geom2_id == self.floor_geom_id:
                hand_geom_id = geom1_id
            else:
                # If neither is floor, prefer first geom
                hand_geom_id = geom1_id

            # Get body part name
            body_part = self.geom_to_body.get(hand_geom_id, f"unknown_{i}")

            # Get velocity and material properties
            slip_speed_mms = self._calculate_slip_speed(hand_geom_id)
            mu_static, mu_dynamic, solref_damping = self._get_material_props(
                hand_geom_id
            )

            # Create OldContactPatch for this contact (legacy format)
            patch = OldContactPatch(
                timestamp_us=self.time_us,
                normal_force_N=normal_force,
                shear_force_N=shear_force,
                slip_speed_mms=slip_speed_mms,
                contact_pos=contact.pos.copy(),
                contact_normal=contact.frame[:3].copy(),
                in_contact=True,
                mu_static=mu_static,
                mu_dynamic=mu_dynamic,
                solref_damping=solref_damping
            )

            contact_dict[body_part] = patch

        return contact_dict

    def _calculate_slip_speed(self, geom_id: int) -> float:
        """
        Calculate slip speed for a given geom.

        Args:
            geom_id: Geometry ID

        Returns:
            Slip speed in mm/s
        """
        try:
            # Get body ID from geom
            body_id = self.model.geom_bodyid[geom_id]

            # Get body's linear velocity
            body_vel = self.data.body(body_id).cvel[:3]

            # Convert to mm/s
            slip_speed_mms = np.linalg.norm(body_vel) * 1000

            return slip_speed_mms
        except Exception:
            return 0.0

    def _get_material_props(self, geom_id: int) -> tuple:
        """
        Get material properties for a given geom.

        Args:
            geom_id: Geometry ID

        Returns:
            (mu_static, mu_dynamic, solref_damping)
        """
        try:
            mu_static = self.model.geom_friction[geom_id, 0]
            mu_dynamic = self.model.geom_friction[geom_id, 1]
            solref_damping = self.model.geom_solref[geom_id, 1]
            return mu_static, mu_dynamic, solref_damping
        except Exception:
            return 0.0, 0.0, 0.0

    def step(self) -> OldContactPatch:
        """
        Backward compatibility: return single contact (strongest force).

        Returns:
            OldContactPatch for strongest contact, or no-contact patch
        """
        contacts = self.step_multi()

        if not contacts:
            # No contact - return zero patch
            return OldContactPatch(
                timestamp_us=self.time_us,
                normal_force_N=0.0,
                shear_force_N=0.0,
                slip_speed_mms=0.0,
                contact_pos=np.zeros(3),
                contact_normal=np.array([0, 0, 1]),
                in_contact=False,
                mu_static=0.0,
                mu_dynamic=0.0,
                solref_damping=0.0
            )

        # Return contact with highest normal force
        strongest_body_part = max(
            contacts.keys(),
            key=lambda k: contacts[k].normal_force_N
        )
        return contacts[strongest_body_part]

    def step_v2(self) -> List[ContactPatch]:
        """
        Run one physics step (1ms) and extract contacts in new schema format.

        This is the new HAPTOS Platform API that returns ContactPatch objects
        compatible with the Somatotopic Router.

        Returns:
            List[ContactPatch]: List of contacts using new schema format.
                               Empty list if no contacts.
        """
        # Step simulation
        mujoco.mj_step(self.model, self.data)
        self.time_us += int(self.dt * 1e6)

        patches = []

        if self.data.ncon == 0:
            return patches  # No contacts

        # Process each contact
        n_contacts = min(self.data.ncon, self.max_contacts)

        for i in range(n_contacts):
            contact = self.data.contact[i]

            # Calculate contact force
            mujoco.mj_contactForce(
                self.model, self.data, i, self.contact_forces[i]
            )

            # Extract forces
            normal_force = abs(self.contact_forces[i][0])
            shear_x = self.contact_forces[i][1]
            shear_y = self.contact_forces[i][2]

            # Identify which body part is in contact
            geom1_id = contact.geom1
            geom2_id = contact.geom2

            # Determine which geom is the hand (not floor)
            if geom1_id == self.floor_geom_id:
                hand_geom_id = geom2_id
            elif geom2_id == self.floor_geom_id:
                hand_geom_id = geom1_id
            else:
                # If neither is floor, prefer first geom
                hand_geom_id = geom1_id

            # Get velocity at contact point
            try:
                body_id = self.model.geom_bodyid[hand_geom_id]
                body_vel = self.data.body(body_id).cvel[:3]  # Linear velocity
                velocity = (float(body_vel[0]), float(body_vel[1]), float(body_vel[2]))
            except Exception:
                velocity = (0.0, 0.0, 0.0)

            # Estimate contact area (rough approximation)
            # Could be improved with actual contact geometry analysis
            contact_area = 0.0001  # Default 1 cm² for now

            # Get material hint (0 = unknown for now)
            # TODO: Add material ID mapping in Phase 2
            material_hint = 0

            # Create new ContactPatch
            patch = ContactPatch(
                body_part_id=hand_geom_id,
                force_normal=normal_force,
                force_shear=(shear_x, shear_y),
                velocity=velocity,
                contact_area=contact_area,
                material_hint=material_hint,
                timestamp_us=self.time_us
            )

            patches.append(patch)

        return patches
