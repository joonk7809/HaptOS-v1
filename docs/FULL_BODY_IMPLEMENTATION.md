# HAPTOS Full Body Implementation Guide

**Version**: 3.2
**Date**: January 2026
**Status**: Phase 5.1-5.2 Complete, Phase 5.3-5.4 Ready for Implementation
**Target**: Claude Code

---

## Overview

This document provides implementation specifications for scaling HAPTOS from hand-only to full body haptic rendering. Phases 5.1 (humanoid physics model) and 5.2 (full body homunculus) are complete. This guide covers the remaining work: scalable contact processing (5.3) and multi-driver architecture (5.4).

### Completed Components

```
/assets/hand_models/haptos_humanoid.xml   # Full body MuJoCo model (173 contact geoms)
/src/routing/humanoid_body_mapping.py      # Geom ID → body part mapping
/src/routing/full_body_homunculus.py       # 114 body part properties table
```

### Architecture Context

```
Layer 1: Physics Engine (1kHz)
    MuJoCo → ContactPatch[] → Router → FilteredContact[]

Layer 2: Neural Renderer (100Hz)
    FilteredContact[] → NeuralRenderer → SensationParams[]

Layer 3: Hardware Driver (2kHz+)
    SensationParams[] → Shaper → Translator → SynthesisCommand[]
```

The challenge: Layer 1 can now produce 50+ simultaneous contacts (full body on textured surface). The pipeline must handle this without exceeding latency budget.

---

## Phase 5.3: Scalable Contact Processing

### Problem Statement

Current system handles ~10 contacts (two hands). Full body scenarios:
- Standing: 2-10 contacts (feet only)
- Sitting: 10-30 contacts (feet + thighs + back)
- Lying on textured surface: 50-100+ contacts

At 100Hz inference with 7.5ms per contact, 50 contacts = 375ms. Unacceptable.

### Solution: Three-Tier Processing Strategy

```
Tier 1 Contacts (VCA body parts):     Full neural inference
Tier 2 Contacts (LRA body parts):     Batched inference, reduced rate
Tier 3 Contacts (ERM body parts):     Lookup table, skip neural inference
```

### 5.3.1 Contact Prioritization

**File**: `src/routing/contact_prioritizer.py`

Contacts compete for processing budget. Priority score determines which contacts get full inference vs simplified processing.

```python
@dataclass
class PrioritizedContact:
    contact: FilteredContact
    priority_score: float
    processing_tier: int  # 1=full, 2=batched, 3=lookup

class ContactPrioritizer:
    """
    Assigns priority scores and processing tiers to contacts.

    Priority Score = sensitivity × force × recency_bonus

    Where:
        sensitivity: From homunculus (0.15 - 1.0)
        force: Normalized force (0 - 1, clamped at 10N)
        recency_bonus: 1.5 if new contact, 1.0 if ongoing
    """

    def __init__(self, homunculus: FullBodyHomunculus, config: PrioritizerConfig):
        self.homunculus = homunculus
        self.config = config
        self.active_contacts: Dict[int, float] = {}  # body_part_id → first_seen_time

    def prioritize(self, contacts: List[FilteredContact],
                   timestamp: float) -> List[PrioritizedContact]:
        """
        Assign priorities and processing tiers to all contacts.

        Args:
            contacts: Raw filtered contacts from router
            timestamp: Current simulation time

        Returns:
            Contacts sorted by priority, with processing tier assigned
        """
        prioritized = []

        for contact in contacts:
            body_part = self._get_body_part_name(contact.patch.body_part_id)
            props = self.homunculus.lookup(body_part)

            # Calculate priority score
            sensitivity = props.sensitivity if props else 0.5
            force = min(contact.patch.force_normal / 10.0, 1.0)

            # Recency bonus for new contacts (impact detection)
            bid = contact.patch.body_part_id
            if bid not in self.active_contacts:
                self.active_contacts[bid] = timestamp
                recency_bonus = 1.5
            else:
                recency_bonus = 1.0

            priority = sensitivity * force * recency_bonus

            # Assign processing tier based on body part tier and priority
            if props and props.rendering_tier == RenderingTier.VCA:
                processing_tier = 1  # Always full inference for VCA parts
            elif priority > self.config.tier2_threshold:
                processing_tier = 2  # Batched inference
            else:
                processing_tier = 3  # Lookup table only

            prioritized.append(PrioritizedContact(
                contact=contact,
                priority_score=priority,
                processing_tier=processing_tier
            ))

        # Clean up stale contacts
        self._cleanup_stale(contacts, timestamp)

        # Sort by priority (highest first)
        prioritized.sort(key=lambda x: x.priority_score, reverse=True)

        return prioritized

    def _cleanup_stale(self, contacts: List[FilteredContact], timestamp: float):
        """Remove contacts not seen for >100ms."""
        current_ids = {c.patch.body_part_id for c in contacts}
        stale = [bid for bid in self.active_contacts
                 if bid not in current_ids and
                 timestamp - self.active_contacts[bid] > 0.1]
        for bid in stale:
            del self.active_contacts[bid]


@dataclass
class PrioritizerConfig:
    tier2_threshold: float = 0.3   # Priority above this gets batched inference
    max_tier1_contacts: int = 10   # Max contacts for full inference
    max_tier2_contacts: int = 20   # Max contacts for batched inference
```

### 5.3.2 Batched Neural Inference

**File**: `src/inference/batched_renderer.py`

Process multiple contacts in single forward pass. Reduces per-contact overhead.

```python
class BatchedNeuralRenderer:
    """
    Batched inference for multiple contacts.

    Instead of N forward passes, does 1 forward pass with batch_size=N.
    Reduces Python loop overhead and enables GPU parallelism.
    """

    def __init__(self, model: NeuralRenderer_v2, max_batch_size: int = 32):
        self.model = model
        self.max_batch_size = max_batch_size
        self.feature_extractor = FeatureExtractorV2()

    def infer_batch(self, contacts: List[PrioritizedContact]) -> Dict[int, SensationParams]:
        """
        Run inference on a batch of contacts.

        Args:
            contacts: Prioritized contacts (tier 1 and 2 only)

        Returns:
            Dict mapping body_part_id → SensationParams
        """
        if not contacts:
            return {}

        # Extract features for all contacts
        features_list = []
        cue_masks = []
        body_part_ids = []

        for pc in contacts[:self.max_batch_size]:
            features = self.feature_extractor.extract(pc.contact.patch)
            features_list.append(features)
            cue_masks.append(pc.contact.cue_mask)
            body_part_ids.append(pc.contact.patch.body_part_id)

        # Stack into batch tensor
        features_batch = np.stack(features_list, axis=0)  # [N, 14]
        features_tensor = torch.from_numpy(features_batch).float()

        # Single forward pass
        with torch.no_grad():
            sensations_batch = self.model.forward_batch(features_tensor, cue_masks)

        # Unpack results
        results = {}
        for i, bid in enumerate(body_part_ids):
            results[bid] = sensations_batch[i]

        return results


# Add to NeuralRenderer_v2:
class NeuralRenderer_v2(nn.Module):
    # ... existing code ...

    def forward_batch(self, features: torch.Tensor,
                      cue_masks: List[int]) -> List[SensationParams]:
        """
        Batched forward pass.

        Args:
            features: [N, 14] tensor
            cue_masks: List of N cue masks

        Returns:
            List of N SensationParams
        """
        batch_size = features.shape[0]

        # Trunk: shared computation
        trunk_out = self.trunk(features)  # [N, 64]

        # Heads: compute all, mask later
        impact_out = torch.sigmoid(self.impact_head(trunk_out))      # [N, 2]
        resonance_out = torch.sigmoid(self.resonance_head(trunk_out)) # [N, 3]
        texture_out = torch.sigmoid(self.texture_head(trunk_out))     # [N, 3]
        slip_out = self.slip_head(trunk_out)                          # [N, 4]
        pressure_out = torch.sigmoid(self.pressure_head(trunk_out))   # [N, 2]

        # Build SensationParams per contact
        results = []
        for i in range(batch_size):
            mask = cue_masks[i]
            s = SensationParams()

            if mask & CUE_IMPACT:
                s.impact_intensity = impact_out[i, 0].item()
                s.impact_sharpness = impact_out[i, 1].item()

            if mask & CUE_RESONANCE:
                s.resonance_intensity = resonance_out[i, 0].item()
                s.resonance_brightness = resonance_out[i, 1].item()
                s.resonance_sustain = resonance_out[i, 2].item()

            if mask & CUE_TEXTURE:
                s.texture_roughness = texture_out[i, 0].item()
                s.texture_density = texture_out[i, 1].item()
                s.texture_depth = texture_out[i, 2].item()

            if mask & CUE_SLIP:
                s.slip_speed = torch.sigmoid(slip_out[i, 0]).item()
                d = slip_out[i, 1:3]
                norm = torch.norm(d) + 1e-8
                s.slip_direction = (d[0].item()/norm.item(), d[1].item()/norm.item())
                s.slip_grip = torch.sigmoid(slip_out[i, 3]).item()

            if mask & CUE_PRESSURE:
                s.pressure_magnitude = pressure_out[i, 0].item()
                s.pressure_spread = pressure_out[i, 1].item()

            results.append(s)

        return results
```

### 5.3.3 Tier 3 Lookup Table (Skip Neural Inference)

**File**: `src/inference/sensation_lookup.py`

For low-sensitivity body parts (torso, thighs), skip neural inference entirely. Use precomputed lookup table.

```python
class SensationLookupTable:
    """
    Precomputed sensation parameters for Tier 3 (ERM) body parts.

    These parts only render IMPACT and PRESSURE, so we can use
    simple analytical functions instead of neural inference.

    Saves ~7.5ms per contact.
    """

    def __init__(self):
        # Precomputed response curves
        # impact_intensity = f(force, material_hardness)
        # pressure_magnitude = f(force)
        pass

    def lookup(self, contact: FilteredContact) -> SensationParams:
        """
        Generate SensationParams from lookup table.

        No neural inference. Pure analytical computation.
        ~0.01ms per contact.
        """
        s = SensationParams()

        force = contact.patch.force_normal
        material = contact.patch.material_hint

        # Impact: simple threshold + scaling
        # Sharp impact if force rose quickly (check force_delta in features)
        s.impact_intensity = min(force / 5.0, 1.0)
        s.impact_sharpness = self._estimate_sharpness(material)
        s.impact_trigger = False  # Set by onset detector, not here

        # Pressure: linear scaling
        s.pressure_magnitude = min(force / 10.0, 1.0)
        s.pressure_spread = 0.5  # Default for large body parts

        # All other channels zero (masked by cue_mask anyway)
        s.resonance_intensity = 0.0
        s.resonance_brightness = 0.0
        s.resonance_sustain = 0.0
        s.texture_roughness = 0.0
        s.texture_density = 0.0
        s.texture_depth = 0.0
        s.slip_speed = 0.0
        s.slip_direction = (0.0, 0.0)
        s.slip_grip = 0.5

        return s

    def _estimate_sharpness(self, material_hint: int) -> float:
        """Estimate impact sharpness from material."""
        # 0=unknown, 1=metal, 2=wood, 3=plastic, 4=rubber, 5=fabric
        sharpness_map = {
            0: 0.5,   # Unknown: medium
            1: 0.9,   # Metal: sharp
            2: 0.7,   # Wood: medium-sharp
            3: 0.6,   # Plastic: medium
            4: 0.3,   # Rubber: soft
            5: 0.2,   # Fabric: very soft
        }
        return sharpness_map.get(material_hint, 0.5)
```

### 5.3.4 Integrated Scalable Renderer

**File**: `src/inference/scalable_renderer.py`

Combines all three processing tiers into unified interface.

```python
class ScalableNeuralRenderer:
    """
    Scalable rendering pipeline for full-body haptics.

    Routes contacts to appropriate processing tier:
        Tier 1: Full neural inference (VCA body parts)
        Tier 2: Batched neural inference (LRA body parts, high priority)
        Tier 3: Lookup table (ERM body parts, low priority)

    Guarantees:
        - Total inference time < 15ms for up to 100 contacts
        - Tier 1 contacts always get full inference
        - Graceful degradation under load
    """

    def __init__(self,
                 model: NeuralRenderer_v2,
                 homunculus: FullBodyHomunculus,
                 config: Optional[ScalableRendererConfig] = None):
        self.config = config or ScalableRendererConfig()
        self.prioritizer = ContactPrioritizer(homunculus, self.config.prioritizer)
        self.batched_renderer = BatchedNeuralRenderer(model, self.config.max_batch_size)
        self.lookup_table = SensationLookupTable()
        self.onset_detector = OnsetDetector()

    def render(self, contacts: List[FilteredContact],
               timestamp: float) -> Dict[int, SensationParams]:
        """
        Render sensations for all contacts.

        Args:
            contacts: Filtered contacts from router
            timestamp: Current simulation time

        Returns:
            Dict mapping body_part_id → SensationParams
        """
        if not contacts:
            return {}

        # Step 1: Prioritize contacts
        prioritized = self.prioritizer.prioritize(contacts, timestamp)

        # Step 2: Separate by processing tier
        tier1 = [p for p in prioritized if p.processing_tier == 1]
        tier2 = [p for p in prioritized if p.processing_tier == 2]
        tier3 = [p for p in prioritized if p.processing_tier == 3]

        # Step 3: Process each tier
        results = {}

        # Tier 1: Full inference (capped at max_tier1)
        tier1_limited = tier1[:self.config.max_tier1_contacts]
        if tier1_limited:
            tier1_results = self.batched_renderer.infer_batch(tier1_limited)
            results.update(tier1_results)

        # Tier 2: Batched inference (capped at max_tier2)
        tier2_limited = tier2[:self.config.max_tier2_contacts]
        if tier2_limited:
            tier2_results = self.batched_renderer.infer_batch(tier2_limited)
            results.update(tier2_results)

        # Tier 3: Lookup table (no limit)
        for pc in tier3:
            bid = pc.contact.patch.body_part_id
            results[bid] = self.lookup_table.lookup(pc.contact)

        # Step 4: Onset detection (all contacts)
        for pc in prioritized:
            bid = pc.contact.patch.body_part_id
            if bid in results:
                results[bid].impact_trigger = self.onset_detector.detect(pc.contact)

        return results


@dataclass
class ScalableRendererConfig:
    max_tier1_contacts: int = 10
    max_tier2_contacts: int = 20
    max_batch_size: int = 32
    prioritizer: PrioritizerConfig = field(default_factory=PrioritizerConfig)
```

### 5.3.5 Performance Targets

```
Scenario              Contacts   Tier1   Tier2   Tier3   Target Time
────────────────────────────────────────────────────────────────────
Standing              5          2       3       0       < 3ms
Sitting               25         4       12      9       < 8ms
Lying (textured)      80         6       20      54      < 12ms
Max load              100        10      20      70      < 15ms
```

### 5.3.6 Tests

**File**: `tests/phase5/test_scalable_renderer.py`

```python
def test_prioritizer_ordering():
    """Fingertip contacts should rank higher than torso contacts."""

def test_tier_assignment():
    """VCA parts → tier 1, low priority → tier 3."""

def test_batched_inference_correctness():
    """Batched output matches single-contact output."""

def test_lookup_table_speed():
    """Lookup table < 0.1ms per contact."""

def test_scalable_renderer_latency():
    """100 contacts processed in < 15ms."""

def test_graceful_degradation():
    """Tier 1 contacts always processed, even at max load."""
```

---

## Phase 5.4: Multi-Driver Architecture

### Problem Statement

Single Teensy 4.1 can drive ~8 actuators with current firmware. Full body suit needs 50+ actuators distributed across body segments. Need multi-driver coordination.

### Solution: Segmented Driver Network

```
Host (Python) ──────┬─────────────────┬─────────────────┐
                    │                 │                 │
              [Serial 0]        [Serial 1]        [Serial 2]
                    │                 │                 │
              ┌─────▼─────┐    ┌─────▼─────┐    ┌─────▼─────┐
              │  Teensy   │    │  Teensy   │    │  Teensy   │
              │  (Torso)  │    │ (L. Arm)  │    │ (R. Arm)  │
              └───────────┘    └───────────┘    └───────────┘
                    │                 │                 │
              [8 actuators]    [10 actuators]   [10 actuators]
```

### 5.4.1 Body Segment Definition

**File**: `src/hardware/body_segments.py`

```python
from enum import IntEnum
from typing import Dict, List, Set


class BodySegment(IntEnum):
    """Body segments, each driven by one Teensy."""
    HEAD = 0
    TORSO = 1
    LEFT_ARM = 2
    RIGHT_ARM = 3
    LEFT_LEG = 4
    RIGHT_LEG = 5


# Mapping from body part to segment
BODY_PART_TO_SEGMENT: Dict[str, BodySegment] = {
    # Head
    "head_lips": BodySegment.HEAD,
    "head_forehead": BodySegment.HEAD,
    "head_cheek_left": BodySegment.HEAD,
    "head_cheek_right": BodySegment.HEAD,
    "head_nose": BodySegment.HEAD,
    "head_chin": BodySegment.HEAD,
    "head_ear_left": BodySegment.HEAD,
    "head_ear_right": BodySegment.HEAD,
    "head_top": BodySegment.HEAD,
    "head_back": BodySegment.HEAD,
    "head_side_left": BodySegment.HEAD,
    "head_side_right": BodySegment.HEAD,

    # Torso (includes pelvis)
    "torso_chest_upper": BodySegment.TORSO,
    "torso_chest_lower": BodySegment.TORSO,
    "torso_abdomen_upper": BodySegment.TORSO,
    "torso_abdomen_lower": BodySegment.TORSO,
    "torso_back_upper": BodySegment.TORSO,
    "torso_back_lower": BodySegment.TORSO,
    "torso_side_left": BodySegment.TORSO,
    "torso_side_right": BodySegment.TORSO,
    "pelvis_front": BodySegment.TORSO,
    "pelvis_back": BodySegment.TORSO,
    "pelvis_side_left": BodySegment.TORSO,
    "pelvis_side_right": BodySegment.TORSO,

    # Left arm (shoulder to fingertips)
    "left_shoulder": BodySegment.LEFT_ARM,
    "left_upper_arm": BodySegment.LEFT_ARM,
    "left_upper_arm_inner": BodySegment.LEFT_ARM,
    "left_elbow": BodySegment.LEFT_ARM,
    "left_forearm": BodySegment.LEFT_ARM,
    "left_forearm_inner": BodySegment.LEFT_ARM,
    "left_wrist": BodySegment.LEFT_ARM,
    # ... all left hand parts ...
    **{f"left_{p}": BodySegment.LEFT_ARM for p in [
        "palm", "palm_heel", "hand_back",
        "thumb_base", "thumb_mid", "thumb_tip", "thumb_nail",
        "index_base", "index_mid", "index_tip", "index_nail",
        "middle_base", "middle_mid", "middle_tip", "middle_nail",
        "ring_base", "ring_mid", "ring_tip", "ring_nail",
        "pinky_base", "pinky_mid", "pinky_tip", "pinky_nail",
    ]},

    # Right arm (mirror of left)
    "right_shoulder": BodySegment.RIGHT_ARM,
    "right_upper_arm": BodySegment.RIGHT_ARM,
    "right_upper_arm_inner": BodySegment.RIGHT_ARM,
    "right_elbow": BodySegment.RIGHT_ARM,
    "right_forearm": BodySegment.RIGHT_ARM,
    "right_forearm_inner": BodySegment.RIGHT_ARM,
    "right_wrist": BodySegment.RIGHT_ARM,
    **{f"right_{p}": BodySegment.RIGHT_ARM for p in [
        "palm", "palm_heel", "hand_back",
        "thumb_base", "thumb_mid", "thumb_tip", "thumb_nail",
        "index_base", "index_mid", "index_tip", "index_nail",
        "middle_base", "middle_mid", "middle_tip", "middle_nail",
        "ring_base", "ring_mid", "ring_tip", "ring_nail",
        "pinky_base", "pinky_mid", "pinky_tip", "pinky_nail",
    ]},

    # Left leg (hip to toes)
    **{f"left_{p}": BodySegment.LEFT_LEG for p in [
        "thigh_front", "thigh_back", "thigh_outer", "thigh_inner",
        "knee", "knee_back", "shin", "calf", "ankle",
        "heel", "sole_arch", "sole_ball", "foot_top",
        "toe_big", "toe_other",
    ]},

    # Right leg (mirror of left)
    **{f"right_{p}": BodySegment.RIGHT_LEG for p in [
        "thigh_front", "thigh_back", "thigh_outer", "thigh_inner",
        "knee", "knee_back", "shin", "calf", "ankle",
        "heel", "sole_arch", "sole_ball", "foot_top",
        "toe_big", "toe_other",
    ]},
}


def get_segment(body_part: str) -> BodySegment:
    """Get the body segment for a body part."""
    return BODY_PART_TO_SEGMENT.get(body_part, BodySegment.TORSO)


def get_parts_for_segment(segment: BodySegment) -> List[str]:
    """Get all body parts in a segment."""
    return [bp for bp, seg in BODY_PART_TO_SEGMENT.items() if seg == segment]
```

### 5.4.2 Multi-Driver Manager

**File**: `src/hardware/multi_driver_manager.py`

```python
import serial
import threading
from typing import Dict, List, Optional
from queue import Queue
from dataclasses import dataclass


@dataclass
class DriverConfig:
    segment: BodySegment
    port: str                    # e.g., "/dev/ttyUSB0"
    baud_rate: int = 115200
    actuator_count: int = 8


class SegmentDriver:
    """
    Driver for a single body segment (one Teensy).

    Handles serial communication and packet routing for
    all body parts in its segment.
    """

    def __init__(self, config: DriverConfig):
        self.config = config
        self.segment = config.segment
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        self.send_queue: Queue = Queue(maxsize=100)
        self._sender_thread: Optional[threading.Thread] = None

    def connect(self) -> bool:
        """Open serial connection to Teensy."""
        try:
            self.serial = serial.Serial(
                port=self.config.port,
                baudrate=self.config.baud_rate,
                timeout=0.01
            )
            self.connected = True
            self._start_sender_thread()
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to {self.config.port}: {e}")
            return False

    def disconnect(self):
        """Close serial connection."""
        self.connected = False
        if self._sender_thread:
            self._sender_thread.join(timeout=1.0)
        if self.serial:
            self.serial.close()

    def send(self, body_part_id: int, sensation: SensationParams):
        """
        Queue a sensation packet for transmission.

        Non-blocking. Drops packet if queue is full.
        """
        if self.connected:
            packet = self._serialize(body_part_id, sensation)
            try:
                self.send_queue.put_nowait(packet)
            except:
                pass  # Queue full, drop packet

    def _serialize(self, body_part_id: int, sensation: SensationParams) -> bytes:
        """Serialize SensationParams to 38-byte packet."""
        # Header
        data = bytearray([0xBB, 0x66, 0x01, body_part_id & 0xFF])

        # Timestamp (4 bytes)
        ts = sensation.timestamp_us & 0xFFFFFFFF
        data.extend(ts.to_bytes(4, 'little'))

        # Pack float16 values (28 bytes)
        import struct
        values = [
            sensation.impact_intensity,
            sensation.impact_sharpness,
            sensation.resonance_intensity,
            sensation.resonance_brightness,
            sensation.resonance_sustain,
            sensation.texture_roughness,
            sensation.texture_density,
            sensation.texture_depth,
            sensation.slip_speed,
            sensation.slip_direction[0],
            sensation.slip_direction[1],
            sensation.slip_grip,
            sensation.pressure_magnitude,
            sensation.pressure_spread,
        ]
        for v in values:
            data.extend(struct.pack('<e', v))  # float16 little-endian

        # Flags (1 byte)
        flags = 0x01 if sensation.impact_trigger else 0x00
        data.append(flags)

        # Checksum (1 byte)
        checksum = 0
        for b in data:
            checksum ^= b
        data.append(checksum)

        return bytes(data)

    def _start_sender_thread(self):
        """Start background thread for serial transmission."""
        def sender_loop():
            while self.connected:
                try:
                    packet = self.send_queue.get(timeout=0.01)
                    if self.serial and self.serial.is_open:
                        self.serial.write(packet)
                except:
                    pass

        self._sender_thread = threading.Thread(target=sender_loop, daemon=True)
        self._sender_thread.start()


class MultiDriverManager:
    """
    Manages multiple segment drivers for full-body haptics.

    Routes SensationParams to the correct driver based on body part.
    """

    def __init__(self, configs: List[DriverConfig]):
        self.drivers: Dict[BodySegment, SegmentDriver] = {}
        for config in configs:
            self.drivers[config.segment] = SegmentDriver(config)

        self._body_part_to_id: Dict[str, int] = {}
        self._build_body_part_map()

    def _build_body_part_map(self):
        """Build mapping from body part name to local ID (0-255)."""
        # Each segment has its own ID space
        for segment in BodySegment:
            parts = get_parts_for_segment(segment)
            for i, part in enumerate(parts):
                self._body_part_to_id[part] = i

    def connect_all(self) -> Dict[BodySegment, bool]:
        """Connect to all configured drivers."""
        results = {}
        for segment, driver in self.drivers.items():
            results[segment] = driver.connect()
        return results

    def disconnect_all(self):
        """Disconnect all drivers."""
        for driver in self.drivers.values():
            driver.disconnect()

    def send(self, sensations: Dict[str, SensationParams]):
        """
        Send sensations to appropriate drivers.

        Args:
            sensations: Dict mapping body_part_name → SensationParams
        """
        # Group by segment
        by_segment: Dict[BodySegment, List[tuple]] = {s: [] for s in BodySegment}

        for body_part, sensation in sensations.items():
            segment = get_segment(body_part)
            local_id = self._body_part_to_id.get(body_part, 0)
            by_segment[segment].append((local_id, sensation))

        # Send to each driver
        for segment, packets in by_segment.items():
            driver = self.drivers.get(segment)
            if driver and driver.connected:
                for local_id, sensation in packets:
                    driver.send(local_id, sensation)

    def get_status(self) -> Dict[BodySegment, bool]:
        """Get connection status of all drivers."""
        return {seg: drv.connected for seg, drv in self.drivers.items()}
```

### 5.4.3 Firmware Update Requirements

**File**: `firmware/FIRMWARE_SPEC.md`

Each Teensy needs updated firmware to:

1. **Accept segment ID** in initialization handshake
2. **Map local body part IDs** to actuator channels
3. **Handle multi-body-part packets** (8-10 body parts per Teensy)
4. **Report status** back to host (connected, actuator health)

```cpp
// firmware/teensy/config.h

#define SEGMENT_ID 2  // Set per device: 0=HEAD, 1=TORSO, 2=LEFT_ARM, etc.

// Body part ID → actuator channel mapping
// This is segment-specific and configured at flash time
const uint8_t BODY_PART_TO_ACTUATOR[32] = {
    // For LEFT_ARM segment:
    0,  // left_shoulder → actuator 0
    1,  // left_upper_arm → actuator 1
    1,  // left_upper_arm_inner → actuator 1 (shared)
    2,  // left_elbow → actuator 2
    3,  // left_forearm → actuator 3
    3,  // left_forearm_inner → actuator 3 (shared)
    4,  // left_wrist → actuator 4
    5,  // left_palm → actuator 5
    // ... fingertips share actuators in glove configuration
};

// Actuator count for this segment
#define NUM_ACTUATORS 10
```

### 5.4.4 Bandwidth Analysis

```
Per-segment bandwidth:
    38 bytes × 100 Hz × 10 body parts = 38 KB/s

Serial capacity:
    115200 baud = 14.4 KB/s (with overhead)

Solution:
    - Reduce rate for Tier 3 contacts to 50 Hz
    - Use 230400 baud for arm segments (high actuator density)
    - Drop stale packets if queue backs up

Revised estimate:
    Tier 1: 38 bytes × 100 Hz × 2 parts = 7.6 KB/s
    Tier 2: 38 bytes × 100 Hz × 4 parts = 15.2 KB/s
    Tier 3: 38 bytes × 50 Hz × 4 parts = 7.6 KB/s
    Total per segment: ~30 KB/s

    With 230400 baud: 28.8 KB/s capacity (tight but feasible)
```

### 5.4.5 Tests

**File**: `tests/phase5/test_multi_driver.py`

```python
def test_segment_mapping():
    """All 114 body parts map to exactly one segment."""

def test_driver_connect_disconnect():
    """Driver connects and disconnects cleanly."""

def test_packet_routing():
    """Sensations route to correct segment driver."""

def test_bandwidth_limit():
    """Manager handles bandwidth limiting gracefully."""

def test_driver_failure_isolation():
    """One driver failure doesn't affect others."""
```

---

## Implementation Order

```
Week 1-2: Phase 5.3 Core
    1. contact_prioritizer.py
    2. sensation_lookup.py
    3. Add forward_batch() to neural_renderer_v2.py
    4. batched_renderer.py
    5. scalable_renderer.py
    6. Tests for all above

Week 3: Phase 5.3 Integration
    7. Update somatotopic_router.py for full body
    8. Integration tests with humanoid model
    9. Performance benchmarks (100 contacts < 15ms)

Week 4: Phase 5.4 Core
    10. body_segments.py
    11. segment_driver.py
    12. multi_driver_manager.py
    13. Tests for all above

Week 5: Phase 5.4 Integration
    14. main_fullbody.py
    15. Firmware spec finalization
    16. End-to-end test (simulated drivers)

Week 6: Hardware Integration (if hardware available)
    17. Flash firmware to Teensys
    18. Connect physical drivers
    19. Full body validation
```

---

## Success Criteria

### Phase 5.3 Complete When:
- [ ] 100 contacts processed in < 15ms
- [ ] Tier 1 contacts always get full inference
- [ ] Batched inference matches single-contact output
- [ ] Lookup table used for Tier 3 contacts
- [ ] All unit tests pass

### Phase 5.4 Complete When:
- [ ] All 114 body parts map to a segment
- [ ] Multi-driver manager routes packets correctly
- [ ] Bandwidth stays within serial limits
- [ ] Driver failure isolated (doesn't crash system)
- [ ] All unit tests pass

### Full Body Complete When:
- [ ] Humanoid model contacts detected correctly
- [ ] Full pipeline latency < 20ms end-to-end
- [ ] Graceful degradation under max load
- [ ] Integration tests pass with simulated drivers
