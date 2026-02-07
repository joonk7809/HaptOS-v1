#!/usr/bin/env python3
"""
Body Segment Definitions - Hardware Driver Segmentation.

Maps 114 body parts to 6 body segments for distributed multi-driver architecture.
Each segment is controlled by one Teensy 4.1 (8-10 actuators per segment).

Segment Layout:
    HEAD:      12 parts (forehead, cheeks, nose, chin, ears, top, back, sides)
    TORSO:     12 parts (chest, abdomen, back, sides, pelvis)
    LEFT_ARM:  28 parts (shoulder, upper arm, elbow, forearm, wrist, hand, fingers)
    RIGHT_ARM: 28 parts (mirror of left arm)
    LEFT_LEG:  17 parts (thigh, knee, shin, calf, ankle, foot, toes)
    RIGHT_LEG: 17 parts (mirror of left leg)

Total: 114 body parts across 6 segments
"""

from enum import IntEnum
from typing import Dict, List

class BodySegment(IntEnum):
    """
    Body segments, each driven by one Teensy.

    Each segment has its own serial connection and actuator set.
    """
    HEAD = 0
    TORSO = 1
    LEFT_ARM = 2
    RIGHT_ARM = 3
    LEFT_LEG = 4
    RIGHT_LEG = 5


# Complete mapping from body part name to segment
BODY_PART_TO_SEGMENT: Dict[str, BodySegment] = {
    # ===== HEAD (12 parts) =====
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

    # ===== TORSO (12 parts) =====
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

    # ===== LEFT ARM (28 parts) =====
    # Shoulder to wrist
    "left_shoulder": BodySegment.LEFT_ARM,
    "left_upper_arm": BodySegment.LEFT_ARM,
    "left_upper_arm_inner": BodySegment.LEFT_ARM,
    "left_elbow": BodySegment.LEFT_ARM,
    "left_forearm": BodySegment.LEFT_ARM,
    "left_forearm_inner": BodySegment.LEFT_ARM,
    "left_wrist": BodySegment.LEFT_ARM,

    # Hand
    "left_palm": BodySegment.LEFT_ARM,
    "left_palm_heel": BodySegment.LEFT_ARM,
    "left_hand_back": BodySegment.LEFT_ARM,

    # Thumb (4 parts)
    "left_thumb_base": BodySegment.LEFT_ARM,
    "left_thumb_mid": BodySegment.LEFT_ARM,
    "left_thumb_tip": BodySegment.LEFT_ARM,
    "left_thumb_nail": BodySegment.LEFT_ARM,

    # Index finger (4 parts)
    "left_index_base": BodySegment.LEFT_ARM,
    "left_index_mid": BodySegment.LEFT_ARM,
    "left_index_tip": BodySegment.LEFT_ARM,
    "left_index_nail": BodySegment.LEFT_ARM,

    # Middle finger (4 parts)
    "left_middle_base": BodySegment.LEFT_ARM,
    "left_middle_mid": BodySegment.LEFT_ARM,
    "left_middle_tip": BodySegment.LEFT_ARM,
    "left_middle_nail": BodySegment.LEFT_ARM,

    # Ring finger (4 parts)
    "left_ring_base": BodySegment.LEFT_ARM,
    "left_ring_mid": BodySegment.LEFT_ARM,
    "left_ring_tip": BodySegment.LEFT_ARM,
    "left_ring_nail": BodySegment.LEFT_ARM,

    # Pinky finger (4 parts)
    "left_pinky_base": BodySegment.LEFT_ARM,
    "left_pinky_mid": BodySegment.LEFT_ARM,
    "left_pinky_tip": BodySegment.LEFT_ARM,
    "left_pinky_nail": BodySegment.LEFT_ARM,

    # ===== RIGHT ARM (28 parts - mirror of left) =====
    "right_shoulder": BodySegment.RIGHT_ARM,
    "right_upper_arm": BodySegment.RIGHT_ARM,
    "right_upper_arm_inner": BodySegment.RIGHT_ARM,
    "right_elbow": BodySegment.RIGHT_ARM,
    "right_forearm": BodySegment.RIGHT_ARM,
    "right_forearm_inner": BodySegment.RIGHT_ARM,
    "right_wrist": BodySegment.RIGHT_ARM,

    "right_palm": BodySegment.RIGHT_ARM,
    "right_palm_heel": BodySegment.RIGHT_ARM,
    "right_hand_back": BodySegment.RIGHT_ARM,

    "right_thumb_base": BodySegment.RIGHT_ARM,
    "right_thumb_mid": BodySegment.RIGHT_ARM,
    "right_thumb_tip": BodySegment.RIGHT_ARM,
    "right_thumb_nail": BodySegment.RIGHT_ARM,

    "right_index_base": BodySegment.RIGHT_ARM,
    "right_index_mid": BodySegment.RIGHT_ARM,
    "right_index_tip": BodySegment.RIGHT_ARM,
    "right_index_nail": BodySegment.RIGHT_ARM,

    "right_middle_base": BodySegment.RIGHT_ARM,
    "right_middle_mid": BodySegment.RIGHT_ARM,
    "right_middle_tip": BodySegment.RIGHT_ARM,
    "right_middle_nail": BodySegment.RIGHT_ARM,

    "right_ring_base": BodySegment.RIGHT_ARM,
    "right_ring_mid": BodySegment.RIGHT_ARM,
    "right_ring_tip": BodySegment.RIGHT_ARM,
    "right_ring_nail": BodySegment.RIGHT_ARM,

    "right_pinky_base": BodySegment.RIGHT_ARM,
    "right_pinky_mid": BodySegment.RIGHT_ARM,
    "right_pinky_tip": BodySegment.RIGHT_ARM,
    "right_pinky_nail": BodySegment.RIGHT_ARM,

    # ===== LEFT LEG (17 parts) =====
    "left_thigh_front": BodySegment.LEFT_LEG,
    "left_thigh_back": BodySegment.LEFT_LEG,
    "left_thigh_outer": BodySegment.LEFT_LEG,
    "left_thigh_inner": BodySegment.LEFT_LEG,
    "left_knee": BodySegment.LEFT_LEG,
    "left_knee_back": BodySegment.LEFT_LEG,
    "left_shin": BodySegment.LEFT_LEG,
    "left_calf": BodySegment.LEFT_LEG,
    "left_ankle": BodySegment.LEFT_LEG,
    "left_heel": BodySegment.LEFT_LEG,
    "left_sole_arch": BodySegment.LEFT_LEG,
    "left_sole_ball": BodySegment.LEFT_LEG,
    "left_foot_top": BodySegment.LEFT_LEG,
    "left_toe_big": BodySegment.LEFT_LEG,
    "left_toe_other": BodySegment.LEFT_LEG,
    "left_foot_inner": BodySegment.LEFT_LEG,
    "left_foot_outer": BodySegment.LEFT_LEG,

    # ===== RIGHT LEG (17 parts - mirror of left) =====
    "right_thigh_front": BodySegment.RIGHT_LEG,
    "right_thigh_back": BodySegment.RIGHT_LEG,
    "right_thigh_outer": BodySegment.RIGHT_LEG,
    "right_thigh_inner": BodySegment.RIGHT_LEG,
    "right_knee": BodySegment.RIGHT_LEG,
    "right_knee_back": BodySegment.RIGHT_LEG,
    "right_shin": BodySegment.RIGHT_LEG,
    "right_calf": BodySegment.RIGHT_LEG,
    "right_ankle": BodySegment.RIGHT_LEG,
    "right_heel": BodySegment.RIGHT_LEG,
    "right_sole_arch": BodySegment.RIGHT_LEG,
    "right_sole_ball": BodySegment.RIGHT_LEG,
    "right_foot_top": BodySegment.RIGHT_LEG,
    "right_toe_big": BodySegment.RIGHT_LEG,
    "right_toe_other": BodySegment.RIGHT_LEG,
    "right_foot_inner": BodySegment.RIGHT_LEG,
    "right_foot_outer": BodySegment.RIGHT_LEG,
}


def get_segment(body_part: str) -> BodySegment:
    """
    Get the body segment for a body part.

    Args:
        body_part: Body part name (e.g., "left_index_tip")

    Returns:
        BodySegment enum value

    Raises:
        KeyError: If body part not found (defaults to TORSO)
    """
    return BODY_PART_TO_SEGMENT.get(body_part, BodySegment.TORSO)


def get_parts_for_segment(segment: BodySegment) -> List[str]:
    """
    Get all body parts in a segment.

    Args:
        segment: Body segment enum

    Returns:
        List of body part names in this segment
    """
    return [bp for bp, seg in BODY_PART_TO_SEGMENT.items() if seg == segment]


def get_segment_count() -> Dict[BodySegment, int]:
    """
    Get count of body parts per segment.

    Returns:
        Dict mapping segment → part count
    """
    counts = {seg: 0 for seg in BodySegment}
    for segment in BODY_PART_TO_SEGMENT.values():
        counts[segment] += 1
    return counts


def validate_coverage() -> bool:
    """
    Validate that all expected body parts are mapped.

    Returns:
        True if all 114 parts are mapped, False otherwise
    """
    expected_count = 114
    actual_count = len(BODY_PART_TO_SEGMENT)

    if actual_count != expected_count:
        print(f"WARNING: Expected {expected_count} body parts, got {actual_count}")
        return False

    return True


if __name__ == '__main__':
    # Print segment summary
    print("Body Segment Summary")
    print("=" * 50)

    counts = get_segment_count()
    total = 0

    for segment in BodySegment:
        count = counts[segment]
        total += count
        print(f"{segment.name:15s}: {count:3d} parts")

    print("=" * 50)
    print(f"{'TOTAL':15s}: {total:3d} parts")

    # Validate
    if validate_coverage():
        print("\n✓ All body parts mapped correctly")
    else:
        print("\n✗ Mapping incomplete!")

    # Show examples
    print("\nExample mappings:")
    examples = [
        "left_index_tip",
        "torso_chest_upper",
        "right_ankle",
        "head_forehead"
    ]

    for bp in examples:
        seg = get_segment(bp)
        print(f"  {bp:25s} → {seg.name}")
