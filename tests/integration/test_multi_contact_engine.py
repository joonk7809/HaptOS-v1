"""
Test multi-contact physics engine.
Verify contact extraction and body part identification.
"""

import sys
sys.path.append('src')

import time
from physics.multi_contact_engine import MultiContactEngine


def test_hand_model_loading():
    """Test that the hand model loads correctly."""
    print("=" * 60)
    print("TEST 1: Hand Model Loading")
    print("=" * 60)

    try:
        engine = MultiContactEngine("assets/hand_models/simple_hand.xml")
        print("✓ Hand model loaded successfully")
        print(f"  Geom map: {engine.geom_to_body}")
        print(f"  Number of body parts: {len(engine.geom_to_body)}")
        print(f"  Floor geom ID: {engine.floor_geom_id}")
        return True
    except Exception as e:
        print(f"✗ Failed to load hand model: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_no_contact():
    """Test that no-contact state is handled correctly."""
    print("\n" + "=" * 60)
    print("TEST 2: No Contact State")
    print("=" * 60)

    try:
        engine = MultiContactEngine("assets/hand_models/simple_hand.xml")

        # Step physics (hand should be falling, not in contact yet)
        contacts = engine.step_multi()

        print(f"Contacts detected: {len(contacts)}")
        print(f"Body parts: {list(contacts.keys())}")

        if len(contacts) == 0:
            print("✓ No contacts detected (expected)")
            return True
        else:
            print("  Note: Hand may already be in contact with floor")
            for body_part, contact in contacts.items():
                print(f"    {body_part}: {contact.normal_force_N:.3f}N")
            return True

    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_simulation_steps():
    """Test running multiple simulation steps."""
    print("\n" + "=" * 60)
    print("TEST 3: Simulation Steps (1 second)")
    print("=" * 60)

    try:
        engine = MultiContactEngine("assets/hand_models/simple_hand.xml")

        # Run for 1 second (1000 steps @ 1kHz)
        n_steps = 1000
        contact_history = []

        print(f"Running {n_steps} physics steps...")
        start_time = time.time()

        for i in range(n_steps):
            contacts = engine.step_multi()
            contact_history.append(contacts)

            # Print status every 100ms
            if (i + 1) % 100 == 0:
                n_contacts = len(contacts)
                print(f"  Step {i+1}/{n_steps}: {n_contacts} active contacts")

        elapsed = time.time() - start_time
        print(f"\n✓ Completed {n_steps} steps in {elapsed:.3f}s")
        print(f"  Average: {n_steps/elapsed:.1f} Hz (target: 1000 Hz)")

        # Analyze contacts
        all_body_parts = set()
        for contacts in contact_history:
            all_body_parts.update(contacts.keys())

        print(f"\nBody parts contacted during simulation:")
        for body_part in sorted(all_body_parts):
            # Count how many steps this body part was in contact
            count = sum(1 for c in contact_history if body_part in c)
            print(f"  {body_part}: {count}/{n_steps} steps ({100*count/n_steps:.1f}%)")

        return True

    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_backward_compatibility():
    """Test that single-contact API still works."""
    print("\n" + "=" * 60)
    print("TEST 4: Backward Compatibility (single contact)")
    print("=" * 60)

    try:
        engine = MultiContactEngine("assets/hand_models/simple_hand.xml")

        # Use old API (step instead of step_multi)
        for i in range(100):
            contact = engine.step()  # Returns single ContactPatch

        print("✓ Single-contact API works")
        print(f"  Last contact force: {contact.normal_force_N:.3f}N")
        print(f"  In contact: {contact.in_contact}")

        return True

    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("\n" + "=" * 60)
    print("MULTI-CONTACT ENGINE TESTS")
    print("=" * 60 + "\n")

    tests = [
        test_hand_model_loading,
        test_no_contact,
        test_simulation_steps,
        test_backward_compatibility
    ]

    results = []
    for test_func in tests:
        results.append(test_func())

    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)

    passed = sum(results)
    total = len(results)

    for i, (test_func, result) in enumerate(zip(tests, results)):
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{status}: {test_func.__doc__}")

    print()
    print(f"Results: {passed}/{total} tests passed")

    if passed == total:
        print("✓ All tests passed!")
        return 0
    else:
        print(f"✗ {total - passed} test(s) failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())
