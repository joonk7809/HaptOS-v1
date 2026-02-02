#!/usr/bin/env python3
"""
Hardware Integration Example

Demonstrates how to use HAPTOS with real hardware.

IMPORTANT: This example requires:
- Teensy 4.1 with HaptosReceiver.ino firmware
- Serial port connection (USB)
- Voice coil actuator

For testing without hardware, use driver_type="mock" instead.
"""

import haptos
import sys

def main():
    print("=" * 60)
    print("  Hardware Integration Example")
    print("=" * 60)
    print()

    # Check if user wants to use real hardware
    use_real_hardware = False

    if len(sys.argv) > 1 and sys.argv[1] == "--real":
        use_real_hardware = True
        print("⚠️  REAL HARDWARE MODE")
        print()

        # Ask for serial port
        port = input("Enter serial port (e.g., /dev/ttyACM0 or COM3): ").strip()
        if not port:
            print("Error: No port specified")
            return

        driver_type = "serial"
        driver_config = {
            'baudrate': 115200,
            'timeout': 1.0
        }
    else:
        print("MOCK HARDWARE MODE")
        print("(Run with --real flag to use actual hardware)")
        print()
        port = "MOCK"
        driver_type = "mock"
        driver_config = {
            'simulate_latency': True,
            'packet_loss_rate': 0.01  # 1% packet loss
        }

    # Create simulation
    sim = haptos.Simulation(
        model_path="assets/hand_models/simple_hand.xml",
        max_contacts=6
    )
    print()

    # Create renderer
    renderer = haptos.Renderer()
    print()

    # Create driver
    driver = haptos.Driver(driver_type=driver_type)
    print()

    # Register channel for index fingertip
    print(f"Registering hardware channel...")
    driver.register(
        body_part_id=10,  # Index fingertip
        port=port,
        config=driver_config
    )
    print()

    # Run simulation
    duration = 5.0
    steps = int(duration / 0.001)

    print(f"Running simulation for {duration}s...")
    print("(Press Ctrl+C to stop early)")
    print()

    try:
        for step in range(steps):
            # Step physics
            filtered_contacts = sim.step_filtered()

            # Render and send (100Hz)
            if step % 10 == 0 and filtered_contacts:
                cues = renderer.render(filtered_contacts)
                results = driver.send(cues)

                # Status every 500ms
                if step % 500 == 0 and step > 0:
                    elapsed = step * 0.001
                    stats = driver.get_stats()
                    print(f"  {elapsed:.1f}s - Success rate: {stats['success_rate']:.1%}, "
                          f"Latency: {stats['avg_latency_ms']:.2f}ms")

    except KeyboardInterrupt:
        print()
        print("Interrupted by user")
        print()

    # Final statistics
    print()
    print("=" * 60)
    print("  Statistics")
    print("=" * 60)
    print()

    driver_stats = driver.get_stats()
    print(f"Driver:")
    print(f"  Type: {driver_type}")
    print(f"  Packets sent: {driver_stats['total_sent']}")
    print(f"  Packets dropped: {driver_stats['total_dropped']}")
    print(f"  Success rate: {driver_stats['success_rate']:.1%}")
    print(f"  Avg latency: {driver_stats['avg_latency_ms']:.2f}ms")
    print()

    if use_real_hardware:
        bandwidth = driver.get_bandwidth_stats()
        print(f"Bandwidth:")
        print(f"  Packets/sec: {bandwidth['bandwidth_hz']:.1f}")
        print(f"  kbps: {bandwidth['bandwidth_kbps']:.1f}")
        print()

        print("Check Teensy Serial Monitor for firmware output.")
        print()

    # Cleanup
    driver.disconnect_all()
    print("✓ Disconnected")


if __name__ == "__main__":
    main()
