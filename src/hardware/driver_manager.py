"""
Driver Manager - Manages multiple hardware drivers

Orchestrates communication with multiple hardware channels.
Phase 1: Single mock driver
Phase 2+: Multiple drivers for distributed hardware (full body)
"""

from typing import Dict, Optional
from src.core.schemas import CueParams
from src.hardware.mock_driver import MockHardwareDriver


class DriverManager:
    """
    Manages multiple hardware drivers (one per body part in full-body system).

    Phase 1: Single mock driver
    Phase 2+: Multiple drivers for distributed hardware
    """

    def __init__(self, driver_type: str = "mock"):
        """
        Initialize driver manager.

        Args:
            driver_type: "mock" (simulation) or "real" (Teensy serial)
        """
        self.driver_type = driver_type
        self.drivers: Dict[int, MockHardwareDriver] = {}  # body_part_id → driver

        print(f"✓ Driver Manager initialized")
        print(f"  Type: {driver_type}")

    def register_driver(
        self,
        body_part_id: int,
        port: str = "MOCK",
        config: Optional[dict] = None
    ):
        """
        Register a hardware driver for a body part.

        Args:
            body_part_id: Body part identifier
            port: Port identifier (e.g., "/dev/ttyUSB0" or "MOCK")
            config: Optional driver configuration
        """
        if config is None:
            config = {}

        if self.driver_type == "mock":
            driver = MockHardwareDriver(
                port=port,
                baudrate=config.get('baudrate', 115200),
                simulate_latency=config.get('simulate_latency', True),
                packet_loss_rate=config.get('packet_loss_rate', 0.0)
            )
        else:
            raise NotImplementedError(f"Driver type '{self.driver_type}' not implemented")

        driver.connect()
        self.drivers[body_part_id] = driver

        print(f"✓ Registered driver for body_part_id={body_part_id} on {port}")

    def send_all(self, cue_params_dict: Dict[int, CueParams]) -> Dict[int, bool]:
        """
        Send CueParams to all registered drivers.

        Args:
            cue_params_dict: Dict[body_part_id, CueParams]

        Returns:
            Dict[body_part_id, success] - Transmission status per driver
        """
        results = {}

        for body_part_id, cue_params in cue_params_dict.items():
            if body_part_id in self.drivers:
                driver = self.drivers[body_part_id]
                success = driver.send_cue_params(cue_params)
                results[body_part_id] = success
            else:
                # No driver registered for this body part
                results[body_part_id] = False

        return results

    def get_aggregate_stats(self) -> dict:
        """
        Get combined statistics from all drivers.

        Returns:
            Dict with aggregated statistics across all drivers
        """
        total_sent = 0
        total_dropped = 0
        total_received = 0
        total_latency = 0.0
        driver_count = len(self.drivers)

        for driver in self.drivers.values():
            stats = driver.get_stats()
            total_sent += stats['packets_sent']
            total_dropped += stats['packets_dropped']
            total_received += stats['packets_received']
            total_latency += stats['avg_latency_ms'] * stats['packets_sent']

        avg_latency = total_latency / total_sent if total_sent > 0 else 0.0
        total_attempts = total_sent + total_dropped
        success_rate = total_sent / total_attempts if total_attempts > 0 else 0.0

        return {
            'driver_count': driver_count,
            'total_sent': total_sent,
            'total_dropped': total_dropped,
            'total_received': total_received,
            'avg_latency_ms': avg_latency,
            'success_rate': success_rate
        }

    def disconnect_all(self):
        """Disconnect all drivers."""
        for body_part_id, driver in self.drivers.items():
            driver.disconnect()
            print(f"✓ Disconnected driver for body_part_id={body_part_id}")

        self.drivers.clear()

    def reset_all_stats(self):
        """Reset statistics for all drivers."""
        for driver in self.drivers.values():
            driver.reset_stats()

    def __repr__(self) -> str:
        """String representation."""
        return f"DriverManager(type={self.driver_type}, drivers={len(self.drivers)})"
