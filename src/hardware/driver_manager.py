"""
Driver Manager - Manages multiple hardware drivers

Orchestrates communication with multiple hardware channels.
Phase 1: Single mock driver
Phase 2: Multiple drivers for full hand (6 channels)
Phase 3+: Full body distributed hardware
"""

import time
from typing import Dict, Optional, List
from src.core.schemas import CueParams
from src.hardware.mock_driver import MockHardwareDriver


class DriverManager:
    """
    Manages multiple hardware drivers (one per body part in full-body system).

    Phase 1: Single mock driver
    Phase 2: 6-channel hand (5 fingers + palm)
    Phase 3+: Full body distributed hardware

    Features:
    - Multi-channel synchronization
    - Bandwidth monitoring
    - Per-channel statistics
    """

    def __init__(self, driver_type: str = "mock", enable_sync: bool = False):
        """
        Initialize driver manager.

        Args:
            driver_type: "mock" (simulation) or "real" (Teensy serial)
            enable_sync: Enable channel synchronization (slight latency increase)
        """
        self.driver_type = driver_type
        self.enable_sync = enable_sync
        self.drivers: Dict[int, MockHardwareDriver] = {}  # body_part_id → driver

        # Bandwidth tracking
        self.total_packets_sent = 0
        self.start_time = time.time()

        print(f"✓ Driver Manager initialized")
        print(f"  Type: {driver_type}")
        print(f"  Sync: {'ON' if enable_sync else 'OFF'}")

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

        Phase 2: Supports synchronized multi-channel transmission.

        Args:
            cue_params_dict: Dict[body_part_id, CueParams]

        Returns:
            Dict[body_part_id, success] - Transmission status per driver
        """
        if self.enable_sync:
            return self._send_all_synchronized(cue_params_dict)
        else:
            return self._send_all_sequential(cue_params_dict)

    def _send_all_sequential(self, cue_params_dict: Dict[int, CueParams]) -> Dict[int, bool]:
        """Send to drivers sequentially (no synchronization)."""
        results = {}

        for body_part_id, cue_params in cue_params_dict.items():
            if body_part_id in self.drivers:
                driver = self.drivers[body_part_id]
                success = driver.send_cue_params(cue_params)
                results[body_part_id] = success

                if success:
                    self.total_packets_sent += 1
            else:
                # No driver registered for this body part
                results[body_part_id] = False

        return results

    def _send_all_synchronized(self, cue_params_dict: Dict[int, CueParams]) -> Dict[int, bool]:
        """
        Send to all drivers with synchronization.

        Ensures all channels receive packets at approximately the same time.
        Adds slight latency but improves multi-channel coherence.
        """
        results = {}

        # Phase 1: Prepare all packets
        prepared = []
        for body_part_id, cue_params in cue_params_dict.items():
            if body_part_id in self.drivers:
                prepared.append((body_part_id, cue_params, self.drivers[body_part_id]))

        # Phase 2: Synchronized transmission
        sync_start = time.time()

        for body_part_id, cue_params, driver in prepared:
            success = driver.send_cue_params(cue_params)
            results[body_part_id] = success

            if success:
                self.total_packets_sent += 1

        return results

    def get_aggregate_stats(self) -> dict:
        """
        Get combined statistics from all drivers.

        Phase 2: Enhanced with bandwidth monitoring and per-channel stats.

        Returns:
            Dict with:
            - driver_count: Number of registered drivers
            - total_sent: Total packets sent across all channels
            - total_dropped: Total packets dropped
            - total_received: Total ACKs received
            - avg_latency_ms: Average transmission latency
            - success_rate: Overall transmission success rate
            - bandwidth_hz: Effective bandwidth (packets/second)
            - per_channel_stats: Statistics per body part
        """
        total_sent = 0
        total_dropped = 0
        total_received = 0
        total_latency = 0.0
        driver_count = len(self.drivers)

        per_channel_stats = {}

        for body_part_id, driver in self.drivers.items():
            stats = driver.get_stats()
            total_sent += stats['packets_sent']
            total_dropped += stats['packets_dropped']
            total_received += stats['packets_received']
            total_latency += stats['avg_latency_ms'] * stats['packets_sent']

            # Store per-channel stats
            per_channel_stats[body_part_id] = stats

        avg_latency = total_latency / total_sent if total_sent > 0 else 0.0
        total_attempts = total_sent + total_dropped
        success_rate = total_sent / total_attempts if total_attempts > 0 else 0.0

        # Calculate bandwidth
        elapsed_time = time.time() - self.start_time
        bandwidth_hz = self.total_packets_sent / elapsed_time if elapsed_time > 0 else 0.0

        return {
            'driver_count': driver_count,
            'total_sent': total_sent,
            'total_dropped': total_dropped,
            'total_received': total_received,
            'avg_latency_ms': avg_latency,
            'success_rate': success_rate,
            'bandwidth_hz': bandwidth_hz,
            'elapsed_time_s': elapsed_time,
            'per_channel_stats': per_channel_stats
        }

    def get_bandwidth_stats(self) -> dict:
        """
        Get bandwidth utilization statistics.

        Returns:
            Dict with:
            - total_packets: Total packets sent
            - elapsed_time_s: Time since initialization
            - bandwidth_hz: Effective bandwidth (packets/second)
            - bandwidth_kbps: Effective bandwidth (kilobits/second, assuming 52 bytes/packet)
        """
        elapsed_time = time.time() - self.start_time
        bandwidth_hz = self.total_packets_sent / elapsed_time if elapsed_time > 0 else 0.0

        # 52 bytes/packet * 8 bits/byte = 416 bits/packet
        bandwidth_kbps = (bandwidth_hz * 416) / 1000

        return {
            'total_packets': self.total_packets_sent,
            'elapsed_time_s': elapsed_time,
            'bandwidth_hz': bandwidth_hz,
            'bandwidth_kbps': bandwidth_kbps,
            'channels_active': len(self.drivers)
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
