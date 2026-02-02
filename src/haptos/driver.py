"""
haptos.Driver - Hardware Driver Wrapper (Layer 3)

Manages communication with haptic hardware (real or simulated).
"""

from typing import Dict, Optional, List

from src.core.schemas import CueParams
from src.hardware.driver_manager import DriverManager


class Driver:
    """
    Hardware driver for haptic output.

    Supports multiple driver types:
    - "mock": Simulated hardware (for testing without physical device)
    - "serial": Real hardware via serial port (Phase 4+)

    Example:
        >>> driver = Driver(driver_type="mock")
        >>> driver.register(body_part_id=10, port="MOCK")
        >>> driver.send(cue_params_dict)

    Args:
        driver_type: Type of driver ("mock" or "serial")
        enable_sync: Enable channel synchronization (default: False)
    """

    def __init__(
        self,
        driver_type: str = "mock",
        enable_sync: bool = False
    ):
        """Initialize driver manager."""
        if driver_type not in ["mock", "serial"]:
            raise ValueError(f"Unknown driver type: {driver_type}. Use 'mock' or 'serial'")

        self._manager = DriverManager(
            driver_type=driver_type,
            enable_sync=enable_sync
        )

        self.driver_type = driver_type
        self.enable_sync = enable_sync

        print(f"✓ Driver initialized")
        print(f"  Type: {driver_type}")
        print(f"  Sync: {'ON' if enable_sync else 'OFF'}")

    def register(
        self,
        body_part_id: int,
        port: str = "MOCK",
        config: Optional[dict] = None
    ):
        """
        Register a hardware channel for a body part.

        Args:
            body_part_id: Body part identifier (e.g., 10 for index fingertip)
            port: Port identifier:
                - "MOCK" for simulated hardware
                - "/dev/ttyACM0" for real serial port (Linux/Mac)
                - "COM3" for real serial port (Windows)
            config: Optional driver configuration dict:
                - baudrate: Serial baud rate (default: 115200)
                - packet_loss_rate: Simulated packet loss (mock only)
                - simulate_latency: Enable latency simulation (mock only)

        Example:
            >>> driver.register(10, "MOCK")  # Index finger (mock)
            >>> driver.register(11, "/dev/ttyACM0")  # Thumb (real)
        """
        self._manager.register_driver(body_part_id, port, config)

    def send(self, cue_params_dict: Dict[int, CueParams]) -> Dict[int, bool]:
        """
        Send cue parameters to all registered channels.

        Args:
            cue_params_dict: Dict mapping body_part_id → CueParams

        Returns:
            Dict mapping body_part_id → success (True/False)

        Note:
            This method handles packet serialization, transmission,
            and optional synchronization automatically.
        """
        return self._manager.send_all(cue_params_dict)

    def disconnect_all(self):
        """
        Disconnect all hardware channels.

        Should be called before exiting to ensure clean shutdown.
        """
        self._manager.disconnect_all()

    def reset_stats(self):
        """Reset statistics for all channels."""
        self._manager.reset_all_stats()

    def get_stats(self) -> dict:
        """
        Get aggregated statistics from all channels.

        Returns:
            Dict with:
            - driver_count: Number of registered channels
            - total_sent: Total packets sent
            - total_dropped: Total packets dropped
            - success_rate: Transmission success rate (0-1)
            - avg_latency_ms: Average transmission latency
            - bandwidth_hz: Effective bandwidth (packets/second)
            - bandwidth_kbps: Effective bandwidth (kilobits/second)
            - per_channel_stats: Statistics per body part
        """
        return self._manager.get_aggregate_stats()

    def get_bandwidth_stats(self) -> dict:
        """
        Get bandwidth utilization statistics.

        Returns:
            Dict with:
            - total_packets: Total packets sent
            - elapsed_time_s: Time since initialization
            - bandwidth_hz: Packets per second
            - bandwidth_kbps: Kilobits per second (52 bytes/packet)
            - channels_active: Number of active channels
        """
        return self._manager.get_bandwidth_stats()

    def __repr__(self) -> str:
        """String representation."""
        stats = self.get_stats()
        return (
            f"Driver(type={self.driver_type}, "
            f"channels={stats['driver_count']}, "
            f"success_rate={stats['success_rate']:.1%})"
        )

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit (auto-disconnect)."""
        self.disconnect_all()
