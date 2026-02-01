"""
Mock Hardware Driver - Simulated serial communication

Simulates communication with Teensy firmware without real hardware.
Used for testing and validation of the complete pipeline.

Features:
- Virtual serial port (tx/rx buffers)
- Realistic latency simulation based on baud rate
- Configurable packet loss
- ACK/NACK response generation
- Statistics tracking
"""

from typing import List, Dict, Optional
from collections import deque
import time
import random

from src.core.schemas import CueParams


class MockHardwareDriver:
    """
    Simulated hardware driver for testing (no real Teensy).

    Simulates:
    - Serial port communication (virtual buffer)
    - CueParams binary packet transmission
    - Firmware acknowledgment (ACK/NACK)
    - Latency modeling (1-5ms per packet)
    - Packet loss simulation (configurable error rate)
    """

    def __init__(
        self,
        port: str = "MOCK",
        baudrate: int = 115200,
        simulate_latency: bool = True,
        packet_loss_rate: float = 0.0
    ):
        """
        Initialize mock driver.

        Args:
            port: Virtual port identifier (ignored in simulation)
            baudrate: Simulated baud rate (affects latency calculation)
            simulate_latency: Enable realistic transmission delays
            packet_loss_rate: Probability of packet drop (0.0-1.0)
        """
        self.port = port
        self.baudrate = baudrate
        self.simulate_latency = simulate_latency
        self.packet_loss_rate = max(0.0, min(1.0, packet_loss_rate))

        # Virtual transmission buffers
        self.tx_buffer: deque = deque()  # Transmitted packets
        self.rx_buffer: deque = deque()  # Received ACK/NACK

        # Statistics
        self.packets_sent = 0
        self.packets_received = 0
        self.packets_dropped = 0
        self.total_latency_ms = 0.0

        # Connection state
        self.is_connected = False

        print(f"✓ Mock Hardware Driver initialized")
        print(f"  Port: {port} (simulated)")
        print(f"  Baud: {baudrate}")
        print(f"  Latency simulation: {'ON' if simulate_latency else 'OFF'}")
        print(f"  Packet loss rate: {packet_loss_rate * 100:.1f}%")

    def connect(self) -> bool:
        """
        Simulate connection establishment.

        Returns:
            True (always succeeds in simulation)
        """
        self.is_connected = True
        print(f"✓ Connected to {self.port}")
        return True

    def disconnect(self):
        """Simulate disconnection."""
        self.is_connected = False
        self.tx_buffer.clear()
        self.rx_buffer.clear()
        print(f"✓ Disconnected from {self.port}")

    def send_cue_params(self, cue_params: CueParams) -> bool:
        """
        Send CueParams packet to simulated hardware.

        Args:
            cue_params: Haptic parameters to transmit

        Returns:
            True if packet sent successfully, False if dropped

        Simulation:
        1. Serialize to binary (52 bytes)
        2. Calculate transmission latency (bytes/baudrate)
        3. Simulate packet loss
        4. Add to virtual tx_buffer
        5. Generate ACK response
        """
        if not self.is_connected:
            raise RuntimeError("Driver not connected")

        # Serialize to binary
        packet = cue_params.to_bytes()

        # Simulate packet loss
        if random.random() < self.packet_loss_rate:
            self.packets_dropped += 1
            return False

        # Calculate transmission latency
        if self.simulate_latency:
            # Serial transmission time: (bits / baudrate)
            # 52 bytes = 416 bits (8N1 encoding = 10 bits/byte)
            tx_time_ms = (len(packet) * 10 / self.baudrate) * 1000
            self.total_latency_ms += tx_time_ms

        # Add to tx buffer (simulate successful transmission)
        self.tx_buffer.append({
            'packet': packet,
            'timestamp': time.time(),
            'cue_params': cue_params  # Store for validation
        })

        self.packets_sent += 1

        # Simulate immediate ACK response
        self._generate_ack(success=True)

        return True

    def _generate_ack(self, success: bool):
        """
        Simulate firmware ACK/NACK response.

        Args:
            success: True for ACK, False for NACK
        """
        ack_byte = 0x06 if success else 0x15  # ACK = 0x06, NACK = 0x15
        self.rx_buffer.append(ack_byte)
        self.packets_received += 1

    def read_ack(self) -> Optional[bool]:
        """
        Read ACK/NACK from firmware.

        Returns:
            True if ACK, False if NACK, None if no response
        """
        if not self.rx_buffer:
            return None

        ack = self.rx_buffer.popleft()
        return ack == 0x06

    def get_tx_buffer(self) -> List[dict]:
        """
        Get all transmitted packets (for testing).

        Returns:
            List of transmitted packet dictionaries
        """
        return list(self.tx_buffer)

    def clear_buffers(self):
        """Clear all buffers."""
        self.tx_buffer.clear()
        self.rx_buffer.clear()

    def get_stats(self) -> dict:
        """
        Get transmission statistics.

        Returns:
            Dict with:
            - packets_sent: Total packets transmitted
            - packets_dropped: Packets lost to simulated errors
            - packets_received: ACKs received
            - avg_latency_ms: Average transmission latency
            - success_rate: sent / (sent + dropped)
        """
        total_attempts = self.packets_sent + self.packets_dropped
        success_rate = (
            self.packets_sent / total_attempts
            if total_attempts > 0 else 0.0
        )

        avg_latency = (
            self.total_latency_ms / self.packets_sent
            if self.packets_sent > 0 else 0.0
        )

        return {
            'packets_sent': self.packets_sent,
            'packets_dropped': self.packets_dropped,
            'packets_received': self.packets_received,
            'avg_latency_ms': avg_latency,
            'success_rate': success_rate
        }

    def reset_stats(self):
        """Reset all statistics counters."""
        self.packets_sent = 0
        self.packets_received = 0
        self.packets_dropped = 0
        self.total_latency_ms = 0.0

    def __repr__(self) -> str:
        """String representation."""
        status = "connected" if self.is_connected else "disconnected"
        return f"MockHardwareDriver(port={self.port}, status={status}, sent={self.packets_sent})"
