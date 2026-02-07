#!/usr/bin/env python3
"""
Multi-Driver Manager - Distributed Haptic Hardware Control.

Manages multiple segment drivers (Teensy 4.1) for full-body haptics.
Routes SensationParams to the correct driver based on body part → segment mapping.

Architecture:
    Host (Python) → 6× Serial Connections → 6× Teensy → 50+ Actuators

Each Teensy drives 8-10 actuators for one body segment:
    - HEAD: 8 actuators
    - TORSO: 8 actuators
    - LEFT_ARM: 10 actuators (glove)
    - RIGHT_ARM: 10 actuators (glove)
    - LEFT_LEG: 8 actuators
    - RIGHT_LEG: 8 actuators
"""

import serial
import threading
import logging
import struct
from queue import Queue, Full
from typing import Dict, List, Optional
from dataclasses import dataclass

from src.core.schemas import SensationParams
from src.hardware.body_segments import BodySegment, get_segment, get_parts_for_segment

logger = logging.getLogger(__name__)


@dataclass
class DriverConfig:
    """Configuration for a single segment driver (Teensy)."""
    segment: BodySegment
    port: str                    # e.g., "/dev/ttyUSB0" (Linux) or "COM3" (Windows)
    baud_rate: int = 230400      # 230400 for arm segments, 115200 for others
    actuator_count: int = 8      # Number of actuators on this Teensy


class SegmentDriver:
    """
    Driver for a single body segment (one Teensy).

    Handles serial communication and packet routing for all body parts
    in its segment. Runs in background thread for non-blocking transmission.

    Protocol:
        - 38-byte SensationParams packets
        - Header: 0xBB 0x66
        - XOR checksum
        - 100Hz update rate target
    """

    def __init__(self, config: DriverConfig):
        """
        Initialize segment driver.

        Args:
            config: Driver configuration (port, baud, actuator count)
        """
        self.config = config
        self.segment = config.segment
        self.serial: Optional[serial.Serial] = None
        self.connected = False

        # Send queue for non-blocking transmission
        self.send_queue: Queue = Queue(maxsize=100)
        self._sender_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Statistics
        self.packets_sent = 0
        self.packets_dropped = 0

        logger.info(f"SegmentDriver created: {self.segment.name} on {config.port}")

    def connect(self) -> bool:
        """
        Open serial connection to Teensy.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial = serial.Serial(
                port=self.config.port,
                baudrate=self.config.baud_rate,
                timeout=0.01,
                write_timeout=0.01
            )
            self.connected = True
            self._start_sender_thread()
            logger.info(f"{self.segment.name}: Connected to {self.config.port} "
                       f"at {self.config.baud_rate} baud")
            return True

        except serial.SerialException as e:
            logger.error(f"{self.segment.name}: Failed to connect to {self.config.port}: {e}")
            return False

    def disconnect(self):
        """Close serial connection and stop sender thread."""
        self.connected = False
        self._stop_event.set()

        if self._sender_thread and self._sender_thread.is_alive():
            self._sender_thread.join(timeout=1.0)

        if self.serial and self.serial.is_open:
            self.serial.close()

        logger.info(f"{self.segment.name}: Disconnected (sent={self.packets_sent}, "
                   f"dropped={self.packets_dropped})")

    def send(self, body_part_id: int, sensation: SensationParams):
        """
        Queue a sensation packet for transmission.

        Non-blocking. Drops packet if queue is full.

        Args:
            body_part_id: Local body part ID (0-255) within this segment
            sensation: SensationParams to transmit
        """
        if not self.connected:
            return

        packet = self._serialize(body_part_id, sensation)

        try:
            self.send_queue.put_nowait(packet)
        except Full:
            self.packets_dropped += 1
            if self.packets_dropped % 100 == 0:
                logger.warning(f"{self.segment.name}: Dropped {self.packets_dropped} packets "
                             "(queue full)")

    def _serialize(self, body_part_id: int, sensation: SensationParams) -> bytes:
        """
        Serialize SensationParams to 38-byte packet.

        Format:
            [0-1]   Header: 0xBB 0x66
            [2]     Version: 0x01
            [3]     Body Part ID: 0-255
            [4-7]   Timestamp: uint32_t (microseconds)
            [8-35]  Sensation params: 14× float16
            [36]    Flags: impact_trigger
            [37]    Checksum: XOR of bytes [0-36]

        Args:
            body_part_id: Local ID within segment
            sensation: SensationParams to serialize

        Returns:
            38-byte packet
        """
        data = bytearray([0xBB, 0x66, 0x01, body_part_id & 0xFF])

        # Timestamp (4 bytes, little-endian)
        ts = sensation.timestamp_us & 0xFFFFFFFF
        data.extend(ts.to_bytes(4, 'little'))

        # Pack 14 float16 values (28 bytes)
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
            # struct.pack('<e', v) for float16 little-endian
            data.extend(struct.pack('<e', v))

        # Flags (1 byte)
        flags = 0x01 if sensation.impact_trigger else 0x00
        data.append(flags)

        # Checksum (XOR of all bytes)
        checksum = 0
        for b in data:
            checksum ^= b
        data.append(checksum)

        return bytes(data)

    def _start_sender_thread(self):
        """Start background thread for serial transmission."""
        def sender_loop():
            while not self._stop_event.is_set():
                try:
                    packet = self.send_queue.get(timeout=0.01)
                    if self.serial and self.serial.is_open:
                        self.serial.write(packet)
                        self.packets_sent += 1
                except:
                    pass  # Timeout or queue empty

        self._sender_thread = threading.Thread(
            target=sender_loop,
            daemon=True,
            name=f"Sender-{self.segment.name}"
        )
        self._sender_thread.start()


class MultiDriverManager:
    """
    Manages multiple segment drivers for full-body haptics.

    Routes SensationParams to the correct driver based on body part → segment mapping.

    Example:
        configs = [
            DriverConfig(BodySegment.LEFT_ARM, "/dev/ttyUSB0", 230400, 10),
            DriverConfig(BodySegment.RIGHT_ARM, "/dev/ttyUSB1", 230400, 10),
            DriverConfig(BodySegment.TORSO, "/dev/ttyUSB2", 115200, 8),
        ]

        manager = MultiDriverManager(configs)
        manager.connect_all()

        sensations = {"left_index_tip": sensation1, "right_palm": sensation2}
        manager.send(sensations)
    """

    def __init__(self, configs: List[DriverConfig]):
        """
        Initialize multi-driver manager.

        Args:
            configs: List of driver configurations (one per segment)
        """
        self.drivers: Dict[BodySegment, SegmentDriver] = {}

        for config in configs:
            self.drivers[config.segment] = SegmentDriver(config)

        # Build mapping from body part name → local ID within segment
        self._body_part_to_id: Dict[str, int] = {}
        self._build_body_part_map()

        logger.info(f"MultiDriverManager initialized with {len(configs)} drivers")

    def _build_body_part_map(self):
        """
        Build mapping from body part name to local ID (0-255).

        Each segment has its own ID space. Within a segment, body parts
        are numbered sequentially starting from 0.
        """
        for segment in BodySegment:
            parts = get_parts_for_segment(segment)
            for i, part in enumerate(parts):
                self._body_part_to_id[part] = i

        logger.debug(f"Built body part map: {len(self._body_part_to_id)} parts")

    def connect_all(self) -> Dict[BodySegment, bool]:
        """
        Connect to all configured drivers.

        Returns:
            Dict mapping segment → connection success
        """
        results = {}
        for segment, driver in self.drivers.items():
            results[segment] = driver.connect()

        connected_count = sum(results.values())
        logger.info(f"Connected {connected_count}/{len(self.drivers)} drivers")

        return results

    def disconnect_all(self):
        """Disconnect all drivers."""
        for driver in self.drivers.values():
            driver.disconnect()

        logger.info("All drivers disconnected")

    def send(self, sensations: Dict[str, SensationParams]):
        """
        Send sensations to appropriate drivers.

        Routes each sensation to the correct segment driver based on
        body part → segment mapping.

        Args:
            sensations: Dict mapping body_part_name → SensationParams

        Example:
            sensations = {
                "left_index_tip": sensation1,
                "right_palm": sensation2,
                "torso_chest_upper": sensation3,
            }
            manager.send(sensations)
        """
        # Group sensations by segment
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

    def get_status(self) -> Dict[BodySegment, Dict[str, any]]:
        """
        Get status of all drivers.

        Returns:
            Dict mapping segment → status dict
        """
        status = {}
        for segment, driver in self.drivers.items():
            status[segment] = {
                'connected': driver.connected,
                'packets_sent': driver.packets_sent,
                'packets_dropped': driver.packets_dropped,
                'queue_size': driver.send_queue.qsize(),
            }
        return status

    def reset_statistics(self):
        """Reset packet statistics for all drivers."""
        for driver in self.drivers.values():
            driver.packets_sent = 0
            driver.packets_dropped = 0

        logger.info("Driver statistics reset")
