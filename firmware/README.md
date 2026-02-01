# HAPTOS Firmware

This directory contains firmware implementations for the HAPTOS Platform hardware layer (Layer 3).

## Firmware Variants

### HaptosReceiver.ino ⭐ **Current (Phase 1/2 Compatible)**

**Architecture**: Three-layer distributed system
- **Host (Python)**: Runs physics @ 1kHz, neural inference @ 100Hz
- **Firmware (Teensy)**: Receives CueParams packets, synthesizes @ 2kHz+

**Protocol**: 52-byte CueParams packets over serial (115200 baud)
- Packet format matches `src/core/schemas.py` CueParams structure
- Header: `0xAA 0x55`
- Checksum: XOR of all bytes
- See `docs/specs/firmware_protocol.md` for full specification

**Features**:
- Serial packet reception with header sync
- Checksum validation
- Parameter interpolation @ 2kHz (smooth transitions)
- 5-cue synthesis pipeline:
  - Texture (grain oscillator + noise)
  - Shear (directional modulation)
  - Impact (exponential decay transient)
  - Ring (resonant mode decay)
  - Weight (DC offset / amplitude)
- Statistics tracking (packets received, dropped, checksum errors)

**Hardware**:
- **Target**: Teensy 4.1 + Audio Shield or MAX98357A I2S DAC
- **Output**: I2S stereo audio (converted to haptic signal)
- **Future**: VCA amplifier for voice coil actuator

**Status**: ✅ Ready for Phase 4 hardware testing

---

### HaptosRuntime.ino (Legacy - Standalone)

**Architecture**: Monolithic on-device system
- Reads load cell directly (ADC)
- Runs neural inference on Teensy
- Synthesizes audio output

**Protocol**: None (standalone, no host communication)

**Features**:
- Load cell sampling @ 1kHz
- Feature extraction (13-dim vector)
- Phase detection FSM (NO_CONTACT → IMPACT → HOLD → RELEASE)
- On-device neural inference (see `inference_core.h`)
- Audio synthesis via Teensy Audio Library

**Hardware**:
- **Target**: Teensy 4.1 + Audio Shield
- **Input**: AD620 load cell amplifier on A0
- **Output**: I2S audio

**Status**: 🗄️ Legacy reference (pre-refactor architecture)

**Note**: This firmware predates the three-layer architecture refactor. It was used for initial prototyping but is NOT compatible with the current Phase 1/2 Python pipeline.

---

## Hardware Setup Guide

### Phase 1/2 Testing (HaptosReceiver.ino)

**Required Hardware**:
1. Teensy 4.1
2. Audio Shield (SGTL5000) OR MAX98357A I2S DAC
3. Voice coil actuator (for haptic output)
4. USB cable (for serial communication)

**Wiring**:
```
Teensy 4.1 → Audio Shield / DAC
---------------------------------
I2S Pins (built-in):
- Pin 7  → BCLK (bit clock)
- Pin 20 → LRCLK (left/right clock)
- Pin 8  → TX (data out)

Audio Shield (if using):
- Insert directly into Teensy 4.1 pins

MAX98357A (if using):
- BCLK → Teensy Pin 7
- LRCLK → Teensy Pin 20
- DIN → Teensy Pin 8
- VIN → 5V
- GND → GND
- OUT+ / OUT- → Voice coil actuator
```

**Flashing Firmware**:
1. Open `HaptosReceiver.ino` in Arduino IDE
2. Install Teensy support: https://www.pjrc.com/teensy/td_download.html
3. Install Audio Library: Tools → Manage Libraries → "Audio"
4. Select: Tools → Board → Teensy 4.1
5. Select: Tools → USB Type → Serial
6. Click Upload

**Testing**:
1. Flash firmware to Teensy
2. Connect Teensy via USB
3. Run Python mock driver tests:
   ```bash
   cd /Users/joon/haptOS
   python -m pytest tests/phase1/test_full_pipeline.py -v
   ```
4. Replace MockHardwareDriver with real serial port:
   ```python
   # In test or application code:
   driver = SerialHardwareDriver(port="/dev/ttyACM0", baudrate=115200)
   ```

**Expected Behavior**:
- Teensy Serial Monitor shows:
  ```
  HAPTOS Receiver - Phase 1/2 Protocol
  Waiting for CueParams packets...
  Packet size: 52 bytes
  Ready.

  RX #10 | texture: 150.0Hz @ 0.45 | weight: 0.60 | impulse: YES
  RX #20 | texture: 155.2Hz @ 0.48 | weight: 0.62 | impulse: no
  ...
  ```

---

## Serial Protocol Reference

### Packet Format (52 bytes)

| Offset | Size | Type | Field | Range |
|--------|------|------|-------|-------|
| 0-1 | 2 | uint8 | Header | 0xAA 0x55 |
| 2-5 | 4 | uint32 | timestamp_us | 0 - 2^32 |
| 6-9 | 4 | float32 | texture_grain_hz | 50-500 Hz |
| 10-13 | 4 | float32 | texture_amplitude | 0.0-1.0 |
| 14-17 | 4 | float32 | shear_direction_x | -1.0 to 1.0 |
| 18-21 | 4 | float32 | shear_direction_y | -1.0 to 1.0 |
| 22-25 | 4 | float32 | shear_magnitude | 0.0-1.0 |
| 26-29 | 4 | float32 | weight_offset | 0.0-1.0 |
| 30-33 | 4 | float32 | impact_amplitude | 0.0-1.0 |
| 34-37 | 4 | float32 | impact_decay_ms | 10-500 ms |
| 38-41 | 4 | float32 | impact_frequency_hz | 80-400 Hz |
| 42-45 | 4 | float32 | ring_amplitude | 0.0-1.0 |
| 46-49 | 4 | float32 | ring_decay_ms | 50-1000 ms |
| 50 | 1 | uint8 | trigger_impulse | 0 or 1 |
| 51 | 1 | uint8 | checksum | XOR of bytes [0-50] |

**Byte Order**: Little-endian (x86/ARM default)

**Checksum Calculation**:
```c
uint8_t checksum = 0;
for (int i = 0; i < 51; i++) {
    checksum ^= packet[i];
}
```

**Python Serialization**: See `src/core/schemas.py` CueParams.to_bytes()

**Firmware Parsing**: See HaptosReceiver.ino parsePacket()

---

## Synthesis Pipeline

### Continuous Cues (Interpolated @ 2kHz)

**1. Texture Cue**
- **Input**: `texture_grain_hz`, `texture_amplitude`
- **Synthesis**:
  - Grain oscillator @ texture_grain_hz (sine wave)
  - White noise filtered by grain frequency
  - Mixed 50% grain + 30% noise
- **Purpose**: Surface roughness perception

**2. Shear Cue**
- **Input**: `shear_direction_x`, `shear_direction_y`, `shear_magnitude`
- **Synthesis**:
  - Directional modulation (encode direction as frequency shift)
  - Base frequency: 100 Hz + direction_x * 50 Hz
  - Amplitude: shear_magnitude
- **Purpose**: Lateral force perception

**3. Weight Cue**
- **Input**: `weight_offset`
- **Synthesis**:
  - DC offset / amplitude modulation
  - Applied as final gain stage
- **Purpose**: Normal force / pressure perception

### Transient Cues (Triggered by impulse flag)

**4. Impact Cue**
- **Input**: `impact_amplitude`, `impact_decay_ms`, `impact_frequency_hz`
- **Trigger**: `trigger_impulse == 1`
- **Synthesis**:
  - Sine wave @ impact_frequency_hz
  - Exponential decay: `amp * exp(-3 * t / decay_ms)`
  - Duration: ~3x decay_ms
- **Purpose**: Initial contact / collision perception

**5. Ring Cue**
- **Input**: `ring_amplitude`, `ring_decay_ms`
- **Trigger**: `trigger_impulse == 1` AND `ring_amplitude > 0.01`
- **Synthesis**:
  - Sine wave @ impact_frequency_hz * 1.2 (resonant mode)
  - Exponential decay (slower than impact)
- **Purpose**: Material resonance perception

### Interpolation Strategy

**Continuous Parameters**: Smoothed every synthesis tick (2kHz)
```c
current = current + alpha * (target - current)
```
where `alpha = 0.3` (adjustable smoothing factor)

**Transient Parameters**: NOT interpolated (discrete triggers)
- Impact/ring are triggered once per impulse
- Decay naturally over time
- New trigger replaces previous transient

---

## Performance Characteristics

### Latency Budget

| Stage | Time | Notes |
|-------|------|-------|
| Serial TX (host) | ~4.5 ms | 52 bytes @ 115200 baud |
| Serial RX (firmware) | ~4.5 ms | Buffered reception |
| Packet parsing | <0.1 ms | Validation + memcpy |
| Synthesis tick | 0.5 ms | 2kHz update rate |
| **Total** | **~10 ms** | Meets <20ms Phase 1 target |

### Bandwidth

**Single Channel**:
- Packet size: 52 bytes = 416 bits
- Packet rate: 100 Hz (from neural renderer)
- Bandwidth: 41.6 kbps (well within 115200 baud)

**6 Channels (Phase 2)**:
- Total bandwidth: 249.6 kbps (2.2x serial capacity)
- **Solution**: Time-multiplexed transmission or higher baud rate (460800)

---

## Firmware Development Roadmap

### Phase 3: Serial Driver Implementation (Python)
- [ ] Replace MockHardwareDriver with SerialHardwareDriver
- [ ] Port detection and auto-connection
- [ ] ACK/NACK handling (optional)
- [ ] File: `src/hardware/serial_driver.py`

### Phase 4: Real Hardware Validation
- [ ] Breadboard Teensy + VCA circuit
- [ ] Load cell feedback loop (validate CueParams accuracy)
- [ ] Measure end-to-end latency on real hardware
- [ ] Calibration tools for VCA output

### Phase 4+: Multi-Channel Support
- [ ] Implement channel multiplexing in firmware
- [ ] 6-channel DriverManager → 6 serial ports OR I2C/SPI bus
- [ ] Synchronized output across channels

### Phase 5: Firmware Optimization
- [ ] On-device neural inference (optional latency reduction)
- [ ] Model quantization for Teensy (INT8 or float16)
- [ ] Custom synthesis algorithms (beyond Teensy Audio Library)

---

## Troubleshooting

### No Serial Communication
1. Check baud rate matches (115200 both sides)
2. Verify USB cable supports data (not power-only)
3. Check port: `ls /dev/tty*` (Linux/Mac) or Device Manager (Windows)
4. Ensure no other program is using the serial port

### Checksum Errors
1. Verify byte order (little-endian)
2. Check packet size exactly 52 bytes
3. Python: Use `struct.pack('<f', value)` for floats
4. Test with known-good packet (see unit tests)

### No Haptic Output
1. Check Audio Shield / DAC wiring
2. Verify actuator connection (not headphones!)
3. Check mixer gain levels (see synthesizeCues())
4. Test with CueParams.weight_offset > 0.5 (should feel constant pressure)

### Packet Loss
1. Reduce packet rate (100 Hz → 50 Hz)
2. Increase serial buffer size in firmware
3. Use hardware flow control (CTS/RTS)
4. Consider higher baud rate (230400 or 460800)

---

## References

- **Architecture**: `docs/ARCHITECTURE.md`
- **Protocol Spec**: `docs/specs/firmware_protocol.md`
- **Phase 1 Summary**: `PHASE1_COMPLETE.md`
- **Phase 2 Summary**: `PHASE2_COMPLETE.md`
- **Python Schemas**: `src/core/schemas.py`
- **Mock Driver**: `src/hardware/mock_driver.py`

---

**Status**: Firmware ready for Phase 4 hardware integration testing.

**Next Steps**: Create SerialHardwareDriver in Python to replace mock driver for real Teensy communication.
