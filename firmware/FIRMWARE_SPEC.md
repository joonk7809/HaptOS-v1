# HAPTOS Full-Body Firmware Specification

**Version**: 2.0
**Target**: Teensy 4.1
**Protocol**: SensationParams (38-byte packets)
**Update Rate**: 100Hz per body part

---

## Overview

Each Teensy 4.1 controls one body segment (8-10 actuators). The firmware receives SensationParams packets over serial and drives actuators using hardware-specific synthesis.

### Segment Assignment

Each Teensy is flashed with a **segment ID** at compile time:

```cpp
// config.h - Set per device
#define SEGMENT_ID 2  // 0=HEAD, 1=TORSO, 2=LEFT_ARM, 3=RIGHT_ARM, 4=LEFT_LEG, 5=RIGHT_LEG
```

---

## Packet Protocol

### Packet Format (38 bytes)

```
Offset  Size  Type      Field
────────────────────────────────────────────────────────
0-1     2     uint8_t   Header: 0xBB 0x66
2       1     uint8_t   Version: 0x01
3       1     uint8_t   Body Part ID (local, 0-255)
4-7     4     uint32_t  Timestamp (microseconds)
8-35    28    float16   Sensation params (14 values)
36      1     uint8_t   Flags (bit 0 = impact_trigger)
37      1     uint8_t   Checksum (XOR of bytes 0-36)
```

### Sensation Parameters (14× float16)

All values in [0, 1]:

```cpp
struct SensationParams {
    float impact_intensity;      // [0] 0=none, 1=max
    float impact_sharpness;       // [1] 0=soft, 1=sharp
    float resonance_intensity;    // [2] Ringing magnitude
    float resonance_brightness;   // [3] 0=dark/low freq, 1=bright/high freq
    float resonance_sustain;      // [4] 0=damped, 1=long ring
    float texture_roughness;      // [5] 0=smooth, 1=rough
    float texture_density;        // [6] 0=sparse, 1=fine
    float texture_depth;          // [7] 0=shallow, 1=deep
    float slip_speed;             // [8] 0=static, 1=fast
    float slip_direction_x;       // [9] Unit vector
    float slip_direction_y;       // [10] Unit vector
    float slip_grip;              // [11] 0=sticky, 1=slippery
    float pressure_magnitude;     // [12] 0=light, 1=heavy
    float pressure_spread;        // [13] 0=point, 1=diffuse
};
```

---

## Body Part → Actuator Mapping

Each Teensy has a compile-time mapping from local body part ID to actuator channel.

### Example: LEFT_ARM Segment (SEGMENT_ID = 2)

```cpp
// config.h - LEFT_ARM mapping
const uint8_t BODY_PART_TO_ACTUATOR[32] = {
    0,  // [0]  left_shoulder → actuator 0
    1,  // [1]  left_upper_arm → actuator 1
    1,  // [2]  left_upper_arm_inner → actuator 1 (shared)
    2,  // [3]  left_elbow → actuator 2
    3,  // [4]  left_forearm → actuator 3
    3,  // [5]  left_forearm_inner → actuator 3 (shared)
    4,  // [6]  left_wrist → actuator 4
    5,  // [7]  left_palm → actuator 5
    5,  // [8]  left_palm_heel → actuator 5 (shared)
    6,  // [9]  left_hand_back → actuator 6
    7,  // [10] left_thumb_base → actuator 7
    7,  // [11] left_thumb_mid → actuator 7 (shared)
    8,  // [12] left_thumb_tip → actuator 8
    8,  // [13] left_thumb_nail → actuator 8 (shared)
    8,  // [14] left_index_base → actuator 8 (glove, shared)
    8,  // [15] left_index_mid → actuator 8
    9,  // [16] left_index_tip → actuator 9 (fingertip, dedicated)
    // ... continue for all 28 parts in LEFT_ARM
};

#define NUM_ACTUATORS 10
```

**Key Points**:
- Multiple body parts can share one actuator
- Fingertips (high resolution) get dedicated actuators
- Forearm/upper arm share actuators (low resolution)

---

## Hardware Tier Support

### Tier 1: VCA (Voice Coil Actuator)
- Frequency range: 20-500Hz
- All 5 sensation channels synthesized
- Used for: Fingertips, toes

**Synthesis**:
```cpp
void synthesize_VCA(SensationParams s, uint8_t channel) {
    // Impact: Decaying sine burst
    if (s.impact_trigger) {
        float freq = lerp(20, 500, s.impact_sharpness);
        float decay_ms = lerp(50, 2, s.impact_sharpness);
        trigger_impact(channel, s.impact_intensity, freq, decay_ms);
    }

    // Resonance: Damped oscillation
    float res_freq = lerp(20, 500, s.resonance_brightness);
    float res_decay = lerp(5, 200, s.resonance_sustain);
    set_resonance(channel, s.resonance_intensity, res_freq, res_decay);

    // Texture: Bandpass filtered noise
    float tex_freq = lerp(20, 500, s.texture_density);
    float tex_bw = lerp(10, 200, s.texture_roughness);
    set_texture(channel, s.texture_depth, tex_freq, tex_bw);

    // Slip: AM noise
    set_slip(channel, s.slip_speed, s.slip_direction_x, s.slip_direction_y);

    // Pressure: Sustained low-freq tone
    set_pressure(channel, s.pressure_magnitude, 20.0);
}
```

### Tier 2: LRA (Linear Resonant Actuator)
- Fixed resonant frequency: ~175Hz
- Amplitude modulation only
- Used for: Palm, sole

**Synthesis**:
```cpp
void synthesize_LRA(SensationParams s, uint8_t channel) {
    const float F_RES = 175.0;  // Resonant frequency

    // All channels map to resonant carrier
    float amplitude = 0.0;
    amplitude += s.impact_intensity * 1.0;
    amplitude += s.resonance_intensity * 0.8;
    amplitude += s.texture_roughness * s.texture_depth * 0.5;
    amplitude += s.pressure_magnitude * 0.3;

    set_lra_amplitude(channel, amplitude, F_RES);
}
```

### Tier 3: ERM (Eccentric Rotating Mass)
- Magnitude only (duty cycle PWM)
- ~50ms spin-up time
- Used for: Torso, thighs

**Synthesis**:
```cpp
void synthesize_ERM(SensationParams s, uint8_t channel) {
    // Collapse all channels to single intensity
    float intensity = 0.0;
    intensity += s.impact_intensity * 1.0;
    intensity += s.texture_roughness * s.texture_depth * 0.5;
    intensity += s.pressure_magnitude * 0.3;

    set_erm_duty_cycle(channel, min(intensity, 1.0));
}
```

---

## Packet Reception

### Main Loop (100Hz)

```cpp
void loop() {
    // Check for incoming packet
    if (Serial.available() >= 38) {
        uint8_t packet[38];
        Serial.readBytes(packet, 38);

        // Validate header
        if (packet[0] != 0xBB || packet[1] != 0x66) {
            return;  // Invalid header
        }

        // Validate checksum
        uint8_t checksum = 0;
        for (int i = 0; i < 37; i++) {
            checksum ^= packet[i];
        }
        if (checksum != packet[37]) {
            return;  // Checksum failed
        }

        // Parse packet
        uint8_t body_part_id = packet[3];
        uint32_t timestamp = *((uint32_t*)&packet[4]);

        SensationParams s;
        deserialize_sensation(&s, &packet[8]);  // 14× float16 from bytes 8-35

        bool impact_trigger = packet[36] & 0x01;
        s.impact_trigger = impact_trigger;

        // Route to actuator
        uint8_t actuator = BODY_PART_TO_ACTUATOR[body_part_id];
        uint8_t tier = ACTUATOR_TIER[actuator];  // Pre-configured

        switch (tier) {
            case 1: synthesize_VCA(s, actuator); break;
            case 2: synthesize_LRA(s, actuator); break;
            case 3: synthesize_ERM(s, actuator); break;
        }
    }

    // Synthesis update (2kHz)
    update_synthesis();
}
```

---

## Configuration Files

### Per-Segment Configs

Each segment needs a custom `config.h`:

**HEAD** (SEGMENT_ID = 0):
```cpp
#define SEGMENT_ID 0
#define NUM_ACTUATORS 8
const uint8_t BODY_PART_TO_ACTUATOR[12] = {
    0, 1, 2, 3, 4, 5, 6, 7, 6, 7, 6, 7
};
const uint8_t ACTUATOR_TIER[8] = {3, 3, 3, 3, 3, 3, 3, 3};  // All ERM
```

**TORSO** (SEGMENT_ID = 1):
```cpp
#define SEGMENT_ID 1
#define NUM_ACTUATORS 8
const uint8_t BODY_PART_TO_ACTUATOR[12] = {
    0, 1, 2, 3, 4, 5, 6, 7, 4, 5, 6, 7
};
const uint8_t ACTUATOR_TIER[8] = {3, 3, 3, 3, 3, 3, 3, 3};  // All ERM
```

**LEFT_ARM** (SEGMENT_ID = 2):
```cpp
#define SEGMENT_ID 2
#define NUM_ACTUATORS 10
const uint8_t BODY_PART_TO_ACTUATOR[28] = {
    0, 1, 1, 2, 3, 3, 4, 5, 5, 6, 7, 7, 8, 8, 8, 8, 9, 9, ...
};
const uint8_t ACTUATOR_TIER[10] = {
    2, 2, 2, 2, 2, 2, 2, 2, 1, 1  // Forearm=LRA, Fingertips=VCA
};
```

---

## Bandwidth Analysis

### Per-Segment Load

```
Tier 1 (VCA):  38 bytes × 100 Hz × 2 parts  = 7.6 KB/s
Tier 2 (LRA):  38 bytes × 100 Hz × 4 parts  = 15.2 KB/s
Tier 3 (ERM):  38 bytes × 50 Hz × 4 parts   = 7.6 KB/s
──────────────────────────────────────────────────────────
Total per segment:                            ~30 KB/s
```

### Serial Capacity

```
115200 baud:  14.4 KB/s (not enough for high-density segments)
230400 baud:  28.8 KB/s (sufficient)
460800 baud:  57.6 KB/s (margin for burst traffic)
```

**Recommendation**:
- LEFT_ARM, RIGHT_ARM: 230400 baud (high actuator density)
- Other segments: 115200 baud (sufficient)

---

## Status Reporting (Future)

Teensy → Host status updates (10Hz):

```
Offset  Size  Type      Field
────────────────────────────────────────────────────────
0-1     2     uint8_t   Header: 0xCC 0xDD
2       1     uint8_t   Segment ID
3       1     uint8_t   Status flags (connected, error, etc.)
4-7     4     uint32_t  Packets received
8-11    4     uint32_t  Checksum errors
12-15   4     uint32_t  Timestamp
16      1     uint8_t   Checksum
```

---

## Testing

### Serial Echo Test

```cpp
// Test firmware responsiveness
void test_echo() {
    if (Serial.available()) {
        uint8_t byte = Serial.read();
        Serial.write(byte);  // Echo back
    }
}
```

### Actuator Test Pattern

```cpp
// Sweep through all actuators
void test_sweep() {
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        set_actuator(i, 0.5);  // 50% intensity
        delay(500);
        set_actuator(i, 0.0);
    }
}
```

---

## Build Instructions

### PlatformIO

```ini
; platformio.ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino

build_flags =
    -D SEGMENT_ID=2           ; Set segment ID
    -D USB_SERIAL             ; Enable USB serial
    -O2                       ; Optimize for speed

lib_deps =
    https://github.com/PaulStoffregen/Audio.git
```

### Arduino IDE

1. Install Teensyduino
2. Set Tools → Board → Teensy 4.1
3. Set Tools → USB Type → Serial
4. Edit `config.h` with correct SEGMENT_ID
5. Upload

---

## Future Enhancements

### Phase 2 Features

- [ ] Bidirectional communication (status, diagnostics)
- [ ] Multi-packet batching (reduce header overhead)
- [ ] Hardware flow control (RTS/CTS)
- [ ] Adaptive update rate (reduce bandwidth for inactive parts)
- [ ] On-device interpolation (smooth 50Hz → 2kHz synthesis)

### Advanced Synthesis

- [ ] Wavetable synthesis for complex textures
- [ ] Physical modeling for realistic materials
- [ ] Spatial audio techniques for direction rendering

---

## References

- **Teensy 4.1 Datasheet**: https://www.pjrc.com/store/teensy41.html
- **Serial Protocol**: UART at 230400 baud, 8N1
- **float16 Spec**: IEEE 754 half-precision (16-bit floating point)
- **Actuator Specs**:
  - VCA: 20-500Hz, <5ms response
  - LRA: ~175Hz resonant, <20ms response
  - ERM: 50-200Hz, ~50ms spin-up

---

**Last Updated**: February 2026
