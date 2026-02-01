/*
 * HaptosReceiver.ino - Phase 1/2 CueParams Protocol Receiver
 *
 * Receives 52-byte CueParams packets over serial from host (Python)
 * Implements interpolated synthesis and hardware output
 *
 * Protocol: Phase 1 Architecture (PHASE1_COMPLETE.md)
 * Target: Teensy 4.1 + Audio Shield / MAX98357A I2S DAC
 *
 * HAPTOS Platform - Three Layer Architecture
 * ============================================
 * Layer 1 (Host): Physics Engine @ 1kHz → ContactPatch
 * Layer 2 (Host): Neural Renderer @ 100Hz → CueParams
 * Layer 3 (THIS): Hardware Driver @ 2kHz+ → Actuator
 */

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>

// ============================================================================
// PACKET PROTOCOL - Matches src/core/schemas.py CueParams
// ============================================================================
#define PACKET_SIZE 52
#define PACKET_HEADER_0 0xAA
#define PACKET_HEADER_1 0x55

// Packet structure (52 bytes total):
// [0-1]   Header (0xAA 0x55)
// [2-5]   timestamp_us (uint32_t)
// [6-9]   texture_grain_hz (float32)
// [10-13] texture_amplitude (float32)
// [14-17] shear_direction_x (float32)
// [18-21] shear_direction_y (float32)
// [22-25] shear_magnitude (float32)
// [26-29] weight_offset (float32)
// [30-33] impact_amplitude (float32)
// [34-37] impact_decay_ms (float32)
// [38-41] impact_frequency_hz (float32)
// [42-45] ring_amplitude (float32)
// [46-49] ring_decay_ms (float32)
// [50]    trigger_impulse (uint8_t)
// [51]    checksum (XOR of bytes [0-50])

struct CueParams {
    uint32_t timestamp_us;

    // Continuous cues (interpolated)
    float texture_grain_hz;
    float texture_amplitude;
    float shear_direction_x;
    float shear_direction_y;
    float shear_magnitude;
    float weight_offset;

    // Transient cues (triggered)
    float impact_amplitude;
    float impact_decay_ms;
    float impact_frequency_hz;
    float ring_amplitude;
    float ring_decay_ms;
    bool trigger_impulse;
};

// ============================================================================
// CONFIGURATION
// ============================================================================
#define SERIAL_BAUD 115200
#define SYNTH_RATE_HZ 2000          // Hardware synthesis update rate
#define SYNTH_INTERVAL_US 500       // 1000000 / SYNTH_RATE_HZ
#define ACK_ENABLED false           // Send ACK/NACK responses

// ============================================================================
// AUDIO SYSTEM - Phase 1: Single VCA Channel
// ============================================================================
AudioSynthWaveformSine   textureOsc;         // Texture grain oscillator
AudioSynthNoiseWhite     textureNoise;       // Texture noise
AudioSynthWaveformSine   impactOsc;          // Impact transient
AudioSynthWaveformSine   ringOsc;            // Ring-down transient
AudioSynthWaveformSine   shearOsc;           // Shear direction cue
AudioMixer4              mixer1;             // Mix texture + impact
AudioMixer4              mixer2;             // Mix ring + shear
AudioMixer4              finalMix;           // Final output mix
AudioAmplifier           weightGain;         // Weight (DC offset)
AudioOutputI2S           i2sOutput;          // I2S DAC output

// Audio connections
AudioConnection pc1(textureOsc, 0, mixer1, 0);
AudioConnection pc2(textureNoise, 0, mixer1, 1);
AudioConnection pc3(impactOsc, 0, mixer1, 2);
AudioConnection pc4(ringOsc, 0, mixer2, 0);
AudioConnection pc5(shearOsc, 0, mixer2, 1);
AudioConnection pc6(mixer1, 0, finalMix, 0);
AudioConnection pc7(mixer2, 0, finalMix, 1);
AudioConnection pc8(finalMix, 0, weightGain, 0);
AudioConnection pc9(weightGain, 0, i2sOutput, 0);
AudioConnection pc10(weightGain, 0, i2sOutput, 1);

// ============================================================================
// STATE VARIABLES
// ============================================================================
CueParams currentCues;
CueParams targetCues;       // For interpolation
uint8_t packetBuffer[PACKET_SIZE];
int packetIndex = 0;
bool packetReady = false;

// Transient state
unsigned long impactStartTime = 0;
unsigned long ringStartTime = 0;
bool impactActive = false;
bool ringActive = false;

// Statistics
unsigned long packetsReceived = 0;
unsigned long packetsDropped = 0;
unsigned long checksumErrors = 0;

// Timing
unsigned long lastSynthTime = 0;

// ============================================================================
// PACKET PARSING
// ============================================================================
uint8_t calculateChecksum(uint8_t* data, int len) {
    uint8_t xor_sum = 0;
    for (int i = 0; i < len; i++) {
        xor_sum ^= data[i];
    }
    return xor_sum;
}

bool parsePacket(uint8_t* buffer, CueParams* params) {
    // Validate header
    if (buffer[0] != PACKET_HEADER_0 || buffer[1] != PACKET_HEADER_1) {
        return false;
    }

    // Validate checksum
    uint8_t computed = calculateChecksum(buffer, PACKET_SIZE - 1);
    if (computed != buffer[PACKET_SIZE - 1]) {
        checksumErrors++;
        return false;
    }

    // Parse fields (little-endian)
    int offset = 2;

    memcpy(&params->timestamp_us, buffer + offset, 4);
    offset += 4;

    memcpy(&params->texture_grain_hz, buffer + offset, 4);
    offset += 4;

    memcpy(&params->texture_amplitude, buffer + offset, 4);
    offset += 4;

    memcpy(&params->shear_direction_x, buffer + offset, 4);
    offset += 4;

    memcpy(&params->shear_direction_y, buffer + offset, 4);
    offset += 4;

    memcpy(&params->shear_magnitude, buffer + offset, 4);
    offset += 4;

    memcpy(&params->weight_offset, buffer + offset, 4);
    offset += 4;

    memcpy(&params->impact_amplitude, buffer + offset, 4);
    offset += 4;

    memcpy(&params->impact_decay_ms, buffer + offset, 4);
    offset += 4;

    memcpy(&params->impact_frequency_hz, buffer + offset, 4);
    offset += 4;

    memcpy(&params->ring_amplitude, buffer + offset, 4);
    offset += 4;

    memcpy(&params->ring_decay_ms, buffer + offset, 4);
    offset += 4;

    params->trigger_impulse = buffer[offset];

    return true;
}

// ============================================================================
// INTERPOLATION - Smooth parameter updates @ 2kHz
// ============================================================================
float interpolate(float current, float target, float alpha) {
    return current + alpha * (target - current);
}

void updateInterpolation() {
    // Interpolation factor (alpha = 0.3 gives smooth transitions)
    const float alpha = 0.3f;

    // Continuous parameters
    currentCues.texture_grain_hz = interpolate(
        currentCues.texture_grain_hz,
        targetCues.texture_grain_hz,
        alpha
    );

    currentCues.texture_amplitude = interpolate(
        currentCues.texture_amplitude,
        targetCues.texture_amplitude,
        alpha
    );

    currentCues.shear_magnitude = interpolate(
        currentCues.shear_magnitude,
        targetCues.shear_magnitude,
        alpha
    );

    currentCues.weight_offset = interpolate(
        currentCues.weight_offset,
        targetCues.weight_offset,
        alpha
    );

    // Shear direction (interpolate separately for smooth rotation)
    currentCues.shear_direction_x = interpolate(
        currentCues.shear_direction_x,
        targetCues.shear_direction_x,
        alpha
    );

    currentCues.shear_direction_y = interpolate(
        currentCues.shear_direction_y,
        targetCues.shear_direction_y,
        alpha
    );

    // Transient parameters are NOT interpolated (discrete triggers)
}

// ============================================================================
// SYNTHESIS PIPELINE
// ============================================================================
void synthesizeCues() {
    unsigned long now = millis();

    // === TEXTURE CUE ===
    if (currentCues.texture_amplitude > 0.01f) {
        // Texture = grain oscillator + noise (mixed)
        float grain_freq = constrain(currentCues.texture_grain_hz, 50.0f, 500.0f);
        textureOsc.frequency(grain_freq);
        textureOsc.amplitude(currentCues.texture_amplitude * 0.5f);
        textureNoise.amplitude(currentCues.texture_amplitude * 0.3f);
    } else {
        textureOsc.amplitude(0.0f);
        textureNoise.amplitude(0.0f);
    }

    // === SHEAR CUE ===
    if (currentCues.shear_magnitude > 0.01f) {
        // Shear = directional modulation (encode direction in freq shift)
        float shear_freq = 100.0f + (currentCues.shear_direction_x * 50.0f);
        shear_freq = constrain(shear_freq, 80.0f, 200.0f);
        shearOsc.frequency(shear_freq);
        shearOsc.amplitude(currentCues.shear_magnitude * 0.4f);
    } else {
        shearOsc.amplitude(0.0f);
    }

    // === IMPACT CUE (Transient) ===
    if (targetCues.trigger_impulse && !impactActive) {
        // Trigger new impact
        impactActive = true;
        impactStartTime = now;

        float impact_freq = constrain(targetCues.impact_frequency_hz, 80.0f, 400.0f);
        impactOsc.frequency(impact_freq);
        impactOsc.amplitude(targetCues.impact_amplitude);
    }

    if (impactActive) {
        // Exponential decay
        unsigned long elapsed = now - impactStartTime;
        float decay_ms = max(10.0f, targetCues.impact_decay_ms);
        float decay_factor = exp(-3.0f * (float)elapsed / decay_ms);

        if (decay_factor < 0.01f) {
            // Decay complete
            impactOsc.amplitude(0.0f);
            impactActive = false;
        } else {
            impactOsc.amplitude(targetCues.impact_amplitude * decay_factor);
        }
    }

    // === RING CUE (Transient) ===
    if (targetCues.trigger_impulse && targetCues.ring_amplitude > 0.01f && !ringActive) {
        // Trigger new ring
        ringActive = true;
        ringStartTime = now;

        // Ring frequency from impact frequency (ring = resonant mode)
        float ring_freq = constrain(targetCues.impact_frequency_hz * 1.2f, 100.0f, 500.0f);
        ringOsc.frequency(ring_freq);
        ringOsc.amplitude(targetCues.ring_amplitude);
    }

    if (ringActive) {
        // Exponential decay (slower than impact)
        unsigned long elapsed = now - ringStartTime;
        float decay_ms = max(50.0f, targetCues.ring_decay_ms);
        float decay_factor = exp(-3.0f * (float)elapsed / decay_ms);

        if (decay_factor < 0.01f) {
            ringOsc.amplitude(0.0f);
            ringActive = false;
        } else {
            ringOsc.amplitude(targetCues.ring_amplitude * decay_factor);
        }
    }

    // === WEIGHT CUE (DC offset) ===
    float weight = constrain(currentCues.weight_offset, 0.0f, 1.0f);
    weightGain.gain(weight);

    // === MIXER LEVELS ===
    mixer1.gain(0, 0.5f);  // Texture oscillator
    mixer1.gain(1, 0.3f);  // Texture noise
    mixer1.gain(2, 0.8f);  // Impact
    mixer1.gain(3, 0.0f);

    mixer2.gain(0, 0.6f);  // Ring
    mixer2.gain(1, 0.4f);  // Shear
    mixer2.gain(2, 0.0f);
    mixer2.gain(3, 0.0f);

    finalMix.gain(0, 0.7f);  // Texture/impact mix
    finalMix.gain(1, 0.5f);  // Ring/shear mix
    finalMix.gain(2, 0.0f);
    finalMix.gain(3, 0.0f);

    // Clear trigger flag after processing
    targetCues.trigger_impulse = false;
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    // Initialize serial
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000);  // Wait max 3s for serial

    Serial.println("=========================================");
    Serial.println("  HAPTOS Receiver - Phase 1/2 Protocol");
    Serial.println("  Waiting for CueParams packets...");
    Serial.println("=========================================");
    Serial.print("Packet size: ");
    Serial.print(PACKET_SIZE);
    Serial.println(" bytes");
    Serial.print("Baud rate: ");
    Serial.println(SERIAL_BAUD);
    Serial.print("Synthesis rate: ");
    Serial.print(SYNTH_RATE_HZ);
    Serial.println(" Hz");
    Serial.println();

    // Initialize audio system
    AudioMemory(20);

    // Initialize oscillators (silent)
    textureOsc.amplitude(0.0f);
    textureOsc.frequency(150.0f);

    textureNoise.amplitude(0.0f);

    impactOsc.amplitude(0.0f);
    impactOsc.frequency(150.0f);

    ringOsc.amplitude(0.0f);
    ringOsc.frequency(200.0f);

    shearOsc.amplitude(0.0f);
    shearOsc.frequency(100.0f);

    weightGain.gain(0.0f);

    // Initialize cue params to zero
    memset(&currentCues, 0, sizeof(CueParams));
    memset(&targetCues, 0, sizeof(CueParams));

    Serial.println("Ready.\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
    unsigned long now = micros();

    // === SERIAL PACKET RECEPTION ===
    while (Serial.available() > 0) {
        uint8_t byte = Serial.read();

        // State machine: look for header
        if (packetIndex == 0) {
            if (byte == PACKET_HEADER_0) {
                packetBuffer[packetIndex++] = byte;
            }
        } else if (packetIndex == 1) {
            if (byte == PACKET_HEADER_1) {
                packetBuffer[packetIndex++] = byte;
            } else {
                // Invalid header, reset
                packetIndex = 0;
            }
        } else {
            // Accumulate packet bytes
            packetBuffer[packetIndex++] = byte;

            if (packetIndex >= PACKET_SIZE) {
                // Packet complete
                packetReady = true;
                packetIndex = 0;
            }
        }
    }

    // === PACKET PROCESSING ===
    if (packetReady) {
        packetReady = false;

        CueParams newCues;
        if (parsePacket(packetBuffer, &newCues)) {
            // Valid packet received
            targetCues = newCues;
            packetsReceived++;

            // Optional: Send ACK
            if (ACK_ENABLED) {
                Serial.write(0x06);  // ACK byte
            }

            // Debug output (every 10th packet)
            if (packetsReceived % 10 == 0) {
                Serial.print("RX #");
                Serial.print(packetsReceived);
                Serial.print(" | texture: ");
                Serial.print(targetCues.texture_grain_hz, 1);
                Serial.print("Hz @ ");
                Serial.print(targetCues.texture_amplitude, 2);
                Serial.print(" | weight: ");
                Serial.print(targetCues.weight_offset, 2);
                Serial.print(" | impulse: ");
                Serial.println(targetCues.trigger_impulse ? "YES" : "no");
            }
        } else {
            // Invalid packet
            packetsDropped++;

            if (ACK_ENABLED) {
                Serial.write(0x15);  // NACK byte
            }
        }
    }

    // === SYNTHESIS @ 2kHz ===
    if (now - lastSynthTime >= SYNTH_INTERVAL_US) {
        lastSynthTime = now;

        // Update interpolation
        updateInterpolation();

        // Synthesize audio output
        synthesizeCues();
    }

    // === STATISTICS (every 5 seconds) ===
    static unsigned long lastStatsTime = 0;
    if (millis() - lastStatsTime > 5000) {
        lastStatsTime = millis();

        Serial.println("\n--- Statistics ---");
        Serial.print("Packets received: ");
        Serial.println(packetsReceived);
        Serial.print("Packets dropped: ");
        Serial.println(packetsDropped);
        Serial.print("Checksum errors: ");
        Serial.println(checksumErrors);

        if (packetsReceived > 0) {
            float success_rate = 100.0f * (float)packetsReceived / (float)(packetsReceived + packetsDropped);
            Serial.print("Success rate: ");
            Serial.print(success_rate, 1);
            Serial.println("%");
        }
        Serial.println();
    }
}
