/*
 * HaptosRuntime.ino - Neural Haptic Rendering Firmware
 * 
 * Reads load cell → extracts features → runs neural inference → outputs haptic feedback
 * Target: Teensy 4.1 + AD620 Load Cell Amp + MAX98357A I2S DAC
 */

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

#include "inference_core.h"

// ============================================================================
// CONFIGURATION
// ============================================================================
#define LOAD_CELL_PIN       A0
#define ADC_RESOLUTION      12        // 12-bit (0-4095)
#define ADC_SAMPLES_PER_WINDOW 10     // 10 samples @ 1kHz = 10ms window
#define LOOP_INTERVAL_US    1000      // 1ms = 1kHz sample rate
#define INFERENCE_INTERVAL  10        // Run inference every 10 ADC samples (100Hz)

// Calibration: Convert ADC to Newtons
// Adjust these based on your load cell calibration
#define ADC_ZERO_OFFSET     100       // ADC value at zero force
#define FORCE_SCALE         10.0f     // Max force in Newtons at full scale

// Phase detection thresholds
#define IMPACT_FORCE_THRESHOLD  0.3f  // N
#define RELEASE_THRESHOLD       0.1f  // N
#define HOLD_TIME_MS            20    // ms before transitioning IMPACT → HOLD

// Material constants (metal bar)
#define HARDNESS_01         0.7f
#define MU_01               0.5f
#define ROUGHNESS_UM        5.0f

// ============================================================================
// AUDIO SYSTEM
// ============================================================================
AudioSynthWaveformSine   carrier;           // Primary carrier wave
AudioSynthWaveformSine   modulator;         // For ring modulation
AudioSynthNoiseWhite     textureNoise;      // Texture layer
AudioEffectMultiply      ringMod;           // Ring modulation
AudioMixer4              mixer;             // Mix all components
AudioAmplifier           outputGain;        // Final amplitude control
AudioOutputI2S           i2sOutput;

// Patch cords
AudioConnection          pc1(carrier, 0, ringMod, 0);
AudioConnection          pc2(modulator, 0, ringMod, 1);
AudioConnection          pc3(ringMod, 0, mixer, 0);
AudioConnection          pc4(textureNoise, 0, mixer, 1);
AudioConnection          pc5(mixer, 0, outputGain, 0);
AudioConnection          pc6(outputGain, 0, i2sOutput, 0);
AudioConnection          pc7(outputGain, 0, i2sOutput, 1);

// ============================================================================
// NEURAL INFERENCE ENGINE
// ============================================================================
HaptosInference neuralNet;
HapticParams    currentParams;

// ============================================================================
// PHASE DETECTION STATE
// ============================================================================
enum Phase {
    PHASE_NO_CONTACT = 0,
    PHASE_IMPACT = 1,
    PHASE_HOLD = 2,
    PHASE_SLIP = 3,
    PHASE_RELEASE = 4
};

Phase currentPhase = PHASE_NO_CONTACT;
Phase prevPhase = PHASE_NO_CONTACT;
unsigned long phaseStartTime = 0;
float prevForce = 0.0f;

// ============================================================================
// ADC BUFFER
// ============================================================================
float forceBuffer[ADC_SAMPLES_PER_WINDOW];
int bufferIndex = 0;
unsigned long lastSampleTime = 0;
int sampleCount = 0;

// ============================================================================
// FEATURE EXTRACTION
// ============================================================================
float extractedFeatures[13];

void buildFeatureVector(float avgForce) {
    // Phase one-hot encoding [0-4]
    for (int i = 0; i < 5; i++) {
        extractedFeatures[i] = (i == currentPhase) ? 1.0f : 0.0f;
    }
    
    // Phase confidence [5]
    extractedFeatures[5] = 0.85f;
    
    // Normal force (log scale) [6]
    extractedFeatures[6] = log10f(avgForce + 1.0f);
    
    // Shear force (not available, set to 0) [7]
    extractedFeatures[7] = 0.0f;
    
    // Slip speed (not available, set to 0) [8]
    extractedFeatures[8] = 0.0f;
    
    // Material properties (constants for metal bar) [9-11]
    extractedFeatures[9] = HARDNESS_01;
    extractedFeatures[10] = MU_01;
    extractedFeatures[11] = ROUGHNESS_UM;
    
    // Uncertainty [12]
    extractedFeatures[12] = 0.0f;
}

// ============================================================================
// PHASE DETECTION FSM
// ============================================================================
Phase detectPhase(float force, unsigned long now) {
    float dForce = force - prevForce;
    
    // NO_CONTACT → check for impact
    if (force < RELEASE_THRESHOLD) {
        phaseStartTime = now;
        return PHASE_NO_CONTACT;
    }
    
    // IMPACT: Rising force edge from no contact
    if (prevPhase == PHASE_NO_CONTACT && force > IMPACT_FORCE_THRESHOLD) {
        phaseStartTime = now;
        return PHASE_IMPACT;
    }
    
    // IMPACT → HOLD after settling time
    if (prevPhase == PHASE_IMPACT) {
        if ((now - phaseStartTime) > HOLD_TIME_MS) {
            return PHASE_HOLD;
        }
        return PHASE_IMPACT;
    }
    
    // HOLD → RELEASE on force drop
    if (prevPhase == PHASE_HOLD && force < IMPACT_FORCE_THRESHOLD * 0.5f) {
        phaseStartTime = now;
        return PHASE_RELEASE;
    }
    
    // RELEASE → NO_CONTACT
    if (prevPhase == PHASE_RELEASE && force < RELEASE_THRESHOLD) {
        return PHASE_NO_CONTACT;
    }
    
    // Stay in current phase
    return prevPhase;
}

// ============================================================================
// APPLY HAPTIC PARAMS TO AUDIO
// ============================================================================
void applyHapticParams() {
    // Base carrier frequency from ring modes
    float carrierFreq = currentParams.ring_f_Hz[0];
    carrierFreq = constrain(carrierFreq, 80.0f, 400.0f);
    carrier.frequency(carrierFreq);
    
    // Modulator for texture
    modulator.frequency(carrierFreq * 0.5f);
    modulator.amplitude(currentParams.ring_a[0] * 0.3f);
    
    // Base amplitude from weight
    float baseAmp = currentParams.weight_A;
    
    // Impact transient boost
    float impactBoost = 0.0f;
    if (currentPhase == PHASE_IMPACT) {
        impactBoost = currentParams.impact_A * 0.5f;
    }
    
    // Texture noise level
    float textureLevel = currentParams.texture_A * 0.1f;
    textureNoise.amplitude(textureLevel);
    
    // Mixer levels
    mixer.gain(0, 0.8f);  // Ring mod carrier
    mixer.gain(1, textureLevel);  // Texture noise
    
    // Final output amplitude
    float outputAmp = constrain(baseAmp + impactBoost, 0.0f, 1.0f);
    outputGain.gain(outputAmp);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // Wait for serial (max 3s)
    
    Serial.println("=================================");
    Serial.println("  HaptOS Runtime v1.0");
    Serial.println("  Neural Haptic Rendering");
    Serial.println("=================================");
    
    // Configure ADC
    analogReadResolution(ADC_RESOLUTION);
    pinMode(LOAD_CELL_PIN, INPUT);
    
    // Initialize audio system
    AudioMemory(20);
    
    carrier.amplitude(1.0f);
    carrier.frequency(150.0f);
    
    modulator.amplitude(0.0f);
    modulator.frequency(75.0f);
    
    textureNoise.amplitude(0.0f);
    
    mixer.gain(0, 0.8f);
    mixer.gain(1, 0.1f);
    mixer.gain(2, 0.0f);
    mixer.gain(3, 0.0f);
    
    outputGain.gain(0.0f);
    
    // Initialize force buffer
    for (int i = 0; i < ADC_SAMPLES_PER_WINDOW; i++) {
        forceBuffer[i] = 0.0f;
    }
    
    // Initialize default haptic params
    currentParams = neuralNet.predict(extractedFeatures);
    
    Serial.println("\nReady. Press the load cell...\n");
}

// ============================================================================
// MAIN LOOP - 1kHz sampling, 100Hz inference
// ============================================================================
void loop() {
    unsigned long now = micros();
    
    // Sample at 1kHz
    if (now - lastSampleTime >= LOOP_INTERVAL_US) {
        lastSampleTime = now;
        
        // Read ADC
        int adcRaw = analogRead(LOAD_CELL_PIN);
        
        // Convert to Newtons
        float force = ((float)(adcRaw - ADC_ZERO_OFFSET) / (float)(4095 - ADC_ZERO_OFFSET)) * FORCE_SCALE;
        force = max(0.0f, force);  // Clamp negative
        
        // Store in buffer
        forceBuffer[bufferIndex] = force;
        bufferIndex = (bufferIndex + 1) % ADC_SAMPLES_PER_WINDOW;
        sampleCount++;
        
        // Every 10 samples (100Hz), run inference
        if (sampleCount >= INFERENCE_INTERVAL) {
            sampleCount = 0;
            
            // Calculate average force
            float avgForce = 0.0f;
            for (int i = 0; i < ADC_SAMPLES_PER_WINDOW; i++) {
                avgForce += forceBuffer[i];
            }
            avgForce /= ADC_SAMPLES_PER_WINDOW;
            
            // Detect phase
            currentPhase = detectPhase(avgForce, millis());
            
            // Build feature vector
            buildFeatureVector(avgForce);
            
            // Run neural inference
            currentParams = neuralNet.predict(extractedFeatures);
            
            // Apply to audio system
            applyHapticParams();
            
            // Debug output
            static unsigned long lastPrint = 0;
            if (millis() - lastPrint > 100) {  // Print at 10Hz
                lastPrint = millis();
                
                Serial.print("Force: ");
                Serial.print(avgForce, 2);
                Serial.print("N | Phase: ");
                
                switch (currentPhase) {
                    case PHASE_NO_CONTACT: Serial.print("NO_CONTACT"); break;
                    case PHASE_IMPACT: Serial.print("IMPACT    "); break;
                    case PHASE_HOLD: Serial.print("HOLD      "); break;
                    case PHASE_SLIP: Serial.print("SLIP      "); break;
                    case PHASE_RELEASE: Serial.print("RELEASE   "); break;
                }
                
                Serial.print(" | weight.A: ");
                Serial.print(currentParams.weight_A, 3);
                Serial.print(" | impact.A: ");
                Serial.print(currentParams.impact_A, 3);
                Serial.print(" | ring.f: ");
                Serial.println(currentParams.ring_f_Hz[0], 1);
            }
            
            // Update previous state
            prevPhase = currentPhase;
            prevForce = avgForce;
        }
    }
}
