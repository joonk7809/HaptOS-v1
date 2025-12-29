#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// ============================================================================
// 1. AUDIO SYSTEM (The Output Pipeline)
// ============================================================================
AudioSynthWaveformSine   carrierWave;    // The Source (150Hz vibration)
AudioAmplifier           hapticGain;     // The Volume Knob
AudioOutputI2S           i2sOutput;      // The Driver (MAX98357A)

AudioConnection          patchCord1(carrierWave, 0, hapticGain, 0);
AudioConnection          patchCord2(hapticGain, 0, i2sOutput, 0);
AudioConnection          patchCord3(hapticGain, 0, i2sOutput, 1); // Copy to right ch

// ============================================================================
// 2. SETUP
// ============================================================================
void setup() {
  AudioMemory(12);

  // Set the "Texture" of the vibration
  // 150Hz is the "Sweet Spot" for most exciters (Maximum resonance)
  carrierWave.amplitude(1.0); 
  carrierWave.frequency(150.0);

  // Start at 0 volume
  hapticGain.gain(0.0);
  
  Serial.begin(9600);
  Serial.println("Haptic Output Test Initialized...");
}

// ============================================================================
// 3. THE PHANTOM LOOP
// ============================================================================
void loop() {
  // Instead of reading a sensor, we use MATH to create a "Virtual Finger"
  // millis() counts up forever. 
  // We use sin() to oscillate between -1 and 1.
  
  float time_sec = millis() / 1000.0;
  
  // Create a 0.5Hz wave (Press... Release... Press...)
  float phantom_force = (sin(time_sec * PI) + 1.0) / 2.0; 
  // Result: A value that smoothly floats between 0.0 and 1.0
  
  // Apply this "phantom force" to the amplifier
  hapticGain.gain(phantom_force);

  // Debug visualization
  Serial.print("Phantom_Force:");
  Serial.println(phantom_force);

  // Update speed
  delay(10);
}