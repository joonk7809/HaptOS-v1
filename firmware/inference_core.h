#ifndef INFERENCE_CORE_H
#define INFERENCE_CORE_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#include "../tests/mock_arduino.h"
#endif
#include <math.h>
#include "weights.h"

// ============================================================================
// MATH UTILS (TinyML Style)
// ============================================================================

// ReLU Activation: f(x) = max(0, x)
inline float relu(float x) {
    return x > 0.0f ? x : 0.0f;
}

// Sigmoid Activation: f(x) = 1 / (1 + exp(-x))
inline float sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}

// Dense Layer: Output = Weights * Input + Bias
// input: [in_dim]
// weights: [out_dim * in_dim] (Flattened, Row-Major)
// bias: [out_dim]
// output: [out_dim]
void dense_layer(const float* input, int in_dim,
                 const float* weights, const float* bias,
                 float* output, int out_dim,
                 bool apply_relu) {
    
    for (int i = 0; i < out_dim; i++) {
        float sum = bias[i];
        
        // Dot product
        for (int j = 0; j < in_dim; j++) {
            // Weight matrix is [out_dim][in_dim] flattened
            // index = i * in_dim + j
            sum += input[j] * weights[i * in_dim + j];
        }
        
        if (apply_relu) {
            output[i] = relu(sum);
        } else {
            output[i] = sum;
        }
    }
}

// ============================================================================
// HAPTOS INFERENCE ENGINE
// ============================================================================

struct HapticParams {
    // Impact
    float impact_A;
    float impact_rise_ms;
    float impact_fall_ms;
    float impact_hf_weight;
    
    // Ring
    float ring_f_Hz[3];
    float ring_tau_ms[3];
    float ring_a[3];
    
    // Shear
    float shear_A;
    float shear_band_Hz;
    
    // Weight
    float weight_A;
    float weight_rate_ms;
    
    // Texture
    float texture_A;
    int texture_color_idx; // 0, 1, or 2
};

class HaptosInference {
private:
    // Buffers for intermediate layers
    // Max width in network is 128
    float buf_128_A[128];
    float buf_128_B[128];
    float buf_64[64];
    float buf_32[32];
    float buf_out[9]; // Max output dim is 9 (Ring)

public:
    HaptosInference() {}
    
    HapticParams predict(float* input_features) {
        // Input Features: 13 dims
        
        // ---------------------------------------------------------
        // 1. SHARED TRUNK
        // ---------------------------------------------------------
        
        // Layer 1: 13 -> 128 (ReLU)
        dense_layer(input_features, 13, trunk_0_weight, trunk_0_bias, buf_128_A, 128, true);
        
        // Layer 2: 128 -> 128 (ReLU)
        dense_layer(buf_128_A, 128, trunk_3_weight, trunk_3_bias, buf_128_B, 128, true);
        
        // Layer 3: 128 -> 64 (ReLU) -> "Features"
        dense_layer(buf_128_B, 128, trunk_6_weight, trunk_6_bias, buf_64, 64, true);
        
        // ---------------------------------------------------------
        // 2. HEADS
        // ---------------------------------------------------------
        HapticParams result;
        
        // --- IMPACT HEAD ---
        // 64 -> 64 (ReLU)
        dense_layer(buf_64, 64, impact_head_net_0_weight, impact_head_net_0_bias, buf_128_A, 64, true);
        // 64 -> 32 (ReLU)
        dense_layer(buf_128_A, 64, impact_head_net_3_weight, impact_head_net_3_bias, buf_32, 32, true);
        // 32 -> 4 (Linear)
        dense_layer(buf_32, 32, impact_head_net_6_weight, impact_head_net_6_bias, buf_out, 4, false);
        
        // Post-process Impact
        result.impact_A = sigmoid(buf_out[0]);
        result.impact_rise_ms = relu(buf_out[1]) + 0.5f;
        result.impact_fall_ms = relu(buf_out[2]) + 2.0f;
        result.impact_hf_weight = sigmoid(buf_out[3]);
        
        // --- RING HEAD ---
        // 64 -> 64 (ReLU)
        dense_layer(buf_64, 64, ring_head_net_0_weight, ring_head_net_0_bias, buf_128_A, 64, true);
        // 64 -> 32 (ReLU)
        dense_layer(buf_128_A, 64, ring_head_net_3_weight, ring_head_net_3_bias, buf_32, 32, true);
        // 32 -> 9 (Linear)
        dense_layer(buf_32, 32, ring_head_net_6_weight, ring_head_net_6_bias, buf_out, 9, false);
        
        // Post-process Ring
        for(int i=0; i<3; i++) {
            result.ring_f_Hz[i] = relu(buf_out[i]) + 100.0f;
            result.ring_tau_ms[i] = relu(buf_out[i+3]) + 20.0f;
            result.ring_a[i] = sigmoid(buf_out[i+6]);
        }
        
        // --- SHEAR HEAD ---
        // 64 -> 64 (ReLU)
        dense_layer(buf_64, 64, shear_head_net_0_weight, shear_head_net_0_bias, buf_128_A, 64, true);
        // 64 -> 32 (ReLU)
        dense_layer(buf_128_A, 64, shear_head_net_3_weight, shear_head_net_3_bias, buf_32, 32, true);
        // 32 -> 2 (Linear)
        dense_layer(buf_32, 32, shear_head_net_6_weight, shear_head_net_6_bias, buf_out, 2, false);
        
        result.shear_A = sigmoid(buf_out[0]);
        result.shear_band_Hz = relu(buf_out[1]) + 30.0f;
        
        // --- WEIGHT HEAD ---
        // 64 -> 64 (ReLU)
        dense_layer(buf_64, 64, weight_head_net_0_weight, weight_head_net_0_bias, buf_128_A, 64, true);
        // 64 -> 32 (ReLU)
        dense_layer(buf_128_A, 64, weight_head_net_3_weight, weight_head_net_3_bias, buf_32, 32, true);
        // 32 -> 2 (Linear)
        dense_layer(buf_32, 32, weight_head_net_6_weight, weight_head_net_6_bias, buf_out, 2, false);
        
        result.weight_A = sigmoid(buf_out[0]);
        result.weight_rate_ms = relu(buf_out[1]) + 10.0f;
        
        // --- TEXTURE HEAD ---
        // 64 -> 64 (ReLU)
        dense_layer(buf_64, 64, texture_head_net_0_weight, texture_head_net_0_bias, buf_128_A, 64, true);
        // 64 -> 32 (ReLU)
        dense_layer(buf_128_A, 64, texture_head_net_3_weight, texture_head_net_3_bias, buf_32, 32, true);
        // 32 -> 4 (Linear)
        dense_layer(buf_32, 32, texture_head_net_6_weight, texture_head_net_6_bias, buf_out, 4, false);
        
        result.texture_A = sigmoid(buf_out[0]);
        
        // Softmax for color (indices 1,2,3)
        float max_val = buf_out[1];
        if(buf_out[2] > max_val) max_val = buf_out[2];
        if(buf_out[3] > max_val) max_val = buf_out[3];
        
        float exp1 = expf(buf_out[1] - max_val);
        float exp2 = expf(buf_out[2] - max_val);
        float exp3 = expf(buf_out[3] - max_val);
        float sum_exp = exp1 + exp2 + exp3;
        
        float p1 = exp1 / sum_exp;
        float p2 = exp2 / sum_exp;
        float p3 = exp3 / sum_exp;
        
        // Argmax
        if(p1 >= p2 && p1 >= p3) result.texture_color_idx = 0;
        else if(p2 >= p3) result.texture_color_idx = 1;
        else result.texture_color_idx = 2;
        
        return result;
    }
};

#endif // INFERENCE_CORE_H
