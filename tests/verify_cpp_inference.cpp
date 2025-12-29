#include "../tests/mock_arduino.h"
#include "../firmware/inference_core.h"
#include <stdio.h>

int main() {
    printf("Running Twin Check (C++ Side)...\n");
    
    HaptosInference brain;
    
    // Create deterministic input (13 dims)
    // [0.1, 0.2, ..., 1.3]
    float test_input[13];
    for(int i=0; i<13; i++) {
        test_input[i] = 0.1f * (i+1);
    }
    
    HapticParams result = brain.predict(test_input);
    
    printf("CPP_IMPACT_A: %.6f\n", result.impact_A);
    printf("CPP_RING_F0: %.6f\n", result.ring_f_Hz[0]);
    printf("CPP_SHEAR_A: %.6f\n", result.shear_A);
    
    return 0;
}
