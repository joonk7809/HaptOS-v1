#ifndef ARDUINO_H
#define ARDUINO_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// Mock expf if needed (usually in math.h)
// Mock Serial
class MockSerial {
public:
    void print(const char* s) { printf("%s", s); }
    void print(float f) { printf("%f", f); }
    void println(const char* s) { printf("%s\n", s); }
    void println(float f, int d) { printf("%.*f\n", d, f); }
};

static MockSerial Serial;

#endif
