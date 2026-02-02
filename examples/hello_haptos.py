#!/usr/bin/env python3
"""
Hello HAPTOS - Minimal Example

The simplest possible HAPTOS program.
Runs a 1-second simulation with mock hardware.
"""

import haptos

# Run built-in demo
if __name__ == "__main__":
    print("Hello HAPTOS!")
    print()
    haptos.demo(duration=1.0)
