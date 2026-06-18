#ifndef MOCK_WIRE_H
#define MOCK_WIRE_H

// Minimal stand-in for the Arduino TwoWire I2C bus class.
class TwoWire {
};

// The real Wire.h declares a global `Wire` instance; mirror that here.
// The single definition lives in the test driver (test_main.cpp).
extern TwoWire Wire;

#endif // MOCK_WIRE_H
