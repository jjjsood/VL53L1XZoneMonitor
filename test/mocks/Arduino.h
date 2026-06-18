#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

// Use the C headers so the fixed-width integer types and size_t land in the
// GLOBAL namespace, matching real Arduino.h behavior (the library uses bare
// uint16_t / uint32_t / size_t without a std:: qualifier).
#include <stdint.h>
#include <stddef.h>
#include <cstdint>

// Mock Arduino runtime for host-compiled tests.
// Provides a controllable millis() backed by a global counter.

namespace mock_arduino_detail {
inline uint32_t &millis_counter()
{
    static uint32_t value = 0;
    return value;
}
}

inline uint32_t millis()
{
    return mock_arduino_detail::millis_counter();
}

// Test helpers to control simulated time.
inline void mock_setMillis(uint32_t value)
{
    mock_arduino_detail::millis_counter() = value;
}

inline void mock_advanceMillis(uint32_t delta)
{
    mock_arduino_detail::millis_counter() += delta;
}

#endif // MOCK_ARDUINO_H
