#ifndef MOCK_VL53L1X_H
#define MOCK_VL53L1X_H

// Matches the real Pololu VL53L1X transitive includes: VL53L1X.h pulls in
// Arduino.h and Wire.h, so millis() and TwoWire resolve through this header.
#include "Arduino.h"
#include "Wire.h"

#include <cstdint>

// Controllable fake VL53L1X sensor for host-compiled tests.
//
// The library owns its VL53L1X by value as a private member, so the test
// driver cannot reach a particular instance's knobs. To let tests drive the
// monitor's internal sensor, the behavior knobs are STATIC (shared by all
// instances): the driver sets them on any VL53L1X (or via the static members)
// and the monitor's hidden sensor observes the same values.
//
// Knobs:
//   mock_dataReady  - value returned by dataReady()
//   mock_readValue  - value returned by read()
//   mock_timeout    - value returned by timeoutOccurred()
// Echoing storage lets getters return what the library last wrote.
class VL53L1X {
public:
    enum DistanceMode { Short, Medium, Long, Unknown };

    // --- Shared test knobs (apply to every VL53L1X instance) ---
    static bool mock_dataReady;
    static uint16_t mock_readValue;
    static bool mock_timeout;

    // Shared call tracking.
    static bool mock_initCalled;
    static bool mock_initReturn;
    static bool mock_startContinuousCalled;
    static uint32_t mock_startContinuousPeriod;
    static TwoWire *mock_bus;
    static int mock_readCallCount;
    static bool mock_lastReadBlocking;

    // Shared echoing storage for getters.
    static DistanceMode mock_distanceMode;
    static uint32_t mock_timingBudget;
    static uint16_t mock_timeoutValue;

    // Reset all shared state to defaults between test cases.
    static void mock_reset()
    {
        mock_dataReady = true;
        mock_readValue = 0;
        mock_timeout = false;
        mock_initCalled = false;
        mock_initReturn = true;
        mock_startContinuousCalled = false;
        mock_startContinuousPeriod = 0;
        mock_bus = nullptr;
        mock_readCallCount = 0;
        mock_lastReadBlocking = true;
        mock_distanceMode = Long;
        mock_timingBudget = 0;
        mock_timeoutValue = 0;
    }

    // --- API used by the library ---
    bool init(bool io_2v8 = true)
    {
        (void)io_2v8;
        mock_initCalled = true;
        return mock_initReturn;
    }

    void setBus(TwoWire *bus)
    {
        mock_bus = bus;
    }

    void startContinuous(uint32_t period_ms)
    {
        mock_startContinuousCalled = true;
        mock_startContinuousPeriod = period_ms;
    }

    void setDistanceMode(DistanceMode mode)
    {
        mock_distanceMode = mode;
    }

    DistanceMode getDistanceMode()
    {
        return mock_distanceMode;
    }

    void setMeasurementTimingBudget(uint32_t budget_us)
    {
        mock_timingBudget = budget_us;
    }

    uint32_t getMeasurementTimingBudget()
    {
        return mock_timingBudget;
    }

    void setTimeout(uint16_t timeout)
    {
        mock_timeoutValue = timeout;
    }

    uint16_t getTimeout()
    {
        return mock_timeoutValue;
    }

    bool dataReady()
    {
        return mock_dataReady;
    }

    uint16_t read(bool blocking = true)
    {
        mock_readCallCount++;
        mock_lastReadBlocking = blocking;
        return mock_readValue;
    }

    bool timeoutOccurred()
    {
        return mock_timeout;
    }
};

#endif // MOCK_VL53L1X_H
