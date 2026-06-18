# VL53L1XZoneMonitor

## Overview
**VL53L1XZoneMonitor** is an Arduino library designed to simplify the use of the VL53L1X time-of-flight sensor. The library provides an intuitive interface for monitoring specific distance zones and triggering actions when objects are detected within those zones. It supports continuous measurement, configurable zones, and callback-based or query-based detection. The library also includes a certainty factor to stabilize measurements before triggering events.

This library is authored by **Johannes Sood**.

## Features
- Continuous distance measurement.
- Zone-based monitoring with independent configuration for each zone.
- Separate callbacks for object entry and exit events.
- Certainty factor for stable detection (number of consecutive measurements).
- Query-based methods for checking zone status.
- Full compatibility with the VL53L1X library by Pololu.
- Easy zone management with add, delete, and query capabilities.
- Adjustable measurement timing budget for precision control.
- Configurable distance modes (Short, Medium, Long).

## Dependencies
This library depends on the [VL53L1X library](https://github.com/pololu/vl53l1x-arduino) for low-level sensor communication.

### Installing Dependencies
1. Open the Arduino IDE.
2. Go to `Tools > Manage Libraries...`.
3. Search for "VL53L1X".
4. Install the library provided by Pololu.

Alternatively, download the library directly from the [VL53L1X GitHub repository](https://github.com/pololu/vl53l1x-arduino) and place it in your Arduino `libraries` folder.

## Installation
1. Download or clone this repository: [VL53L1XZoneMonitor](https://github.com/JohnnyJagatpal/VL53L1XZoneMonitor).
2. Copy the `VL53L1XZoneMonitor` folder into your Arduino `libraries` directory.
3. Restart the Arduino IDE.

## Usage

### Example
Below is an example sketch demonstrating basic usage of the library:

```cpp
#include <VL53L1XZoneMonitor.h>
#include <Wire.h>

// Create a VL53L1X monitor with a 50 ms update interval
VL53L1XZoneMonitor monitor( &Wire,50);

// Callback for Zone 1 on enter
void zoneEnterCallback(uint16_t distance) {
    Serial.print("Zone: Object detected at ");
    Serial.print(distance);
    Serial.println(" mm");
}

// Callback for Zone 1 on exit
void zoneExitCallback() {
    Serial.println("Zone: Object exited.");
}

void setup() {
    Serial.begin(115200);

    // Initialize the I²C bus
    Wire.begin(21, 22); // Example pins for ESP32

    // Initialize the monitor and sensor
    if (!monitor.init()) {
        Serial.println("Sensor initialization failed!");
        while (1);
    }

    // Configure sensor settings
    monitor.setDistanceMode(VL53L1X::Long);
    monitor.setMeasurementTimingBudget(50000); // 50 ms timing budget

    // Add a zone with specific distance range and callbacks
    monitor.addZone(100, 200, zoneEnterCallback, zoneExitCallback); // Zone: 100 mm to 200 mm
}

void loop() {
    monitor.update(); // Process sensor readings and trigger zone callbacks
}
```

### API
#### Initialization
- `bool init()`
  Initializes the VL53L1X sensor and starts continuous measurements. Returns `true` if successful.

#### Configuration
- `void setDistanceMode(VL53L1X::DistanceMode mode)`
  Sets the distance mode (Short, Medium, or Long).
- `VL53L1X::DistanceMode getDistanceMode()`
  Retrieves the current distance mode.
- `void setMeasurementTimingBudget(uint32_t budget_us)`
  Configures the measurement timing budget in microseconds.
- `uint32_t getMeasurementTimingBudget()`
  Gets the current measurement timing budget.
- `void setTimeout(uint16_t timeout)`
  Sets the timeout for sensor operations in milliseconds.
- `uint16_t getTimeout()`
  Retrieves the current timeout duration.
- `void setCertaintyFactor(size_t certainty)`
  Sets the certainty factor for stable detection.
- `size_t getCertaintyFactor()`
  Retrieves the current certainty factor.

#### Zone Management
- `void addZone(uint16_t min, uint16_t max, void (*onEnter)(uint16_t) = nullptr, void (*onExit)() = nullptr)`
  Adds a new zone with a specified minimum and maximum distance and optional callbacks for entry and exit.
- `bool isObjectInZone(size_t zone_index)`
  Checks if an object is detected in the specified zone. Returns `true` if detected.
- `size_t getZoneCount()`
  Returns the total number of zones being monitored.
- `ZoneObserver* getZone(size_t zone_index)`
  Retrieves a pointer to the specified zone. Returns `nullptr` if the index is invalid.
- `void deleteZone(size_t zone_index)`
  Deletes a zone by its index.

#### Processing
- `void update()`
  Updates sensor readings and evaluates all zones. Should be called periodically in the `loop()` function.

## License
This library is released under the GPL License. See the `LICENSE` file for details.

## Author
Johannes Sood

GitHub: [JJJSood](https://github.com/jjjsood/VL53L1XZoneMonitor)

For any questions or issues, feel free to open an issue in the repository.
