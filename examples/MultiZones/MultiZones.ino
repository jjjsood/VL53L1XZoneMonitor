#include <VL53L1XZoneMonitor.h>
#include <Wire.h>

// Create a VL53L1X monitor with a 50 ms update interval, using custom I²C pins
VL53L1XZoneMonitor monitor(&Wire,50);

// Callback for Zone 1 on enter
void zone1EnterCallback(uint16_t distance) {
    Serial.print("Zone 1: Object detected at ");
    Serial.print(distance);
    Serial.println(" mm");
}

// Callback for Zone 1 on exit
void zone1ExitCallback() {
    Serial.println("Zone 1: Object exited.");
}

// Callback for Zone 2 on enter
void zone2EnterCallback(uint16_t distance) {
    Serial.print("Zone 2: Object detected at ");
    Serial.print(distance);
    Serial.println(" mm");
}

// Callback for Zone 2 on exit
void zone2ExitCallback() {
    Serial.println("Zone 2: Object exited.");
}

// Callback for Zone 3 on enter
void zone3EnterCallback(uint16_t distance) {
    Serial.print("Zone 3: Object detected at ");
    Serial.print(distance);
    Serial.println(" mm");
}

// Callback for Zone 3 on exit
void zone3ExitCallback() {
    Serial.println("Zone 3: Object exited.");
}

void setup() {
    Serial.begin(115200);

    // Initialize the custom I²C bus with specific pins
    Wire.begin(21, 22); // Example: SDA = 21, SCL = 22 for ESP32

    // Initialize the monitor and sensor
    if (!monitor.init()) {
        Serial.println("Sensor initialization failed!");
        while (1);
    }

    // Configure sensor settings
    monitor.setDistanceMode(VL53L1X::Long);
    monitor.setMeasurementTimingBudget(50000); // 50 ms timing budget

    // Add zones with specific distance ranges and callbacks
    monitor.addZone(100, 200, zone1EnterCallback, zone1ExitCallback); // Zone 1: 100 mm to 200 mm
    monitor.addZone(300, 500, zone2EnterCallback, zone2ExitCallback); // Zone 2: 300 mm to 500 mm
    monitor.addZone(100, 500, zone3EnterCallback, zone3ExitCallback); // Zone 3: 100 mm to 500 mm
}

void loop() {
    // Update the monitor to process sensor readings
    monitor.update();
}
