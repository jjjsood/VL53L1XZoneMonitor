#ifndef VL53L1XZONEMONITOR_H
#define VL53L1XZONEMONITOR_H

#include <VL53L1X.h>
#include <deque>
#include <functional>

/**
 * @brief Represents a single monitoring zone for the VL53L1X sensor.
 *
 * Each zone is defined by a minimum and maximum distance. The ZoneObserver
 * class tracks whether an object is currently in the zone based on measurements
 * and triggers callbacks for entering and exiting events when the configured
 * certainty threshold is met.
 */
class ZoneObserver {
public:
    uint16_t min_distance;                          /**< Minimum distance for the zone in millimeters. */
    uint16_t max_distance;                          /**< Maximum distance for the zone in millimeters. */
    bool object_present;                            /**< Flag indicating whether an object is currently in the zone. */
    std::function<void(uint16_t distance)> on_enter; /**< Callback function triggered when an object enters the zone. */
    std::function<void()> on_exit;                 /**< Callback function triggered when an object exits the zone. */

    size_t in_zone_count;  /**< Counter for consecutive in-zone measurements. */
    size_t out_zone_count; /**< Counter for consecutive out-of-zone measurements. */

    /**
     * @brief Constructs a ZoneObserver object.
     *
     * @param min Minimum distance of the zone in millimeters.
     * @param max Maximum distance of the zone in millimeters.
     * @param onEnter Callback function to execute when an object enters the zone.
     * @param onExit Callback function to execute when an object exits the zone.
     */
    ZoneObserver(uint16_t min, uint16_t max, std::function<void(uint16_t)> onEnter = nullptr, std::function<void()> onExit = nullptr);

    /**
     * @brief Evaluates the current distance measurement against the zone boundaries.
     *
     * This method checks if the distance is within the zone and updates the
     * counters for consecutive measurements. When the certainty factor is met,
     * the appropriate callback (on_enter or on_exit) is triggered.
     *
     * @param distance Distance measured by the sensor in millimeters.
     * @param certainty Number of consecutive measurements required for stability.
     */
    void evaluate(uint16_t distance, size_t certainty);

    /**
     * @brief Checks if an object is present in the zone.
     *
     * @return True if an object is present, false otherwise.
     */
    bool isObjectPresent() const;
};

/**
 * @brief High-level interface for monitoring multiple zones with a VL53L1X sensor.
 *
 * The VL53L1XZoneMonitor class manages multiple zones and provides configuration
 * options for distance mode, timing budget, and timeout. It also implements a
 * certainty factor to ensure stable detection before triggering callbacks.
 */
class VL53L1XZoneMonitor {
private:
    VL53L1X sensor;                  /**< Instance of the VL53L1X sensor. */
    std::deque<ZoneObserver> zones; /**< List of defined zones. */
    uint32_t update_interval_ms;     /**< Interval for continuous measurements in milliseconds. */
    uint32_t last_update_time;       /**< Timestamp of the last measurement update. */
    uint16_t last_distance;          /**< Last recorded distance measurement in millimeters. */
    size_t certainty_factor;         /**< Number of consecutive measurements required for stability. */

    /**
     * @brief Performs a measurement update and evaluates all zones.
     *
     * This method reads the sensor data and updates the state of each zone,
     * triggering callbacks if the certainty factor is met. Updates the
     * last recorded distance measurement.
     */
    void performUpdate();

public:
    /**
     * @brief Constructs a VL53L1XZoneMonitor object.
     *
     * @param wire Optional pointer to a TwoWire object for custom I²C bus.
     * @param interval_ms Interval for continuous measurements in milliseconds.
     * @param certainty Number of consecutive measurements required for stability.
     */
    VL53L1XZoneMonitor(TwoWire *wire = nullptr, uint32_t interval_ms = 50, size_t certainty = 1);

    /**
     * @brief Initializes the VL53L1X sensor.
     *
     * @return True if initialization is successful, false otherwise.
     */
    bool init();

    /**
     * @brief Sets the distance mode of the sensor.
     *
     * The distance mode determines the trade-off between range and ambient
     * light resistance. Available modes are Short, Medium, and Long.
     *
     * @param mode The desired distance mode.
     */
    void setDistanceMode(VL53L1X::DistanceMode mode);

    /**
     * @brief Gets the current distance mode of the sensor.
     *
     * @return The current distance mode.
     */
    VL53L1X::DistanceMode getDistanceMode();

    /**
     * @brief Sets the measurement timing budget.
     *
     * The timing budget defines the time the sensor spends on a single measurement,
     * affecting accuracy and measurement frequency.
     *
     * @param budget_us The timing budget in microseconds.
     */
    void setMeasurementTimingBudget(uint32_t budget_us);

    /**
     * @brief Gets the current measurement timing budget.
     *
     * @return The timing budget in microseconds.
     */
    uint32_t getMeasurementTimingBudget();

    /**
     * @brief Sets the timeout for sensor operations.
     *
     * @param timeout Timeout duration in milliseconds.
     */
    void setTimeout(uint16_t timeout);

    /**
     * @brief Gets the current timeout duration.
     *
     * @return Timeout duration in milliseconds.
     */
    uint16_t getTimeout();

    /**
     * @brief Adds a new monitoring zone.
     *
     * Each zone is defined by a minimum and maximum distance, with optional
     * callbacks for entering and exiting events.
     *
     * @param min Minimum distance for the zone in millimeters.
     * @param max Maximum distance for the zone in millimeters.
     * @param onEnter Callback function to execute when an object enters the zone.
     * @param onExit Callback function to execute when an object exits the zone.
     */
    void addZone(uint16_t min, uint16_t max, std::function<void(uint16_t)> onEnter = nullptr, std::function<void()> onExit = nullptr);

    /**
     * @brief Updates an existing monitoring zone.
     *
     * Allows modification of the minimum and maximum distance for an existing zone.
     * If a parameter is not provided, the existing value remains unchanged.
     *
     * @param zone_index The index of the zone to update.
     * @param min_distance Optional new minimum distance in millimeters.
     * @param max_distance Optional new maximum distance in millimeters.
     */
    void updateZone(size_t zone_index, uint16_t min_distance = 0, uint16_t max_distance = 0);

    /**
     * @brief Checks if an object is present in a specific zone.
     *
     * Returns the presence state as of the last update() call. This method does
     * not itself trigger a new measurement.
     *
     * @param zone_index The index of the zone to check.
     * @return True if an object is present, false otherwise.
     */
    bool isObjectInZone(size_t zone_index);

    /**
     * @brief Gets the total number of defined zones.
     *
     * @return The number of zones.
     */
    size_t getZoneCount() const;

    /**
     * @brief Gets a pointer to a specific zone.
     *
     * @param zone_index The index of the zone to retrieve.
     * @return Pointer to the ZoneObserver object, or nullptr if the index is invalid.
     *
     * Note: the returned pointer remains valid across addZone() calls, but
     * deleteZone() invalidates pointers to the deleted zone and any zones after it.
     */
    ZoneObserver *getZone(size_t zone_index);

    /**
     * @brief Deletes a specific zone.
     *
     * @param zone_index The index of the zone to delete.
     */
    void deleteZone(size_t zone_index);

    /**
     * @brief Gets the last recorded distance measured by the sensor.
     *
     * @return The last measured distance in millimeters, or 0 if no valid data is available.
     */
    uint16_t getDistance();

    /**
     * @brief Sets the certainty factor for measurements.
     *
     * The certainty factor defines the number of consecutive measurements required
     * to confirm an object's presence or absence.
     *
     * @param certainty The number of consecutive measurements required.
     */
    void setCertaintyFactor(size_t certainty);

    /**
     * @brief Gets the current certainty factor for measurements.
     *
     * @return The number of consecutive measurements required.
     */
    size_t getCertaintyFactor() const;

    /**
     * @brief Updates the sensor measurements and evaluates all zones.
     */
    void update();
};

#endif
