// VL53L1X Monitor Library - Implementation File
// GPL License
// This file contains the implementation of the VL53L1XZoneMonitor and ZoneObserver classes.

#include "VL53L1XZoneMonitor.h"

ZoneObserver::ZoneObserver(uint16_t min, uint16_t max, std::function<void(uint16_t)> onEnter, std::function<void()> onExit)
    : min_distance(min), max_distance(max), object_present(false), on_enter(std::move(onEnter)), on_exit(std::move(onExit)),
      in_zone_count(0), out_zone_count(0)
{
    if (min_distance > max_distance)
    {
        uint16_t tmp = min_distance;
        min_distance = max_distance;
        max_distance = tmp;
    }
}

void ZoneObserver::evaluate(uint16_t distance, size_t certainty)
{
    bool in_zone = (distance >= min_distance && distance <= max_distance);
    if (in_zone)
    {
        if (in_zone_count < certainty)
            in_zone_count++;
        out_zone_count = 0;
        if (in_zone_count >= certainty && !object_present)
        {
            object_present = true;
            if (on_enter)
                on_enter(distance);
        }
    }
    else
    {
        if (out_zone_count < certainty)
            out_zone_count++;
        in_zone_count = 0;
        if (out_zone_count >= certainty && object_present)
        {
            object_present = false;
            if (on_exit)
                on_exit();
        }
    }
}

bool ZoneObserver::isObjectPresent() const
{
    return object_present;
}

VL53L1XZoneMonitor::VL53L1XZoneMonitor(TwoWire *wire, uint32_t interval_ms, size_t certainty)
    : update_interval_ms(interval_ms), last_update_time(0), last_distance(0), certainty_factor(certainty < 1 ? 1 : certainty)
{
    if (wire)
    {
        sensor.setBus(wire);
    }
}

bool VL53L1XZoneMonitor::init()
{
    if (!sensor.init())
        return false;
    sensor.startContinuous(update_interval_ms);
    return true;
}

void VL53L1XZoneMonitor::setDistanceMode(VL53L1X::DistanceMode mode)
{
    sensor.setDistanceMode(mode);
}

VL53L1X::DistanceMode VL53L1XZoneMonitor::getDistanceMode()
{
    return sensor.getDistanceMode();
}

void VL53L1XZoneMonitor::setMeasurementTimingBudget(uint32_t budget_us)
{
    sensor.setMeasurementTimingBudget(budget_us);
}

uint32_t VL53L1XZoneMonitor::getMeasurementTimingBudget()
{
    return sensor.getMeasurementTimingBudget();
}

void VL53L1XZoneMonitor::setTimeout(uint16_t timeout)
{
    sensor.setTimeout(timeout);
}

uint16_t VL53L1XZoneMonitor::getTimeout()
{
    return sensor.getTimeout();
}

void VL53L1XZoneMonitor::addZone(uint16_t min, uint16_t max, std::function<void(uint16_t)> onEnter, std::function<void()> onExit)
{
    zones.emplace_back(min, max, std::move(onEnter), std::move(onExit));
}

bool VL53L1XZoneMonitor::isObjectInZone(size_t zone_index)
{
    if (zone_index < zones.size())
    {
        return zones[zone_index].isObjectPresent();
    }
    return false;
}

size_t VL53L1XZoneMonitor::getZoneCount() const
{
    return zones.size();
}

ZoneObserver *VL53L1XZoneMonitor::getZone(size_t zone_index)
{
    if (zone_index < zones.size())
    {
        return &zones[zone_index];
    }
    return nullptr;
}

void VL53L1XZoneMonitor::updateZone(size_t zone_index, uint16_t min_distance, uint16_t max_distance)
{
    if (zone_index >= zones.size())
        return;
    uint16_t new_min = zones[zone_index].min_distance;
    uint16_t new_max = zones[zone_index].max_distance;
    if (min_distance != 0)
        new_min = min_distance;
    if (max_distance != 0)
        new_max = max_distance;
    if (new_min > new_max)
        return;
    zones[zone_index].min_distance = new_min;
    zones[zone_index].max_distance = new_max;
}

void VL53L1XZoneMonitor::deleteZone(size_t zone_index)
{
    if (zone_index < zones.size())
    {
        zones.erase(zones.begin() + zone_index);
    }
}

uint16_t VL53L1XZoneMonitor::getDistance()
{
    return last_distance;
}

void VL53L1XZoneMonitor::setCertaintyFactor(size_t certainty)
{
    certainty_factor = (certainty < 1) ? 1 : certainty;
}

size_t VL53L1XZoneMonitor::getCertaintyFactor() const
{
    return certainty_factor;
}

void VL53L1XZoneMonitor::performUpdate()
{
    if (millis() - last_update_time < update_interval_ms)
        return;
    if (!sensor.dataReady())
        return;
    last_update_time = millis();
    uint16_t distance = sensor.read(false);
    if (sensor.timeoutOccurred())
        return;
    last_distance = distance;
    for (auto &zone : zones)
    {
        zone.evaluate(distance, certainty_factor);
    }
}


void VL53L1XZoneMonitor::update()
{
    performUpdate();
}
