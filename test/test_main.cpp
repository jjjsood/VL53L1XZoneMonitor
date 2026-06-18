// Host-compiled test driver for VL53L1XZoneMonitor.
// No Arduino hardware, no external test framework.
//
// Builds the real library .cpp against the mock Arduino/Wire/VL53L1X headers
// in test/mocks and exercises the actual current behavior of the source.

#include <cstdio>
#include <cstdint>

#include "Arduino.h"
#include "Wire.h"
#include "VL53L1X.h"
#include "VL53L1XZoneMonitor.h"

// Single definition of the global Wire instance declared in Wire.h.
TwoWire Wire;

// Definitions for the mock's shared (static) state.
bool VL53L1X::mock_dataReady = true;
uint16_t VL53L1X::mock_readValue = 0;
bool VL53L1X::mock_timeout = false;
bool VL53L1X::mock_initCalled = false;
bool VL53L1X::mock_initReturn = true;
bool VL53L1X::mock_startContinuousCalled = false;
uint32_t VL53L1X::mock_startContinuousPeriod = 0;
TwoWire *VL53L1X::mock_bus = nullptr;
int VL53L1X::mock_readCallCount = 0;
bool VL53L1X::mock_lastReadBlocking = true;
VL53L1X::DistanceMode VL53L1X::mock_distanceMode = VL53L1X::Long;
uint32_t VL53L1X::mock_timingBudget = 0;
uint16_t VL53L1X::mock_timeoutValue = 0;

// --- Tiny assertion framework ---
static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        if (cond) {                                                            \
            ++g_pass;                                                          \
        } else {                                                               \
            ++g_fail;                                                          \
            std::printf("FAIL: %s  (%s:%d)\n", (msg), __FILE__, __LINE__);     \
        }                                                                      \
    } while (0)

// Reset all global state (time + sensor knobs) before each test.
static void resetEnv()
{
    mock_setMillis(0);
    VL53L1X::mock_reset();
}

// ---------------------------------------------------------------------------
// ZoneObserver::evaluate tests
// ---------------------------------------------------------------------------
static void test_zoneobserver_enter_requires_certainty()
{
    resetEnv();
    int enter_count = 0;
    uint16_t enter_distance = 0;
    ZoneObserver z(100, 200,
                   [&](uint16_t d) { ++enter_count; enter_distance = d; },
                   nullptr);
    const size_t certainty = 3;

    z.evaluate(150, certainty);
    CHECK(enter_count == 0, "no enter after 1 in-zone read (certainty 3)");
    z.evaluate(150, certainty);
    CHECK(enter_count == 0, "no enter after 2 in-zone reads (certainty 3)");
    z.evaluate(150, certainty);
    CHECK(enter_count == 1, "enter fires on 3rd consecutive in-zone read");
    CHECK(enter_distance == 150, "on_enter receives the distance");
    CHECK(z.isObjectPresent(), "object present after enter");
}

static void test_zoneobserver_enter_only_once()
{
    resetEnv();
    int enter_count = 0;
    ZoneObserver z(100, 200, [&](uint16_t) { ++enter_count; }, nullptr);
    const size_t certainty = 2;

    z.evaluate(150, certainty);
    z.evaluate(150, certainty); // enters here
    CHECK(enter_count == 1, "enter fired once");
    z.evaluate(150, certainty);
    z.evaluate(150, certainty);
    z.evaluate(150, certainty);
    CHECK(enter_count == 1, "enter not re-fired on further in-zone reads");
}

static void test_zoneobserver_exit_requires_certainty()
{
    resetEnv();
    int exit_count = 0;
    ZoneObserver z(100, 200, nullptr, [&]() { ++exit_count; });
    const size_t certainty = 3;

    // Enter the zone first.
    z.evaluate(150, certainty);
    z.evaluate(150, certainty);
    z.evaluate(150, certainty);
    CHECK(z.isObjectPresent(), "entered zone before exit test");

    z.evaluate(300, certainty);
    CHECK(exit_count == 0, "no exit after 1 out-of-zone read");
    z.evaluate(300, certainty);
    CHECK(exit_count == 0, "no exit after 2 out-of-zone reads");
    z.evaluate(300, certainty);
    CHECK(exit_count == 1, "exit fires on 3rd consecutive out-of-zone read");
    CHECK(!z.isObjectPresent(), "object not present after exit");
}

static void test_zoneobserver_single_out_resets_in_progress()
{
    resetEnv();
    int enter_count = 0;
    ZoneObserver z(100, 200, [&](uint16_t) { ++enter_count; }, nullptr);
    const size_t certainty = 3;

    z.evaluate(150, certainty);   // in_zone_count = 1
    z.evaluate(150, certainty);   // in_zone_count = 2
    z.evaluate(300, certainty);   // out: resets in_zone_count to 0
    CHECK(enter_count == 0, "out-of-zone read reset in-zone progress");
    z.evaluate(150, certainty);   // 1
    z.evaluate(150, certainty);   // 2
    CHECK(enter_count == 0, "still not entered after 2 (progress was reset)");
    z.evaluate(150, certainty);   // 3 -> enter
    CHECK(enter_count == 1, "enter only after 3 fresh consecutive reads");
}

static void test_zoneobserver_single_in_resets_out_progress()
{
    resetEnv();
    int exit_count = 0;
    ZoneObserver z(100, 200, nullptr, [&]() { ++exit_count; });
    const size_t certainty = 3;

    // Enter first.
    z.evaluate(150, certainty);
    z.evaluate(150, certainty);
    z.evaluate(150, certainty);

    z.evaluate(300, certainty);   // out_zone_count = 1
    z.evaluate(300, certainty);   // out_zone_count = 2
    z.evaluate(150, certainty);   // in: resets out_zone_count to 0
    CHECK(exit_count == 0, "in-zone read reset out-of-zone progress");
    z.evaluate(300, certainty);   // 1
    z.evaluate(300, certainty);   // 2
    CHECK(exit_count == 0, "still not exited after 2 (progress was reset)");
    z.evaluate(300, certainty);   // 3 -> exit
    CHECK(exit_count == 1, "exit only after 3 fresh consecutive out reads");
}

static void test_zoneobserver_boundaries_inclusive()
{
    resetEnv();
    int enter_count_min = 0;
    ZoneObserver zmin(100, 200, [&](uint16_t) { ++enter_count_min; }, nullptr);
    zmin.evaluate(100, 1);
    CHECK(enter_count_min == 1, "distance == min_distance is in-zone");

    int enter_count_max = 0;
    ZoneObserver zmax(100, 200, [&](uint16_t) { ++enter_count_max; }, nullptr);
    zmax.evaluate(200, 1);
    CHECK(enter_count_max == 1, "distance == max_distance is in-zone");

    // Just outside both edges should NOT enter.
    int enter_count_below = 0;
    ZoneObserver zb(100, 200, [&](uint16_t) { ++enter_count_below; }, nullptr);
    zb.evaluate(99, 1);
    CHECK(enter_count_below == 0, "distance below min is out-of-zone");

    int enter_count_above = 0;
    ZoneObserver za(100, 200, [&](uint16_t) { ++enter_count_above; }, nullptr);
    za.evaluate(201, 1);
    CHECK(enter_count_above == 0, "distance above max is out-of-zone");
}

static void test_zoneobserver_reversed_range_constructor()
{
    resetEnv();
    ZoneObserver z(200, 100, nullptr, nullptr);
    CHECK(z.min_distance == 100, "reversed range normalizes min_distance to 100");
    CHECK(z.max_distance == 200, "reversed range normalizes max_distance to 200");
}

static void test_zoneobserver_counters_capped()
{
    resetEnv();
    ZoneObserver z(100, 200, nullptr, nullptr);
    const size_t certainty = 3;

    for (int i = 0; i < 50; ++i)
        z.evaluate(150, certainty);
    CHECK(z.in_zone_count == certainty, "in_zone_count capped at certainty");
    CHECK(z.out_zone_count == 0, "out_zone_count zeroed while in zone");

    for (int i = 0; i < 50; ++i)
        z.evaluate(300, certainty);
    CHECK(z.out_zone_count == certainty, "out_zone_count capped at certainty");
    CHECK(z.in_zone_count == 0, "in_zone_count zeroed while out of zone");
}

// ---------------------------------------------------------------------------
// VL53L1XZoneMonitor tests
// ---------------------------------------------------------------------------
static void test_monitor_distance_zero_before_read()
{
    resetEnv();
    VL53L1XZoneMonitor monitor(&Wire);
    CHECK(monitor.init(), "init() returns true (mock)");
    CHECK(monitor.getDistance() == 0, "getDistance() is 0 before any successful read");
}

static void test_monitor_addzone_getzonecount_getzone()
{
    resetEnv();
    VL53L1XZoneMonitor monitor(&Wire);
    CHECK(monitor.getZoneCount() == 0, "zone count starts at 0");
    monitor.addZone(100, 200);
    monitor.addZone(300, 400);
    CHECK(monitor.getZoneCount() == 2, "zone count is 2 after two addZone");
    CHECK(monitor.getZone(0) != nullptr, "getZone(0) non-null in range");
    CHECK(monitor.getZone(1) != nullptr, "getZone(1) non-null in range");
    CHECK(monitor.getZone(2) == nullptr, "getZone(2) null out of range");
}

static void test_monitor_addzone_reversed_normalized()
{
    resetEnv();
    VL53L1XZoneMonitor monitor(&Wire);
    monitor.addZone(200, 100);
    ZoneObserver *z = monitor.getZone(0);
    CHECK(z != nullptr, "zone exists");
    CHECK(z->min_distance <= z->max_distance, "addZone normalizes min<=max");
    CHECK(z->min_distance == 100 && z->max_distance == 200,
          "addZone reversed range stored as (100,200)");
}

static void test_monitor_updatezone()
{
    resetEnv();
    VL53L1XZoneMonitor monitor(&Wire);
    monitor.addZone(100, 200);

    // 0 means "unchanged" for both params.
    monitor.updateZone(0, 0, 0);
    ZoneObserver *z = monitor.getZone(0);
    CHECK(z->min_distance == 100 && z->max_distance == 200,
          "updateZone(0,0,0) leaves zone unchanged");

    // Valid change applies.
    monitor.updateZone(0, 120, 180);
    CHECK(z->min_distance == 120 && z->max_distance == 180,
          "updateZone applies valid change");

    // A change yielding min>max is rejected (unchanged).
    monitor.updateZone(0, 500, 0); // would set min=500, max stays 180 -> min>max
    CHECK(z->min_distance == 120 && z->max_distance == 180,
          "updateZone rejects change that makes min>max");

    // Out-of-range index is a no-op (no crash).
    monitor.updateZone(99, 10, 20);
    CHECK(monitor.getZoneCount() == 1, "updateZone on bad index is a no-op");
}

static void test_monitor_certainty_clamp()
{
    resetEnv();
    VL53L1XZoneMonitor m1(&Wire);
    m1.setCertaintyFactor(0);
    CHECK(m1.getCertaintyFactor() >= 1, "setCertaintyFactor(0) clamps to >= 1");

    VL53L1XZoneMonitor m2(&Wire, 50, 0);
    CHECK(m2.getCertaintyFactor() >= 1, "constructor certainty 0 clamps to >= 1");

    VL53L1XZoneMonitor m3(&Wire, 50, 5);
    CHECK(m3.getCertaintyFactor() == 5, "constructor keeps certainty >= 1 value");
}

// Helper: drive enough successful updates to satisfy the certainty count.
// performUpdate gates on millis()-last_update_time >= update_interval_ms, so
// we advance time past the interval before each update() call.
static void driveUpdates(VL53L1XZoneMonitor &monitor, uint32_t interval_ms, int times)
{
    for (int i = 0; i < times; ++i)
    {
        mock_advanceMillis(interval_ms);
        monitor.update();
    }
}

static void test_monitor_update_enters_zone()
{
    resetEnv();
    const uint32_t interval = 50;
    const size_t certainty = 2;
    VL53L1XZoneMonitor monitor(&Wire, interval, certainty);
    CHECK(monitor.init(), "init() true");

    int enter_count = 0;
    monitor.addZone(100, 200, [&](uint16_t) { ++enter_count; }, nullptr);

    VL53L1X::mock_dataReady = true;
    VL53L1X::mock_timeout = false;
    VL53L1X::mock_readValue = 150; // inside the zone

    driveUpdates(monitor, interval, 1);
    CHECK(enter_count == 0, "not entered after 1 update (certainty 2)");
    CHECK(monitor.isObjectInZone(0) == false, "not present after 1 update");
    CHECK(monitor.getDistance() == 150, "getDistance reflects mock_readValue after read");

    driveUpdates(monitor, interval, 1);
    CHECK(enter_count == 1, "entered after 2 updates (certainty 2)");
    CHECK(monitor.isObjectInZone(0) == true, "object present in zone 0 after enter");
    CHECK(monitor.getDistance() == 150, "getDistance still reflects mock_readValue");
}

static void test_monitor_interval_gate()
{
    resetEnv();
    const uint32_t interval = 50;
    VL53L1XZoneMonitor monitor(&Wire, interval, 1);
    CHECK(monitor.init(), "init() true");
    monitor.addZone(100, 200);

    VL53L1X::mock_dataReady = true;
    VL53L1X::mock_readValue = 150;

    // millis() == 0, last_update_time == 0 -> 0 < interval, gate closed.
    monitor.update();
    CHECK(monitor.getDistance() == 0, "update before interval elapsed does nothing");
    CHECK(monitor.isObjectInZone(0) == false, "no evaluation before interval elapsed");

    // Advance past the interval -> gate opens.
    mock_advanceMillis(interval);
    monitor.update();
    CHECK(monitor.getDistance() == 150, "update after interval reads distance");
    CHECK(monitor.isObjectInZone(0) == true, "zone evaluated after interval (certainty 1)");
}

static void test_monitor_timeout_path()
{
    resetEnv();
    const uint32_t interval = 50;
    VL53L1XZoneMonitor monitor(&Wire, interval, 1);
    CHECK(monitor.init(), "init() true");

    int enter_count = 0;
    monitor.addZone(100, 200, [&](uint16_t) { ++enter_count; }, nullptr);

    VL53L1X::mock_dataReady = true;
    VL53L1X::mock_timeout = true;   // sensor reports timeout
    VL53L1X::mock_readValue = 150;  // would be in-zone if it counted

    driveUpdates(monitor, interval, 5);
    CHECK(monitor.getDistance() == 0, "timeout path leaves getDistance unchanged (0)");
    CHECK(enter_count == 0, "timeout path does not evaluate zones (no callback)");
    CHECK(monitor.isObjectInZone(0) == false, "timeout path keeps presence false");
}

static void test_monitor_dataready_false_path()
{
    resetEnv();
    const uint32_t interval = 50;
    VL53L1XZoneMonitor monitor(&Wire, interval, 1);
    CHECK(monitor.init(), "init() true");
    monitor.addZone(100, 200);

    VL53L1X::mock_dataReady = false; // sensor not ready
    VL53L1X::mock_readValue = 150;

    driveUpdates(monitor, interval, 5);
    CHECK(monitor.getDistance() == 0, "no read when dataReady() is false");
    CHECK(monitor.isObjectInZone(0) == false, "no evaluation when dataReady() is false");
}

static void test_monitor_isobjectinzone_is_pure_query()
{
    resetEnv();
    const uint32_t interval = 50;
    VL53L1XZoneMonitor monitor(&Wire, interval, 1);
    CHECK(monitor.init(), "init() true");
    monitor.addZone(100, 200);

    // Conditions are set so that a real update() WOULD enter the zone.
    VL53L1X::mock_dataReady = true;
    VL53L1X::mock_readValue = 150;
    mock_advanceMillis(interval); // interval gate would be open

    // But we never call update(); isObjectInZone must not trigger a measurement.
    for (int i = 0; i < 10; ++i)
        CHECK(monitor.isObjectInZone(0) == false,
              "isObjectInZone does not itself trigger a measurement");
    CHECK(monitor.getDistance() == 0, "isObjectInZone did not advance a measurement");
    CHECK(VL53L1X::mock_readCallCount == 0, "no sensor read happened from isObjectInZone");

    // Out-of-range index returns false safely.
    CHECK(monitor.isObjectInZone(99) == false, "isObjectInZone bad index returns false");
}

static void test_monitor_getzone_pointer_stability()
{
    resetEnv();
    VL53L1XZoneMonitor monitor(&Wire);
    monitor.addZone(100, 200);
    ZoneObserver *p = monitor.getZone(0);
    CHECK(p != nullptr, "got pointer to zone 0");
    uint16_t before_min = p->min_distance;
    uint16_t before_max = p->max_distance;

    // Adding more zones must not invalidate the pointer (deque guarantee).
    for (int i = 0; i < 20; ++i)
        monitor.addZone(300 + i, 400 + i);

    CHECK(monitor.getZone(0) == p, "getZone(0) returns same pointer after addZone");
    CHECK(p->min_distance == before_min && p->max_distance == before_max,
          "zone 0 data unchanged after addZone (pointer still valid)");
}

static void test_monitor_deletezone()
{
    resetEnv();
    VL53L1XZoneMonitor monitor(&Wire);
    monitor.addZone(10, 20);
    monitor.addZone(30, 40);
    monitor.addZone(50, 60);
    CHECK(monitor.getZoneCount() == 3, "3 zones before delete");

    monitor.deleteZone(1); // remove the (30,40) zone
    CHECK(monitor.getZoneCount() == 2, "deleteZone reduces count");
    CHECK(monitor.getZone(0)->min_distance == 10, "zone 0 still (10,..) after delete");
    CHECK(monitor.getZone(1)->min_distance == 50,
          "zone formerly at index 2 shifted to index 1 after delete");

    // Out-of-range delete is a no-op.
    monitor.deleteZone(99);
    CHECK(monitor.getZoneCount() == 2, "deleteZone bad index is a no-op");
}

static void test_monitor_setters_getters_echo()
{
    resetEnv();
    VL53L1XZoneMonitor monitor(&Wire);
    monitor.setDistanceMode(VL53L1X::Medium);
    CHECK(monitor.getDistanceMode() == VL53L1X::Medium, "distance mode getter echoes setter");
    monitor.setMeasurementTimingBudget(33000);
    CHECK(monitor.getMeasurementTimingBudget() == 33000, "timing budget getter echoes setter");
    monitor.setTimeout(500);
    CHECK(monitor.getTimeout() == 500, "timeout getter echoes setter");
}

static void test_monitor_init_wires_bus_and_continuous()
{
    resetEnv();
    const uint32_t interval = 50;
    VL53L1XZoneMonitor monitor(&Wire, interval, 1);
    CHECK(VL53L1X::mock_bus == &Wire, "constructor with &Wire calls setBus(&Wire)");
    CHECK(monitor.init(), "init returns true");
    CHECK(VL53L1X::mock_startContinuousCalled, "init calls startContinuous");
    CHECK(VL53L1X::mock_startContinuousPeriod == interval,
          "startContinuous uses update interval");
}

int main()
{
    test_zoneobserver_enter_requires_certainty();
    test_zoneobserver_enter_only_once();
    test_zoneobserver_exit_requires_certainty();
    test_zoneobserver_single_out_resets_in_progress();
    test_zoneobserver_single_in_resets_out_progress();
    test_zoneobserver_boundaries_inclusive();
    test_zoneobserver_reversed_range_constructor();
    test_zoneobserver_counters_capped();

    test_monitor_distance_zero_before_read();
    test_monitor_addzone_getzonecount_getzone();
    test_monitor_addzone_reversed_normalized();
    test_monitor_updatezone();
    test_monitor_certainty_clamp();
    test_monitor_update_enters_zone();
    test_monitor_interval_gate();
    test_monitor_timeout_path();
    test_monitor_dataready_false_path();
    test_monitor_isobjectinzone_is_pure_query();
    test_monitor_getzone_pointer_stability();
    test_monitor_deletezone();
    test_monitor_setters_getters_echo();
    test_monitor_init_wires_bus_and_continuous();

    std::printf("\n==============================\n");
    std::printf("Tests passed: %d\n", g_pass);
    std::printf("Tests failed: %d\n", g_fail);
    std::printf("==============================\n");
    return g_fail == 0 ? 0 : 1;
}
