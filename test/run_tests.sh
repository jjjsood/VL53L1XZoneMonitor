#!/usr/bin/env bash
# Compile and run the host-based VL53L1XZoneMonitor test suite.
# -I test/mocks comes before -I src so the mock VL53L1X.h is used.
set -euo pipefail

# Resolve the repo root from this script's location so it works from anywhere.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

g++ -std=c++14 -Wall -Wextra -I test/mocks -I src \
    test/test_main.cpp src/VL53L1XZoneMonitor.cpp \
    -o /tmp/vl53_tests

/tmp/vl53_tests
