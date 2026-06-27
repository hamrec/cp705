#pragma once
// Mock ESP-IDF esp_timer — shadows real esp_timer.h via include-path ordering
// Returns mock clock time (in microseconds) from host_mocks.cpp

#include <cstdint>

// Declared in host_mocks.h; use mock_clock for deterministic timing in tests.
int64_t mock_clock_get();

static inline int64_t esp_timer_get_time(void) {
    return mock_clock_get() * 1000;  // ms → us
}
