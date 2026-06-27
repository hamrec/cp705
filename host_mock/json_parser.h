#pragma once
// JSON test-data parser for Mini-FT8 host mock.
// Adapted from dxft8-tests — uses C++ UiRxLine with std::string fields.

#include <string>
#include <vector>
#include "ui.h"              // UiRxLine

// ---------- Test data structures ----------

struct TestConfig {
    std::string my_callsign;
    std::string my_grid;
    std::string dx_callsign;
    std::string dx_grid;
    bool        tx_on_even = false;
    std::string cq_type;       // "CQ", "FD", "SOTA", etc.
    std::string free_text;     // FD exchange or freetext (e.g. "1A SCV")
    int         max_retry = 5; // autoseq_set_max_retry (default 5)
};

struct BeaconChange {
    float time_offset = 0.0f;
    int   beacon_on   = 0;       // 0=off, 1=even, 2=odd
};

struct TouchEvent {
    float time_offset    = 0.0f;
    int   message_index  = 0;
};

struct FreeTextEvent {
    float       time_offset = 0.0f;
    std::string text;
};

struct TestPeriod {
    std::vector<UiRxLine> messages;     // RX messages for this period
    bool         has_beacon_change = false;
    BeaconChange beacon_change;
    bool         has_touch_event = false;
    TouchEvent   touch_event;
    bool          has_freetext_event = false;
    FreeTextEvent freetext_event;
};

struct TestData {
    TestConfig              config;
    std::vector<TestPeriod> periods;
};

// ---------- API ----------

bool load_test_data(const std::string& filename, TestData& out);
