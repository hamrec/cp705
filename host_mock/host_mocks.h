#pragma once
// Host mock declarations shared between host_main.cpp and host_mocks.cpp

#include <cstdint>
#include <string>

// ---------- Mock clock ----------
void     mock_clock_set(int64_t ms);
int64_t  mock_clock_get();
void     mock_clock_advance(int64_t delta_ms);

// ---------- Extern stubs that autoseq / main.cpp references ----------
// (defined in host_mocks.cpp so autoseq.cpp links without the real ESP32 code)

// rtc_now_ms() — used by main.cpp scheduling; not called by autoseq directly,
// but we provide it so test harness can call it.
int64_t rtc_now_ms();

// CAT/CDC stubs
bool       cat_cdc_ready(void);
int        cat_cdc_send(const uint8_t* data, size_t len, uint32_t timeout_ms);

// State names for human-readable output
const char* autoseq_state_name(int state);
const char* tx_msg_type_name(int tx);
