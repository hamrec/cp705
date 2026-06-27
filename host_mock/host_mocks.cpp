#include "host_mocks.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ============== Mock clock ==============
static int64_t s_mock_ms = 0;

void mock_clock_set(int64_t ms)           { s_mock_ms = ms; }
int64_t mock_clock_get()                  { return s_mock_ms; }
void mock_clock_advance(int64_t delta_ms) { s_mock_ms += delta_ms; }

int64_t rtc_now_ms() { return s_mock_ms; }

// ============== CAT/CDC stubs ==============
bool cat_cdc_ready(void) { return false; }

int cat_cdc_send(const uint8_t* data, size_t len, uint32_t timeout_ms) {
    printf("[CAT] send %zu bytes\n", len);
    return 0; // ESP_OK
}

// ============== State name helpers ==============
const char* autoseq_state_name(int state) {
    static const char* names[] = {
        "CALLING", "REPLYING", "REPORT", "ROGER_REPORT",
        "ROGERS", "SIGNOFF", "IDLE"
    };
    if (state >= 0 && state <= 6) return names[state];
    return "???";
}

const char* tx_msg_type_name(int tx) {
    static const char* names[] = {
        "TX_NONE", "TX1", "TX2", "TX3", "TX4", "TX5", "TX6"
    };
    if (tx >= 0 && tx <= 6) return names[tx];
    return "???";
}
