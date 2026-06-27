#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// WiFi connection states
typedef enum {
    WIFI_MGR_IDLE,
    WIFI_MGR_CONNECTING,
    WIFI_MGR_CONNECTED,
    WIFI_MGR_RESOLVING,
    WIFI_MGR_READY,      // connected + IC-705 IP resolved
    WIFI_MGR_ERROR,
} wifi_mgr_state_t;

// Configure and start WiFi connection to IC-705 AP.
// ssid / password: WiFi credentials for IC-705 AP.
// hostname: mDNS hostname to resolve (e.g. "ic-705.local").
esp_err_t wifi_mgr_start(const char* ssid, const char* password, const char* hostname);

// Stop WiFi and release resources.
void wifi_mgr_stop(void);

// Current connection state.
wifi_mgr_state_t wifi_mgr_get_state(void);

// Returns true when connected and IC-705 IP has been resolved.
bool wifi_mgr_is_ready(void);

// Resolved IC-705 IPv4 address in network byte order (valid when ready).
uint32_t wifi_mgr_get_ic705_ip(void);

// Human-readable status string for display.
const char* wifi_mgr_status_string(void);

// Trigger a fresh mDNS resolve of the IC-705 hostname. Call if the radio
// restarted and its IP may have changed.
void wifi_mgr_resolve_now(void);

#ifdef __cplusplus
}
#endif
