#include "wifi_manager.h"

#include <cstring>
#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
// (mdns.h removed — mDNS no longer used; see wifi_mgr_start)
#include "lwip/ip4_addr.h"

static const char* TAG = "WIFI_MGR";

// Bit flags for the internal event group
#define WIFI_CONNECTED_BIT   BIT0
#define WIFI_FAIL_BIT        BIT1

static wifi_mgr_state_t s_state       = WIFI_MGR_IDLE;
static uint32_t         s_ic705_ip    = 0;    // network-byte-order IPv4
static char             s_hostname[64] = "ic-705.local";
static char             s_ssid[33]    = {};
static char             s_pass[65]    = {};
static char             s_status[64]  = "Idle";
static EventGroupHandle_t s_event_group = NULL;
static bool             s_initialized = false;
static int              s_retry_count = 0;
#define MAX_RETRIES 10

// ---------------------------------------------------------------------------
// mDNS hostname resolution
// ---------------------------------------------------------------------------

// Hardcoded for now, used instead of mDNS resolution — the radio's address
// when running its own AP.
#define IC705_STATIC_IP_A 192
#define IC705_STATIC_IP_B 168
#define IC705_STATIC_IP_C 59
#define IC705_STATIC_IP_D 1

static void try_mdns_resolve(void) {
    esp_ip4_addr_t addr;
    IP4_ADDR(&addr, IC705_STATIC_IP_A, IC705_STATIC_IP_B, IC705_STATIC_IP_C, IC705_STATIC_IP_D);
    s_ic705_ip = addr.addr;
    char ip_str[16];
    snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&addr));
    snprintf(s_status, sizeof(s_status), "IC-705 @ %s", ip_str);
    s_state = WIFI_MGR_READY;
    ESP_LOGI(TAG, "Using static IC-705 IP %s", ip_str);
}

// ---------------------------------------------------------------------------
// Event handlers
// ---------------------------------------------------------------------------

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi started, connecting to %s", s_ssid);
        snprintf(s_status, sizeof(s_status), "Connecting to %s", s_ssid);
        s_state = WIFI_MGR_CONNECTING;
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_state == WIFI_MGR_IDLE) return;  // intentional stop
        if (s_retry_count < MAX_RETRIES) {
            s_retry_count++;
            snprintf(s_status, sizeof(s_status), "Reconnecting... (%d/%d)", s_retry_count, MAX_RETRIES);
            s_state = WIFI_MGR_CONNECTING;
            esp_wifi_connect();
            ESP_LOGI(TAG, "WiFi disconnected, retry %d/%d", s_retry_count, MAX_RETRIES);
        } else {
            snprintf(s_status, sizeof(s_status), "WiFi connect failed");
            s_state = WIFI_MGR_ERROR;
            xEventGroupSetBits(s_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "WiFi connect failed after %d retries", MAX_RETRIES);
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        char ip_str[16];
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Got IP: %s", ip_str);
        snprintf(s_status, sizeof(s_status), "IP: %s, resolving...", ip_str);
        s_state = WIFI_MGR_RESOLVING;
        s_retry_count = 0;
        xEventGroupSetBits(s_event_group, WIFI_CONNECTED_BIT);
    }
}

// ---------------------------------------------------------------------------
// Background resolve task
// ---------------------------------------------------------------------------

static void resolve_task(void* arg) {
    // Wait for WiFi connection
    EventBits_t bits = xEventGroupWaitBits(s_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(30000));
    if (bits & WIFI_CONNECTED_BIT) {
        // Give DHCP a moment to fully settle
        vTaskDelay(pdMS_TO_TICKS(500));
        try_mdns_resolve();
    } else {
        s_state = WIFI_MGR_ERROR;
        snprintf(s_status, sizeof(s_status), "WiFi connection timed out");
        ESP_LOGE(TAG, "WiFi connection timed out");
    }
    vTaskDelete(NULL);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t wifi_mgr_start(const char* ssid, const char* password, const char* hostname) {
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized; call wifi_mgr_stop first");
        return ESP_ERR_INVALID_STATE;
    }

    strncpy(s_ssid,     ssid     ? ssid     : "", sizeof(s_ssid) - 1);
    strncpy(s_pass,     password ? password : "", sizeof(s_pass) - 1);
    strncpy(s_hostname, hostname ? hostname : "ic-705.local", sizeof(s_hostname) - 1);

    s_state = WIFI_MGR_CONNECTING;
    s_retry_count = 0;
    s_ic705_ip = 0;
    snprintf(s_status, sizeof(s_status), "Starting WiFi...");

    s_event_group = xEventGroupCreate();

    // Init TCP/IP stack and netif (safe to call multiple times)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));

    wifi_config_t wifi_cfg = {};
    strncpy((char*)wifi_cfg.sta.ssid,     s_ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char*)wifi_cfg.sta.password, s_pass, sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_cfg.sta.pmf_cfg.capable  = true;
    wifi_cfg.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Disable WiFi modem power-save. The default (WIFI_PS_MIN_MODEM) sleeps the
    // radio between AP beacons and only services traffic in periodic bursts,
    // which stalls our real-time TX audio stream every ~1s — the carrier drops
    // to zero, then bursts back (visible as rhythmic pulsing + splatter on the
    // radio). Real-time audio over WiFi needs the modem always on. Costs a bit
    // more power, which is fine while actively operating.
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // NOTE: mDNS intentionally NOT initialized — the IC-705 is reached via a
    // hardcoded static IP (try_mdns_resolve() just assigns it; it does no
    // actual mDNS query), so the mDNS task + buffers were pure overhead on a
    // RAM-starved (no-PSRAM) board.

    // Kick off background resolve task
    xTaskCreate(resolve_task, "wifi_resolve", 4096, NULL, 3, NULL);

    s_initialized = true;
    ESP_LOGI(TAG, "WiFi started, SSID=%s host=%s", s_ssid, s_hostname);
    return ESP_OK;
}

void wifi_mgr_stop(void) {
    if (!s_initialized) return;
    s_state = WIFI_MGR_IDLE;
    snprintf(s_status, sizeof(s_status), "Idle");
    s_ic705_ip = 0;

    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();

    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler);
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);

    if (s_event_group) {
        vEventGroupDelete(s_event_group);
        s_event_group = NULL;
    }

    s_initialized = false;
    ESP_LOGI(TAG, "WiFi stopped");
}

wifi_mgr_state_t wifi_mgr_get_state(void) {
    return s_state;
}

bool wifi_mgr_is_ready(void) {
    return s_state == WIFI_MGR_READY && s_ic705_ip != 0;
}

uint32_t wifi_mgr_get_ic705_ip(void) {
    return s_ic705_ip;
}

const char* wifi_mgr_status_string(void) {
    return s_status;
}

void wifi_mgr_resolve_now(void) {
    if (s_state == WIFI_MGR_CONNECTED || s_state == WIFI_MGR_RESOLVING || s_state == WIFI_MGR_READY) {
        s_state = WIFI_MGR_RESOLVING;
        snprintf(s_status, sizeof(s_status), "Re-resolving %.40s...", s_hostname);
        try_mdns_resolve();
    }
}
