// ---------------------------------------------------------------------------
// radio_control_ic705.cpp
//
// Icom IC-705 CAT control via CI-V, carried over Icom's WLAN remote-control
// network protocol (the same proprietary protocol used by RS-BA1 / wfview /
// SDR-Control / FT8CN — UDP login+auth, not plain CI-V over a socket). The
// session/auth/transport layer lives in ic705_netctrl.cpp; this file only
// builds and sends CI-V frames.
//
// CI-V framing:
//   Preamble:  0xFE 0xFE
//   Dest addr: configurable (default 0xA4 for IC-705)
//   Src addr:  0xE0  (controller)
//   Command:   see below
//   Data:      variable
//   End:       0xFD
//
// Commands used:
//   0x05  Set operating frequency (5-byte BCD, LSB first)
//   0x06  Set operating mode (0x01 = USB, filter 0x02 = DATA filter preset)
//   0x1C  0x00  Transmit control  (0x01 = TX on, 0x00 = TX off)
//
// TX audio is NOT sent through this backend; it is streamed over UDP by
// stream_wifi_ic705.cpp.  set_tone_hz() is a no-op here — the DDS inside
// stream_wifi_ic705 generates tones for both CPFSK and tune modes.
// set_tune() calls ic705_tx_begin_tune() to start the DDS-driven UDP stream.
// ---------------------------------------------------------------------------

#include "radio_control_backend.h"
#include "stream_wifi_ic705.h"
#include "ic705_netctrl.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <inttypes.h>

#include "esp_log.h"

// CI-V address of this controller (source)
#define CIV_SRC_ADDR  0xE0u

static const char* TAG = "RADIO_IC705";

static uint8_t  s_civ_addr  = 0xA4u;   // IC-705 default; overridable at runtime
static uint32_t s_ic705_ip  = 0;       // network byte order, set by ic705_cat_set_target

// ---------------------------------------------------------------------------
// Connection management (called from on_audio_start)
// ---------------------------------------------------------------------------

// Public: called by main.cpp to pass the resolved IP and CI-V address
// before any ops are called.
void ic705_cat_set_target(uint32_t ic705_ip, uint8_t civ_addr) {
    s_ic705_ip = ic705_ip;
    s_civ_addr = civ_addr;
}

static esp_err_t cat_connect(void) {
    if (ic705_net_is_ready()) return ESP_OK;   // already connected
    if (s_ic705_ip == 0) {
        ESP_LOGW(TAG, "IC-705 IP not set");
        return ESP_ERR_INVALID_STATE;
    }
    return ic705_net_connect(s_ic705_ip);   // connects in the background
}

static void cat_disconnect(void) {
    ic705_net_disconnect();
}

// ---------------------------------------------------------------------------
// CI-V helpers
// ---------------------------------------------------------------------------

// Send a CI-V frame.  Frames up to 32 bytes cover every command we issue.
static esp_err_t civ_send(const uint8_t* data, uint8_t cmd,
                           const uint8_t* payload, size_t payload_len) {
    (void)data;  // unused: we build the frame here
    uint8_t buf[64];
    size_t  idx = 0;

    buf[idx++] = 0xFE;
    buf[idx++] = 0xFE;
    buf[idx++] = s_civ_addr;
    buf[idx++] = CIV_SRC_ADDR;
    buf[idx++] = cmd;
    if (payload && payload_len) {
        if (idx + payload_len + 1 > sizeof(buf)) return ESP_ERR_INVALID_SIZE;
        memcpy(buf + idx, payload, payload_len);
        idx += payload_len;
    }
    buf[idx++] = 0xFD;

    esp_err_t err = ic705_net_send_civ(buf, idx);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "CI-V send failed: %s", esp_err_to_name(err));
    }
    return err;
}

// Encode frequency in Hz to 5-byte BCD (LSB first), IC-705 CI-V format.
static void freq_to_bcd(uint32_t freq_hz, uint8_t* out) {
    // BCD digit pairs, each byte holds two BCD digits:
    //  out[0] = (1Hz digit) | (10Hz digit << 4)
    //  out[1] = (100Hz digit) | (1kHz digit << 4)
    //  ...
    for (int i = 0; i < 5; ++i) {
        uint8_t lo = (uint8_t)(freq_hz % 10); freq_hz /= 10;
        uint8_t hi = (uint8_t)(freq_hz % 10); freq_hz /= 10;
        out[i] = (uint8_t)(lo | (hi << 4));
    }
}

// ---------------------------------------------------------------------------
// radio_control_ops_t callbacks
// ---------------------------------------------------------------------------

static bool ic705_ready(void) {
    return ic705_net_is_ready();
}

static esp_err_t ic705_on_audio_start(void) {
    esp_err_t err = cat_connect();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "IC-705 network login started, civ_addr=0x%02X", (unsigned)s_civ_addr);
    }
    return err;
}

static esp_err_t ic705_sync_frequency_mode(int freq_hz) {
    // Ensure DATA mode is ON (USB-D) WITHOUT touching the base mode. Command
    // 0x1A 0x06, p1=0x01 (DATA ON), p2=0x01 (FIL1). We deliberately do NOT send
    // the base-mode command (0x06): that forces the radio through plain USB
    // first, which (a) visibly flickers the mode icon and (b) — in plain USB —
    // makes the modulator use the MIC, ignoring our network/LAN TX audio (keys
    // up, no RF). Sending ONLY the DATA-mode setting is a no-op when the radio
    // is already USB-D on FIL1 (no change, matching SDR-Control) and cleanly
    // promotes plain USB → USB-D when needed. The user operates FT8 from the
    // radio's USB-D preset on FIL1, so the base mode is already correct.
    // NOTE: byte 2 is the FILTER selector (0x01=FIL1, 0x02=FIL2, 0x03=FIL3),
    // NOT a sub-mode. It was previously 0x02, which yanked the radio from the
    // user's FIL1 (best for FT8) to FIL2 on every connect/TX. Must be 0x01.
    uint8_t datamode_payload[3] = { 0x06u, 0x01u, 0x01u };
    esp_err_t err = civ_send(nullptr, 0x1Au, datamode_payload, sizeof(datamode_payload));
    if (err != ESP_OK) return err;

    uint8_t bcd[5];
    freq_to_bcd((uint32_t)freq_hz, bcd);
    err = civ_send(nullptr, 0x05u, bcd, sizeof(bcd));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "IC-705 sync: mode=USB-D freq=%d Hz", freq_hz);
    }
    return err;
}

static esp_err_t ic705_begin_tx(int freq_hz, int tx_base_hz) {
    (void)tx_base_hz;
    // Re-assert USB-D before every PTT, same as ic705_set_tune() does. Without
    // this, if the radio ever drifted to plain USB (mic input) the real TX
    // path keyed up but ignored the network/LAN audio entirely — meter never
    // moved, no RF — while tune (which already syncs mode) kept working.
    esp_err_t err = ic705_sync_frequency_mode(freq_hz);
    if (err != ESP_OK) return err;

    uint8_t payload[2] = { 0x00u, 0x01u };   // sub=0x00, data=0x01 (TX on)
    err = civ_send(nullptr, 0x1Cu, payload, sizeof(payload));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "IC-705 TX on");
    }
    return err;
}

// No-op: tones are generated inside the DDS/UDP audio path, not via CI-V.
static esp_err_t ic705_set_tone_hz(float /*tone_hz*/) {
    return ESP_OK;
}

static esp_err_t ic705_end_tx(void) {
    // Stop UDP audio first so the IC-705 drops back to RX immediately after PTT releases
    ic705_tx_end();

    uint8_t payload[2] = { 0x00u, 0x00u };   // sub=0x00, data=0x00 (TX off)
    esp_err_t err = civ_send(nullptr, 0x1Cu, payload, sizeof(payload));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "IC-705 TX off");
    }
    return err;
}

// NOTE: the AH-705 auto-tune version of this (CI-V 0x1C 0x01 0x02, verified
// against the IC-705 CI-V Reference Guide) is the operational default and is
// documented in project notes — re-apply it once the carrier-pump bug is fixed.
// For now we use the audio-carrier tune so the pump can be debugged into the
// 50-ohm dummy load (a 1:1 match makes the full carrier safe for the finals).
static esp_err_t ic705_set_tune(bool enable, int freq_hz, int tone_hz) {
    if (!enable) {
        return ic705_end_tx();
    }
    esp_err_t err = ic705_sync_frequency_mode(freq_hz);
    if (err != ESP_OK) return err;

    uint8_t ptton[2] = { 0x00u, 0x01u };
    err = civ_send(nullptr, 0x1Cu, ptton, sizeof(ptton));
    if (err != ESP_OK) return err;

    if (!ic705_tx_begin_tune((float)tone_hz)) {
        uint8_t pttoff[2] = { 0x00u, 0x00u };
        civ_send(nullptr, 0x1Cu, pttoff, sizeof(pttoff));
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

// IC-705 does not have a CI-V time-set command (unlike QMX), so we return
// NOT_SUPPORTED; the caller gracefully ignores this.
static esp_err_t ic705_set_time(int /*hour*/, int /*minute*/, int /*second*/) {
    return ESP_ERR_NOT_SUPPORTED;
}

// ---------------------------------------------------------------------------
// Ops table
// ---------------------------------------------------------------------------

static const radio_control_ops_t k_ops = {
    .name                = "ic705",
    .ready               = ic705_ready,
    .on_audio_start      = ic705_on_audio_start,
    .sync_frequency_mode = ic705_sync_frequency_mode,
    .begin_tx            = ic705_begin_tx,
    .set_tone_hz         = ic705_set_tone_hz,
    .end_tx              = ic705_end_tx,
    .set_tune            = ic705_set_tune,
    .set_time            = ic705_set_time,
};

const radio_control_ops_t* radio_control_ic705_get_ops(void) {
    return &k_ops;
}

// Expose for disconnection on app shutdown / backend switch
void ic705_cat_disconnect(void) {
    cat_disconnect();
}
