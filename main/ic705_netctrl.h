#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Icom WLAN remote-control network login (the same protocol used by RS-BA1,
// wfview, SDR-Control and kappanhang). Must be set before ic705_net_connect().
void ic705_net_set_credentials(const char* username, const char* password);

// Starts the background connect task: SID handshake + login + auth on the
// control stream (UDP 50001), then SID handshake + open on the serial/CI-V
// stream (UDP 50002). ic705_ip is network-byte-order IPv4. Non-blocking.
esp_err_t ic705_net_connect(uint32_t ic705_ip);

void ic705_net_disconnect(void);

// True once the CI-V serial stream is open and frames can be sent.
bool ic705_net_is_ready(void);

// Sends one full CI-V frame (FE FE ... FD) over the authenticated serial
// stream. Returns ESP_ERR_INVALID_STATE if not yet ready.
esp_err_t ic705_net_send_civ(const uint8_t* frame, size_t len);

const char* ic705_net_status_string(void);

// Audio stream (SID3, UDP port negotiated/granted during Open — see
// ic705_netctrl.cpp). True once its own SID handshake has succeeded;
// false (but CAT control still works) if it failed or hasn't come up yet.
bool ic705_net_audio_is_ready(void);

// Queues up to AUDIO_PCM_MAX (512) bytes of raw 16-bit PCM for transmit on
// the audio stream. Returns ESP_ERR_INVALID_STATE if audio isn't ready,
// ESP_ERR_INVALID_SIZE if len exceeds the per-packet maximum.
esp_err_t ic705_net_send_audio_pcm(const uint8_t* pcm, size_t len);

// TX audio send-success diagnostics: ok = full sends, fail = WiFi send() that
// couldn't take the packet (dropped audio). Used to chase the irregular pump.
void ic705_net_get_audio_tx_stats(uint32_t* ok, uint32_t* fail, uint32_t* rexmit);
void ic705_net_reset_audio_tx_stats(void);

// Radio's true sample rate (Hz) recovered from the RX stream; 48000.0 until
// enough continuous RX has been measured. TX paces to this to avoid clock drift.
double ic705_net_get_measured_rx_rate(void);

// Dump the next N actual TX audio packets' header bytes over the log (for
// byte-for-byte comparison against wfview's captured packets).
void ic705_net_dump_audio_pkts(int n);

// Pulls one received raw-PCM frame (already stripped of the 24-byte audio
// header) into out, up to max_len bytes, waiting up to timeout_ms.
// Returns the number of bytes copied, or 0 on timeout/no audio session.
int ic705_net_recv_audio_pcm(uint8_t* out, size_t max_len, int timeout_ms);

#ifdef __cplusplus
}
#endif
