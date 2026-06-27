#pragma once

// ---------------------------------------------------------------------------
// stream_wifi_ic705.h
//
// WiFi audio stream for the Icom IC-705 WLAN remote control protocol.
// Provides the same external interface contract as stream_uac.h so that
// higher-level code (audio_source, main) can call either implementation.
//
// The actual UDP session for audio (SID3 — its own "are you there" SID
// handshake, on the local/remote ports negotiated during the control
// stream's Open request) lives in ic705_netctrl.cpp/.h, alongside the
// control (SID1) and CI-V (SID2) sessions — see ic705_net_send_audio_pcm()
// / ic705_net_recv_audio_pcm() / ic705_net_audio_is_ready(). This file only
// renders/decodes PCM and feeds the FT8 pipeline; it does not open sockets
// or know about ports, matching the same separation of concerns already
// used for CI-V in radio_control_ic705.cpp.
// ---------------------------------------------------------------------------

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "ft8_audio_pipeline.h"

#ifdef __cplusplus
extern "C" {
#endif

// Audio format (must match IC-705 WLAN configuration)
#define IC705_SAMPLE_RATE    48000
#define IC705_CHANNELS       1        // mono over network
#define IC705_BITS           16

// Size of each UDP RX read attempt (bytes).
// At 48kHz / 16-bit / mono, 1 ms = 96 bytes.  128 samples = 256 bytes ≈ 2.7 ms.
#define IC705_RX_BUF_BYTES   1364

#define IC705_WATERFALL_ROW_WIDTH FT8_AUDIO_WATERFALL_ROW_WIDTH

typedef enum {
    IC705_STREAM_IDLE,
    IC705_STREAM_WAITING,    // WiFi manager not yet ready
    IC705_STREAM_CONNECTED,
    IC705_STREAM_STREAMING,
    IC705_STREAM_ERROR,
} ic705_stream_state_t;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

// Start RX audio streaming from IC-705.
// ic705_ip: IC-705 IPv4 address in network byte order (from wifi_mgr_get_ic705_ip).
bool ic705_stream_start(uint32_t ic705_ip);

// Stop all streaming (RX + TX).
void ic705_stream_stop(void);

ic705_stream_state_t ic705_stream_get_state(void);
bool ic705_stream_is_streaming(void);

const char* ic705_stream_get_status_string(void);
const char* ic705_stream_get_debug_line1(void);
const char* ic705_stream_get_debug_line2(void);
bool ic705_stream_get_latest_waterfall_row(uint8_t* out_row, int out_len);

// ---------------------------------------------------------------------------
// TX audio (mirrors stream_uac TX API)
// ---------------------------------------------------------------------------

// Begin continuous-phase FSK transmission.
// Generates audio samples from the DDS and streams them to the IC-705 via UDP.
bool ic705_tx_begin_cpfsk(float base_hz,
                           const uint8_t* symbols,
                           size_t symbol_count,
                           float tone_spacing_hz,
                           uint32_t samples_per_symbol);

// Begin a continuous tune tone.
bool ic705_tx_begin_tune(float tone_hz);

// Stop TX audio streaming.
void ic705_tx_end(void);

#ifdef __cplusplus
}
#endif
