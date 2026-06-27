#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AUDIO_SOURCE_IC705_WIFI = 0,   // Icom IC-705 over WiFi UDP
    AUDIO_SOURCE_KH1_MIC   = 2,   // KH1 native I2S mic (value kept for saved configs)
} audio_source_backend_t;

void audio_source_set_backend(audio_source_backend_t backend);
audio_source_backend_t audio_source_get_backend(void);
const char* audio_source_backend_name(audio_source_backend_t backend);

// ic705_ip: resolved IC-705 IP in network byte order (ignored for KH1_MIC).
// Pass 0 when not using IC-705.
bool audio_source_start_ic705(uint32_t ic705_ip);

bool audio_source_start(void);   // uses last ic705_ip set via audio_source_start_ic705
void audio_source_stop(void);

bool audio_source_is_streaming(void);
bool audio_source_ic705_detected(void);   // replaces audio_source_qmx_detected
const char* audio_source_get_status_string(void);
const char* audio_source_get_debug_line1(void);
const char* audio_source_get_debug_line2(void);
bool audio_source_get_latest_waterfall_row(uint8_t* out_row, int out_len);

#ifdef __cplusplus
}
#endif
