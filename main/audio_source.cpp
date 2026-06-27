// audio_source.cpp — qc705 is IC-705-over-WiFi only. This used to dispatch
// between the IC-705 WiFi stream and a KH1 USB-mic stream; the mic backend has
// been removed, so this is now a thin pass-through to stream_wifi_ic705.

#include "audio_source.h"

#include "ft8_audio_pipeline.h"
#include "stream_wifi_ic705.h"

#include "esp_log.h"

static const char* TAG = "AUDIO_SRC";
static uint32_t s_ic705_ip = 0;   // network-byte-order IPv4

void audio_source_set_backend(audio_source_backend_t /*backend*/) {}

audio_source_backend_t audio_source_get_backend(void) {
    return AUDIO_SOURCE_IC705_WIFI;
}

const char* audio_source_backend_name(audio_source_backend_t /*backend*/) {
    return "ic705_wifi";
}

bool audio_source_start_ic705(uint32_t ic705_ip) {
    s_ic705_ip = ic705_ip;
    return audio_source_start();
}

bool audio_source_start(void) {
    ESP_LOGI(TAG, "Start audio source backend=ic705_wifi ip=%08x", (unsigned)s_ic705_ip);
    return ic705_stream_start(s_ic705_ip);
}

void audio_source_stop(void) {
    ic705_stream_stop();
}

bool audio_source_is_streaming(void) {
    return ic705_stream_is_streaming();
}

bool audio_source_ic705_detected(void) {
    return ic705_stream_is_streaming();
}

const char* audio_source_get_status_string(void) {
    return ic705_stream_get_status_string();
}

const char* audio_source_get_debug_line1(void) {
    return ic705_stream_get_debug_line1();
}

const char* audio_source_get_debug_line2(void) {
    return ic705_stream_get_debug_line2();
}

bool audio_source_get_latest_waterfall_row(uint8_t* out_row, int out_len) {
    return ft8_audio_pipeline_get_latest_waterfall_row(out_row, out_len);
}
