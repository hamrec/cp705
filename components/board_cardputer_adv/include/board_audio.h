#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int sample_rate;
    int channels;
    int bits_per_sample;
    float mic_gain_db;
    int speaker_volume;
} board_audio_config_t;

esp_err_t board_audio_init(const board_audio_config_t* cfg);
esp_err_t board_audio_read(int16_t* samples, size_t sample_count, size_t* bytes_read);
esp_err_t board_audio_write(const int16_t* samples, size_t sample_count, size_t* bytes_written);
void board_audio_deinit(void);
bool board_audio_is_initialized(void);

#ifdef __cplusplus
}
#endif
