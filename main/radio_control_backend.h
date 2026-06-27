#pragma once

#include <stdbool.h>

#include "esp_err.h"

typedef struct {
    const char* name;
    bool (*ready)(void);
    esp_err_t (*on_audio_start)(void);
    esp_err_t (*sync_frequency_mode)(int freq_hz);
    esp_err_t (*begin_tx)(int freq_hz, int tx_base_hz);
    esp_err_t (*set_tone_hz)(float tone_hz);
    esp_err_t (*end_tx)(void);
    esp_err_t (*set_tune)(bool enable, int freq_hz, int tone_hz);
    esp_err_t (*set_time)(int hour, int minute, int second);
} radio_control_ops_t;

const radio_control_ops_t* radio_control_ic705_get_ops(void);

// IC-705 specific: configure CI-V target before ops are called
void ic705_cat_set_target(uint32_t ic705_ip, uint8_t civ_addr);
void ic705_cat_disconnect(void);
