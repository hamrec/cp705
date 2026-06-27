#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    int voltage_mv;
    int percent;
    bool charging_known;
    bool charging;
} board_power_status_t;

esp_err_t board_power_init(void);
esp_err_t board_power_read(board_power_status_t* out_status);

#ifdef __cplusplus
}
#endif