#include "board_i2c.h"

#include "board_pins.h"
#include "esp_check.h"
#include "esp_log.h"

static const char* TAG = "BOARD_I2C";

static i2c_master_bus_handle_t s_i2c_bus = nullptr;

esp_err_t board_i2c_init(void)
{
    if (s_i2c_bus) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = BOARD_I2C_SDA;
    bus_cfg.scl_io_num = BOARD_I2C_SCL;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "I2C0 already exists; getting existing handle");
        err = i2c_master_get_bus_handle(I2C_NUM_0, &s_i2c_bus);
    }

    ESP_RETURN_ON_ERROR(err, TAG, "I2C init failed");
    ESP_LOGI(TAG, "I2C init OK: SDA=%d SCL=%d", BOARD_I2C_SDA, BOARD_I2C_SCL);
    return ESP_OK;
}

esp_err_t board_i2c_get_bus(i2c_master_bus_handle_t* out_bus)
{
    if (!out_bus) return ESP_ERR_INVALID_ARG;

    esp_err_t err = board_i2c_init();
    if (err != ESP_OK) {
        *out_bus = nullptr;
        return err;
    }

    *out_bus = s_i2c_bus;
    return ESP_OK;
}
