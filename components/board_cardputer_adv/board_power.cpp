#include "board_power.h"

#include <string.h>

#include "esp_log.h"
#include "esp_check.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char* TAG = "BOARD_POWER";

// M5Unified uses ADC1 GPIO10 for M5CardputerADV battery measurement.
// Battery divider ratio is 2.0.
static constexpr adc_unit_t kAdcUnit = ADC_UNIT_1;
static constexpr adc_channel_t kBatAdcChannel = ADC_CHANNEL_9;  // GPIO10 on ESP32-S3 ADC1
static constexpr float kAdcRatio = 2.0f;

static adc_oneshot_unit_handle_t g_adc = nullptr;
static adc_cali_handle_t g_cali = nullptr;
static bool g_initialized = false;
static bool g_cali_ok = false;

static int voltage_to_percent(int mv)
{
    // Simple Li-ion approximation.
    // Good enough for display. Not a fuel gauge.
    if (mv >= 4200) return 100;
    if (mv >= 4100) return 90;
    if (mv >= 4000) return 80;
    if (mv >= 3900) return 65;
    if (mv >= 3800) return 50;
    if (mv >= 3700) return 35;
    if (mv >= 3600) return 20;
    if (mv >= 3500) return 10;
    if (mv >= 3400) return 5;
    return 0;
}

esp_err_t board_power_init(void)
{
    if (g_initialized) {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t init_cfg = {};
    init_cfg.unit_id = kAdcUnit;

    ESP_RETURN_ON_ERROR(
        adc_oneshot_new_unit(&init_cfg, &g_adc),
        TAG,
        "adc_oneshot_new_unit failed"
    );

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten = ADC_ATTEN_DB_12;
    chan_cfg.bitwidth = ADC_BITWIDTH_12;

    ESP_RETURN_ON_ERROR(
        adc_oneshot_config_channel(g_adc, kBatAdcChannel, &chan_cfg),
        TAG,
        "adc_oneshot_config_channel failed"
    );

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {};
    cali_cfg.unit_id = kAdcUnit;
    cali_cfg.chan = kBatAdcChannel;
    cali_cfg.atten = ADC_ATTEN_DB_12;
    cali_cfg.bitwidth = ADC_BITWIDTH_12;

    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &g_cali) == ESP_OK) {
        g_cali_ok = true;
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {};
    cali_cfg.unit_id = kAdcUnit;
    cali_cfg.atten = ADC_ATTEN_DB_12;
    cali_cfg.bitwidth = ADC_BITWIDTH_12;

    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &g_cali) == ESP_OK) {
        g_cali_ok = true;
    }
#endif

    ESP_LOGI(TAG,
             "battery ADC init OK: unit=1 channel=9 GPIO10 ratio=%.1f cali=%d",
             (double)kAdcRatio,
             (int)g_cali_ok);

    g_initialized = true;
    return ESP_OK;
}

esp_err_t board_power_read(board_power_status_t* out_status)
{
    if (!out_status) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_status, 0, sizeof(*out_status));
    out_status->voltage_mv = -1;
    out_status->percent = -1;
    out_status->charging_known = false;
    out_status->charging = false;

    ESP_RETURN_ON_ERROR(board_power_init(), TAG, "board_power_init failed");

    // Average several samples: the cell voltage jitters and sags transiently
    // under load (WiFi/TX current), so a single read can be ~300 mV low and
    // report a wildly wrong %. Averaging gives a stable reading.
    constexpr int kSamples = 16;
    int64_t raw_sum = 0;
    int raw_count = 0;
    for (int i = 0; i < kSamples; ++i) {
        int r = 0;
        if (adc_oneshot_read(g_adc, kBatAdcChannel, &r) == ESP_OK) {
            raw_sum += r;
            ++raw_count;
        }
    }
    if (raw_count == 0) {
        ESP_LOGW(TAG, "adc_oneshot_read failed");
        return ESP_FAIL;
    }
    int raw = (int)(raw_sum / raw_count);

    int adc_mv = raw;

    if (g_cali_ok && g_cali) {
        ESP_RETURN_ON_ERROR(
            adc_cali_raw_to_voltage(g_cali, raw, &adc_mv),
            TAG,
            "adc_cali_raw_to_voltage failed"
        );
    }

    int bat_mv = (int)(adc_mv * kAdcRatio + 0.5f);

    out_status->valid = true;
    out_status->voltage_mv = bat_mv;
    out_status->percent = voltage_to_percent(bat_mv);

    return ESP_OK;
}