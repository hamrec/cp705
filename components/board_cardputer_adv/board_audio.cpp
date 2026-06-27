#include "board_audio.h"

#include <string.h>

#include "board_i2c.h"
#include "board_pins.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"

static const char* TAG = "BOARD_AUDIO";

static i2c_master_bus_handle_t s_i2c_bus = nullptr;
static i2s_chan_handle_t s_i2s_tx = nullptr;
static i2s_chan_handle_t s_i2s_rx = nullptr;
static esp_codec_dev_handle_t s_codec = nullptr;

static board_audio_config_t s_cfg = {
    .sample_rate = 48000,
    .channels = 1,
    .bits_per_sample = 16,
    .mic_gain_db = 30.0f,
    .speaker_volume = 80,
};

static bool s_initialized = false;

static esp_err_t board_audio_init_i2c(void)
{
    ESP_RETURN_ON_ERROR(board_i2c_get_bus(&s_i2c_bus), TAG, "board_i2c_get_bus failed");
    ESP_LOGI(TAG, "Using shared board I2C bus");
    return ESP_OK;
}

static esp_err_t board_audio_init_i2s(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = 240;

    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_i2s_tx, &s_i2s_rx),
                        TAG, "i2s_new_channel full-duplex failed");

    const uint32_t sample_rate_hz = (uint32_t)s_cfg.sample_rate;

    i2s_std_config_t rx_cfg = {};
    rx_cfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz);
    rx_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                          I2S_SLOT_MODE_MONO);
    rx_cfg.gpio_cfg.mclk = BOARD_I2S_MCLK;
    rx_cfg.gpio_cfg.bclk = BOARD_I2S_BCLK;
    rx_cfg.gpio_cfg.ws = BOARD_I2S_WS;
    rx_cfg.gpio_cfg.dout = GPIO_NUM_NC;
    rx_cfg.gpio_cfg.din = BOARD_I2S_DIN;
    rx_cfg.gpio_cfg.invert_flags.mclk_inv = false;
    rx_cfg.gpio_cfg.invert_flags.bclk_inv = false;
    rx_cfg.gpio_cfg.invert_flags.ws_inv = false;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_i2s_rx, &rx_cfg),
                        TAG, "RX i2s_channel_init_std_mode failed");

    i2s_std_config_t tx_cfg = {};
    tx_cfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz);
    tx_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                          I2S_SLOT_MODE_MONO);
    tx_cfg.gpio_cfg.mclk = BOARD_I2S_MCLK;
    tx_cfg.gpio_cfg.bclk = BOARD_I2S_BCLK;
    tx_cfg.gpio_cfg.ws = BOARD_I2S_WS;
    tx_cfg.gpio_cfg.dout = BOARD_I2S_DOUT;
    tx_cfg.gpio_cfg.din = GPIO_NUM_NC;
    tx_cfg.gpio_cfg.invert_flags.mclk_inv = false;
    tx_cfg.gpio_cfg.invert_flags.bclk_inv = false;
    tx_cfg.gpio_cfg.invert_flags.ws_inv = false;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_i2s_tx, &tx_cfg),
                        TAG, "TX i2s_channel_init_std_mode failed");

    ESP_LOGI(TAG, "I2S init OK: BCLK=%d WS=%d DIN=%d DOUT=%d rate=%d",
             BOARD_I2S_BCLK, BOARD_I2S_WS, BOARD_I2S_DIN, BOARD_I2S_DOUT,
             s_cfg.sample_rate);
    return ESP_OK;
}

static esp_err_t board_audio_init_codec(void)
{
    audio_codec_i2s_cfg_t i2s_cfg = {};
    i2s_cfg.rx_handle = s_i2s_rx;
    i2s_cfg.tx_handle = s_i2s_tx;

    const audio_codec_data_if_t* data_if = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_RETURN_ON_FALSE(data_if, ESP_FAIL, TAG, "audio_codec_new_i2s_data failed");

    audio_codec_i2c_cfg_t i2c_cfg = {};
    i2c_cfg.addr = ES8311_CODEC_DEFAULT_ADDR;
    i2c_cfg.bus_handle = s_i2c_bus;

    const audio_codec_ctrl_if_t* ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(ctrl_if, ESP_FAIL, TAG, "audio_codec_new_i2c_ctrl failed");

    const audio_codec_gpio_if_t* gpio_if = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(gpio_if, ESP_FAIL, TAG, "audio_codec_new_gpio failed");

    es8311_codec_cfg_t es8311_cfg = {};
    es8311_cfg.codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH;
    es8311_cfg.ctrl_if = ctrl_if;
    es8311_cfg.gpio_if = gpio_if;
    es8311_cfg.pa_pin = GPIO_NUM_NC;
    es8311_cfg.use_mclk = false;

    const audio_codec_if_t* codec_if = es8311_codec_new(&es8311_cfg);
    ESP_RETURN_ON_FALSE(codec_if, ESP_FAIL, TAG, "es8311_codec_new failed");

    esp_codec_dev_cfg_t dev_cfg = {};
    dev_cfg.codec_if = codec_if;
    dev_cfg.data_if = data_if;
    dev_cfg.dev_type = ESP_CODEC_DEV_TYPE_IN_OUT;

    s_codec = esp_codec_dev_new(&dev_cfg);
    ESP_RETURN_ON_FALSE(s_codec, ESP_FAIL, TAG, "esp_codec_dev_new failed");

    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = s_cfg.sample_rate;
    fs.channel = s_cfg.channels;
    fs.bits_per_sample = s_cfg.bits_per_sample;

    ESP_RETURN_ON_ERROR(esp_codec_dev_open(s_codec, &fs), TAG, "esp_codec_dev_open failed");
    ESP_RETURN_ON_ERROR(esp_codec_dev_set_in_gain(s_codec, s_cfg.mic_gain_db),
                        TAG, "esp_codec_dev_set_in_gain failed");
    ESP_RETURN_ON_ERROR(esp_codec_dev_set_out_vol(s_codec, s_cfg.speaker_volume),
                        TAG, "esp_codec_dev_set_out_vol failed");

    ESP_LOGI(TAG, "ES8311 opened BOTH: rate=%d ch=%d bits=%d mic_gain=%.1f speaker_vol=%d",
             s_cfg.sample_rate, s_cfg.channels, s_cfg.bits_per_sample,
             (double)s_cfg.mic_gain_db, s_cfg.speaker_volume);
    return ESP_OK;
}

esp_err_t board_audio_init(const board_audio_config_t* cfg)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "already initialized");
        return ESP_OK;
    }

    if (cfg) s_cfg = *cfg;
    if (s_cfg.sample_rate <= 0) s_cfg.sample_rate = 48000;
    if (s_cfg.channels <= 0) s_cfg.channels = 1;
    if (s_cfg.bits_per_sample <= 0) s_cfg.bits_per_sample = 16;
    if (s_cfg.speaker_volume < 0) s_cfg.speaker_volume = 0;
    if (s_cfg.speaker_volume > 100) s_cfg.speaker_volume = 100;

    esp_err_t err = board_audio_init_i2c();
    if (err != ESP_OK) {
        board_audio_deinit();
        return err;
    }

    err = board_audio_init_i2s();
    if (err != ESP_OK) {
        board_audio_deinit();
        return err;
    }

    err = board_audio_init_codec();
    if (err != ESP_OK) {
        board_audio_deinit();
        return err;
    }

    int16_t throwaway[480];
    for (int i = 0; i < 3; ++i) {
        memset(throwaway, 0, sizeof(throwaway));
        (void)esp_codec_dev_read(s_codec, throwaway, sizeof(throwaway));
    }

    ESP_LOGI(TAG, "discarded first 3 startup frames");
    s_initialized = true;
    return ESP_OK;
}

esp_err_t board_audio_read(int16_t* samples, size_t sample_count, size_t* bytes_read)
{
    if (!s_initialized || !s_codec) return ESP_ERR_INVALID_STATE;
    if (!samples || sample_count == 0) return ESP_ERR_INVALID_ARG;

    const int bytes_to_read = (int)(sample_count * sizeof(int16_t));
    int ret = esp_codec_dev_read(s_codec, samples, bytes_to_read);
    if (ret == ESP_CODEC_DEV_OK) {
        if (bytes_read) *bytes_read = bytes_to_read;
        return ESP_OK;
    }

    if (bytes_read) *bytes_read = 0;
    return ESP_FAIL;
}

esp_err_t board_audio_write(const int16_t* samples, size_t sample_count, size_t* bytes_written)
{
    if (!s_initialized || !s_codec) return ESP_ERR_INVALID_STATE;
    if (!samples || sample_count == 0) return ESP_ERR_INVALID_ARG;

    const int bytes_to_write = (int)(sample_count * sizeof(int16_t));
    int ret = esp_codec_dev_write(s_codec, (void*)samples, bytes_to_write);
    if (ret == ESP_CODEC_DEV_OK) {
        if (bytes_written) *bytes_written = bytes_to_write;
        return ESP_OK;
    }

    if (bytes_written) *bytes_written = 0;
    return ESP_FAIL;
}

void board_audio_deinit(void)
{
    if (s_codec) {
        esp_codec_dev_close(s_codec);
        s_codec = nullptr;
    }

    if (s_i2s_rx) {
        (void)i2s_channel_disable(s_i2s_rx);
        (void)i2s_del_channel(s_i2s_rx);
        s_i2s_rx = nullptr;
    }

    if (s_i2s_tx) {
        (void)i2s_channel_disable(s_i2s_tx);
        (void)i2s_del_channel(s_i2s_tx);
        s_i2s_tx = nullptr;
    }

    s_i2c_bus = nullptr;
    s_initialized = false;
}

bool board_audio_is_initialized(void)
{
    return s_initialized;
}
