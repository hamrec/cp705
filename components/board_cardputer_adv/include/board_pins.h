#pragma once

#include "driver/gpio.h"

// M5Stack Cardputer-Adv ES8311/TCA8418 shared board pins.
//
// ES8311 SDA    -> GPIO8
// ES8311 SCL    -> GPIO9
// ES8311 SCLK   -> GPIO41   I2S BCLK
// ES8311 ASDOUT -> GPIO46   I2S data from codec to ESP32
// ES8311 LRCK   -> GPIO43   I2S WS / LRCK
// ES8311 DSDIN  -> GPIO42   I2S data from ESP32 to codec
#define BOARD_I2C_SDA   GPIO_NUM_8
#define BOARD_I2C_SCL   GPIO_NUM_9

#define BOARD_I2S_BCLK  GPIO_NUM_41
#define BOARD_I2S_DIN   GPIO_NUM_46
#define BOARD_I2S_WS    GPIO_NUM_43
#define BOARD_I2S_DOUT  GPIO_NUM_42

// Cardputer-Adv works without explicit MCLK in the verified native-audio path.
#define BOARD_I2S_MCLK  GPIO_NUM_NC
