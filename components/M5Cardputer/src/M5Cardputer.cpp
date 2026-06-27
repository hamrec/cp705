/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "M5Cardputer.h"
// #include "utility/Keyboard/KeyboardReader/IOMatrix.h"

using namespace m5;

M5_CARDPUTER M5Cardputer;

void M5_CARDPUTER::beginDisplayOnly(bool enableKeyboard)
{
    _enableKeyboard = enableKeyboard;
    _displayOnly = true;
    Display.begin();
    Display.setRotation(1);
    if (enableKeyboard) {
        Keyboard.beginCardputerADV();
        // Install the original-Cardputer IOMatrix (GPIO scan) reader directly.
        // The auto-detecting Keyboard.begin() depends on M5.getBoard(), which
        // is only populated after M5.begin() has run — and we deliberately
        // don't call M5.begin() in display-only mode (FT8 audio backends own
        // the speaker/mic/I2S resources). The previous default of
        // beginCardputerADV() forced the TCA8418 (I2C) reader, which silently
        // dropped every keypress on original-Cardputer hardware.
        // Keyboard.begin(std::unique_ptr<KeyboardReader>(new IOMatrixKeyboardReader()));
    }
}

void M5_CARDPUTER::begin(bool enableKeyboard)
{
    M5.begin();
    _enableKeyboard = enableKeyboard;
    _displayOnly = false;
    if (enableKeyboard) {
        Keyboard.begin();
    }
}

void M5_CARDPUTER::begin(m5::M5Unified::config_t cfg, bool enableKeyboard)
{
    M5.begin(cfg);
    _enableKeyboard = enableKeyboard;
    _displayOnly = false;
    if (enableKeyboard) {
        Keyboard.begin();
    }
}

void M5_CARDPUTER::update(void)
{
    if (!_displayOnly) {
        M5.update();
    }
    if (_enableKeyboard) {
        Keyboard.updateKeyList();
        Keyboard.updateKeysState();
    }
}
