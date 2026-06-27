#pragma once

// ============================================================================
// core_api_internal.h
//
// Internal hooks called by the functional-core implementation and by the
// existing mutation sites in main.cpp / stream_uac.cpp. NOT a public API —
// external consumers use core_api.h.
//
// These helpers exist so that changes produced inside main.cpp's legacy
// code paths can also wake up registered core_on_*_changed() callbacks
// without the core API having to monkey-patch every write site.
// ============================================================================

#include <cstdint>

// Called by any code that mutates RX list state.
// Fires the registered core_on_rx_changed callback (if any).
void core_fire_rx_changed();

// Called by any code that mutates the autoseq / QSO queue / next-TX state.
// Fires the registered core_on_qso_changed callback (if any).
void core_fire_qso_changed();

// Called by any code that mutates station configuration.
// Fires the registered core_on_config_changed callback (if any).
void core_fire_config_changed();

// Called by the DSP task once per symbol (~6.25 Hz) with a fresh waterfall row.
// mag may be null during TX; num_bins is zero in that case.
// Fires the registered core_on_waterfall_row callback (if any).
void core_fire_waterfall_row(int sym,
                             const uint8_t* mag, int num_bins,
                             float swr, float pwr, bool ptt);
