#pragma once
#include <vector>
#include <string>
#include <stdint.h>

// ---------------------------------------------------------------------------
// Screen layout constants — shared between ui.cpp and callers that need to
// position content relative to the countdown bar.
// ---------------------------------------------------------------------------
#define SCREEN_W    240
#define SCREEN_H    135
#define COUNTDOWN_H   3    ///< Pixel height of the TX countdown bar, top edge
#define RX_LINES      6    ///< Visible text rows below the countdown bar
/// Y-coordinate where the text area begins. COUNTDOWN_H alone (3px) put text
/// immediately touching the bar's bottom edge with no gap -- same fix as the
/// hero card's row0 (+4px), applied at the source so every list-style screen
/// (STATUS, MENU, band select, long-edit) that derives its top row from this
/// constant clears the bar too.
#define UI_START_Y  (COUNTDOWN_H + 4)   // 7 px

// A lightweight RX line format you can fill from your decoder
struct UiRxLine {
    std::string text;  // already formatted for display
    int snr = 0;
    int offset_hz = 0; // audio-bin offset in Hz relative to passband center
    int slot_id = 0;   // 0 = even slot (0/30s), 1 = odd slot (15/45s)
    std::string field1; // parsed token 1 (call/CQ marker)
    std::string field2; // parsed token 2
    std::string field3; // parsed token 3 (grid/report/etc)
    bool is_cq = false;
    bool is_to_me = false;
};

// Plain-C RX entry used for zero-heap decode/display pipeline.
// Fixed-size char arrays avoid std::string heap allocations.
#define RX_MAX_DECODES  32
#define RX_TEXT_MAX     64
#define RX_FIELD_MAX    20

struct RxDecodeEntry {
    char text[RX_TEXT_MAX];
    char field1[RX_FIELD_MAX];
    char field2[RX_FIELD_MAX];
    char field3[RX_FIELD_MAX];
    int  snr;
    int  offset_hz;
    int  slot_id;
    float time_s;
    bool is_cq;
    bool is_to_me;
    int64_t heard_ms = 0;  // wall-clock ms this station was last (re-)heard;
                           // drives newest-first display order in the
                           // persistent, refresh-in-place decode list.
};

// Hero-card display data for the active QSO — populated by main.cpp from
// autoseq's QsoContext, kept plain (no autoseq.h dependency here).
struct QsoHeroInfo {
    bool calling_cq = false;   // true = our own CQ one-shot, no dxcall yet
    std::string cq_text;       // calling_cq only: actual outgoing CQ message
                                // (e.g. "CQ POTA KD3AN EM66"), shown in place
                                // of the generic status label so any modifier
                                // (POTA/SOTA/QRP/...) is visible at a glance
    std::string dxcall;
    std::string dxgrid;
    int stage = 0;             // 0..5, matches AutoseqState CALLING..SIGNOFF; 6 = all done (qso_done)
    std::string freq_band;     // e.g. "14.074  20m"
    int snr = -99;             // -99 = unknown/not yet reported (our measurement of them)
    std::string clock_hm;      // "HH:MM", already formatted
    int qso_count = 0;         // total logged QSOs this session, shown bottom-right
    bool qso_done = false;     // true during the post-QSO "COMPLETE" hold (all stages green)
    bool qso_gave_up = false;  // true during a post-give-up hold (retries exhausted, no
                               // reply ever heard) -- same auto-clear timing as qso_done,
                               // but the tracker stays frozen at its real stage (not all
                               // green) and the label reads differently.
};

void ui_init(bool display_only = false);
// Icom-styled boot splash: app title, version, and the operator's callsign.
void ui_draw_splash(const std::string& callsign, const std::string& version);
void ui_draw_countdown(float fraction, bool even_slot);  // 0.0-1.0 fill of the countdown bar, top edge
// Blanks the countdown bar strip to black. Called once, right before a TX
// starts, so the bar (frozen at whatever fraction it had when TX began,
// since redraws are held during TX) doesn't sit there stale for the whole
// transmission and then visibly jump once RX resumes and it starts ticking
// again -- a plain blank reads as "intentionally hidden" instead.
void ui_clear_countdown();
void ui_set_rx_list(const std::vector<UiRxLine>& lines);
// Zero-heap RX list setter — preferred when callers use RxDecodeEntry directly.
void ui_set_rx_list_static(const RxDecodeEntry* entries, int count);
// Copy a single RX entry by index (for touch handler, thread-safe via disp mutex).
// Returns true if idx is valid and out was populated.
bool ui_get_rx_entry(int idx, RxDecodeEntry* out);
// Current RX list count.
int ui_get_rx_count();
void ui_draw_rx(int flash_index = -1);
void ui_force_redraw_rx();
// QSO progress card, replaces the decode list while a QSO/CQ is active.
// Diff-aware: only the regions whose content actually changed since the last
// call are repainted, so a routine redraw (e.g. just the clock digit) is a
// few hundred bytes of SPI instead of a full ~65KB screen blast. A full-screen
// blast contends with the WiFi DMA on the S3 (documented in main.cpp) and,
// during the RX window, can perturb the radio link / keepalive timing -- so
// keeping each redraw small directly protects the connection. Call
// ui_hero_invalidate() to force the next call to do a full repaint (needed
// whenever something else -- menu, status, a full clear -- has painted over
// the card since it was last shown).
void ui_draw_qso_hero(const QsoHeroInfo& info);
// Force the next ui_draw_qso_hero() call to fully repaint the card (drops the
// diff cache). Call when transitioning into the hero view or after anything
// else has drawn over the screen.
void ui_hero_invalidate();
// Returns selected absolute index or -1 if none
int ui_handle_rx_key(char c);
// Generic list draw (6 lines per page)
void ui_draw_list(const std::vector<std::string>& lines, int page, int highlight_abs = -1);
void ui_draw_debug(const std::vector<std::string>& lines, int page);
// Returns the currently rendered text rows (exact strings drawn for lines 1..6).
void ui_get_visible_text_lines(std::vector<std::string>& out);
// Override one mirrored row for custom render paths outside ui.cpp (e.g. STATUS).
void ui_set_visible_text_line(int row_idx, const std::string& text);
// RX paging info (1-based current page, total pages >= 1).
void ui_get_rx_page_info(int& current_page, int& total_pages);
