#include "ui.h"
#include <M5Unified.h>
#include <M5Cardputer.h>
#include <cstring>
#include "freertos/semphr.h"

// Static RX list — zero-heap display pipeline
static RxDecodeEntry rx_lines[RX_MAX_DECODES];
static int rx_lines_count = 0;
static int rx_page = 0;
static int rx_selected = -1;  // global index into rx_lines
struct RxDrawCacheEntry {
    char text[RX_TEXT_MAX];
    bool is_cq;
    bool is_to_me;
    bool valid;
};
static RxDrawCacheEntry last_drawn_cache[RX_MAX_DECODES];
static int last_drawn_count = 0;
static int last_page = -1;
static std::string g_visible_rows[RX_LINES];

static SemaphoreHandle_t g_disp_mutex = nullptr;

static void disp_lock() {
    if (g_disp_mutex) {
        xSemaphoreTake(g_disp_mutex, portMAX_DELAY);
    }
}

static void disp_unlock() {
    if (g_disp_mutex) {
        xSemaphoreGive(g_disp_mutex);
    }
}

struct DispGuard {
    DispGuard() { disp_lock(); }
    ~DispGuard() { disp_unlock(); }
};

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void ui_init(bool display_only) {
    g_disp_mutex = xSemaphoreCreateMutex();

    if (display_only) {
        // Display-only board init: full M5Unified startup can claim
        // ES8311/I2S audio resources the radio backend needs to own itself.
        M5Cardputer.beginDisplayOnly(true);
    } else {
        auto cfg = M5.config();
        cfg.output_power = true;
        cfg.external_rtc = false;
        cfg.internal_mic = false;
        cfg.internal_spk = false;
        cfg.external_speaker_value = 0;
        M5Cardputer.begin(cfg, true);
    }
    M5.Display.setRotation(1);
    M5.Display.fillScreen(TFT_BLACK);
    ui_draw_countdown(0.0f, true);
}

// Countdown/progress bar lives at the very top edge of the display now that
// the waterfall strip above it is gone.
void ui_draw_countdown(float fraction, bool even_slot) {
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;
    int filled = (int)(fraction * SCREEN_W);
    // Draw a faint background to make the bar visible even at 0%
    DispGuard guard;
    M5.Display.fillRect(0, 0, SCREEN_W, COUNTDOWN_H, rgb565(20, 20, 40));
    if (filled > 0) {
        uint16_t color = even_slot ? rgb565(0, 180, 0) : rgb565(180, 0, 0);
        M5.Display.fillRect(0, 0, filled, COUNTDOWN_H, color);
    }
}

void ui_clear_countdown() {
    DispGuard guard;
    M5.Display.fillRect(0, 0, SCREEN_W, COUNTDOWN_H, TFT_BLACK);
}

// Helper: copy a UiRxLine into a RxDecodeEntry with bounded string copies
static void ui_copy_uirxline_to_entry(const UiRxLine& src, RxDecodeEntry* dst) {
    strncpy(dst->text,   src.text.c_str(),   RX_TEXT_MAX  - 1); dst->text[RX_TEXT_MAX - 1] = '\0';
    strncpy(dst->field1, src.field1.c_str(), RX_FIELD_MAX - 1); dst->field1[RX_FIELD_MAX - 1] = '\0';
    strncpy(dst->field2, src.field2.c_str(), RX_FIELD_MAX - 1); dst->field2[RX_FIELD_MAX - 1] = '\0';
    strncpy(dst->field3, src.field3.c_str(), RX_FIELD_MAX - 1); dst->field3[RX_FIELD_MAX - 1] = '\0';
    dst->snr       = src.snr;
    dst->offset_hz = src.offset_hz;
    dst->slot_id   = src.slot_id;
    dst->time_s    = 0.0f;
    dst->is_cq     = src.is_cq;
    dst->is_to_me  = src.is_to_me;
}

void ui_set_rx_list(const std::vector<UiRxLine>& lines) {
    int n = (int)lines.size();
    if (n > RX_MAX_DECODES) n = RX_MAX_DECODES;
    for (int i = 0; i < n; ++i) ui_copy_uirxline_to_entry(lines[i], &rx_lines[i]);
    rx_lines_count = n;
    rx_page = 0;
    rx_selected = -1;
    last_drawn_count = 0;
    last_page = -1;
}

void ui_set_rx_list_static(const RxDecodeEntry* entries, int count) {
    DispGuard guard;
    int n = count;
    if (n > RX_MAX_DECODES) n = RX_MAX_DECODES;
    if (n < 0) n = 0;
    for (int i = 0; i < n; ++i) rx_lines[i] = entries[i];  // POD copy, no heap
    rx_lines_count = n;
    rx_page = 0;
    rx_selected = -1;
    last_drawn_count = 0;
    last_page = -1;
}

bool ui_get_rx_entry(int idx, RxDecodeEntry* out) {
    if (!out) return false;
    DispGuard guard;
    if (idx < 0 || idx >= rx_lines_count) return false;
    *out = rx_lines[idx];  // POD copy
    return true;
}

int ui_get_rx_count() {
    DispGuard guard;
    return rx_lines_count;
}

void ui_force_redraw_rx() {
    last_drawn_count = 0;
    last_page = -1;
    // "The RX screen needs a full repaint" implies something drew over it
    // (mode entry, a full clear). The hero card shares that screen, so drop
    // its diff cache too -- otherwise a return to an active QSO from the menu
    // would leave the card only partially painted.
    ui_hero_invalidate();
}

static void draw_rx_line(int y, const RxDecodeEntry& l, int line_no, bool selected, bool cyan_index_marker) {
    uint16_t color = TFT_WHITE;
    if (l.is_to_me) {
        color = rgb565(255, 0, 0);
    } else if (l.is_cq) {
        color = rgb565(0, 220, 0);
    }
    // Sticky line number in first column
    uint16_t bg = selected ? rgb565(30, 30, 60) : TFT_BLACK;
    const uint16_t index_color = cyan_index_marker ? rgb565(0, 255, 255) : TFT_WHITE;
    M5.Display.fillRect(0, y, SCREEN_W, 16, bg);  // clear text band; gap handled by line_h
    M5.Display.setTextColor(index_color, bg);
    M5.Display.setCursor(0, y);
    M5.Display.printf("%d ", line_no);
    M5.Display.setTextColor(color, bg);
    M5.Display.printf("%s", l.text);
    int row_idx = line_no - 1;
    if (row_idx >= 0 && row_idx < RX_LINES) {
        char buf[RX_TEXT_MAX + 8];
        snprintf(buf, sizeof(buf), "%d %s", line_no, l.text);
        g_visible_rows[row_idx] = buf;
    }
}

void ui_draw_rx(int flash_index) {
    const int line_h = 19; // 16 text + 3 gap
    // Add a 3px gap below the countdown before the first line
    const int start_y = UI_START_Y;
    // Only redraw when page changes or content changes, but always draw if list is empty
    if (rx_lines_count > 0 && flash_index < 0) {
        if (rx_page == last_page && last_drawn_count == rx_lines_count) {
            bool same = true;
            for (int i = 0; i < rx_lines_count; ++i) {
                if (strcmp(rx_lines[i].text, last_drawn_cache[i].text) != 0 ||
                    rx_lines[i].is_cq != last_drawn_cache[i].is_cq ||
                    rx_lines[i].is_to_me != last_drawn_cache[i].is_to_me) {
                    same = false;
                    break;
                }
            }
            if (same) return;
        }
    }

    DispGuard guard;
    M5.Display.startWrite();
    M5.Display.setTextSize(2);
    const bool can_page_up = (rx_page > 0);
    const bool can_page_down = ((rx_page + 1) * RX_LINES < rx_lines_count);
    int start = rx_page * RX_LINES;
    for (int i = 0; i < RX_LINES; ++i) {
        int idx = start + i;
        int y = start_y + i * line_h;
        M5.Display.fillRect(0, y, SCREEN_W, line_h, TFT_BLACK);
        if (idx < rx_lines_count) {
            bool selected = (idx == flash_index);
            bool cyan_marker = ((i == 0) && can_page_up) || ((i == RX_LINES - 1) && can_page_down);
            draw_rx_line(y, rx_lines[idx], i + 1, selected, cyan_marker);
        } else {
            g_visible_rows[i].clear();
        }
    }
    M5.Display.endWrite();

    // cache drawn content
    if (flash_index < 0) {
        last_page = rx_page;
        last_drawn_count = rx_lines_count;
        for (int i = 0; i < rx_lines_count; ++i) {
            strncpy(last_drawn_cache[i].text, rx_lines[i].text, RX_TEXT_MAX - 1);
            last_drawn_cache[i].text[RX_TEXT_MAX - 1] = '\0';
            last_drawn_cache[i].is_cq = rx_lines[i].is_cq;
            last_drawn_cache[i].is_to_me = rx_lines[i].is_to_me;
            last_drawn_cache[i].valid = true;
        }
    } else {
        last_page = -1;
        last_drawn_count = 0;
    }
}

// --- QSO hero card -----------------------------------------------------
// QSO/CQ progress view, replaces the decode list while autoseq has an
// active context. DIFF-AWARE: each of the card's five row bands is
// repainted only when its own inputs changed since the last call, so a
// routine redraw touches a few hundred bytes of SPI instead of blasting the
// full ~65KB screen. That matters for the radio link, not just speed: a
// full-screen SPI/DMA burst contends with the WiFi DMA on the S3 (see the
// note in main.cpp's update_countdown()), and unlike TX-time draws these
// hero redraws land during the RX window where the burst can perturb the
// incoming audio stream and the timing of our outgoing keepalives. A full
// repaint still happens on the first draw and after ui_hero_invalidate()
// (called whenever something else -- menu, status, a full clear -- has
// painted over the card), which also covers the between-band gaps (y19-32,
// y60-73, y100-102) that the per-row fillRects don't touch.
static QsoHeroInfo s_hero_prev;
static bool s_hero_prev_valid = false;

void ui_hero_invalidate() { s_hero_prev_valid = false; }

static void hero_draw_stage_box(int x, int y, int w, int h, const char* label, int box_state) {
    // box_state: 0 = pending (dim outline), 1 = current (navy fill, white
    // outline — reuses the existing "selected" navy from the list views),
    // 2 = done (green outline, matches the app's existing CQ/is_cq green).
    uint16_t outline = TFT_WHITE;
    uint16_t fill = TFT_BLACK;
    uint16_t text_color = rgb565(90, 90, 90);
    if (box_state == 2) {
        outline = rgb565(0, 220, 0);
        text_color = outline;
    } else if (box_state == 1) {
        outline = TFT_WHITE;
        fill = rgb565(30, 30, 60);
        text_color = TFT_WHITE;
    } else {
        outline = rgb565(68, 68, 68);
    }
    M5.Display.fillRect(x, y, w, h, fill);
    M5.Display.drawRect(x, y, w, h, outline);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(text_color, fill);
    int tw = (int)strlen(label) * 6;  // Font0 glyph width at textSize(1)
    M5.Display.setCursor(x + (w - tw) / 2, y + (h - 8) / 2);
    M5.Display.print(label);
}

void ui_draw_qso_hero(const QsoHeroInfo& info) {
    DispGuard guard;
    const bool full = !s_hero_prev_valid;
    const QsoHeroInfo& p = s_hero_prev;

    // Which row bands changed? On a full repaint everything is "changed".
    const bool row0_ch = full ||
        info.qso_done != p.qso_done || info.qso_gave_up != p.qso_gave_up ||
        info.calling_cq != p.calling_cq || info.cq_text != p.cq_text ||
        info.clock_hm != p.clock_hm;
    const bool row1_ch = full ||
        info.dxcall != p.dxcall || info.dxgrid != p.dxgrid;
    const bool row2_ch = full || info.stage != p.stage;
    const bool row3_ch = full ||
        info.freq_band != p.freq_band || info.snr != p.snr;
    const bool row4_ch = full ||
        info.qso_done != p.qso_done || info.qso_gave_up != p.qso_gave_up ||
        info.qso_count != p.qso_count;

    if (!(row0_ch || row1_ch || row2_ch || row3_ch || row4_ch)) return;

    M5.Display.startWrite();
    // Full repaint: clear the whole screen once so the between-band gaps
    // (y19-32, y60-73, y100-102) that no per-row fillRect covers start black.
    // Incremental repaints skip this -- each changed row clears its own band.
    if (full) M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextWrap(false);

    // Row 0: status label + clock, y 4-23 (nudged down from y 0-19 so it
    // clears the countdown bar now living at the top edge, y 0-3).
    if (row0_ch) {
        M5.Display.fillRect(0, 4, SCREEN_W, 19, TFT_BLACK);
        M5.Display.setTextSize(1);
        M5.Display.setCursor(4, 7);
        if (info.qso_done) {
            M5.Display.setTextColor(rgb565(0, 220, 0), TFT_BLACK);
            M5.Display.print("QSO COMPLETE");
        } else if (info.qso_gave_up) {
            M5.Display.setTextColor(rgb565(230, 160, 0), TFT_BLACK);
            M5.Display.print("NO REPLY");
        } else if (info.calling_cq) {
            // Show the actual outgoing CQ text (incl. any POTA/SOTA/QRP prefix)
            // instead of a generic label, so it's visible at a glance that the
            // right message is armed -- falls back to the label if unavailable.
            M5.Display.setTextColor(rgb565(120, 170, 255), TFT_BLACK);
            M5.Display.print(info.cq_text.empty() ? "CALLING CQ" : info.cq_text.c_str());
        } else {
            M5.Display.setTextColor(rgb565(120, 170, 255), TFT_BLACK);
            M5.Display.print("WORKING");
        }
        if (!info.clock_hm.empty()) {
            int tw = (int)info.clock_hm.size() * 6;
            M5.Display.setTextColor(rgb565(150, 150, 150), TFT_BLACK);
            M5.Display.setCursor(SCREEN_W - tw - 4, 7);
            M5.Display.print(info.clock_hm.c_str());
        }
    }

    // Row 1: callsign + grid, y 32-60. Cursor at y=35 centers the size-3
    // glyph (24px tall) on y=47, the midpoint between row 0's bottom edge
    // (y=19) and the tracker's top edge (box_y=73).
    if (row1_ch) {
        M5.Display.fillRect(0, 32, SCREEN_W, 28, TFT_BLACK);
        M5.Display.setTextSize(3);
        M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        M5.Display.setCursor(4, 35);
        M5.Display.print(info.dxcall.empty() ? "--" : info.dxcall.c_str());
        if (!info.dxgrid.empty()) {
            M5.Display.setTextSize(2);
            M5.Display.setTextColor(rgb565(150, 150, 150), TFT_BLACK);
            int gx = SCREEN_W - (int)info.dxgrid.size() * 12 - 4;
            M5.Display.setCursor(gx, 39);
            M5.Display.print(info.dxgrid.c_str());
        }
        M5.Display.setTextSize(1);
    }

    // Row 2: six-stage tracker, y 73-100. Shifted +22px down from the
    // original y=51 (as a block with rows 3/4 below, spacing between the
    // three unchanged) so the footer's bottom gap matches row 0's top gap.
    if (row2_ch) {
        static const char* kStageLabels[6] = { "CQ", "GR", "RP", "R", "RR73", "73" };
        const int box_y = 73, box_h = 28, gap = 4;
        const int box_w = (SCREEN_W - gap * 5) / 6;
        for (int i = 0; i < 6; ++i) {
            int box_state = (i < info.stage) ? 2 : (i == info.stage ? 1 : 0);
            hero_draw_stage_box(i * (box_w + gap), box_y, box_w, box_h, kStageLabels[i], box_state);
        }
    }

    // Row 3: freq/band + SNR/DT, y 102-121 (TX line row removed -- redundant
    // with the CQ text now shown up in row 0 / the tracker's own state).
    if (row3_ch) {
        M5.Display.fillRect(0, 102, SCREEN_W, 19, TFT_BLACK);
        M5.Display.setTextSize(1);
        M5.Display.setTextColor(rgb565(150, 150, 150), TFT_BLACK);
        M5.Display.setCursor(4, 105);
        M5.Display.print(info.freq_band.c_str());
        if (info.snr > -99) {
            char buf[24];
            snprintf(buf, sizeof(buf), "SNR %+d", info.snr);
            int tw = (int)strlen(buf) * 6;
            M5.Display.setCursor(SCREEN_W - tw - 4, 105);
            M5.Display.print(buf);
        }
    }

    // Row 4: footer legend, y 121-135. Text at y=124 puts its bottom edge
    // (~132, 8px glyph) 3px above the screen bottom (135), matching row 0's
    // 3px top gap (its text sits at y=3).
    if (row4_ch) {
        M5.Display.fillRect(0, 121, SCREEN_W, SCREEN_H - 121, TFT_BLACK);
        M5.Display.setTextSize(1);
        if (info.qso_done) {
            M5.Display.setTextColor(rgb565(0, 220, 0), TFT_BLACK);
            M5.Display.setCursor(4, 124);
            M5.Display.print("Logged - ESC to dismiss");
        } else if (info.qso_gave_up) {
            M5.Display.setTextColor(rgb565(230, 160, 0), TFT_BLACK);
            M5.Display.setCursor(4, 124);
            M5.Display.print("Clearing...");
        } else {
            M5.Display.setTextColor(rgb565(0, 255, 255), TFT_BLACK);
            M5.Display.setCursor(4, 124);
            M5.Display.print("ESC to Stop");
        }
        char qbuf[24];
        snprintf(qbuf, sizeof(qbuf), "QSOs: %d", info.qso_count);
        int tw = (int)strlen(qbuf) * 6;
        M5.Display.setTextColor(rgb565(150, 150, 150), TFT_BLACK);
        M5.Display.setCursor(SCREEN_W - tw - 4, 124);
        M5.Display.print(qbuf);
    }

    M5.Display.setTextWrap(true);
    M5.Display.endWrite();

    s_hero_prev = info;
    s_hero_prev_valid = true;
}

// --- Boot splash ---------------------------------------------------------
// Icom-styled: red/blue accent bars (bracketing the screen top/bottom, the
// way Icom's own branding blocks color), white title, grey version, blue
// callsign. Drawn once at boot and held until dismissed — no dirty-tracking
// needed.
void ui_draw_splash(const std::string& callsign, const std::string& version) {
    DispGuard guard;
    M5.Display.startWrite();
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextWrap(false);

    const uint16_t icom_red  = rgb565(210, 20, 30);
    const uint16_t icom_blue = rgb565(0, 90, 170);
    const uint16_t dark_grey = rgb565(55, 55, 55);

    M5.Display.fillRect(0, 0, SCREEN_W, 6, icom_red);
    M5.Display.fillRect(0, SCREEN_H - 6, SCREEN_W, 6, icom_blue);

    M5.Display.setTextSize(4);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    static const char* kTitle = "CP705";
    int title_w = (int)strlen(kTitle) * 24;  // 6px glyph * textSize(4)
    M5.Display.setCursor((SCREEN_W - title_w) / 2, 28);
    M5.Display.print(kTitle);

    M5.Display.drawFastHLine(20, 62, SCREEN_W - 40, dark_grey);

    M5.Display.setTextSize(2);
    M5.Display.setTextColor(rgb565(150, 150, 150), TFT_BLACK);
    std::string vbuf = "v" + version;
    int vw = (int)vbuf.size() * 12;
    M5.Display.setCursor((SCREEN_W - vw) / 2, 72);
    M5.Display.print(vbuf.c_str());

    M5.Display.setTextColor(icom_blue, TFT_BLACK);
    std::string cbuf = callsign.empty() ? "--" : callsign;
    int cw = (int)cbuf.size() * 12;
    M5.Display.setCursor((SCREEN_W - cw) / 2, 98);
    M5.Display.print(cbuf.c_str());

    M5.Display.setTextSize(1);
    M5.Display.setTextWrap(true);
    M5.Display.endWrite();
}

// Simple keyboard: dot/‘.’ scroll forward page, comma/‘,’ scroll back.
int ui_handle_rx_key(char c) {
    int selected_idx = -1;
    if (c == 0) return selected_idx;
    if (c == ';') {
        if (rx_page > 0) {
            rx_page--;
            ui_draw_rx();
        }
    } else if (c == '.') {
        if ((rx_page + 1) * RX_LINES < rx_lines_count) {
            rx_page++;
            ui_draw_rx();
        }
    } else if (c >= '1' && c <= '6') {
        int line = c - '1';
        int idx = rx_page * RX_LINES + line;
        if (idx >= 0 && idx < rx_lines_count) {
            rx_selected = idx;
            ui_draw_rx();
            selected_idx = idx;
        }
    }
    return selected_idx;
}

// Simple numbered list drawing helper (6 lines/page), optional highlight by absolute index
void ui_draw_list(const std::vector<std::string>& lines, int page, int highlight_abs) {
    const int line_h = 19; // 16 text + 3 gap
    const int start_y = UI_START_Y;
    DispGuard guard;
    M5.Display.startWrite();
    M5.Display.setTextSize(2);
    // Clip long rows at the screen edge; without this M5GFX wraps the overflow
    // onto the next row and the text visibly stacks.
    M5.Display.setTextWrap(false);
    for (int i = 0; i < RX_LINES; ++i) {
        int idx = page * RX_LINES + i;
        int y = start_y + i * line_h;
        uint16_t bg = (idx == highlight_abs) ? rgb565(30, 30, 60) : TFT_BLACK;
        M5.Display.fillRect(0, y, SCREEN_W, line_h, bg);
        if (idx < (int)lines.size()) {
            M5.Display.setTextColor(TFT_WHITE, bg);
            M5.Display.setCursor(0, y);
            M5.Display.printf("%d %s", i + 1, lines[idx].c_str());
            g_visible_rows[i] = std::to_string(i + 1) + " " + lines[idx];
        } else {
            g_visible_rows[i].clear();
        }
    }
    M5.Display.setTextWrap(true);
    M5.Display.endWrite();
}

void ui_draw_debug(const std::vector<std::string>& lines, int page) {
    const int line_h = 19;
    const int start_y = UI_START_Y;
    DispGuard guard;
    M5.Display.startWrite();
    M5.Display.setTextSize(2);
    M5.Display.setTextWrap(false);  // clip long rows; don't wrap/stack onto next line
    for (int i = 0; i < RX_LINES; ++i) {
        int idx = page * RX_LINES + i;
        int y = start_y + i * line_h;
        M5.Display.fillRect(0, y, SCREEN_W, line_h, TFT_BLACK);
        if (idx < (int)lines.size()) {
            M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
            M5.Display.setCursor(0, y);
            M5.Display.printf("%s", lines[idx].c_str());
            g_visible_rows[i] = lines[idx];
        } else {
            g_visible_rows[i].clear();
        }
    }
    M5.Display.setTextWrap(true);
    M5.Display.endWrite();
}

void ui_get_visible_text_lines(std::vector<std::string>& out) {
    out.clear();
    out.reserve(RX_LINES);
    for (int i = 0; i < RX_LINES; ++i) {
        out.push_back(g_visible_rows[i]);
    }
}

void ui_set_visible_text_line(int row_idx, const std::string& text) {
    if (row_idx < 0 || row_idx >= RX_LINES) return;
    g_visible_rows[row_idx] = text;
}

void ui_get_rx_page_info(int& current_page, int& total_pages) {
    total_pages = (rx_lines_count <= 0) ? 1 : ((rx_lines_count + RX_LINES - 1) / RX_LINES);
    if (total_pages < 1) total_pages = 1;
    current_page = rx_page + 1;
    if (current_page < 1) current_page = 1;
    if (current_page > total_pages) current_page = total_pages;
}

