#define DEBUG_LOG 1

#include <cstdio>
#include <cmath>
#include <cinttypes>
#include "esp_log.h"
extern "C" {
  #include "ft8/decode.h"
  #include "ft8/constants.h"
  #include "ft8/message.h"
  #include "ft8/encode.h"
  #include "ft8/debug.h"
  #include "common/monitor.h"
  }

#include "board_power.h"
#include "ui.h"
#include <vector>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_freertos_hooks.h"
#include "autoseq.h"
#include "core_api.h"
#include "qso_log.h"
#include "core_api_internal.h"
#include <M5Cardputer.h>
#include <sstream>
#include <iterator>
#include <cstdio>
#include <string>
#include <cstdint>
#include <vector>
#include <array>
#include <cstring>
#include <algorithm>
#include <memory>
#include "driver/usb_serial_jtag.h"
#include "hal/uart_ll.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "soc/uart_pins.h"
#include "esp_system.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "nvs.h"   // config persistence (Station.txt blob in NVS)
#include <cctype>
#include <cstdlib>
#include <ctime>
#include <sys/time.h>
#include "esp_timer.h"
#include "esp_sleep.h"
#include "audio_source.h"
#include "stream_wifi_ic705.h"
#include "wifi_manager.h"
#include "dds_q15.h"
#include "radio_control.h"
#include "radio_control_backend.h"
#include "ic705_netctrl.h"
#include "gps.h"
#include "external_rtc.h"

#include "storage_service.h"
#include <dirent.h>        // opendir/readdir (SD self-test)

static const char* STATION_FILE = "Station.txt";

#include "feature_flags.h"
#include "protocol.h"

// Active protocol for this boot session — set once by load_station_data() from
// Station.txt (protocol_mode=FT4), defaults to FT8.  Never changed mid-session;
// reboot to apply a mode change.
const ProtocolConfig* g_protocol = &kProtocolFT8;

#ifndef FT8_SAMPLE_RATE
#define FT8_SAMPLE_RATE 6000
#endif

int64_t rtc_now_ms();
using CopyLogsResult = StorageCopyResult;

static void debug_log_line(const std::string& msg);
//exported symbol (linkable from other .cpp)
void debug_log_line_public(const std::string& msg) {
  debug_log_line(msg);
}

static std::string today_qso_file_name() {
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);
  char name[20];
  snprintf(name, sizeof(name), "%04d%02d%02d.txt",
           (t.tm_year + 1900) % 10000, (t.tm_mon + 1) % 100, t.tm_mday % 100);
  return name;
}

static std::string storage_basename(const std::string& name_or_path) {
  size_t slash = name_or_path.find_last_of('/');
  if (slash == std::string::npos) return name_or_path;
  return name_or_path.substr(slash + 1);
}

static bool storage_is_active_log_name(const std::string& name_or_path) {
  const std::string name = storage_basename(name_or_path);
  return name == today_qso_file_name() ||
         name == "fieldday.txt";
}

static CopyLogsResult copy_logs_to_sd_overwrite() {
  return storage_copy_all_to_sd(today_qso_file_name());
}

// 128 entries × 16 bytes = 2 KB of BSS. 256 was the original size but
// well over typical working set (FT8 rarely sees >50 unique hashed
// callsigns in an active period; the aging + eviction logic keeps it
// fresh). Reducing by 2 KB gives the USB DMA buffer (4608 bytes) a
// Keep this compact so USB host setup retains contiguous heap.
// fragmentation.
#define CALLSIGN_HASHTABLE_SIZE 128

static struct
{
    char callsign[12]; /// Up to 11 symbols of callsign + trailing zero
    uint32_t hash;     /// 8 MSBs = age, 22 LSBs = hash value
} callsign_hashtable[CALLSIGN_HASHTABLE_SIZE];

static int callsign_hashtable_size;

void hashtable_init(void)
{
    callsign_hashtable_size = 0;
    memset(callsign_hashtable, 0, sizeof(callsign_hashtable));
}

// Increment age for all existing entries (saturate at 255). Call once per slot.
static void hashtable_age_all(void)
{
    for (int i = 0; i < CALLSIGN_HASHTABLE_SIZE; ++i)
    {
        if (callsign_hashtable[i].callsign[0] != '\0')
        {
            uint8_t age = (uint8_t)(callsign_hashtable[i].hash >> 24);
            if (age < 255)
            {
                age++;
                callsign_hashtable[i].hash =
                    ((uint32_t)age << 24) | (callsign_hashtable[i].hash & 0x003FFFFFu);
            }
        }
    }
}

// Trim the hash table if it grows too large by evicting the oldest entries
void hashtable_trim_size(int max_size)
{
    while (callsign_hashtable_size > max_size)
    {
        int oldest_idx = -1;
        uint8_t oldest_age = 0;

        for (int i = 0; i < CALLSIGN_HASHTABLE_SIZE; ++i)
        {
            if (callsign_hashtable[i].callsign[0] == '\0')
                continue;

            uint8_t age = (uint8_t)(callsign_hashtable[i].hash >> 24);
            if (oldest_idx < 0 || age > oldest_age)
            {
                oldest_idx = i;
                oldest_age = age;
            }
        }

        if (oldest_idx < 0)
            break;

        LOG(LOG_INFO, "Hashtable trim: removing oldest [%s], age=%u\n",
            callsign_hashtable[oldest_idx].callsign, (unsigned)oldest_age);

        callsign_hashtable[oldest_idx].callsign[0] = '\0';
        callsign_hashtable[oldest_idx].hash = 0;
        callsign_hashtable_size--;
    }
}

void hashtable_add(const char* callsign, uint32_t hash)
{
    if (!callsign || !callsign[0])
        return;

    uint32_t hash_payload = hash & 0x003FFFFFu;   // 22-bit value
    uint16_t hash10 = (hash_payload >> 12) & 0x03FFu;
    int idx = (hash10 * 23) % CALLSIGN_HASHTABLE_SIZE;
    int start_idx = idx;

    while (callsign_hashtable_size >= CALLSIGN_HASHTABLE_SIZE)
    {
        hashtable_trim_size(CALLSIGN_HASHTABLE_SIZE - 50);
        if (callsign_hashtable_size >= CALLSIGN_HASHTABLE_SIZE)
        {
            LOG(LOG_INFO, "Hash table full; ignoring new callsign [%s]\n", callsign);
            return;
        }
    }

    // Linear probing: must match lookup logic
    while (callsign_hashtable[idx].callsign[0] != '\0')
    {
        uint32_t existing_hash = callsign_hashtable[idx].hash & 0x003FFFFFu;

        if ((existing_hash == hash_payload) &&
            (strcmp(callsign_hashtable[idx].callsign, callsign) == 0))
        {
            // Refresh age to 0, keep same callsign/hash
            callsign_hashtable[idx].hash = hash_payload;
            LOG(LOG_DEBUG, "Found duplicate [%s], refreshed age\n", callsign);
            return;
        }

        if (existing_hash == hash_payload)
        {
            // Same 22-bit hash but different callsign: replace old one
            LOG(LOG_INFO, "Replacing [%s] with [%s] on same hash\n",
                callsign_hashtable[idx].callsign, callsign);

            strncpy(callsign_hashtable[idx].callsign, callsign, 11);
            callsign_hashtable[idx].callsign[11] = '\0';
            callsign_hashtable[idx].hash = hash_payload;
            return;
        }

        idx = (idx + 1) % CALLSIGN_HASHTABLE_SIZE;
        if (idx == start_idx)
        {
            LOG(LOG_INFO, "Hash table probe wrapped; abort insert for [%s]\n", callsign);
            return;
        }
    }

    strncpy(callsign_hashtable[idx].callsign, callsign, 11);
    callsign_hashtable[idx].callsign[11] = '\0';
    callsign_hashtable[idx].hash = hash_payload;  // age=0
    callsign_hashtable_size++;
}

bool hashtable_lookup(ftx_callsign_hash_type_t hash_type, uint32_t hash, char* callsign)
{
    if (!callsign)
        return false;

    uint8_t hash_shift =
        (hash_type == FTX_CALLSIGN_HASH_10_BITS) ? 12 :
        (hash_type == FTX_CALLSIGN_HASH_12_BITS) ? 10 : 0;

    // Derive the same start bucket from the top 10 bits of the 22-bit hash.
    // For 10-bit lookup: hash is already the top 10 bits.
    // For 12-bit lookup: top 10 bits are hash >> 2.
    // For 22-bit lookup: top 10 bits are hash >> 12.
    uint16_t hash10 =
        (hash_type == FTX_CALLSIGN_HASH_10_BITS) ? (hash & 0x03FFu) :
        (hash_type == FTX_CALLSIGN_HASH_12_BITS) ? ((hash >> 2) & 0x03FFu) :
                                                   ((hash >> 12) & 0x03FFu);

    int idx = (hash10 * 23) % CALLSIGN_HASHTABLE_SIZE;
    // Important: entries can be deleted by hashtable_trim_size(), which creates
    // empty holes in probe chains. Stopping at the first empty slot can miss
    // valid entries that were inserted later in that chain. Scan the full table.
    for (int probe = 0; probe < CALLSIGN_HASHTABLE_SIZE; ++probe)
    {
        int scan_idx = (idx + probe) % CALLSIGN_HASHTABLE_SIZE;
        if (callsign_hashtable[scan_idx].callsign[0] == '\0')
            continue;

        uint32_t existing_hash = callsign_hashtable[scan_idx].hash & 0x003FFFFFu;

        if ((existing_hash >> hash_shift) == hash)
        {
            strcpy(callsign, callsign_hashtable[scan_idx].callsign);

            // Reset age to 0 on successful hit, preserve 22-bit payload.
            callsign_hashtable[scan_idx].hash = existing_hash;
            return true;
        }
    }

    callsign[0] = '\0';
    return false;
}

ftx_callsign_hash_interface_t hash_if = {
    .lookup_hash = hashtable_lookup,
    .save_hash = hashtable_add
};

static std::string normalize_call_token(std::string s) {
  // trim <> wrappers used for hashed nonstd calls
  if (!s.empty() && s.front() == '<') s.erase(s.begin());
  if (!s.empty() && s.back()  == '>') s.pop_back();

  for (auto& ch : s) ch = (char)toupper((unsigned char)ch);
  return s;
}

static bool rewrite_dxpedition_for_mycall(const std::string& raw_text,
                                          const std::string& mycall_up,
                                          std::string& rewritten_text) {
  std::istringstream iss(raw_text);
  std::string call1, rr73_tok, call2, foxcall, rpt;
  if (!(iss >> call1 >> rr73_tok >> call2 >> foxcall >> rpt)) return false;

  std::string trailing;
  if (iss >> trailing) return false;
  if (rr73_tok != "RR73;") return false;

  std::string call1_up = normalize_call_token(call1);
  std::string call2_up = normalize_call_token(call2);
  if (call1_up.empty() || call2_up.empty() || mycall_up.empty()) return false;

  if (call1_up == mycall_up) {
    rewritten_text = call1 + " " + foxcall + " RR73";
    return true;
  }
  if (call2_up == mycall_up) {
    rewritten_text = call2 + " " + foxcall + " " + rpt;
    return true;
  }
  return false;
}

static const char* TAG = "FT8";
enum class UIMode { RX, BAND, MENU, STATUS, GPS, PERF };
enum class RtcTimeSource : uint8_t {
  SAVED = 0,
  ESP_RTC,
  DS3231,
  GPS,
  MANUAL,
};
static UIMode ui_mode = UIMode::RX;
// NOTE: previous `std::vector<UiRxLine> g_rx_lines` was removed to eliminate
// the last heap allocation in the decode/display path. The RX list now lives
// as a static RxDecodeEntry array inside ui.cpp, populated via
// ui_set_rx_list_static() and read back via ui_get_rx_entry()/ui_get_rx_count().
int64_t g_decode_slot_idx = -1; // set at decode trigger to tag RX lines with slot parity
// Monotonic index of the most recent slot whose decode has been fully applied to
// autoseq state (or whose audio was never decoded, e.g. paused/skipped). Enforces
// the sequential invariant "TX in slot N is blocked until decode for slot N-1 is
// applied." Written by ic705 stream task on core 1, read by check_slot_boundary on
// core 0. Initialized to -1 so the first TX after boot isn't blocked.
volatile int64_t g_decode_applied_slot_idx = -1;

// Set after IC-705 CI-V TCP connection opens. The main loop consumes it once
// to synchronize the selected band and mode to the IC-705.
volatile bool g_ic705_initial_sync_pending = false;

// Legacy alias so any remaining references in unchanged code still compile.
volatile bool& g_cdc_initial_sync_pending = g_ic705_initial_sync_pending;

// Deferred-save flag. main.cpp owns storage; core_api commands only request
// a deferred save.
volatile bool g_config_save_pending = false;

// State machine variables (matching reference project architecture)
// TX is scheduled by setting these flags; actual TX starts at slot boundary
// Global TX-arming state: read by tx_tick on the next slot boundary.
// Non-static so core_api.cpp can arm it from any UI consumer.
volatile bool g_qso_xmit = false;        // TX is pending
volatile int g_target_slot_parity = 0;   // 0=even, 1=odd - parity of slot to TX on
static volatile bool g_was_txing = false;       // We were transmitting (for tick timing)
volatile bool g_decode_in_progress = false; // Block TX trigger while decoding
static int g_last_slot_parity = -1;             // For slot boundary detection (just parity, like reference)

static volatile uint32_t g_perf_idle_count[2] = {0, 0};
static uint32_t g_perf_prev_idle_count[2] = {0, 0};
static TickType_t g_perf_prev_sample_tick = 0;
static uint8_t g_perf_cpu_busy_pct[2] = {0, 0};
static bool g_perf_cpu_hook_ok[2] = {false, false};
static bool g_perf_cpu_sample_valid = false;

// BandItem now defined in station_types.h
#include "station_types.h"
std::vector<BandItem> g_bands = {   // visible to core_api.cpp
    {"160m", 1840},   {"80m", 3573},   {"60m", 5357},   {"40m", 7074},
    {"30m", 10136},   {"20m", 14074},  {"17m", 18100},  {"15m", 21074},
    {"12m", 24915},   {"10m", 28074},  {"6m", 50313},   {"2m", 144174},
};
static std::string g_active_band_text = "40 20 15 10";
static std::vector<int> g_active_band_indices;
static int band_page = 0;
static int band_edit_idx = -1;       // absolute index into g_bands
static std::string band_edit_buffer; // text while editing
void update_autoseq_cq_type();  // visible to core_api.cpp
int g_offset_hz = 1500;                  // visible to core_api.cpp
int g_band_sel = 5; // default 20m (index into g_bands)  // visible to core_api.cpp
static bool g_tune = false;
static int64_t g_tune_stop_at_ms = 0;
constexpr int kTuneAutoStopMs = 5000;  // brief automatic tone burst, not a manual toggle; matches TD705's 5s hard cap
// Calling CQ (the `C` prompt) IS the beacon trigger now -- no separate STATUS
// toggle. True from the moment a CQ is confirmed; keeps decode_monitor_results()
// re-issuing that same CQ (enqueue_running_cq()) every cycle the queue goes
// idle -- whether nobody answered, or a full QSO just finished -- until the
// user answers someone else's decode or presses ESC.
static bool g_cq_running = false;
// "QSO COMPLETE" hero-card hold, set the instant a signed-off exchange leaves
// the active queue (check_slot_boundary(), right after autoseq_tick() evicts
// it -- see there for why this is the correct, already-logged completion
// point). Held for a few seconds (matching TD705's reference behavior) before
// either resuming the running CQ or falling back to the plain RX list.
static bool g_qso_done_active = false;
static int64_t g_qso_done_since_ms = 0;
static bool g_qso_done_was_cq_running = false;
// True when this hold is a give-up (retries exhausted mid-exchange, no RR73/73
// ever heard) rather than a genuine sign-off -- same hold/auto-clear timing,
// different label/color so a failed contact doesn't read as a success.
static bool g_qso_done_gave_up = false;
constexpr int64_t kQsoDoneResumeCqMs = 4000;   // TD705: hold, then resume calling CQ
constexpr int64_t kQsoDoneReturnRxMs = 12000;  // TD705: hold, then fall back to RX (answered someone else's CQ)
[[maybe_unused]] static bool g_cat_toggle_high = false;
std::string g_date = "2025-12-11";      // visible to core_api.cpp
std::string g_time = "10:10:00";        // visible to core_api.cpp
static int status_edit_idx = -1;     // 0-5
static std::string status_edit_buffer;
static int status_cursor_pos = -1;
static std::vector<std::string> g_debug_lines;
static int debug_page = 0;
static const size_t DEBUG_MAX_LINES = 18; // 3 pages
static const size_t DEBUG_HUD_LINES = 2;  // slots 0-1 reserved for live HUD
static constexpr uint32_t APP_CORE0_STACK_BYTES = 12288; // Tune to 16384/18432 if Amin < 1536B
static TickType_t g_app_core0_stack_last_sample_tick = 0;
static uint32_t g_app_core0_stack_cur_free_bytes = 0;
static uint32_t g_app_core0_stack_min_free_bytes = 0;

static void host_handle_line(const std::string& line);
void save_station_data();  // visible to core_api.cpp

// Core commands request a save; the main task performs storage I/O.
extern volatile bool g_config_save_pending;
// TX entry for display and scheduling (populated by autoseq)
// Non-static for the same reason as g_qso_xmit / g_target_slot_parity
// above — core_api.cpp's tap_rx RPC arms these on user-pick events.
AutoseqTxEntry g_pending_tx;
bool g_pending_tx_valid = false;

// Forward declarations — definitions live near check_slot_boundary, where
// g_offset_src has been declared.
void arm_pending_tx(const AutoseqTxEntry& pending);
volatile bool g_tx_cancel_requested = false;   // visible to core_api.cpp
static void host_process_bytes(const uint8_t* buf, size_t len);
[[maybe_unused]] static void poll_host_uart();
static void enter_mode(UIMode new_mode);
static std::string menu_sleep_batt_line();
static int normalize_gps_baud_value(int value);
static gps_pins_t gps_pins_for_current_source();
static const char* gps_source_name();
static void apply_debug_uart_pin_policy();
static bool rtc_set_from_strings_source(RtcTimeSource source);
static esp_err_t rtc_write_external_from_soft(const char* reason);
static const char* rtc_time_source_suffix();
bool rtc_set_from_strings();
bool rtc_apply_manual_time_from_strings();   // visible to core_api.cpp
void rtc_sync_to_esp_rtc();                  // visible to core_api.cpp
static bool g_rx_dirty = false;



// HELP text for the serial host-command console (host_handle_line()) --
// unrelated to any on-screen UI mode.
static std::vector<std::string> g_host_help_lines = {
    "WRITE/APPEND/READ",
    "DELETE/LIST <file>",
    "WRITEBIN <file> <n>",
    "DATE/TIME/SLEEP",
    "INFO/HELP/EXIT",
};

static const char* kAppVersion = "3.0-beta3";

// Runtime latch: when true, we're still showing the startup screen. Either
// a keypress or the 5 s auto-dismiss timer (g_startup_start_ms) takes us
// out.
static bool    g_startup_active  = true;
static int64_t g_startup_start_ms = 0;    // set on the first tick we see in the splash branch
static constexpr int64_t kStartupAutoDismissMs = 5000;

static bool is_startup_direct_mode_key(char c) {
  const char k = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  switch (k) {
    case 'S':
    case 'R':
    case 'T':
    case 'G':
    case 'Q':
    case 'M':
    case 'N':
    case 'O':
    case 'B':
    case 'F':
    case 'P':
    case 'C':
    case 'D':
      return true;
    default:
      return false;
  }
}

// "Clear QSO log" lives in the settings menu (Logging category): its item
// number doubles as the confirm button — pressing it arms a 2-step confirm
// for wiping the NVS QSO log (e.g. starting fresh between POTA activations),
// pressing the SAME number again within kQClearArmMs actually clears it.
static bool g_q_clear_armed = false;
static int64_t g_q_clear_arm_deadline = 0;
static std::string g_q_clear_feedback;
static int64_t g_q_clear_feedback_deadline = 0;
constexpr int64_t kQClearArmMs = 3000;
static std::string host_input;
static const char* HOST_PROMPT = "CP705> ";
static bool usb_ready = false;
static QueueHandle_t s_key_inject_queue = nullptr;
static bool host_bin_active = false;
static size_t host_bin_remaining = 0;
static StorageStream* host_bin_stream = nullptr;
static uint32_t host_bin_crc = 0;
static uint32_t host_bin_expected_crc = 0;
static size_t host_bin_received = 0;
static std::vector<uint8_t> host_bin_buf;
static const size_t HOST_BIN_CHUNK = 512;
static size_t host_bin_chunk_expect = 0; // payload bytes this chunk (excludes CRC trailer)
static uint8_t host_bin_first8[8] = {0};
static uint8_t host_bin_last8[8] = {0};
static size_t host_bin_first_filled = 0;
static std::string host_bin_path;

// Software RTC
static time_t rtc_epoch_base = 0;
static int64_t rtc_ms_start = 0;
static int64_t rtc_last_update = 0;
static bool rtc_valid = false;
static RtcTimeSource g_rtc_time_source = RtcTimeSource::SAVED;

// RTC deep sleep compensation
// rtc_sleep_epoch: epoch time when entering deep sleep (for calculating elapsed time)
// rtc_comp is seconds per 10000 seconds. It remains load/save/core-API
// compatible, but the local O-page editor is no longer exposed.
static constexpr int kRtcCompFixed = 120;
static time_t g_rtc_sleep_epoch = 0;
int g_rtc_comp = kRtcCompFixed;        // visible to core_api.cpp
static int clamp_rtc_comp_value(int value) {
  if (value < -9000) return -9000;
  if (value > 9000) return 9000;
  return value;
}

// CqType, OffsetSrc, RadioType now defined in station_types.h
struct RadioProfileBinding {
  audio_source_backend_t audio_backend;
  radio_control_backend_t radio_backend;
};
CqType g_cq_type = CqType::CQ;                // visible to core_api.cpp
std::string g_cq_freetext = "FreeText";       // visible to core_api.cpp
bool g_skip_tx1 = false;                      // visible to core_api.cpp
int g_autoseq_max_retry = AUTOSEQ_MAX_RETRY;  // visible to core_api.cpp
// Display brightness, in steps of 1..10 (10% .. 100%). Persisted; applied to
// the panel via apply_brightness() on load and whenever changed in the menu.
static int g_brightness_step = 10;
static void apply_brightness() {
  int pct = g_brightness_step;
  if (pct < 1) pct = 1;
  if (pct > 10) pct = 10;
  M5.Display.setBrightness((uint8_t)(255 * pct / 10));
}
static std::string g_free_text = "TNX 73";
std::string g_call = "";   // visible to core_api.cpp; set via MENU P1 / Station.txt
std::string g_grid = "";    // visible to core_api.cpp; set via MENU P1 / Station.txt
static std::string g_grid_saved_manual = "";
static bool g_grid_from_gps = false;
static bool g_time_synced_from_gps = false;
static std::string g_grid_gps_display8;
bool g_decode_enabled = true;
// time_osr=1 (was 2): keeps the static waterfall at ~40KB instead of ~80KB on
// this no-PSRAM board so WiFi TX buffers + heap have room. MUST match
// WF_STATIC_SIZE in components/ft8_lib/common/monitor.c. Trade: slightly lower
// weak-signal decode sensitivity.
int g_time_osr = 1;
int g_freq_osr = 1;
OffsetSrc g_offset_src = OffsetSrc::RANDOM;  // visible to core_api.cpp
RadioType g_radio = RadioType::IC705;         // visible to core_api.cpp

// IC-705 WiFi settings (loaded from Station.txt)
static std::string g_ic705_wifi_ssid = "IC-705";
static std::string g_ic705_wifi_pass = "";     // set via MENU (IC-705 WiFi page) / Station.txt
static std::string g_ic705_hostname = "ic-705.local";
static int         g_ic705_civ_addr = 0xA4;   // default IC-705 CI-V address

// IC-705 network-control login (Settings > WLAN Set > Network Control on the
// radio). Empty by default — supply your own via Station.txt; never commit real
// credentials to source.
static std::string g_ic705_net_user = "";
static std::string g_ic705_net_pass = "";

static int g_gps_baud = 115200;
static RadioType canonical_radio_type(RadioType r);
static bool radio_type_uses_display_only(RadioType r);
static RadioProfileBinding get_radio_profile_binding(RadioType r);
void apply_radio_profile_binding();   // visible to core_api.cpp
static void gps_runtime_tick();
static std::string normalize_grid_maidenhead(const std::string& src);
// Non-static so core_api.cpp's set_call / set_grid RPCs can refresh the
// autoseq station info exactly like the on-device MENU/STATUS edits do.
std::string grid_ft8_4(const std::string& grid);
// Single-threaded TX state machine (replaces separate tx_send_task)
// TX runs in main loop via tx_tick(), one tone at a time
static bool g_tx_active = false;           // TX state machine is running
static int g_tx_tone_idx = 0;              // Current tone index (0..total_symbols-1)
static int64_t g_tx_next_tone_time = 0;    // When to send next tone (ms)
static int64_t g_tx_slot_start_ms = 0;     // Slot boundary time for tone alignment
static uint8_t g_tx_tones[FT4_NN];         // Encoded tones — sized for FT4 (105 > FT8's 79)
static int g_tx_base_hz = 0;               // Base frequency for TA commands
static int64_t g_tx_slot_idx = 0;          // Slot index for autoseq_mark_sent
static bool g_tx_cat_ok = false;           // CAT available for this TX
static int g_tx_last_ta_int = -1;          // For TA command deduplication
static int g_tx_last_ta_frac = -1;

static bool storage_should_guard_active_logs() {
  return g_tx_active || g_decode_in_progress || audio_source_is_streaming() || host_bin_active;
}

static bool storage_reject_active_log_user_mutation(const std::string& name_or_path) {
  return storage_should_guard_active_logs() && storage_is_active_log_name(name_or_path);
}

// Settings menu: M opens a category picker; picking one (1-4) shows that
// category's settings. -1 = picker, 0-3 = category index (see kCat* below).
// Each category fits in exactly one page of 6 rows, so unlike the old flat
// paged menu there's no in-category paging. menu_edit_idx stays a GLOBAL
// index across all categories (matches the pre-existing convention elsewhere
// in this file); MENU_CAT_BASE converts it to/from the local 0-5 row index
// each category's own small `lines` vector needs for ui_draw_list().
static int menu_category = -1;
constexpr int kCatStation  = 0;
constexpr int kCatOperating = 1;
constexpr int kCatNetwork  = 2;
constexpr int kCatLogging  = 3;
constexpr int kCatSystem  = 4;
constexpr int MENU_CAT_BASE = 6;  // global_index = category*6 + local_index
static int menu_edit_idx = -1;
// Tracks the protocol mode that has been saved to Station.txt and will take
// effect on next reboot.  Initialised from g_protocol after load_station_data().
// Differs from g_protocol when the user has toggled Mode but not yet rebooted.
#if ENABLE_FT4
static bool g_protocol_pending_ft4 = false;
#endif
static std::string menu_edit_buf;
static int menu_cursor_edit_original = 0;
static bool menu_long_edit = false;
static enum { LONG_NONE, LONG_FT, LONG_ACTIVE } menu_long_kind = LONG_NONE;
static std::string menu_long_buf;
static std::string menu_long_backup;
// -1 = cursor always at buffer end (ActiveBand's existing behavior).
// >=0 = insert/backspace operate at this position instead (used by the CQ
// text prompt, pre-positioned right after "CQ " so typing a prefix needs no
// backspacing/navigation first).
static int menu_long_cursor_pos = -1;
static int menu_flash_idx = -1;          // absolute index to flash highlight
static int64_t menu_flash_deadline = 0;  // ms timestamp when flash ends
static std::string menu_copy_feedback_text;
static int64_t menu_copy_feedback_deadline = 0;
static constexpr int64_t kMenuCopyFeedbackMs = 1800;
static int rx_flash_idx = -1;
static int64_t rx_flash_deadline = 0;
bool g_streaming = false;
static void draw_menu_view();
static void draw_battery_icon(int x, int y, int w, int h, int level, bool charging);
static void draw_status_view();
static void draw_status_line(int idx, const std::string& text, bool highlight);
void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui);
static void update_countdown();
static void consume_cdc_initial_sync();
// Non-static so core_api.cpp can push band changes to the radio immediately.
bool sync_radio_to_current_band(const char* reason);
static void menu_flash_tick();
static void rx_flash_tick();
static void tune_tick();
static bool looks_like_grid(const std::string& s);
static bool looks_like_report(const std::string& s, int& out);
static std::string g_last_reply_text;
void rebuild_active_bands();   // visible to core_api.cpp
static void schedule_tx_if_idle();
static int64_t s_last_tx_slot_idx = -1000;  // Track last TX slot for retry scheduling
[[maybe_unused]] static bool g_sync_pending = false;
[[maybe_unused]] static int g_sync_delta_ms = 0;
static void enqueue_running_cq();
static void qso_done_tick();
static void clear_decode_list();
static bool log_adif_entry(const std::string& dxcall, const std::string& dxgrid, int rst_sent, int rst_rcvd);
// Count of QSO records logged this session (shown in the QSO view). Incremented
// by log_adif_entry(), which may run on the core1 decode task — plain int only.
static volatile uint32_t g_adif_sd_seq  = 0;
// Non-static: qso_log.cpp reuses this for its best-effort internal-flash copy.
bool storage_append_text_locked_path(const std::string& path,
                                            const std::string& line,
                                            const std::string& header_if_new,
                                            bool sync_to_flash);
static bool storage_write_cabrillo_fd_entry(const std::string& mycall,
                                             const std::string& location,
                                             const std::string& qso_line);
#if !MIC_PROBE_APP
void log_heap(const char* tag) {
  size_t free_sz = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
  size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  ESP_LOGI(tag, "HEAP: free=%u min=%u largest=%u", (unsigned)free_sz, (unsigned)min_free, (unsigned)largest);
}
static void log_mem_caps(const char* tag) {
  size_t free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t largest_8bit = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  size_t largest_internal = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  size_t free_dma = heap_caps_get_free_size(MALLOC_CAP_DMA);
  size_t largest_dma = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
  size_t min_8bit = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
  ESP_LOGI(tag,
           "MEM: 8bit_free=%u 8bit_largest=%u internal_free=%u internal_largest=%u dma_free=%u dma_largest=%u 8bit_min=%u",
           (unsigned)free_8bit,
           (unsigned)largest_8bit,
           (unsigned)free_internal,
           (unsigned)largest_internal,
           (unsigned)free_dma,
           (unsigned)largest_dma,
           (unsigned)min_8bit);
}
static std::string fd_trim(const std::string& s) {
  size_t a = 0, b = s.size();
  while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) ++a;
  while (b > a && (s[b-1] == ' ' || s[b-1] == '\t' || s[b-1] == '\r' || s[b-1] == '\n')) --b;
  return s.substr(a, b - a);
}

static std::string fd_strip_R(const std::string& s) {
  std::string t = fd_trim(s);
  if (t.size() >= 2 && t[0] == 'R' && t[1] == ' ') return fd_trim(t.substr(2));
  return t;
}

static std::string fd_get_section_from_exchange(const std::string& ex) {
  // ex: "1B SCV" (or "R 1B SCV")
  std::string t = fd_strip_R(ex);
  size_t sp = t.find(' ');
  if (sp == std::string::npos) return "DX";
  return fd_trim(t.substr(sp + 1));
}

// Called by autoseq when an FD QSO completes. We derive freq/time from current radio state
// and use FreeText as our FD exchange (e.g. "1B SCV").
static bool log_cabrillo_fd_entry(const std::string& dxcall, const std::string& their_fd_exchange) {
  if (g_cq_type != CqType::CQFD) return true;

  const std::string my_fd = fd_strip_R(g_free_text);
  const std::string their_fd = fd_strip_R(their_fd_exchange);

  if (my_fd.empty() || their_fd.empty() || dxcall.empty()) return false;

  // Time (UTC assumed as RTC timebase, same as ADIF writer)
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);

  char date_ymd[16];
  snprintf(date_ymd, sizeof(date_ymd), "%04d-%02d-%02d",
           (t.tm_year + 1900) % 10000, (t.tm_mon + 1) % 100, t.tm_mday % 100);

  char time_hhmm[8];
  snprintf(time_hhmm, sizeof(time_hhmm), "%02d%02d", t.tm_hour % 100, t.tm_min % 100);

  // Frequency: use selected band dial frequency (kHz); round float to nearest integer.
  int freq_khz = (int)(g_bands[g_band_sel].freq + 0.5f);

  std::string location = fd_get_section_from_exchange(my_fd);

  char qso_line[128];
  snprintf(qso_line, sizeof(qso_line), "QSO: %d DG %s %s %s %s %s %s",
           freq_khz,
           date_ymd,
           time_hhmm,
           g_call.c_str(),
           my_fd.c_str(),
           dxcall.c_str(),
           their_fd.c_str());

  return storage_write_cabrillo_fd_entry(g_call, location, qso_line);
}

#else
static inline void log_heap(const char*) {}
static inline void log_mem_caps(const char*) {}
static bool log_cabrillo_fd_entry(const std::string&, const std::string&) { return true; }
#endif

bool storage_append_text_locked_path(const std::string& path,
                                             const std::string& line,
                                             const std::string& header_if_new,
                                             bool sync_to_flash) {
  return storage_file_append(path, line, header_if_new, sync_to_flash);
}

static bool storage_write_cabrillo_fd_entry(const std::string& mycall,
                                            const std::string& location,
                                            const std::string& qso_line) {
#if !MIC_PROBE_APP
  return storage_file_append_cabrillo(mycall, location, qso_line);
#else
  (void)mycall;
  (void)location;
  (void)qso_line;
  return true;
#endif
}

static bool nvs_save_station(const std::string& content);  // defined below
static bool nvs_load_station(std::string& out);            // defined below

// Autoseq's log callback: gather live station/band state into a QsoLogRecord and
// hand it to the qso_log module (ADIF formatting + durable NVS + SD/flash live
// there now). May run on the core1 decode task — record assembly only touches
// globals that are set once at startup / on band change.
static bool log_adif_entry(const std::string& dxcall, const std::string& dxgrid, int rst_sent, int rst_rcvd) {
  QsoLogRecord r;
  r.dxcall   = dxcall;
  r.dxgrid   = dxgrid;
  r.rst_sent = rst_sent;
  r.rst_rcvd = rst_rcvd;
  r.freq_mhz = 0.001 * (double)g_bands[g_band_sel].freq;
  r.mode     = g_protocol->name;
  r.mycall   = g_call;
  r.mygrid   = grid_ft8_4(g_grid);
  r.comment  = "CP705 IC-705";
  r.utc_ms   = rtc_now_ms();
  qso_log_write(r);
  g_adif_sd_seq = g_adif_sd_seq + 1;   // QSO count for the on-screen status
  return true;       // NVS write makes the record durable; never force a retry
}


static void ensure_usb() {
  if (usb_ready) return;
  usb_serial_jtag_driver_config_t cfg = {
    .tx_buffer_size = 1024,
    .rx_buffer_size = 4096,
  };
  if (usb_serial_jtag_driver_install(&cfg) == ESP_OK) {
    usb_ready = true;
  }
}

static bool uart_inject_last_was_cr = false;
static bool g_debug_uart_pins_enabled = true;

static void poll_uart_inject_keys() {
  if (!s_key_inject_queue || !g_debug_uart_pins_enabled) return;
  // Read directly from the console UART FIFO — no driver needed.
  // sdkconfig configures ESP console on UART0 peripheral with custom
  // pins TX=G4, RX=G5 (see CONFIG_ESP_CONSOLE_UART_CUSTOM_NUM_0
  // and CONFIG_ESP_CONSOLE_UART_TX_GPIO / _RX_GPIO).
  uart_dev_t *hw = UART_LL_GET_HW(0);
  while (true) {
    uint32_t avail = uart_ll_get_rxfifo_len(hw);
    if (avail == 0) break;
    if (avail > 64) avail = 64;
    uint8_t buf[64];
    uart_ll_read_rxfifo(hw, buf, avail);
    for (uint32_t i = 0; i < avail; i++) {
      char ch = (char)buf[i];
      // CR/LF handling: \r -> Enter, \n after \r -> skip (avoid double Enter)
      if (ch == '\r') {
        char enter = '\n';
        xQueueSend(s_key_inject_queue, &enter, 0);
        uart_inject_last_was_cr = true;
      } else if (ch == '\n' && uart_inject_last_was_cr) {
        uart_inject_last_was_cr = false;  // skip LF after CR
      } else {
        uart_inject_last_was_cr = false;
        xQueueSend(s_key_inject_queue, &ch, 0);
      }
    }
  }
}

static void host_write_str(const std::string& s) {
  ensure_usb();
  if (usb_ready) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(s.data());
    size_t remaining = s.size();
    while (remaining > 0) {
      size_t chunk = remaining;
      if (chunk > 256) chunk = 256;
      int written = usb_serial_jtag_write_bytes(p, chunk, portMAX_DELAY);
      if (written <= 0) break;
      p += written;
      remaining -= written;
    }
  }
}

// ================================================================
// UART screen mirror
//
// Debug aid for headless boards (e.g. StampS3Bat): every time a
// keystroke arrives over the console UART, dump the text that would
// have been displayed on the Cardputer LCD to the same UART TX, so
// a terminal shows the current page contents.
//
// To disable: comment out the `#define UART_SCREEN_MIRROR 1` below.
// ================================================================
#define UART_SCREEN_MIRROR 1

#if UART_SCREEN_MIRROR
static volatile bool g_uart_mirror_pending = false;

static const char* uart_mirror_mode_label(UIMode mode) {
  switch (mode) {
    case UIMode::RX:      return "RX";
    case UIMode::BAND:    return "BAND";
    case UIMode::MENU:    return "MENU";
    case UIMode::STATUS:  return "STATUS";
    case UIMode::GPS:     return "GPS";
    case UIMode::PERF:    return "PERF";
  }
  return "?";
}

static void uart_mirror_dump_screen() {
  std::vector<std::string> lines;
  ui_get_visible_text_lines(lines);

  // RX mode has proper paging info; other modes fall back to "page 1/1".
  int cur = 1, total = 1;
  if (ui_mode == UIMode::RX) {
    ui_get_rx_page_info(cur, total);
  }

  const char* label = uart_mirror_mode_label(ui_mode);
  printf("\n---- [%s %d/%d] ----\n", label, cur, total);
  for (size_t i = 0; i < lines.size(); ++i) {
    printf("%s\n", lines[i].c_str());
  }
  printf("--------------------\n");
  fflush(stdout);
}
#endif  // UART_SCREEN_MIRROR

static void set_gpio_floating_input(gpio_num_t pin) {
  gpio_reset_pin(pin);
  gpio_set_direction(pin, GPIO_MODE_INPUT);
  gpio_set_pull_mode(pin, GPIO_FLOATING);
  gpio_intr_disable(pin);
}

// The LoRa-1262 cap's GNSS is the only GPS source now, and it always needs
// G4/G5 free — so the debug UART on those pins stays parked floating
// unconditionally (previously conditional on the PORTA/GNSS_LoRa toggle).
static void apply_debug_uart_pin_policy() {
  const gpio_num_t tx = (gpio_num_t)U0TXD_GPIO_NUM;
  const gpio_num_t rx = (gpio_num_t)U0RXD_GPIO_NUM;
  if (s_key_inject_queue) xQueueReset(s_key_inject_queue);
  uart_inject_last_was_cr = false;
#if UART_SCREEN_MIRROR
  g_uart_mirror_pending = false;
#endif
  set_gpio_floating_input(tx);
  set_gpio_floating_input(rx);
  const bool changed = g_debug_uart_pins_enabled;
  g_debug_uart_pins_enabled = false;
  if (changed) ESP_LOGI(TAG, "G4/G5 debug UART disabled for GNSS LoRa");
}

struct WAVHeader {
  char riff[4];
  uint32_t file_size;
  char wave[4];
  char fmt[4];
  uint32_t fmt_size;
  uint16_t audio_format;
  uint16_t num_channels;
  uint32_t sample_rate;
  uint32_t byte_rate;
  uint16_t block_align;
  uint16_t bits_per_sample;
  char data[4];
  uint32_t data_size;
};

[[maybe_unused]] static esp_err_t decode_wav(const char* path) {
  ESP_LOGI(TAG, "Decoding %s", path);
  StorageStream* stream = storage_stream_open(path, StorageOpenMode::READ);
  if (!stream) {
    ESP_LOGE(TAG, "Failed to open %s", path);
    return ESP_FAIL;
  }

  WAVHeader hdr;
  if (storage_stream_read(stream, &hdr, sizeof(hdr)) != sizeof(hdr)) {
    ESP_LOGE(TAG, "Failed to read WAV header");
    storage_stream_close(stream);
    return ESP_FAIL;
  }
  if (memcmp(hdr.riff, "RIFF", 4) != 0 || memcmp(hdr.wave, "WAVE", 4) != 0) {
    ESP_LOGE(TAG, "Invalid WAV header");
    storage_stream_close(stream);
    return ESP_FAIL;
  }
  if (hdr.sample_rate != FT8_SAMPLE_RATE || hdr.num_channels != 1) {
    ESP_LOGE(TAG, "WAV must be mono %d Hz (got %" PRIu32 " Hz, %u ch)", FT8_SAMPLE_RATE, hdr.sample_rate, hdr.num_channels);
    storage_stream_close(stream);
    return ESP_FAIL;
  }

  const int bytes_per_sample = hdr.bits_per_sample / 8;

  monitor_config_t mon_cfg;
  mon_cfg.f_min = 200.0f;
  mon_cfg.f_max = 2900.0f;
  mon_cfg.sample_rate = FT8_SAMPLE_RATE;
  mon_cfg.time_osr = g_time_osr;
  mon_cfg.freq_osr = g_freq_osr;
  mon_cfg.protocol = g_protocol->protocol_id;

  monitor_t mon;
  monitor_init(&mon, &mon_cfg);
  monitor_reset(&mon);

  float* chunk = (float*)malloc(sizeof(float) * mon.block_size);
  if (!chunk) {
    ESP_LOGE(TAG, "Chunk alloc failed");
    storage_stream_close(stream);
    monitor_free(&mon);
    return ESP_ERR_NO_MEM;
  }

  bool eof = false;
  while (!eof) {
    int read_samples = 0;
    while (read_samples < mon.block_size && !eof) {
      float sample_value = 0.0f;
      if (bytes_per_sample == 1) {
        uint8_t sample = 0;
        if (storage_stream_read(stream, &sample, 1) != 1) {
          eof = true;
          break;
        }
        int s = sample;
        sample_value = ((float)s - 128.0f) / 128.0f;
      } else if (bytes_per_sample == 2) {
        uint8_t sample[2] = {};
        if (storage_stream_read(stream, sample, sizeof(sample)) != sizeof(sample)) {
          eof = true;
          break;
        }
        int low = sample[0];
        int high = sample[1];
        int16_t s = (int16_t)((high << 8) | low);
        sample_value = (float)s / 32768.0f;
      } else {
        eof = true;
        break;
      }
      chunk[read_samples++] = sample_value;
    }
    if (read_samples == 0) break;
    for (int i = read_samples; i < mon.block_size; ++i) {
      chunk[i] = 0.0f;
    }

    // Simple per-block AGC to ~0.1 target level
    double acc = 0.0;
    for (int i = 0; i < mon.block_size; ++i) acc += fabsf(chunk[i]);
    float level = (float)(acc / mon.block_size);
    float gain = (level > 1e-6f) ? 0.1f / level : 1.0f;
    if (gain < 0.1f) gain = 0.1f;
    if (gain > 10.0f) gain = 10.0f;
    for (int i = 0; i < mon.block_size; ++i) {
      chunk[i] *= gain;
    }

    monitor_process(&mon, chunk);
  }

  free(chunk);
  storage_stream_close(stream);

  if (mon.wf.num_blocks == 0) {
    ESP_LOGW(TAG, "No audio blocks processed");
    monitor_free(&mon);
    return ESP_FAIL;
  }
  decode_monitor_results(&mon, &mon_cfg, false); // defer UI to main loop on core1
  monitor_free(&mon);

  return ESP_OK;
}

// Tracks which view (decode list vs hero card) was drawn last, so switching
// between them gets a one-time full-screen force-redraw instead of visual
// bleed from whichever view was showing before.
static bool s_hero_was_active = false;

// Once a QSO/CQ goes active, the hero card locks on screen and stays there
// even after autoseq finishes and pops the context back to an empty queue --
// it only comes down when the user explicitly presses ESC (backtick). See
// render_rx_or_hero() (sets this true) and the RX-mode '`' handler (clears it).
static bool g_hero_locked = false;

// Snapshot of the last real (non-empty) hero info, so the frozen post-QSO
// display shows the completed exchange instead of a blank/zeroed context
// once autoseq pops the finished entry out of the active queue.
static QsoHeroInfo s_last_hero_info{};

static void build_qso_hero_info(QsoHeroInfo& info) {
  if (autoseq_active_count() == 0) {
    info = s_last_hero_info;
    // g_qso_done_active means this is the exact context that just legitimately
    // ended (see check_slot_boundary()) -- either a real sign-off (show fully
    // complete, all 6 stage boxes green) or a give-up (retries exhausted, no
    // RR73/73 ever heard -- leave the tracker frozen at whatever stage it
    // actually reached instead of falsely marking the whole thing green).
    info.qso_done = g_qso_done_active && !g_qso_done_gave_up;
    info.qso_gave_up = g_qso_done_active && g_qso_done_gave_up;
    if (info.qso_done) info.stage = 6;
    return;
  }
  QsoContext ctx{};
  autoseq_get_active_context(0, &ctx);
  // autoseq_start_cq() enqueues a self-CQ one-shot with the literal
  // placeholder dxcall "CQ" (see enqueue_one_shot in autoseq.cpp) — that's
  // not a real worked station, so detect it by that placeholder rather than
  // by an empty dxcall (which never actually happens for this context).
  info.calling_cq = (ctx.dxcall == "CQ");
  // While calling CQ there's no worked station yet -- leave dxcall/dxgrid
  // empty so ui_draw_qso_hero() falls back to "--" instead of showing our
  // own callsign, which read as if we'd already worked ourselves.
  info.dxcall = info.calling_cq ? "" : ctx.dxcall;
  info.dxgrid = info.calling_cq ? "" : ctx.dxgrid;
  // Only meaningful (and only guaranteed to match what's actually keyed out)
  // when g_cq_type is CQFREETEXT -- that's the only path the C prompt uses,
  // but guard it explicitly rather than assume, since g_cq_type can still be
  // set to something else via a stale saved config or the external core_api.
  info.cq_text = (info.calling_cq && g_cq_type == CqType::CQFREETEXT) ? g_cq_freetext : "";
  // AutoseqState CALLING..SIGNOFF (autoseq.h) maps 0..5 directly onto the
  // 6-stage tracker (CQ/GRID/RPT/R/RR73/73) — same order, same count.
  info.stage = (int)ctx.state;
  if (info.stage < 0) info.stage = 0;
  if (info.stage > 5) info.stage = 5;

  char fbuf[32];
  snprintf(fbuf, sizeof(fbuf), "%.3f  %s  G:%d", 0.001 * (double)g_bands[g_band_sel].freq,
           g_bands[g_band_sel].name, ic705_tx_get_gain_q8());
  info.freq_band = fbuf;
  info.snr = ctx.snr_tx;  // our measurement of their signal
  info.clock_hm = g_time.substr(0, 5);
  info.qso_count = (int)g_adif_sd_seq;
  info.qso_done = false;      // still mid-exchange, not the completion hold
  info.qso_gave_up = false;
  s_last_hero_info = info;
}

// Draws the decode list or the QSO hero card, whichever applies right now.
// Call sites: the two g_rx_dirty-gated spots in the main loop.
static void render_rx_or_hero() {
  // Lock the hero card on as soon as a QSO/CQ goes active. Deliberately does
  // NOT auto-clear when autoseq_active_count() drops back to 0 (QSO signed
  // off and got popped) -- only the RX-mode '`' (ESC) handler clears the lock,
  // so the completed exchange stays on screen for review until dismissed.
  if (autoseq_active_count() > 0) g_hero_locked = true;
  const bool hero_now = g_hero_locked;
  if (hero_now != s_hero_was_active) {
    if (!hero_now) {
      // Hero's status/clock row overlaps the waterfall + countdown bar's
      // screen real estate (both live in the top ~21px), and neither of
      // those get touched by ui_force_redraw_rx() (that only invalidates
      // the decode list's own diff cache) -- so without a full clear here,
      // stale hero pixels (e.g. "CALLING CQ" + the clock) linger at the top
      // until fresh waterfall data happens to paint over them. The decode
      // list, waterfall, and countdown bar all repaint themselves within
      // the next tick or two via their existing dirty-tracked redraws.
      M5.Display.fillScreen(TFT_BLACK);
      ui_force_redraw_rx();
    }
    s_hero_was_active = hero_now;
  }
  if (hero_now) {
    QsoHeroInfo info;
    build_qso_hero_info(info);
    ui_draw_qso_hero(info);
  } else {
    ui_draw_rx(rx_flash_idx);
  }
}

static void draw_band_view() {
  std::vector<std::string> lines;
  lines.reserve(g_bands.size());
  for (size_t i = 0; i < g_bands.size(); ++i) {
    std::string freq_str;
    if ((int)i == band_edit_idx && !band_edit_buffer.empty()) {
      freq_str = band_edit_buffer;
    } else {
      char fbuf[16];
      float f = g_bands[i].freq;
      if (f == (int)f) snprintf(fbuf, sizeof(fbuf), "%d", (int)f);
      else             snprintf(fbuf, sizeof(fbuf), "%.1f", f);
      freq_str = fbuf;
    }
    lines.push_back(std::string(g_bands[i].name) + ": " + freq_str);
  }
  ui_draw_list(lines, band_page, band_edit_idx);
}

static const char* offset_name(OffsetSrc o) {
  switch (o) {
    case OffsetSrc::RANDOM: return "Random";
    case OffsetSrc::CURSOR: return "Fixed";
    case OffsetSrc::RX: return "RX";
  }
  return "Random";
}

static RadioType canonical_radio_type(RadioType r) {
  if (r == RadioType::IC705) return r;
  return RadioType::IC705;  // default all unrecognised to IC-705
}

static bool radio_type_uses_display_only(RadioType r) {
  // Always use display-only board init (upstream design): audio input is owned
  // exclusively by the selected backend, so general M5Unified startup must
  // not claim speaker/mic/audio resources. The keyboard still works because
  // beginDisplayOnly() initializes it via Keyboard.begin() (auto-detects
  // board type) — see components/M5Cardputer/src/M5Cardputer.cpp.
  (void)r;
  return true;
}

static RadioProfileBinding get_radio_profile_binding(RadioType r) {
  switch (canonical_radio_type(r)) {
    case RadioType::IC705:
    default:
      return {AUDIO_SOURCE_IC705_WIFI, RADIO_CONTROL_IC705};
  }
}

static const char* radio_name(RadioType r) {
  switch (canonical_radio_type(r)) {
    case RadioType::IC705:    return "IC-705";
    default: break;
  }
  return "IC-705";
}

void apply_radio_profile_binding() {
  audio_source_backend_t prev_audio = audio_source_get_backend();
  g_radio = canonical_radio_type(g_radio);
  g_gps_baud = normalize_gps_baud_value(g_gps_baud);
  auto start_gps = [&]() {
    gps_start(gps_pins_for_current_source());
  };
  // cp705 is IC-705 only — always run GPS.
  start_gps();

  // For IC-705: pass the resolved IP and CI-V address to the CAT backend
  // before any ops can be called.
  if (canonical_radio_type(g_radio) == RadioType::IC705 && wifi_mgr_is_ready()) {
    ic705_net_set_credentials(g_ic705_net_user.c_str(), g_ic705_net_pass.c_str());
    ic705_cat_set_target(wifi_mgr_get_ic705_ip(), (uint8_t)g_ic705_civ_addr);
  }

  RadioProfileBinding binding = get_radio_profile_binding(g_radio);
  audio_source_set_backend(binding.audio_backend);
  radio_control_set_backend(binding.radio_backend);
  if (audio_source_is_streaming() && prev_audio != binding.audio_backend) {
    ESP_LOGW(TAG, "Audio backend changed while streaming; stop/start audio to apply (%s -> %s)",
             audio_source_backend_name(prev_audio),
             audio_source_backend_name(binding.audio_backend));
  }
  ESP_LOGI(TAG, "Profile bind radio=%s audio=%s control=%s",
           radio_name(g_radio),
           audio_source_backend_name(binding.audio_backend),
           radio_control_backend_name(binding.radio_backend));
}

static bool notify_radio_control_audio_start_if_allowed(const char* reason) {
  esp_err_t rc = radio_control_on_audio_start();
  const bool ok = (rc == ESP_OK);
  ESP_LOGI(TAG, "CAT audio start %s radio=%s reason=%s rc=%d",
           ok ? "ok" : "failed",
           radio_name(g_radio),
           reason ? reason : "",
           (int)rc);
  debug_log_line(ok ? "CAT audio ok" : "CAT audio fail");
  return ok;
}

static std::string lat_lon_to_maidenhead8(double lat, double lon) {
  if (lon < -180.0 || lon > 180.0 || lat < -90.0 || lat > 90.0) return "";
  // Clamp exact upper edge so index math stays in range.
  if (lon >= 180.0) lon = 179.999999;
  if (lat >= 90.0) lat = 89.999999;

  lon += 180.0;
  lat += 90.0;

  int field_lon = (int)(lon / 20.0);
  int field_lat = (int)(lat / 10.0);
  lon -= field_lon * 20.0;
  lat -= field_lat * 10.0;

  int square_lon = (int)(lon / 2.0);
  int square_lat = (int)(lat / 1.0);
  lon -= square_lon * 2.0;
  lat -= square_lat * 1.0;

  const double sub_lon_w = 2.0 / 24.0;
  const double sub_lat_h = 1.0 / 24.0;
  int sub_lon = (int)(lon / sub_lon_w);
  int sub_lat = (int)(lat / sub_lat_h);
  lon -= sub_lon * sub_lon_w;
  lat -= sub_lat * sub_lat_h;

  const double ext_lon_w = sub_lon_w / 10.0;
  const double ext_lat_h = sub_lat_h / 10.0;
  int ext_lon = (int)(lon / ext_lon_w);
  int ext_lat = (int)(lat / ext_lat_h);

  field_lon = std::clamp(field_lon, 0, 17);
  field_lat = std::clamp(field_lat, 0, 17);
  square_lon = std::clamp(square_lon, 0, 9);
  square_lat = std::clamp(square_lat, 0, 9);
  sub_lon = std::clamp(sub_lon, 0, 23);
  sub_lat = std::clamp(sub_lat, 0, 23);
  ext_lon = std::clamp(ext_lon, 0, 9);
  ext_lat = std::clamp(ext_lat, 0, 9);

  std::string out = "AA00aa00";
  out[0] = (char)('A' + field_lon);
  out[1] = (char)('A' + field_lat);
  out[2] = (char)('0' + square_lon);
  out[3] = (char)('0' + square_lat);
  out[4] = (char)('a' + sub_lon);
  out[5] = (char)('a' + sub_lat);
  out[6] = (char)('0' + ext_lon);
  out[7] = (char)('0' + ext_lat);
  return out;
}

static void draw_gps_view(bool force_redraw = false);

static void gps_runtime_tick() {
  static int64_t s_last_apply_ms = 0;
  static bool s_time_synced_once = false;
  static int s_last_time_sync_hour_key = -1;

  gps_tick();

  int detected_baud = 0;
  if (gps_take_baud_update(&detected_baud)) {
    detected_baud = normalize_gps_baud_value(detected_baud);
    if (detected_baud != g_gps_baud) {
      g_gps_baud = detected_baud;
      save_station_data();
      ESP_LOGI(TAG, "GPS baud persisted: %d", g_gps_baud);
    }
  }

  const int64_t now = rtc_now_ms();
  if ((now - s_last_apply_ms) < 1000) return;
  s_last_apply_ms = now;

  if (ui_mode == UIMode::GPS) {
    draw_gps_view();
  }

  gps_state_t st = gps_get_state();
  if (!st.valid_fix) return;

  bool changed = false;
  if (!st.grid_square.empty() && st.grid_square != "    ") {
    const std::string gps_grid = normalize_grid_maidenhead(st.grid_square);
    if (!gps_grid.empty()) {
      const std::string grid8 = lat_lon_to_maidenhead8(st.latitude, st.longitude);
      if (!grid8.empty()) {
        g_grid_gps_display8 = grid8;
      }
      g_grid_from_gps = true;
      if (gps_grid != g_grid) {
        g_grid = gps_grid;
        autoseq_set_station(g_call, grid_ft8_4(g_grid));
        changed = true;
        ESP_LOGI(TAG, "GPS grid synced: %s", g_grid.c_str());
      }
    }
  }

  if (!st.date_utc.empty() && !st.time_utc.empty()) {
    int y = 0, M = 0, d = 0;
    int h = 0, m = 0, s = 0;
    const bool parsed_date = (sscanf(st.date_utc.c_str(), "%d-%d-%d", &y, &M, &d) == 3);
    const bool parsed_time = (sscanf(st.time_utc.c_str(), "%d:%d:%d", &h, &m, &s) == 3);
    int hour_key = -1;
    if (parsed_date && parsed_time) {
      hour_key = (((y * 100) + M) * 100 + d) * 100 + h;
    }

    bool do_time_sync = !s_time_synced_once;
    if (!do_time_sync && parsed_time && !g_tx_active && !g_decode_in_progress) {
      if (m == 0 && s <= 5 && hour_key >= 0 && hour_key != s_last_time_sync_hour_key) {
        do_time_sync = true;
      }
    }

    if (do_time_sync) {
      const std::string old_date = g_date;
      const std::string old_time = g_time;
      g_date = st.date_utc;
      g_time = st.time_utc;
      if (rtc_set_from_strings_source(RtcTimeSource::GPS)) {
        rtc_sync_to_esp_rtc();
        (void)rtc_write_external_from_soft("GPS");
        s_time_synced_once = true;
        g_time_synced_from_gps = true;
        if (hour_key >= 0) s_last_time_sync_hour_key = hour_key;
        changed = true;
        ESP_LOGI(TAG, "GPS time synced: %s %s", g_date.c_str(), g_time.c_str());
        radio_control_set_time(h, m, s);
      } else {
        g_date = old_date;
        g_time = old_time;
      }
    }
  }

  // One session breadcrumb is enough to preserve the GPS grid even if no QSO
  // completes; retry later if logging is disabled or the file write fails.
  if (changed) {
    save_station_data();
  }
}


static std::string normalize_time_hms(const std::string& src) {
  int h = 0, m = 0, s = 0;
  if (sscanf(src.c_str(), "%d:%d:%d", &h, &m, &s) == 3) {
    if (h >= 0 && h <= 23 && m >= 0 && m <= 59 && s >= 0 && s <= 59) {
      char out[16];
      snprintf(out, sizeof(out), "%02d:%02d:%02d", h, m, s);
      return out;
    }
  }

  std::string digits;
  digits.reserve(src.size());
  for (unsigned char ch : src) {
    if (std::isdigit(ch)) digits.push_back((char)ch);
  }
  if (digits.size() >= 6) {
    h = (digits[0] - '0') * 10 + (digits[1] - '0');
    m = (digits[2] - '0') * 10 + (digits[3] - '0');
    s = (digits[4] - '0') * 10 + (digits[5] - '0');
    if (h >= 0 && h <= 23 && m >= 0 && m <= 59 && s >= 0 && s <= 59) {
      char out[16];
      snprintf(out, sizeof(out), "%02d:%02d:%02d", h, m, s);
      return out;
    }
  }
  return src;
}

static int normalize_gps_baud_value(int value) {
  return (value == 9600 || value == 115200) ? value : 115200;
}

static gps_pins_t gps_pins_for_current_source() {
  // The LoRa-1262 cap's GNSS is the only supported GPS source now (PORTA
  // wiring removed) — always UART2/G15/G13 at a fixed 115200 baud.
  gps_pins_t pins = {};
  pins.uart = UART_NUM_2;
  pins.rx = GPIO_NUM_15;
  pins.tx = GPIO_NUM_13;
  pins.default_baud = 115200;
  pins.auto_baud = false;
  return pins;
}

static const char* gps_source_name() {
  return "GNSS_LoRa";
}

static std::string normalize_date_ymd(const std::string& src) {
  auto date_in_range = [](int y, int M, int d) -> bool {
    return (y >= 2024 && y <= 2099 && M >= 1 && M <= 12 && d >= 1 && d <= 31);
  };

  int y = 0, M = 0, d = 0;
  if (sscanf(src.c_str(), "%d-%d-%d", &y, &M, &d) == 3 && date_in_range(y, M, d)) {
    char out[16];
    snprintf(out, sizeof(out), "%04d-%02d-%02d", y, M, d);
    return out;
  }

  std::string digits;
  digits.reserve(src.size());
  for (unsigned char ch : src) {
    if (std::isdigit(ch)) digits.push_back((char)ch);
  }
  if (digits.size() >= 8) {
    y = (digits[0] - '0') * 1000 + (digits[1] - '0') * 100 +
        (digits[2] - '0') * 10 + (digits[3] - '0');
    M = (digits[4] - '0') * 10 + (digits[5] - '0');
    d = (digits[6] - '0') * 10 + (digits[7] - '0');
    if (date_in_range(y, M, d)) {
      char out[16];
      snprintf(out, sizeof(out), "%04d-%02d-%02d", y, M, d);
      return out;
    }
  }

  return "";
}

static std::string normalize_grid_maidenhead(const std::string& src) {
  size_t b = 0;
  size_t e = src.size();
  while (b < e && std::isspace(static_cast<unsigned char>(src[b]))) ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(src[e - 1]))) --e;

  const size_t n = e - b;
  if (n != 4 && n != 6 && n != 8) return "";

  std::string out = src.substr(b, n);
  auto is_digit_char = [](char ch) { return ch >= '0' && ch <= '9'; };
  auto to_upper = [](char ch) { return static_cast<char>(std::toupper(static_cast<unsigned char>(ch))); };
  auto to_lower = [](char ch) { return static_cast<char>(std::tolower(static_cast<unsigned char>(ch))); };

  char c0 = to_upper(out[0]);
  char c1 = to_upper(out[1]);
  if (c0 < 'A' || c0 > 'R' || c1 < 'A' || c1 > 'R') return "";
  if (!is_digit_char(out[2]) || !is_digit_char(out[3])) return "";
  out[0] = c0;
  out[1] = c1;

  if (n >= 6) {
    char c4 = to_upper(out[4]);
    char c5 = to_upper(out[5]);
    if (c4 < 'A' || c4 > 'X' || c5 < 'A' || c5 > 'X') return "";
    out[4] = to_lower(c4);
    out[5] = to_lower(c5);
  }

  if (n == 8) {
    if (!is_digit_char(out[6]) || !is_digit_char(out[7])) return "";
  }

  return out;
}

std::string grid_ft8_4(const std::string& grid) {
  const std::string norm = normalize_grid_maidenhead(grid);
  if (norm.size() >= 4) return norm.substr(0, 4);
  return "CM97";
}

static std::string menu_sleep_batt_line() {
  board_power_status_t ps = {};
  char buf[48];

  if (board_power_read(&ps) == ESP_OK && ps.valid) {
    snprintf(buf, sizeof(buf), "Sleep/Batt %d%%", ps.percent);
  } else {
    snprintf(buf, sizeof(buf), "Sleep/Batt --");
  }

  return std::string(buf);
}

static std::string elide_right(const std::string& s, size_t max_len = 22) {
  if (s.size() <= max_len) return s;
  if (max_len <= 3) return s.substr(s.size() - max_len);
  return std::string("...") + s.substr(s.size() - (max_len - 3));
}

static std::string head_trim(const std::string& s, size_t max_len = 16) {
  if (s.size() <= max_len) return s;
  if (max_len == 0) return "";
  if (max_len == 1) return ">";
  return s.substr(0, max_len - 1) + ">";
}

static std::string highlight_pos(const std::string& s, int pos) {
  if (pos < 0 || pos >= (int)s.size()) return s;
  std::string out;
  out.reserve(s.size() + 2);
  out.append(s, 0, pos);
  out.push_back('[');
  out.push_back(s[pos]);
  out.push_back(']');
  out.append(s, pos + 1, std::string::npos);
  return out;
}

static void draw_status_view();

static const char* rtc_time_source_suffix() {
  switch (g_rtc_time_source) {
    case RtcTimeSource::DS3231: return " R";
    case RtcTimeSource::GPS: return " G";
    case RtcTimeSource::SAVED:
    case RtcTimeSource::ESP_RTC:
    case RtcTimeSource::MANUAL:
    default:
      return "";
  }
}

static void rtc_update_strings_from_epoch(time_t now) {
  struct tm t;
  localtime_r(&now, &t);
  char buf_date[32];
  snprintf(buf_date, sizeof(buf_date), "%04d-%02d-%02d",
           t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
  g_date = buf_date;
  char buf_time[16];
  snprintf(buf_time, sizeof(buf_time), "%02d:%02d:%02d",
           t.tm_hour, t.tm_min, t.tm_sec);
  g_time = buf_time;
}

static time_t rtc_current_epoch_seconds() {
  if (!rtc_valid) {
    return (time_t)(esp_timer_get_time() / 1000000);
  }
  return rtc_epoch_base + (esp_timer_get_time() / 1000 - rtc_ms_start) / 1000;
}

static void rtc_seed_epoch(time_t epoch, int64_t ms_start, RtcTimeSource source) {
  rtc_epoch_base = epoch;
  rtc_ms_start = ms_start;
  rtc_last_update = ms_start;
  rtc_valid = true;
  g_rtc_time_source = source;
  rtc_update_strings_from_epoch(epoch);
}

static bool rtc_set_external_datetime_strings(const external_rtc_datetime_t& datetime) {
  char buf_date[32];
  char buf_time[16];
  snprintf(buf_date, sizeof(buf_date), "%04u-%02u-%02u",
           (unsigned)datetime.year,
           (unsigned)datetime.month,
           (unsigned)datetime.day);
  snprintf(buf_time, sizeof(buf_time), "%02u:%02u:%02u",
           (unsigned)datetime.hour,
           (unsigned)datetime.minute,
           (unsigned)datetime.second);
  g_date = buf_date;
  g_time = buf_time;
  return true;
}

static external_rtc_datetime_t rtc_external_datetime_from_soft() {
  time_t now = rtc_current_epoch_seconds();
  struct tm t;
  localtime_r(&now, &t);

  external_rtc_datetime_t datetime = {};
  datetime.year = (uint16_t)(t.tm_year + 1900);
  datetime.month = (uint8_t)(t.tm_mon + 1);
  datetime.day = (uint8_t)t.tm_mday;
  datetime.hour = (uint8_t)t.tm_hour;
  datetime.minute = (uint8_t)t.tm_min;
  datetime.second = (uint8_t)t.tm_sec;
  return datetime;
}

static bool rtc_set_from_strings_source(RtcTimeSource source) {
  int y, M, d, h, m, s;
  if (sscanf(g_date.c_str(), "%d-%d-%d", &y, &M, &d) != 3) return false;
  if (sscanf(g_time.c_str(), "%d:%d:%d", &h, &m, &s) != 3) return false;
  struct tm t = {};
  t.tm_year = y - 1900;
  t.tm_mon = M - 1;
  t.tm_mday = d;
  t.tm_hour = h;
  t.tm_min = m;
  t.tm_sec = s;
  time_t epoch = mktime(&t);
  if (epoch == (time_t)-1) return false;
  rtc_seed_epoch(epoch, esp_timer_get_time() / 1000, source);
  return true;
}

bool rtc_set_from_strings() {
  return rtc_set_from_strings_source(RtcTimeSource::SAVED);
}

void rtc_sync_to_esp_rtc() {
  if (!rtc_valid) return;

  time_t now = rtc_current_epoch_seconds();
  struct timeval tv = { .tv_sec = now, .tv_usec = 0 };
  settimeofday(&tv, NULL);
  ESP_LOGI(TAG, "ESP RTC synced from soft RTC");
}

static esp_err_t rtc_write_external_from_soft(const char* reason) {
  if (!rtc_valid || !external_rtc_available()) {
    return ESP_ERR_INVALID_STATE;
  }

  external_rtc_datetime_t datetime = rtc_external_datetime_from_soft();
  esp_err_t err = external_rtc_write_datetime(&datetime);
  if (err == ESP_OK) {
    ESP_LOGI(TAG,
             "DS3231 time updated from %s: %04u-%02u-%02u %02u:%02u:%02u",
             reason ? reason : "soft RTC",
             (unsigned)datetime.year,
             (unsigned)datetime.month,
             (unsigned)datetime.day,
             (unsigned)datetime.hour,
             (unsigned)datetime.minute,
             (unsigned)datetime.second);
  } else {
    ESP_LOGW(TAG,
             "DS3231 time update from %s failed: %s",
             reason ? reason : "soft RTC",
             esp_err_to_name(err));
  }
  return err;
}

bool rtc_apply_manual_time_from_strings() {
  if (!rtc_set_from_strings_source(RtcTimeSource::MANUAL)) {
    return false;
  }

  g_time_synced_from_gps = false;
  rtc_sync_to_esp_rtc();
  if (rtc_write_external_from_soft("manual time") == ESP_OK) {
    g_rtc_time_source = RtcTimeSource::DS3231;
  }
  return true;
}

static bool rtc_init_from_ds3231() {
  esp_err_t err = external_rtc_init();
  if (err != ESP_OK) {
    return false;
  }

  external_rtc_datetime_t datetime = {};
  err = external_rtc_read_datetime(&datetime);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "DS3231 time not loaded; using ESP RTC or saved time: %s",
             esp_err_to_name(err));
    return false;
  }

  rtc_set_external_datetime_strings(datetime);
  if (!rtc_set_from_strings_source(RtcTimeSource::DS3231)) {
    ESP_LOGW(TAG, "DS3231 time parse failed; using ESP RTC or saved time");
    return false;
  }

  g_time_synced_from_gps = false;
  g_rtc_sleep_epoch = 0;
  rtc_sync_to_esp_rtc();
  ESP_LOGI(TAG,
           "DS3231 time loaded: %04u-%02u-%02u %02u:%02u:%02u",
           (unsigned)datetime.year,
           (unsigned)datetime.month,
           (unsigned)datetime.day,
           (unsigned)datetime.hour,
           (unsigned)datetime.minute,
           (unsigned)datetime.second);
  return true;
}

// Initialize soft RTC from ESP RTC (persists through deep sleep)
// Applies compensation if we have valid sleep epoch data
static bool rtc_init_from_esp_rtc() {
  struct timeval tv;
  if (gettimeofday(&tv, NULL) != 0) return false;

  // Check if ESP RTC has valid time (year > 2020)
  struct tm t;
  localtime_r(&tv.tv_sec, &t);
  if (t.tm_year + 1900 < 2020) return false;

  time_t compensated_now = tv.tv_sec;

  // Apply compensation if we have valid sleep data
  if (g_rtc_sleep_epoch > 0 && tv.tv_sec > g_rtc_sleep_epoch) {
    int64_t raw_elapsed = tv.tv_sec - g_rtc_sleep_epoch;
    int64_t actual_elapsed = raw_elapsed;

    // Apply compensation: actual = raw * 10000 / (10000 + comp)
    if (g_rtc_comp != 0) {
      actual_elapsed = raw_elapsed * 10000 / (10000 + g_rtc_comp);
    }

    // Fixed 1s boot delay: deep sleep entry → wake → gettimeofday
    static constexpr int64_t BOOT_DELAY_SEC = 1;
    compensated_now = g_rtc_sleep_epoch + actual_elapsed + BOOT_DELAY_SEC;

    ESP_LOGI(TAG, "RTC wake: raw_elapsed=%lld, actual_elapsed=%lld, comp=%d, boot_adj=%lld",
             (long long)raw_elapsed, (long long)actual_elapsed, g_rtc_comp,
             (long long)BOOT_DELAY_SEC);

    // Clear sleep epoch after use (one-time compensation)
    g_rtc_sleep_epoch = 0;
  }

  // Account for sub-second offset: tv.tv_usec tells us how far past the
  // whole second we are, so rewind rtc_ms_start by that amount.
  rtc_seed_epoch(compensated_now,
                 esp_timer_get_time() / 1000 - tv.tv_usec / 1000,
                 RtcTimeSource::ESP_RTC);

  g_time_synced_from_gps = false;
  ESP_LOGI(TAG, "ESP RTC initialized: %s %s (compensated=%s)",
           g_date.c_str(), g_time.c_str(),
           (g_rtc_comp != 0) ? "yes" : "no");
  return true;
}

static void rtc_update_strings() {
  if (!rtc_valid) return;
  rtc_update_strings_from_epoch(rtc_current_epoch_seconds());
}

int64_t rtc_now_ms() {
  if (!rtc_valid) {
    return esp_timer_get_time() / 1000;
  }
  return (int64_t)rtc_epoch_base * 1000 + (esp_timer_get_time() / 1000 - rtc_ms_start);
}

static void rtc_tick() {
  if (!rtc_valid) {
    rtc_set_from_strings();
    if (!rtc_valid) return;
  }
  int64_t now_ms = esp_timer_get_time() / 1000;
  if (now_ms - rtc_last_update >= 1000) {
    rtc_last_update += 1000; // Increment by interval to prevent drift accumulation
    if (status_edit_idx != 4) { // keep time ticking unless editing time
      std::string old_date = g_date;
      std::string old_time = g_time;
      rtc_update_strings();
      // Keep the clock strings current in memory, but do NOT touch the LCD
      // while transmitting/tuning. An LCD SPI/DMA write on core 0 stalls the
      // outgoing audio stream and pulses the carrier ~1x/sec (this is the
      // tune-mode twin of the countdown-redraw 1Hz pulse — tune is toggled
      // from the STATUS view, so this once-per-second clock redraw was the
      // remaining ungated screen write firing the whole time tune is on).
      if (ui_mode == UIMode::STATUS && status_edit_idx == -1 &&
          !(g_tx_active || g_tune)) {
        if (old_date != g_date) {
          draw_status_line(3, std::string("Date: ") + g_date, false);
        }
        if (old_time != g_time) {
          draw_status_line(4, std::string("Time: ") + g_time + rtc_time_source_suffix(), false);
        }
      }
    }
  }
}

// Push current in-memory band to the radio, ensuring it's in RX mode.
// Called from: QMX first-connect path (consume_cdc_initial_sync), STATUS
// exit (enter_mode), and S->3 band-change key handler. Guards:
//   - radio_control_ready(): CAT link must be up
//   - !g_tx_active: never interrupt an ongoing transmission
// Returns true on success. Callers can use the return value to decide
// whether to clear a deferred-sync flag.
// The `reason` string is logged for debugging.
bool sync_radio_to_current_band(const char* reason) {
  if (!radio_control_ready()) return false;
  if (g_tx_active) return false;
  int freq_hz = (int)(g_bands[g_band_sel].freq * 1000.0f);
  radio_control_end_tx();  // ensure RX mode (idempotent)
  esp_err_t rc = radio_control_sync_frequency_mode(freq_hz);
  if (rc == ESP_OK) {
    ESP_LOGI(TAG, "CAT sync ok (%s) freq=%d", reason ? reason : "", freq_hz);
    char msg[80];
    snprintf(msg, sizeof(msg), "CAT sync: %s", reason ? reason : "");
    debug_log_line(msg);
    return true;
  }
  ESP_LOGW(TAG, "CAT sync failed (%s) rc=%d", reason ? reason : "", (int)rc);
  return false;
}

// Consume the "CDC initial sync pending" flag set by stream_uac_task after
// a successful QMX CDC-ACM open. Runs the same sync sequence as the manual
// STATUS->2 button (put radio in RX + push current band to VFO), so users
// don't have to press anything after plugging in QMX. Called from the main
// loop every iteration (before early-exit branches). Fires at most once
// per CDC open — cleared on successful sync, retries on later iterations
// until CAT becomes ready and we're not TXing.
static void consume_cdc_initial_sync() {
  if (!g_ic705_initial_sync_pending) return;
  if (sync_radio_to_current_band("initial IC-705 connect")) {
    g_ic705_initial_sync_pending = false;
  }
}

static void update_countdown() {
  // While transmitting (FT8 TX or tune), do NOT redraw — an LCD SPI/DMA burst
  // contends with the WiFi DMA on the S3 and momentarily stalls the outgoing
  // audio stream, which underruns the radio's TX buffer and pulses the carrier
  // ~1x/sec (this redraw fires once per second on the seconds change). The
  // screen simply holds blank during TX (see ui_clear_countdown() in
  // tx_start()); we're transmitting, not reading the display.
  //
  // holdSlotIdx: our own TX only occupies the first ~12.6s of its 15s slot,
  // leaving a ~2.4s RX tail before the NEXT TX (in a running QSO) blanks it
  // again. Resuming the instant TX ends would draw the bar at its true
  // (already ~84% full) position for that brief tail, then blank again a
  // couple seconds later -- reading as a flash, not useful information. Hold
  // blank through the rest of the CURRENT slot instead, so the bar only ever
  // resumes at a fresh slot boundary, starting from 0% -- never a mid-slot
  // jump. Dean: "during tx, I don't want to see a progress bar at all ever,
  // not even a tiny part of one. I only want to see it during the RX cycle."
  static int64_t s_hold_slot_idx = -1;
  const int slot_period = g_protocol->slot_time_ms;
  if (g_tx_active || g_tune) {
    s_hold_slot_idx = rtc_now_ms() / slot_period;
    return;
  }
  int64_t now_ms = rtc_now_ms();
  int64_t slot_idx = now_ms / slot_period;
  if (slot_idx <= s_hold_slot_idx) return;  // still in this TX slot's RX tail -- stay blank
  int64_t slot_ms = now_ms % slot_period;
  static int64_t last_slot_idx = -1;
  static int last_sec = -1;
  int sec = (int)(slot_ms / 1000);
  if (slot_idx != last_slot_idx || sec != last_sec) {
    float frac = (float)slot_ms / (float)slot_period;
    bool even = (slot_idx % 2) == 0;
    ui_draw_countdown(frac, even);
    last_slot_idx = slot_idx;
    last_sec = sec;
  }
}

static void redraw_countdown_now() {
  int64_t now_ms = rtc_now_ms();
  const int slot_period = g_protocol->slot_time_ms;
  int64_t slot_idx = now_ms / slot_period;
  int64_t slot_ms = now_ms % slot_period;
  float frac = (float)slot_ms / (float)slot_period;
  bool even = (slot_idx % 2) == 0;
  ui_draw_countdown(frac, even);
}

// Mirrors update_countdown()'s hold-until-next-slot pattern for the
// hero-card/RX-list redraw. tx_tick() clears g_tx_active the instant TX
// completes, within the SAME loop iteration that later checks "is it safe
// to redraw" -- so redrawing right then reads as a flash exactly when TX
// stops, then a second one moments later at the real slot boundary. Hold
// through the rest of that TX slot's tail (same ~2.4s window the countdown
// bar holds through) so the hero card only ever refreshes once, cleanly, at
// the start of the next slot. Must be called unconditionally every loop
// iteration (not just when !g_tx_active) so it can actually latch the hold
// while TX is still active.
static bool rx_redraw_should_hold() {
  static int64_t s_hold_slot_idx = -1;
  const int slot_period = g_protocol->slot_time_ms;
  int64_t now_ms = rtc_now_ms();
  int64_t slot_idx = now_ms / slot_period;
  if (g_tx_active || g_tune) {
    s_hold_slot_idx = slot_idx;
    return true;
  }
  return slot_idx <= s_hold_slot_idx;
}

// Forward declarations for single-threaded TX state machine
static void tx_start(int skip_tones);
static void tx_tick();

// Slot boundary check - called from main loop
// Matches reference project: tick after TX slot ends, TX trigger at slot start
// Compute the actual audio offset the next TX will use, given the
// configured g_offset_src and the autoseq pending entry. Storing the
// resolved value at scheduling time (rather than at the slot boundary,
// as the firmware used to do) means UI consumers reading core_get_qso see
// the same number that will actually go on air — important for the
// waterfall offset marker, especially in RANDOM / beacon-CQ modes where
// the random was previously rolled inside check_slot_boundary.
static int resolve_tx_offset(const AutoseqTxEntry& e) {
  if (g_offset_src == OffsetSrc::CURSOR) {
    return g_offset_hz;
  }
  if (g_offset_src == OffsetSrc::RX &&
      e.offset_hz > 0 &&
      e.text.rfind("CQ ", 0) != 0) {
    return e.offset_hz;
  }
  // RANDOM, or RX mode + CQ: roll a fresh offset in [500, 2500] Hz.
  return 500 + (int)(esp_random() % 2001);
}

// Single point of truth for arming the next TX. Replaces the 4-line
// "g_qso_xmit / g_target_slot_parity / g_pending_tx / g_pending_tx_valid"
// block that used to be repeated at every scheduling site (autoseq tick,
// beacon-on, free-text queue, and RX selection).
void arm_pending_tx(const AutoseqTxEntry& pending) {
  g_qso_xmit           = true;
  g_target_slot_parity = pending.slot_id & 1;
  g_pending_tx         = pending;
  g_pending_tx.offset_hz = resolve_tx_offset(g_pending_tx);
  g_pending_tx_valid   = true;
}

static void check_slot_boundary() {
  int64_t now_ms = rtc_now_ms();
  const int slot_period = g_protocol->slot_time_ms;
  int64_t slot_idx = now_ms / slot_period;
  int slot_ms = (int)(now_ms % slot_period);
  int slot_parity = (int)(slot_idx & 1);

  // Detect slot boundary (parity change)
  if (slot_parity != g_last_slot_parity) {
    g_last_slot_parity = slot_parity;
  }

  // Call tick AFTER TX has completed (not while TX is still active)
  // This ensures autoseq_tick() operates on the correct completed TX entry
  if (g_was_txing && !g_tx_active) {
    ESP_LOGI(TAG, "TX completed, calling tick (slot %lld, parity %d)",
             (long long)slot_idx, slot_parity);
    // Snapshot state BEFORE the tick: if we were signing off (just finished
    // sending TX5/73 -- already logged, in autoseq_on_tx_starting(), the
    // instant that TX began), and the tick below evicts it from the active
    // queue (into inactive-parked-for-late-RR73, or straight to gone -- both
    // read the same to the operator: the exchange is over), this is a real,
    // logged QSO completion. Only reachable here for genuine completions:
    // a cancelled TX clears g_was_txing directly and never reaches this branch.
    //
    // Same hold also covers the OTHER way an exchange ends: retries exhausted
    // mid-QSO with no RR73/73 ever heard back (autoseq.cpp's retry_counter >=
    // retry_limit branch, evicted the same tick). Dean: "if something is done
    // or not working, there's no reason to wait" -- a failed contact shouldn't
    // freeze the hero card forever any more than a completed one should.
    QsoContext pre_tick_ctx{};
    const bool had_active_ctx = autoseq_get_active_context(0, &pre_tick_ctx);
    autoseq_tick(slot_idx, slot_parity, 0);
    if (had_active_ctx && autoseq_active_count() == 0) {
      const bool was_signoff = (pre_tick_ctx.state == AutoseqState::SIGNOFF);
      const bool was_mid_exchange_giveup =
          (pre_tick_ctx.state == AutoseqState::REPLYING ||
           pre_tick_ctx.state == AutoseqState::REPORT ||
           pre_tick_ctx.state == AutoseqState::ROGER_REPORT ||
           pre_tick_ctx.state == AutoseqState::ROGERS) &&
          (pre_tick_ctx.retry_counter >= pre_tick_ctx.retry_limit);
      if (was_signoff || was_mid_exchange_giveup) {
        g_qso_done_active = true;
        g_qso_done_since_ms = rtc_now_ms();
        g_qso_done_was_cq_running = g_cq_running;
        g_qso_done_gave_up = was_mid_exchange_giveup;
        g_cq_running = false;  // suspend auto-repeat until the hold elapses
        g_rx_dirty = true;
      }
    }
    g_was_txing = false;
    core_fire_qso_changed();  // propagates to all registered consumers
  }

  // TX trigger: check if we should start TX in this slot
  // Conditions: qso_xmit flag set, correct parity, early enough in slot, not already TXing,
  // and decode must be complete (TX is always triggered by decode results).
  // Additional guard (g_decode_applied_slot_idx): enforces that decode for the
  // previous RX slot (slot_idx - 1) has been fully applied to autoseq state before
  // we fire TX. Without this, a slot boundary that arrives before audio capture
  // has completed (FT8: ~12.6s of 15s slot; FT4: ~5.0s of 7.5s slot) could fire
  // TX based on a prior cycle's state. See AUTOSEQ_INACTIVE_QUEUE.md.
  // Window = 4/15 of slot_time_ms (~26.7%): FT8=4000ms, FT4=2000ms.
  const bool parity_ok = (g_target_slot_parity == slot_parity);
  const bool window_ok = (slot_ms < (g_protocol->slot_time_ms * 4 / 15));
  const bool decode_ok = (g_decode_applied_slot_idx >= slot_idx - 1);
  if (g_qso_xmit &&
      parity_ok &&
      window_ok &&
      !g_tx_active &&
      !g_decode_in_progress &&
      decode_ok) {

    ESP_LOGI(TAG, "TX trigger: starting TX in slot %lld (parity %d)",
             (long long)slot_idx, slot_parity);

    // Calculate skip_tones for partial slot
    const int sym_ms = (int)roundf(g_protocol->symbol_period * 1000.0f);
    int skip_tones = slot_ms / sym_ms;
    if (skip_tones < g_protocol->total_symbols) {
      // Only proceed if we have a valid pending TX
      // NOTE: Don't clear g_qso_xmit until we're sure g_pending_tx is valid.
      // This avoids a race condition where decode_monitor_results is still
      // writing g_pending_tx on core 1 while we read it on core 0.
      if (g_pending_tx_valid && !g_pending_tx.text.empty()) {
        g_qso_xmit = false;  // Clear flag only AFTER validation succeeds
        g_was_txing = true;  // Set IMMEDIATELY when TX starts (prevents decode_monitor_results from re-setting flags)

        tx_start(skip_tones);
      }
    }
  } else if (g_qso_xmit) {
    // A TX is armed (g_qso_xmit) but didn't fire this slot -- throttled to
    // once/sec so a real stall (armed forever, never triggers) leaves a
    // trace showing exactly which guard is blocking it, instead of the
    // silence that made the retry-exhaustion stall so hard to diagnose live.
    static int64_t s_last_xmit_stall_log_ms = 0;
    if (now_ms - s_last_xmit_stall_log_ms >= 1000) {
      s_last_xmit_stall_log_ms = now_ms;
      ESP_LOGW(TAG, "TX armed but not firing: parity_ok=%d(target=%d,slot=%d) "
               "window_ok=%d(slot_ms=%d) tx_active=%d decode_in_progress=%d "
               "decode_ok=%d(applied=%lld,slot_idx=%lld)",
               (int)parity_ok, g_target_slot_parity, slot_parity,
               (int)window_ok, slot_ms, (int)g_tx_active, (int)g_decode_in_progress,
               (int)decode_ok, (long long)g_decode_applied_slot_idx, (long long)slot_idx);
    }
  }
}

  static void menu_flash_tick() {
    if (menu_flash_idx < 0) return;
    int64_t now = rtc_now_ms();
    if (now >= menu_flash_deadline) {
      menu_flash_idx = -1;
      if (ui_mode == UIMode::MENU && !menu_long_edit && menu_edit_idx < 0) {
        draw_menu_view();
      }
  }
}

static void rx_flash_tick() {
  if (rx_flash_idx < 0) return;
  int64_t now = rtc_now_ms();
  if (now >= rx_flash_deadline) {
    rx_flash_idx = -1;
    rx_flash_deadline = 0;
    // Hold like every other redraw here: one of this function's two call
    // sites has no TX/tune guard around it.
    if (ui_mode == UIMode::RX && !(g_tx_active || g_tune)) {
      // Route through the hero-aware dispatcher, not the plain list directly.
      // Answering a CQ locks the hero card on (g_hero_locked) well before
      // this 500ms flash-highlight timer expires -- calling ui_draw_rx()
      // unconditionally here repainted the plain list right over the hero
      // card that a decode-driven redraw had already correctly shown,
      // and it stuck until the next decode cycle fixed it.
      render_rx_or_hero();
    }
  }
}

// Brief automatic tune burst (not a manual toggle): key PTT + stream a
// short tone, then unkey and resume decode on its own after kTuneAutoStopMs,
// no second keypress required. Called unconditionally every main-loop pass
// so it fires even while other UI ticks are held off during TX/tune.
static void tune_tick() {
  if (!g_tune) return;
  int64_t now = rtc_now_ms();
  if (now < g_tune_stop_at_ms) return;
  radio_control_set_tune(false, 0, 0);
  g_tune = false;
  g_decode_enabled = true;
  debug_log_line("CAT tune: RX (auto)");
  if (ui_mode == UIMode::STATUS) draw_status_view();
}

// Auto-revert the Logging category's "Press 2 again: confirm" arm prompt
// (Clear QSO Log) and the "Log cleared" feedback back to normal once their
// window lapses, so a stale prompt doesn't linger without a keypress to
// refresh it.
static void qso_clear_tick() {
  if (ui_mode != UIMode::MENU || menu_category != kCatLogging || menu_long_edit) return;
  int64_t now = rtc_now_ms();
  bool arm_expired = g_q_clear_armed && now >= g_q_clear_arm_deadline;
  bool feedback_expired = !g_q_clear_feedback.empty() && now >= g_q_clear_feedback_deadline;
  if (arm_expired || feedback_expired) {
    if (arm_expired) g_q_clear_armed = false;
    if (feedback_expired) g_q_clear_feedback.clear();
    draw_menu_view();
  }
}

// Advance the "QSO COMPLETE" hero-card hold set by check_slot_boundary().
// Matches TD705's reference behavior: hold a few seconds, then either resume
// the running CQ (this contact came from our own CQ) or fall back to the
// plain RX list (we were answering someone else's).
static void qso_done_tick() {
  if (!g_qso_done_active) return;
  if (autoseq_active_count() > 0) {
    // Something else started during the hold (e.g. a fresh 'C' prompt) --
    // abandon the hold rather than stomp on whatever's active now.
    g_qso_done_active = false;
    g_qso_done_gave_up = false;
    return;
  }
  const int64_t hold_ms = g_qso_done_was_cq_running ? kQsoDoneResumeCqMs : kQsoDoneReturnRxMs;
  if (rtc_now_ms() - g_qso_done_since_ms < hold_ms) return;
  g_qso_done_active = false;
  g_qso_done_gave_up = false;
  if (g_qso_done_was_cq_running) {
    g_cq_running = true;
    enqueue_running_cq();
    AutoseqTxEntry pending;
    if (autoseq_fetch_pending_tx(pending)) arm_pending_tx(pending);
  } else {
    g_hero_locked = false;
  }
  g_rx_dirty = true;
}

static void apply_pending_sync() {}

static int band_number_from_name(const std::string& name) {
  int num = 0;
  for (char c : name) {
    if (c >= '0' && c <= '9') {
      num = num * 10 + (c - '0');
    } else {
      break;
    }
  }
  return num;
}

// Per-band TX gain: load whichever band g_band_sel lands on after a band
// change, or leave the gain untouched if that band has no saved value yet.
// Defined near the other NVS helpers further down; forward-declared here
// since rebuild_active_bands()/advance_active_band() are defined first.
static void apply_band_gain_for_current();

void rebuild_active_bands() {
  std::string cleaned = g_active_band_text;
  for (char& c : cleaned) {
    if (c == ',' || c == '/' || c == '\\' || c == ';') c = ' ';
    if (c == 'm' || c == 'M') c = ' ';
  }
  std::istringstream iss(cleaned);
  std::vector<int> bands;
  int v;
  while (iss >> v) {
    if (v <= 0) continue;
    // match to g_bands by number prefix
    for (size_t i = 0; i < g_bands.size(); ++i) {
      if (band_number_from_name(g_bands[i].name) == v) {
        if (std::find(bands.begin(), bands.end(), (int)i) == bands.end()) {
          bands.push_back((int)i);
        }
        break;
      }
    }
  }
  if (bands.empty()) {
    bands.resize(g_bands.size());
    for (size_t i = 0; i < g_bands.size(); ++i) bands[i] = (int)i;
  }
  g_active_band_indices = bands;
  if (std::find(g_active_band_indices.begin(), g_active_band_indices.end(), g_band_sel) == g_active_band_indices.end()) {
    g_band_sel = g_active_band_indices[0];
  }
  apply_band_gain_for_current();
  // normalize text
  std::ostringstream oss;
  for (size_t i = 0; i < g_active_band_indices.size(); ++i) {
    if (i) oss << ' ';
    oss << band_number_from_name(g_bands[g_active_band_indices[i]].name);
  }
  g_active_band_text = oss.str();
}

void update_autoseq_cq_type() {
  AutoseqCqType t = AutoseqCqType::CQ;
  switch (g_cq_type) {
    case CqType::CQSOTA: t = AutoseqCqType::SOTA; break;
    case CqType::CQPOTA: t = AutoseqCqType::POTA; break;
    case CqType::CQQRP:  t = AutoseqCqType::QRP;  break;
    case CqType::CQFD:   t = AutoseqCqType::FD;   break;
    case CqType::CQFREETEXT: t = AutoseqCqType::FREETEXT; break;
    default: t = AutoseqCqType::CQ; break;
  }
  // g_free_text is the Field Day EXCHANGE string (parsed by format_tx_text's
  // TX2/TX3 is_fd branch in autoseq.cpp) -- only CQFD wants that. CQFREETEXT
  // (the CQ prompt's typed message, incl. any POTA/SOTA/QRP prefix) must use
  // g_cq_freetext instead, since generate_cq_text_into()'s FREETEXT case
  // sends s_cq_freetext out verbatim as the actual CQ payload. Grouping
  // CQFREETEXT with CQFD here was a bug: every custom CQ typed via the C
  // prompt was silently transmitting g_free_text ("TNX 73" by default)
  // instead of what was actually typed.
  const std::string& ft =
    (g_cq_type == CqType::CQFD) ? g_free_text : g_cq_freetext;
  autoseq_set_cq_type(t, ft);
}

static void advance_active_band(int delta) {
  if (g_active_band_indices.empty()) rebuild_active_bands();
  if (g_active_band_indices.empty()) return;
  int pos = 0;
  for (size_t i = 0; i < g_active_band_indices.size(); ++i) {
    if (g_active_band_indices[i] == g_band_sel) { pos = (int)i; break; }
  }
  int n = (int)g_active_band_indices.size();
  pos = (pos + delta + n) % n;
  g_band_sel = g_active_band_indices[pos];
  apply_band_gain_for_current();
  // Decodes from the old band are no longer relevant -- the persistent
  // decode list (decode_monitor_results()) would otherwise keep showing them
  // mixed in with the new band's traffic indefinitely.
  clear_decode_list();
}

static int tx_waterfall_hz_to_x(float tone_hz) {
  constexpr int kScreenW = 240;
  constexpr float kMinHz = 200.0f;
  constexpr float kMaxHz = 3000.0f;
  int x = (int)lrintf((tone_hz - kMinHz) * (float)(kScreenW - 1) / (kMaxHz - kMinHz));
  if (x < 0) x = 0;
  if (x >= kScreenW) x = kScreenW - 1;
  return x;
}

static void tx_waterfall_set_max(std::array<uint8_t, 240>& row, int x, uint8_t value) {
  if (x < 0 || x >= (int)row.size()) return;
  if (row[(size_t)x] < value) row[(size_t)x] = value;
}

static void fft_waterfall_tx_tone(float tone_hz) {
  std::array<uint8_t, 240> row{};
  static uint8_t noise_phase = 0;
  for (size_t i = 0; i < row.size(); ++i) {
    row[i] = (uint8_t)(2 + ((i * 17 + noise_phase) & 0x03));
  }
  noise_phase += 29;

  const int pos = tx_waterfall_hz_to_x(tone_hz);
  tx_waterfall_set_max(row, pos - 2, 50);
  tx_waterfall_set_max(row, pos - 1, 120);
  tx_waterfall_set_max(row, pos, 230);
  tx_waterfall_set_max(row, pos + 1, 120);
  tx_waterfall_set_max(row, pos + 2, 50);
  ui_push_tx_waterfall_row(row.data(), (int)row.size());
}

[[maybe_unused]] static bool is_grid4(const std::string& s) {
  if (s.size() != 4) return false;
  auto is_letter = [](char c){ return c >= 'A' && c <= 'R'; };
  auto is_digitc = [](char c){ return c >= '0' && c <= '9'; };
  return is_letter(toupper((unsigned char)s[0])) &&
         is_letter(toupper((unsigned char)s[1])) &&
         is_digitc(s[2]) &&
         is_digitc(s[3]);
}

[[maybe_unused]] static int parse_report_snr(const std::string& f3) {
  if (f3.empty()) return -99;
  std::string s = f3;
  if (!s.empty() && (s[0] == 'R' || s[0] == 'r')) {
    s = s.substr(1);
  }
  if (s.empty()) return -99;
  bool neg = false;
  size_t idx = 0;
  if (s[0] == '+' || s[0] == '-') {
    neg = (s[0] == '-');
    idx = 1;
  }
  int val = 0;
  bool found = false;
  for (; idx < s.size(); ++idx) {
    char c = s[idx];
    if (c < '0' || c > '9') break;
    val = val * 10 + (c - '0');
    found = true;
    if (val > 99) break;
  }
  if (!found) return -99;
  if (neg) val = -val;
  return val;
}

// ---- Static decode workspace (zero heap allocation) ----
// Use the shared RxDecodeEntry type from ui.h so we can hand it directly
// to ui_set_rx_list_static without any conversion.
#define DEC_MAX       RX_MAX_DECODES       // 32
#define DEC_TEXT_MAX  RX_TEXT_MAX          // 64
#define DEC_FIELD_MAX RX_FIELD_MAX         // 20
typedef RxDecodeEntry DecodeMsg;

static DecodeMsg s_dec[DEC_MAX];
static int       s_dec_count;

// Plain-C field parser: tokenize text into field1/field2/field3.
// Equivalent to the old fill_fields_from_text lambda but uses no heap.
static void dec_fill_fields(DecodeMsg* d) {
  d->field1[0] = d->field2[0] = d->field3[0] = '\0';
  char tmp[DEC_TEXT_MAX];
  strncpy(tmp, d->text, sizeof(tmp));
  tmp[sizeof(tmp) - 1] = '\0';

  char* saveptr = nullptr;
  char* toks[8];
  int ntoks = 0;
  for (char* p = strtok_r(tmp, " ", &saveptr); p && ntoks < 8; p = strtok_r(nullptr, " ", &saveptr)) {
    toks[ntoks++] = p;
  }
  if (ntoks == 0) return;

  // Helpers
  auto all_digits = [](const char* s, int len) {
    for (int i = 0; i < len; ++i) if (s[i] < '0' || s[i] > '9') return false;
    return true;
  };
  auto all_alpha = [](const char* s, int len) {
    for (int i = 0; i < len; ++i) {
      char c = s[i];
      if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))) return false;
    }
    return true;
  };

  // CQ <short_token> CALL GRID pattern
  if (strcmp(toks[0], "CQ") == 0 && ntoks >= 2) {
    int len1 = (int)strlen(toks[1]);
    bool short_tok = (len1 <= 3 && all_digits(toks[1], len1)) ||
                     (len1 <= 4 && all_alpha(toks[1], len1));
    if (short_tok) {
      strncpy(d->field1, toks[1], DEC_FIELD_MAX - 1); d->field1[DEC_FIELD_MAX - 1] = '\0';
      if (ntoks > 2) { strncpy(d->field2, toks[2], DEC_FIELD_MAX - 1); d->field2[DEC_FIELD_MAX - 1] = '\0'; }
      if (ntoks > 3) {
        d->field3[0] = '\0';
        for (int i = 3; i < ntoks; ++i) {
          if (i > 3) strncat(d->field3, " ", DEC_FIELD_MAX - strlen(d->field3) - 1);
          strncat(d->field3, toks[i], DEC_FIELD_MAX - strlen(d->field3) - 1);
        }
      }
      return;
    }
  }

  // Default: first 2 tokens + remainder
  strncpy(d->field1, toks[0], DEC_FIELD_MAX - 1); d->field1[DEC_FIELD_MAX - 1] = '\0';
  if (ntoks > 1) { strncpy(d->field2, toks[1], DEC_FIELD_MAX - 1); d->field2[DEC_FIELD_MAX - 1] = '\0'; }
  if (ntoks > 2) {
    d->field3[0] = '\0';
    for (int i = 2; i < ntoks; ++i) {
      if (i > 2) strncat(d->field3, " ", DEC_FIELD_MAX - strlen(d->field3) - 1);
      strncat(d->field3, toks[i], DEC_FIELD_MAX - strlen(d->field3) - 1);
    }
  }
}

// Plain-C normalize: strip <>, uppercase, write into out[out_sz].
static void dec_normalize_call(const char* src, char* out, int out_sz) {
  const char* p = src;
  if (*p == '<') ++p;
  int len = (int)strlen(p);
  if (len > 0 && p[len - 1] == '>') --len;
  if (len >= out_sz) len = out_sz - 1;
  for (int i = 0; i < len; ++i) out[i] = (char)toupper((unsigned char)p[i]);
  out[len] = '\0';
}

// Sort comparator: to_me first (0), then CQ (1), then others (2)
static int dec_sort_cmp(const void* a, const void* b) {
  const DecodeMsg* da = (const DecodeMsg*)a;
  const DecodeMsg* db = (const DecodeMsg*)b;
  // Messages addressed to us stay pinned above everything else regardless of
  // age. Everything else (including CQs) is newest-heard-first -- the list
  // is persistent now, so recency is what "put the newest ones at the top"
  // means for a station that's been sitting there a while.
  int ga = da->is_to_me ? 0 : 1;
  int gb = db->is_to_me ? 0 : 1;
  if (ga != gb) return ga - gb;
  if (da->heard_ms != db->heard_ms) return (da->heard_ms > db->heard_ms) ? -1 : 1;
  return 0;
}

// Clears the persistent decode list -- call whenever previously-heard
// entries stop being relevant (e.g. a band change; the old band's decodes
// would otherwise keep sitting there mixed in with the new band's traffic).
static void clear_decode_list() {
  s_dec_count = 0;
  ui_set_rx_list_static(s_dec, 0);
}

void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui) {
  // The decode list persists across cycles (Dean: don't blank it away between
  // cycles on a quiet band -- keep everything, newest at top; repeats from
  // the same station refresh their existing row rather than piling up).
  // this_cycle_ms tags every entry added/refreshed THIS call so downstream
  // consumers (autoseq's to-me feed, the RTC soft-sync) can tell "freshly
  // heard this cycle" apart from "still sitting in the list from before."
  const int64_t this_cycle_ms = rtc_now_ms();

  // Candidate cap. Lowered 50→24 to keep the per-slot decode under the ~2.15s
  // gap between the end of the FT8 transmission (12.64s) and the next 15s UTC
  // slot boundary. On this no-PSRAM board the decode runs synchronously in the
  // audio task and drops incoming audio while it runs; if it overruns the slot
  // boundary the pipeline must skip the next slot to stay aligned, so we only
  // decode every OTHER slot — which can miss a QSO reply (replies land on the
  // opposite 15s slot from your TX). Measured decode time was ~2.7-2.8s at 50
  // candidates REGARDLESS of how many actually decoded (the cost is scanning +
  // attempting LDPC on all candidates, not the successful ones), and a busy
  // band always hit the cap. Candidates are returned strongest-sync-first, so
  // trimming the weak tail keeps the real decodes (we never saw >3/slot) while
  // buying back enough time to decode EVERY slot.
  const int max_cand = 24;
  static ftx_candidate_t candidates[max_cand];
  int num_candidates = ftx_find_candidates(&mon->wf, max_cand, candidates, 5);
  ESP_LOGI(TAG, "Candidates found: %d", num_candidates);

  // ---- slot index + once-per-slot hashtable maintenance ----
  int64_t slot_idx = (g_decode_slot_idx >= 0) ? g_decode_slot_idx : rtc_now_ms() / g_protocol->slot_time_ms;
  int slot_id = (int)(slot_idx & 1);

  static int64_t s_last_aged_slot = -1;
  if (slot_idx != s_last_aged_slot) {
    s_last_aged_slot = slot_idx;
    hashtable_age_all();
  }

  // ---- estimate noise floor ----
  float noise_db = -120.0f;
  if (mon->wf.mag && mon->wf.num_blocks > 0) {
    const size_t total = (size_t)mon->wf.num_blocks * (size_t)mon->wf.block_stride;
    static uint32_t hist[256];
    memset(hist, 0, sizeof(hist));
    for (size_t i = 0; i < total; ++i) hist[mon->wf.mag[i]]++;
    uint64_t target = total * 25 / 100;
    uint64_t accum = 0;
    int noise_scaled = 0;
    for (int v = 0; v < 256; ++v) {
      accum += hist[v];
      if (accum >= target) { noise_scaled = v; break; }
    }
    noise_db = 0.5f * ((float)noise_scaled - 240.0f);
  }

  // ---- mycall uppercase (stack, not heap) ----
  char mycall_up[DEC_FIELD_MAX];
  {
    const char* src = g_call.c_str();
    int len = (int)g_call.size();
    if (len >= DEC_FIELD_MAX) len = DEC_FIELD_MAX - 1;
    for (int i = 0; i < len; ++i) mycall_up[i] = (char)toupper((unsigned char)src[i]);
    mycall_up[len] = '\0';
  }

  // ---- decode candidates into static s_dec[] ----
  const int kMaxDecoded = 50;
  static ftx_message_t decoded[kMaxDecoded];
  static ftx_message_t* decoded_hashtable[kMaxDecoded];
  for (int i = 0; i < kMaxDecoded; ++i) decoded_hashtable[i] = nullptr;
  int num_decoded = 0;

  // Track this cycle's own timing samples for the RTC soft-sync below --
  // s_dec[] now holds decodes from many cycles back, so it can no longer be
  // scanned directly for that (it would sync against stale timing).
  float this_cycle_time_s[DEC_MAX];
  int this_cycle_n = 0;
  int fresh_this_cycle = 0;  // count of entries actually touched this cycle, for the log below

  if (num_candidates <= 0) {
    ESP_LOGW(TAG, "No candidates found");
    // Keep showing whatever's already accumulated -- a quiet slot adds
    // nothing new, but shouldn't blank out what's already on screen.
    ui_set_rx_list_static(s_dec, s_dec_count);
    if (update_ui) { ui_draw_rx(); }
    else core_fire_rx_changed();
    // No candidates means we processed the slot's audio but found nothing —
    // still counts as "applied" for the TX-trigger guard.
    if (g_decode_slot_idx > g_decode_applied_slot_idx) {
      g_decode_applied_slot_idx = g_decode_slot_idx;
    }
    g_decode_in_progress = false;
    return;
  }

  for (int i = 0; i < num_candidates; ++i) {
    ftx_message_t message;
    ftx_decode_status_t status;
    memset(&message, 0, sizeof(message));
    memset(&status, 0, sizeof(status));

    if (!ftx_decode_candidate(&mon->wf, &candidates[i], 25, &message, &status))
      continue;

    // payload/hash dedupe (open addressing)
    int idx_hash = (int)(message.hash % kMaxDecoded);
    bool found_empty = false, found_dup = false;
    for (int probe = 0; probe < kMaxDecoded; ++probe) {
      ftx_message_t* p = decoded_hashtable[idx_hash];
      if (!p) { found_empty = true; break; }
      if (p->hash == message.hash &&
          0 == memcmp(p->payload, message.payload, sizeof(message.payload))) {
        found_dup = true; break;
      }
      idx_hash = (idx_hash + 1) % kMaxDecoded;
    }
    if (found_dup || !found_empty) continue;

    memcpy(&decoded[idx_hash], &message, sizeof(message));
    decoded_hashtable[idx_hash] = &decoded[idx_hash];
    ++num_decoded;

    // decode to human text
    char text[FTX_MAX_MESSAGE_LENGTH] = {0};
    ftx_message_offsets_t offsets;
    ftx_message_rc_t urc = ftx_message_decode(&message, &hash_if, text, &offsets);
    if (urc != FTX_MESSAGE_RC_OK || text[0] == '\0') continue;

    // freq / time / SNR
    float freq_hz = (mon->min_bin + candidates[i].freq_offset +
                    candidates[i].freq_sub / (float)cfg->freq_osr) / mon->symbol_period;
    float time_s = (candidates[i].time_offset +
                   candidates[i].time_sub / (float)cfg->time_osr) * mon->symbol_period;

    float cand_db = noise_db;
    {
      int t_index = candidates[i].time_offset * mon->wf.time_osr + candidates[i].time_sub;
      const int t_count = mon->wf.num_blocks * mon->wf.time_osr;
      if (t_count > 0) { if (t_index < 0) t_index = 0; if (t_index >= t_count) t_index = t_count - 1; }
      else t_index = 0;

      int f_index = candidates[i].freq_sub * mon->wf.num_bins + candidates[i].freq_offset;
      const int f_count = mon->wf.freq_osr * mon->wf.num_bins;
      if (f_count > 0) { if (f_index < 0) f_index = 0; if (f_index >= f_count) f_index = f_count - 1; }
      else f_index = 0;

      size_t offset2 = (size_t)t_index * (size_t)f_count + (size_t)f_index;
      size_t total2 = (size_t)mon->wf.num_blocks * (size_t)mon->wf.block_stride;
      if (mon->wf.mag && offset2 < total2) cand_db = 0.5f * ((float)mon->wf.mag[offset2] - 240.0f);
    }

    int snr_q = (int)lrintf(cand_db - noise_db);
    if (snr_q < -30) snr_q = -30;
    if (snr_q >  99) snr_q = 99;

    // DXpedition rewrite (uses heap briefly via std::string — bounded, rare path)
    char final_text[DEC_TEXT_MAX];
    {
      std::string raw(text);
      std::string rewritten(text);
      if (rewrite_dxpedition_for_mycall(raw, mycall_up, rewritten)) {
        ESP_LOGI(TAG, "DXpedition raw match: %s", text);
      }
      strncpy(final_text, rewritten.c_str(), DEC_TEXT_MAX - 1);
      final_text[DEC_TEXT_MAX - 1] = '\0';
    }

    if (this_cycle_n < DEC_MAX) this_cycle_time_s[this_cycle_n++] = time_s;

    // UI text dedup (linear scan through the whole persistent list — 32 max)
    int dup_idx = -1;
    for (int j = 0; j < s_dec_count; ++j) {
      if (strcmp(s_dec[j].text, final_text) == 0) { dup_idx = j; break; }
    }
    if (dup_idx >= 0) {
      // Exact same text already in the list (this cycle or an earlier one) --
      // refresh it in place instead of adding a duplicate line.
      if (snr_q > s_dec[dup_idx].snr) {
        s_dec[dup_idx].snr = snr_q;
        s_dec[dup_idx].offset_hz = (int)lrintf(freq_hz);
        s_dec[dup_idx].slot_id = slot_id;
      }
      s_dec[dup_idx].heard_ms = this_cycle_ms;
      fresh_this_cycle++;
      continue;
    }

    ESP_LOGI(TAG, "Decoded[%d] t=%.2fs f=%.1fHz snr=%d : %s",
             fresh_this_cycle, time_s, freq_hz, snr_q, final_text);

    // Parse into a scratch entry so we can match it against the existing
    // list by station (field2 = the transmitting callsign, for both CQ and
    // directed messages) before deciding whether to refresh an existing row
    // or add a new one.
    DecodeMsg scratch{};
    strncpy(scratch.text, final_text, DEC_TEXT_MAX - 1); scratch.text[DEC_TEXT_MAX - 1] = '\0';
    scratch.snr = snr_q;
    scratch.offset_hz = (int)lrintf(freq_hz);
    scratch.slot_id = slot_id;
    scratch.time_s = time_s;

    dec_fill_fields(&scratch);

    scratch.is_cq = (strncmp(scratch.text, "CQ ", 3) == 0 || strcmp(scratch.text, "CQ") == 0);

    char f1_norm[DEC_FIELD_MAX];
    dec_normalize_call(scratch.field1, f1_norm, DEC_FIELD_MAX);
    scratch.is_to_me = (mycall_up[0] != '\0' && strcmp(f1_norm, mycall_up) == 0);
    scratch.heard_ms = this_cycle_ms;

    // Same-station refresh: a station repeating (or advancing to its next
    // message) updates its existing row and jumps to the top via heard_ms,
    // instead of spawning a duplicate line for the same caller.
    char f2_norm[DEC_FIELD_MAX];
    dec_normalize_call(scratch.field2, f2_norm, DEC_FIELD_MAX);
    int station_idx = -1;
    if (f2_norm[0] != '\0') {
      for (int j = 0; j < s_dec_count; ++j) {
        char existing_f2[DEC_FIELD_MAX];
        dec_normalize_call(s_dec[j].field2, existing_f2, DEC_FIELD_MAX);
        if (strcmp(existing_f2, f2_norm) == 0) { station_idx = j; break; }
      }
    }

    if (station_idx >= 0) {
      s_dec[station_idx] = scratch;
    } else if (s_dec_count < DEC_MAX) {
      s_dec[s_dec_count++] = scratch;
    } else {
      // List's at capacity -- evict the least-recently-heard entry to make
      // room, rather than dropping the newest decode on the floor.
      int oldest_idx = 0;
      for (int j = 1; j < s_dec_count; ++j) {
        if (s_dec[j].heard_ms < s_dec[oldest_idx].heard_ms) oldest_idx = j;
      }
      s_dec[oldest_idx] = scratch;
    }
    fresh_this_cycle++;
  }

  ESP_LOGI(TAG, "Decoded %d unique messages (%d total in list)", fresh_this_cycle, s_dec_count);

  // ---- Auto sync soft RTC from decode timing (this cycle's samples only --
  // s_dec[] holds decodes from many cycles back now, so it can't be used
  // directly here without syncing against stale timing) ----
  if (this_cycle_n > 3) {
    // Simple insertion sort to find median of time_s values
    float sorted_t[DEC_MAX];
    int nt = this_cycle_n;
    memcpy(sorted_t, this_cycle_time_s, sizeof(float) * (size_t)nt);
    for (int i = 1; i < nt; ++i) {
      float key = sorted_t[i];
      int j = i - 1;
      while (j >= 0 && sorted_t[j] > key) { sorted_t[j + 1] = sorted_t[j]; --j; }
      sorted_t[j + 1] = key;
    }
    float median = sorted_t[nt / 2];
    if (fabsf(median) > 0.3f) {
      int delta_ms = (int)lrintf(-median * 1000.0f);
      if (delta_ms > 320) delta_ms = 320;
      if (delta_ms < -320) delta_ms = -320;
      rtc_ms_start -= delta_ms;
      rtc_last_update -= delta_ms;
      rtc_update_strings();
      rtc_sync_to_esp_rtc();
      ESP_LOGI("SYNC", "Applied RTC sync: median=%.2fs delta=%dms", median, delta_ms);
    }
  }

  // ---- Sort in-place: to-me pinned first, everything else newest-heard-first ----
  qsort(s_dec, s_dec_count, sizeof(DecodeMsg), dec_sort_cmp);

  // ---- Autoseq: build small to_me vector at boundary (only FRESH to_me
  // entries -- s_dec[] is persistent now, so a to-me row can still be
  // sitting there from a prior cycle if the station never repeated; feeding
  // that stale entry to autoseq again would reprocess an already-handled
  // decode every single cycle) ----
  if (!g_was_txing) {
    std::vector<UiRxLine> to_me_auto;
    for (int i = 0; i < s_dec_count; ++i) {
      if (!s_dec[i].is_to_me) break;  // sorted, so once we pass to_me we're done
      if (s_dec[i].heard_ms != this_cycle_ms) continue;  // stale carry-over, not new this cycle
      UiRxLine rx;
      rx.text = s_dec[i].text;
      rx.field1 = s_dec[i].field1;
      rx.field2 = s_dec[i].field2;
      rx.field3 = s_dec[i].field3;
      rx.snr = s_dec[i].snr;
      rx.offset_hz = s_dec[i].offset_hz;
      rx.slot_id = s_dec[i].slot_id;
      rx.is_cq = s_dec[i].is_cq;
      rx.is_to_me = true;
      to_me_auto.push_back(std::move(rx));
    }

    if (!to_me_auto.empty()) {
      autoseq_on_decodes(to_me_auto);
      core_fire_qso_changed();  // propagates to all registered consumers
      g_last_reply_text = to_me_auto.front().text;
    }

    AutoseqTxEntry pending;
    if (autoseq_fetch_pending_tx(pending)) {
      arm_pending_tx(pending);
      ESP_LOGI(TAG, "TX ready: %s parity=%d", pending.text.c_str(), g_target_slot_parity);
    } else if (g_cq_running) {
      // Calling CQ is the beacon trigger now: keep re-issuing the same CQ
      // every cycle the queue is idle (unanswered CQ, or a finished QSO)
      // until the user answers someone else or presses ESC.
      enqueue_running_cq();
      if (autoseq_fetch_pending_tx(pending)) {
        arm_pending_tx(pending);
        ESP_LOGI(TAG, "Running CQ ready: %s parity=%d", pending.text.c_str(), g_target_slot_parity);
      }
    }
  }

  // ---- Zero-heap handoff: static s_dec[] → ui.cpp's static rx_lines[] ----
  ui_set_rx_list_static(s_dec, s_dec_count);

  if (update_ui) {
    ui_draw_rx();
    char buf[64];
    snprintf(buf, sizeof(buf), "Heap %u", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    debug_log_line(buf);
  } else {
    core_fire_rx_changed();
  }

  // Mark this slot's decode as fully applied BEFORE clearing the in-progress
  // flag. Readers (TX trigger on core 0) must see the applied marker as soon
  // as in_progress drops, not later.
  if (g_decode_slot_idx > g_decode_applied_slot_idx) {
    g_decode_applied_slot_idx = g_decode_slot_idx;
  }
  g_decode_in_progress = false;
}

static void draw_menu_long_edit() {
  std::vector<std::string> lines(6, "");
  // Mark the cursor position before chunking into display rows. Mid-buffer
  // (e.g. the CQ prompt's pre-positioned cursor) brackets the character it
  // sits on, same convention as the Date/Time editor's highlight_pos() --
  // splicing in a literal "_" there would look like part of the text and
  // can't be backspaced (it isn't really in the buffer). At the end of the
  // buffer (the common append-while-typing case) there's no character to
  // bracket, so fall back to a trailing "_".
  int cpos = (menu_long_cursor_pos >= 0 && menu_long_cursor_pos <= (int)menu_long_buf.size())
                 ? menu_long_cursor_pos : (int)menu_long_buf.size();
  std::string display;
  if (cpos < (int)menu_long_buf.size()) {
    display = highlight_pos(menu_long_buf, cpos);
  } else {
    display = menu_long_buf;
    display.push_back('_');
  }
  size_t idx = 0;
  int line = 0;
  while (idx < display.size() && line < 6) {
    size_t chunk = std::min<size_t>(18, display.size() - idx);
    lines[line] = display.substr(idx, chunk);
    idx += chunk;
    line++;
  }
  ui_draw_debug(lines, 0);
}

[[maybe_unused]] static bool looks_like_grid(const std::string& s) {
  if (s.size() != 4) return false;
  return std::isalpha((unsigned char)s[0]) && std::isalpha((unsigned char)s[1]) &&
         std::isdigit((unsigned char)s[2]) && std::isdigit((unsigned char)s[3]);
}

[[maybe_unused]] static bool looks_like_report(const std::string& s, int& out) {
  if (s.empty()) return false;
  int sign = 1;
  size_t idx = 0;
  if (s[0] == '-') { sign = -1; idx = 1; }
  else if (s[0] == '+') { idx = 1; }
  if (idx >= s.size()) return false;
  int val = 0;
  for (; idx < s.size(); ++idx) {
    if (!std::isdigit((unsigned char)s[idx])) return false;
    val = val * 10 + (s[idx] - '0');
  }
  out = sign * val;
  return true;
}

// Re-issue the CQ that's already running (g_cq_running), targeting the next
// available slot after now -- same parity calc the C-prompt confirm uses.
// Duplicate prevention is handled by autoseq_start_cq().
// TX trigger happens at slot boundary via check_slot_boundary().
static void enqueue_running_cq() {
  const int64_t now_ms = rtc_now_ms();
  const int slot_period = g_protocol->slot_time_ms;
  const int next_parity = (int)(((now_ms / slot_period) + 1) & 1);
  autoseq_start_cq(next_parity);
  core_fire_qso_changed();  // propagates to all registered consumers
}

static bool autoseq_has_pending_tx() {
  AutoseqTxEntry tmp;
  return autoseq_fetch_pending_tx(tmp);
}

// Schedule a one-off pending TX (e.g., manual FreeText) without touching autoseq state.
// Returns false if TX is already active or if scheduling failed.
// Uses the single-threaded state machine - TX will trigger at next matching slot boundary.
static bool schedule_manual_pending_tx(const AutoseqTxEntry& pending) {
  // Already transmitting or TX pending?
  if (g_tx_active || g_qso_xmit) {
    return false;
  }

  arm_pending_tx(pending);
  ESP_LOGI(TAG, "schedule_manual_pending_tx: queued TX=%s for parity=%d",
           pending.text.c_str(), g_target_slot_parity);
  return true;
}

// NOTE: This function is now mostly superseded by the state machine approach.
// TX scheduling is done via g_qso_xmit and g_target_slot_parity flags,
// and check_slot_boundary() triggers TX at the right time.
// Keeping this as a no-op for now in case any code still calls it.
[[maybe_unused]] static void schedule_tx_if_idle() {
  // No-op: TX scheduling is now handled by decode_monitor_results setting
  // g_qso_xmit and check_slot_boundary triggering TX at slot start.
}

// Helper to send TA command (deduplicated)
static void tx_send_ta(float tone_hz) {
  int ta_int = (int)lrintf(tone_hz);
  float frac = tone_hz - (float)ta_int;
  int ta_frac = (int)lrintf(frac * 100.0f);
  if (ta_int == g_tx_last_ta_int && ta_frac == g_tx_last_ta_frac) return;
  if (radio_control_set_tone_hz(tone_hz) == ESP_OK) {
    g_tx_last_ta_int = ta_int;
    g_tx_last_ta_frac = ta_frac;
  }
}

// Start TX (single-threaded state machine initialization)
// Called from check_slot_boundary at the right time
// Uses g_pending_tx which was prepared by check_slot_boundary with correct offset
static void tx_start(int skip_tones) {
  // Already transmitting?
  if (g_tx_active) {
    return;
  }

  // Use g_pending_tx which was prepared by check_slot_boundary
  if (!g_pending_tx_valid || g_pending_tx.text.empty()) {
    ESP_LOGW(TAG, "tx_start: no pending TX");
    return;
  }

  // Blank the countdown bar now, before any audio/CAT setup below -- once TX
  // is actually underway, redraws are held (SPI/DMA contention with the
  // outgoing audio), so this is the only safe moment to touch it. Otherwise
  // it sits frozen at whatever fraction it had when TX began for the whole
  // transmission, then visibly jumps once RX resumes and it starts ticking
  // again from the real elapsed time -- blank reads as intentional instead.
  ui_clear_countdown();

  // Get current slot info
  int64_t now_ms = rtc_now_ms();
  const int slot_period = g_protocol->slot_time_ms;
  g_tx_slot_idx = now_ms / slot_period;

  ESP_LOGI(TAG, "tx_start: TX=%s offset=%d skip=%d slot=%lld proto=%s",
           g_pending_tx.text.c_str(), g_pending_tx.offset_hz, skip_tones, (long long)g_tx_slot_idx,
           g_protocol->name);

  // Notify autoseq that TX emission is starting. This is the single canonical
  // logging trigger — if we're about to emit TX4 (RR73) or TX5 (73), autoseq
  // writes the ADIF entry now.
  autoseq_on_tx_starting();

  // Encode message to tones
  ftx_message_t msg;
  ftx_message_rc_t rc = ftx_message_encode(&msg, &hash_if, g_pending_tx.text.c_str());
  if (rc != FTX_MESSAGE_RC_OK) {
    ESP_LOGE(TAG, "Encode failed for TX");
    return;
  }
  if (g_protocol->protocol_id == FTX_PROTOCOL_FT4) {
    ft4_encode(msg.payload, g_tx_tones);
  } else {
    ft8_encode(msg.payload, g_tx_tones);
  }

  // Set up TX state machine
  // IMPORTANT: Tone timing must be based on slot boundary, not TX start time.
  // This ensures TX ends at the correct time even if TX started late,
  // allowing RX to start cleanly at the next slot boundary.
  g_tx_base_hz = g_pending_tx.offset_hz;
  g_tx_slot_start_ms = (now_ms / slot_period) * slot_period;  // Slot boundary time
  g_tx_tone_idx = (skip_tones >= g_protocol->total_symbols) ? g_protocol->total_symbols : skip_tones;
  // Next tone time = slot_start + tone_idx * symbol_period_ms
  // Aligns all tones to the slot boundary, not to when TX started
  const int sym_ms = (int)roundf(g_protocol->symbol_period * 1000.0f);
  g_tx_next_tone_time = g_tx_slot_start_ms + g_tx_tone_idx * sym_ms;
  g_tx_last_ta_int = -1;
  g_tx_last_ta_frac = -1;

  ESP_LOGI(TAG, "TX base_hz=%d (from pre-computed offset, text=%s)", g_tx_base_hz, g_pending_tx.text.c_str());

  // Send CAT setup commands
  g_tx_cat_ok = radio_control_ready();
  if (g_tx_cat_ok) {
    int freq_hz = (int)(g_bands[g_band_sel].freq * 1000.0f);
    esp_err_t err = radio_control_begin_tx(freq_hz, g_tx_base_hz);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "tx_start: radio TX begin failed: %s", esp_err_to_name(err));
      g_tx_cat_ok = false;
    }
  }

  // IC-705 uses DDS-driven WiFi UDP audio (same CPFSK approach as QDX/UAC).
  // The PTT was already keyed by radio_control_begin_tx above; now start
  // streaming the audio symbols to the radio.
  if (g_tx_cat_ok && canonical_radio_type(g_radio) == RadioType::IC705) {
    const int remaining_tones = g_protocol->total_symbols - g_tx_tone_idx;
    if (remaining_tones <= 0 ||
        !ic705_tx_begin_cpfsk(static_cast<float>(g_tx_base_hz),
                               g_tx_tones + g_tx_tone_idx,
                               static_cast<size_t>(remaining_tones),
                               g_protocol->tone_spacing,
                               g_protocol->samples_per_symbol)) {
      ESP_LOGW(TAG, "tx_start: IC-705 WiFi audio start failed");
      radio_control_end_tx();
      g_tx_cat_ok = false;
      return;
    }
  }

  if (skip_tones > 0) {
    ESP_LOGI("TXTONE", "Skipping first %d tones due to late start", skip_tones);
  }

  // Send first tone TA if CAT is ready
  if (g_tx_cat_ok && g_tx_tone_idx < g_protocol->total_symbols) {
    float tone_hz = g_tx_base_hz + g_protocol->tone_spacing * g_tx_tones[g_tx_tone_idx];
    tx_send_ta(tone_hz);
  }

  // Mark TX as active. FT8 is half-duplex and this device has no use for
  // RX decode while transmitting (that's only useful feeding a logger on a
  // PC) — pause the decoder so it can't contend with the TX path at all.
  ui_set_rx_waterfall_muted(true);
  g_tx_active = true;
  g_decode_enabled = false;
}

// TX state machine tick - called from main loop
// Sends one tone at a time, non-blocking
static void tx_tick() {
  if (!g_tx_active) {
    return;
  }

  int64_t now_ms = rtc_now_ms();

  // Check for cancel request
  if (g_tx_cancel_requested) {
    ESP_LOGI(TAG, "tx_tick: TX cancelled at tone %d", g_tx_tone_idx);
    if (g_tx_cat_ok) {
      radio_control_end_tx();
    }
    ui_set_rx_waterfall_muted(false);
    g_tx_active = false;
    g_decode_enabled = true;
    g_pending_tx_valid = false;
    g_tx_cancel_requested = false;
    g_was_txing = false;  // TX was cancelled - don't call tick at slot boundary
    core_fire_qso_changed();  // propagates to all registered consumers
    return;
  }

  // Time to send next tone?
  if (now_ms < g_tx_next_tone_time) {
    return;  // Not yet
  }

  // All tones sent?
  if (g_tx_tone_idx >= g_protocol->total_symbols) {
    ESP_LOGI(TAG, "tx_tick: TX complete, all %d tones sent", g_protocol->total_symbols);
    if (g_tx_cat_ok) {
      radio_control_end_tx();
    }
    ui_set_rx_waterfall_muted(false);
    // Record slot index for spacing and notify autoseq
    s_last_tx_slot_idx = g_tx_slot_idx;
    autoseq_mark_sent(g_tx_slot_idx);
    // g_was_txing stays true - tick will be called at slot boundary

    g_tx_active = false;
    g_decode_enabled = true;
    g_pending_tx_valid = false;
    g_tx_cancel_requested = false;
    core_fire_qso_changed();  // propagates to all registered consumers
    return;
  }

  // Send current tone to the local visualizer and selected radio backend.
  ESP_LOGD("TXTONE", "%02d %u", g_tx_tone_idx, (unsigned)g_tx_tones[g_tx_tone_idx]);
  float tone_hz = g_tx_base_hz + g_protocol->tone_spacing * g_tx_tones[g_tx_tone_idx];
  fft_waterfall_tx_tone(tone_hz);
  if (g_tx_cat_ok) {
    tx_send_ta(tone_hz);
  }

  // Advance to next tone
  g_tx_tone_idx++;
  // Calculate next tone time from slot boundary to ensure TX ends at correct time
  // This guarantees RX can start cleanly at the next slot boundary
  g_tx_next_tone_time = g_tx_slot_start_ms + g_tx_tone_idx * (int)roundf(g_protocol->symbol_period * 1000.0f);
}

static void draw_menu_view() {
    if (menu_long_edit) {
      draw_menu_long_edit();
      return;
    }
  int64_t now = rtc_now_ms();
  if (menu_copy_feedback_deadline > 0 && now >= menu_copy_feedback_deadline) {
    menu_copy_feedback_deadline = 0;
    menu_copy_feedback_text.clear();
  }
  if (!g_q_clear_feedback.empty() && now >= g_q_clear_feedback_deadline) {
    g_q_clear_feedback.clear();
  }
  if (g_q_clear_armed && now >= g_q_clear_arm_deadline) {
    g_q_clear_armed = false;
  }

  if (menu_category < 0) {
    // Category picker.
    std::vector<std::string> cats = {"Station", "Operating", "IC-705/Network", "Logging", "System"};
    ui_draw_list(cats, 0, -1);
    return;
  }

  std::vector<std::string> lines;
  lines.reserve(6);

  if (menu_category == kCatStation) {
    lines.push_back(std::string("Call:") + elide_right(menu_edit_idx == kCatStation * MENU_CAT_BASE + 0 ? menu_edit_buf : g_call));
    std::string display_grid = g_grid;
    if (menu_edit_idx == kCatStation * MENU_CAT_BASE + 1) {
      display_grid = menu_edit_buf;
    } else if (g_time_synced_from_gps && g_grid_from_gps && g_grid_gps_display8.size() == 8) {
      display_grid = g_grid_gps_display8.substr(0, 6);
    }
    lines.push_back(std::string("Grid:") + elide_right(display_grid));
    lines.push_back(std::string("ActiveBand:") + head_trim(g_active_band_text, 16));
    lines.push_back("Edit Band Freqs");
    lines.push_back(std::string("UTC:") + g_time + rtc_time_source_suffix());
  } else if (menu_category == kCatOperating) {
    lines.push_back(std::string("Offset:") + offset_name(g_offset_src));
    if (menu_edit_idx == kCatOperating * MENU_CAT_BASE + 1) {
      lines.push_back(std::string("Fixed:") + menu_edit_buf);
    } else {
      lines.push_back(std::string("Fixed:") + std::to_string(g_offset_hz));
    }
    lines.push_back(std::string("SkipTX1:") + (g_skip_tx1 ? "ON" : "OFF"));
    if (menu_edit_idx == kCatOperating * MENU_CAT_BASE + 3) {
      lines.push_back(std::string("Max Retry:") + menu_edit_buf);
    } else {
      lines.push_back(std::string("Max Retry:") + std::to_string(g_autoseq_max_retry));
    }
#if ENABLE_FT4
    {
      // Show the saved (pending) mode.  Add '*' if it differs from the running
      // boot mode so the user knows a reboot is needed to apply the change.
      const char* pending_name = g_protocol_pending_ft4 ? "FT4" : "FT8";
      bool needs_reboot = g_protocol_pending_ft4 != (g_protocol == &kProtocolFT4);
      lines.push_back(std::string("Mode: ") + pending_name + (needs_reboot ? "*" : ""));
    }
#endif
  } else if (menu_category == kCatNetwork) {
    // Keys 1-6: 1 SSID, 2 WiFi Pass, 3 Net User, 4 Net Pass, 5 CI-V Addr,
    // 6 Re-resolve. menu_edit_idx equals kCatNetwork*MENU_CAT_BASE + local.
    {
      std::string ssid_disp = g_ic705_wifi_ssid.empty() ? "(not set)" : g_ic705_wifi_ssid;
      if (menu_edit_idx == kCatNetwork * MENU_CAT_BASE + 0) ssid_disp = menu_edit_buf;
      lines.push_back(std::string("WiFi SSID:") + head_trim(ssid_disp, 10));
    }
    {
      // Mask secrets as asterisks unless actively editing.
      std::string pass_disp;
      if (menu_edit_idx == kCatNetwork * MENU_CAT_BASE + 1) {
        pass_disp = menu_edit_buf;
      } else {
        pass_disp = g_ic705_wifi_pass.empty() ? "(not set)" : std::string(g_ic705_wifi_pass.size(), '*');
      }
      lines.push_back(std::string("WiFi Pass:") + head_trim(pass_disp, 10));
    }
    {
      std::string user_disp = g_ic705_net_user.empty() ? "(not set)" : g_ic705_net_user;
      if (menu_edit_idx == kCatNetwork * MENU_CAT_BASE + 2) user_disp = menu_edit_buf;
      lines.push_back(std::string("Net User:") + head_trim(user_disp, 10));
    }
    {
      std::string npass_disp;
      if (menu_edit_idx == kCatNetwork * MENU_CAT_BASE + 3) {
        npass_disp = menu_edit_buf;
      } else {
        npass_disp = g_ic705_net_pass.empty() ? "(not set)" : std::string(g_ic705_net_pass.size(), '*');
      }
      lines.push_back(std::string("Net Pass:") + head_trim(npass_disp, 10));
    }
    {
      char civ_str[8];
      if (menu_edit_idx == kCatNetwork * MENU_CAT_BASE + 4) {
        snprintf(civ_str, sizeof(civ_str), "%s", menu_edit_buf.c_str());
      } else {
        snprintf(civ_str, sizeof(civ_str), "0x%02X", (unsigned)g_ic705_civ_addr);
      }
      lines.push_back(std::string("CI-V Addr:") + civ_str);
    }
  } else if (menu_category == kCatLogging) {
    if (menu_copy_feedback_deadline > 0 && !menu_copy_feedback_text.empty()) {
      lines.push_back(menu_copy_feedback_text);
    } else {
      lines.push_back("End+Export Log SD");
    }
    if (!g_q_clear_feedback.empty()) {
      lines.push_back(g_q_clear_feedback);
    } else if (g_q_clear_armed) {
      lines.push_back("Press 2 again: confirm");
    } else {
      lines.push_back("Clear QSO Log: " + std::to_string((unsigned)g_adif_sd_seq));
    }
    lines.push_back("Performance");
  } else if (menu_category == kCatSystem) {
    lines.push_back(menu_sleep_batt_line());
    lines.push_back("Sleep Now");
    lines.push_back("GPS Status");
    lines.push_back(std::string("Reslv/Conn:") + wifi_mgr_status_string());
    lines.push_back("Brightness: " + std::to_string(g_brightness_step) + "/10");
  }

  int highlight_local = -1;
  if (menu_edit_idx >= 0) {
    highlight_local = menu_edit_idx - menu_category * MENU_CAT_BASE;
  } else if (menu_flash_idx >= 0 && now < menu_flash_deadline) {
    highlight_local = menu_flash_idx - menu_category * MENU_CAT_BASE;
  } else {
    menu_flash_idx = -1;
  }
  if (menu_flash_idx >= 0 && now >= menu_flash_deadline) {
    menu_flash_idx = -1;
  }
  if (highlight_local < 0 || highlight_local >= (int)lines.size()) highlight_local = -1;
  ui_draw_list(lines, 0, highlight_local);
}

static std::string status_sync_line() {
  const bool streaming = audio_source_is_streaming();
  const RadioType radio = canonical_radio_type(g_radio);

  if (radio == RadioType::IC705) {
    wifi_mgr_state_t ws = wifi_mgr_get_state();
    switch (ws) {
      case WIFI_MGR_IDLE:       return "705 WiFi off";
      case WIFI_MGR_CONNECTING: return "705 Connecting";
      case WIFI_MGR_CONNECTED:  return "705 Resolving";
      case WIFI_MGR_RESOLVING:  return "705 Resolving";
      case WIFI_MGR_ERROR:      return "705 WiFi error";
      case WIFI_MGR_READY:
        if (streaming && radio_control_ready()) return "705 RX+TX ready";
        if (streaming) return std::string("705 RX ") + ic705_net_status_string();
        if (radio_control_ready())               return "705 CAT ready";
        return std::string("705 ") + ic705_net_status_string();
    }
  }

  if (streaming) return std::string("Sync to ") + radio_name(radio);
  return std::string("Connect to ") + radio_name(radio);
}

// Background tasks (e.g. the IC-705 network login) update their status
// strings continuously. Without polling this on every loop tick — not just
// after a keypress — the STATUS screen would show a frozen snapshot taken
// at the last key press, even while a connect attempt actively progresses
// in the background between presses.
static void refresh_status_view_if_dirty() {
  static int last_sig = -1;
  static std::string last_text;
  if (ui_mode != UIMode::STATUS) {
    last_sig = -1;
    last_text.clear();
    return;
  }
  int cur_sig = audio_source_is_streaming() ? 1 : 0;
  cur_sig |= ((int)canonical_radio_type(g_radio) << 4);
  std::string cur_text = status_sync_line();
  if (cur_sig != last_sig || cur_text != last_text) {
    draw_status_view();
    last_sig = cur_sig;
    last_text = cur_text;
  }
}

static std::string s_last_gps_lines[6];

static void draw_gps_view(bool force_redraw) {
  std::vector<std::string> lines;
  lines.reserve(6);
  gps_state_t state = gps_get_state();
  lines.push_back(std::string("Src:") + gps_source_name());
  if (state.valid_fix) {
    lines.push_back(std::string("Fix: 3D (") + std::to_string(state.satellites) + " Sats)");
  } else {
    lines.push_back(std::string("Fix: NO FIX (") + std::to_string(state.satellites) + " Sats)");
  }
  lines.push_back(std::string("Time: ") + (state.time_utc.empty() ? "Wait..." : state.time_utc));
  lines.push_back(std::string("Grid: ") + (state.grid_square.empty() ? "----" : state.grid_square));
  char loc[64];
  snprintf(loc, sizeof(loc), "L: %.3f, %.3f", state.latitude, state.longitude);
  lines.push_back(loc);
  if (state.last_rx_ms > 0) {
    uint32_t diff = (xTaskGetTickCount() * portTICK_PERIOD_MS - state.last_rx_ms) / 1000;
    lines.push_back(std::string("Sync: Good (") + std::to_string(diff) + "s ago)");
  } else {
    lines.push_back("Sync: Pending...");
  }
  
  const int line_h = 19;
  const int start_y = UI_START_Y;

  M5.Display.startWrite();
  M5.Display.setTextSize(2);
  for (size_t i = 0; i < 6; ++i) {
    std::string text = (i < lines.size()) ? lines[i] : "";
    // Keep the UI text snapshot in sync regardless of LCD redraw.
    ui_set_visible_text_line((int)i, text);
    if (force_redraw || text != s_last_gps_lines[i]) {
      s_last_gps_lines[i] = text;
      int y = start_y + i * line_h;
      M5.Display.fillRect(0, y, 240, line_h, TFT_BLACK);
      if (!text.empty()) {
        M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        M5.Display.setCursor(0, y);
        M5.Display.printf("%s", text.c_str());
      }
    }
  }
  M5.Display.endWrite();
}

static void draw_status_view() {
  std::string lines[6];
  lines[0] = status_sync_line();
  {
    char fbuf[16];
    float f = g_bands[g_band_sel].freq;
    if (f == (int)f) snprintf(fbuf, sizeof(fbuf), "%d", (int)f);
    else             snprintf(fbuf, sizeof(fbuf), "%.1f", f);
    lines[1] = std::string("Band: ") + std::string(g_bands[g_band_sel].name) + " " + fbuf;
  }
  lines[2] = std::string("Tune: ") + (g_tune ? "ON" : "OFF") +
             "  G:" + std::to_string(ic705_tx_get_gain_q8());
  if (status_edit_idx == 3 && !status_edit_buffer.empty()) {
    lines[3] = std::string("Date: ") + highlight_pos(status_edit_buffer, status_cursor_pos);
  } else {
    lines[3] = std::string("Date: ") + g_date;
  }
  if (status_edit_idx == 4 && !status_edit_buffer.empty()) {
    lines[4] = std::string("Time: ") + highlight_pos(status_edit_buffer, status_cursor_pos);
  } else {
    lines[4] = std::string("Time: ") + g_time + rtc_time_source_suffix();
  }
  lines[5] = "Disconnect 705";
  for (int i = 0; i < 6; ++i) {
    bool hl = (status_edit_idx == i);
    draw_status_line(i, lines[i], hl);
  }
}

static bool perf_idle_hook_cpu0() {
  g_perf_idle_count[0] = g_perf_idle_count[0] + 1u;
  return true;
}

static bool perf_idle_hook_cpu1() {
  g_perf_idle_count[1] = g_perf_idle_count[1] + 1u;
  return true;
}

static uint8_t perf_busy_percent(uint32_t idle_delta, TickType_t tick_delta) {
  if (tick_delta == 0) return 0;
  uint32_t idle_pct = ((idle_delta * 100u) + ((uint32_t)tick_delta / 2u)) / (uint32_t)tick_delta;
  if (idle_pct > 100u) idle_pct = 100u;
  return (uint8_t)(100u - idle_pct);
}

static void perf_monitor_sample(TickType_t now_ticks) {
  if (g_perf_prev_sample_tick == 0) {
    g_perf_prev_sample_tick = now_ticks;
    g_perf_prev_idle_count[0] = g_perf_idle_count[0];
    g_perf_prev_idle_count[1] = g_perf_idle_count[1];
    return;
  }

  TickType_t tick_delta = now_ticks - g_perf_prev_sample_tick;
  if (tick_delta == 0) return;

  for (int core = 0; core < 2; ++core) {
    uint32_t idle_now = g_perf_idle_count[core];
    uint32_t idle_delta = idle_now - g_perf_prev_idle_count[core];
    g_perf_prev_idle_count[core] = idle_now;
    if (g_perf_cpu_hook_ok[core]) {
      g_perf_cpu_busy_pct[core] = perf_busy_percent(idle_delta, tick_delta);
    }
  }

  g_perf_prev_sample_tick = now_ticks;
  g_perf_cpu_sample_valid = g_perf_cpu_hook_ok[0] || g_perf_cpu_hook_ok[1];
}

static void perf_monitor_init() {
  static bool initialized = false;
  if (initialized) return;

  esp_err_t err0 = esp_register_freertos_idle_hook_for_cpu(perf_idle_hook_cpu0, 0);
  if (err0 == ESP_OK) {
    g_perf_cpu_hook_ok[0] = true;
  } else {
    ESP_LOGW(TAG, "CPU0 perf idle hook failed: %s", esp_err_to_name(err0));
  }

#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
  esp_err_t err1 = esp_register_freertos_idle_hook_for_cpu(perf_idle_hook_cpu1, 1);
  if (err1 == ESP_OK) {
    g_perf_cpu_hook_ok[1] = true;
  } else {
    ESP_LOGW(TAG, "CPU1 perf idle hook failed: %s", esp_err_to_name(err1));
  }
#endif

  g_perf_prev_sample_tick = xTaskGetTickCount();
  g_perf_prev_idle_count[0] = g_perf_idle_count[0];
  g_perf_prev_idle_count[1] = g_perf_idle_count[1];
  initialized = true;
}

static uint16_t perf_color_for_pct(uint8_t pct) {
  if (pct >= 85) return TFT_RED;
  if (pct >= 65) return TFT_YELLOW;
  return TFT_GREEN;
}

static unsigned perf_kib_rounded(size_t bytes) {
  return (unsigned)((bytes + 512u) / 1024u);
}

static uint8_t perf_heap_used_pct(uint32_t caps, size_t free_bytes) {
  size_t total = heap_caps_get_total_size(caps);
  if (total == 0 || free_bytes >= total) return 0;
  return (uint8_t)(((total - free_bytes) * 100u + (total / 2u)) / total);
}

static void perf_make_cpu_line(char* out, size_t out_len, int core) {
  char bar[9];
  uint8_t pct = g_perf_cpu_busy_pct[core];
  int filled = g_perf_cpu_sample_valid && g_perf_cpu_hook_ok[core] ? (pct * 8 + 50) / 100 : 0;
  if (filled < 0) filled = 0;
  if (filled > 8) filled = 8;
  for (int i = 0; i < 8; ++i) bar[i] = (i < filled) ? '#' : '-';
  bar[8] = '\0';

  if (g_perf_cpu_sample_valid && g_perf_cpu_hook_ok[core]) {
    snprintf(out, out_len, "C%d %3u%% [%s]", core, (unsigned)pct, bar);
  } else {
    snprintf(out, out_len, "C%d --%% [%s]", core, bar);
  }
}

static void perf_make_heap_line(char* out, size_t out_len, const char* label, uint32_t caps) {
  size_t free_bytes = heap_caps_get_free_size(caps);
  size_t largest = heap_caps_get_largest_free_block(caps);
  uint8_t used_pct = perf_heap_used_pct(caps, free_bytes);
  snprintf(out, out_len, "%s %3u%% F%uK L%uK",
           label,
           (unsigned)used_pct,
           perf_kib_rounded(free_bytes),
           perf_kib_rounded(largest));
}

static void draw_perf_view(bool force_redraw) {
  static char last_lines[6][32] = {{0}};
  char lines[6][32];
  uint16_t colors[6] = {
      perf_color_for_pct(g_perf_cpu_busy_pct[0]),
      perf_color_for_pct(g_perf_cpu_busy_pct[1]),
      TFT_WHITE,
      TFT_WHITE,
      TFT_WHITE,
      TFT_WHITE,
  };

  perf_make_cpu_line(lines[0], sizeof(lines[0]), 0);
  perf_make_cpu_line(lines[1], sizeof(lines[1]), 1);
  perf_make_heap_line(lines[2], sizeof(lines[2]), "8B", MALLOC_CAP_8BIT);
  perf_make_heap_line(lines[3], sizeof(lines[3]), "IN", MALLOC_CAP_INTERNAL);
  perf_make_heap_line(lines[4], sizeof(lines[4]), "DM", MALLOC_CAP_DMA);
  snprintf(lines[5], sizeof(lines[5]), "ST C%uK M%uK",
           perf_kib_rounded(g_app_core0_stack_cur_free_bytes),
           perf_kib_rounded(g_app_core0_stack_min_free_bytes));

  const int line_h = 19;
  const int start_y = UI_START_Y;
  M5.Display.startWrite();
  M5.Display.setTextSize(2);
  for (int i = 0; i < 6; ++i) {
    ui_set_visible_text_line(i, lines[i]);
    if (force_redraw || strcmp(last_lines[i], lines[i]) != 0) {
      snprintf(last_lines[i], sizeof(last_lines[i]), "%s", lines[i]);
      int y = start_y + i * line_h;
      M5.Display.fillRect(0, y, 240, line_h, TFT_BLACK);
      M5.Display.setTextColor(colors[i], TFT_BLACK);
      M5.Display.setCursor(0, y);
      M5.Display.printf("%s", lines[i]);
    }
  }
  M5.Display.endWrite();
}

static void debug_ensure_hud_lines() {
  while (g_debug_lines.size() < DEBUG_HUD_LINES) {
    g_debug_lines.emplace_back();
  }
}

static void debug_update_app_core0_stack_hud(bool redraw_now) {
  debug_ensure_hud_lines();
  char cur_line[20];
  char min_line[20];
  std::snprintf(cur_line, sizeof(cur_line), "Acur %luB",
                (unsigned long)g_app_core0_stack_cur_free_bytes);
  std::snprintf(min_line, sizeof(min_line), "Amin %luB",
                (unsigned long)g_app_core0_stack_min_free_bytes);
  g_debug_lines[0] = cur_line;
  g_debug_lines[1] = min_line;
  (void)redraw_now;
}

static void debug_log_line(const std::string& msg) {
  debug_ensure_hud_lines();
  if (g_debug_lines.size() >= DEBUG_MAX_LINES) {
    if (g_debug_lines.size() > DEBUG_HUD_LINES) {
      g_debug_lines.erase(g_debug_lines.begin() + DEBUG_HUD_LINES);
    } else {
      return;
    }
  }
  g_debug_lines.push_back(msg);
  debug_page = (int)((g_debug_lines.size() - 1) / 6);
}

static std::string trim_copy(const std::string& s) {
  size_t b = 0, e = s.size();
  while (b < e && isspace((unsigned char)s[b])) ++b;
  while (e > b && isspace((unsigned char)s[e - 1])) --e;
  return s.substr(b, e - b);
}

static void ascii_upper_inplace(std::string& s) {
  for (auto& ch : s) {
    ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
  }
}

static std::string trim_upper_copy(const std::string& s) {
  std::string out = trim_copy(s);
  ascii_upper_inplace(out);
  return out;
}

static uint32_t parse_crc_hex(const std::string& hex) {
  if (hex.empty()) return 0;
  char* end = nullptr;
  unsigned long v = strtoul(hex.c_str(), &end, 16);
  if (end == hex.c_str() || *end != '\0') return 0;
  return (uint32_t)v;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  crc = crc ^ 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

static void host_debug_hex8(const char* prefix, const uint8_t* b) {
  char buf[64];
  int n = snprintf(buf, sizeof(buf), "%s ", prefix);
  for (int i = 0; i < 8 && n + 3 < (int)sizeof(buf); ++i) {
    n += snprintf(buf + n, sizeof(buf) - n, "%02X ", b[i]);
  }
  if (n > 0 && buf[n - 1] == ' ') buf[n - 1] = 0;
  host_write_str(std::string(buf) + "\r\n");
}

static void host_handle_line(const std::string& line_in) {
  bool send_prompt = true;
  std::string line = trim_copy(line_in);
  if (line.empty()) { /* host_write_str(HOST_PROMPT);*/ return; }
  debug_log_line(std::string("[HOST RX] ") + line);
  //std::string echo = std::string("ECHO: ") + line + "\r\n";
  //host_write_str(echo);

  auto to_upper = [](std::string s) {
    for (auto& c : s) c = toupper((unsigned char)c);
    return s;
  };
  std::istringstream iss(line);
  std::string cmd;
  iss >> cmd;
  std::string cmd_up = to_upper(cmd);
  std::string rest;
  std::getline(iss, rest);
  rest = trim_copy(rest);

  auto send = [](const std::string& msg) { host_write_str(msg + "\r\n"); };

  if (cmd_up == "WRITE" || cmd_up == "APPEND") {
    std::istringstream rs(rest);
    std::string fname;
    rs >> fname;
    std::string content;
    std::getline(rs, content);
    content = trim_copy(content);
    if (fname.empty()) {
      send("ERROR: filename required");
    } else if (cmd_up == "WRITE" && storage_reject_active_log_user_mutation(fname)) {
      send("ERROR: active log protected");
    } else {
      if (cmd_up == "WRITE") {
        send(storage_file_write_atomic(fname, content) ? "OK" : "ERROR: write failed");
      } else {
        send(storage_file_append(fname, content, "", true) ? "OK" : "ERROR: write failed");
      }
    }
  } else if (cmd_up == "READ") {
    if (rest.empty()) send("ERROR: filename required");
    else {
      StorageStream* stream = storage_stream_open(rest, StorageOpenMode::READ);
      if (!stream) send("ERROR: open failed");
      else {
        char buf[128];
        while (storage_stream_read_line(stream, buf, sizeof(buf))) {
          host_write_str(std::string(buf));
        }
        storage_stream_close(stream);
        send_prompt = false;
      }
    }
  } else if (cmd_up == "DELETE") {
    if (rest.empty()) send("ERROR: filename required");
    else if (storage_reject_active_log_user_mutation(rest)) send("ERROR: active log protected");
    else {
      if (storage_file_remove(rest)) send("OK"); else send("ERROR: delete failed");
    }
  } else if (cmd_up == "LIST") {
    std::vector<std::string> files;
    if (!storage_file_list(files)) send("ERROR: storage unavailable");
    else {
      for (const auto& file : files) send(file);
      send("OK");
    }
  } else if (cmd_up == "WRITEBIN") {
    std::istringstream rs(rest);
    std::string fname;
    size_t size = 0;
    std::string crc_hex;
    rs >> fname >> size >> crc_hex;
    uint32_t crc_exp = parse_crc_hex(crc_hex);
    if (fname.empty() || size == 0 || crc_hex.empty()) {
      send("ERROR: filename, size, crc32_hex required");
    } else if (host_bin_active) {
      send("ERROR: binary upload in progress");
    } else if (storage_reject_active_log_user_mutation(fname)) {
      send("ERROR: active log protected");
    } else {
      StorageStream* stream = storage_stream_open(fname, StorageOpenMode::WRITE_TRUNCATE);
      if (!stream) {
        send("ERROR: open failed");
      } else {
          host_bin_path = fname;
          host_bin_active = true;
          host_bin_remaining = size;
          host_bin_stream = stream;
          host_bin_crc = 0;
          host_bin_expected_crc = crc_exp;
          host_bin_received = 0;
          host_bin_buf.clear();
          host_bin_buf.reserve(HOST_BIN_CHUNK);
          host_bin_chunk_expect = (host_bin_remaining < HOST_BIN_CHUNK) ? host_bin_remaining : HOST_BIN_CHUNK;
          host_bin_first_filled = 0;
          memset(host_bin_first8, 0, sizeof(host_bin_first8));
          memset(host_bin_last8, 0, sizeof(host_bin_last8));
          host_write_str("OK: send " + std::to_string(size) + " bytes, chunk " + std::to_string(HOST_BIN_CHUNK) + " +4crc\r\n");
          send_prompt = false; // prompt after binary upload completes
      }
    }
  } else if (cmd_up == "DATE") {
    if (rest.empty()) {
      send("DATE " + g_date);
    } else {
      int y, M, d;
      if (sscanf(rest.c_str(), "%d-%d-%d", &y, &M, &d) != 3 ||
          y < 2024 || y > 2099 || M < 1 || M > 12 || d < 1 || d > 31) {
        send("ERROR: use DATE YYYY-MM-DD");
      } else {
        char buf[16];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d", y, M, d);
        g_date = buf;
        if (rtc_apply_manual_time_from_strings()) { save_station_data(); send("OK"); }
        else send("ERROR: invalid date");
      }
    }
  } else if (cmd_up == "TIME") {
    if (rest.empty()) {
      send("TIME " + g_time);
    } else {
      int h, m, s;
      if (sscanf(rest.c_str(), "%d:%d:%d", &h, &m, &s) != 3 ||
          h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) {
        send("ERROR: use TIME HH:MM:SS");
      } else {
        char buf[16];
        snprintf(buf, sizeof(buf), "%02d:%02d:%02d", h, m, s);
        g_time = buf;
        if (rtc_apply_manual_time_from_strings()) { save_station_data(); send("OK"); }
        else send("ERROR: invalid time");
      }
    }
  } else if (cmd_up == "SLEEP") {
    if (rtc_valid) {
      // Compute current time in milliseconds, round up to next second boundary
      int64_t elapsed_ms = esp_timer_get_time() / 1000 - rtc_ms_start;
      int64_t now_ms = (time_t)rtc_epoch_base * 1000LL + elapsed_ms;
      int64_t frac = now_ms % 1000;
      int64_t wait_ms = (frac > 0) ? (1000 - frac) : 0;
      time_t sleep_epoch = (time_t)((now_ms + 999) / 1000);  // ceil to next second

      g_rtc_sleep_epoch = sleep_epoch;
      save_station_data();

      // Wait until the second boundary, then set ESP RTC and sleep
      if (wait_ms > 0) vTaskDelay(pdMS_TO_TICKS(wait_ms));
      struct timeval tv = { .tv_sec = sleep_epoch, .tv_usec = 0 };
      settimeofday(&tv, NULL);
    }
    send("OK: entering deep sleep");
    M5.Display.sleep();
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
    esp_deep_sleep_start();
  } else if (cmd_up == "INFO") {
    send("Heap: " + std::to_string(heap_caps_get_free_size(MALLOC_CAP_DEFAULT)));
    send("OK");
  } else if (cmd_up == "HELP") {
    for (auto& l : g_host_help_lines) send(l);
  } else if (cmd_up == "EXIT") {
    send("OK: exit host");
    enter_mode(UIMode::RX);
    return;
  } else {
    send("ERROR: Unknown command. Type HELP.");
  }

  if (send_prompt) host_write_str(std::string(HOST_PROMPT));
}

static void host_bin_close_release() {
  if (host_bin_stream) {
    storage_stream_sync(host_bin_stream);
    storage_stream_close(host_bin_stream);
    host_bin_stream = nullptr;
  }
  host_bin_active = false;
  host_bin_remaining = 0;
  host_bin_buf.clear();
}

static void host_process_bytes(const uint8_t* buf, size_t len) {
  ESP_LOGD(TAG, "host_process_bytes len=%u", (unsigned)len);
  for (size_t i = 0; i < len; ) {
    if (host_bin_active) {
      // Skip any stray CR/LF before first payload byte
      if (host_bin_received == 0 && host_bin_buf.empty() && (buf[i] == '\r' || buf[i] == '\n')) {
        ++i;
        continue;
      }
      size_t payload_need = host_bin_chunk_expect;
      size_t total_need = payload_need + 4; // payload + crc32 trailer
      size_t avail = len - i;
      size_t copy = total_need - host_bin_buf.size();
      if (copy > avail) copy = avail;
      host_bin_buf.insert(host_bin_buf.end(), buf + i, buf + i + copy);
      i += copy;

      if (host_bin_buf.size() >= total_need) {
        size_t payload_len = payload_need;
        uint32_t recv_crc = (uint32_t(host_bin_buf[payload_len])) |
                            (uint32_t(host_bin_buf[payload_len + 1]) << 8) |
                            (uint32_t(host_bin_buf[payload_len + 2]) << 16) |
                            (uint32_t(host_bin_buf[payload_len + 3]) << 24);
        uint32_t calc_crc = crc32_update(0, host_bin_buf.data(), payload_len);
        if (calc_crc != recv_crc) {
          char dbg[128];
          snprintf(dbg, sizeof(dbg), "ERROR: chunk crc off=%u len=%u calc=%08X recv=%08X\r\n",
                   (unsigned)(host_bin_received + payload_len), (unsigned)payload_len,
                   (unsigned)calc_crc, (unsigned)recv_crc);
          host_write_str(std::string(dbg));
          // Send first/last bytes of the chunk to compare
          if (payload_len >= 8) host_debug_hex8("DBG CHUNK FIRST8", host_bin_buf.data());
          if (payload_len >= 8) host_debug_hex8("DBG CHUNK LAST8", host_bin_buf.data() + payload_len - 8);
          if (payload_len < 8) host_debug_hex8("DBG CHUNK PART", host_bin_buf.data());
          // Also report the CRC trailer bytes as seen
          uint8_t crc_bytes[4] = {
            host_bin_buf[payload_len],
            host_bin_buf[payload_len + 1],
            host_bin_buf[payload_len + 2],
            host_bin_buf[payload_len + 3]
          };
          host_debug_hex8("DBG CRC BYTES", crc_bytes);
          host_bin_close_release();
          host_write_str(std::string(HOST_PROMPT));
          continue;
        }

        // Capture first/last bytes for debugging
        if (host_bin_first_filled < 8) {
          size_t need = 8 - host_bin_first_filled;
          if (need > payload_len) need = payload_len;
          memcpy(host_bin_first8 + host_bin_first_filled, host_bin_buf.data(), need);
          host_bin_first_filled += need;
        }
        // update last8 buffer
        if (payload_len >= 8) {
          memcpy(host_bin_last8, host_bin_buf.data() + payload_len - 8, 8);
        } else {
          // shift existing and append
          size_t shift = payload_len;
          if (shift > 0) {
            memmove(host_bin_last8, host_bin_last8 + shift, 8 - shift);
            memcpy(host_bin_last8 + (8 - payload_len), host_bin_buf.data(), payload_len);
          }
        }

        size_t written = storage_stream_write(host_bin_stream, host_bin_buf.data(), payload_len);
        if (written != payload_len) {
          host_write_str("ERROR: write failed\r\n");
          host_bin_close_release();
          host_write_str(std::string(HOST_PROMPT));
          continue;
        }
        host_bin_crc = crc32_update(host_bin_crc, host_bin_buf.data(), payload_len);
        host_bin_remaining -= payload_len;
        host_bin_received += payload_len;
        host_bin_buf.clear();
        host_write_str("ACK " + std::to_string(host_bin_received) + "\r\n");

        if (host_bin_remaining == 0) {
          uint32_t crc_final = host_bin_crc;
          host_bin_close_release();
          // Reopen file to send first/last 8 bytes for debugging
          host_debug_hex8("DBG FIRST8", host_bin_first8);
          host_debug_hex8("DBG LAST8", host_bin_last8);
          char crc_line[64];
          snprintf(crc_line, sizeof(crc_line), "DBG CRC %08X EXPECT %08X\r\n",
                   (unsigned)crc_final, (unsigned)host_bin_expected_crc);
          host_write_str(std::string(crc_line));
          if (crc_final != host_bin_expected_crc) {
            host_write_str("ERROR: crc mismatch\r\n");
          } else {
            host_write_str("OK crc " + std::to_string(crc_final) + "\r\n");
          }
          host_write_str(std::string(HOST_PROMPT));
        } else {
          host_bin_chunk_expect = (host_bin_remaining < HOST_BIN_CHUNK) ? host_bin_remaining : HOST_BIN_CHUNK;
        }
      }
      continue;
    }
    char ch = (char)buf[i++];
    if (ch == '\r' || ch == '\n') {
      if (!host_input.empty()) {
    //ESP_LOGI(TAG, "HOST line: %s", host_input.c_str());
        host_handle_line(host_input);
        host_input.clear();
      } else {
        //host_write_str(std::string(HOST_PROMPT));
      }
    } else if (ch == 0x08 || ch == 0x7f) {
      if (!host_input.empty()) host_input.pop_back();
    } else if (ch >= 32 && ch < 127) {
      host_input.push_back(ch);
    }
  }
}

[[maybe_unused]] static void poll_host_uart() {
  ensure_usb();
  if (!usb_ready) return;
  uint8_t buf[512];
  while (true) {
    int r = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0);
    if (r <= 0) break;
    host_process_bytes(buf, (size_t)r);
  }
}

// --- Config persistence in NVS ---------------------------------------------
// The internal FATFS partition doesn't exist on this board and the SD card's
// writes fail at the driver level, so the whole Station.txt content is stored as
// one blob in NVS (the standard ESP key/value store, always available). SD/flash
// are kept only as best-effort secondaries.
static const char* kNvsNamespace = "cp705";
static const char* kNvsStationKey = "station";

static bool nvs_save_station(const std::string& content) {
  nvs_handle_t h;
  if (nvs_open(kNvsNamespace, NVS_READWRITE, &h) != ESP_OK) return false;
  esp_err_t e = nvs_set_blob(h, kNvsStationKey, content.data(), content.size());
  if (e == ESP_OK) e = nvs_commit(h);
  nvs_close(h);
  return e == ESP_OK;
}

static bool nvs_load_station(std::string& out) {
  out.clear();
  nvs_handle_t h;
  if (nvs_open(kNvsNamespace, NVS_READONLY, &h) != ESP_OK) return false;
  size_t len = 0;
  esp_err_t e = nvs_get_blob(h, kNvsStationKey, nullptr, &len);
  if (e != ESP_OK || len == 0) { nvs_close(h); return false; }
  out.resize(len);
  e = nvs_get_blob(h, kNvsStationKey, &out[0], &len);
  nvs_close(h);
  return e == ESP_OK;
}

// --- Per-band TX gain persistence (NVS) -------------------------------------
// The clean-TX drive level differs per band, so remember one per band and
// reload it whenever g_band_sel changes (see apply_band_gain_for_current()'s
// two call sites in rebuild_active_bands()/advance_active_band()). Matches
// TD705's "g_<bandname>" key scheme in the same NVS namespace used for the
// rest of this app's config.
static void band_gain_save(int band_idx, int gain) {
  if (band_idx < 0 || band_idx >= (int)g_bands.size()) return;
  nvs_handle_t h;
  if (nvs_open(kNvsNamespace, NVS_READWRITE, &h) != ESP_OK) return;
  char key[16];
  snprintf(key, sizeof(key), "g_%s", g_bands[band_idx].name);
  nvs_set_u8(h, key, (uint8_t)gain);
  nvs_commit(h);
  nvs_close(h);
}

static int band_gain_load(int band_idx) {  // stored gain, or -1 if none saved
  if (band_idx < 0 || band_idx >= (int)g_bands.size()) return -1;
  nvs_handle_t h;
  if (nvs_open(kNvsNamespace, NVS_READONLY, &h) != ESP_OK) return -1;
  char key[16];
  snprintf(key, sizeof(key), "g_%s", g_bands[band_idx].name);
  uint8_t v = 0;
  esp_err_t e = nvs_get_u8(h, key, &v);
  nvs_close(h);
  return (e == ESP_OK) ? (int)v : -1;
}

static void apply_band_gain_for_current() {
  int g = band_gain_load(g_band_sel);
  if (g > 0) ic705_tx_set_gain_q8(g);
  // else: no saved value for this band yet -- leave the current gain alone.
}

// ADIF logging (NVS store + verified SD export) now lives in qso_log.cpp.

static void split_into_lines(const std::string& content, std::vector<std::string>& lines) {
  size_t start = 0;
  while (start <= content.size()) {
    size_t nl = content.find('\n', start);
    std::string s = (nl == std::string::npos) ? content.substr(start)
                                               : content.substr(start, nl - start);
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n')) s.pop_back();
    if (!s.empty()) lines.push_back(s);
    if (nl == std::string::npos) break;
    start = nl + 1;
  }
}

// Reads Station.txt as a list of lines, preferring NVS (reliable), then the SD
// card, then the internal flash FATFS. Trailing CR/LF stripped, blank lines
// dropped. Returns false if no source has it.
static bool read_station_lines(std::vector<std::string>& lines) {
  lines.clear();
  std::string content;
  if (nvs_load_station(content) && !content.empty()) {
    split_into_lines(content, lines);
    if (!lines.empty()) return true;
  }
  if (storage_sd_read_file(STATION_FILE, content) && !content.empty()) {
    split_into_lines(content, lines);
    return !lines.empty();
  }
  // Fallback: internal flash.
  StorageStream* stream = storage_stream_open(STATION_FILE, StorageOpenMode::READ);
  if (!stream) return false;
  char buf[256];
  while (storage_stream_read_line(stream, buf, sizeof(buf))) {
    std::string s(buf);
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n')) s.pop_back();
    if (!s.empty()) lines.push_back(s);
  }
  storage_stream_close(stream);
  return !lines.empty();
}

static void load_station_data() {
  // NOTE: do NOT call storage_sync_station_from_sd() here. It mounts the SD then
  // unmounts it (frees the SPI bus) before the SD log-mount is pinned, and the
  // immediate remount in read_station_lines() could fail and strand the load on
  // the (unavailable) flash. Config is read straight from the SD card below.

  // Load-time defaults for runtime settings.
  g_rtc_comp = kRtcCompFixed;
  g_autoseq_max_retry = AUTOSEQ_MAX_RETRY;
  g_brightness_step = 10;
  g_gps_baud = 115200;
  g_grid_saved_manual = g_grid;
  g_grid_from_gps = false;
  g_grid_gps_display8.clear();

  {
    std::vector<std::string> cfg_lines;
    if (!read_station_lines(cfg_lines)) {
      autoseq_set_max_retry(g_autoseq_max_retry);
      return;
    }

#if ENABLE_FT4
    // Pass 1: detect protocol_mode so we can set correct band defaults before
    // the full parse overwrites them.  Band entries are written before
    // protocol_mode in Station.txt, so a single-pass parse would load FT8
    // frequencies and never correct them when switching to FT4.
    {
      for (const std::string& cl : cfg_lines) {
        const char* line1 = cl.c_str();
        if (strncmp(line1, "protocol_mode=", 14) == 0) {
          char mode[8] = {};
          sscanf(line1 + 14, "%7s", mode);
          if (strcmp(mode, "FT4") == 0) {
            g_protocol = &kProtocolFT4;
            // Reset band frequencies to FT4 defaults.  The full parse below
            // will overwrite individual entries if the user has saved custom
            // FT4 frequencies (band0=…, band1=…, …).
            g_bands = {
              {"160m", 1843.0f}, {"80m",  3575.0f}, {"60m",  5357.0f}, {"40m",  7047.5f},
              {"30m", 10140.0f}, {"20m", 14080.0f}, {"17m", 18104.0f}, {"15m", 21140.0f},
              {"12m", 24919.0f}, {"10m", 28180.0f}, {"6m",  50318.0f}, {"2m", 144170.0f},
            };
            ESP_LOGI(TAG, "Station.txt: protocol_mode=FT4 — reset bands to FT4 defaults");
          }
          break;
        }
      }
    }
#endif  // ENABLE_FT4

    // Pass 2 (or only pass when ENABLE_FT4=0): full field parse.
    for (const std::string& cl : cfg_lines) {
      char line[256];
      strncpy(line, cl.c_str(), sizeof(line) - 1);
      line[sizeof(line) - 1] = '\0';
      int idx = -1;
      int val = 0;
      float fval = 0.0f;
#if ENABLE_FT4
      // Per-protocol band keys: FT4 uses "ft4_band%d=", FT8 uses "band%d=".
      // This prevents FT4 frequencies (e.g. 14080) from overwriting FT8
      // defaults (e.g. 14074) when switching protocol and rebooting.
      const bool is_ft4_boot = (g_protocol == &kProtocolFT4);
      const int band_parse_ok = is_ft4_boot
          ? sscanf(line, "ft4_band%d=%f", &idx, &fval)
          : sscanf(line, "band%d=%f",     &idx, &fval);
      if (band_parse_ok == 2) {
#else
      if (sscanf(line, "band%d=%f", &idx, &fval) == 2) {
#endif
      if (idx >= 0 && idx < (int)g_bands.size()) {
        g_bands[idx].freq = fval;
      }
    } else if (sscanf(line, "offset=%d", &val) == 1) {
      g_offset_hz = val;
    } else if (sscanf(line, "band_sel=%d", &val) == 1) {
      if (val >= 0 && val < (int)g_bands.size()) g_band_sel = val;
    } else if (sscanf(line, "date=%63s", line) == 1) {
      g_date = line;
    } else if (sscanf(line, "time=%63s", line) == 1) {
      g_time = normalize_time_hms(line);
    } else if (sscanf(line, "cq_type=%d", &val) == 1) {
      if (val >= 0 && val <= 5) g_cq_type = (CqType)val;
    } else if (sscanf(line, "offset_src=%d", &val) == 1) {
      if (val >= 0 && val <= 2) g_offset_src = (OffsetSrc)val;
    } else if (strncmp(line, "ic705_host=", 11) == 0) {
      std::string h = trim_copy(line + 11);
      if (!h.empty()) g_ic705_hostname = h;
    } else if (sscanf(line, "ic705_civ_addr=%d", &val) == 1) {
      if (val >= 0x00 && val <= 0xFF) g_ic705_civ_addr = val;
    } else if (strncmp(line, "call=", 5) == 0) {
      g_call = trim_upper_copy(line + 5);
    } else if (strncmp(line, "grid=", 5) == 0) {
      std::string g = trim_upper_copy(line + 5);
      if (!g.empty()) { g_grid = g; g_grid_saved_manual = g; }
    } else if (strncmp(line, "ic705_wifi_ssid=", 16) == 0) {
      // Credentials are case-sensitive: trim only surrounding whitespace/newline.
      g_ic705_wifi_ssid = trim_copy(line + 16);
    } else if (strncmp(line, "ic705_wifi_pass=", 16) == 0) {
      g_ic705_wifi_pass = trim_copy(line + 16);
    } else if (strncmp(line, "ic705_net_user=", 15) == 0) {
      g_ic705_net_user = trim_copy(line + 15);
    } else if (strncmp(line, "ic705_net_pass=", 15) == 0) {
      g_ic705_net_pass = trim_copy(line + 15);
    } else if (sscanf(line, "gps_baud=%d", &val) == 1) {
      g_gps_baud = normalize_gps_baud_value(val);
    } else if (strncmp(line, "cq_ft=", 6) == 0) {
      g_cq_freetext = trim_upper_copy(line + 6);
    } else if (strncmp(line, "free_text=", 10) == 0) {
      g_free_text = trim_upper_copy(line + 10);
    } else if (sscanf(line, "skiptx1=%d", &val) == 1) {
      g_skip_tx1 = (val != 0); autoseq_set_skip_tx1(g_skip_tx1);
    } else if (sscanf(line, "active_band=%d", &val) == 1) { // legacy single value
      g_active_band_text = std::to_string(val);
    } else if (strncmp(line, "active_bands=", 13) == 0) {
      g_active_band_text = trim_upper_copy(line + 13);
    } else if (sscanf(line, "autoseq_max_retry=%d", &val) == 1) {
      if (val >= 0) g_autoseq_max_retry = val;
    } else if (sscanf(line, "brightness=%d", &val) == 1) {
      if (val >= 1 && val <= 10) g_brightness_step = val;
    } else if (strncmp(line, "protocol_mode=", 14) == 0) {
      // Already handled in pass 1 above (g_protocol + band defaults set there).
      // Nothing to do here in pass 2.
      (void)0;
    } else if (sscanf(line, "rtc_comp=%d", &val) == 1) {
      g_rtc_comp = clamp_rtc_comp_value(val);
    } else {
      long long epoch_tmp = 0;
      if (sscanf(line, "rtc_sleep_epoch=%lld", &epoch_tmp) == 1) {
        g_rtc_sleep_epoch = (time_t)epoch_tmp;
      }
    }
    }
  }
  autoseq_set_max_retry(g_autoseq_max_retry);
  // Prefer an external DS3231 when present, then ESP RTC/deep-sleep
  // compensation, then the saved Station.txt strings.
  if (!rtc_init_from_ds3231() && !rtc_init_from_esp_rtc()) {
    ESP_LOGI(TAG, "No valid DS3231 or ESP RTC time; using saved time strings");
    rtc_set_from_strings_source(RtcTimeSource::SAVED);
  }
  rebuild_active_bands();
  g_cq_running = false; // never resume a running CQ across a reboot
#if ENABLE_FT4
  g_protocol_pending_ft4 = (g_protocol == &kProtocolFT4);
#endif
}

void save_station_data() {
  std::ostringstream out;
#if ENABLE_FT4
  // Per-protocol band keys keep FT8 and FT4 frequencies independent so that
  // switching protocol (reboot-to-apply) doesn't cross-contaminate band lists.
  const char* band_prefix = (g_protocol == &kProtocolFT4) ? "ft4_band" : "band";
#else
  const char* band_prefix = "band";
#endif
  for (size_t i = 0; i < g_bands.size(); ++i) {
    char fbuf[16];
    float f = g_bands[i].freq;
    if (f == (int)f) snprintf(fbuf, sizeof(fbuf), "%d", (int)f);
    else             snprintf(fbuf, sizeof(fbuf), "%.1f", f);
    out << band_prefix << (unsigned)i << "=" << fbuf << "\n";
  }
  out << "offset=" << g_offset_hz << "\n";
  out << "band_sel=" << g_band_sel << "\n";
  out << "date=" << g_date << "\n";
  out << "time=" << g_time << "\n";
  out << "cq_type=" << (int)g_cq_type << "\n";
  out << "cq_ft=" << g_cq_freetext << "\n";
  out << "skiptx1=" << (g_skip_tx1 ? 1 : 0) << "\n";
  out << "free_text=" << g_free_text << "\n";
  out << "call=" << g_call << "\n";
  out << "grid=" << g_grid_saved_manual << "\n";
  out << "offset_src=" << (int)g_offset_src << "\n";
  out << "ic705_wifi_ssid=" << g_ic705_wifi_ssid << "\n";
  out << "ic705_wifi_pass=" << g_ic705_wifi_pass << "\n";
  out << "ic705_net_user=" << g_ic705_net_user << "\n";
  out << "ic705_net_pass=" << g_ic705_net_pass << "\n";
  out << "ic705_host=" << g_ic705_hostname << "\n";
  out << "ic705_civ_addr=" << g_ic705_civ_addr << "\n";
  out << "gps_baud=" << normalize_gps_baud_value(g_gps_baud) << "\n";
  out << "active_bands=" << g_active_band_text << "\n";
  out << "rtc_sleep_epoch=" << (long long)g_rtc_sleep_epoch << "\n";
  out << "rtc_comp=" << g_rtc_comp << "\n";
  out << "autoseq_max_retry=" << g_autoseq_max_retry << "\n";
  out << "brightness=" << g_brightness_step << "\n";
#if ENABLE_FT4
  // Save the pending protocol mode (may differ from g_protocol if user toggled
  // Mode in the menu without rebooting yet).
  if (g_protocol_pending_ft4) {
    out << "protocol_mode=FT4\n";
  }
#endif
  const std::string content = out.str();
  // PRIMARY: NVS (always available; SD writes fail and the FATFS partition is
  // absent on this board). This is also the source of truth for load().
  const bool nvs_ok = nvs_save_station(content);
  // SECONDARY (best-effort): SD card, then internal flash, when they work.
  const bool sd_ok = storage_sd_write_file(STATION_FILE, content);
  bool flash_ok = false;
  if (storage_service_firmware_available()) {
    flash_ok = storage_file_write_atomic(STATION_FILE, content);
  }
  if (!nvs_ok && !sd_ok && !flash_ok) {
    ESP_LOGE(TAG, "Failed to persist %s to NVS/SD/flash", STATION_FILE);
  }
  // Every config mutation in the Cardputer UI funnels through here, so this
  // is the canonical place to notify core_api consumers.
  core_fire_config_changed();
}

static void enter_mode(UIMode new_mode) {
  if (ui_mode == UIMode::STATUS && new_mode != UIMode::STATUS) {
    status_edit_idx = -1;
    status_edit_buffer.clear();

    // Auto-sync VFO + RX mode on STATUS exit. Picks up any in-STATUS
    // changes (band advance via S->3, etc.) without needing a manual
    // "Sync to QMX" button press. Idempotent — safe even if the same
    // sync already fired (e.g. from S->3 in-menu push, or from the
    // initial-connect path for QMX).
    sync_radio_to_current_band("STATUS exit");
  }
  ui_mode = new_mode;
  rx_flash_idx = -1;
  switch (ui_mode) {
    case UIMode::RX:
      // Force RX list redraw
      ui_force_redraw_rx();
      ui_draw_rx();
      break;
    case UIMode::BAND:
      band_page = 0;
      band_edit_idx = -1;
      draw_band_view();
      break;
    case UIMode::MENU:
      menu_category = -1;
      menu_edit_idx = -1;
      menu_edit_buf.clear();
      g_q_clear_armed = false;
      g_q_clear_feedback.clear();
      draw_menu_view();
      break;
    case UIMode::STATUS:
      status_edit_idx = -1;
      status_cursor_pos = -1;
      draw_status_view();
      break;
    case UIMode::GPS:
      draw_gps_view(true);
      break;
    case UIMode::PERF:
      draw_perf_view(true);
      break;
  }
}


// Once WiFi reaches IC-705 once a manual '2' press has gotten WiFi up,
// automatically kick off (and retry) the CAT network login — without this,
// the login only ever starts/retries on another explicit keypress, even
// though the underlying connect attempt runs in the background regardless.
// Set true when the user gracefully disconnects (STATUS key 7) so the
// auto-reconnect below stays OFF until they explicitly press 2 again —
// otherwise it instantly re-logs-in and the radio never gets released.
// Cleared (and the auto-attempt counter reset) in begin_usb_host_mode().
static bool g_ic705_manual_disconnect = false;
static int g_ic705_auto_attempts = 0;
static TickType_t g_ic705_last_auto_attempt = 0;

// End-of-session SD export. The SD card mounts over SPI, which needs DMA-capable
// heap — and during operation WiFi + audio streaming + FT8 decode starve that
// pool, so the mount fails with 0x108 (ESP_ERR_INVALID_RESPONSE) and the export
// "fails" even though the QSOs are safe in NVS. The fix is to tear the whole
// radio stack DOWN first (stop audio/decode, release the radio session, drop
// WiFi), freeing that DMA memory, and only THEN mount + write the card. This is
// a one-way "I'm done operating, get my log off" action: reconnect needs a '2'
// press (or reboot). Returns the human result; also logs DMA headroom so we can
// see the teardown actually helped.
static std::string end_session_export_to_sd() {
  const size_t dma_before = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);

  // Full quiesce, mirroring the graceful-disconnect path (STATUS key 7).
  audio_source_stop();                 // stop RX/TX audio + decode tasks
  if (canonical_radio_type(g_radio) == RadioType::IC705) {
    g_ic705_manual_disconnect = true;  // stay disconnected (no auto-reconnect)
    ic705_cat_disconnect();            // release the radio's network session
  }
  ic705_stream_stop();                 // stop the WiFi audio stream task
  wifi_mgr_stop();                     // drop WiFi — the biggest DMA/heap holder
  g_streaming = false;
  vTaskDelay(pdMS_TO_TICKS(600));      // let the tasks unwind and free buffers

  const size_t dma_after = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
  std::string result = qso_log_export_to_sd(rtc_now_ms());
  // On failure show a SHORT line so the full error code fits the narrow display:
  // "SDfail c<code>" (2+esp_err = mount fail, -6 = EIO write, -1-errno = open).
  if (result.find("Verified") == std::string::npos) {
    result = "SDfail c" + std::to_string(g_storage_sd_log_last_code);
  }

  ESP_LOGW(TAG, "END-SESSION EXPORT: DMA largest %uK -> %uK, sd_code=%d, result=%s",
           (unsigned)(dma_before / 1024), (unsigned)(dma_after / 1024),
           g_storage_sd_log_last_code, result.c_str());
  debug_log_line(std::string("Export: ") + result +
                 " DMA" + std::to_string(dma_after / 1024) + "K");
  return result;
}

static void auto_advance_ic705_connect() {
  if (canonical_radio_type(g_radio) != RadioType::IC705) return;
  if (g_ic705_manual_disconnect) return;   // user asked to stay disconnected
  if (!wifi_mgr_is_ready()) return;
  if (radio_control_ready()) return;

  // Capped, not indefinite: repeated automatic login attempts over a long
  // session were observed to degrade the radio's own network-control state
  // to the point where even other clients (e.g. a separate app) couldn't
  // log in until the radio itself was power-cycled. Past this many auto
  // attempts, stop and require an explicit '2' press to try again — a
  // human deciding to retry is a very different load than an unattended
  // loop doing it every 12 seconds for hours.
  constexpr int kMaxAutoAttempts = 5;
  if (g_ic705_auto_attempts >= kMaxAutoAttempts) return;

  TickType_t now = xTaskGetTickCount();
  // Slow and patient on purpose: retrying too soon after a failed attempt
  // may not give the radio enough time to release the previous half-open
  // session before we start a fresh one, which can itself be the cause of
  // the next attempt failing.
  if (g_ic705_last_auto_attempt != 0 && (now - g_ic705_last_auto_attempt) < pdMS_TO_TICKS(12000)) return;
  g_ic705_last_auto_attempt = now;
  ++g_ic705_auto_attempts;

  ic705_net_set_credentials(g_ic705_net_user.c_str(), g_ic705_net_pass.c_str());
  ic705_cat_set_target(wifi_mgr_get_ic705_ip(), (uint8_t)g_ic705_civ_addr);

  // Deliberately NOT starting audio streaming here: it competes for the
  // same scarce DMA-capable memory the WiFi/network stack needs, and we've
  // measured that pool getting starved (down to single-digit free bytes)
  // once audio is running concurrently with WiFi traffic. Give the CAT
  // login a clear shot at that resource first; only start audio once CAT
  // is actually ready.
  notify_radio_control_audio_start_if_allowed("auto wifi ready/retry");
}

// Once CAT is up, start the audio stream that rides alongside it. Kept
// separate from auto_advance_ic705_connect() so audio never starts while
// CAT is still trying to log in.
// Once CAT is up, start the audio stream that rides alongside it.
// (Re-enabled with crash diagnostics — see the heap logging below and the
// BOOT reset-reason log. The previous unconditional-disable was a stopgap
// while CAT control was validated.)
static void auto_start_ic705_audio_once_cat_ready() {
  if (canonical_radio_type(g_radio) != RadioType::IC705) return;
  // ONE-SHOT per connection. Reset when CAT drops so the NEXT connection tries
  // again. Previously this ran every main-loop pass whenever audio wasn't
  // streaming — and when SID3 audio failed to come up (audio_ready=0) it
  // re-set g_ic705_initial_sync_pending forever, so sync_radio_to_current_band
  // (mode + TX-off) fired ~30x/sec: it flooded the radio with CI-V, flickered
  // the mode icon, jammed any TX keyup with constant TX-off, and helped crash
  // the box. Retrying audio_source_start_ic705 doesn't help anyway — SID3 is
  // established (or not) during the initial connect, not later.
  static bool s_attempted = false;
  if (!radio_control_ready()) { s_attempted = false; return; }
  if (audio_source_is_streaming()) { s_attempted = true; return; }
  if (s_attempted) return;
  // Wait for the audio session (SID3) to actually be ready before creating the
  // RX/TX audio tasks. SID3 can come up a beat AFTER CAT — netctrl now retries
  // the audio handshake from its own loop instead of giving up after one shot —
  // so do NOT latch s_attempted until audio is ready; keep checking each pass.
  // (No CI-V is sent on this path until we proceed, so this can't flood the radio.)
  if (!ic705_net_audio_is_ready()) return;
  s_attempted = true;
  ESP_LOGW(TAG, "AUDIO start: heap8=%u largest8=%u dma=%u dmaLargest=%u",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
  audio_source_start_ic705(wifi_mgr_get_ic705_ip());
  g_ic705_initial_sync_pending = true;
}

// Perform the STATUS -> '1' action: connect IC-705 (WiFi + audio + CAT).
// Syncs the selected band to the radio.
static void begin_usb_host_mode() {
  const bool on_status_page = (ui_mode == UIMode::STATUS);
  if (on_status_page) {
    status_edit_idx = 0;  // highlight the sync-status row while connecting
    draw_status_view();
  }

  const RadioType radio = canonical_radio_type(g_radio);

  if (radio == RadioType::IC705) {
    // Explicit user connect — clear any prior graceful-disconnect latch and
    // give the auto-reconnect its full retry budget again.
    g_ic705_manual_disconnect = false;
    g_ic705_auto_attempts = 0;
    g_ic705_last_auto_attempt = 0;
    // Start WiFi if not already running
    if (wifi_mgr_get_state() == WIFI_MGR_IDLE) {
      if (g_ic705_wifi_ssid.empty()) {
        debug_log_line("Set WiFi SSID in Settings (Menu pg3)");
      } else {
        wifi_mgr_start(g_ic705_wifi_ssid.c_str(),
                       g_ic705_wifi_pass.c_str(),
                       g_ic705_hostname.c_str());
        debug_log_line("WiFi connecting...");
      }
    }
    // Pass current IP and CI-V address to CAT backend
    if (wifi_mgr_is_ready()) {
      ic705_net_set_credentials(g_ic705_net_user.c_str(), g_ic705_net_pass.c_str());
    ic705_cat_set_target(wifi_mgr_get_ic705_ip(), (uint8_t)g_ic705_civ_addr);
    }
    // Audio streaming is deliberately NOT started here before CAT is ready
    // — it competes with the WiFi/network stack for the same scarce
    // DMA-capable memory pool (measured down to single-digit free bytes
    // under concurrent load), which was likely interfering with the CAT
    // login itself. auto_start_ic705_audio_once_cat_ready() in the main
    // loop starts it once radio_control_ready() is actually true.
    if (!radio_control_ready()) {
      notify_radio_control_audio_start_if_allowed("status key 2");
    }
  }

  int freq_hz = (int)(g_bands[g_band_sel].freq * 1000.0f);
  if (radio_control_ready()) {
    bool ok = (radio_control_sync_frequency_mode(freq_hz) == ESP_OK);
    debug_log_line(ok ? "CAT sync sent" : "CAT sync pending");
  } else {
    debug_log_line("CAT not ready yet");
  }

  if (on_status_page) {
    status_edit_idx = -1;
    draw_status_view();
  }
}

static void app_task_core0(void* /*param*/) {
  esp_err_t storage_err = storage_service_init();
  if (storage_err != ESP_OK) {
    ESP_LOGE(TAG, "Storage service init failed: %s; continuing without log/config storage",
             esp_err_to_name(storage_err));
    debug_log_line(std::string("Storage init fail: ") + esp_err_to_name(storage_err));
  }

  if (storage_service_firmware_available()) {
    storage_sync_station_from_sd();
  }
  board_power_init();
  // Radio selection is no longer user-exposed (menu toggle removed) — cp705
  // is IC-705 only.
  g_radio = RadioType::IC705;
  ui_init(radio_type_uses_display_only(g_radio));
  // Mount + pin the SD card now — AFTER display init (SPI-ordering safe) but
  // BEFORE load_station_data() reads Station.txt from it, and before WiFi/audio
  // exhaust DMA. Without this the config load did its own on-demand mount, which
  // could fail (leaving config blank) while the later log-premount succeeded.
  // mount_sd_locked() is idempotent, so the later log-premount call is a no-op.
  storage_sd_log_premount();   // best-effort; SD is a secondary log target
  hashtable_init();

  // Q15 NCO LUT for UAC OUT FT8 audio synthesis. One-time table fill,
  // ~514 B in BSS. Must run before the speaker pump task starts.
  dds_init();

  // Initialize autoseq engine
  autoseq_init();

  // Initialize the functional-core API (creates internal sync primitives).
  // After this, UI consumers can call core_get_*, core_cmd_*, and register
  // callbacks.
  core_init();

  // Register the Cardputer UI as a core_api consumer. The callbacks just set
  // the existing dirty flags — the UI main loop drains them on each tick.
  // Trivial handlers only (spec in docs/NATIVE_CLIENT_ARCHITECTURE.md).
  core_on_rx_changed    ([]{ g_rx_dirty = true; });
  // Also mark RX dirty: the hero card (shown in place of the decode list
  // while a QSO/CQ is active) needs to redraw on every QSO state change,
  // not just on new decodes.
  core_on_qso_changed   ([]{ g_rx_dirty = true; });
  // config changes redraw whatever view is showing them (MENU/STATUS);
  core_on_config_changed([]{ g_rx_dirty = true; });
  
autoseq_set_adif_callback(log_adif_entry);
autoseq_set_cabrillo_fd_callback(log_cabrillo_fd_entry);


  ui_mode = UIMode::RX;
  load_station_data();
  apply_brightness();
  // Seed the on-screen QSO counter from the durable NVS log so it reflects the
  // real logged count instead of resetting to 0 on every power-on.
  g_adif_sd_seq = (uint32_t)qso_log_count_nvs();
  apply_debug_uart_pin_policy();
  apply_radio_profile_binding();
  update_autoseq_cq_type();

  // Update autoseq with station info after loading
  autoseq_set_station(g_call, grid_ft8_4(g_grid));

  // Prepare RX list (but don't draw yet - startup screen may be shown)
  std::vector<UiRxLine> empty;
  ui_set_rx_list(empty);

  if (g_startup_active) {
    ui_draw_splash(g_call, kAppVersion);
  } else {
    ui_force_redraw_rx();
    ui_draw_rx();
  }

  ESP_LOGI(TAG, "Free heap: %u, internal: %u, 8bit: %u",
           heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           heap_caps_get_free_size(MALLOC_CAP_8BIT));
  {
    char buf[64];
    snprintf(buf, sizeof(buf), "Heap %u", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    debug_log_line(buf);
  }
  log_heap("BOOT");

  g_app_core0_stack_last_sample_tick = xTaskGetTickCount();
  {
    UBaseType_t free_words = uxTaskGetStackHighWaterMark2(NULL);
    uint32_t free_bytes = (uint32_t)free_words * (uint32_t)sizeof(StackType_t);
    g_app_core0_stack_cur_free_bytes = free_bytes;
    g_app_core0_stack_min_free_bytes = free_bytes;
    debug_update_app_core0_stack_hud(false);
  }
  perf_monitor_init();

  // Key injection queue for console UART RX
  s_key_inject_queue = xQueueCreate(32, sizeof(char));

  // sdkconfig puts the ESP console on UART0's default pins, but IDF's
  // console init only guarantees the TX pin routing — it doesn't always
  // hook up RX. Explicitly route the default RX GPIO to UART0 RXD. This
  // is a no-op if already set, and doesn't install a driver.
  uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE,
               (gpio_num_t)U0RXD_GPIO_NUM,
               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  // Drain any stale bytes left in the FIFO from ROM-bootloader time
  // (when UART0 RX was still on its IO_MUX default pin, likely floating).
  {
    uart_dev_t *hw = UART_LL_GET_HW(0);
    uint8_t scratch[64];
    while (uart_ll_get_rxfifo_len(hw) > 0) {
      uint32_t n = uart_ll_get_rxfifo_len(hw);
      if (n > 64) n = 64;
      uart_ll_read_rxfifo(hw, scratch, n);
    }
  }
  apply_debug_uart_pin_policy();

  // (SD was already mounted + pinned right after display init, above.)

  // --- Crash diagnostics (no-serial device: read these back off the SD) ---
  // Log the reason for the LAST boot — if audio crashed the device, this is
  // how we learn whether it was a panic/exception, a watchdog, a brownout
  // (WiFi-TX current spike), etc. — plus the heap we have to work with.
  {
    const char* rr = "?";
    switch (esp_reset_reason()) {
      case ESP_RST_POWERON:   rr = "POWERON";   break;
      case ESP_RST_SW:        rr = "SW";        break;
      case ESP_RST_PANIC:     rr = "PANIC";     break;
      case ESP_RST_INT_WDT:   rr = "INT_WDT";   break;
      case ESP_RST_TASK_WDT:  rr = "TASK_WDT";  break;
      case ESP_RST_WDT:       rr = "WDT";       break;
      case ESP_RST_BROWNOUT:  rr = "BROWNOUT";  break;
      case ESP_RST_DEEPSLEEP: rr = "DEEPSLEEP"; break;
      case ESP_RST_EXT:       rr = "EXT";       break;
      default:                rr = "OTHER";     break;
    }
    char buf[200];
    snprintf(buf, sizeof(buf),
             "BOOT reset=%s heap8=%u heapInt=%u heapDMA=%u dmaLargest=%u PSRAM=%u\n",
             rr,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    storage_sd_log_append("IC705DBG.txt", buf);
  }

  // UI loop
  char last_key = 0;
  while (true) {
    M5Cardputer.update();
    M5Cardputer.Keyboard.updateKeysState();
    auto &state = M5Cardputer.Keyboard.keysState();
    char c = 0;
    if (!state.word.empty()) {
      c = state.word.back();
      state.word.clear();  // consume key
    } else if (state.del) {
      c = 0x7f;  // treat delete/backspace
    } else if (state.enter) {
      c = '\n';  // enter/return
    }
    auto_advance_ic705_connect();
    auto_start_ic705_audio_once_cat_ready();

    // Merge injected keys from console UART RX (G4/G5 per sdkconfig)
    poll_uart_inject_keys();
    if (c == 0 && s_key_inject_queue && g_debug_uart_pins_enabled) {
      char injected = 0;
      if (xQueueReceive(s_key_inject_queue, &injected, 0) == pdTRUE) {
        c = injected;
        last_key = 0;  // Reset debounce so same-key injection works
#if UART_SCREEN_MIRROR
        g_uart_mirror_pending = true;  // dump screen at top of next iteration
#endif
      }
    }

#if UART_SCREEN_MIRROR
    // Dump screen on the iteration AFTER a UART keypress was consumed,
    // once the UI has had a chance to process the key and redraw.
    static bool s_uart_mirror_fire = false;
    if (!g_debug_uart_pins_enabled) {
      g_uart_mirror_pending = false;
      s_uart_mirror_fire = false;
    } else if (s_uart_mirror_fire) {
      uart_mirror_dump_screen();
      s_uart_mirror_fire = false;
    }
    if (g_uart_mirror_pending) {
      g_uart_mirror_pending = false;
      s_uart_mirror_fire = true;  // fire on the next iteration
    }
#endif
    gps_runtime_tick();
    TickType_t now_ticks = xTaskGetTickCount();
    if ((now_ticks - g_app_core0_stack_last_sample_tick) >= pdMS_TO_TICKS(1000)) {
      g_app_core0_stack_last_sample_tick = now_ticks;
      UBaseType_t free_words = uxTaskGetStackHighWaterMark2(NULL);
      uint32_t free_bytes = (uint32_t)free_words * (uint32_t)sizeof(StackType_t);
      g_app_core0_stack_cur_free_bytes = free_bytes;
      if (g_app_core0_stack_min_free_bytes == 0 || free_bytes < g_app_core0_stack_min_free_bytes) {
        g_app_core0_stack_min_free_bytes = free_bytes;
      }
      debug_update_app_core0_stack_hud(true);
      perf_monitor_sample(now_ticks);
      if (ui_mode == UIMode::PERF) {
        draw_perf_view(false);
      }
    }
    // Startup splash: show briefly, then land on STATUS by default. Radio
    // connection is explicit through STATUS -> 2; direct-mode keys still
    // work immediately.
    if (g_startup_active) {
      if (g_startup_start_ms == 0) {
        g_startup_start_ms = esp_timer_get_time() / 1000;
      }

      if (c != 0 && c != last_key) {
        const bool direct_mode_entry = is_startup_direct_mode_key(c);
        g_startup_active = false;
        save_station_data();
        // The splash paints its own full-screen graphic (red/blue Icom bars,
        // title). Nothing else on exit does a full clear -- STATUS's own
        // draw only touches rows below UI_START_Y, and the top waterfall/
        // countdown strip is normally kept fresh by the running RX loop,
        // which hasn't executed yet here -- so blank once at the real exit
        // point, before any subsequent mode (STATUS or a direct-mode key) draws.
        M5.Display.fillScreen(TFT_BLACK);
        if (!direct_mode_entry) {
          // Non-mode key: dismiss, land on STATUS, consume the key.
          last_key = c;
          enter_mode(UIMode::STATUS);
          vTaskDelay(pdMS_TO_TICKS(10));
          continue;
        }
        // Direct-mode key: fall through so the main dispatcher handles it.
        last_key = 0;
      } else {
        const int64_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - g_startup_start_ms >= kStartupAutoDismissMs) {
          // No key within the window: dismiss the splash and land on STATUS.
          g_startup_active = false;
          save_station_data();
          M5.Display.fillScreen(TFT_BLACK);
          enter_mode(UIMode::STATUS);
          last_key = 0;
          vTaskDelay(pdMS_TO_TICKS(10));
          continue;
        }
        last_key = c;  // 0 or the same key still held
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
    }


    rtc_tick();
    update_countdown();
    consume_cdc_initial_sync();  // auto-sync VFO on first QMX connect (every iter)
    check_slot_boundary();  // TX trigger at slot boundary (matching reference architecture)
    tx_tick();              // Process TX state machine (single-threaded, non-blocking)
    tune_tick();            // Auto-stop the tune burst once its window elapses

    // Drain deferred config saves requested by core commands.
    if (g_config_save_pending && storage_service_firmware_available()) {
      g_config_save_pending = false;
      save_station_data();
    }

    // Must run every iteration (not just once we already know !g_tx_active)
    // so it can actually latch the hold while TX is still active -- see
    // rx_redraw_should_hold()'s comment.
    const bool redraw_holding = rx_redraw_should_hold();

    // Global TX cancel (Esc/` in RX/TX/Status when not editing). This runs
    // BEFORE the per-mode key dispatch below and continues the loop, so it's
    // the only place that actually sees backtick while the hero card is up
    // (any per-mode '`' handling under case UIMode::RX would never be
    // reached). ESC is the ONLY key that dismisses the hero card, at any
    // point -- including mid-TX: stop the TX, drop the QSO/CQ context, and
    // always land back on the plain RX list.
    if (c == '`' &&
        (ui_mode == UIMode::RX || ui_mode == UIMode::STATUS) &&
        status_edit_idx == -1) {
      core_cmd_cancel_tx();
      if (ui_mode == UIMode::RX && g_hero_locked) {
        core_cmd_drop_qso(0);
        g_hero_locked = false;
        g_cq_running = false;  // explicit stop
        g_qso_done_active = false;  // dismiss the completion/give-up hold immediately too
        g_qso_done_gave_up = false;
        render_rx_or_hero();
      }
      debug_log_line("TX cancel requested");
      last_key = c;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (c == 0) {
      // NOTE: running-CQ re-scheduling lives in decode_monitor_results() -- a
      // fresh CQ is only re-enqueued after decodes are processed.
      // Hold ALL LCD updates during TX/tune — screen SPI/DMA contends with the
      // WiFi DMA on the S3 and stalls the outgoing audio (root cause of the 1Hz
      // carrier pulse). Resume normally the moment TX/tune ends. This also
      // covers the RX-dirty redraw below: a decode for the *other* slot parity
      // can land (and set g_rx_dirty) well before TX finishes, and rendering
      // it then — hero card or plain list — is the same forbidden mid-TX
      // redraw, just triggered by a decode instead of a keypress. Leaving
      // g_rx_dirty set here defers the redraw to the first idle tick after
      // TX/tune actually ends, instead of dropping it.
      if (!(g_tx_active || g_tune)) {
        if (g_rx_dirty && ui_mode == UIMode::RX && !redraw_holding) {
          // decode_monitor_results already called ui_set_rx_list_static(),
          // so UI's internal list is current. Just redraw (list or hero card).
          render_rx_or_hero();
          g_rx_dirty = false;
        }
        // Waterfall stays off the hero card entirely (Dean's preference) --
        // the timer/countdown bar (update_countdown(), unaffected here) is
        // still shown there.
        if (!(ui_mode == UIMode::RX && g_hero_locked)) ui_draw_waterfall_if_dirty();
        menu_flash_tick();
        rx_flash_tick();
        qso_clear_tick();
        qso_done_tick();
        refresh_status_view_if_dirty();
      }
      last_key = 0;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
  if (c == last_key) {
    // No new keypress - still need to refresh dirty views
    // NOTE: running-CQ re-scheduling lives in decode_monitor_results()
    if (!(g_tx_active || g_tune)) {
      if (!(ui_mode == UIMode::RX && g_hero_locked)) ui_draw_waterfall_if_dirty();
      refresh_status_view_if_dirty();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    continue;
  }
  last_key = c;

  rtc_tick();
  update_countdown();
  // consume_cdc_initial_sync() already called above, before the early-exit
  // branches; no need to repeat here.
  check_slot_boundary();  // TX trigger at slot boundary (matching reference architecture)
  tx_tick();              // Process TX state machine (single-threaded, non-blocking)
  menu_flash_tick();
  rx_flash_tick();
  qso_clear_tick();
  qso_done_tick();
  apply_pending_sync();

  // NOTE: TX scheduling now follows reference architecture:
  // 1. decode_monitor_results() sets g_qso_xmit flag after processing
  // 2. check_slot_boundary() triggers TX at slot boundary when parity matches
  // 3. autoseq_tick() is called at slot boundary AFTER TX slot ends

  refresh_status_view_if_dirty();

  // Ensure decode is enabled whenever streaming becomes active.
  if (audio_source_is_streaming() && !g_decode_enabled) {
    g_decode_enabled = true;
    ui_set_paused(false);
  }

  // Same TX/tune hold as the c==0 idle branch above -- a keypress landing
  // mid-TX (e.g. gain +/- from the hero card, which IS allowed while locked)
  // must not trigger a decode-driven redraw here either.
  if (!(g_tx_active || g_tune)) {
    if (g_rx_dirty && ui_mode == UIMode::RX && !redraw_holding) {
        // decode already populated ui.cpp's internal list via ui_set_rx_list_static
        render_rx_or_hero();
        g_rx_dirty = false;
    }
    if (!(ui_mode == UIMode::RX && g_hero_locked)) ui_draw_waterfall_if_dirty();
  }

  bool switched = false;
  auto cancel_status_edit = []() {
    if (status_edit_idx != -1) {
      status_edit_idx = -1;
      status_edit_buffer.clear();
      status_cursor_pos = -1;
    }
  };
  // Disabled while editing in MENU, and disabled entirely while the hero
  // card is locked on screen -- ESC (handled earlier, globally) and gain
  // +/- (handled below, under case UIMode::RX) are the only keys that
  // should do anything while it's up. Without this, these otherwise-global
  // mode-switch keys (S, M, N, O, B, Q, D, G, P, T, R) would jump straight
  // to their target mode and silently take the hero card down.
  if (!(ui_mode == UIMode::MENU && (menu_edit_idx >= 0 || menu_long_edit)) &&
      !(ui_mode == UIMode::RX && g_hero_locked)) {
      // Mode switch keys (disabled while editing in MENU)
      if (c == 'r' || c == 'R') { cancel_status_edit(); enter_mode(UIMode::RX); ui_force_redraw_rx(); ui_draw_rx(); switched = true; }
      else if (c == 'c' || c == 'C') {
        // Call CQ works from any mode now, same as the other global
        // mode-switch keys -- no need to back out of STATUS/MENU/etc first
        // (Dean's report: could jump straight to R from S, but not C).
        // Always re-opens with the default text, cursor right after "CQ "
        // so a prefix (e.g. POTA) can be typed immediately -- doesn't
        // remember a previous CQ's edit.
        cancel_status_edit();
        menu_long_edit = true;
        menu_long_kind = LONG_FT;
        menu_long_buf = "CQ " + g_call + " " + grid_ft8_4(g_grid);
        menu_long_backup = menu_long_buf;
        menu_long_cursor_pos = 3;
        enter_mode(UIMode::MENU);
        switched = true;
      }
      else if (c == 'm' || c == 'M') {
        // M opens the category picker; from inside a category it goes back
        // up to the picker; from the picker it exits to RX (matches the old
        // "press again to exit" toggle convention).
        cancel_status_edit();
        if (ui_mode == UIMode::MENU) {
          if (menu_category < 0) {
            enter_mode(UIMode::RX);
          } else {
            menu_category = -1;
            draw_menu_view();
          }
        } else {
          enter_mode(UIMode::MENU);
        }
        switched = true;
      }
      else if (c == 's' || c == 'S') { cancel_status_edit(); enter_mode(ui_mode == UIMode::STATUS ? UIMode::RX : UIMode::STATUS); switched = true; }
    }

  if (!switched && c) {
    // Mode-specific handling
    switch (ui_mode) {
      case UIMode::GPS: {
        // GPS is only reachable via the System category now -- backtick
        // returns to it (there's no standalone G hotkey anymore).
        if (c == '`') {
          enter_mode(UIMode::MENU);
          menu_category = kCatSystem;
          draw_menu_view();
        }
        break;
      }
      case UIMode::PERF: {
        // Performance monitor is only reachable via the Logging category --
        // backtick returns to it (there's no standalone P hotkey anymore).
        if (c == '`') {
          enter_mode(UIMode::MENU);
          menu_category = kCatLogging;
          draw_menu_view();
        }
        break;
      }
      case UIMode::RX: {
        if (g_hero_locked) {
          // Hero card locked on screen: ESC (handled earlier, globally) and
          // gain +/- are the ONLY keys that do anything. Everything else
          // here -- tap-to-reply, Call CQ -- is a legitimate RX-mode action
          // that would otherwise still fire even though the decode list
          // isn't the thing on screen right now, so skip straight to the
          // gain-only handling instead of falling into the normal RX logic.
          if (c == '+' || c == '=') {
            ic705_tx_set_gain_q8(ic705_tx_get_gain_q8() + 8);
            band_gain_save(g_band_sel, ic705_tx_get_gain_q8());
            // Gain itself applies immediately either way; only the redraw is
            // held during TX/tune (same reason as everywhere else in this
            // file) -- the hero card will show the new value the moment
            // TX/tune ends.
            if (!(g_tx_active || g_tune)) render_rx_or_hero();
          } else if (c == '-' || c == '_') {
            ic705_tx_set_gain_q8(ic705_tx_get_gain_q8() - 8);
            band_gain_save(g_band_sel, ic705_tx_get_gain_q8());
            if (!(g_tx_active || g_tune)) render_rx_or_hero();
          }
          break;
        }
        int sel = ui_handle_rx_key(c);
        if (sel >= 0 && core_cmd_tap_rx(sel)) {
          // TX-state arming lives inside core_cmd_tap_rx for every UI path.
          // Answering someone else's CQ means we're no longer running our
          // own -- stop the auto-repeat.
          g_cq_running = false;
          rx_flash_idx = sel;
          rx_flash_deadline = rtc_now_ms() + 500;
          // Show the hero card NOW, synchronously, instead of just flashing
          // the list and waiting for the next g_rx_dirty-driven render.
          // core_cmd_tap_rx() just created an active autoseq context, so
          // autoseq_active_count() is already >0 here -- render_rx_or_hero()
          // will lock the hero card on immediately. This matters because
          // check_slot_boundary() runs earlier in the main loop than this key
          // dispatch; if the reply's TX happens to fire on parity within the
          // next iteration or two (tapping right after a TX slot begins), a
          // deferred render would stay stuck on the plain list for the whole
          // transmission (redraws are held during TX) -- looking broken even
          // though the radio is transmitting correctly. Rendering here, one
          // full loop iteration ahead of the next check_slot_boundary() call,
          // guarantees the hero card is already up before that can happen.
          if (!(g_tx_active || g_tune)) {
            render_rx_or_hero();
            g_rx_dirty = false;
          }
        } else if (c == '+' || c == '=') {
          // Live TX drive level, same as the STATUS screen's +/-.
          ic705_tx_set_gain_q8(ic705_tx_get_gain_q8() + 8);
          band_gain_save(g_band_sel, ic705_tx_get_gain_q8());
          if (!(g_tx_active || g_tune)) render_rx_or_hero();
        } else if (c == '-' || c == '_') {
          ic705_tx_set_gain_q8(ic705_tx_get_gain_q8() - 8);
          band_gain_save(g_band_sel, ic705_tx_get_gain_q8());
          if (!(g_tx_active || g_tune)) render_rx_or_hero();
        }
        break;
      }
        case UIMode::BAND: {
          if (band_edit_idx >= 0) {
            if ((c >= '0' && c <= '9') || c == '.') { band_edit_buffer.push_back(c); draw_band_view(); }
            else if (c == 0x08 || c == 0x7f) {
              if (!band_edit_buffer.empty()) { band_edit_buffer.pop_back(); draw_band_view(); }
            } else if (c == '\r' || c == '\n') {
              if (!band_edit_buffer.empty()) {
                float val = std::stof(band_edit_buffer);
                g_bands[band_edit_idx].freq = val;
                save_station_data();
              }
              band_edit_idx = -1;
              band_edit_buffer.clear();
              draw_band_view();
            }
          } else {
            if (c == ';') {
              if (band_page > 0) { band_page--; draw_band_view(); }
            } else if (c == '.') {
              if ((band_page + 1) * 6 < (int)g_bands.size()) { band_page++; draw_band_view(); }
            } else if (c >= '1' && c <= '6') {
              int idx = band_page * 6 + (c - '1');
              if (idx >= 0 && idx < (int)g_bands.size()) {
                band_edit_idx = idx;
                {
                  char fbuf[16];
                  float f = g_bands[idx].freq;
                  if (f == (int)f) snprintf(fbuf, sizeof(fbuf), "%d", (int)f);
                  else             snprintf(fbuf, sizeof(fbuf), "%.1f", f);
                  band_edit_buffer = fbuf;
                }
                draw_band_view();
              }
            } else if (c == '`') {
              // Band editing is only reachable via the Station category now
              // -- backtick returns there (no standalone B hotkey anymore).
              enter_mode(UIMode::MENU);
              menu_category = kCatStation;
              draw_menu_view();
            }
          }
          break;
        }
        case UIMode::STATUS: {
        if (status_edit_idx == -1) {
          if (c == '1') {
            begin_usb_host_mode();
          }
          else if (c == '2') {
            // Active band list is stored low-to-high (e.g. 40/20/15/10);
            // step with delta=-1 so pressing 2 goes high-to-low instead
            // (Dean's operating preference: start on the high bands).
            advance_active_band(-1);
            save_station_data();
            draw_status_view();
            debug_log_line("Band changed");
            // In-memory only. CAT push is deferred to:
            //   - STATUS exit (enter_mode), or
            //   - QMX initial-connect (consume_cdc_initial_sync reads
            //     current g_band_sel at sync time, so band edits made
            //     while QMX was still enumerating get picked up).
            // Why deferred: don't spam the radio with a CAT command on
            // every S->2 press.
          }
              else if (c == '3') {
                // Audio-carrier tune (debugging the carrier pump into the dummy
                // load). Brief automatic trigger, not a toggle: keys PTT +
                // streams a short tone burst, then tune_tick() auto-stops it
                // after kTuneAutoStopMs. Pressing again while already tuning
                // is treated as an early manual stop (safety escape hatch).
                if (g_tune) {
                  radio_control_set_tune(false, 0, 0);
                  g_tune = false;
                  g_decode_enabled = true;
                  debug_log_line("CAT tune: RX");
                } else if (radio_control_ready()) {
                  int freq_hz = (int)(g_bands[g_band_sel].freq * 1000.0f);
                  int tune_hz = (g_offset_src == OffsetSrc::CURSOR) ? g_offset_hz : 1500;
                  g_decode_enabled = false;
                  if (radio_control_set_tune(true, freq_hz, tune_hz) == ESP_OK) {
                    g_tune = true;
                    g_tune_stop_at_ms = rtc_now_ms() + kTuneAutoStopMs;
                    debug_log_line("CAT tune: TX");
                  } else {
                    g_decode_enabled = true;
                    ESP_LOGW(TAG, "CAT tune command failed");
                    debug_log_line("CAT tune failed");
                  }
                } else {
                  ESP_LOGW(TAG, "CAT not ready; tune skipped");
                }
                draw_status_view();
              }
              else if (c == '4') {
                status_edit_idx = 3; status_edit_buffer = g_date; status_cursor_pos = 0; while (status_cursor_pos < (int)status_edit_buffer.size() && (status_edit_buffer[status_cursor_pos] == '-')) status_cursor_pos++; draw_status_view();
              }
              else if (c == '5') {
                status_edit_idx = 4; status_edit_buffer = g_time; status_cursor_pos = 0; while (status_cursor_pos < (int)status_edit_buffer.size() && (status_edit_buffer[status_cursor_pos] == ':')) status_cursor_pos++; draw_status_view();
              }
              else if (c == '6') {
                // Gracefully disconnect the IC-705 so the radio releases its
                // session immediately (protocol disconnect, type 0x05) instead
                // of holding it until timeout — lets you flash/power-cycle the
                // Cardputer without rebooting the radio. Press 1 to reconnect.
                if (canonical_radio_type(g_radio) == RadioType::IC705) {
                  ESP_LOGI(TAG, "STATUS key 6: graceful IC-705 disconnect");
                  debug_log_line("Disconnected 705 — press 1 to reconnect");
                  g_ic705_manual_disconnect = true;  // keep auto-reconnect OFF
                  audio_source_stop();      // stop RX/TX audio tasks first
                  ic705_cat_disconnect();   // send 0x05 disconnect, then close
                  draw_status_view();
                }
              }
              else if (c == '+' || c == '=') {
                // Live TX drive level, like TD705. Saved per-band so each
                // band keeps its own clean-drive level across band changes.
                ic705_tx_set_gain_q8(ic705_tx_get_gain_q8() + 8);
                band_gain_save(g_band_sel, ic705_tx_get_gain_q8());
                draw_status_view();
              }
              else if (c == '-' || c == '_') {
                ic705_tx_set_gain_q8(ic705_tx_get_gain_q8() - 8);
                band_gain_save(g_band_sel, ic705_tx_get_gain_q8());
                draw_status_view();
              }
            } else {
              if (status_edit_idx == 3 || status_edit_idx == 4) {
                if (c == '`') { status_edit_idx = -1; status_edit_buffer.clear(); status_cursor_pos = -1; draw_status_view(); }
                else if (c == ',') { // left
                  int pos = status_cursor_pos - 1;
                  while (pos >= 0 && (status_edit_buffer[pos] == '-' || status_edit_buffer[pos] == ':')) pos--;
                  if (pos >= 0) status_cursor_pos = pos;
                  draw_status_view();
                } else if (c == '/') { // right
                  int pos = status_cursor_pos + 1;
                  while (pos < (int)status_edit_buffer.size() && (status_edit_buffer[pos] == '-' || status_edit_buffer[pos] == ':')) pos++;
                  if (pos < (int)status_edit_buffer.size()) status_cursor_pos = pos;
                  draw_status_view();
                } else if (c >= '0' && c <= '9') {
                  if (status_cursor_pos >= 0 && status_cursor_pos < (int)status_edit_buffer.size()) {
                    status_edit_buffer[status_cursor_pos] = c;
                    int pos = status_cursor_pos + 1;
                    while (pos < (int)status_edit_buffer.size() && (status_edit_buffer[pos] == '-' || status_edit_buffer[pos] == ':')) pos++;
                    if (pos < (int)status_edit_buffer.size()) status_cursor_pos = pos;
                  }
                  draw_status_view();
                } else if (c == '\n') {
                  if (status_edit_idx == 3) g_date = status_edit_buffer;
                  else g_time = normalize_time_hms(status_edit_buffer);
                  if (rtc_apply_manual_time_from_strings()) {
                    save_station_data();
                  } else {
                    debug_log_line("Invalid date/time");
                  }
                  status_edit_idx = -1;
                  status_cursor_pos = -1;
                  status_edit_buffer.clear();
                  draw_status_view();
                }
              } else {
                if (c == '`') { status_edit_idx = -1; status_edit_buffer.clear(); status_cursor_pos = -1; draw_status_view(); }
                else if (c == '\n') { status_edit_idx = -1; status_edit_buffer.clear(); status_cursor_pos = -1; draw_status_view(); }
              }
            }
            break;
          }
        case UIMode::MENU: {
          if (ui_mode == UIMode::MENU) {
            if (menu_long_edit) {
              if (c == '\n' || c == '\r') {
                if (menu_long_kind == LONG_FT) {
                  // Only remaining caller of LONG_FT: the CQ text prompt
                  // (F:/Send FreeText as standalone menu items are gone).
                  // Confirming sends the (possibly edited) text as a CQ,
                  // reusing the existing FREETEXT CQ-type machinery so
                  // running-CQ repeats keep reusing this same text.
                  g_cq_freetext = menu_long_buf;
                  g_cq_type = CqType::CQFREETEXT;
                  update_autoseq_cq_type();
                  const int64_t now_ms = rtc_now_ms();
                  const int slot_period = g_protocol->slot_time_ms;
                  const int next_parity = (int)(((now_ms / slot_period) + 1) & 1);
                  autoseq_start_cq(next_parity);
                  // Calling CQ IS the beacon trigger now -- keep re-issuing
                  // this same CQ every idle cycle until answered or stopped.
                  g_cq_running = true;
                  AutoseqTxEntry pending;
                  if (autoseq_fetch_pending_tx(pending)) {
                    arm_pending_tx(pending);
                  }
                  core_fire_qso_changed();
                  menu_long_edit = false;
                  menu_long_kind = LONG_NONE;
                  menu_long_buf.clear();
                  menu_long_backup.clear();
                  menu_long_cursor_pos = -1;
                  enter_mode(UIMode::RX);
                } else if (menu_long_kind == LONG_ACTIVE) {
                  g_active_band_text = menu_long_buf;
                  rebuild_active_bands();
                  save_station_data();
                  menu_long_edit = false;
                  menu_long_kind = LONG_NONE;
                  menu_long_buf.clear();
                  menu_long_backup.clear();
                  menu_long_cursor_pos = -1;
                  draw_menu_view();
                }
              } else if (c == '`') {
                bool was_ft = (menu_long_kind == LONG_FT);
                menu_long_edit = false;
                menu_long_kind = LONG_NONE;
                menu_long_buf.clear();
                menu_long_backup.clear();
                menu_long_cursor_pos = -1;
                if (was_ft) {
                  enter_mode(UIMode::RX);
                } else {
                  draw_menu_view();
                }
              } else if (c == 0x08 || c == 0x7f) {
                if (menu_long_cursor_pos >= 0) {
                  if (menu_long_cursor_pos > 0) {
                    menu_long_buf.erase((size_t)(menu_long_cursor_pos - 1), 1);
                    menu_long_cursor_pos--;
                  }
                } else if (!menu_long_buf.empty()) {
                  menu_long_buf.pop_back();
                }
                draw_menu_view();
              } else if (c >= 32 && c < 127) {
                char ch = c;
                if (menu_long_kind == LONG_FT) {
                  ch = toupper((unsigned char)ch);
                }
                if (menu_long_cursor_pos >= 0) {
                  menu_long_buf.insert((size_t)menu_long_cursor_pos, 1, ch);
                  menu_long_cursor_pos++;
                } else {
                  menu_long_buf.push_back(ch);
                }
                draw_menu_view();
              }
              break;
            } else if (menu_edit_idx >= 0) {
              if (c == '\n' || c == '\r') {
                bool should_save = true;
                // Global indices: category*MENU_CAT_BASE + local row (0-based).
                if (menu_edit_idx == kCatStation * MENU_CAT_BASE + 0) {
                  g_call = menu_edit_buf; autoseq_set_station(g_call, grid_ft8_4(g_grid));
                } else if (menu_edit_idx == kCatStation * MENU_CAT_BASE + 1) {
                  const std::string norm_grid = normalize_grid_maidenhead(menu_edit_buf);
                  if (!norm_grid.empty()) {
                    g_grid = norm_grid;
                    g_grid_saved_manual = g_grid;
                    g_grid_from_gps = false;
                    autoseq_set_station(g_call, grid_ft8_4(g_grid));
                  } else {
                    should_save = false;
                    debug_log_line("Grid format: AA00/AA00aa/AA00aa00");
                  }
                } else if (menu_edit_idx == kCatOperating * MENU_CAT_BASE + 1) {
                  g_offset_hz = atoi(menu_edit_buf.c_str()); redraw_countdown_now();
                } else if (menu_edit_idx == kCatOperating * MENU_CAT_BASE + 3) {
                  char* end = nullptr;
                  long v = std::strtol(menu_edit_buf.c_str(), &end, 10);
                  if (end != menu_edit_buf.c_str() && end && *end == '\0') {
                    if (v < 0) v = 0;
                    g_autoseq_max_retry = (int)v;
                    autoseq_set_max_retry(g_autoseq_max_retry);
                  }
                } else if (menu_edit_idx == kCatNetwork * MENU_CAT_BASE + 0) {
                  g_ic705_wifi_ssid = menu_edit_buf;
                } else if (menu_edit_idx == kCatNetwork * MENU_CAT_BASE + 1) {
                  g_ic705_wifi_pass = menu_edit_buf;
                } else if (menu_edit_idx == kCatNetwork * MENU_CAT_BASE + 2) {
                  g_ic705_net_user = menu_edit_buf;
                  ic705_net_set_credentials(g_ic705_net_user.c_str(), g_ic705_net_pass.c_str());
                } else if (menu_edit_idx == kCatNetwork * MENU_CAT_BASE + 3) {
                  g_ic705_net_pass = menu_edit_buf;
                  ic705_net_set_credentials(g_ic705_net_user.c_str(), g_ic705_net_pass.c_str());
                } else if (menu_edit_idx == kCatNetwork * MENU_CAT_BASE + 4) {
                  // Accept decimal or "0xNN" hex
                  char* end = nullptr;
                  long v = std::strtol(menu_edit_buf.c_str(), &end, 0);
                  if (end != menu_edit_buf.c_str() && v >= 0 && v <= 0xFF) {
                    g_ic705_civ_addr = (int)v;
                  }
                }
                if (should_save) {
                  save_station_data();
                }
                menu_edit_idx = -1;
                menu_edit_buf.clear();
                draw_menu_view();
              } else if (c == 0x08 || c == 0x7f) {
                if (!menu_edit_buf.empty()) menu_edit_buf.pop_back();
                draw_menu_view();
                if (menu_edit_idx == kCatOperating * MENU_CAT_BASE + 1) {
                  g_offset_hz = atoi(menu_edit_buf.c_str());
                  redraw_countdown_now();
                }
              } else if (c == '`') {
                if (menu_edit_idx == kCatOperating * MENU_CAT_BASE + 1) {
                  g_offset_hz = menu_cursor_edit_original;
                  redraw_countdown_now();
                }
                menu_edit_idx = -1;
                menu_edit_buf.clear();
                draw_menu_view();
              } else if (menu_edit_idx == kCatOperating * MENU_CAT_BASE + 1 &&
                         (c == ';' || c == '.' || c == ',' || c == '/')) {
                // Arrow mode starts from the currently shown edit value.
                int cursor_val = g_offset_hz;
                if (!menu_edit_buf.empty()) {
                  cursor_val = atoi(menu_edit_buf.c_str());
                }
                if (c == ';') cursor_val += 100;
                else if (c == '.') cursor_val -= 100;
                else if (c == ',') cursor_val -= 10;
                else cursor_val += 10; // '/'
                // Clamp applies only to arrow mode.
                if (cursor_val < 200) cursor_val = 200;
                if (cursor_val > 3000) cursor_val = 3000;
                g_offset_hz = cursor_val;
                menu_edit_buf = std::to_string(cursor_val);
                draw_menu_view();
                redraw_countdown_now();
              } else if (c >= 32 && c < 127) {
                char ch = c;
                if (menu_edit_idx == kCatOperating * MENU_CAT_BASE + 3) {
                  if (ch < '0' || ch > '9') break;
                  if (menu_edit_buf.size() >= 10) break;
                }
                // Force uppercase only where it's correct: callsign, grid, and
                // the CI-V hex address. Credentials (WiFi SSID/pass, net
                // user/pass) are case-sensitive and must NOT be uppercased.
                if (menu_edit_idx == kCatStation * MENU_CAT_BASE + 0 ||
                    menu_edit_idx == kCatStation * MENU_CAT_BASE + 1 ||
                    menu_edit_idx == kCatNetwork * MENU_CAT_BASE + 4) {
                  ch = toupper((unsigned char)ch);
                }
                menu_edit_buf.push_back(ch);
                draw_menu_view();
                if (menu_edit_idx == kCatOperating * MENU_CAT_BASE + 1) {
                  g_offset_hz = atoi(menu_edit_buf.c_str());
                  redraw_countdown_now();
                }
              }
              break;
            }

        if (menu_category < 0) {
          // Category picker.
          if (c == '1') { menu_category = kCatStation; draw_menu_view(); }
          else if (c == '2') { menu_category = kCatOperating; draw_menu_view(); }
          else if (c == '3') { menu_category = kCatNetwork; draw_menu_view(); }
          else if (c == '4') { menu_category = kCatLogging; draw_menu_view(); }
          else if (c == '5') { menu_category = kCatSystem; draw_menu_view(); }
        } else if (c == '`') {
          // Back up to the picker from any category.
          menu_category = -1;
          draw_menu_view();
        } else if (menu_category == kCatStation) {
              if (c == '1') {
                menu_edit_idx = kCatStation * MENU_CAT_BASE + 0; // Call
                menu_edit_buf = g_call;
                draw_menu_view();
              } else if (c == '2') {
                menu_edit_idx = kCatStation * MENU_CAT_BASE + 1; // Grid
                menu_edit_buf = g_grid;
                draw_menu_view();
              } else if (c == '3') {
                menu_long_edit = true;
                menu_long_kind = LONG_ACTIVE;
                menu_long_buf = g_active_band_text;
                menu_long_backup = g_active_band_text;
                draw_menu_view();
              } else if (c == '4') {
                enter_mode(UIMode::BAND);
              }
            } else if (menu_category == kCatOperating) {
                if (c == '1') {
                  g_offset_src = (OffsetSrc)(((int)g_offset_src + 1) % 3);
                  save_station_data();
                  draw_menu_view();
                } else if (c == '2') {
                  menu_edit_idx = kCatOperating * MENU_CAT_BASE + 1; // Fixed
                  menu_cursor_edit_original = g_offset_hz;
                  menu_edit_buf = std::to_string(g_offset_hz);
                  draw_menu_view();
                } else if (c == '3') {
                  g_skip_tx1 = !g_skip_tx1;
                  autoseq_set_skip_tx1(g_skip_tx1);
                  save_station_data();
                  draw_menu_view();
                } else if (c == '4') {
                  menu_edit_idx = kCatOperating * MENU_CAT_BASE + 3; // Max Retry
                  menu_edit_buf = std::to_string(g_autoseq_max_retry);
                  draw_menu_view();
                } else if (c == '5') {
#if ENABLE_FT4
                  // Toggle the pending protocol mode (FT8 <-> FT4).
                  // g_protocol stays as-is for this boot session; the change
                  // takes effect on next reboot.
                  g_protocol_pending_ft4 = !g_protocol_pending_ft4;
                  save_station_data();
                  draw_menu_view();
#endif
                }
            } else if (menu_category == kCatNetwork) {
              if (c == '1') {
                menu_edit_idx = kCatNetwork * MENU_CAT_BASE + 0; // WiFi SSID
                menu_edit_buf = g_ic705_wifi_ssid;
                draw_menu_view();
              } else if (c == '2') {
                menu_edit_idx = kCatNetwork * MENU_CAT_BASE + 1; // WiFi Pass
                menu_edit_buf = g_ic705_wifi_pass;
                draw_menu_view();
              } else if (c == '3') {
                menu_edit_idx = kCatNetwork * MENU_CAT_BASE + 2; // Net User
                menu_edit_buf = g_ic705_net_user;
                draw_menu_view();
              } else if (c == '4') {
                menu_edit_idx = kCatNetwork * MENU_CAT_BASE + 3; // Net Pass
                menu_edit_buf = g_ic705_net_pass;
                draw_menu_view();
              } else if (c == '5') {
                menu_edit_idx = kCatNetwork * MENU_CAT_BASE + 4; // CI-V Addr
                char civ_str[8];
                snprintf(civ_str, sizeof(civ_str), "0x%02X", (unsigned)g_ic705_civ_addr);
                menu_edit_buf = civ_str;
                draw_menu_view();
              }
            } else if (menu_category == kCatLogging) {
              if (c == '1') {
                // Export the accumulated NVS ADIF log to the SD card in one
                // shot. Best done while idle — that's when SD writes work.
                menu_copy_feedback_text = end_session_export_to_sd();
                menu_flash_idx = kCatLogging * MENU_CAT_BASE + 0;
                menu_flash_deadline = rtc_now_ms() + 500;
                if (menu_copy_feedback_text.size() > 19) {
                  menu_copy_feedback_text.resize(19);
                }
                menu_copy_feedback_deadline = rtc_now_ms() + kMenuCopyFeedbackMs;
                debug_log_line(std::string("Export log: ") + menu_copy_feedback_text);
                draw_menu_view();
              } else if (c == '2') {
                // "Clear QSO Log": its own number ('2') doubles as the confirm
                // button. First press arms it; the same number again within
                // kQClearArmMs clears the durable NVS log. Export first (above)
                // if you want a copy — this does not touch the SD card.
                const int64_t now2 = rtc_now_ms();
                if (g_q_clear_armed && now2 < g_q_clear_arm_deadline) {
                  qso_log_clear_nvs();
                  g_adif_sd_seq = 0;
                  g_q_clear_armed = false;
                  g_q_clear_feedback = "Log cleared";
                  g_q_clear_feedback_deadline = now2 + 1500;
                } else {
                  g_q_clear_armed = true;
                  g_q_clear_arm_deadline = now2 + kQClearArmMs;
                }
                draw_menu_view();
              } else if (c == '3') {
                enter_mode(UIMode::PERF);
              }
            } else if (menu_category == kCatSystem) {
              if (c == '2') {
                ESP_LOGI(TAG, "Entering deep sleep (GPIO0 wake)");
                // Save current accurate time for compensation after wake-up
                if (rtc_valid) {
                  g_rtc_sleep_epoch = rtc_epoch_base +
                      (esp_timer_get_time() / 1000 - rtc_ms_start) / 1000;
                  rtc_sync_to_esp_rtc();
                  save_station_data();
                  ESP_LOGI(TAG, "Saved sleep epoch: %ld, comp=%d",
                           (long)g_rtc_sleep_epoch, g_rtc_comp);
                }
                M5.Display.sleep();
                vTaskDelay(pdMS_TO_TICKS(100));
                // Configure GPIO0 as wake-up source (active low)
                esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
                esp_deep_sleep_start();
              } else if (c == '3') {
                enter_mode(UIMode::GPS);
              } else if (c == '4') {
                // Re-resolve IC-705 target IP
                wifi_mgr_resolve_now();
                draw_menu_view();
              } else if (c == '5') {
                // Cycle 1..10 (10%..100%), wrapping back to 1 after 10.
                g_brightness_step = (g_brightness_step % 10) + 1;
                apply_brightness();
                save_station_data();
                draw_menu_view();
              }
            }
          }
          break;
        }
      }
    }


    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

extern "C" void app_main(void) {
  // esp_wifi_init() requires NVS to be initialized first.
  esp_err_t nvs_err = nvs_flash_init();
  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvs_err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvs_err);

  // Run the main application loop on core0.
  xTaskCreatePinnedToCore(app_task_core0, "app_core0", APP_CORE0_STACK_BYTES, nullptr, 5, nullptr, 0);
}
static void draw_status_line(int idx, const std::string& text, bool highlight) {
  const int line_h = 19;
  const int start_y = UI_START_Y;
  int y = start_y + idx * line_h;
  uint16_t bg = highlight ? M5.Display.color565(30, 30, 60) : TFT_BLACK;
  M5.Display.setTextSize(2);
  M5.Display.fillRect(0, y, 240, line_h, bg);
  M5.Display.setTextColor(TFT_WHITE, bg);
  M5.Display.setCursor(0, y);
  char buf[160];
  std::snprintf(buf, sizeof(buf), "%d %s", idx + 1, text.c_str());
  ui_set_visible_text_line(idx, buf);
  M5.Display.printf("%s", buf);
}
[[maybe_unused]] static void draw_battery_icon(int x, int y, int w, int h, int level, bool charging) {
  if (level < 0) level = 0;
  if (level > 100) level = 100;
  // Outline
  M5.Display.startWrite();
  M5.Display.fillRect(x, y, w, h, TFT_BLACK);
  M5.Display.drawRect(x, y, w - 3, h, TFT_WHITE);
  M5.Display.fillRect(x + w - 3, y + h / 4, 3, h / 2, TFT_WHITE); // tab
  // Fill
  int inner_w = w - 5;
  int inner_h = h - 4;
  int fill_w = (inner_w * level) / 100;
  uint16_t fill_color = (level > 30) ? M5.Display.color565(0, 200, 0)
                        : (level > 15) ? M5.Display.color565(200, 180, 0)
                                        : M5.Display.color565(200, 0, 0);
  M5.Display.fillRect(x + 2, y + 2, fill_w, inner_h, fill_color);
  // Charging bolt
  if (charging) {
    int bx = x + w / 2 - 2;
    int by = y + 2;
    M5.Display.fillTriangle(bx, by, bx + 4, by + h / 2, bx + 2, by, M5.Display.color565(255, 255, 0));
    M5.Display.fillTriangle(bx + 2, by + h / 2, bx + 6, by + h - 2, bx + 4, by + h - 2, M5.Display.color565(255, 255, 0));
  }
  M5.Display.endWrite();
}
