// ---------------------------------------------------------------------------
// stream_wifi_ic705.cpp
//
// Icom IC-705 WLAN remote audio — RX (decode) and TX (transmit).
//
// All session/transport work (the audio stream's own SID handshake, the
// 24-byte audio_packet header, port negotiation) lives in ic705_netctrl.cpp,
// matching wfview's icomUdpAudio class exactly (verified against wfview's
// source and its real audio_packet struct, not guessed). This file only
// renders PCM for TX and feeds received PCM into the FT8 decode pipeline.
// ---------------------------------------------------------------------------

#include "stream_wifi_ic705.h"
#include "ft8_audio_pipeline.h"
#include "resample.h"
#include "dds_q15.h"
#include "ic705_netctrl.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "storage_service.h"

#include <cstring>
#include <cmath>
#include <cstdio>
#include <inttypes.h>

static void heap_log(const char* where) {
    // Serial (USB console), NOT the SD card — keep the audio path independent
    // of SD health.
    ESP_LOGW("IC705_STREAM", "AUDIO %s: heap8=%u largest8=%u dma=%u dmaLargest=%u", where,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
}

static const char* TAG = "IC705_STREAM";

// External flag maintained by main.cpp (same pattern as stream_uac)
extern bool g_streaming;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static ic705_stream_state_t s_state       = IC705_STREAM_IDLE;
static volatile bool        s_stop_req    = false;
static uint32_t             s_ic705_ip    = 0;    // network byte order, unused by this file now
static char                 s_status[64]  = "Idle";
static char                 s_dbg1[64]    = "";
static char                 s_dbg2[64]    = "";
static TaskHandle_t         s_stream_task = NULL;
static resample_state_t     s_resample;

// TX writer
static volatile bool   s_tx_run       = false;
static volatile bool   s_tx_stop_perm = false;
static TaskHandle_t    s_tx_task      = NULL;

// Hardware-timer pacing: an esp_timer (microsecond-precise, crystal/PLL-derived —
// the same clock domain the I2S peripheral would use) fires every 10ms and gives
// this semaphore, so the SEND cadence is locked to a hardware timer instead of
// FreeRTOS tick scheduling. This is the practical embodiment of "pace it off a
// hardware audio clock like wfview's sound card." (True I2S-DMA pacing uses the
// same crystal, so it would behave the same for cadence precision.)
static SemaphoreHandle_t s_tx_tick_sem  = NULL;
static esp_timer_handle_t s_tx_pace_tmr = NULL;
static void tx_pace_cb(void*) { if (s_tx_tick_sem) xSemaphoreGive(s_tx_tick_sem); }

// ---------------------------------------------------------------------------
// TX writer task — mirrors spk_writer_task in stream_uac.cpp
// ---------------------------------------------------------------------------

// TX audio: one uniform 480-sample (960-byte = 10ms) frame per HARDWARE-TIMER
// tick. The 682/278 alternating frames seen in wfview's capture were just an
// artifact of wfview chopping its audio block at a 1364-byte limit (confirmed in
// wfview source: audio.data.mid(len,1364)), NOT a radio requirement — so uniform
// frames are equally valid and match our offline-verified clean waveform exactly.
// Pacing is driven by an esp_timer (microsecond-precise, crystal/PLL clock) so
// the send cadence is hardware-locked rather than FreeRTOS-tick scheduled — the
// closest we can get to wfview's sound-card-clocked delivery without I2S DMA.
#define TX_RENDER_BLK        480     // DDS render granularity AND frame size (samples)
#define TX_PACE_US           10000   // 480 samples @ 48kHz = 10ms hardware tick
#define TX_LEAD_SAMPLES      (48000 * 40 / 1000)   // pre-render ~40ms cushion in the ring
#define TX_LOOK_N            9600    // look-ahead ring capacity (samples) = 200ms
#define TX_STEREO_BUF_BYTES  (TX_RENDER_BLK * 6)
#define TX_MONO16_BUF_BYTES  (TX_RENDER_BLK * 2)

// TX audio output level as a Q8 fraction of full scale (256 = 1.0 = raw DDS).
// Full scale overdrives the IC-705 input (ALC pumps, meters bounce). ~0.5 with
// a normal WLAN MOD Level (~10-20%) reaches full power without ALC. Lower if
// still hot, raise if the radio can't reach power.
#define TX_GAIN_Q8           128   // 0.50 of full scale ≈ -6dB (steady-trigger level
                                   // before the low-level blip test). Level is NOT
                                   // the splatter cause (0.5 and 0.6 both splatter;
                                   // <-12dB just blips below the TX threshold).

// Look-ahead ring (filled by the DDS render) drained one 480-sample frame per
// hardware-timer tick for steady, crystal-locked send cadence.
static void tx_writer_task(void* /*arg*/) {
    static int16_t look[TX_LOOK_N];
    static uint8_t stereo_buf[TX_STEREO_BUF_BYTES];
    static uint8_t frame_buf[TX_MONO16_BUF_BYTES];

    ESP_LOGI(TAG, "TX writer task started");
    heap_log("tx-task-entry");

    bool     was_running   = false;
    int      rd = 0, wr = 0, count = 0;          // look-ahead ring state (samples)
    int64_t  worst_render_us = 0, worst_gap_us = 0, prev_send_us = 0;
    int      frames = 0, underruns = 0;
    // DIAGNOSTIC: envelope of the ACTUAL transmitted samples. Each frame's peak
    // |sample| is bucketed into ~160ms slots; dumped at TX end. For constant-
    // envelope GFSK this MUST be flat — if it pumps, the corruption is in OUR
    // generated audio (not WiFi), which is what we're testing.
    static uint16_t env[96];
    int      env_n = 0;
    uint16_t frame_pk = 0, pk_min = 0xFFFF, pk_max = 0;

    // Render one DDS block into the look-ahead ring.
    auto render_one = [&]() {
        int64_t r0 = esp_timer_get_time();
        dds_render_24bit_stereo(stereo_buf, TX_RENDER_BLK);
        for (int i = 0; i < TX_RENDER_BLK; ++i) {
            int32_t val = (int32_t)stereo_buf[i * 6]
                        | ((int32_t)stereo_buf[i * 6 + 1] << 8)
                        | ((int32_t)stereo_buf[i * 6 + 2] << 16);
            if (val & 0x800000) val |= 0xFF000000;  // sign-extend
            int16_t s16 = (int16_t)(((val >> 8) * TX_GAIN_Q8) >> 8);
            look[wr] = s16;
            if (++wr >= TX_LOOK_N) wr = 0;
            ++count;
        }
        int64_t r1 = esp_timer_get_time();
        if (r1 - r0 > worst_render_us) worst_render_us = r1 - r0;
    };

    while (!s_tx_stop_perm) {
        if (!s_tx_run) {
            if (was_running) {
                if (s_tx_pace_tmr) esp_timer_stop(s_tx_pace_tmr);
                uint32_t ok = 0, fail = 0, rexmit = 0;
                ic705_net_get_audio_tx_stats(&ok, &fail, &rexmit);
                ESP_LOGW(TAG, "TXDONE ok=%u fail=%u rexmit=%u | frames=%d underruns=%d worstGap=%lldus worstRender=%lldus",
                         (unsigned)ok, (unsigned)fail, (unsigned)rexmit,
                         frames, underruns, (long long)worst_gap_us, (long long)worst_render_us);
                // Envelope of the actual transmitted samples (peak per ~160ms).
                // FLAT = our audio is clean (splatter is external/WiFi/radio);
                // VARYING = our generated audio itself pumps (NOT a WiFi problem).
                ESP_LOGW(TAG, "TXENV pk_min=%u pk_max=%u (ratio=%.3f) n=%d",
                         (unsigned)pk_min, (unsigned)pk_max,
                         pk_max ? (double)pk_min / (double)pk_max : 0.0, env_n);
                char line[320]; int p = 0;
                for (int i = 0; i < env_n && p < (int)sizeof(line) - 6; ++i)
                    p += snprintf(line + p, sizeof(line) - p, "%u ", (unsigned)env[i]);
                ESP_LOGW(TAG, "TXENV[] %s", line);
            }
            was_running = false;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (!was_running) {
            was_running = true;
            rd = wr = count = 0;
            worst_render_us = worst_gap_us = prev_send_us = 0;
            frames = underruns = 0;
            env_n = 0; frame_pk = 0; pk_min = 0xFFFF; pk_max = 0;
            ic705_net_reset_audio_tx_stats();
            ic705_net_dump_audio_pkts(2);  // dump first 2 real packets for wfview byte-compare
            // Prime the ~40ms cushion, then start the hardware pacing timer at a
            // period matched to the RADIO's measured sample clock (not our nominal
            // 48kHz) so 480 samples are delivered in exactly the time the radio
            // consumes them — eliminating the buffer drift that splatters FT8.
            while (count <= TX_LOOK_N - TX_RENDER_BLK && count < TX_LEAD_SAMPLES)
                render_one();
            double radio_rate = ic705_net_get_measured_rx_rate();
            uint64_t pace_us = (uint64_t)((double)TX_RENDER_BLK * 1000000.0 / radio_rate + 0.5);
            if (pace_us < 8000 || pace_us > 12000) pace_us = TX_PACE_US;  // sanity clamp
            ESP_LOGW(TAG, "TX pace: radio_rate=%.2f Hz -> %llu us/frame", radio_rate, (unsigned long long)pace_us);
            xSemaphoreTake(s_tx_tick_sem, 0);  // clear any stale tick
            if (s_tx_pace_tmr) esp_timer_start_periodic(s_tx_pace_tmr, pace_us);
        }

        // Block until the hardware timer fires (precise 10ms). The send happens
        // immediately on wake from a block already rendered, so its timing is
        // locked to the crystal, not to render duration or task scheduling.
        if (xSemaphoreTake(s_tx_tick_sem, pdMS_TO_TICKS(100)) != pdTRUE) continue;

        if (count >= TX_RENDER_BLK) {
            uint16_t pk = 0;
            for (int i = 0; i < TX_RENDER_BLK; ++i) {
                int16_t s = look[rd];
                if (++rd >= TX_LOOK_N) rd = 0;
                frame_buf[i * 2]     = (uint8_t)(s & 0xFF);
                frame_buf[i * 2 + 1] = (uint8_t)((s >> 8) & 0xFF);
                uint16_t a = (s < 0) ? (uint16_t)(-s) : (uint16_t)s;
                if (a > pk) pk = a;
            }
            // envelope: peak per frame, bucketed ~160ms (16 frames)
            if (pk < pk_min) pk_min = pk;
            if (pk > pk_max) pk_max = pk;
            if (pk > frame_pk) frame_pk = pk;
            if ((frames % 16) == 15) {
                if (env_n < (int)(sizeof(env)/sizeof(env[0]))) env[env_n++] = frame_pk;
                frame_pk = 0;
            }
            count -= TX_RENDER_BLK;
            int64_t t0 = esp_timer_get_time();
            ic705_net_send_audio_pcm(frame_buf, TX_MONO16_BUF_BYTES);
            if (prev_send_us && (t0 - prev_send_us) > worst_gap_us) worst_gap_us = t0 - prev_send_us;
            prev_send_us = t0;
            ++frames;
        } else {
            underruns++;
        }

        // Refill the ring after the send (idle part of the tick).
        while (count <= TX_LOOK_N - TX_RENDER_BLK && count < TX_LEAD_SAMPLES + TX_RENDER_BLK)
            render_one();
    }

    if (s_tx_pace_tmr) esp_timer_stop(s_tx_pace_tmr);
    s_tx_task = NULL;
    vTaskDelete(NULL);
}

static bool tx_start_writer(void) {
    if (s_tx_run) return true;  // already running
    if (!s_tx_task) {
        ESP_LOGW(TAG, "TX writer task not created");
        return false;
    }
    s_tx_run = true;
    return true;
}

// ---------------------------------------------------------------------------
// RX audio → FT8 pipeline callbacks
// ---------------------------------------------------------------------------

static uint8_t s_rx_pcm[IC705_RX_BUF_BYTES];

static int ic705_read_ft8_samples(void* ctx, float* out, int max_samples) {
    (void)max_samples;
    (void)ctx;

    static bool first_call_logged = false;
    if (!first_call_logged) { first_call_logged = true; heap_log("first-read (pipeline allocated)"); }

    if (s_stop_req) return 0;

    int len = ic705_net_recv_audio_pcm(s_rx_pcm, sizeof(s_rx_pcm), 1000);
    if (len <= 0) return 0;

    // Peek at first sample for debug display
    if (len >= 2) {
        int16_t v = (int16_t)(s_rx_pcm[0] | (s_rx_pcm[1] << 8));
        snprintf(s_dbg1, sizeof(s_dbg1), "len=%d v=%d", len, (int)v);
        snprintf(s_dbg2, sizeof(s_dbg2), "rx bytes=%d", len);
    }

    // Convert 16-bit mono → float via resample helper (IC-705 @ 48kHz, FT8 needs 6kHz)
    return uac_pcm_to_ft8_samples(&s_resample, s_rx_pcm, len, out, 16, 1);
}

static bool ic705_should_stop(void* ctx) {
    (void)ctx;
    return s_stop_req || !ic705_net_audio_is_ready();
}

static void ic705_on_block_processed(void* /*ctx*/) {
    // No-op for IC-705: CI-V connection is managed separately
}

// ---------------------------------------------------------------------------
// Stream task (RX audio pipeline)
// ---------------------------------------------------------------------------

static void stream_task(void* /*arg*/) {
    ESP_LOGI(TAG, "RX stream task started");
    heap_log("rx-task-entry");
    resample_init(&s_resample);

    ft8_audio_pipeline_config_t cfg = {
        .tag             = TAG,
        .ctx             = nullptr,
        .read            = ic705_read_ft8_samples,
        .should_stop     = ic705_should_stop,
        .on_block_processed = ic705_on_block_processed,
    };
    heap_log("pre-pipeline-run");
    ft8_audio_pipeline_run(&cfg);

    g_streaming = false;
    s_stream_task = NULL;
    ESP_LOGI(TAG, "RX stream task stopped");
    vTaskDelete(NULL);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool ic705_stream_start(uint32_t ic705_ip) {
    if (s_state != IC705_STREAM_IDLE) {
        ESP_LOGW(TAG, "Already started");
        return false;
    }
    if (ic705_ip == 0) {
        ESP_LOGE(TAG, "IC-705 IP not resolved");
        return false;
    }
    if (!ic705_net_audio_is_ready()) {
        ESP_LOGW(TAG, "Audio session (SID3) not ready — CAT control unaffected");
        snprintf(s_status, sizeof(s_status), "Audio not ready");
        s_state = IC705_STREAM_ERROR;
        return false;
    }

    s_ic705_ip = ic705_ip;
    s_stop_req = false;
    s_tx_run   = false;
    s_tx_stop_perm = false;
    ft8_audio_pipeline_clear_latest_waterfall_row();
    resample_init(&s_resample);

    // RX decode task FIRST — it's the priority (without it, no decodes).
    // STATIC task stacks (reserved in .bss at boot, from clean memory): on
    // this no-PSRAM board the heap fragments during connect so the largest
    // free block gets permanently stuck around 7KB — a dynamic 8KB-stack task
    // then NEVER creates, which is "connected but nothing decodes". Static
    // stacks remove the fragmentation dependency entirely, matching how the
    // FT8 waterfall/FFT buffers are already static. StackType_t is uint8_t on
    // ESP-IDF Xtensa, so these arrays are sized in bytes.
    static StackType_t s_rx_stack[8192];
    static StaticTask_t s_rx_tcb;
    static StackType_t s_tx_stack[6144];  // +headroom: the writer now builds the
    static StaticTask_t s_tx_tcb;         // ~1KB audio packet on its own stack

    heap_log("pre-task-create");
    s_stream_task = xTaskCreateStaticPinnedToCore(stream_task, "ic705_rx", sizeof(s_rx_stack),
                                                  NULL, 4, s_rx_stack, &s_rx_tcb, 1);
    // Pinned to CORE 1, PRIORITY 7 (ABOVE the net task's 6). At equal priority
    // FreeRTOS round-robin-timesliced the writer against the net task every tick,
    // so the writer got suspended mid-stream — measured worst delivery gaps of
    // ~20ms (=2 ticks) and constant catch-up bursting, which jitters the audio
    // and splatters FT8. Priority 7 lets the writer run the instant it's ready and
    // preempt the net task; it sleeps (vTaskDelayUntil) ~3-5ms of every 10ms, so
    // the net task still gets ample core-1 time for keepalives/RX drain.
    // Hardware-timer pacing primitives for the TX writer.
    if (!s_tx_tick_sem) s_tx_tick_sem = xSemaphoreCreateBinary();
    if (!s_tx_pace_tmr) {
        esp_timer_create_args_t ta = {};
        ta.callback = tx_pace_cb;
        ta.dispatch_method = ESP_TIMER_TASK;
        ta.name = "tx_pace";
        esp_timer_create(&ta, &s_tx_pace_tmr);
    }
    s_tx_task = xTaskCreateStaticPinnedToCore(tx_writer_task, "ic705_tx", sizeof(s_tx_stack),
                                              NULL, 7, s_tx_stack, &s_tx_tcb, 1);

    snprintf(s_status, sizeof(s_status), "Streaming");
    s_state = IC705_STREAM_STREAMING;
    g_streaming = true;
    ESP_LOGI(TAG, "IC-705 audio stream started, IC-705 IP=%08" PRIx32, ic705_ip);
    return true;
}

void ic705_stream_stop(void) {
    if (s_state == IC705_STREAM_IDLE) return;

    ESP_LOGI(TAG, "Stopping IC-705 audio stream");
    s_stop_req     = true;
    s_tx_run       = false;
    s_tx_stop_perm = true;
    g_streaming    = false;

    // Give tasks a moment to notice the stop flag
    for (int i = 0; i < 20 && (s_stream_task || s_tx_task); ++i) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ft8_audio_pipeline_clear_latest_waterfall_row();

    dds_cpfsk_end();
    dds_set_freq_hz(0.0);

    s_state = IC705_STREAM_IDLE;
    snprintf(s_status, sizeof(s_status), "Idle");
    ESP_LOGI(TAG, "IC-705 stream stopped");
}

ic705_stream_state_t ic705_stream_get_state(void) { return s_state; }

bool ic705_stream_is_streaming(void) {
    return s_state == IC705_STREAM_STREAMING;
}

const char* ic705_stream_get_status_string(void) { return s_status; }
const char* ic705_stream_get_debug_line1(void)   { return s_dbg1;   }
const char* ic705_stream_get_debug_line2(void)   { return s_dbg2;   }

bool ic705_stream_get_latest_waterfall_row(uint8_t* out_row, int out_len) {
    return ft8_audio_pipeline_get_latest_waterfall_row(out_row, out_len);
}

// ---------------------------------------------------------------------------
// TX API (mirrors stream_uac TX API for QDX/CPFSK)
// ---------------------------------------------------------------------------

bool ic705_tx_begin_cpfsk(float base_hz,
                           const uint8_t* symbols,
                           size_t symbol_count,
                           float tone_spacing_hz,
                           uint32_t samples_per_symbol) {
    if (!dds_cpfsk_begin((double)base_hz, symbols, symbol_count,
                          (double)tone_spacing_hz, samples_per_symbol)) {
        ESP_LOGE(TAG, "CPFSK schedule failed count=%u sps=%u",
                 (unsigned)symbol_count, (unsigned)samples_per_symbol);
        return false;
    }

    if (!tx_start_writer()) {
        dds_cpfsk_end();
        return false;
    }

    ESP_LOGI(TAG, "TX CPFSK start base=%.2f count=%u spacing=%.4f sps=%u",
             (double)base_hz, (unsigned)symbol_count,
             (double)tone_spacing_hz, (unsigned)samples_per_symbol);
    return true;
}

bool ic705_tx_begin_tune(float tone_hz) {
    dds_cpfsk_end();
    dds_reset_phase();
    dds_set_freq_hz((double)tone_hz);

    if (!tx_start_writer()) {
        dds_set_freq_hz(0.0);
        return false;
    }

    ESP_LOGI(TAG, "TX tune start tone=%.2f Hz", (double)tone_hz);
    return true;
}

void ic705_tx_end(void) {
    if (!s_tx_run) {
        dds_cpfsk_end();
        dds_set_freq_hz(0.0);
        return;
    }
    s_tx_run = false;
    vTaskDelay(pdMS_TO_TICKS(30));  // let writer drain
    dds_cpfsk_end();
    dds_set_freq_hz(0.0);
    ESP_LOGI(TAG, "TX audio stopped");
}
