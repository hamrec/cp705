#include "ft8_audio_pipeline.h"
#include "protocol.h"

#include <cmath>
#include <cstring>
#include <ctime>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

extern "C" {
#include "common/monitor.h"
#include "ft8/constants.h"
#include "ft8/decode.h"
}

extern void log_heap(const char* tag);
extern bool g_decode_enabled;
extern int g_time_osr;
extern int g_freq_osr;
extern int64_t g_decode_slot_idx;
extern volatile bool g_decode_in_progress;
extern volatile int64_t g_decode_applied_slot_idx;
void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui);
int64_t rtc_now_ms();

#ifndef FT8_SAMPLE_RATE
#define FT8_SAMPLE_RATE 6000
#endif


void ft8_audio_pipeline_run(const ft8_audio_pipeline_config_t* cfg)
{
    if (!cfg || !cfg->read || !cfg->should_stop) return;
    const char* tag = cfg->tag ? cfg->tag : "FT8_AUDIO";

    monitor_config_t mon_cfg = {
        .f_min = 200.0f,
        .f_max = 2900.0f,
        .sample_rate = FT8_SAMPLE_RATE,
        .time_osr = g_time_osr,
        .freq_osr = g_freq_osr,
        .protocol = g_protocol->protocol_id
    };

    log_heap("AUDIO_PIPE_BEFORE_MONITOR_INIT");
    monitor_t mon;
    monitor_init(&mon, &mon_cfg);
    log_heap("AUDIO_PIPE_AFTER_MONITOR_INIT");

    monitor_reset(&mon);

    float* ft8_buffer = (float*)heap_caps_malloc(sizeof(float) * mon.block_size, MALLOC_CAP_DEFAULT);
    float* temp_dec = (float*)heap_caps_malloc(sizeof(float) * 512, MALLOC_CAP_DEFAULT);
    log_heap("AUDIO_PIPE_AFTER_SAMPLE_BUFFERS");

    if (!ft8_buffer || !temp_dec) {
        ESP_LOGE(tag, "pipeline buffer allocation failed");
        if (ft8_buffer) free(ft8_buffer);
        if (temp_dec) free(temp_dec);
        monitor_free(&mon);
        return;
    }

    const int64_t slot_ms      = g_protocol->slot_time_ms;
    const int     target_blocks = g_protocol->total_symbols + 1;

    int64_t now_ms = rtc_now_ms();
    int64_t rem = now_ms % slot_ms;
    int64_t wait_ms = (rem < 100) ? 0 : (slot_ms - rem);
    if (wait_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)wait_ms));
    }

    int ft8_buffer_idx = 0;
    int slot_blocks = 0;
    int64_t slot_idx = rtc_now_ms() / slot_ms;
    int64_t slot_start_ms = slot_idx * slot_ms;
    (void)slot_start_ms;

    while (!cfg->should_stop(cfg->ctx)) {
        // Fully idle the RX pipeline while decode is disabled (TX/tune). This
        // skips not just the final decode but the per-block FFT/waterfall work
        // too, freeing core 1 for the TX writer task — FT8 is half-duplex and the
        // radio sends no RX audio while we transmit anyway.
        if (!g_decode_enabled) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        int samples_dec = cfg->read(cfg->ctx, temp_dec, 512);
        if (samples_dec <= 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        if (samples_dec > 512) samples_dec = 512;

        for (int i = 0; i < samples_dec && !cfg->should_stop(cfg->ctx); i++) {
            ft8_buffer[ft8_buffer_idx++] = temp_dec[i];

            if (ft8_buffer_idx >= mon.block_size) {
                double acc = 0.0;
                for (int j = 0; j < mon.block_size; ++j) {
                    acc += fabsf(ft8_buffer[j]);
                }
                float level = (float)(acc / mon.block_size);
                float gain = (level > 1e-6f) ? 0.1f / level : 1.0f;
                if (gain < 0.1f) gain = 0.1f;
                if (gain > 10.0f) gain = 10.0f;
                for (int j = 0; j < mon.block_size; ++j) {
                    ft8_buffer[j] *= gain;
                }

                if (mon.wf.num_blocks < target_blocks) {
                    monitor_process(&mon, ft8_buffer);
                }

                if (cfg->on_block_processed) {
                    cfg->on_block_processed(cfg->ctx);
                }

                ft8_buffer_idx = 0;
                // No explicit per-block pacing here (there used to be a
                // vTaskDelayUntil to the symbol period). cfg->read() already
                // blocks waiting for the next queued audio packet when none is
                // ready, which paces this loop to real-time on its own -- the
                // extra sleep was redundant with that, and while it slept, the
                // network task kept pushing new packets into the (6-slot)
                // audio queue with nobody draining it, dropping the rest.
                // Traced 2026-07-18 to a steady ~40-49% RX audio packet drop
                // rate, present from the first minute of any session: each
                // block-processing pass has ~160ms (FT8 symbol period) between
                // it and the next queue drain, while packets arrive every
                // ~10ms -- ~16 packets could queue up in that gap against only
                // 6 slots. Removing the redundant sleep lets this loop keep
                // draining the queue near real-time instead of falling behind
                // every single block.

                slot_blocks++;
                int64_t now_idx = rtc_now_ms() / slot_ms;
                if (now_idx != slot_idx) {
                    ESP_LOGI(tag, "Slot boundary %lld->%lld blocks=%d wf=%d",
                             (long long)slot_idx, (long long)now_idx,
                             slot_blocks, mon.wf.num_blocks);
                    if (slot_idx > g_decode_applied_slot_idx) {
                        g_decode_applied_slot_idx = slot_idx;
                    }
                    slot_idx = now_idx;
                    slot_start_ms = slot_idx * slot_ms;
                    slot_blocks = 0;
                    mon.wf.num_blocks = 0;
                    monitor_reset(&mon);
                } else if (slot_blocks >= g_protocol->total_symbols &&
                           mon.wf.num_blocks >= g_protocol->total_symbols) {
                    ESP_LOGI(tag, "Triggering decode at slot %lld blocks=%d wf=%d",
                             (long long)slot_idx, slot_blocks, mon.wf.num_blocks);
                    if (g_decode_enabled) {
                        g_decode_slot_idx = slot_idx;
                        g_decode_in_progress = true;
                        decode_monitor_results(&mon, &mon_cfg, false);
                    } else {
                        ESP_LOGI(tag, "Decode paused; skipping");
                        if (slot_idx > g_decode_applied_slot_idx) {
                            g_decode_applied_slot_idx = slot_idx;
                        }
                    }
                    monitor_reset(&mon);
                    mon.wf.num_blocks = 0;
                    slot_blocks = 0;

                    // RE-ANCHOR the capture window to the UTC clock. The decode
                    // above runs synchronously and takes ~2-3s on this board,
                    // during which incoming audio is dropped. If we resumed
                    // accumulating immediately, block 0 would start deep inside
                    // the next 15s slot and the analysis window would WALK away
                    // from the radio's transmit timing every slot — decodes
                    // appear for one slot then vanish. Waiting for the next 15s
                    // boundary makes block 0 line up with the start of the next
                    // FT8 transmission, keeping the window phase-locked to UTC.
                    // (If the decode finished before the boundary, this wait is
                    // short and we still decode the very next slot; if it
                    // overran the boundary, we cleanly skip to the following
                    // slot instead of drifting.)
                    {
                        int64_t a_rem  = rtc_now_ms() % slot_ms;
                        int64_t a_wait = (a_rem < 100) ? 0 : (slot_ms - a_rem);
                        if (a_wait > 0) {
                            vTaskDelay(pdMS_TO_TICKS((uint32_t)a_wait));
                        }
                    }
                    slot_idx = rtc_now_ms() / slot_ms;
                    monitor_reset(&mon);
                    mon.wf.num_blocks = 0;
                    slot_blocks = 0;
                }
            }
        }
    }

    free(ft8_buffer);
    free(temp_dec);
    monitor_free(&mon);
}
