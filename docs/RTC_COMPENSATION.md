# ESP RTC Deep Sleep Compensation

## Problem

The ESP32's internal RTC oscillator (150kHz RC) drifts significantly during deep sleep.
Users have reported ~10 minutes drift over 9 hours of sleep (~1.85% error).
This makes the clock inaccurate after wake-up.

## Solution

Implement a compensation mechanism that:
1. Records the accurate time before entering deep sleep
2. Measures the raw ESP RTC elapsed time during sleep
3. Applies a user-calibrated compensation factor to correct drift

## Data Model

Two new fields in station data (`Station.txt`):

```
rtc_sleep_epoch=1735689600    # Unix epoch when entering deep sleep
rtc_comp=150                  # Compensation: seconds per 10000 seconds
```

### Compensation Factor Definition

`rtc_comp` represents how many extra (or fewer) seconds the ESP RTC counts per 10000 real seconds:

| rtc_comp | Meaning | ESP RTC Behavior |
|----------|---------|--------------|
| +150 | 1.5% fast | After 10000 real sec, ESP RTC shows 10150 sec |
| -123 | 1.23% slow | After 10000 real sec, ESP RTC shows 9877 sec |
| 0 | Perfect | No compensation needed |

### Compensation Formula

```
actual_elapsed = raw_elapsed * 10000 / (10000 + rtc_comp)
```

Example (ESP RTC is 1.5% fast, slept for ~10 hours):
- Raw ESP RTC shows: 36540 seconds elapsed
- Compensation: `36540 * 10000 / 10150 = 36000` seconds (10 hours actual)

## Implementation

### Before Deep Sleep

```cpp
void enter_deep_sleep() {
    // 1. Get current accurate time from soft RTC
    time_t now = rtc_epoch_base + (esp_timer_get_time()/1000 - rtc_ms_start) / 1000;

    // 2. Save to station data
    g_rtc_sleep_epoch = now;
    save_station_data();

    // 3. Configure wake-up and sleep
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
    esp_deep_sleep_start();
}
```

### After Wake-up

```cpp
bool rtc_init_from_esp_rtc() {
    struct timeval tv;
    gettimeofday(&tv, NULL);  // Read ESP RTC

    // Check if we have valid sleep data
    if (g_rtc_sleep_epoch <= 0 || tv.tv_sec < g_rtc_sleep_epoch) {
        // No valid sleep data, fall back to raw ESP RTC
        return init_from_raw_esp_rtc(tv.tv_sec);
    }

    // Calculate raw elapsed time
    int64_t raw_elapsed = tv.tv_sec - g_rtc_sleep_epoch;

    // Apply compensation
    int64_t actual_elapsed = raw_elapsed;
    if (g_rtc_comp != 0) {
        actual_elapsed = raw_elapsed * 10000 / (10000 + g_rtc_comp);
    }

    // Set soft RTC to compensated time
    time_t compensated_now = g_rtc_sleep_epoch + actual_elapsed;

    // Initialize soft RTC
    rtc_epoch_base = compensated_now;
    rtc_ms_start = esp_timer_get_time() / 1000;
    rtc_valid = true;

    // Log for debugging
    ESP_LOGI(TAG, "RTC wake: slept %lld raw sec, compensated %lld sec (comp=%d)",
             raw_elapsed, actual_elapsed, g_rtc_comp);

    return true;
}
```

## Calibration Procedure

To determine the compensation factor:

1. Set the clock accurately (via FT8 sync or manual)
2. Note the exact time and put device to deep sleep
3. Wait a known duration (e.g., 10 hours = 36000 seconds)
4. Wake up and note the displayed time
5. Calculate: `rtc_comp = (displayed_elapsed - actual_elapsed) * 10000 / actual_elapsed`

Example:
- Actual sleep: 36000 seconds (10 hours)
- ESP RTC shows: 36540 seconds elapsed
- `rtc_comp = (36540 - 36000) * 10000 / 36000 = 150` (1.5% fast)

## Station Data Format

```ini
# Existing fields...
rtc_sleep_epoch=1735689600
rtc_comp=0
```

## UI Considerations

The compensation factor could be:
1. Manually entered via menu
2. Auto-calculated if user confirms accurate time after wake

For now, implement manual entry in menu page 2 (settings).

## Edge Cases

1. **First boot / no sleep data**: `rtc_sleep_epoch = 0`, fall back to raw ESP RTC or saved strings
2. **rtc_sleep_epoch > current RTC**: Invalid state, RTC was reset, ignore compensation
3. **Very long sleep**: Compensation still applies linearly (RC oscillator drift is approximately linear)
4. **User changes time during operation**: Update `rtc_sleep_epoch` before next sleep

## Testing

1. Set time accurately via FT8 sync
2. Enter deep sleep for known duration
3. Wake and verify time matches expected (within acceptable tolerance)
4. Adjust `rtc_comp` if needed and repeat
