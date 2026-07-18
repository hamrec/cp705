#pragma once

#include <cstdint>

// Shared plumbing for the periodic SD-card diagnostic logs (CQHEALTH, CQFAIL,
// TXSTALL, TXDONE, HEAPTREND, WIFI DISCONNECT, ...) added while chasing the
// CQ-beacon stall. They all write to the same file; the throttled ones share
// the same "log at most once every N ms" gate.

// Filename all periodic SD-card diagnostic logs write to.
inline constexpr const char* kDiagLogFile = "IC705DBG.txt";

// Returns true (and advances *last_ms to now_ms) if at least interval_ms has
// elapsed since the last true return -- the shared throttle gate for
// millisecond-clocked periodic diagnostics. Callers on a TickType_t/
// xTaskGetTickCount() clock (e.g. a task loop that already throttles other
// diagnostics on ticks) should keep using that idiom instead of converting
// just to use this helper -- consistency with neighboring code in the same
// function matters more than forcing every call site onto one time base.
inline bool diag_log_due(int64_t* last_ms, int64_t now_ms, int64_t interval_ms) {
    if (now_ms - *last_ms < interval_ms) return false;
    *last_ms = now_ms;
    return true;
}
