#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// RAII lock over a lazily-created FreeRTOS mutex. Construct with a reference
// to a file-static SemaphoreHandle_t (created on first use), takes the mutex
// for the guard's lifetime, gives it back on every exit path including early
// returns. Shared base for the small per-lock guard types (AutoseqLockGuard,
// PendingTxLockGuard, ...) so the take/give mechanics live in one place while
// each lock keeps its own distinct, zero-argument guard type at call sites.
struct LazyMutexGuard {
    SemaphoreHandle_t& sem;
    explicit LazyMutexGuard(SemaphoreHandle_t& s) : sem(s) {
        if (!sem) sem = xSemaphoreCreateMutex();
        xSemaphoreTake(sem, portMAX_DELAY);
    }
    ~LazyMutexGuard() { xSemaphoreGive(sem); }
    LazyMutexGuard(const LazyMutexGuard&) = delete;
    LazyMutexGuard& operator=(const LazyMutexGuard&) = delete;
};
