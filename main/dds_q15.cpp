#include "dds_q15.h"

#include <atomic>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static constexpr unsigned QTABLE_BITS = 8;
static constexpr unsigned FRAC_BITS = 10;
static constexpr unsigned QTABLE_SIZE = 1u << QTABLE_BITS;
static constexpr unsigned VISIBLE_BITS = 2 + QTABLE_BITS + FRAC_BITS;
static constexpr unsigned PHASE_BITS = 64;

static int16_t s_sin_quarter[QTABLE_SIZE + 1];
static std::atomic<uint64_t> s_inc{0};
static uint64_t s_phase = 0;

// GFSK (not plain FSK): FT8 requires the frequency to glide smoothly between
// tones, shaped by a Gaussian pulse (BT=2.0). Hard frequency steps splatter
// energy across a wide band on every symbol transition. We can't precompute the
// whole per-sample frequency waveform here (no PSRAM — it'd be megabytes), so we
// keep the raw symbols and evaluate the Gaussian-shaped instantaneous frequency
// on the fly per sample using a small precomputed pulse table.
static uint8_t  s_cpfsk_symbols[DDS_CPFSK_MAX_SYMBOLS];
static size_t   s_cpfsk_symbol_count = 0;
static uint32_t s_cpfsk_samples_per_symbol = 0;
static uint32_t s_cpfsk_sample_counter = 0;
static double   s_cpfsk_base_inc = 0.0;     // 64-bit phase inc for base_hz (as double)
static double   s_cpfsk_spacing_inc = 0.0;  // 64-bit phase inc per 1.0 tone unit
static std::atomic<bool> s_cpfsk_active{false};

// Gaussian frequency-shaping pulse P(t), t in symbol units over [-1.5, 1.5].
// P(t) = (erf(K*BT*(t+0.5)) - erf(K*BT*(t-0.5)))/2, K = pi*sqrt(2/ln2). The
// three overlapping copies (t, t-1, t+1) sum to 1 (partition of unity), so a
// held tone yields a constant frequency. Sampled into a LUT + interpolated so
// we avoid erf() in the per-sample hot path.
static constexpr int GFSK_LUT_N = 512;
static float s_gfsk_pulse[GFSK_LUT_N];
static bool  s_gfsk_pulse_ready = false;

static void gfsk_pulse_init(void) {
    const double BT = 2.0;
    const double K_BT = 3.14159265358979323846 * sqrt(2.0 / log(2.0)) * BT; // ~10.6729
    for (int i = 0; i < GFSK_LUT_N; ++i) {
        double t = -1.5 + 3.0 * (double)i / (double)(GFSK_LUT_N - 1);
        s_gfsk_pulse[i] = (float)((erf(K_BT * (t + 0.5)) - erf(K_BT * (t - 0.5))) * 0.5);
    }
    s_gfsk_pulse_ready = true;
}

// Precomputed LUT-position scale = (GFSK_LUT_N-1)/3.0, so the per-sample hot
// path multiplies instead of dividing (3 calls/sample × 48k samples/s).
static constexpr float GFSK_POS_SCALE = (float)(GFSK_LUT_N - 1) / 3.0f;

// Evaluate the pulse at t in [-1.5, 1.5] via linear interpolation of the LUT.
static inline float gfsk_pulse_at(float t) {
    if (t <= -1.5f || t >= 1.5f) return 0.0f;
    float pos = (t + 1.5f) * GFSK_POS_SCALE;
    int idx = (int)pos;
    float frac = pos - (float)idx;
    if (idx >= GFSK_LUT_N - 1) return s_gfsk_pulse[GFSK_LUT_N - 1];
    return s_gfsk_pulse[idx] + (s_gfsk_pulse[idx + 1] - s_gfsk_pulse[idx]) * frac;
}

static inline uint64_t inc_from_hz(double frequency_hz) {
    long double numerator = static_cast<long double>(frequency_hz)
                          * static_cast<long double>(1ULL << 63) * 2.0L;
    return static_cast<uint64_t>(
        llroundl(numerator / static_cast<long double>(DDS_FS_HZ)));
}

void dds_init(void) {
    const double step = (M_PI / 2.0) / static_cast<double>(QTABLE_SIZE);
    for (unsigned n = 0; n <= QTABLE_SIZE; ++n) {
        double value = sin(step * static_cast<double>(n));
        int32_t q15 = static_cast<int32_t>(lround(value * 32767.0));
        if (q15 > 32767) q15 = 32767;
        if (q15 < -32768) q15 = -32768;
        s_sin_quarter[n] = static_cast<int16_t>(q15);
    }
}

void dds_set_freq_hz(double frequency_hz) {
    if (frequency_hz < 0.0) frequency_hz = 0.0;
    s_inc.store(inc_from_hz(frequency_hz), std::memory_order_relaxed);
}

void dds_reset_phase(void) {
    s_phase = 0;
}

bool dds_cpfsk_begin(double base_hz,
                     const uint8_t* symbols,
                     size_t symbol_count,
                     double tone_spacing_hz,
                     uint32_t samples_per_symbol) {
    if (!symbols || symbol_count == 0 ||
        symbol_count > DDS_CPFSK_MAX_SYMBOLS ||
        base_hz < 0.0 || tone_spacing_hz <= 0.0 ||
        samples_per_symbol == 0) {
        return false;
    }

    if (!s_gfsk_pulse_ready) gfsk_pulse_init();

    s_cpfsk_active.store(false, std::memory_order_release);
    for (size_t i = 0; i < symbol_count; ++i) s_cpfsk_symbols[i] = symbols[i];

    // Precompute the 64-bit phase increment for the carrier (base_hz) and for one
    // unit of tone spacing, as doubles so the per-sample GFSK frequency
    // (base + spacing * gaussian_weighted_tone) is just two mul/adds.
    s_cpfsk_base_inc    = (double)inc_from_hz(base_hz);
    s_cpfsk_spacing_inc = (double)inc_from_hz(tone_spacing_hz);

    s_phase = 0;
    s_cpfsk_symbol_count = symbol_count;
    s_cpfsk_samples_per_symbol = samples_per_symbol;
    s_cpfsk_sample_counter = 0;
    s_inc.store(inc_from_hz(base_hz + tone_spacing_hz * (double)symbols[0]),
                std::memory_order_relaxed);
    s_cpfsk_active.store(true, std::memory_order_release);
    return true;
}

void dds_cpfsk_end(void) {
    s_cpfsk_active.store(false, std::memory_order_release);
}

static inline int16_t sin_q15_from_phase(uint64_t phase) {
    uint32_t value =
        static_cast<uint32_t>(phase >> (PHASE_BITS - VISIBLE_BITS));
    uint32_t quadrant = value >> (QTABLE_BITS + FRAC_BITS);
    uint32_t position =
        value & ((1u << (QTABLE_BITS + FRAC_BITS)) - 1u);
    uint32_t index = position >> FRAC_BITS;
    uint32_t fraction = position & ((1u << FRAC_BITS) - 1u);

    if (quadrant & 1u) {
        index = QTABLE_SIZE -
                ((position + ((1u << FRAC_BITS) - 1u)) >> FRAC_BITS);
        fraction = (-fraction) & ((1u << FRAC_BITS) - 1u);
    }

    int32_t sample;
    if (index >= QTABLE_SIZE) {
        sample = s_sin_quarter[QTABLE_SIZE];
    } else {
        int32_t y0 = s_sin_quarter[index];
        int32_t y1 = s_sin_quarter[index + 1];
        int32_t interpolated =
            (y1 - y0) * static_cast<int32_t>(fraction);
        sample = y0 +
                 ((interpolated + (1 << (FRAC_BITS - 1))) >> FRAC_BITS);
    }

    if (quadrant >= 2u) sample = -sample;
    return static_cast<int16_t>(sample);
}

static inline void emit_stereo_frame(uint8_t* out, int16_t sample_q15) {
    int32_t sample_24 = static_cast<int32_t>(sample_q15) << 8;
    uint8_t b0 = static_cast<uint8_t>(sample_24 & 0xFF);
    uint8_t b1 = static_cast<uint8_t>((sample_24 >> 8) & 0xFF);
    uint8_t b2 = static_cast<uint8_t>((sample_24 >> 16) & 0xFF);
    out[0] = b0;
    out[1] = b1;
    out[2] = b2;
    out[3] = b0;
    out[4] = b1;
    out[5] = b2;
}

static inline void emit_silence(uint8_t*& out, unsigned frames) {
    while (frames-- > 0) {
        out[0] = out[1] = out[2] = 0;
        out[3] = out[4] = out[5] = 0;
        out += 6;
    }
}

void dds_render_24bit_stereo(uint8_t* out, unsigned frames) {
    uint64_t phase = s_phase;

    if (s_cpfsk_active.load(std::memory_order_acquire)) {
        const uint32_t samples_per_symbol = s_cpfsk_samples_per_symbol;
        const uint64_t total_samples =
            static_cast<uint64_t>(s_cpfsk_symbol_count) * samples_per_symbol;
        uint32_t counter = s_cpfsk_sample_counter;

        const size_t last_sym = s_cpfsk_symbol_count - 1;
        const float inv_sps = 1.0f / (float)samples_per_symbol;
        // Track the current symbol index and within-symbol position INCREMENTALLY
        // instead of `counter / samples_per_symbol` every sample — that 32-bit
        // divide (×48k samples/s) was a big chunk of the render cost that made the
        // writer fall behind and deliver a bursty/jittery stream (FT8 splatter).
        uint32_t j   = counter / samples_per_symbol;
        uint32_t pos = counter - j * samples_per_symbol;
        const int64_t base_inc_i = (int64_t)(s_cpfsk_base_inc + 0.5);

        for (; frames > 0; --frames) {
            if (counter >= total_samples) {
                emit_silence(out, frames);
                frames = 0;
                break;
            }

            float tau = (float)pos * inv_sps;                   // 0..1 within symbol

            // GFSK: instantaneous tone = Gaussian-weighted sum of the previous,
            // current, and next symbol. Edges reuse the first/last symbol (a
            // ramp), matching WSJT-X's dummy-symbol handling.
            float sm1 = (float)s_cpfsk_symbols[j > 0 ? j - 1 : 0];
            float s0  = (float)s_cpfsk_symbols[j];
            float sp1 = (float)s_cpfsk_symbols[j < last_sym ? j + 1 : last_sym];
            float w = sm1 * gfsk_pulse_at(tau + 0.5f)
                    + s0  * gfsk_pulse_at(tau - 0.5f)
                    + sp1 * gfsk_pulse_at(tau - 1.5f);

            // base is integer-exact; only the small spacing*w term needs a mul.
            uint64_t increment =
                (uint64_t)(base_inc_i + (int64_t)(s_cpfsk_spacing_inc * (double)w + 0.5));

            emit_stereo_frame(out, sin_q15_from_phase(phase));
            out += 6;
            phase += increment;
            ++counter;
            if (++pos >= samples_per_symbol) { pos = 0; ++j; }  // next symbol, no divide
        }

        s_cpfsk_sample_counter = counter;
        s_phase = phase;
        return;
    }

    uint64_t increment = s_inc.load(std::memory_order_relaxed);
    for (unsigned n = 0; n < frames; ++n) {
        emit_stereo_frame(out, sin_q15_from_phase(phase));
        out += 6;
        phase += increment;
    }
    s_phase = phase;
}
