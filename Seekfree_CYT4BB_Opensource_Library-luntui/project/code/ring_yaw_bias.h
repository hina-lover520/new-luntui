#ifndef CODE_RING_YAW_BIAS_H_
#define CODE_RING_YAW_BIAS_H_

#include <stdint.h>
#include <math.h>
#include <string.h>

/* Ring-only raw Z bias. No IMU, balance state, heading, motor or wait is owned
 * here. Call update once per 1 ms tick; integrate rate only while standing.
 * Range/mean checks reject motion with varying rate. Means above10dps are
 * rejected as implausible bias. Slower constant rotation cannot be
 * distinguished from an offset using gyro Z alone, so idle windows continue
 * to replace an earlier estimate until RUN freezes it. */
#define RING_YAW_BIAS_STARTUP_MS       1000u
#define RING_YAW_BIAS_WINDOW_MS         250u
#define RING_YAW_BIAS_MIN_SAMPLES       125u
#define RING_YAW_BIAS_MAX_GAP_MS          5u
#define RING_YAW_BIAS_RANGE_DPS         3.0f
#define RING_YAW_BIAS_MEAN_TOL_DPS      1.0f
#ifndef RING_YAW_BIAS_MAX_ABS_DPS
#define RING_YAW_BIAS_MAX_ABS_DPS      10.0f
#endif

enum {
    RING_YAW_BIAS_STARTUP = 0,
    RING_YAW_BIAS_COLLECTING = 1,
    RING_YAW_BIAS_FIRST_STABLE = 2,
    RING_YAW_BIAS_ACCEPTED = 3,
    RING_YAW_BIAS_UNSTABLE = 4,
    RING_YAW_BIAS_RUN_FROZEN = 5,
    RING_YAW_BIAS_STANDING_FROZEN = 6,
    RING_YAW_BIAS_INVALID_SCALE = 7
};

typedef struct {
    uint32_t boot_ms, window_started_ms, last_sample_ms;
    uint32_t accepted, updated_ms;
    float scale, bias, raw_dps;
    float sum_dps, minimum_dps, maximum_dps, previous_mean_dps;
    uint16_t samples;
    int16_t raw;
    unsigned ready, warmup_done, window_active, previous_stable;
    unsigned last_sample_valid, window_state;
} ring_yaw_bias_t;

static inline void ring_yaw_bias_init_at(ring_yaw_bias_t *s, float scale, uint32_t now)
{
    memset(s, 0, sizeof(*s));
    s->scale = scale;
    s->boot_ms = now;
    s->window_state = RING_YAW_BIAS_STARTUP;
}

static inline void ring_yaw_bias_init(ring_yaw_bias_t *s, float scale)
{
    ring_yaw_bias_init_at(s, scale, 0);
}

static inline void ring_yaw_bias_clear_window(ring_yaw_bias_t *s)
{
    s->window_active = s->previous_stable = s->last_sample_valid = 0;
    s->samples = 0;
    s->sum_dps = 0.0f;
}

static inline void ring_yaw_bias_update(ring_yaw_bias_t *s,
    int16_t raw, uint32_t now, int standing, int run)
{
    float mean, dps_per_count = s->scale;
    int stable;
    s->raw = raw;
    s->raw_dps = (float)raw * dps_per_count;
    if (!isfinite(dps_per_count) || dps_per_count <= 0.0f ||
        !isfinite(s->raw_dps)) {
        ring_yaw_bias_clear_window(s);
        s->window_state = RING_YAW_BIAS_INVALID_SCALE;
        return;
    }
    /* Standing always freezes, even if no usable estimate exists. RUN also
     * freezes a ready estimate before servos start moving. If RUN was already
     * set at boot, an unready estimate may finish initial calibration; motion
     * must still pass both complete windows and cannot re-calibrate afterward. */
    if (standing || (run && s->ready)) {
        ring_yaw_bias_clear_window(s);
        s->window_state = standing ? RING_YAW_BIAS_STANDING_FROZEN :
            RING_YAW_BIAS_RUN_FROZEN;
        return;
    }
    if (!s->warmup_done) {
        if (now - s->boot_ms < RING_YAW_BIAS_STARTUP_MS) {
            ring_yaw_bias_clear_window(s);
            s->window_state = RING_YAW_BIAS_STARTUP;
            return;
        }
        s->warmup_done = 1;
    }
    if (s->last_sample_valid) {
        uint32_t gap = now - s->last_sample_ms;
        if (!gap) return; /* a repeated poll cannot manufacture samples */
        if (gap > RING_YAW_BIAS_MAX_GAP_MS)
            ring_yaw_bias_clear_window(s);
    }
    if (!s->window_active) {
        s->window_active = 1;
        s->window_started_ms = now;
        s->samples = 0;
        s->sum_dps = 0.0f;
        s->window_state = RING_YAW_BIAS_COLLECTING;
    }
    s->last_sample_ms = now;
    s->last_sample_valid = 1;
    if (!s->samples) s->minimum_dps = s->maximum_dps = s->raw_dps;
    if (s->raw_dps < s->minimum_dps) s->minimum_dps = s->raw_dps;
    if (s->raw_dps > s->maximum_dps) s->maximum_dps = s->raw_dps;
    s->sum_dps += s->raw_dps;
    ++s->samples;
    if (now - s->window_started_ms < RING_YAW_BIAS_WINDOW_MS) return;

    mean = s->sum_dps / (float)s->samples;
    stable = s->samples >= RING_YAW_BIAS_MIN_SAMPLES &&
        s->maximum_dps - s->minimum_dps <= RING_YAW_BIAS_RANGE_DPS &&
        fabsf(mean) <= RING_YAW_BIAS_MAX_ABS_DPS;
    if (stable) {
        if (s->previous_stable &&
            fabsf(mean - s->previous_mean_dps) <= RING_YAW_BIAS_MEAN_TOL_DPS) {
            s->bias = (mean + s->previous_mean_dps) * 0.5f / dps_per_count;
            s->ready = 1;
            ++s->accepted;
            s->updated_ms = now;
            s->window_state = RING_YAW_BIAS_ACCEPTED;
        } else s->window_state = RING_YAW_BIAS_FIRST_STABLE;
        s->previous_mean_dps = mean;
        s->previous_stable = 1;
    } else {
        s->previous_stable = 0;
        s->window_state = RING_YAW_BIAS_UNSTABLE;
    }
    /* Do not reuse a boundary sample. The next window starts at this time and
     * collects subsequent ticks, making two complete windows ready at1500ms. */
    s->window_started_ms = now;
    s->samples = 0;
    s->sum_dps = 0.0f;
}

static inline float ring_yaw_bias_rate(const ring_yaw_bias_t *s,
    int16_t raw)
{
    float dps_per_count = s->scale;
    if (!s->ready || !isfinite(dps_per_count) || dps_per_count <= 0.0f)
        return 0.0f;
    return ((float)raw - s->bias) * dps_per_count;
}

#endif
