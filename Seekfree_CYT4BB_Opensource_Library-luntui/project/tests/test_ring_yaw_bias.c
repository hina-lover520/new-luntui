#include "ring_yaw_bias.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(c) do { if (!(c)) { \
    fprintf(stderr, "FAIL yaw bias:%d %s\n", __LINE__, #c); exit(1); \
} } while (0)
#define NEAR(a,b) (fabsf((a) - (b)) < 0.01f)

static void feed(ring_yaw_bias_t *s, uint32_t first, uint32_t last,
    int16_t raw, int standing, int run)
{
    uint32_t now;
    for (now = first; now <= last; ++now)
        ring_yaw_bias_update(s, raw, now, standing, run);
}

static void startup_and_offset(void)
{
    ring_yaw_bias_t s;
    ring_yaw_bias_init(&s, 0.1f);
    feed(&s, 0, 999, 73, 0, 0);
    CHECK(!s.ready && !s.samples && s.window_state == RING_YAW_BIAS_STARTUP);
    feed(&s, 1000, 1249, 73, 0, 0);
    CHECK(!s.ready);
    feed(&s, 1250, 1499, 73, 0, 0);
    CHECK(!s.ready);
    feed(&s, 1500, 1500, 73, 0, 0);
    CHECK(s.ready && NEAR(s.bias, 73.0f));
    CHECK(NEAR(ring_yaw_bias_rate(&s, 73), 0.0f));
    CHECK(s.accepted == 1 && s.updated_ms == 1500 && s.raw == 73);
}

static void implausible_constant_rotation(void)
{
    ring_yaw_bias_t s;
    ring_yaw_bias_init(&s, 0.1f);
    feed(&s, 0, 2000, 1000, 0, 1);
    CHECK(!s.ready && s.accepted == 0);
    CHECK(s.window_state == RING_YAW_BIAS_UNSTABLE);
    feed(&s, 2001, 2500, 25, 0, 1);
    CHECK(s.ready && NEAR(s.bias, 25.0f));
}

static void noisy_start_then_still(void)
{
    ring_yaw_bias_t s;
    uint32_t now;
    ring_yaw_bias_init(&s, 0.1f);
    for (now = 0; now <= 2000; ++now)
        ring_yaw_bias_update(&s, (int16_t)(now % 2 ? -100 : 100),
            now, 0, 0);
    CHECK(!s.ready && s.window_state == RING_YAW_BIAS_UNSTABLE);
    feed(&s, 2001, 2500, 20, 0, 0);
    CHECK(s.ready && NEAR(s.bias, 20.0f));
}

static void changing_means_need_consecutive_windows(void)
{
    ring_yaw_bias_t s;
    ring_yaw_bias_init(&s, 0.1f);
    feed(&s, 0, 1250, 0, 0, 0);
    CHECK(!s.ready && s.previous_stable);
    feed(&s, 1251, 1500, 20, 0, 0); /* individually stable, mean differs2dps */
    CHECK(!s.ready && s.window_state == RING_YAW_BIAS_FIRST_STABLE);
    feed(&s, 1501, 1750, 20, 0, 0);
    CHECK(s.ready && NEAR(s.bias, 20.0f));
}

static void idle_recalibration(void)
{
    ring_yaw_bias_t s;
    uint32_t now;
    ring_yaw_bias_init(&s, 0.1f);
    /* Z alone cannot reject a constant turn. Later genuine rest must replace
     * this estimate before RUN, rather than latch a startup mistake forever. */
    feed(&s, 0, 2000, 70, 0, 0);
    CHECK(s.ready && NEAR(s.bias, 70.0f));
    for (now = 2001; now <= 2500; ++now)
        ring_yaw_bias_update(&s, (int16_t)(now % 2 ? -100 : 100),
            now, 0, 0);
    CHECK(s.ready && NEAR(s.bias, 70.0f));
    feed(&s, 2501, 3000, -30, 0, 0);
    CHECK(s.ready && NEAR(s.bias, -30.0f));
    CHECK(NEAR(ring_yaw_bias_rate(&s, -30), 0.0f));
}

static void run_and_standing_freeze(void)
{
    ring_yaw_bias_t s;
    unsigned accepted;
    float heading = 0.0f;
    uint32_t now;
    ring_yaw_bias_init(&s, 0.1f);
    feed(&s, 0, 2000, 10, 0, 0);
    CHECK(s.ready);
    accepted = s.accepted;
    feed(&s, 2001, 3000, 400, 0, 1);
    CHECK(s.window_state == RING_YAW_BIAS_RUN_FROZEN);
    CHECK(NEAR(s.bias, 10.0f) && s.accepted == accepted);
    for (now = 3001; now <= 4000; ++now) {
        ring_yaw_bias_update(&s, 1010, now, 1, 1);
        heading += ring_yaw_bias_rate(&s, 1010) * 0.001f;
    }
    CHECK(NEAR(heading, 100.0f));
    for (now = 4001; now <= 5000; ++now) {
        ring_yaw_bias_update(&s, -990, now, 1, 0);
        heading += ring_yaw_bias_rate(&s, -990) * 0.001f;
    }
    CHECK(NEAR(heading, 0.0f));
    CHECK(NEAR(s.bias, 10.0f) && s.accepted == accepted);
    CHECK(s.window_state == RING_YAW_BIAS_STANDING_FROZEN);
}

static void early_run_and_unready_standing(void)
{
    ring_yaw_bias_t s;
    ring_yaw_bias_init(&s, 0.1f);
    feed(&s, 0, 2000, 10, 0, 1); /* preserve existing boot with RUN already set */
    CHECK(s.ready && NEAR(s.bias, 10.0f));
    CHECK(s.window_state == RING_YAW_BIAS_RUN_FROZEN);
    ring_yaw_bias_init(&s, 0.1f);
    feed(&s, 0, 3000, 1010, 1, 1);
    CHECK(!s.ready && NEAR(ring_yaw_bias_rate(&s, 1010), 0.0f));
}

static void gaps_duplicates_invalid_and_wrap(void)
{
    ring_yaw_bias_t s;
    uint32_t now, boot = UINT32_MAX - 1200u;
    ring_yaw_bias_init(&s, 0.1f);
    feed(&s, 0, 1250, 10, 0, 0);
    ring_yaw_bias_update(&s, 10, 1300, 0, 0);
    CHECK(!s.previous_stable && s.samples == 1);
    ring_yaw_bias_update(&s, 10, 1300, 0, 0);
    CHECK(s.samples == 1 && !s.ready);
    s.scale = 0.0f;
    ring_yaw_bias_update(&s, 10, 1301, 0, 0);
    CHECK(!s.ready && !s.samples && s.window_state == RING_YAW_BIAS_INVALID_SCALE);
    CHECK(ring_yaw_bias_rate(&s, 10) == 0.0f);
    s.scale = NAN;
    ring_yaw_bias_update(&s, 10, 1302, 0, 0);
    CHECK(!s.ready && s.window_state == RING_YAW_BIAS_INVALID_SCALE);
    ring_yaw_bias_init_at(&s, 0.1f, boot);
    for (now = 0; now <= 1500; ++now)
        ring_yaw_bias_update(&s, -15, boot + now, 0, 0);
    CHECK(s.ready && NEAR(s.bias, -15.0f));
}

int main(void)
{
    startup_and_offset();
    implausible_constant_rotation();
    noisy_start_then_still();
    changing_means_need_consecutive_windows();
    idle_recalibration();
    run_and_standing_freeze();
    early_run_and_unready_standing();
    gaps_duplicates_invalid_and_wrap();
    puts("PASS: two stable250ms windows ready at1500ms; noisy/moving/implausible startup rejected; idle recalibration; RUN/standing freeze; true signed100dps preserved; mean continuity; diagnostics/gaps/duplicates/invalid scale/wrap");
    return 0;
}
