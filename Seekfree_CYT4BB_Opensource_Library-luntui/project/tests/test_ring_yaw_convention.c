#include "ring_test_geometry.h"
#include "ring_runtime.h"
#include "ring_link.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "FAIL yaw line %d: %s ph=%d route=%d e=%d raw=%.3f net=%.3f branch=%u flags=%u decision=%u\n", \
        __LINE__, #x, ring_result.phase, ring_result.route, ring_result.steer_error, \
        motion.yaw_deg, ring_result.turn_deg, ring_result.branch_ok, \
        ring_result.entry_flags, ring_result.decision); exit(1); } } while (0)

static uint8_t image[RING_H][RING_W];
static ring_motion_t motion;
static unsigned frames;
static float absolute_yaw;

/* These are the existing main's positive P relationship and saturation,
 * not a substitute yaw convention. The hardware/log convention is that a
 * sustained positive turn output produces positive RAW gyro Z/heading.
 * Neither the route nor RING_YAW_SIGN chooses the gyro sample below. */
static float original_turn_p(int16_t error)
{
    float e = (float)error;
    return 1.35f * e / (1.0f + fabsf(e) / 40.0f);
}

static int sign_of(float value)
{
    return value > 0.0f ? 1 : (value < 0.0f ? -1 : 0);
}

static void read_runtime_motion(float wheel_revs)
{
    ring_packet_t packet;
    float target = 100.0f, turn = 0.0f;
    /* No command is published: the real runtime still publishes motion,
     * while the original controller's outputs must remain untouched. */
    ring_runtime_apply(&target, &turn, wheel_revs, 1);
    CHECK(target == 100.0f && turn == 0.0f);
    CHECK(ring_link_read(ring_motion_mailbox, &packet));
    CHECK((packet.word[RM_READY] & 3) == 3);
    motion.now_ms = (uint32_t)packet.word[RM_MS];
    motion.yaw_deg = packet.word[RM_YAW_MDEG] / 1000.0f;
    motion.wheel_revs = packet.word[RM_WHEEL_MREV] / 1000.0f;
    motion.ready = 1;
}

static void step(int raw_turn_sign, float forward_revs)
{
    int i, y;
    float previous_heading = motion.yaw_deg;
    float wheel_revs = motion.wheel_revs + forward_revs;
    /* Bias is 10 raw counts. A signed 1000-count deviation, at the real
     * runtime's 0.1 degree/s/count scale, integrates two degrees in 20 ms. */
    for (i = 0; i < 20; ++i)
        (void)ring_runtime_tick((int16_t)(10 + raw_turn_sign * 1000), 1);
    read_runtime_motion(wheel_revs);
    absolute_yaw += fabsf(motion.yaw_deg - previous_heading);
    ring_observe_track(1);
    ring_observe_frame_interval(20);
    ring_update((const uint8_t (*)[RING_W])image, &motion);
    ++frames;
    CHECK(!ring_result.fault && !ring_result.stop);
    if (ring_result.branch_ok)
        for (y = 0; y < RING_H; ++y)
            if (ring_result.mid[y] >= 0)
                CHECK(image[y][ring_result.mid[y]] != 0);
}

static int enter_from_geometry(int route)
{
    int i, lap_turn_sign;
    memset(ring_command_mailbox, 0, sizeof(int32_t) * RING_LINK_WORDS);
    memset(ring_motion_mailbox, 0, sizeof(int32_t) * RING_LINK_WORDS);
    ring_runtime_init(0.1f);
    ring_runtime_set_operator(1, 0);
    for (i = 0; i < 2100; ++i) (void)ring_runtime_tick(10, 0);
    (void)ring_runtime_tick(10, 1);
    motion = (ring_motion_t){0, 0, 0, 1};
    read_runtime_motion(0.0f);
    absolute_yaw = 0.0f;
    ring_init();
    rg_entry(image, route, 0, 0);
    for (i = 0; i < RING_CONFIRM_FRAMES; ++i) step(0, 0.0f);
    CHECK(ring_result.phase == RING_APPROACH && ring_result.route == route);
    CHECK(ring_result.detected_events == 1 && !ring_result.committed);

    /* Pixel geometry really advances from an inlet into a tangent ring.
     * Zero yaw here deliberately isolates visual entry history; this is
     * synthetic geometry, not a replay or a vehicle dynamics simulation. */
    for (i = 1; i <= 12; ++i) {
        rg_transition(image, route, i, 12);
        step(0, 0.03f);
        CHECK(ring_result.branch_ok && ring_result.path_source == 1);
        CHECK(!ring_result.committed && ring_result.phase != RING_CIRCLE);
    }
    rg_orbit(image, route, 90);
    step(0, 0.03f);
    CHECK(ring_result.branch_ok && ring_result.path_source == 1);
    lap_turn_sign = sign_of(original_turn_p(ring_result.steer_error));
    CHECK(lap_turn_sign != 0);
    /* A right-of-island route bends left in this tangent view, and its
     * mirrored left-of-island route bends right. This expectation comes
     * from pixels and the unchanged positive P controller, not SIGN. */
    CHECK(lap_turn_sign * route < 0);
    for (i = 0; i < 40 && ring_result.phase != RING_CIRCLE; ++i) {
        CHECK(sign_of(original_turn_p(ring_result.steer_error)) == lap_turn_sign);
        step(lap_turn_sign, 0.03f);
    }
    CHECK(motion.yaw_deg * lap_turn_sign > 35.0f);
    CHECK(ring_result.phase == RING_CIRCLE && ring_result.committed);
    CHECK(ring_result.turn_deg > 35.0f && ring_result.turn_deg < 100.0f);
    return lap_turn_sign;
}

static void full_lap_and_sway(int route)
{
    int i, lap_turn_sign = enter_from_geometry(route);
    float circle_start = ring_result.turn_deg, forward_end;
    float swing_absolute_start = absolute_yaw;
    for (i = 0; i < 80; ++i) step(lap_turn_sign, 0.03f);
    forward_end = ring_result.turn_deg;
    CHECK(fabsf(forward_end - circle_start - 160.0f) < 0.1f);
    /* An imposed reverse yaw disturbance must subtract from the net lap,
     * even though the chosen visual branch and steering retain identity. */
    for (i = 0; i < 80; ++i) step(-lap_turn_sign, 0.03f);
    CHECK(fabsf(ring_result.turn_deg - circle_start) < 0.1f);
    for (i = 0; i < 80; ++i) step(lap_turn_sign, 0.03f);
    CHECK(fabsf(ring_result.turn_deg - forward_end) < 0.1f);
    CHECK(absolute_yaw - swing_absolute_start > 479.0f);
    CHECK(ring_result.turn_deg < RING_EXIT_MIN_DEG);
    CHECK(ring_result.phase == RING_CIRCLE && ring_result.exited_events == 0);

    /* An actual opening is necessary but cannot replace a NET full lap.
     * Keep the already measured tangent turn sign through the aperture;
     * its source is the orbit's pixel path, not route*SIGN. */
    rg_exit(image, route);
    for (i = 0; i < RING_EXIT_CONFIRM_FRAMES + 1; ++i) {
        step(0, 0.03f);
        CHECK(ring_result.phase == RING_CIRCLE);
        CHECK(ring_result.branch_ok && ring_result.exited_events == 0);
    }
    for (i = 0; i < 120 && ring_result.phase == RING_CIRCLE; ++i) {
        step(lap_turn_sign, 0.03f);
        if (ring_result.turn_deg < RING_EXIT_MIN_DEG)
            CHECK(ring_result.phase == RING_CIRCLE);
    }
    CHECK(ring_result.phase == RING_EXIT);
    CHECK(ring_result.turn_deg >= RING_EXIT_MIN_DEG);
    CHECK(ring_result.detected_events == 1 && ring_result.exited_events == 0);
    rg_road(image, 0);
    for (i = 0; i < 2 * RING_RECOVER_FRAMES + 2 &&
                ring_result.phase != RING_COOLDOWN; ++i) step(0, 0.03f);
    CHECK(ring_result.phase == RING_COOLDOWN && ring_result.exited_events == 1);
    printf("PASS: pixel route=%d -> P/RAW gyro sign=%d -> signed net full lap, sway cancellation and exit\n",
           route, lap_turn_sign);
}

int main(void)
{
    full_lap_and_sway(-1);
    full_lap_and_sway(1);
    printf("PASS: %u direction/runtime integration frames (synthetic geometry, not field replay)\n", frames);
    return 0;
}
