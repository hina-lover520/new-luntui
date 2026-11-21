#include "ring_vision.h"
#include "ring_runtime.h"
#include "ring_link.h"
#include "ring_test_geometry.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL line %d: %s (phase=%d)\n", __LINE__, #x, ring_result.phase); exit(1); } } while (0)
static uint8_t image[RING_H][RING_W], saved[RING_H][RING_W];
static unsigned beep_ons, beep_offs, normal_cross_calls, frames;
static float wheel_revs;
static uint8_t standing;
static int16_t gyro_raw = 10;
static int published_error, published_state;
static float target, turn;

static void tick(unsigned ms)
{
    unsigned i;
    for (i = 0; i < ms; ++i) {
        int8_t event = ring_runtime_tick(gyro_raw, standing);
        if (event == 1) ++beep_ons;
        if (event == 0) ++beep_offs;
        if (standing) wheel_revs += 90.0f / 60000.0f;
    }
}

static void road(void)
{
    rg_road(image, 0);
}

static void island(int tip)
{
    rg_entry(image, 1, tip >= RING_ENTRY_ROW ? 13 : 0, 0);
}

static void frame(int normal_error, int normal_state)
{
    int y;
    tick(20);
    memcpy(saved, image, sizeof(image));
    ring_vision_before_frame((const uint8_t (*)[RING_W])image);
    ++normal_cross_calls; /* Original cross processing still runs; selection wins. */
    published_error = normal_error;
    published_state = normal_state;
    ring_vision_select(&published_error, &published_state);
    /* The real CM7_0 publishes the original array at THIS point. */
    ring_vision_after_frame((const uint8_t (*)[RING_W])image);
    target = 190.0f;
    turn = 17.25f;
    ring_runtime_apply(&target, &turn, wheel_revs, published_state);
    CHECK(memcmp(image, saved, sizeof(image)) == 0);
    if (ring_result.path_valid && (ring_result.path_source == 1 || ring_result.path_source == 4))
        for (y = 0; y < RING_H; ++y)
            if (ring_result.mid[y] >= 0) CHECK(image[y][ring_result.mid[y]]);
    ++frames;
}

static void restart(void)
{
    ring_runtime_init(0.1f);
    ring_vision_init();
    beep_ons = beep_offs = normal_cross_calls = frames = 0;
    standing = 0;
    gyro_raw = 10;
    wheel_revs = 0.0f;
    road();
}

static void startup(void)
{
    int i;
    ring_runtime_set_operator(1, 0);
    /* Actual 1ms ticks collect the bias; yaw is NOT injected into the module. */
    for (i = 0; i < 110; ++i) frame(13, 1);
    standing = 1;
    frame(13, 1);
}

static void command(int cap, int phase, unsigned detected, unsigned exited,
                    uint32_t command_epoch, uint32_t *seq)
{
    ring_packet_t p = {{0}};
    p.word[RL_MAGIC] = RING_LINK_MAGIC;
    p.word[RL_EPOCH] = (int32_t)command_epoch;
    p.word[RC_CAP] = ring_command_cap_turn(cap, 0);
    p.word[RC_PHASE] = phase;
    p.word[RC_DETECTED] = (int32_t)detected;
    p.word[RC_EXITED] = (int32_t)exited;
    ring_link_write(ring_command_mailbox, &p, seq);
}

int main(void)
{
    int i;
    ring_packet_t p, previous;
    uint32_t seq = 200, epoch;
    restart();
    island(65);
    for (i = 0; i < 110; ++i) {
        frame(31, 1);
        CHECK(ring_result.phase == RING_NORMAL);
        CHECK(published_error == 31 && published_state == 1);
        CHECK(target == 190.0f && turn == 17.25f);
    }
    CHECK(beep_ons == 1 && beep_offs == 1); /* Standby recognition, no control. */
    beep_ons = beep_offs = 0;
    ring_runtime_set_operator(1, 0);
    standing = 1;
    road();
    for (i = 0; i < 40; ++i) {
        frame(i - 20, i % 3);
        CHECK(published_error == i - 20 && published_state == i % 3);
        CHECK(target == 190.0f && turn == 17.25f);
    }
    CHECK(normal_cross_calls == frames);
    tick(1000); /* An old NORMAL packet must never stop normal tracking. */
    CHECK(!ring_runtime_watchdog_stop());
    target = 237.5f; turn = -53.0f;
    ring_runtime_apply(&target, &turn, wheel_revs, published_state);
    CHECK(target == 237.5f && turn == -53.0f);

    CHECK(ring_link_read(ring_motion_mailbox, &p));
    epoch = (uint32_t)p.word[RL_EPOCH];
    previous = p;
    ring_motion_mailbox[RL_SEQUENCE] |= 1;
    CHECK(!ring_link_read(ring_motion_mailbox, &p));
    CHECK(memcmp(&p, &previous, sizeof(p)) == 0);
    ring_motion_mailbox[RL_SEQUENCE] &= ~1;
    command(0, RING_FAULT, 1, 1, epoch - 1, &seq); /* retained prior boot */
    target = 190.0f; turn = 17.25f;
    ring_runtime_apply(&target, &turn, wheel_revs, published_state);
    CHECK(target == 190.0f && turn == 17.25f);
    tick(250);
    CHECK(beep_ons == 0);

    /* No acquired ring: malformed or unmatched commands cannot park a car. */
    CHECK(ring_link_read(ring_motion_mailbox, &p));
    epoch = (uint32_t)p.word[RL_EPOCH];
    command(0, RING_FAULT, 1, 0, epoch, &seq);
    ring_runtime_apply(&target, &turn, wheel_revs, 1);
    CHECK(target == 190.0f && turn == 17.25f);
    command(0, RING_NORMAL, 1, 0, epoch, &seq);
    ring_runtime_apply(&target, &turn, wheel_revs, 1);
    CHECK(target == 190.0f && turn == 17.25f);
    command(90, RING_APPROACH, 1, 0, epoch, &seq);
    ring_runtime_apply(&target, &turn, wheel_revs, 1);
    CHECK(target == 190.0f && turn == 17.25f);
    tick(RING_LINK_TIMEOUT_MS + 1);
    CHECK(!ring_runtime_watchdog_stop());

    restart(); startup();
    gyro_raw = 1010; /* 100 deg/s after subtracting the sampled 10-count bias. */
    tick(1000);
    ring_runtime_apply(&target, &turn, wheel_revs, published_state);
    CHECK(ring_link_read(ring_motion_mailbox, &p));
    CHECK(fabsf(p.word[RM_YAW_MDEG] * 0.001f - 100.0f) < 0.01f);
    CHECK(beep_ons == 0);

    restart(); startup();
    island(65);
    for (i = 0; i < RING_CONFIRM_FRAMES - 1; ++i) {
        frame(-11, 1);
        CHECK(published_error == -11 && published_state == 1);
        CHECK(target == 190.0f && turn == 17.25f);
        CHECK(!ring_blocks_cross());
    }
    frame(-11, 1);
    CHECK(ring_result.phase == RING_APPROACH && ring_owns_path());
    CHECK(ring_result.path_valid && target == 190.0f); /* not yet handed over */
    frame(-11, 1);
    CHECK(published_error == ring_result.steer_error && published_state == 3);
    CHECK(published_error > 0 && target == RING_APPROACH_RPM);
    frame(-11, 1);
    CHECK(beep_ons == 1);
    /* Repeated event counter snapshots must not restart/extend a beep. */
    for (i = 0; i < 8; ++i) frame(-11, 1);
    CHECK(beep_ons == 1 && beep_offs == 1);

    for (i = 1; i <= 4; ++i) {
        static const int a[] = {3, 6, 9, 13};
        rg_entry(image, 1, a[i - 1], 0); frame(-11, 1);
    }
    CHECK(ring_result.phase == RING_ENTER);
    /* Route-progress fixture at 200 deg/s, converted to the board's raw Z
     * convention; the independent convention test derives sign from pixels. */
    gyro_raw = (int16_t)(10 + RING_YAW_SIGN * 2000.0f);
    for (i = 1; i <= 12; ++i) {
        rg_transition(image, 1, i, 12); frame(0, 1);
        CHECK(ring_result.branch_ok && ring_result.path_source == 1);
    }
    rg_orbit(image, 1, 90);
    for (i = 0; i < 16 && ring_result.phase != RING_CIRCLE; ++i) frame(0, 1);
    CHECK(ring_result.phase == RING_CIRCLE);
    rg_exit(image, 1);
    for (i = 0; i < 80 && ring_result.turn_deg < 180.0f; ++i) {
        frame(0, 1);
        CHECK(ring_result.phase == RING_CIRCLE);
    }
    CHECK(ring_result.exited_events == 0);
    for (i = 0; i < 80 && ring_result.phase == RING_CIRCLE; ++i) frame(0, 1);
    CHECK(ring_result.phase == RING_EXIT);
    CHECK(ring_result.turn_deg >= RING_EXIT_MIN_DEG);
    CHECK(beep_ons == 1);
    gyro_raw = 10;
    road();
    for (i = 0; i < 40 && ring_result.phase != RING_COOLDOWN; ++i) frame(37, 1);
    CHECK(ring_result.phase == RING_COOLDOWN);
    CHECK(published_error == 37 && published_state == 1 && target == 190.0f);
    island(82);
    for (i = 0; i < 85; ++i) frame(37, 1);
    CHECK(ring_result.phase == RING_COOLDOWN);
    CHECK(beep_ons == 3 && beep_offs == 3);
    CHECK(ring_result.detected_events == 1 && ring_result.exited_events == 1);
    CHECK(target == 190.0f && published_error == 37);

    /* Acquire before trusting a ring command or completion event. */
    restart(); startup();
    CHECK(ring_link_read(ring_motion_mailbox, &p));
    epoch = (uint32_t)p.word[RL_EPOCH];
    seq = 200;
    published_state = 3;
    command(90, RING_CIRCLE, 1, 0, epoch, &seq);
    ring_runtime_apply(&target, &turn, wheel_revs, published_state);
    command(-1, RING_COOLDOWN, 1, 1, epoch, &seq);
    ring_runtime_apply(&target, &turn, wheel_revs, published_state);
    tick(500);
    CHECK(beep_ons == 3 && beep_offs == 3);
    command(90, RING_CIRCLE, 1, 1, epoch, &seq);
    ring_runtime_apply(&target, &turn, wheel_revs, published_state);
    tick(RING_LINK_TIMEOUT_MS + 1);
    CHECK(ring_runtime_watchdog_stop());
    target = 190.0f; turn = 17.25f;
    ring_runtime_apply(&target, &turn, wheel_revs, published_state);
    CHECK(target == 0.0f && turn == 0.0f);
    command(-1, RING_NORMAL, 1, 1, epoch, &seq);
    target = 190.0f; turn = 17.25f;
    ring_runtime_apply(&target, &turn, wheel_revs, published_state);
    CHECK(target == 190.0f && turn == 17.25f && !ring_runtime_watchdog_stop());

    /* Restarting the motion core changes its epoch and rejects old ring caps. */
    command(0, RING_FAULT, 1, 1, epoch, &seq);
    ring_runtime_init(0.1f);
    target = 190.0f; turn = 17.25f;
    ring_runtime_apply(&target, &turn, wheel_revs, published_state);
    CHECK(target == 190.0f && turn == 17.25f);
    CHECK(ring_link_read(ring_motion_mailbox, &p));
    CHECK((uint32_t)p.word[RL_EPOCH] != epoch);
    /* raw=2 alone is ambiguous: the real curved branch is allowed to acquire.
     * A subsequent geometric cross cancels before actual entry/commitment. */
    restart(); startup(); island(65);
    for (i = 0; i <= RING_CONFIRM_FRAMES; ++i) frame(-23, 2);
    CHECK(ring_owns_path() && published_state == 3);
    CHECK(ring_result.detected_events == 1 && !ring_result.cross_veto);
    rg_cross(image, 0);
    for (i = 0; i < RING_CANCEL_FRAMES; ++i) frame(-23, 2);
    CHECK(ring_result.phase == RING_COOLDOWN);
    CHECK(published_error == -23 && published_state == 2);
    CHECK(target == 190.0f && turn == 17.25f);
    tick(500);
    CHECK(beep_ons == 1 && beep_offs == 1 && !ring_result.exited_events);
    restart(); startup(); rg_cross(image, 1);
    for (i = 0; i < 12; ++i) {
        frame(-23, 1);
        CHECK(published_error == -23 && published_state == 1);
        CHECK(target == 190.0f && turn == 17.25f);
    }
    CHECK(!ring_result.detected_events && !beep_ons);
    rg_conflict(image, 1);
    for (i = 0; i < 12; ++i) frame(-23, 2);
    CHECK(!ring_result.detected_events && !beep_ons);
    CHECK(published_error == -23 && published_state == 2 && target == 190.0f);

    /* Compensation cannot leak into candidates/normal tracking, invalid
     * packets, disabled forward motion, or a completed cancellation. */
    restart(); startup();
    CHECK(ring_link_read(ring_motion_mailbox, &p));
    epoch = (uint32_t)p.word[RL_EPOCH]; seq = 200;
    command(90, ring_command_phase(RING_ENTER, 45), 1, 0, epoch, &seq);
    target = 190.0f; turn = 17.25f;
    ring_runtime_apply(&target, &turn, wheel_revs, 1);
    CHECK(target == 190.0f && turn == 17.25f);
    command(90, ring_command_phase(RING_ENTER, 46), 1, 0, epoch, &seq);
    ring_runtime_apply(&target, &turn, wheel_revs, 3);
    CHECK(target == 190.0f && turn == 17.25f);
    command(90, ring_command_phase(RING_ENTER, 45), 1, 0, epoch, &seq);
    ring_runtime_apply(&target, &turn, wheel_revs, 3);
    CHECK(target == 90.0f && turn == 17.25f + RING_BOOST_STEP_RPM);
    target = 190.0f; turn = 17.25f;
    ring_runtime_apply(&target, &turn, wheel_revs, 3);
    CHECK(turn == 17.25f + 2 * RING_BOOST_STEP_RPM);
    target = 0.0f; turn = 0.0f;
    ring_runtime_apply(&target, &turn, wheel_revs, 3);
    CHECK(target == 0.0f && turn == 0.0f);
    command(-1, RING_COOLDOWN, 1, 0, epoch, &seq);
    target = 237.5f; turn = -53.0f;
    ring_runtime_apply(&target, &turn, wheel_revs, 2);
    CHECK(target == 237.5f && turn == -53.0f);

    /* Field failure: a detection beep precedes branch loss at a wide mouth,
     * without physical commitment. This T-shaped image has no far through
     * lane and fewer than 54 ordinary-width rows, so it cannot satisfy the
     * old cross/observed_ordinary cancellation. The bounded branch hold must
     * release to each valid original state, never latch a ring fault. */
    for (i = 1; i <= 2; ++i) {
        int j;
        restart(); startup(); island(65);
        for (j = 0; j <= RING_CONFIRM_FRAMES; ++j) frame(-23, i);
        CHECK(ring_owns_path() && published_state == 3 && !ring_result.committed);
        rg_t(image);
        for (j = 0; j < RING_CANCEL_FRAMES; ++j) {
            frame(-23, i);
            CHECK(ring_owns_path() && !ring_result.cross_veto && !ring_result.stop);
            CHECK(!ring_result.branch_ok && ring_result.path_source == 2);
        }
        for (j = 0; j < 30; ++j) {
            frame(-23, i);
            CHECK(!ring_result.stop && !ring_result.fault);
        }
        CHECK(ring_result.phase == RING_COOLDOWN && !ring_result.committed);
        CHECK(published_error == -23 && published_state == i);
        CHECK(target == 190.0f && turn == 17.25f);
        CHECK(ring_result.detected_events == 1 && !ring_result.exited_events);
        CHECK(beep_ons == 1 && beep_offs == 1);
    }

    /* The release above is NOT permission to move when the original
     * controller also reports lost line. Keep zero-speed/fault protection. */
    restart(); startup(); island(65);
    for (i = 0; i <= RING_CONFIRM_FRAMES; ++i) frame(-23, 1);
    CHECK(ring_owns_path() && !ring_result.committed);
    memset(image, 0, sizeof(image));
    for (i = 0; i < 30; ++i) frame(-23, 0);
    CHECK(ring_result.phase == RING_FAULT && ring_result.fault == 3);
    CHECK(ring_result.stop && target == 0.0f && turn == 0.0f);
    CHECK(beep_ons == 1 && beep_offs == 1 && !ring_result.exited_events);
    puts("PASS: original passthrough/startup, independent motion integration, synthetic curved-image full lap, two event beeps, cooldown, timeout and boot isolation (not a real-camera replay)");
    return 0;
}
