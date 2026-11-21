#include "ring_test_geometry.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "FAIL assist-safety line %d: %s ph=%d dir=%d e=%d br=%u src=%u mode=%u flags=%u cm=%u rej=%u stop=%u\n", \
        __LINE__, #x, ring_result.phase, ring_result.route, ring_result.steer_error, \
        ring_result.branch_ok, ring_result.path_source, ring_result.branch_mode, \
        ring_result.entry_flags, ring_result.committed, ring_result.reject, ring_result.stop); exit(1); } } while (0)

static uint8_t image[RING_H][RING_W], saved[RING_H][RING_W];
static ring_motion_t motion;
static unsigned frames;

static int assisting(void)
{
    return (ring_result.entry_flags & RING_ENTRY_ASSIST) != 0;
}

static void step(unsigned interval, int track)
{
    int y;
    motion.now_ms += interval == 65535u ? 20u : interval;
    ring_observe_track(track);
    ring_observe_frame_interval((uint16_t)interval);
    memcpy(saved, image, sizeof(saved));
    ring_update((const uint8_t (*)[RING_W])image, &motion);
    ++frames;
    CHECK(!memcmp(saved, image, sizeof(saved)));
    CHECK(!ring_result.fault);
    if (ring_result.path_source == 1)
        for (y = 0; y < RING_H; ++y)
            if (ring_result.mid[y] >= 0) CHECK(image[y][ring_result.mid[y]]);
}

static void reset(void)
{
    ring_init();
    motion = (ring_motion_t){0, 0, 0, 1};
}

static void begin(int route)
{
    int i;
    reset();
    rg_entry(image, route, 6, 0);
    for (i = 0; i < RING_CONFIRM_FRAMES + RING_ASSIST_CONFIRM + 1; ++i) step(20, 1);
    CHECK(ring_result.phase == RING_ENTER && ring_result.detected_events == 1);
    CHECK(route * ring_result.steer_error >= RING_ASSIST_MIN_ERROR);
    CHECK(assisting() && ring_result.branch_ok && ring_result.path_source == 1);
    CHECK(ring_result.branch_mode == RING_BRANCH_ASSIST);
    CHECK(!ring_result.committed && !ring_result.stop);
}

static void no_cold_takeover(void)
{
    int sample, i, route;
    for (sample = 0; sample < 3; ++sample) {
        reset();
        if (sample == 0) rg_road(image, 0);
        else if (sample == 1) rg_cross(image, 0);
        else rg_t(image);
        for (i = 0; i < 20; ++i) {
            step(20, sample == 1 ? 2 : 1);
            CHECK(!assisting() && !ring_result.detected_events);
            CHECK(ring_result.phase == RING_NORMAL && !ring_owns_path());
            CHECK(ring_blend_error(17) == 17 && ring_turn_boost() == 0);
            CHECK(ring_speed_limit() == -1);
        }
    }
    for (route = -1; route <= 1; route += 2) {
        reset();
        rg_entry(image, route, 6, 0);
        for (i = 0; i < RING_CONFIRM_FRAMES - 1; ++i) {
            step(20, 1);
            CHECK(!assisting() && !ring_result.detected_events);
            CHECK(ring_blend_error(17) == 17 && ring_turn_boost() == 0);
        }
    }
}

static void fixed_geometry_has_no_credit(int route)
{
    int i;
    begin(route);
    for (i = 0; i < 5; ++i) {
        /* Plenty of motion, but the actual inner edge has not advanced. */
        motion.yaw_deg -= route * 4.0f;
        motion.wheel_revs += 0.04f;
        step(20, 1);
        CHECK(!ring_result.committed && ring_result.phase != RING_CIRCLE);
        CHECK(!(ring_result.entry_flags & RING_ENTRY_PROGRESSED));
        CHECK(ring_result.exited_events == 0);
    }
    CHECK(assisting() && ring_result.branch_ok);
}

static void strong_evidence_cancels_assist(int route)
{
    int i, seen_reverse = 0;
    begin(route);
    for (i = 1; i <= 12; ++i) {
        rg_transition(image, route, i, 12);
        step(20, 1);
        if (ring_result.branch_ok && route * ring_result.steer_error <= 0) {
            seen_reverse = 1;
            CHECK(!assisting());
        }
    }
    rg_orbit(image, route, 90);
    for (i = 0; i < 3; ++i) {
        step(20, 1);
        CHECK(ring_result.branch_ok);
        CHECK(!assisting() && ring_result.path_source == 1);
        CHECK(route * ring_result.steer_error <= 0);
    }
    CHECK(seen_reverse);
    /* Returning to the old mouth cannot arm a second force episode. */
    rg_entry(image, route, 6, 0);
    for (i = 0; i < RING_ASSIST_CONFIRM + 2; ++i) {
        step(20, 1);
        CHECK(!assisting());
    }
}

static void ordinary_cross_black_stop_force(int route)
{
    int sample, i;
    for (sample = 0; sample < 3; ++sample) {
        int raw_visual_error;
        begin(route);
        raw_visual_error = (ring_result.near_x - RING_W / 2 - 1) +
            (ring_result.guide_x - ring_result.near_x) * 20 /
            (RING_NEAR_ROW - ring_result.guide_y);
        if (sample == 0) rg_road(image, 0);
        else if (sample == 1) rg_cross(image, 0);
        else memset(image, 0, sizeof(image));
        step(20, sample == 1 ? 2 : 1);
        CHECK(!assisting());
        CHECK(ring_result.path_source != 5);
        CHECK(!ring_result.branch_ok && !ring_result.committed);
        CHECK(ring_result.phase != RING_CIRCLE && !ring_result.exited_events);
        if (sample == 2) {
            CHECK(ring_result.path_source == 2);
            CHECK(ring_result.steer_error == raw_visual_error);
            CHECK(abs(ring_result.steer_error) < RING_ASSIST_MIN_ERROR);
        }
        for (i = 0; i < 2; ++i) {
            step(20, sample == 1 ? 2 : 1);
            CHECK(!assisting() && ring_result.path_source != 5);
        }
    }
}

static void one_episode_has_finite_budget(int route)
{
    int cause, i;
    for (cause = 0; cause < 4; ++cause) {
        begin(route);
        if (cause == 0) {
            for (i = 0; i < (int)(RING_ASSIST_TOTAL_MS / 20u) + 2; ++i) step(20, 1);
        } else if (cause == 1) {
            motion.wheel_revs += RING_ASSIST_ENTRY_REVS + 0.01f;
            step(20, 1);
        } else if (cause == 2) {
            motion.yaw_deg += route * (RING_ASSIST_ENTRY_YAW_DEG + 0.1f);
            step(20, 1);
        } else step(65535u, 1);
        CHECK(!assisting());
        CHECK(!ring_result.committed && ring_result.phase != RING_CIRCLE);
        CHECK(!ring_result.exited_events);
        for (i = 0; i < RING_ASSIST_CONFIRM + 2; ++i) {
            step(20, 1);
            CHECK(!assisting());
        }
    }
}

static void missing_branch_is_not_measured(int route)
{
    int advance, i;
    begin(route);
    for (advance = 7; advance <= 26; ++advance) {
        rg_entry(image, route, advance, 0);
        step(20, 1);
        CHECK(ring_result.branch_ok && !ring_result.committed);
    }
    CHECK(ring_result.entry_flags & RING_ENTRY_PROGRESSED);
    CHECK(ring_result.entry_flags & RING_ENTRY_NEAR);
    CHECK(assisting());
    /* The prior measured path is still on connected white pavement, but
     * every island-edge pixel is gone. This is control permission only. */
    memset(image, 255, sizeof(image));
    motion.yaw_deg = -route * 22.0f;
    motion.wheel_revs = 0.2f;
    step(20, 1);
    CHECK(assisting() && ring_result.path_source == 5);
    CHECK(!ring_result.branch_ok && !ring_result.committed);
    CHECK(!(ring_result.entry_flags & RING_ENTRY_MEASURED));
    CHECK(!(ring_result.entry_flags & RING_ENTRY_HISTORY_FRESH));
    CHECK(ring_result.phase == RING_ENTER && !ring_result.exited_events);
    CHECK(route * ring_result.steer_error >= RING_ASSIST_MIN_ERROR);
    for (i = 0; i <= (int)RING_HOLD_FRAMES; ++i) {
        step(20, 1);
        CHECK(!ring_result.branch_ok && !ring_result.committed);
        CHECK(ring_result.phase != RING_CIRCLE && !ring_result.exited_events);
    }
    CHECK(!assisting());
}

static void missing_budget_is_finite(int route)
{
    int cause, i;
    for (cause = 0; cause < 5; ++cause) {
        begin(route);
        memset(image, 255, sizeof(image));
        if (cause == 0) step(RING_ASSIST_HOLD_MS + 1u, 1);
        else if (cause == 1) {
            motion.yaw_deg += RING_ASSIST_HOLD_YAW_DEG + 0.1f;
            step(20, 1);
        } else if (cause == 2) {
            motion.wheel_revs += RING_ASSIST_HOLD_REVS + 0.01f;
            step(20, 1);
        } else if (cause == 3) step(65535u, 1);
        else for (i = 0; i <= (int)RING_HOLD_FRAMES; ++i) step(20, 1);
        CHECK(!assisting() && ring_result.path_source != 5);
        CHECK(ring_result.entry_flags & RING_ENTRY_ASSIST_SPENT);
        CHECK(!ring_result.branch_ok && !ring_result.committed);
        CHECK(ring_result.phase != RING_CIRCLE && !ring_result.exited_events);
    }
}

static void oscillation_spends_absolute_budget(int route)
{
    int i;
    begin(route);
    for (i = 0; i < 4; ++i) {
        motion.yaw_deg = (i & 1) ? 0.0f : 15.0f;
        step(20, 1);
    }
    CHECK(motion.yaw_deg == 0.0f && !assisting());
    CHECK(!ring_result.committed);
    begin(route);
    for (i = 0; i < 4; ++i) {
        motion.wheel_revs = (i & 1) ? 0.0f : 0.21f;
        step(20, 1);
    }
    CHECK(motion.wheel_revs == 0.0f && !assisting());
    CHECK(!ring_result.committed && !ring_result.exited_events);
}

static void reboot_and_not_ready(int route)
{
    begin(route);
    motion.ready = 0;
    step(20, 1);
    CHECK(!assisting() && !ring_result.branch_ok);
    reset();
    CHECK(!ring_owns_path() && !assisting());
    CHECK(!ring_result.detected_events && !ring_result.exited_events);
    CHECK(ring_blend_error(17) == 17 && ring_turn_boost() == 0);
    rg_road(image, 0);
    step(20, 1);
    CHECK(!assisting() && ring_result.phase == RING_NORMAL);
}

int main(void)
{
    int route;
    no_cold_takeover();
    for (route = -1; route <= 1; route += 2) {
        fixed_geometry_has_no_credit(route);
        strong_evidence_cancels_assist(route);
        ordinary_cross_black_stop_force(route);
        one_episode_has_finite_budget(route);
        missing_branch_is_not_measured(route);
        missing_budget_is_finite(route);
        oscillation_spends_absolute_budget(route);
        reboot_and_not_ready(route);
    }
    puts("PASS: ordinary/cross/candidate passthrough; mirrored confirmed-only entry assist; real reverse bend releases force");
    puts("PASS: force cannot create branch entry/CIRCLE/exit; time/travel/yaw/unknown/reboot limits end the episode");
    printf("PASS: %u entry-assist safety frames (synthetic geometry, not field replay)\n", frames);
    return 0;
}
