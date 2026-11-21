#include "ring_test_geometry.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "FAIL slow-progress line %d: %s ph=%d dir=%d tip=%d mode=%u branch=%u cm=%u flags=%u hold=%u decision=%u\n", \
        __LINE__, #x, ring_result.phase, ring_result.route, ring_result.island_near, \
        ring_result.branch_mode, ring_result.branch_ok, ring_result.committed, \
        ring_result.entry_flags, ring_result.hold_reject, ring_result.decision); exit(1); } } while (0)

static uint8_t image[RING_H][RING_W], saved[RING_H][RING_W];
static ring_motion_t motion;
static unsigned frames;

static void step(unsigned interval)
{
    int y;
    motion.now_ms += interval == 65535u ? 34u : interval;
    ring_observe_track(1);
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

static void begin(int route, int advance)
{
    int i;
    ring_init();
    motion = (ring_motion_t){0, 0, 0, 1};
    rg_entry(image, route, advance, 0);
    for (i = 0; i < RING_CONFIRM_FRAMES; ++i) step(34);
    CHECK(ring_result.phase == RING_APPROACH || ring_result.phase == RING_ENTER);
    CHECK(ring_result.branch_ok && ring_result.branch_mode == RING_BRANCH_STRONG);
    CHECK(!ring_result.committed && ring_result.detected_events == 1);
}

static void measured_entry(int route, int advance)
{
    rg_entry(image, route, advance, 0);
    step(34);
    /* Assistance describes control ownership; source/MEASURED continue to
     * prove that this frame's progress came from actual strong pixels. */
    CHECK(ring_result.branch_ok && ring_result.path_source == 1);
    CHECK(ring_result.entry_flags & RING_ENTRY_MEASURED);
    CHECK(ring_result.branch_mode == RING_BRANCH_STRONG ||
          ring_result.branch_mode == RING_BRANCH_ASSIST);
}

/* 0/1-row pixel changes model image quantisation. The gyro fixture uses the
 * actual raw-Z convention (-route), independently of RING_YAW_SIGN. These
 * are pixel/evidence regressions; vehicle dynamics is not simulated. */
static void slow_entry(int route, int cadence)
{
    int advance, repeated;
    begin(route, -17);
    for (advance = -16; advance <= 34; ++advance) {
        for (repeated = 0; repeated < cadence; ++repeated) {
            motion.yaw_deg -= route * 2.0f;
            motion.wheel_revs += 0.025f;
            measured_entry(route, advance);
            CHECK(!ring_result.stop);
        }
        if (advance < 26) CHECK(!ring_result.committed);
        if (advance >= 26) {
            CHECK(ring_result.entry_flags & RING_ENTRY_PROGRESSED);
            CHECK(ring_result.committed);
        }
    }
    CHECK(ring_result.phase == RING_CIRCLE);
    CHECK(ring_result.island_near == 104);
    CHECK(ring_result.detected_events == 1 && !ring_result.exited_events);
}

static void no_oscillation_credit(int route)
{
    int pass, advance;
    begin(route, 25); /* Current tip=95; a seven-row excursion never reaches +8. */
    for (pass = 0; pass < 5; ++pass) {
        for (advance = 26; advance <= 32; ++advance) {
            motion.yaw_deg -= route * 2.0f;
            motion.wheel_revs += 0.025f;
            measured_entry(route, advance);
            CHECK(!(ring_result.entry_flags & RING_ENTRY_PROGRESSED));
            CHECK(!ring_result.committed && ring_result.phase != RING_CIRCLE);
        }
        for (advance = 31; advance >= 25; --advance) {
            motion.yaw_deg -= route * 2.0f;
            motion.wheel_revs += 0.025f;
            measured_entry(route, advance);
            CHECK(!(ring_result.entry_flags & RING_ENTRY_PROGRESSED));
            CHECK(!ring_result.committed && ring_result.phase != RING_CIRCLE);
        }
    }
}

static void reset_history(int route)
{
    int advance, gap;
    begin(route, 0);
    for (advance = 2; advance <= 10; advance += 2) measured_entry(route, advance);
    CHECK(ring_result.entry_flags & RING_ENTRY_PROGRESSED);
    measured_entry(route, 0); /* A real current landmark retreats ten rows. */
    CHECK(!(ring_result.entry_flags & RING_ENTRY_PROGRESSED));
    CHECK(!(ring_result.entry_flags & RING_ENTRY_HISTORY));
    measured_entry(route, 2);
    measured_entry(route, 4);
    CHECK(!(ring_result.entry_flags & RING_ENTRY_PROGRESSED));

    for (gap = 0; gap < 2; ++gap) {
        begin(route, 0);
        for (advance = 2; advance <= 10; advance += 2) measured_entry(route, advance);
        CHECK(ring_result.entry_flags & RING_ENTRY_PROGRESSED);
        step(gap ? 65535u : RING_PATH_HOLD_MS + 1u);
        CHECK(!(ring_result.entry_flags & RING_ENTRY_PROGRESSED));
        CHECK(!(ring_result.entry_flags & RING_ENTRY_HISTORY));
        motion.yaw_deg = -route * 45.0f;
        motion.wheel_revs = 0.3f;
        step(34);
        CHECK(!ring_result.committed);
        CHECK(!(ring_result.entry_flags & RING_ENTRY_PROGRESSED));
    }
}

static void gradual_retreat_rebuilds_history(int route)
{
    int advance;
    begin(route, 0);
    for (advance = 2; advance <= 12; advance += 2) measured_entry(route, advance);
    CHECK(ring_result.entry_flags & RING_ENTRY_PROGRESSED);
    for (advance = 11; advance >= 0; --advance) {
        measured_entry(route, advance);
        if (advance == 3) {
            /* Nine rows below the earlier peak, reached one row at a time.
             * Check the actual history flag: loss of NET progress alone
             * would not prove that obsolete observations were discarded. */
            CHECK(!(ring_result.entry_flags & RING_ENTRY_HISTORY));
            CHECK(!(ring_result.entry_flags & RING_ENTRY_PROGRESSED));
        }
        if (advance <= 3) CHECK(!(ring_result.entry_flags & RING_ENTRY_PROGRESSED));
        CHECK(!ring_result.committed);
    }
    for (advance = 1; advance <= 10; ++advance) {
        measured_entry(route, advance);
        CHECK(!(ring_result.entry_flags & RING_ENTRY_PROGRESSED));
    }
    measured_entry(route, 11);
    CHECK(ring_result.entry_flags & RING_ENTRY_PROGRESSED);
    CHECK(!ring_result.committed);
}

static void fixed_image_and_disconnection(int route)
{
    int i;
    begin(route, 26);
    for (i = 0; i < 20; ++i) {
        motion.yaw_deg -= route * 3.0f;
        motion.wheel_revs += 0.025f;
        step(34);
        CHECK(ring_result.branch_ok && !ring_result.committed);
        CHECK(!(ring_result.entry_flags & RING_ENTRY_PROGRESSED));
        CHECK(ring_result.phase != RING_CIRCLE);
    }
    begin(route, 0);
    for (i = 2; i <= 10; i += 2) measured_entry(route, i);
    CHECK(ring_result.entry_flags & RING_ENTRY_PROGRESSED);
    memset(image, 0, sizeof(image));
    for (i = 0; i <= (int)RING_HOLD_FRAMES; ++i) step(34);
    CHECK(ring_result.phase == RING_COOLDOWN && !ring_result.committed);
    CHECK(!ring_result.stop && !ring_result.branch_ok);
    /* An expired disconnected branch cannot reacquire via yaw alone. */
    rg_entry(image, route, 26, 0);
    motion.yaw_deg = -route * 45.0f;
    motion.wheel_revs += 0.3f;
    step(34);
    CHECK(ring_result.phase == RING_COOLDOWN && !ring_result.committed);
}

static void partial_pixels(int route)
{
    int x, y;
    for (y = 24; y <= 117; ++y)
        if (!((y >= 70 && y <= 86) || (y >= 100 && y <= 117))) {
            if (route > 0) for (x = 0; x < 94; ++x) image[y][x] = 255;
            else for (x = 94; x < RING_W; ++x) image[y][x] = 255;
        }
}

static void weak_has_no_entry_credit(int route)
{
    int i;
    begin(route, 0);
    for (i = 1; i <= 9; ++i) {
        rg_transition(image, route, i, 12);
        motion.yaw_deg = -route * (12.0f * i / 9.0f);
        motion.wheel_revs += 0.025f;
        step(34);
        CHECK(ring_result.branch_ok && !ring_result.committed);
    }
    CHECK(ring_result.entry_flags & RING_ENTRY_PROGRESSED);
    partial_pixels(route);
    motion.yaw_deg = -route * 22.0f; /* Ten degrees from the last STRONG edge. */
    step(34);
    CHECK(ring_result.branch_mode == RING_BRANCH_PARTIAL && ring_result.branch_ok);
    CHECK(!(ring_result.entry_flags & RING_ENTRY_MEASURED));
    CHECK(!(ring_result.entry_flags & RING_ENTRY_HISTORY_FRESH));
    CHECK(!ring_result.committed && ring_result.phase == RING_ENTER);
}

int main(int argc, char **argv)
{
    int route, cadence;
    int guards_only = argc > 1 && !strcmp(argv[1], "guards");
    for (route = -1; route <= 1; route += 2) {
        if (!guards_only)
            for (cadence = 1; cadence <= 3; ++cadence) slow_entry(route, cadence);
        no_oscillation_credit(route);
        reset_history(route);
        gradual_retreat_rebuilds_history(route);
        fixed_image_and_disconnection(route);
        weak_has_no_entry_credit(route);
    }
    if (!guards_only)
        puts("PASS: mirrored 0/1-row and intermittent progress counts accumulated measured displacement");
    puts("PASS: oscillation, retreat, expired/unknown timing, disconnected branch and fixed-image yaw cannot create entry");
    puts("PASS: partial current pixels supply no new strong entry credit");
    printf("PASS: %u slow-entry geometry frames (synthetic, not field replay)\n", frames);
    return 0;
}
