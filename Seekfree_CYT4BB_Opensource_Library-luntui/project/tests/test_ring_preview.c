#include "ring_vision.h"
#include "ring_runtime.h"
#include "ring_test_geometry.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL preview line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
static uint8_t image[RING_H][RING_W], saved[RING_H][RING_W];
static unsigned beeps;
static int error, state;
static float target, turn;

static void frame(uint8_t standing)
{
    int i;
    for (i = 0; i < 20; ++i)
        if (ring_runtime_tick(10, standing) == 1) ++beeps;
    target = 100.0f; turn = 17.0f;
    ring_runtime_apply(&target, &turn, 0.0f, 1);
    memcpy(saved, image, sizeof(saved));
    error = -13; state = 1;
    ring_vision_before_frame((const uint8_t (*)[RING_W])image);
    ring_vision_select(&error, &state);
    ring_vision_after_frame((const uint8_t (*)[RING_W])image);
    CHECK(memcmp(saved, image, sizeof(saved)) == 0);
    target = 100.0f; turn = 17.0f;
    ring_runtime_apply(&target, &turn, 0.0f, state);
}

int main(void)
{
    int i;
    const ring_result_t *d;
    ring_runtime_init(0.1f);
    ring_vision_init();
    CHECK(!ring_vision_binary_display() && !ring_vision_run_requested());
    rg_entry(image, 1, 0, 0);
    ring_runtime_set_operator(0, 1);
    for (i = 0; i < 400; ++i) {
        frame(0);
        CHECK(ring_vision_binary_display() && !ring_vision_run_requested());
        CHECK(ring_vision_preview());
        CHECK(ring_result.phase == RING_NORMAL && !ring_owns_path());
        CHECK(!ring_result.detected_events && !ring_result.exited_events);
        CHECK(error == -13 && state == 1 && target == 100.0f && turn == 17.0f);
    }
    d = ring_vision_display_result();
    CHECK(d->candidate_detected && d->branch_ok && d->path_valid);
    CHECK(d->path_source == 1 && d->confirm_frames == RING_CONFIRM_FRAMES);
    CHECK(beeps == 1 && !ring_runtime_watchdog_stop());
    ring_runtime_set_operator(0, 0);
    frame(0);
    CHECK(!ring_vision_binary_display() && !ring_vision_run_requested());

    /* Brief occlusion cannot repeat the same notification; leaving the
     * structure for 500ms and eight frames re-arms a new observation. */
    rg_road(image, 0);
    for (i = 0; i < 3; ++i) frame(0);
    rg_entry(image, 1, 0, 0);
    for (i = 0; i < 20; ++i) frame(0);
    CHECK(beeps == 1);
    rg_road(image, 0);
    for (i = 0; i < 30; ++i) frame(0);
    rg_entry(image, 1, 0, 0);
    for (i = 0; i < 20; ++i) frame(0);
    CHECK(beeps == 2 && !ring_result.detected_events && !ring_owns_path());

    /* Preview frames cannot count towards actual driving confirmation. */
    ring_runtime_set_operator(1, 1);
    for (i = 0; i < RING_CONFIRM_FRAMES - 1; ++i) {
        frame(1);
        CHECK(!ring_vision_preview() && ring_vision_run_requested());
        CHECK(!ring_owns_path() && !ring_result.detected_events);
        CHECK(error == -13 && state == 1);
    }
    frame(1);
    CHECK(ring_result.detected_events == 1 && ring_owns_path());
    for (i = 0; i < 12; ++i) {
        ring_runtime_set_operator(1, (uint8_t)(i & 1));
        frame(1);
        CHECK(ring_vision_binary_display() == (uint8_t)(i & 1));
        CHECK(!ring_vision_preview() && ring_result.branch_ok && !ring_result.stop);
        CHECK(state == 3 && error == ring_result.steer_error);
        CHECK(ring_vision_display_result() == &ring_result);
    }
    CHECK(beeps == 3 && ring_result.detected_events == 1 && !ring_result.exited_events);
    puts("PASS: standby recognition beeps once without control; brief dropout does not repeat; start requires fresh confirmation; display toggling preserves control");
    return 0;
}
