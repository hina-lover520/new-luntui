#include "ring_test_geometry.h"
#include <stdio.h>
#include <stdlib.h>

/* Synthetic image geometry checks logic; it is not a real-car replay. */
#define CHECK(c) do { if (!(c)) { \
    fprintf(stderr, "FAIL ring line %d: %s phase=%d route=%d source=%u branch=%u reject=%u\n", \
        __LINE__, #c, ring_result.phase, ring_result.route, ring_result.path_source, \
        ring_result.branch_ok, ring_result.reject); exit(1); } } while (0)
static uint8_t image[RING_H][RING_W], saved[RING_H][RING_W];
static ring_motion_t motion;
static unsigned checks;
static int original_state;

static void reset(void)
{
    ring_init(); motion = (ring_motion_t){0, 0, 0, 1};original_state=1;
}
static void step(unsigned ms)
{
    int y;
    ring_motion_t sensor;
    motion.now_ms += ms;
    ring_observe_track(original_state);
    memcpy(saved, image, sizeof(image));
    /* Fixtures express net route progress. The board feeds raw Z-integral
     * heading; the independent yaw-convention test checks its real sign. */
    sensor=motion;sensor.yaw_deg=motion.yaw_deg*RING_YAW_SIGN;
    ring_update((const uint8_t (*)[RING_W])image, &sensor);
    CHECK(!memcmp(saved, image, sizeof(image)));
    for (y = 0; y < RING_H; ++y) {
        CHECK(ring_result.mid[y] >= -1 && ring_result.mid[y] < RING_W);
        if ((ring_result.path_source == 1 || ring_result.path_source == 4) &&
            ring_result.path_valid && ring_result.mid[y] >= 0)
            CHECK(image[y][ring_result.mid[y]] != 0);
    }
    CHECK(ring_result.steer_error >= -90 && ring_result.steer_error <= 90);
    ++checks;
}
static void entry_image(int center, int advance)
{
    rg_annulus(image, center, 45 + advance, 26, 86, 1, 0);
}
static void confirm(int center)
{
    int i;
    uint32_t events = ring_result.detected_events;
    entry_image(center, 0);
    for (i = 0; i < RING_CONFIRM_FRAMES; ++i) step(20);
    if (ring_result.phase != RING_APPROACH) fprintf(stderr,"confirm center=%d events=%u actual=%u\n",center,(unsigned)events,(unsigned)ring_result.detected_events);
    CHECK(ring_result.phase == RING_APPROACH);
    CHECK(ring_result.detected_events == events + 1);
    CHECK(ring_result.branch_ok && ring_result.path_source == 1);
    CHECK(ring_speed_limit() == RING_APPROACH_RPM);
}
static void enter(int center)
{
    int i;
    static const int advance[] = {3, 6, 9, 13};
    confirm(center);
    for (i = 0; i < 4; ++i) { entry_image(center, advance[i]); step(20); }
    CHECK(ring_result.phase == RING_ENTER && ring_result.path_valid);
    CHECK(ring_result.branch_ok && ring_result.path_source == 1 && !ring_result.stop);
}
static void circle(int center)
{
    int i, route;
    enter(center);
    route = ring_result.route;
    /* Actual curved inner-boundary pixels move from entry to a tangent view.
     * Heading is independent sensor input, never the sole circle evidence. */
    for (i = 1; i <= 12; ++i) {
        rg_transition_from(image, route, center, i, 12);
        motion.yaw_deg = route * 4.0f * i;
        motion.wheel_revs += 0.025f;
        step(20);
        CHECK(ring_result.branch_ok && ring_result.path_source == 1);
    }
    rg_orbit(image, route, 90);
    for (i = 0; i < 6; ++i) {
        motion.yaw_deg += route * 2.0f; motion.wheel_revs += 0.03f; step(20);
    }
    CHECK(ring_result.phase == RING_CIRCLE && ring_result.committed);
}
static void full_lap(int center)
{
    int i, route, old_look;
    reset(); circle(center); route = ring_result.route;
    CHECK(route == (RING_ROUTE ? RING_ROUTE : (center <= RING_W / 2 ? 1 : -1)));
    rg_exit(image, route); motion.wheel_revs = 1.0f;
    for (i = 0; i < 8; ++i) {
        motion.yaw_deg = route * 180.0f; step(20);
        CHECK(ring_result.phase == RING_CIRCLE);
    }
    for (i = 0; i < 20; ++i) {
        motion.yaw_deg = route * (i % 2 ? 160.0f : 220.0f); step(20);
        CHECK(ring_result.phase == RING_CIRCLE);
    }
    motion.yaw_deg = route * 370.0f;
    for (i = 0; i < RING_EXIT_CONFIRM_FRAMES; ++i) step(20);
    CHECK(ring_result.phase == RING_EXIT && !ring_result.exited_events);
    old_look = ring_result.look_x; step(20);
    /* Near tangent may quantize to the same look pixel as the final arc. */
    CHECK(ring_result.path_source == 4 && route * (ring_result.look_x - old_look) >= 0);
    rg_road(image, 0);
    for (i = 0; i < RING_RECOVER_FRAMES * 2; ++i) step(20);
    CHECK(ring_result.phase == RING_COOLDOWN && ring_result.exited_events == 1);
    CHECK(ring_result.detected_events == 1 && !ring_blocks_cross());
    entry_image(center, 13);
    for (i = 0; i < 10; ++i) step(200);
    CHECK(ring_result.phase == RING_COOLDOWN);
    rg_road(image, 0); motion.wheel_revs += RING_COOLDOWN_REVS + 0.1f; step(20);
    CHECK(ring_result.phase == RING_NORMAL);
    confirm(center); CHECK(ring_result.detected_events == 2);
}
int main(void)
{
    int i, x, y;
    reset(); rg_road(image, 0);
    for (i = 0; i < 60; ++i) step(20);
    CHECK(ring_result.phase == RING_NORMAL && !ring_blocks_cross());
    CHECK(ring_speed_limit() == -1);
    memset(image, 0, sizeof(image));
    rg_rect(image, 24, 0, 70, 119, 255); rg_rect(image, 115, 0, 160, 119, 255);
    for (i = 0; i < 10; ++i) step(20);
    CHECK(ring_result.phase == RING_NORMAL);
    rg_road(image, 0); rg_rect(image, 24, 0, 163, 101, 255); rg_rect(image, 70, 0, 110, 82, 0);
    for (i = 0; i < 10; ++i) step(20);
    CHECK(ring_result.phase == RING_NORMAL);
    rg_cross(image, 0); for (i = 0; i < 10; ++i) step(20);
    CHECK(ring_result.phase == RING_NORMAL);
    rg_road(image, 0); rg_rect(image, 88, 50, 98, 52, 0);
    for (i = 0; i < 10; ++i) step(20);
    CHECK(ring_result.phase == RING_NORMAL);
    motion.ready = 0; entry_image(80, 0);
    for (i = 0; i < 10; ++i) step(20);
    CHECK(ring_result.phase == RING_NORMAL);
    motion.ready = 1;
    for (i = 0; i < RING_CONFIRM_FRAMES - 1; ++i) step(20);
    CHECK(!ring_result.detected_events && ring_speed_limit() == -1 && !ring_blocks_cross());
    rg_road(image, 0); step(20);
    CHECK(!ring_result.detected_events && !ring_owns_path());
    full_lap(80); full_lap(108); full_lap(RING_W / 2);

    reset(); circle(80); memset(image, 0, sizeof(image)); step(40);
    CHECK(!ring_result.stop && ring_result.path_source == 2 && !ring_result.branch_ok);
    step(RING_PATH_HOLD_MS);
    CHECK(ring_result.stop && ring_speed_limit() == 0);
    rg_orbit(image, ring_result.route, 90); step(20);
    CHECK(!ring_result.stop && ring_result.branch_ok && ring_result.path_source == 1);
    memset(image, 0, sizeof(image)); step(RING_PATH_TIMEOUT_MS + 1);
    CHECK(ring_result.phase == RING_FAULT && ring_result.fault == 3);
    rg_road(image, 0); step(20); CHECK(ring_result.phase == RING_FAULT);
    reset(); confirm(80); step(RING_APPROACH_TIMEOUT_MS + 1);
    CHECK(ring_result.phase == RING_COOLDOWN && !ring_result.stop && !ring_result.fault);
    CHECK(ring_speed_limit() == -1 && !ring_result.exited_events);
    reset(); confirm(80); original_state=0; step(RING_APPROACH_TIMEOUT_MS + 1);
    CHECK(ring_result.phase == RING_FAULT && ring_result.fault == 2);
    reset(); confirm(80); original_state=0; memset(image,0,sizeof(image));
    step(RING_PATH_HOLD_MS + 1);
    CHECK(!ring_result.committed && ring_result.stop && ring_speed_limit() == 0);
    step(RING_PATH_TIMEOUT_MS + 1);
    CHECK(ring_result.phase == RING_FAULT && ring_result.fault == 3);
    reset(); circle(80); step(RING_TOTAL_TIMEOUT_MS + 1);
    CHECK(ring_result.phase == RING_FAULT && ring_result.fault == 1);
    reset();
    for (i = 0; i < 40; ++i) {
        for (y = 0; y < RING_H; ++y)
            for (x = 0; x < RING_W; ++x)
                image[y][x] = ((x * 31 + y * 17 + i * 13) % 23 < 11) ? 255 : 0;
        step(20);
    }
    memset(image, 255, sizeof(image)); step(20);
    memset(image, 0, sizeof(image)); step(20);
    CHECK(ring_result.phase == RING_NORMAL);
    printf("PASS: %u synthetic curved-geometry frame checks, route=%d, full lap + exit + events + faults\n", checks, RING_ROUTE);
    return 0;
}
