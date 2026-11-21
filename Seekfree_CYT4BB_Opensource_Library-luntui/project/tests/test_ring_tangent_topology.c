#include "ring_tangent_test_geometry.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL tangent-topology line %d: %s leg=%s pose=(%.2f,%.2f,%.2f) ph=%d side=%d stage=%u owns=%u left=%u br=%u cm=%u src=%u e=%d lap=%.2f origin=%.2f near=%d far=%d rej=%u hold=%u stop=%u\n", \
        __LINE__, #condition, leg, pose.x, pose.y, pose.heading_deg, ring_result.phase, ring_result.ring_side, \
        ring_result.approach_stage, ring_owns_path(), ring_result.left_entry, ring_result.branch_ok, \
        ring_result.committed, ring_result.path_source, ring_result.steer_error, ring_result.lap_deg, \
        ring_result.lap_origin_yaw, ring_result.near_row, ring_result.far_row, \
        ring_result.reject, ring_result.hold_reject, ring_result.stop); exit(1); } } while (0)

static uint8_t image[RING_H][RING_W], saved[RING_H][RING_W];
static ring_motion_t motion;
static rtg_pose_t pose, previous_pose;
static const char *leg = "reset";
static unsigned frames;
static int allow_protective_stop;
static float route_far_y;

static void image_target_world(int row, int x, float *wx, float *wy)
{
    float theta = pose.heading_deg * RTG_PI / 180.0f;
    float lateral = (float)(x - RING_W / 2);
    float forward = (float)(RING_H - 1 - row);
    *wx = pose.x + cosf(theta) * lateral + sinf(theta) * forward;
    *wy = pose.y - sinf(theta) * lateral + cosf(theta) * forward;
}

/* Independent of production inner[]/branch labels: measure a target's
 * distance to the PREDECLARED fixed-world route centreline. A white point
 * on the separate through road cannot stand in for a ring branch. */
static float branch_centre_distance(int side, float wx, float wy, int allow_shared)
{
    float best = 1.0e6f;
    int i;
    for (i = 0; i <= 200; ++i) {
        float t = (float)i / 200.0f;
        rtg_pose_t q = rtg_bezier_at(side, t, 0, route_far_y);
        float dx = wx - q.x, dy = wy - q.y, distance = sqrtf(dx * dx + dy * dy);
        if (distance < best) best = distance;
        q = rtg_bezier_at(side, t, 1, route_far_y);
        dx = wx - q.x; dy = wy - q.y; distance = sqrtf(dx * dx + dy * dy);
        if (distance < best) best = distance;
    }
    /* The outside semicircle is on the island's outer side; do not admit
     * the straight/shared side of a complete radius90 circle here. */
    if (side * wx >= 90.0f) {
        float dx = wx - side * 90.0f;
        float distance = fabsf(sqrtf(dx * dx + wy * wy) - 90.0f);
        if (distance < best) best = distance;
    }
    /* Near the end of the physical return bend, the camera legitimately
     * sees the NEXT shared-straight segment. Enable it from the independent
     * world heading, never from production phase/branch declarations. */
    if (allow_shared && wy >= -route_far_y && wy <= route_far_y) {
        float distance = fabsf(wx);
        if (distance < best) best = distance;
    }
    return best;
}

static void target_belongs_to_route(int side, int shared)
{
    static const int rows[] = {RING_LOOK_ROW, 94, RING_NEAR_ROW};
    unsigned i;
    for (i = 0; i < sizeof(rows) / sizeof(rows[0]); ++i) {
        int row = rows[i], x = ring_result.mid[row];
        float wx, wy;
        if (x < 0) continue; /* A bounded hold has no newly claimed path. */
        image_target_world(row, x, &wx, &wy);
        if (shared) CHECK(fabsf(wx) <= 8.0f);
        else CHECK(branch_centre_distance(side, wx, wy,
                    side * pose.heading_deg >= 345.0f) <= 34.0f);
    }
}

static void reset(void)
{
    ring_init();
    motion = (ring_motion_t){0, 0, 0, 1};
    pose = previous_pose = rtg_approach(0);
    allow_protective_stop = 0;
}

static void observe(unsigned interval, int track)
{
    int y;
    motion.now_ms += interval == 65535u ? 20u : interval;
    ring_observe_track(track);
    ring_observe_frame_interval((uint16_t)interval);
    memcpy(saved, image, sizeof(saved));
    ring_update((const uint8_t (*)[RING_W])image, &motion);
    ++frames;
    CHECK(!memcmp(saved, image, sizeof(saved)));
    if (!allow_protective_stop) CHECK(!ring_result.fault && !ring_result.stop);
    if (ring_result.path_source == 1 || ring_result.path_source == 4 || ring_result.path_source == 6)
        for (y = 0; y < RING_H; ++y)
            if (ring_result.mid[y] >= 0) CHECK(image[y][ring_result.mid[y]]);
}

static void world_step(int side, rtg_pose_t next)
{
    float dx = next.x - previous_pose.x, dy = next.y - previous_pose.y;
    pose = next;
    CHECK(rtg_world_white(side, pose.x, pose.y));
    CHECK(fabsf(pose.heading_deg - previous_pose.heading_deg) <= 8.0f);
    motion.wheel_revs += sqrtf(dx * dx + dy * dy) / RTG_WHEEL_CIRCUMFERENCE;
    motion.yaw_deg = pose.heading_deg;
    rtg_render(image, side, &pose);
    CHECK(image[RING_H - 1][RING_W / 2] != 0);
    observe(20, 1);
    previous_pose = pose;
}

static void still_inside_lap(void)
{
    CHECK(ring_owns_path());
    CHECK(ring_result.phase == RING_ENTER || ring_result.phase == RING_CIRCLE);
    CHECK(ring_result.exited_events == 0);
    CHECK(ring_speed_limit() >= 0);
}

static float approach_to_far(int side)
{
    int i, saw_near = 0, saw_passed = 0;
    leg = "first straight: pass near mouth, wait for far mouth";
    /* F is a predeclared feasible turn region, not a fitted test threshold.
     * The fixed world/Bezier geometry supports every F from y60 to y70.
     * Stop straight motion at the FIRST production entry in that region. */
    for (i = 0; i <= RTG_APPROACH_STEPS + 5; ++i) {
        world_step(side, rtg_approach(i));
        if (ring_result.approach_stage >= RING_TANGENT_NEAR_SEEN) saw_near = 1;
        if (ring_result.approach_stage >= RING_TANGENT_NEAR_PASSED) saw_passed = 1;
        CHECK(!ring_result.exited_events && !ring_result.left_entry);
        if (pose.y < RTG_FAR_MIN_WORLD) {
            CHECK(!ring_owns_path());
            CHECK(ring_blend_error(7) == 7 && ring_turn_boost() == 0);
            CHECK(ring_speed_limit() == -1);
        }
        if (ring_owns_path()) CHECK(saw_near && saw_passed);
        if (ring_result.phase == RING_ENTER) break;
    }
    CHECK(ring_result.phase == RING_ENTER && ring_owns_path());
    CHECK(saw_near && saw_passed);
    CHECK(ring_result.ring_side == side && ring_result.route == -side);
    CHECK(ring_result.detected_events == 1);
    CHECK(ring_result.lap_valid && fabsf(ring_result.lap_origin_yaw) < 0.01f);
    CHECK(side * ring_result.steer_error > 0);
    CHECK(pose.y >= RTG_FAR_MIN_WORLD && pose.y <= RTG_FAR_MAX_WORLD);
    return pose.y;
}

static void full_tangent_route(int side)
{
    int i, saw_circle = 0;
    float far_y;
    reset();
    far_y = approach_to_far(side);
    route_far_y = far_y;
    target_belongs_to_route(side, 0);

    leg = "far mouth: actual turn into ring";
    for (i = 1; i <= RTG_JOIN_STEPS; ++i) {
        world_step(side, rtg_join_at(side, i, far_y));
        still_inside_lap();
        target_belongs_to_route(side, 0);
        if (ring_result.phase == RING_CIRCLE) saw_circle = 1;
        CHECK(fabsf(ring_result.lap_origin_yaw) < 0.01f);
        CHECK(fabsf(ring_result.lap_deg - side * pose.heading_deg) < 0.1f);
    }
    CHECK(ring_result.left_entry && ring_result.committed);

    leg = "outside half-circle: no early exit";
    for (i = 1; i <= RTG_ARC_STEPS; ++i) {
        world_step(side, rtg_outer_arc(side, i));
        still_inside_lap();
        target_belongs_to_route(side, 0);
        if (ring_result.phase == RING_CIRCLE) saw_circle = 1;
        CHECK(ring_result.left_entry && ring_result.committed);
        CHECK(ring_result.lap_deg < 360.0f);
    }
    CHECK(saw_circle);

    leg = "near mouth on return: keep ring ownership";
    for (i = 1; i <= RTG_JOIN_STEPS; ++i) {
        world_step(side, rtg_return_near_at(side, i, far_y));
        still_inside_lap();
        target_belongs_to_route(side, 0);
        CHECK(ring_result.left_entry && ring_result.committed);
    }
    CHECK(fabsf(ring_result.lap_deg - 360.0f) < 0.1f);

    leg = "shared straight on return: 360 alone is not the far exit";
    for (i = 1; i < RTG_SHARED_STEPS; ++i) {
        world_step(side, rtg_shared_return_at(side, i, far_y));
        if (pose.y < RTG_FAR_MIN_WORLD) still_inside_lap();
        else {
            CHECK(ring_owns_path() && !ring_result.exited_events);
            CHECK(ring_result.phase != RING_NORMAL && ring_result.phase != RING_COOLDOWN);
        }
        CHECK(ring_result.left_entry && ring_result.committed);
        CHECK(fabsf(ring_result.lap_origin_yaw) < 0.01f);
        CHECK(fabsf(ring_result.lap_deg - 360.0f) < 0.1f);
        target_belongs_to_route(side, 1);
    }

    leg = "same far mouth after full circuit: take straight exit";
    world_step(side, rtg_shared_return_at(side, RTG_SHARED_STEPS, far_y));
    for (i = 1; i <= RTG_DEPART_STEPS; ++i) world_step(side, rtg_depart_at(side, i, far_y));
    CHECK(ring_result.exited_events == 1);
    CHECK(ring_result.phase == RING_COOLDOWN || ring_result.phase == RING_NORMAL);
    CHECK(!ring_owns_path() && ring_speed_limit() == -1);
    CHECK(ring_blend_error(7) == 7 && ring_turn_boost() == 0);
    CHECK(ring_result.detected_events == 1);
    printf("PASS: island side=%d, near straight -> far entry -> outer arc -> near/shared -> same far exit\n", side);
}

static void cold_ordinary_and_cross(void)
{
    int sample, i;
    leg = "ordinary/cross original output isolation";
    for (sample = 0; sample < 2; ++sample) {
        reset();
        if (sample) rtg_cross(image); else rtg_straight(image);
        for (i = 0; i < 20; ++i) {
            observe(20, sample ? 2 : 1);
            CHECK(ring_result.phase == RING_NORMAL && !ring_result.detected_events);
            CHECK(!ring_owns_path() && !ring_blocks_cross());
            CHECK(ring_blend_error(17) == 17 && ring_turn_boost() == 0);
            CHECK(ring_speed_limit() == -1);
        }
    }
}

static void straight_yaw_cannot_complete_a_ring(int side)
{
    int i;
    float far_y;
    reset();
    leg = "cold straight plus fabricated full-turn sensor input";
    rtg_straight(image);
    for (i = 0; i < 45; ++i) {
        motion.yaw_deg += side * 10.0f;
        motion.wheel_revs += 0.025f;
        observe(20, 1);
        CHECK(ring_result.phase == RING_NORMAL && !ring_result.detected_events);
        CHECK(!ring_result.left_entry && !ring_result.exited_events);
        CHECK(!ring_owns_path() && ring_speed_limit() == -1);
    }

    reset();
    leg = "far mouth recognised, but no actual branch departure";
    (void)approach_to_far(side);
    CHECK(ring_result.phase == RING_ENTER && !ring_result.left_entry);
    rtg_straight(image);
    allow_protective_stop = 1;
    for (i = 0; i < 45; ++i) {
        motion.yaw_deg += side * 10.0f;
        motion.wheel_revs += 0.025f;
        observe(20, 1);
        CHECK(!ring_result.left_entry && !ring_result.committed);
        CHECK(ring_result.phase != RING_CIRCLE && ring_result.phase != RING_EXIT);
        CHECK(!ring_result.exited_events);
    }

    reset();
    leg = "real departure then lost branch: release without successful exit";
    far_y = approach_to_far(side);
    for (i = 1; i <= RTG_JOIN_STEPS; ++i) world_step(side, rtg_join_at(side, i, far_y));
    CHECK(ring_result.left_entry && ring_result.committed);
    rtg_straight(image);
    allow_protective_stop = 1;
    for (i = 0; i < 45; ++i) {
        motion.yaw_deg += side * 10.0f;
        motion.wheel_revs += 0.025f;
        observe(20, 1);
        CHECK(!ring_result.exited_events);
        CHECK(ring_result.phase != RING_EXIT && ring_result.phase != RING_RECOVER);
        CHECK(!ring_result.stop && !ring_result.fault);
        if(!ring_owns_path()) {
            CHECK(ring_result.phase==RING_COOLDOWN || ring_result.phase==RING_NORMAL);
            CHECK(ring_speed_limit()==-1 && ring_blend_error(17)==17);
        }
    }
    CHECK(!ring_owns_path() && !ring_result.exited_events);
}

static void far_mouth_without_near_history(int side)
{
    int i;
    reset();
    leg = "stationary far mouth without a preceding near-mouth traversal";
    pose = previous_pose = rtg_approach(RTG_APPROACH_STEPS);
    rtg_render(image, side, &pose);
    for (i = 0; i < 20; ++i) {
        observe(20, 1);
        CHECK(!ring_owns_path() && !ring_result.left_entry);
        CHECK(ring_result.phase != RING_ENTER && ring_result.phase != RING_CIRCLE);
        CHECK(!ring_result.exited_events);
        CHECK(ring_blend_error(17) == 17 && ring_turn_boost() == 0);
    }
}

static void unknown_visual_interval_cannot_advance_entry(int side)
{
    int i;
    float far_y;
    reset();
    far_y = approach_to_far(side);
    leg = "unknown visual intervals cannot authorise actual entry";
    allow_protective_stop = 1;
    for (i = 1; i <= RTG_JOIN_STEPS; ++i) {
        rtg_pose_t next = rtg_join_at(side, i, far_y);
        float dx = next.x - previous_pose.x, dy = next.y - previous_pose.y;
        pose = next;
        motion.wheel_revs += sqrtf(dx * dx + dy * dy) / RTG_WHEEL_CIRCUMFERENCE;
        motion.yaw_deg = pose.heading_deg;
        rtg_render(image, side, &pose);
        observe(65535u, 1);
        previous_pose = pose;
        CHECK(!ring_result.left_entry && !ring_result.committed);
        CHECK(ring_result.phase != RING_CIRCLE && ring_result.phase != RING_EXIT);
        CHECK(!ring_result.exited_events);
        CHECK(!(ring_result.entry_flags & RING_ENTRY_HISTORY_FRESH));
    }
}

static void check_reference_pose(int side, rtg_pose_t next)
{
    float dx = next.x - previous_pose.x, dy = next.y - previous_pose.y;
    pose = next;
    CHECK(rtg_world_white(side, pose.x, pose.y));
    CHECK(sqrtf(dx * dx + dy * dy) <= 4.0f);
    CHECK(fabsf(pose.heading_deg - previous_pose.heading_deg) <= 8.0f);
    previous_pose = pose;
}

static void validate_independent_entry_region(void)
{
    int side, i, far;
    leg = "independent geometry feasibility of F in y60..70";
    for (side = -1; side <= 1; side += 2)
        for (far = (int)RTG_FAR_MIN_WORLD; far <= (int)RTG_FAR_MAX_WORLD; far += 2) {
            previous_pose = (rtg_pose_t){0.0f, (float)far, 0.0f};
            for (i = 1; i <= RTG_JOIN_STEPS; ++i) check_reference_pose(side, rtg_join_at(side, i, (float)far));
            for (i = 1; i <= RTG_ARC_STEPS; ++i) check_reference_pose(side, rtg_outer_arc(side, i));
            for (i = 1; i <= RTG_JOIN_STEPS; ++i) check_reference_pose(side, rtg_return_near_at(side, i, (float)far));
            for (i = 1; i <= RTG_SHARED_STEPS; ++i) check_reference_pose(side, rtg_shared_return_at(side, i, (float)far));
            for (i = 1; i <= RTG_DEPART_STEPS; ++i) check_reference_pose(side, rtg_depart_at(side, i, (float)far));
        }
    puts("PASS: independently fixed F region y60..70 has mirrored continuous white-road paths at both bounds and every 2-unit sample");
}

int main(void)
{
    CHECK(RING_TRACK_MODE == 1);
    validate_independent_entry_region();
    cold_ordinary_and_cross();
    far_mouth_without_near_history(1);
    far_mouth_without_near_history(-1);
    full_tangent_route(1);
    full_tangent_route(-1);
    straight_yaw_cannot_complete_a_ring(1);
    straight_yaw_cannot_complete_a_ring(-1);
    unknown_visual_interval_cannot_advance_entry(1);
    unknown_visual_interval_cannot_advance_entry(-1);
    printf("PASS: %u moving-camera frames over a shared-straight/world-annulus map (synthetic, not a field replay)\n", frames);
    return 0;
}
