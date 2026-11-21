#ifndef RING_TANGENT_TEST_GEOMETRY_H
#define RING_TANGENT_TEST_GEOMETRY_H

#include "ring.h"
#include <math.h>
#include <string.h>

/* A synthetic, fixed WORLD map matching the photographs' topology:
 * one straight road shares the left part of a circular ring. The island
 * never blocks the incoming road. Units are arbitrary world pixels, not
 * an assertion about the real camera calibration or wheel circumference.
 * The image is produced solely by a moving/rotating camera over this map.
 * +side places the island on the car's right; -side mirrors the whole map.
 * Raw heading is clockwise-positive, matching the existing vehicle IMU. */
#define RTG_PI 3.14159265358979323846f
#define RTG_WHEEL_CIRCUMFERENCE 120.0f
#define RTG_APPROACH_STEPS 120
#define RTG_JOIN_STEPS 40
#define RTG_ARC_STEPS 120
#define RTG_SHARED_STEPS 60
#define RTG_DEPART_STEPS 45
#define RTG_FAR_MIN_WORLD 60.0f
#define RTG_FAR_MAX_WORLD 70.0f

typedef struct {
    float x, y, heading_deg;
} rtg_pose_t;

static inline int rtg_world_white(int side, float x, float y)
{
    float dx = x - side * 90.0f;
    float d2 = dx * dx + y * y;
    return fabsf(x) <= 30.0f || (d2 >= 60.0f * 60.0f && d2 <= 120.0f * 120.0f);
}

static inline void rtg_render(uint8_t image[RING_H][RING_W], int side,
                              const rtg_pose_t *pose)
{
    int x, y;
    float theta = pose->heading_deg * RTG_PI / 180.0f;
    float forward_x = sinf(theta), forward_y = cosf(theta);
    float right_x = cosf(theta), right_y = -sinf(theta);
    for (y = 0; y < RING_H; ++y)
        for (x = 0; x < RING_W; ++x) {
            float lateral = (float)(x - RING_W / 2);
            float forward = (float)(RING_H - 1 - y);
            float wx = pose->x + right_x * lateral + forward_x * forward;
            float wy = pose->y + right_y * lateral + forward_y * forward;
            image[y][x] = (uint8_t)(rtg_world_white(side, wx, wy) ? 255 : 0);
        }
}

static inline float rtg_unwrap(float angle, float reference)
{
    while (angle - reference > 180.0f) angle -= 360.0f;
    while (angle - reference < -180.0f) angle += 360.0f;
    return angle;
}

static inline rtg_pose_t rtg_approach(int frame)
{
    rtg_pose_t p = {0.0f, -180.0f + 240.0f * frame / RTG_APPROACH_STEPS, 0.0f};
    return p;
}

static inline rtg_pose_t rtg_bezier_at(int side, float t, int returning, float far_y)
{
    rtg_pose_t p;
    float u = 1.0f - t, x, y, dx, dy, reference;
    if (!returning) {
        /* Far mouth F(0,60), facing north, to the ring's top tangent
         * (90,90), facing east. All intermediate points remain white. */
        x = 3.0f * u * t * t * 45.0f + t * t * t * 90.0f;
        y = u * u * u * far_y + (1.0f - u * u * u) * 90.0f;
        dx = 270.0f * t - 135.0f * t * t;
        dy = 3.0f * (90.0f - far_y) * u * u;
        reference = 45.0f;
    } else {
        /* Bottom tangent (90,-90), facing west, to near mouth N(0,-60),
         * facing north. This closes the turn before the shared straight. */
        x = u * u * u * 90.0f + 3.0f * u * u * t * 45.0f;
        y = -90.0f + (90.0f - far_y) * t * t * t;
        dx = -135.0f * (1.0f - t * t);
        dy = 3.0f * (90.0f - far_y) * t * t;
        reference = 315.0f;
    }
    p.x = side * x;
    p.y = y;
    p.heading_deg = side * rtg_unwrap(atan2f(dx, dy) * 180.0f / RTG_PI, reference);
    return p;
}

static inline rtg_pose_t rtg_join(int side, int frame)
{
    return rtg_bezier_at(side, (float)frame / RTG_JOIN_STEPS, 0, RTG_FAR_MIN_WORLD);
}

static inline rtg_pose_t rtg_join_at(int side, int frame, float far_y)
{
    return rtg_bezier_at(side, (float)frame / RTG_JOIN_STEPS, 0, far_y);
}

static inline rtg_pose_t rtg_outer_arc(int side, int frame)
{
    float phi = (90.0f - 180.0f * frame / RTG_ARC_STEPS) * RTG_PI / 180.0f;
    rtg_pose_t p;
    p.x = side * (90.0f + 90.0f * cosf(phi));
    p.y = 90.0f * sinf(phi);
    p.heading_deg = side * (180.0f - phi * 180.0f / RTG_PI);
    return p;
}

static inline rtg_pose_t rtg_return_near(int side, int frame)
{
    return rtg_bezier_at(side, (float)frame / RTG_JOIN_STEPS, 1, RTG_FAR_MIN_WORLD);
}

static inline rtg_pose_t rtg_return_near_at(int side, int frame, float far_y)
{
    return rtg_bezier_at(side, (float)frame / RTG_JOIN_STEPS, 1, far_y);
}

static inline rtg_pose_t rtg_shared_return(int side, int frame)
{
    rtg_pose_t p = {0.0f, -60.0f + 120.0f * frame / RTG_SHARED_STEPS, side * 360.0f};
    return p;
}

static inline rtg_pose_t rtg_depart(int side, int frame)
{
    rtg_pose_t p = {0.0f, 60.0f + 90.0f * frame / RTG_DEPART_STEPS, side * 360.0f};
    return p;
}

static inline rtg_pose_t rtg_shared_return_at(int side, int frame, float far_y)
{
    rtg_pose_t p = {0.0f, -far_y + 2.0f * far_y * frame / RTG_SHARED_STEPS, side * 360.0f};
    return p;
}

static inline rtg_pose_t rtg_depart_at(int side, int frame, float far_y)
{
    rtg_pose_t p = {0.0f, far_y + 90.0f * frame / RTG_DEPART_STEPS, side * 360.0f};
    return p;
}

static inline void rtg_straight(uint8_t image[RING_H][RING_W])
{
    int x, y;
    for (y = 0; y < RING_H; ++y)
        for (x = 0; x < RING_W; ++x)
            image[y][x] = (uint8_t)(x >= 64 && x <= 123 ? 255 : 0);
}

static inline void rtg_cross(uint8_t image[RING_H][RING_W])
{
    int y;
    rtg_straight(image);
    for (y = 45; y <= 75; ++y) memset(image[y], 255, RING_W);
}

#endif
