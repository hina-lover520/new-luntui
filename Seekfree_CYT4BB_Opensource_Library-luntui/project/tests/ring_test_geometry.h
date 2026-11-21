#ifndef RING_TEST_GEOMETRY_H
#define RING_TEST_GEOMETRY_H

#include "ring.h"
#include <string.h>

/* Synthetic IMAGE-PIXEL geometry, not metric/IPM calibration or a replay.
 * These functions only write pixels. They never set motion, route or phase. */
static inline void rg_rect(uint8_t image[RING_H][RING_W], int x0, int y0,
                           int x1, int y1, uint8_t color)
{
    int x, y;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= RING_W) x1 = RING_W - 1;
    if (y1 >= RING_H) y1 = RING_H - 1;
    for (y = y0; y <= y1; ++y)
        for (x = x0; x <= x1; ++x) image[y][x] = color;
}

static inline void rg_road(uint8_t image[RING_H][RING_W], int slope)
{
    int y;
    memset(image, 0, RING_H * RING_W * sizeof(image[0][0]));
    for (y = 0; y < RING_H; ++y) {
        int shift = (119 - y) * slope / 4;
        rg_rect(image, 64 + shift, y, 123 + shift, y, 255);
    }
}

static inline void rg_annulus(uint8_t image[RING_H][RING_W], int cx, int cy,
                              int inner, int outer, int inlet, int slope)
{
    int x, y;
    for (y = 0; y < RING_H; ++y)
        for (x = 0; x < RING_W; ++x) {
            int local_x = x - (119 - y) * slope / 4;
            int dx = local_x - cx, dy = y - cy;
            int d2 = dx * dx + dy * dy;
            int on_ring = d2 >= inner * inner && d2 <= outer * outer;
            int on_inlet = inlet && y >= cy && local_x >= 64 && local_x <= 123;
            image[y][x] = (uint8_t)(((on_ring || on_inlet) &&
                                   d2 >= inner * inner) ? 255 : 0);
        }
}

/* The closed curved inner island has a narrowing lower tip, widening
 * middle, upper narrowing return and a connected incoming white lane. */
static inline void rg_entry(uint8_t image[RING_H][RING_W], int route,
                            int advance, int slope)
{
    rg_annulus(image, route > 0 ? 80 : 108, 45 + advance, 26, 86, 1, slope);
}

/* The island's far cap is above the image. Its visible curved boundary and
 * connected inlet are retained; no rectangle stands in for the island. */
static inline void rg_partial_entry(uint8_t image[RING_H][RING_W], int route,
                                    int advance, int slope)
{
    /* Keep the lower tip on its chosen side when shearing the view. */
    int cx = (route > 0 ? 80 : 108) - 12 * slope;
    rg_annulus(image, cx, 20 + advance, 50, 110, 1, slope);
}

/* A tangent view of an annular lane. The inner island lies mostly outside
 * the image and its inner boundary bends visibly across near/look rows.
 * The same geometry can persist during a lap while measured yaw advances. */
static inline void rg_orbit(uint8_t image[RING_H][RING_W], int route, int radius)
{
    rg_annulus(image, 94 - route * radius, 119,
               radius - 30, radius + 30, 0, 0);
}

static inline void rg_transition_from(uint8_t image[RING_H][RING_W], int route,
                                      int start_x, int frame, int frames)
{
    int target_x = 94 - route * 90;
    int cx = start_x + (target_x - start_x) * frame / frames;
    int cy = 58 + 61 * frame / frames;
    int inner = 26 + 34 * frame / frames;
    rg_annulus(image, cx, cy, inner, inner + 60, 1, 0);
}

static inline void rg_transition(uint8_t image[RING_H][RING_W], int route,
                                 int frame, int frames)
{
    rg_transition_from(image, route, route > 0 ? 80 : 108, frame, frames);
}

static inline void rg_exit(uint8_t image[RING_H][RING_W], int route)
{
    rg_orbit(image, route, 90);
    /* Only the outer side opens. Preserve the curved inner boundary. */
    if (route > 0) rg_rect(image, 95, 75, 180, 90, 255);
    else rg_rect(image, 7, 75, 93, 90, 255);
}

static inline void rg_cross(uint8_t image[RING_H][RING_W], int slope)
{
    int x, y;
    rg_road(image, 0);
    for (y = 0; y < RING_H; ++y)
        for (x = 0; x < RING_W; ++x) {
            int shifted_y = y - (x - 94) * slope / 7;
            if (shifted_y >= 40 && shifted_y <= 70) image[y][x] = 255;
        }
}

/* Both ring-like dark structure and an aligned through-road above two
 * lateral mouths are present. Ambiguous evidence must not acquire a ring. */
static inline void rg_conflict(uint8_t image[RING_H][RING_W], int route)
{
    rg_entry(image, route, 0, 0);
    rg_rect(image, 0, 71, 187, 119, 0);
    rg_rect(image, 64, 71, 123, 119, 255);
    rg_rect(image, 0, 80, 187, 92, 255);
}

static inline void rg_t(uint8_t image[RING_H][RING_W])
{
    memset(image, 0, RING_H * RING_W * sizeof(image[0][0]));
    rg_rect(image, 64, 55, 123, 119, 255);
    rg_rect(image, 0, 40, 187, 70, 255);
}

static inline void rg_fork(uint8_t image[RING_H][RING_W])
{
    int y;
    rg_road(image, 0);
    rg_rect(image, 0, 0, 187, 79, 0);
    for (y = 0; y < 80; ++y) {
        int opening = (80 - y) / 2;
        rg_rect(image, 76 - opening, y, 110 - opening, y, 255);
        rg_rect(image, 77 + opening, y, 111 + opening, y, 255);
    }
}

static inline void rg_unrelated_patches(uint8_t image[RING_H][RING_W])
{
    rg_road(image, 0);
    rg_rect(image, 8, 35, 35, 75, 255);
    rg_rect(image, 151, 20, 179, 60, 255);
}

static inline void rg_rectangular_dark(uint8_t image[RING_H][RING_W])
{
    rg_road(image, 0);
    rg_rect(image, 24, 20, 163, 101, 255);
    rg_rect(image, 78, 20, 110, 65, 0);
}

#endif
