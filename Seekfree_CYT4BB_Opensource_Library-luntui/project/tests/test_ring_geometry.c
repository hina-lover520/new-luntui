#include "ring_test_geometry.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

extern uint32_t ring_scan_pixels;
extern uint32_t ring_graph_checks;
static uint8_t image[RING_H][RING_W];
static ring_motion_t motion = {0, 0, 0, 1};
static uint32_t peak_scan;
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL geometry line %d: %s phase=%d route=%d\n", __LINE__, #x, ring_result.phase, ring_result.route); exit(1); } } while (0)

static void rect(int x0, int y0, int x1, int y1, uint8_t color)
{
    int x, y;
    for (y = y0; y <= y1; ++y)
        for (x = x0; x <= x1; ++x) image[y][x] = color;
}

static void road(void)
{
    memset(image, 0, sizeof(image));
    rect(64, 0, 123, 119, 255);
}

static void step(void)
{
    motion.now_ms += 20;
    ring_update((const uint8_t (*)[RING_W])image, &motion);
    /* A deterministic work bound, rather than a host-time claim about MCU
     * response. Active path processing reuses the exact same row runs. */
    CHECK(ring_scan_pixels <= 3u * RING_W * RING_H);
    CHECK(ring_graph_checks <= 4u * RING_W * RING_H);
    if (ring_scan_pixels > peak_scan) peak_scan = ring_scan_pixels;
}

static void ordinary(const char *name)
{
    int i;
    ring_init();
    for (i = 0; i < 12; ++i) {
        step();
        CHECK(ring_result.phase == RING_NORMAL);
        CHECK(ring_speed_limit() == -1 && !ring_owns_path());
        CHECK(ring_result.detected_events == 0);
    }
    printf("PASS: ordinary %s does not hand over\n", name);
}

static void side_entry(int side, int bulge)
{
    road();
    if (side > 0) {
        rect(124, 50, 187, 80, 255);
        if (bulge) rect(100, 34, 123, 49, 0);
    } else {
        rect(0, 50, 63, 80, 255);
        if (bulge) rect(64, 34, 87, 49, 0);
    }
}

int main(void)
{
    int i, side, slope, partial, x, y;
    clock_t begin;
    road(); ordinary("straight");
    for (side = -1; side <= 1; side += 2) {
        side_entry(side, 0); ordinary("one-sided spur without inner bulge");
    }
    road(); rect(0, 50, 187, 80, 255); ordinary("cross");
    road(); rect(24, 20, 163, 101, 255); rect(78, 20, 110, 65, 0);
    ordinary("rectangular dark patch (old false candidate)");
    road(); rect(24, 0, 163, 101, 255); rect(70, 0, 110, 82, 0);
    ordinary("open hairpin peninsula");
    memset(image, 0, sizeof(image));
    for (y = 0; y < RING_H; ++y) {
        int shift = (RING_H - y) * (RING_H - y) / 500;
        rect(64 + shift, y, 123 + shift, y, 255);
    }
    ordinary("smooth curve");
    road(); rect(0, 0, 187, 30, 255); ordinary("T junction");
    for (side = -1; side <= 1; side += 2) {
        side_entry(side, 1);
        ordinary("rectangular side bulge is not sufficient ring evidence");
    }
    rg_t(image); ordinary("T geometry");
    rg_fork(image); ordinary("fork without upper return");
    rg_unrelated_patches(image); ordinary("disconnected white patches");
    rg_conflict(image, 1); ordinary("conflicting ring and through-cross geometry");
    for (side = -1; side <= 1; side += 2)
        for (slope = -1; slope <= 1; ++slope)
            for (partial = 0; partial <= 1; ++partial) {
                ring_init();
                if (partial) rg_partial_entry(image, side, 0, slope);
                else rg_entry(image, side, 0, slope);
                for (i = 0; i < RING_CONFIRM_FRAMES; ++i) step();
                if (!ring_owns_path()) fprintf(stderr,"entry side=%d slope=%d partial=%d seen=%u score=%u reject=%u cf=%u\n",side,slope,partial,ring_result.candidate_detected,ring_result.score,ring_result.reject,ring_result.confirm_frames);
                CHECK(ring_owns_path() && ring_result.path_valid && !ring_result.stop);
                CHECK(ring_result.route == side && ring_result.branch_ok);
                CHECK(ring_result.path_source == 1 && ring_result.detected_events == 1);
                CHECK(ring_result.near_x >= 0 && ring_result.look_x >= 0);
                CHECK(image[RING_NEAR_ROW][ring_result.near_x]);
                CHECK(image[RING_LOOK_ROW][ring_result.look_x]);
            }
    puts("PASS: mirrored/slanted curved entry and far-cap-outside-image entry");
    /* An island over disconnected or absent near road must not park a car. */
    road(); rect(24, 20, 163, 101, 255); rect(78, 20, 110, 65, 0);
    for (y = 57; y <= 65; ++y) {
        rect(78, y, 110, y, 255);
        rect(94 - (67 - y), y, 94 + (67 - y), y, 0);
    }
    rect(0, 102, 187, 119, 0);
    ordinary("candidate without connected near path");

    begin = clock();
    for (i = 0; i < 1000; ++i) {
        ring_init();
        for (y = 0; y < RING_H; ++y)
            for (x = 0; x < RING_W; ++x)
                image[y][x] = ((x * 31 + y * 17 + i * 13) % 23 < 11) ? 255 : 0;
        step();
    }
    printf("PASS: row-scan work <= %u pixel reads/frame; noise stress %.3f ms/frame on HOST (not MCU timing)\n",
           peak_scan, 1000.0 * (clock() - begin) / CLOCKS_PER_SEC / 1000);
    return 0;
}
