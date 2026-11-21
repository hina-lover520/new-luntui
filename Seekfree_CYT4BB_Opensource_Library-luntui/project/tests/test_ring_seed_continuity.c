#include "ring_test_geometry.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL seed-continuity line %d: %s ph=%d dir=%d src=%u mode=%u branch=%u near=%d rej=%u hold=%u stop=%u\n",__LINE__,#x,ring_result.phase,ring_result.route,ring_result.path_source,ring_result.branch_mode,ring_result.branch_ok,ring_result.near_x,ring_result.reject,ring_result.hold_reject,ring_result.stop);exit(1);}}while(0)
static uint8_t image[RING_H][RING_W],saved[RING_H][RING_W];
static ring_motion_t motion;
static unsigned frames;
static void step(unsigned dt)
{
    int y;
    ring_motion_t sensor=motion;
    motion.now_ms+=dt;sensor.now_ms=motion.now_ms;sensor.yaw_deg=motion.yaw_deg/RING_YAW_SIGN;
    memcpy(saved,image,sizeof(image));ring_observe_track(1);ring_observe_frame_interval((uint16_t)dt);
    ring_update((const uint8_t (*)[RING_W])image,&sensor);++frames;
    CHECK(!memcmp(saved,image,sizeof(image)));
    if(ring_result.branch_ok) for(y=0;y<RING_H;++y)
        if(ring_result.mid[y]>=0) CHECK(image[y][ring_result.mid[y]]);
}
static void start_circle(int route)
{
    int i;
    ring_init();motion=(ring_motion_t){0,0,0,1};rg_entry(image,route,0,0);
    for(i=0;i<RING_CONFIRM_FRAMES;++i) step(20);
    for(i=1;i<=12;++i){rg_transition(image,route,i,12);motion.yaw_deg=route*4.0f*i;motion.wheel_revs+=0.025f;step(20);}
    rg_orbit(image,route,90);
    for(i=0;i<6;++i){motion.yaw_deg+=route*2;motion.wheel_revs+=0.01f;step(20);}
    CHECK(ring_result.phase==RING_CIRCLE && ring_result.committed);
}
static void translated(int route,int shift)
{
    rg_annulus(image,94-route*90+route*shift,119,60,120,0,0);
}
static void advance(int route,int last)
{
    int i;
    start_circle(route);
    for(i=1;i<=last;++i) {
        translated(route,i);motion.yaw_deg+=route*0.5f;motion.wheel_revs+=0.005f;step(20);
        /* The entire measured arc moves by one pixel. v9 fails at +49px
         * because its bottom connected component still starts at x=94. */
        CHECK(ring_result.branch_ok && ring_result.path_source==1);
        CHECK(ring_result.branch_mode==RING_BRANCH_STRONG && !ring_result.stop);
    }
}
static void false_road(int route)
{
    if(route>0) rg_rect(image,64,0,101,119,255);
    else rg_rect(image,87,0,124,119,255);
}
int main(void)
{
    int route,i;
    for(route=-1;route<=1;route+=2) {
        int16_t last_error;
        advance(route,55);
        CHECK(route*(ring_result.near_x-RING_W/2)>45);
        translated(route,55);false_road(route);step(20);
        CHECK(ring_result.branch_ok && ring_result.branch_mode==RING_BRANCH_STRONG);
        CHECK(route*(ring_result.near_x-RING_W/2)>45);
        for(i=0;i<5;++i) {step(20);CHECK(ring_result.branch_ok && ring_result.path_source==1);}
        CHECK(ring_result.detected_events==1 && !ring_result.exited_events);

        /* A centre road remains visible after the acquired bottom run is
         * removed. It can never refresh the acquired branch or its budget. */
        advance(route,55);last_error=ring_result.steer_error;rg_road(image,0);
        for(i=0;i<(int)RING_HOLD_FRAMES;++i) {
            step(20);CHECK(!ring_result.branch_ok && ring_result.path_source==2);
            CHECK(ring_result.steer_error==last_error && !ring_result.stop);
        }
        step(20);CHECK(ring_result.stop && !ring_result.branch_ok && ring_result.path_source==0);

        /* The branch is still at the bottom, but a dark band cuts its
         * connection to the upper visible arc; a separate white road does
         * not bridge that gap on the acquired branch's behalf. */
        advance(route,55);translated(route,55);false_road(route);
        rg_rect(image,route>0?110:0,98,route>0?187:78,101,0);
        for(i=0;i<6;++i){step(20);CHECK(!ring_result.branch_ok && ring_result.path_source!=1);}
        CHECK(ring_result.stop);

        /* Off-centre history is bounded by the same original deadline.
         * A late frame cannot revive it just because the road is visible. */
        advance(route,55);translated(route,55);step(RING_PATH_HOLD_MS+1);
        CHECK(!ring_result.branch_ok && ring_result.stop);
        CHECK(ring_result.hold_reject&RING_HOLD_TIME);
        step(20);CHECK(!ring_result.branch_ok && ring_result.stop);
    }
    puts("PASS: mirrored acquired arcs remain connected beyond the centre seed; unrelated centre roads, cuts and expired history cannot replace them");
    for(i=0;i<2;++i) {
        ring_init();motion=(ring_motion_t){0,0,0,1};
        if(i) rg_cross(image,0);else rg_road(image,0);
        for(route=0;route<12;++route) {
            step(20);CHECK(ring_result.phase==RING_NORMAL && !ring_result.detected_events);
            CHECK(ring_blend_error(17)==17 && ring_speed_limit()==-1);
        }
    }
    puts("PASS: cold ordinary/cross acquisition and original steering passthrough remain unchanged");
    printf("PASS: %u seed-continuity image frames (synthetic, not field replay)\n",frames);
    return 0;
}
