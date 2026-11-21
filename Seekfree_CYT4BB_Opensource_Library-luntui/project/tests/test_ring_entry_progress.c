#include "ring_test_geometry.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do { if(!(x)) { fprintf(stderr,"FAIL entry line %d: %s ph=%d cm=%u branch=%u src=%u flags=%u hold=%u decision=%u\n",__LINE__,#x,ring_result.phase,ring_result.committed,ring_result.branch_ok,ring_result.path_source,ring_result.entry_flags,ring_result.hold_reject,ring_result.decision);exit(1); } } while(0)
static uint8_t image[RING_H][RING_W];
static ring_motion_t motion;
static unsigned frames;
static void step(unsigned dt)
{
    int y;
    ring_motion_t sensor;
    motion.now_ms+=20;ring_observe_track(1);ring_observe_frame_interval((uint16_t)dt);
    sensor=motion;sensor.yaw_deg=motion.yaw_deg*RING_YAW_SIGN;
    ring_update((const uint8_t (*)[RING_W])image,&sensor);++frames;
    if(ring_result.branch_ok) for(y=0;y<RING_H;++y)
        if(ring_result.mid[y]>=0) CHECK(image[y][ring_result.mid[y]]);
}
static void start(int route)
{
    int i;
    ring_init();motion=(ring_motion_t){0,0,0,1};rg_entry(image,route,0,0);
    for(i=0;i<RING_CONFIRM_FRAMES;++i) step(20);
    CHECK(ring_result.phase==RING_APPROACH && !ring_result.committed);
}
static void progress(int route)
{
    int i;
    start(route);
    /* Same island moves towards the car, but its tip is still above row108.
     * These are actual pixel changes, not motion injected into a fixed view. */
    for(i=1;i<=2;++i) {
        rg_transition(image,route,i,12);motion.yaw_deg=route*8.0f*i;
        motion.wheel_revs+=0.025f;step(20);
        CHECK(ring_result.branch_ok && ring_result.path_source==1);
    }
    CHECK(ring_result.phase==RING_ENTER && !ring_result.committed);
    CHECK(ring_result.island_near>=96 && ring_result.island_near<RING_NEAR_ROW);
}
static void finish_entry(int route)
{
    int i;
    for(i=3;i<=12;++i) {
        rg_transition(image,route,i,12);motion.yaw_deg=route*(20.0f+i*3.0f);
        motion.wheel_revs+=0.025f;step(20);
        CHECK(ring_result.branch_ok && ring_result.path_source==1 && !ring_result.stop);
    }
    rg_orbit(image,route,90);
    for(i=0;i<6;++i) {motion.yaw_deg+=route*2;motion.wheel_revs+=0.025f;step(20);}
    CHECK(ring_result.phase==RING_CIRCLE && ring_result.committed && !ring_result.stop);
}
static void mirror(void)
{
    int x,y;
    for(y=0;y<RING_H;++y) for(x=0;x<RING_W/2;++x) {
        uint8_t p=image[y][x];image[y][x]=image[y][RING_W-1-x];image[y][RING_W-1-x]=p;
    }
}
static void tangent_island(int route)
{
    int x,y;
    rg_transition(image,1,2,12);
    /* A cropped island presents an almost straight selected edge. Its
     * other edge still curves, closes and joins the incoming white lane. */
    for(y=37;y<=98;++y) {
        int dy=y-68;
        if(dy*dy<=31*31) {
            int span=(int)sqrtf((float)(31*31-dy*dy));
            for(x=68-span;x<=68+span;++x) image[y][x]=255;
            for(x=68-span;x<=90;++x) image[y][x]=0;
        }
    }
    if(route<0) mirror();
}
int main(int argc,char **argv)
{
    int route,i;
    int tangent_only=argc>1 && !strcmp(argv[1],"tangent");
    if(!tangent_only) for(route=-1;route<=1;route+=2) {
        int16_t held_error,raw_error;
        progress(route);held_error=ring_result.steer_error;
        /* The strong path's measured geometry is retained independently
         * of the temporary control floor. Black occlusion ends assistance
         * and must hold this raw geometry error, not the force floor. */
        raw_error=(int16_t)((ring_result.near_x-RING_W/2-1)+
            (ring_result.guide_x-ring_result.near_x)*(RING_NEAR_ROW-RING_LOOK_ROW)*2/
            (3*(RING_NEAR_ROW-ring_result.guide_y)));
        CHECK(abs(held_error)>=abs(raw_error));
        memset(image,0,sizeof(image));motion.yaw_deg=route*21;motion.wheel_revs+=0.025f;step(20);
        CHECK(ring_result.committed && ring_result.phase==RING_ENTER);
        CHECK(!ring_result.branch_ok && ring_result.path_source==2 && !ring_result.stop);
        CHECK(ring_result.steer_error==raw_error && ring_result.decision==RING_DECISION_COMMIT);
        CHECK(!(ring_result.entry_flags&RING_ENTRY_ASSIST));
        CHECK(!ring_result.hold_reject && ring_result.hold_turn10==50);
        motion.yaw_deg=route*24;motion.wheel_revs+=0.01f;step(20);
        CHECK(ring_result.path_source==2 && !ring_result.branch_ok && !ring_result.stop);
        finish_entry(route);
    }
    if(!tangent_only) puts("PASS: advancing island, short occlusion before row108, retained control, reacquisition and CIRCLE");
    for(route=-1;route<=1;route+=2) {
        progress(route);tangent_island(route);motion.yaw_deg=route*21;motion.wheel_revs+=0.025f;step(20);
        CHECK(ring_result.branch_ok && ring_result.path_source==1 && ring_result.committed);
        CHECK(!ring_result.stop && ring_result.island_near>=96);
        /* This maintenance is unavailable to a through-road. */
        rg_road(image,0);step(20);
        CHECK(!ring_result.branch_ok && ring_result.path_source!=1);
    }
    puts("PASS: measured tangent of the same island maintains branch; straight lane cannot refresh it");
    if(tangent_only) return 0;
    for(route=-1;route<=1;route+=2) {
        start(route);rg_entry(image,route,13,0);
        for(i=0;i<8;++i) {motion.yaw_deg=route*45;motion.wheel_revs+=0.03f;step(20);CHECK(!ring_result.committed && ring_result.phase!=RING_CIRCLE);}
        for(i=0;i<2;++i) {
            progress(route);if(i) rg_cross(image,0);else rg_road(image,0);
            motion.yaw_deg=route*25;motion.wheel_revs+=0.03f;
            step(20);CHECK(!ring_result.committed);
            step(20);step(20);
            CHECK(ring_result.phase==RING_COOLDOWN && !ring_result.stop && !ring_result.fault);
            CHECK(ring_result.decision==(i?RING_DECISION_CANCEL_CROSS:RING_DECISION_CANCEL_ORDINARY));
            step(20);CHECK(ring_result.decision==(i?RING_DECISION_CANCEL_CROSS:RING_DECISION_CANCEL_ORDINARY));
        }
        progress(route);memset(image,0,sizeof(image));motion.yaw_deg=route*35;motion.wheel_revs+=0.03f;step(20);
        CHECK(!ring_result.committed && ring_result.phase==RING_COOLDOWN && !ring_result.stop);
        CHECK(ring_result.hold_reject&RING_HOLD_YAW);
    }
    puts("PASS: fixed island, straight/cross and expired history cannot turn motion into physical entry");
    start(1);
    for(i=3;i<=30;i+=3) {rg_entry(image,1,i,0);step(65535);CHECK(!ring_result.committed);}
    motion.yaw_deg=25;motion.wheel_revs=0.06f;step(20);
    CHECK(!ring_result.committed && !(ring_result.entry_flags&RING_ENTRY_PROGRESSED));
    puts("PASS: unknown visual intervals cannot accumulate/revive entry progress on the next fresh frame");
    printf("PASS: %u entry continuity regression frames (synthetic geometry, not field replay)\n",frames);
    return 0;
}
