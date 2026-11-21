#include "ring_test_geometry.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do { if(!(x)) { fprintf(stderr,"FAIL assist line %d: %s ph=%d src=%u mode=%u e=%d flags=%u dec=%u rej=%u hold=%u\n",__LINE__,#x,ring_result.phase,ring_result.path_source,ring_result.branch_mode,ring_result.steer_error,ring_result.entry_flags,ring_result.decision,ring_result.reject,ring_result.hold_reject);exit(1); } } while(0)
static uint8_t image[RING_H][RING_W];
static ring_motion_t motion;
static unsigned frames;
static void step(unsigned dt)
{
    ring_motion_t sensor;
    motion.now_ms+=dt;
    ring_observe_track(1);ring_observe_frame_interval((uint16_t)dt);
    sensor=motion;sensor.yaw_deg=motion.yaw_deg*RING_YAW_SIGN;
    ring_update((const uint8_t (*)[RING_W])image,&sensor);++frames;
}
static int active(void) {return !!(ring_result.entry_flags&RING_ENTRY_ASSIST);}
static void start(int route)
{
    int i;
    ring_init();motion=(ring_motion_t){0,0,0,1};rg_entry(image,route,10,0);
    for(i=0;i<RING_CONFIRM_FRAMES;++i) step(20);
    CHECK(ring_result.phase==RING_ENTER && !active() && !ring_result.committed);
    CHECK(ring_result.branch_ok && ring_result.path_source==1);
    step(20);
    CHECK(active() && ring_result.decision==RING_DECISION_ASSIST);
    CHECK(ring_result.branch_ok && ring_result.path_source==1);
    CHECK(ring_result.branch_mode==RING_BRANCH_ASSIST);
    CHECK(ring_result.steer_error*route>=RING_ASSIST_MIN_ERROR);
    CHECK(!!(ring_result.entry_flags&RING_ENTRY_ASSIST_LEFT)==(route<0));
}
static void clipped(void)
{
    /* The actual lower white lane and previous accepted control points
     * remain visible; the upper island contour is temporarily occluded. */
    rg_rect(image,0,0,RING_W-1,70,0);
}
int main(void)
{
    int route,i;
    for(route=-1;route<=1;route+=2) {
        int old_error;
        start(route);old_error=ring_result.steer_error;
        clipped();motion.yaw_deg=25;motion.wheel_revs=0.2f;step(120);
        CHECK(ring_result.phase==RING_ENTER && active() && !ring_result.stop);
        CHECK(ring_result.path_source==5 && !ring_result.branch_ok && !ring_result.path_valid);
        CHECK(ring_result.branch_mode==RING_BRANCH_ASSIST && ring_result.steer_error==old_error);
        CHECK(ring_result.hold_reject&RING_HOLD_YAW);
        CHECK(!(ring_result.entry_flags&(RING_ENTRY_MEASURED|RING_ENTRY_HISTORY_FRESH|RING_ENTRY_COMMITTED)));
        CHECK(!ring_result.committed && ring_result.phase!=RING_CIRCLE && ring_result.exited_events==0);
        CHECK(ring_result.path_age_ms==120 && ring_result.strong_age_ms==120);
        motion.yaw_deg=30;motion.wheel_revs=0.3f;step(80);
        CHECK(active() && ring_result.path_source==5 && !ring_result.committed);
        step(1);
        CHECK(!active() && ring_result.phase==RING_COOLDOWN && !ring_result.stop);
        CHECK(ring_result.decision==RING_DECISION_ASSIST_EXPIRED);
        CHECK(ring_result.entry_flags&RING_ENTRY_ASSIST_SPENT);
    }
    puts("PASS: mirrored confirmed entry bridges a current white corridor through 120ms/25deg without inventing branch or entry evidence; 200ms bound expires");
    for(route=-1;route<=1;route+=2) {
        int released=0;
        start(route);
        for(i=0;i<=12;++i) {
            rg_transition(image,route,i,12);step(20);
            CHECK(ring_result.branch_ok && ring_result.path_source==1);
            if(!active()) released=1;
            if(released) {
                CHECK(!active() && ring_result.entry_flags&RING_ENTRY_ASSIST_SPENT);
                CHECK(ring_result.branch_mode==RING_BRANCH_STRONG);
            } else CHECK(ring_result.steer_error*route>=RING_ASSIST_MIN_ERROR);
        }
        CHECK(released && ring_result.steer_error*route<0);
        CHECK(!ring_result.committed && ring_result.phase!=RING_CIRCLE);
    }
    puts("PASS: real evolving arc releases the entry direction as the measured turn reduces/reverses; it cannot re-arm in the same entry");
    for(route=-1;route<=1;route+=2) {
        start(route);
        for(i=0;i<32;++i) step(20);
        CHECK(!active() && ring_result.entry_flags&RING_ENTRY_ASSIST_SPENT);
        CHECK(ring_result.branch_ok && ring_result.path_source==1);
        CHECK(!ring_result.committed && ring_result.phase==RING_ENTER);
    }
    puts("PASS: continuing strong fixed-island pixels cannot extend the total assist budget or turn supplied yaw into CIRCLE");
    for(i=0;i<3;++i) {
        int j;
        ring_init();motion=(ring_motion_t){0,0,0,1};
        if(i==0) rg_road(image,0);else if(i==1) rg_cross(image,0);else rg_unrelated_patches(image);
        for(j=0;j<12;++j) {
            motion.yaw_deg+=15;motion.wheel_revs+=0.04f;step(20);
            CHECK(!active() && ring_result.phase==RING_NORMAL && !ring_result.detected_events);
            CHECK(ring_blend_error((int16_t)(j-5))==j-5 && ring_turn_boost()==0 && ring_speed_limit()==-1);
        }
    }
    puts("PASS: ordinary road, cross and disconnected flecks preserve original error, speed and turn ownership");
    printf("PASS: %u bounded-entry-assist image frames (synthetic, not real-camera replay)\n",frames);
    return 0;
}
