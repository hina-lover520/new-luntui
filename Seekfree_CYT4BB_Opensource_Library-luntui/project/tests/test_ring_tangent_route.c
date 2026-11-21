#include "ring_tangent_test_geometry.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(c) do { if(!(c)) { fprintf(stderr,"FAIL tangent-route:%d %s ph=%d stage=%u left=%u cm=%u src=%u lap=%.1f\n",__LINE__,#c,ring_result.phase,ring_result.approach_stage,ring_result.left_entry,ring_result.committed,ring_result.path_source,ring_result.lap_deg);exit(1);}}while(0)
static uint8_t image[RING_H][RING_W], saved[RING_H][RING_W];
static ring_motion_t motion;
static rtg_pose_t last;
static unsigned frames;

static void update(uint16_t vdt)
{
    motion.now_ms+=20;ring_observe_track(1);ring_observe_frame_interval(vdt);
    memcpy(saved,image,sizeof(saved));ring_update((const uint8_t(*)[RING_W])image,&motion);
    CHECK(!memcmp(saved,image,sizeof(saved)));++frames;
}
static void world(int side,rtg_pose_t p)
{
    float dx=p.x-last.x,dy=p.y-last.y;
    motion.wheel_revs+=sqrtf(dx*dx+dy*dy)/RTG_WHEEL_CIRCUMFERENCE;
    motion.yaw_deg=p.heading_deg;rtg_render(image,side,&p);update(20);last=p;
}
static float approach(int side)
{
    int i;
    ring_init();motion=(ring_motion_t){0,0,0,1};last=rtg_approach(0);
    for(i=0;i<=125;++i) {
        rtg_pose_t p=rtg_approach(i);world(side,p);
        CHECK(!ring_result.stop && !ring_result.fault);
        if(ring_owns_path()) {
            CHECK(p.y>=RTG_FAR_MIN_WORLD && p.y<=RTG_FAR_MAX_WORLD);
            CHECK(ring_result.phase==RING_ENTER && side*ring_result.steer_error>0);
            return p.y;
        }
        CHECK(ring_blend_error(19)==19 && ring_speed_limit()==-1 && ring_turn_boost()==0);
    }
    CHECK(0);return 0;
}
static void frozen_far_cannot_depart(int side)
{
    int i;
    (void)approach(side);
    for(i=0;i<40;++i) {
        /* Sensor motion alone cannot turn one frozen mouth into a real
         * visual departure, nor produce full-circle task completion. */
        motion.yaw_deg+=side*5.0f;motion.wheel_revs+=0.02f;update(20);
        CHECK(!ring_result.left_entry && !ring_result.committed);
        CHECK(ring_result.phase!=RING_CIRCLE && !ring_result.exited_events);
    }
}
static void unknown_frame_cannot_commit(int side)
{
    float f=approach(side);
    int i;
    for(i=1;i<=RTG_JOIN_STEPS;++i) {
        rtg_pose_t p=rtg_join_at(side,i,f);
        motion.wheel_revs+=0.02f;motion.yaw_deg=p.heading_deg;
        rtg_render(image,side,&p);update(65535u);
        CHECK(!ring_result.left_entry && !ring_result.committed);
        CHECK(ring_result.phase!=RING_CIRCLE && !ring_result.exited_events);
    }
}
static void direct_budget_does_not_refund(int side,int yaw_sway)
{
    int i,saw_assist=0;
    float total=0;
    (void)approach(side);
    for(i=0;i<16;++i) {
        if(yaw_sway) {
            float change=(i%2?-1.0f:1.0f)*side*10.0f;
            motion.yaw_deg+=change;total+=fabsf(change);
        } else {
            float change=(i%2?-1.0f:1.0f)*0.2f;
            motion.wheel_revs+=change;total+=fabsf(change);
        }
        update(20);
        if(total>(yaw_sway?RING_FORCE_ENTRY_YAW_DEG:RING_FORCE_ENTRY_REVS))
            CHECK(ring_result.path_source!=5);
        else if(ring_result.path_source==5) saw_assist=1;
        CHECK(!ring_result.left_entry && !ring_result.committed && !ring_result.exited_events);
    }
    CHECK(saw_assist);
}
int main(void)
{
    CHECK(RING_TRACK_MODE==1);
    frozen_far_cannot_depart(1);frozen_far_cannot_depart(-1);
    unknown_frame_cannot_commit(1);unknown_frame_cannot_commit(-1);
    direct_budget_does_not_refund(1,1);direct_budget_does_not_refund(-1,1);
    direct_budget_does_not_refund(1,0);direct_budget_does_not_refund(-1,0);
    printf("PASS: %u tangent frames, frozen/unknown cannot fabricate departure, direct sway/reverse cannot refund motion budget\n",frames);
    return 0;
}
