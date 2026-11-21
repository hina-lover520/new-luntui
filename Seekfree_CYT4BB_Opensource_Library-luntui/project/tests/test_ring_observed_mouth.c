#include "ring_tangent_test_geometry.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(c) do { if(!(c)) { \
    fprintf(stderr,"FAIL observed-mouth:%d %s case=%s ph=%d stage=%u gate=%u N=(%d,%d) F=(%d,%d) count=%u progress=%d br=%u left=%u cm=%u src=%u yaw=%.1f\n", \
        __LINE__,#c,label,ring_result.phase,ring_result.approach_stage,ring_result.mouth_gate, \
        ring_result.mouth_near_x,ring_result.mouth_near_y,ring_result.mouth_far_x,ring_result.mouth_far_y, \
        ring_result.mouth_far_count,ring_result.mouth_progress,ring_result.branch_ok,ring_result.left_entry, \
        ring_result.committed,ring_result.path_source,motion.yaw_deg);exit(1); }}while(0)

static uint8_t image[RING_H][RING_W],saved[RING_H][RING_W];
static ring_motion_t motion;
static rtg_pose_t last;
static const char *label="initial";
static unsigned frames;

static void reset(void)
{
    ring_init();motion=(ring_motion_t){0,0,0,1};last=rtg_approach(0);
}
static void observe(uint16_t vdt,int track)
{
    motion.now_ms+=20;ring_observe_track(track);ring_observe_frame_interval(vdt);
    memcpy(saved,image,sizeof(saved));ring_update((const uint8_t (*)[RING_W])image,&motion);
    CHECK(!memcmp(saved,image,sizeof(saved)));++frames;
}
static void world(int side,rtg_pose_t p,uint16_t vdt)
{
    float dx=p.x-last.x,dy=p.y-last.y;
    motion.wheel_revs+=sqrtf(dx*dx+dy*dy)/RTG_WHEEL_CIRCUMFERENCE;
    motion.yaw_deg=p.heading_deg;rtg_render(image,side,&p);observe(vdt,1);last=p;
}
static void ordinary_output(void)
{
    CHECK(!ring_owns_path() && !ring_blocks_cross());
    CHECK(ring_blend_error(17)==17 && ring_turn_boost()==0 && ring_speed_limit()==-1);
    CHECK(!ring_result.stop && !ring_result.fault && !ring_result.left_entry && !ring_result.committed);
}
static void no_departure(void)
{
    CHECK(!ring_result.left_entry && !ring_result.committed);
    CHECK(ring_result.phase!=RING_CIRCLE && ring_result.phase!=RING_EXIT && ring_result.phase!=RING_RECOVER);
    CHECK(!ring_result.exited_events);
}

/* The mouth history comes from the unchanged fixed-world map. Stop at
 * the first readiness observation, before its required consecutive run. */
static int approach_to_pending(int side)
{
    int i;reset();
    for(i=0;i<=125;++i) {
        world(side,rtg_approach(i),20);ordinary_output();
        if(ring_result.phase==RING_APPROACH && ring_result.mouth_far_count>0) {
            CHECK(ring_result.approach_stage==RING_TANGENT_NEAR_PASSED);
            CHECK(ring_result.mouth_far_count<RING_TANGENT_CONFIRM);
            return i;
        }
    }
    CHECK(0);return 0;
}
static float approach_to_enter(int side)
{
    int i;reset();
    for(i=0;i<=125;++i) {
        rtg_pose_t p=rtg_approach(i);world(side,p,20);
        if(ring_result.phase==RING_ENTER) {
            CHECK(p.y>=RTG_FAR_MIN_WORLD && p.y<=RTG_FAR_MAX_WORLD);
            CHECK(ring_result.detected_events==1 && ring_owns_path());no_departure();
            return p.y;
        }
        ordinary_output();
    }
    CHECK(0);return 0;
}

static void cross_breaks_confirmation(int side)
{
    int i;label="cross breaks pending mouth confirmation";i=approach_to_pending(side);
    rtg_cross(image);motion.wheel_revs+=.01f;observe(20,2);
    CHECK(ring_result.cross_veto && (ring_result.mouth_gate&RING_MOUTH_CROSS));
    CHECK(ring_result.mouth_far_count==0);ordinary_output();
    /* Resuming genuine geometry requires new confirmation; a cross frame
     * cannot stand in for the middle of three continuous observations. */
    world(side,rtg_approach(i+1),20);
    CHECK(ring_result.mouth_far_count<RING_TANGENT_CONFIRM);ordinary_output();
}
static void unknown_breaks_confirmation(int side)
{
    int i;label="unknown visual clock breaks pending mouth confirmation";i=approach_to_pending(side);
    world(side,rtg_approach(i+1),65535u);
    CHECK(ring_result.mouth_gate&RING_MOUTH_CLOCK);
    CHECK(ring_result.mouth_far_count==0);ordinary_output();
    world(side,rtg_approach(i+2),20);
    CHECK(ring_result.mouth_far_count<RING_TANGENT_CONFIRM);ordinary_output();
}
static void sensor_jump_breaks_confirmation(int side)
{
    int i;label="sensor jump breaks pending mouth confirmation";i=approach_to_pending(side);
    rtg_render(image,side,&(rtg_pose_t){0,-180.f+2.f*(i+1),0});
    motion.wheel_revs+=.5f;observe(20,1);
    CHECK(ring_result.mouth_gate&RING_MOUTH_MOTION);
    CHECK(ring_result.mouth_far_count==0);ordinary_output();
}
static void frozen_pending_is_not_progress(int side,int sensor_motion)
{
    int i,first_y;label="exact frozen mouth cannot finish visual confirmation";
    (void)approach_to_pending(side);first_y=ring_result.mouth_first_y;
    for(i=0;i<120;++i) {
        if(sensor_motion){motion.wheel_revs+=.025f;motion.yaw_deg+=side*.5f;}
        observe(20,1);
        CHECK(ring_result.mouth_gate&RING_MOUTH_VISUAL_PROGRESS);
        CHECK(ring_result.mouth_far_count==0);
        CHECK(ring_result.mouth_first_y==first_y);ordinary_output();
    }
}
static void no_wheel_progress_reports_motion_gate(int side)
{
    int i,saw_passed=0;float frozen_revs=0;
    label="missing post-N displacement is visible as MOTION gate";reset();
    for(i=0;i<=125;++i) {
        rtg_pose_t p=rtg_approach(i);
        if(!saw_passed)world(side,p,20);
        else {rtg_render(image,side,&p);motion.wheel_revs=frozen_revs;observe(20,1);last=p;}
        if(ring_result.approach_stage==RING_TANGENT_NEAR_PASSED && !saw_passed) {
            saw_passed=1;frozen_revs=motion.wheel_revs;
        }
        if(saw_passed)CHECK(ring_result.mouth_gate&RING_MOUTH_MOTION);
        ordinary_output();
    }
    CHECK(saw_passed);
}

/* This starts from a genuine measured F entry, not from cold NORMAL.
 * A legal through-road image is still a different branch. Neither 45
 * degrees held for many frames nor continuing synthetic yaw may convert
 * that ordinary image into a ring branch or a CIRCLE commitment. */
static void straight_after_enter_cannot_commit(int side,int mode)
{
    int i;label="ENTER then ordinary straight + yaw is not a branch";
    (void)approach_to_enter(side);rtg_straight(image);
    for(i=0;i<80;++i) {
        if(mode==0)motion.yaw_deg=side*45.f;
        else if(mode==1)motion.yaw_deg=side*(float)(i<9?(i+1)*5:45);
        else motion.yaw_deg+=side*5.f;
        motion.wheel_revs+=.025f;observe(20,1);
        CHECK(!ring_result.branch_ok);no_departure();
    }
}
static void first_inner_frozen_cannot_commit(int side)
{
    int i,saw_inner=0;float far_y;
    label="first measured near island frozen + yaw cannot commit";
    far_y=approach_to_enter(side);
    for(i=1;i<=RTG_JOIN_STEPS;++i) {
        world(side,rtg_join_at(side,i,far_y),20);
        if(ring_result.branch_ok) {
            CHECK(!ring_result.committed && ring_result.phase==RING_ENTER);
            saw_inner=1;break;
        }
    }
    CHECK(saw_inner);
    for(i=0;i<80;++i) {
        motion.yaw_deg+=side*3.f;motion.wheel_revs+=.025f;observe(20,1);
        CHECK(!ring_result.committed && ring_result.phase!=RING_CIRCLE);
        CHECK(ring_result.phase!=RING_EXIT && !ring_result.exited_events);
    }
}

int main(void)
{
    int side,mode;CHECK(RING_TRACK_MODE==1);
    for(side=-1;side<=1;side+=2) {
        cross_breaks_confirmation(side);unknown_breaks_confirmation(side);
        sensor_jump_breaks_confirmation(side);
        frozen_pending_is_not_progress(side,0);frozen_pending_is_not_progress(side,1);
        no_wheel_progress_reports_motion_gate(side);
        for(mode=0;mode<3;++mode)straight_after_enter_cannot_commit(side,mode);
        first_inner_frozen_cannot_commit(side);
    }
    printf("PASS: %u observed-mouth frames; cross/clock/jump break confirmation, frozen mouth cannot progress, ENTER straight+45deg/yaw cannot commit\n",frames);
    return 0;
}
