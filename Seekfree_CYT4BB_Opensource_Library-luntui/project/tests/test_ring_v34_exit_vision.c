/* Prepared-state geometry tests. Real movement is covered separately by
 * the runtime gyro test; these pictures do not claim a driven full lap. */
#include "../code/ring.c"
#include <stdio.h>
static uint8_t image[120][188];
static ring_motion_t motion;
static int test_side;
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL v34 exit vision:%d %s side%d phase%d lap%.2f\n",__LINE__,#c,test_side,ring_result.phase,ring_result.lap_deg);exit(1);}}while(0)
static void road(float relative_angle)
{
    int x,y;float a=relative_angle*.01745329252f;
    memset(image,0,sizeof(image));
    for(y=0;y<120;++y)for(x=0;x<188;++x){
        float dx=(x-94.f)*cosf(a)-(108.f-y)*sinf(a);
        if(fabsf(dx)<=29.5f)image[y][x]=255;
    }
}
static void prepare(int side,float progress)
{
    int y;test_side=side;ring_init();
    motion=(ring_motion_t){1000,side*progress,1.f,1};
    ring_result.phase=RING_CIRCLE;ring_result.route=(int8_t)-side;
    ring_result.ring_side=(int8_t)side;ring_result.entry_side=(int8_t)side;
    ring_result.committed=ring_result.left_entry=ring_result.lap_valid=1;
    ring_result.lap_deg=progress;ring_result.lane_width=60;
    maneuver_feedback=3;timed_started=1;lap_completed=progress>=360.f;
    have_path=1;last_path_ms=vision_ms;last_path_fault_ms=motion.now_ms;
    tangent_lap_revs=0;ring_since=phase_since=motion.now_ms;
    for(y=0;y<120;++y)ring_result.mid[y]=94;
}
static int observe(float angle)
{
    motion.now_ms+=50;motion.wheel_revs+=.01f;
    ring_result.visual_dt_ms=50;++ring_result.frame_sequence;vision_ms+=50;
    road(angle);scan_geometry((const uint8_t(*)[188])image,&motion);
    return tangent_exit_observe(&motion);
}
static void early_corridor(int side,float progress)
{
    int i;prepare(side,progress);
    for(i=0;i<6 && ring_result.phase==RING_CIRCLE;++i)
        CHECK(observe(side*(360.f-progress)));
    CHECK(ring_result.phase==RING_EXIT && ring_result.exit_latched);
    CHECK(ring_result.exit_turn_request && !ring_result.exited_events);
    CHECK(!ring_result.exit_confirmed && !lap_completed);
    /* A prepared controller completion cannot turn an incomplete net lap
     * into success. Formal handover still needs current360 and road. */
    ring_observe_exit_turn_feedback(2);
    CHECK(ring_result.lap_deg<360.f && !ring_result.exited_events);
    ring_observe_exit_turn_feedback(1);
    motion.yaw_deg=side*(progress+4.f);ring_result.lap_deg=progress+4.f;
    motion.now_ms+=50;motion.wheel_revs+=.01f;
    ring_result.visual_dt_ms=50;
    road(side*(356.f-progress));scan_geometry((const uint8_t(*)[188])image,&motion);
    CHECK(tangent_exit_continue(&motion));
    CHECK(ring_result.phase==RING_EXIT && !ring_result.exited_events);
}
static void mismatch_rejected(int side,float progress,float road_angle)
{
    int i;prepare(side,progress);
    for(i=0;i<6;++i)(void)observe(road_angle);
    CHECK(ring_result.phase==RING_CIRCLE && !ring_result.exit_turn_request);
    CHECK(!ring_result.exit_latched && !ring_result.exited_events);
}
static void no_yaw_only_exit(int side)
{
    int i;prepare(side,330.f);ring_result.committed=0;
    for(i=0;i<6;++i)(void)observe(side*30.f);
    CHECK(!ring_result.exit_latched && !ring_result.exited_events);
    prepare(side,330.f);
    for(i=0;i<6;++i){
        motion.now_ms+=50;ring_result.visual_dt_ms=50;
        memset(image,0,sizeof(image));scan_geometry((const uint8_t(*)[188])image,&motion);
        (void)tangent_exit_observe(&motion);
    }
    CHECK(!ring_result.exit_latched && !ring_result.exited_events);
}
/* These steps execute the complete visual EXIT/RECOVER state machine. The
 * controller feedback is supplied explicitly; a yaw sample alone does not
 * certify that the finite exit turn has finished. */
static void active_step(float progress,unsigned feedback,int visible)
{
    island_t island;
    motion.now_ms+=50;motion.wheel_revs+=.03f;
    motion.yaw_deg=test_side*progress;
    ring_result.visual_dt_ms=50;++ring_result.frame_sequence;vision_ms+=50;
    if(visible)road(test_side*(360.f-progress));
    else memset(image,0,sizeof(image));
    scan_geometry((const uint8_t(*)[188])image,&motion);island=find_island();
    ring_observe_exit_turn_feedback(feedback);ring_result.reject=0;
    tangent_active(&island,&motion,0);
}
static void complete_handover(float progress)
{
    int i;
    for(i=0;i<RING_RECOVER_FRAMES;++i){
        active_step(progress,2,1);
        CHECK(ring_result.phase==(i+1<RING_RECOVER_FRAMES?RING_EXIT:RING_RECOVER));
        CHECK(!ring_result.exited_events && ring_result.exit_turn_request);
        CHECK(!ring_result.release_reason && ring_result.path_valid);
    }
    for(i=0;i<RING_RECOVER_FRAMES;++i){
        active_step(progress,2,1);
        CHECK(ring_result.phase==(i+1<RING_RECOVER_FRAMES?RING_RECOVER:RING_COOLDOWN));
        CHECK(!ring_result.release_reason);
    }
    CHECK(ring_result.exited_events==1 && ring_result.exit_confirmed);
    CHECK(!ring_result.exit_turn_request && !ring_result.timed_entry);
    CHECK(!ring_owns_path() && !ring_blocks_cross() && ring_speed_limit()==-1);
    /* Completion must stop sending the exit command, and the normal public
     * update must leave cooldown after its existing time/travel conditions. */
    motion.now_ms+=RING_COOLDOWN_MS+1;motion.wheel_revs+=RING_COOLDOWN_REVS+.1f;
    road(0);ring_observe_frame_interval(50);
    ring_update((const uint8_t(*)[188])image,&motion);
    CHECK(ring_result.phase==RING_NORMAL && ring_result.exited_events==1);
    CHECK(!ring_result.exit_turn_request && !ring_owns_path());
}
static void full_exit_with_temporary_missing_road(int side)
{
    float p;
    early_corridor(side,315.f);
    active_step(319.f,0,0);
    CHECK(ring_result.phase==RING_EXIT && ring_result.exit_turn_request);
    CHECK(!ring_result.path_valid && !ring_result.branch_ok);
    CHECK(!ring_result.release_reason && !ring_result.exit_confirm_frames);
    active_step(324.f,1,0);
    CHECK(ring_result.phase==RING_EXIT && ring_result.exit_turn_request);
    CHECK(!ring_result.path_valid && !ring_result.release_reason);
    for(p=329.f;p<=359.f;p+=5.f){
        active_step(p,1,1);
        CHECK(ring_result.phase==RING_EXIT && !ring_result.exited_events);
        CHECK(ring_result.path_valid && !ring_result.release_reason);
    }
    active_step(360.1f,1,1);
    CHECK(lap_completed && !ring_result.exit_confirm_frames);
    CHECK(ring_result.phase==RING_EXIT && !ring_result.exited_events);
    complete_handover(360.1f);
}
static void latched_exit_beyond_old_window(int side)
{
    early_corridor(side,350.f);
    active_step(360.1f,1,1);
    CHECK(lap_completed && !ring_result.exit_confirm_frames);
    active_step(376.f,1,1);
    CHECK(ring_result.phase==RING_EXIT && ring_result.path_valid);
    CHECK(ring_result.exit_latched && !ring_result.release_reason);
    complete_handover(376.f);
}
static void completion_feedback_allows_small_return(int side)
{
    int i;
    early_corridor(side,350.f);
    active_step(360.1f,1,1);
    CHECK(lap_completed);
    for(i=0;i<RING_RECOVER_FRAMES+1;++i){
        active_step(359.5f,1,1);
        CHECK(ring_result.phase==RING_EXIT && !ring_result.exited_events);
        CHECK(!ring_result.exit_confirm_frames && !ring_result.release_reason);
    }
    complete_handover(359.5f);
}
int main(void)
{
    int side;
    for(side=-1;side<=1;side+=2){
        early_corridor(side,315.f);early_corridor(side,330.f);early_corridor(side,350.f);
        mismatch_rejected(side,330.f,0.f);
        mismatch_rejected(side,330.f,-side*30.f);
        mismatch_rejected(side,270.f,side*30.f);
        no_yaw_only_exit(side);
        full_exit_with_temporary_missing_road(side);
        latched_exit_beyond_old_window(side);
        completion_feedback_allows_small_return(side);
    }
    puts("PASS: mirrored measured exit corridor, bounded missing-road feedback0/1, complete EXIT/RECOVER,376deg latch and359.5deg completed-feedback handover; request clears and cooldown releases; no yaw-only exit");
    return 0;
}
