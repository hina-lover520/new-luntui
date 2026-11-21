/* Retain the earlier exit suite and add the actual long-yaw visibility gap. */
#define main v34_existing_suite_main
#include "test_ring_v34_exit_vision.c"
#undef main

static void rotated_out_then_current_road(int side)
{
    early_corridor(side,315.f);
    active_step(319.f,1,1);
    CHECK(ring_result.path_valid);
    active_step(330.f,1,0);
    active_step(350.f,1,0);
    CHECK(ring_result.phase==RING_EXIT && !ring_result.path_valid);
    CHECK(!ring_result.release_reason && !ring_result.exit_confirm_frames);
    active_step(360.1f,2,1);
    CHECK(ring_result.phase==RING_EXIT && ring_result.path_valid);
    CHECK(!ring_result.release_reason && ring_result.exit_confirm_frames==1);
    /* Already supplied the first current measured confirmation frame. */
    stable_count=0;
    complete_handover(360.1f);
}

static void wrong_heading_cannot_reacquire(int side)
{
    island_t island;
    early_corridor(side,315.f);
    active_step(330.f,1,0);
    motion.now_ms+=50;motion.wheel_revs+=.03f;motion.yaw_deg=side*360.1f;
    ring_result.visual_dt_ms=50;vision_ms+=50;
    road(side*28.f);scan_geometry((const uint8_t(*)[188])image,&motion);
    island=find_island();ring_observe_exit_turn_feedback(1);
    tangent_active(&island,&motion,0);
    CHECK(!ring_result.path_valid && !ring_result.exit_confirm_frames);
    CHECK(!ring_result.exited_events && ring_result.phase==RING_EXIT);
}

int main(void)
{
    int side;
    v34_existing_suite_main();
    for(side=-1;side<=1;side+=2){
        rotated_out_then_current_road(side);
        wrong_heading_cannot_reacquire(side);
    }
    puts("PASS v35: current outgoing road reacquired after >30deg occlusion; wrong heading denied, both sides");
    return 0;
}
