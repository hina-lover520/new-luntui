/* Low net exit travel keeps RECOVER active after its visual blend finishes.
 * The original-controller weight must stay within the intended0..1 range. */
#define main existing_v33_exit_test_main
#include "test_ring_v33_exit.c"
#undef main

/* The side corridor has32px usable width and joins the selected corridor
 * through a mouth. It is therefore truly connected, not an unrelated blob. */
static void parallel_side_picture(int near)
{
    int y;exit_picture(0,0);
    if(near){
        for(y=78;y<=109;++y)white_run(y,8,39);
        for(y=110;y<=115;++y)white_run(y,8,123);
    } else {
        for(y=24;y<=65;++y)white_run(y,8,39);
        for(y=66;y<=73;++y)white_run(y,8,123);
    }
}

static void island_in_ordinary_horizon_delays_handover(int side)
{
    island_t none={0};int i;
    test_name="connected near side road blocks ordinary-controller handover";
    reach_exit(side,360,0);
    for(i=0;i<RING_RECOVER_FRAMES*3;++i){
        advance();parallel_side_picture(1);run_geometry();tangent_active(&none,&motion,0);
        CHECK(ring_result.phase==RING_EXIT && ring_result.exit_latched);
        CHECK(ring_result.path_valid && !tangent_exit_handover_ready());
        CHECK(stable_count==0 && !ring_result.exit_confirm_frames);
        CHECK(!ring_result.exit_confirmed && !ring_result.exited_events);
        CHECK(ring_blend_error(40)==ring_result.steer_error);
        check_probe_path(&exit_locked_path);
    }
    CHECK(motion.wheel_revs-exit_start_revs>RING_EXIT_CLEAR_REVS);
    test_name="only distant island allows ordinary-controller handover";
    for(i=0;i<RING_RECOVER_FRAMES;++i){
        advance();parallel_side_picture(0);run_geometry();tangent_active(&none,&motion,0);
        CHECK(tangent_exit_handover_ready());
        CHECK(!ring_result.exit_confirmed && !ring_result.exited_events);
        if(i+1<RING_RECOVER_FRAMES)CHECK(ring_result.phase==RING_EXIT);
        check_probe_path(&exit_locked_path);
    }
    CHECK(ring_result.phase==RING_RECOVER);
    for(i=0;i<RING_RECOVER_FRAMES;++i){
        advance();parallel_side_picture(0);run_geometry();tangent_active(&none,&motion,0);
        if(i+1<RING_RECOVER_FRAMES)CHECK(ring_result.phase==RING_RECOVER && !ring_result.exited_events);
    }
    CHECK(ring_result.phase==RING_COOLDOWN && ring_result.exited_events==1);
    CHECK(ring_result.exit_confirmed && !ring_owns_path());
}

int main(void)
{
    island_t none={0};int i,normal=40,blended;
    test_name="slow exit cannot extrapolate recovery blend";
    reach_exit(-1,360,0);
    for(i=0;i<RING_RECOVER_FRAMES*3;++i){
        ++test_frame;motion.now_ms+=30;motion.wheel_revs+=.001f;
        exit_picture(0,5);run_geometry();tangent_active(&none,&motion,0);
        CHECK(ring_result.phase==RING_EXIT || ring_result.phase==RING_RECOVER);
        CHECK(!ring_result.exited_events && !ring_result.exit_confirmed);
        if(ring_result.phase==RING_RECOVER){
            blended=ring_blend_error((int16_t)normal);
            CHECK(blended>=ring_result.steer_error && blended<=normal);
            CHECK(ring_turn_boost()>=0);
        }
    }
    CHECK(ring_result.phase==RING_RECOVER);
    CHECK(stable_count==RING_RECOVER_FRAMES);
    CHECK(ring_blend_error((int16_t)normal)==normal);
    CHECK(ring_turn_boost()==0);
    CHECK(motion.wheel_revs-exit_start_revs<RING_EXIT_CLEAR_REVS);
    motion.now_ms=exit_start_ms+RING_EXIT_TIMEOUT_MS+1;
    exit_picture(0,5);run_geometry();tangent_active(&none,&motion,0);
    CHECK(ring_result.phase==RING_COOLDOWN && ring_result.release_reason==3);
    CHECK(!ring_result.exited_events && !ring_result.exit_confirmed);
    island_in_ordinary_horizon_delays_handover(-1);
    island_in_ordinary_horizon_delays_handover(1);
    puts("PASS: slow RECOVER keeps bounded blend and shares the original4s exit deadline");
    puts("PASS: connected near side roads block ordinary-controller handover; distant islands permit both recovery phases, including mirrors");
    return 0;
}
