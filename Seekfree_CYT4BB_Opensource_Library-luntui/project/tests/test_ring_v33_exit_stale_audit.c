/* Focused production integration regression: an unknown capture interval
 * breaks an unlatched exit candidate streak. No vehicle trajectory claim. */
#define main existing_v33_exit_test_main
#include "test_ring_v33_exit.c"
#undef main

int main(void)
{
    island_t none={0};int i;
    test_name="unknown capture interval breaks candidate streak";
    prepare(-1,360);
    for(i=0;i<2;++i){
        advance();exit_picture(0,0);run_geometry();
        CHECK(tangent_exit_observe(&motion));
        CHECK(ring_result.phase==RING_CIRCLE && !ring_result.exit_latched);
    }
    CHECK(exit_candidate_count==2);
    advance();exit_picture(0,0);run_geometry();ring_result.visual_dt_ms=65535u;
    tangent_active(&none,&motion,0);
    CHECK(ring_result.phase==RING_CIRCLE && !ring_result.exit_latched);
    CHECK(exit_candidate_count==0 && !ring_result.exit_candidate);
    advance();exit_picture(0,0);run_geometry();
    CHECK(tangent_exit_observe(&motion));
    CHECK(exit_candidate_count==1);
    CHECK(ring_result.phase==RING_CIRCLE && !ring_result.exit_latched);
    test_name="long fresh-frame gap starts new exit evidence";
    prepare(-1,360);
    for(i=0;i<2;++i){
        advance();exit_picture(0,0);run_geometry();CHECK(tangent_exit_observe(&motion));
    }
    motion.now_ms+=RING_PATH_HOLD_MS+1;motion.wheel_revs+=.01f;
    exit_picture(0,0);run_geometry();CHECK(tangent_exit_observe(&motion));
    CHECK(exit_candidate_count==1);
    CHECK(ring_result.phase==RING_CIRCLE && !ring_result.exit_latched);
    puts("PASS: unknown capture interval and long fresh-frame gap clear unlatched exit evidence");
    return 0;
}
