/* Actual default400 vision -> publisher -> control integration. Initial
 * recognition/committed entry is prepared; no vehicle dynamics claim. */
#define main historical_episode_suite
#include "test_ring_v35_episode_integration.c"
#undef main

static void actual400_episode(int side)
{
    unsigned n,seen_recover=0;
    prepare75(side,1,1);memset(camera,0,sizeof(camera));
    while(diag()->maneuver_turn10<4002) {
        int raw=diag()->maneuver_turn10<3980?1000:100;
        ticks(side*raw,20u);revs+=.03f;frame();
        VERIFY(ring_result.phase==RING_CIRCLE && diag()->active &&
            !ring_result.exited_events && !ring_result.exit_latched);
        VERIFY(track==1 && error==12 && turn==normal_turn && speed>90.f);
    }
    ticks(0,20u);revs+=.03f;frame();
    VERIFY(exit_net_reached && ring_result.lap_deg>=400.f);
    for(n=0;n<12u && ring_result.phase!=RING_EXIT;++n) {
        ticks(0,20u);revs+=.03f;
        road(side*(360.f-diag()->maneuver_turn10*.1f));frame();
    }
    VERIFY(ring_result.phase==RING_EXIT && ring_result.exit_latched &&
        ring_result.exit_turn_request && diag()->exit_force_stage==1u &&
        diag()->exit_force_side==-side && diag()->active);
    for(n=0;n<50u && diag()->exit_force_stage==1u;++n) {
        ticks(-side*1000,20u);revs+=.03f;
        road(side*(360.f-diag()->maneuver_turn10*.1f));frame();
        VERIFY(diag()->active && !ring_result.exited_events && speed>0.f);
    }
    VERIFY(diag()->exit_force_stage==2u &&
        abs(diag()->maneuver_turn10-3600)<=80);
    for(n=0;n<45u && !ring_result.exited_events;++n) {
        ticks(0,20u);revs+=.03f;
        road(side*(360.f-diag()->maneuver_turn10*.1f));frame();
        VERIFY(speed>0.f && !ring_result.release_reason);
        if(ring_result.phase==RING_RECOVER)seen_recover=1;
    }
    VERIFY(seen_recover && ring_result.exited_events==1u &&
        ring_result.phase==RING_COOLDOWN && ring_result.exit_confirmed &&
        diag()->exited==1u && !diag()->active && turn==normal_turn);
    ticks(0,600u);apply();VERIFY(pulses==6u);
}
int main(void)
{
    VERIFY(RING_EXIT_NET_MIN_DEG==400.f);
    actual400_episode(-1);actual400_episode(1);
    puts("PASS: v36 actualdefault400 bothcore vision/publish/control, uncapped observation, current400 exit locks reverse finite correction to original360, current-road recovery and fast1/3/2 exactly once; prepared entry, synthetic camera and gyro, not field proof");
    return 0;
}
