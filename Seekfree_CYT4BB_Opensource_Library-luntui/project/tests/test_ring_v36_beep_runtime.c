/* Actual mailbox/runtime events and gyro history. Beeps are counted from
 * existing1ms callback, never inferred from optional UART records. */
#define main inherited_v33_runtime_main
#include "test_ring_v33_runtime.c"
#undef main

static void event_command(int phase,unsigned exited,int requested,int path)
{
    ring_packet_t p={{0}};
    int owned=phase>=RING_APPROACH && phase<=RING_RECOVER;
    p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)epoch;
    p.word[RC_PHASE]=ring_command_versioned_phase(phase,0,RING_PAIR_VERSION);
    p.word[RC_CAP]=ring_command_cap_turn(owned?90:-1,diag()->maneuver_turn10/10);
    p.word[RC_DETECTED]=(int32_t)detected;p.word[RC_EXITED]=(int32_t)exited;
    p.word[RC_FAULT]=ring_command_diagnostics(0,1,0,1,path,0,1,0,80,-side);
    if(requested)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_EXIT_TURN);
    ring_link_write(ring_command_mailbox,&p,&sequence);apply();
}
static void found_and_finish(int direction,int successful)
{
    unsigned n;
    boot(direction);advance(4002u,RING_ENTER);
    CHECK(beeps==1u && diag()->maneuver_stage==3u);
    event_command(RING_EXIT,0,1,1);
    CHECK(diag()->active && diag()->phase==RING_EXIT && diag()->exit_force_stage==1u);
    if(!successful) {
        /* An accepted exit-found event may be followed by failed release.
         * It may finish its triple but must never fabricate success tones. */
        event_command(RING_COOLDOWN,0,0,0);
        CHECK(!diag()->active && !diag()->exited);
        tick(0,800u,1);CHECK(beeps==4u);
        event_command(RING_COOLDOWN,0,0,0);tick(0,300u,1);CHECK(beeps==4u);
        return;
    }
    for(n=0;n<40u && diag()->exit_force_stage==1u;++n) {
        tick(-side*1000,20u,1);revs+=.03f;event_command(RING_EXIT,0,1,0);
        CHECK(diag()->active && !diag()->exited && beeps<=4u);
    }
    CHECK(diag()->exit_force_stage==2u && beeps==4u);
    for(n=0;n<10u;++n) {
        tick(0,20u,1);revs+=.03f;event_command(RING_EXIT,0,1,1);
    }
    CHECK(beeps==4u && !diag()->exited);
    event_command(RING_RECOVER,0,1,1);
    CHECK(diag()->active && diag()->phase==RING_RECOVER);
    event_command(RING_COOLDOWN,1,0,1);
    CHECK(!diag()->active && diag()->exited==1u);
    tick(0,300u,1);CHECK(beeps==6u);
    for(n=0;n<10u;++n) {
        event_command(RING_COOLDOWN,1,0,1);tick(0,20u,1);
    }
    CHECK(beeps==6u); /* Repeated packets cannot queue another triple/pair. */
}
static void aligned_exit_still_announces(int direction)
{
    unsigned n;
    boot(direction);advance(4002u,RING_ENTER);
    for(n=0;n<20u;++n) {
        tick(-side*1000,20u,1);revs+=.03f;command(RING_ENTER,0);
    }
    CHECK(diag()->maneuver_turn10>=3600 && diag()->maneuver_turn10<3610);
    event_command(RING_EXIT,0,0,1);
    CHECK(diag()->active && diag()->phase==RING_EXIT && !diag()->exit_force_stage);
    for(n=0;n<20u;++n) {
        tick(0,20u,1);revs+=.03f;event_command(RING_EXIT,0,0,1);
    }
    CHECK(beeps==4u && !diag()->exited);
    event_command(RING_RECOVER,0,0,1);
    CHECK(diag()->active && diag()->phase==RING_RECOVER);
    event_command(RING_COOLDOWN,1,0,1);tick(0,300u,1);
    CHECK(!diag()->active && diag()->exited==1u && beeps==6u);
}
int main(void)
{
    found_and_finish(-1,1);found_and_finish(1,1);
    found_and_finish(-1,0);found_and_finish(1,0);
    aligned_exit_still_announces(-1);aligned_exit_still_announces(1);
    puts("PASS: v36 real runtime entryfast1 + acceptedEXITfast3 + successfulENDfast2 exactly once, repeated packets silent, failed release never success-beeps, both sides");
    return 0;
}

