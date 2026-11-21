/* Production default net400 gate exercised through integrated gyro and
 * real control mailboxes. These tests do not model vehicle dynamics. */
#define main inherited_gate400_v33_main
#include "test_ring_v33_runtime.c"
#undef main

static void exit400_command(int phase,unsigned exited,int force,int advertised,
    uint32_t event,int route,int path)
{
    ring_packet_t p={{0}};
    int owned=phase>=RING_APPROACH && phase<=RING_RECOVER;
    p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)epoch;
    p.word[RC_PHASE]=ring_command_versioned_phase(phase,0,RING_PAIR_VERSION);
    p.word[RC_CAP]=ring_command_cap_turn(owned?90:-1,advertised);
    p.word[RC_DETECTED]=(int32_t)event;p.word[RC_EXITED]=(int32_t)exited;
    p.word[RC_FAULT]=ring_command_diagnostics(0,1,0,1,path,0,1,0,80,route);
    if(force)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_EXIT_TURN);
    ring_link_write(ring_command_mailbox,&p,&sequence);apply();
}
static void exit400(int phase,unsigned exited,int force)
{exit400_command(phase,exited,force,430,detected,-side,1);}

static void reject_before_net400(int direction,unsigned progress_ms)
{
    uint32_t accepted;
    boot(direction);advance(progress_ms,RING_ENTER);accepted=diag()->command_sequence;
    /* Even an advertised430-degree packet cannot replace integrated yaw. */
    exit400(RING_EXIT,0,1);
    CHECK(diag()->command_sequence==accepted && !diag()->exit_force_stage);
    exit400(RING_RECOVER,0,1);exit400(RING_COOLDOWN,1,0);
    CHECK(diag()->command_sequence==accepted && !diag()->exited && diag()->active);
}
static void absolute_travel_is_not_net(int direction)
{
    unsigned n;uint32_t accepted;
    boot(direction);advance(3600u,RING_ENTER);
    for(n=0;n<50u;++n){tick(-side*1000,20u,1);revs+=.03f;command(RING_ENTER,0);}
    advance(1000u,RING_ENTER);
    CHECK(diag()->maneuver_turn10>=3599 && diag()->maneuver_turn10<=3601);
    accepted=diag()->command_sequence;exit400(RING_EXIT,0,1);
    CHECK(diag()->command_sequence==accepted && !diag()->exit_force_stage && !diag()->exited);
}
static void complete_net400(int direction,unsigned angle_ms,int align_first)
{
    unsigned n;uint32_t accepted;
    boot(direction);advance(angle_ms,RING_ENTER);
    CHECK(diag()->maneuver_turn10>=4000 && diag()->maneuver_turn10<=4300);
    if(align_first) {
        while(diag()->maneuver_turn10>3601) {
            tick(-side*1000,1u,1);revs+=.0015f;command(RING_ENTER,0);
        }
        CHECK(diag()->maneuver_turn10>=3599);
        exit400(RING_EXIT,0,0);
        CHECK(diag()->active && diag()->phase==RING_EXIT && !diag()->exit_force_stage);
    } else {
        exit400(RING_EXIT,0,1);
        CHECK(diag()->active && diag()->phase==RING_EXIT && diag()->exit_force_stage==1u &&
            diag()->exit_force_side==-side && diag()->exit_force_remaining10<=700);
        accepted=diag()->command_sequence;
        exit400(RING_RECOVER,0,1);
        CHECK(diag()->command_sequence==accepted && !diag()->exited);
        for(n=0;n<75u && diag()->exit_force_stage==1u;++n) {
            tick(-side*1000,10u,1);revs+=.015f;exit400(RING_EXIT,0,1);
            CHECK(diag()->active && speed>0.f && !diag()->exited);
        }
        CHECK(diag()->exit_force_stage==2u && diag()->maneuver_turn10>=3580 &&
            diag()->maneuver_turn10<=3600);
    }
    /* Historical400 remains true after returning to the straight heading. */
    tick(0,320u,1);exit400(RING_EXIT,0,!align_first);
    exit400(RING_RECOVER,0,!align_first);
    CHECK(diag()->active && diag()->phase==RING_RECOVER);
    ordinary_turn=-37.f;exit400(RING_COOLDOWN,1,0);
    CHECK(!diag()->active && diag()->exited==1u && turn==-37.f && speed>0.f);
}
static void reject_identity_and_bounds(int direction,int kind)
{
    uint32_t accepted;
    boot(direction);advance(kind==0?4305u:4002u,RING_ENTER);
    accepted=diag()->command_sequence;
    if(kind==0)exit400(RING_EXIT,0,1);
    else if(kind==1)exit400_command(RING_EXIT,0,1,430,detected+1u,-side,1);
    else if(kind==2)exit400_command(RING_EXIT,0,1,430,detected,side,1);
    else exit400(RING_RECOVER,0,1);
    CHECK(diag()->command_sequence==accepted && !diag()->exit_force_stage && !diag()->exited);
}
static void gate_resets_between_events(int direction)
{
    uint32_t accepted;
    complete_net400(direction,4002u,0);
    boot(direction);advance(3602u,RING_ENTER);accepted=diag()->command_sequence;
    exit400(RING_EXIT,0,1);
    CHECK(diag()->command_sequence==accepted && !diag()->exit_force_stage && !diag()->exited);
}
static void reject_unaligned_no_force(int direction)
{
    uint32_t accepted;
    boot(direction);advance(4102u,RING_ENTER);accepted=diag()->command_sequence;
    exit400(RING_EXIT,0,0);exit400(RING_COOLDOWN,1,0);
    CHECK(diag()->command_sequence==accepted && !diag()->exited &&
        diag()->active && !diag()->exit_force_stage);
}
static void endpoint46ms(int direction)
{
    unsigned n;
    boot(direction);advance(4002u,RING_ENTER);exit400(RING_EXIT,0,1);
    for(n=0;n<18u;++n) {
        tick(-side*1000,20u,1);revs+=.03f;exit400(RING_EXIT,0,1);
        CHECK(diag()->active && diag()->exit_force_stage==1u);
    }
    tick(-side*1000,17u,1);revs+=.0255f;exit400(RING_EXIT,0,1);
    CHECK(diag()->maneuver_turn10>=3624 && diag()->maneuver_turn10<=3626);
    tick(-side*1000,46u,1);revs+=.069f;exit400(RING_EXIT,0,1);
    CHECK(diag()->active && diag()->exit_force_stage==2u &&
        diag()->maneuver_turn10>=3578 && diag()->maneuver_turn10<=3580);
    tick(0,320u,1);exit400(RING_EXIT,0,1);exit400(RING_RECOVER,0,1);
    CHECK(diag()->active && diag()->phase==RING_RECOVER);
    exit400(RING_COOLDOWN,1,0);CHECK(!diag()->active && diag()->exited==1u);
}
static void speed_packet(int cap,int path,int exit_request,float requested)
{
    ring_packet_t p={{0}};
    p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)epoch;
    p.word[RC_PHASE]=ring_command_versioned_phase(exit_request?RING_EXIT:RING_CIRCLE,0,RING_PAIR_VERSION);
    p.word[RC_CAP]=ring_command_cap_turn(cap,diag()->maneuver_turn10/10);
    p.word[RC_DETECTED]=(int32_t)detected;
    p.word[RC_FAULT]=ring_command_diagnostics(0,0,0,1,path,0,1,0,80,-side);
    if(exit_request)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_EXIT_TURN);
    ring_link_write(ring_command_mailbox,&p,&sequence);
    speed=requested;turn=ordinary_turn;ring_runtime_observe_motion(100.f,0,0,0,0);
    ring_runtime_apply(&speed,&turn,revs,3);
}
static void observation_cap_reacquisition(int direction,int exiting)
{
    unsigned n;float previous;
    boot(direction);advance(4002u,RING_ENTER);
    for(n=0;n<150u;++n) {
        tick(0,20u,1);revs+=.03f;speed_packet(-1,0,0,185.f);
        CHECK(diag()->active && diag()->cap==-1 && !diag()->held_target10);
    }
    CHECK(speed>184.9f);previous=speed;
    speed_packet(90,1,exiting,185.f);
    CHECK(diag()->active && diag()->cap==90 && fabsf(speed-previous)<.01f);
    CHECK(exiting?diag()->exit_force_stage==1u:diag()->phase==RING_CIRCLE);
    for(n=0;n<(exiting?16u:150u);++n) {
        tick(0,20u,1);revs+=.03f;speed_packet(90,1,exiting,185.f);
        CHECK(diag()->active && speed<=previous+.001f && speed>=previous-1.601f);
        previous=speed;
    }
    CHECK(exiting?(speed<184.f):(speed<=90.001f));
    CHECK(exiting?(diag()->speed_goal10<1850 && diag()->speed_goal10>=900):
        diag()->speed_goal10==900);
}
int main(void)
{
    int direction,kind;
    CHECK(RING_EXIT_NET_MIN_DEG==400.f && RING_EXIT_SEARCH_END_DEG==430.f);
    for(direction=-1;direction<=1;direction+=2) {
        reject_before_net400(direction,3502u);reject_before_net400(direction,3602u);
        reject_before_net400(direction,3998u);absolute_travel_is_not_net(direction);
        complete_net400(direction,4002u,0);complete_net400(direction,4202u,0);
        complete_net400(direction,4298u,0);complete_net400(direction,4002u,1);
        for(kind=0;kind<4;++kind)reject_identity_and_bounds(direction,kind);
        gate_resets_between_events(direction);
        reject_unaligned_no_force(direction);endpoint46ms(direction);
        observation_cap_reacquisition(direction,0);observation_cap_reacquisition(direction,1);
    }
    puts("PASS: v36 production signed-net400 runtime gate both directions; packet yaw and cumulative travel cannot arm; actual400..430 unlocks bounded correction toward original360; aligned post400 departure,185RPM observe-to-cap reacquisition,46ms endpoint crossing, unaligned no-force rejection, identity/bounds and per-episode reset");
    return 0;
}
