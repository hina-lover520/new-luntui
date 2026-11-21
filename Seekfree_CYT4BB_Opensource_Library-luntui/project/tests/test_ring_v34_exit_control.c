/* Real runtime/mailbox/gyro integration. These checks prove the bounded
 * final-lap request and control contract, not actual camera geometry. */
#define main inherited_v33_runtime_main
#include "test_ring_v33_runtime.c"
#undef main

static void force_command_full(int phase,int path,unsigned exited,int requested,int committed)
{
    ring_packet_t p={{0}};
    int owned=phase>=RING_APPROACH && phase<=RING_RECOVER;
    p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)epoch;
    p.word[RC_PHASE]=ring_command_versioned_phase(phase,0,RING_PAIR_VERSION);
    p.word[RC_CAP]=ring_command_cap_turn(owned?90:-1,diag()->maneuver_turn10/10);
    p.word[RC_DETECTED]=(int32_t)detected;p.word[RC_EXITED]=(int32_t)exited;
    p.word[RC_FAULT]=ring_command_diagnostics(0,1,0,committed,path,0,1,0,80,-side);
    if(requested)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_EXIT_TURN);
    ring_link_write(ring_command_mailbox,&p,&sequence);apply();
}
static void force_command(int phase,int path,unsigned exited,int requested)
{force_command_full(phase,path,exited,requested,1);}
static void forced_advance(unsigned milliseconds,int path)
{
    while(milliseconds) {
        unsigned n=milliseconds>20u?20u:milliseconds;
        tick(side*1000,n,1);revs+=.0015f*n;
        force_command(RING_EXIT,path,0,1);milliseconds-=n;
    }
}
static void force_start(int direction,unsigned turn_ms)
{
    boot(direction);advance(turn_ms,RING_ENTER);
    force_command(RING_EXIT,1,0,1);
    CHECK(diag()->active && diag()->phase==RING_EXIT && diag()->maneuver_stage==3u);
    CHECK(diag()->exit_force_stage==1u && diag()->force_active);
    CHECK(ring_motion_exit_stage(ring_motion_mailbox[RM_READY])==1u);
    CHECK(diag()->exit_force_remaining10>0 && diag()->exit_force_remaining10<=700);
}
static void complete(int direction,unsigned initial_ms)
{
    unsigned n,seen;
    uint32_t accepted;
    int initial;
    force_start(direction,initial_ms);initial=diag()->exit_force_remaining10;
    CHECK(abs(diag()->exit_force_start10-(int)initial_ms)<3);
    accepted=diag()->command_sequence;
    force_command(RING_RECOVER,1,0,1);
    CHECK(diag()->command_sequence==accepted && diag()->exit_force_stage==1u);
    force_command(RING_COOLDOWN,1,1,0);
    CHECK(diag()->command_sequence==accepted && !diag()->exited);
    /* A live finite yaw target survives cropped exit pixels. */
    for(n=0;n<3u;++n) {
        tick(0,20,1);revs+=.03f;force_command(RING_EXIT,0,0,1);
        CHECK(diag()->active && !diag()->path_valid && diag()->exit_force_stage==1u);
        CHECK(side*turn>0.f && speed>0.f);
    }
    measured=5;tick(0,1,1);force_command(RING_EXIT,0,0,1);
    CHECK(fabsf(turn)<=5.001f && speed>0.f && diag()->active);
    measured=100;
    for(n=0;n<45u && diag()->exit_force_stage==1u;++n)forced_advance(20u,1);
    CHECK(diag()->exit_force_stage==2u && diag()->maneuver_turn10>=3600);
    CHECK(ring_motion_exit_stage(ring_motion_mailbox[RM_READY])==2u);
    CHECK(diag()->exit_force_turn10<=initial+21 && diag()->exit_force_turn10<=720);
    CHECK(diag()->active && !diag()->force_active && !diag()->exited);
    tick(0,280,1);force_command(RING_EXIT,1,0,1);seen=beeps;
    CHECK(seen==4u); /* One fast entry pulse and three fast exit-found pulses. */
    force_command(RING_EXIT,1,0,1);tick(0,200,1);
    CHECK(beeps==seen);
    force_command(RING_RECOVER,1,0,1);
    CHECK(diag()->active && diag()->phase==RING_RECOVER);
    ordinary_turn=-37;force_command(RING_COOLDOWN,1,1,0);
    CHECK(!diag()->active && diag()->exited==1u && turn==-37.f);
    tick(0,200,1);CHECK(beeps==seen+2u);
    force_command(RING_COOLDOWN,1,1,0);tick(0,200,1);
    CHECK(beeps==seen+2u);
}
static void rejection(int direction,int kind)
{
    uint32_t accepted;
    boot(direction);advance(kind==0?2800u:2902u,RING_ENTER);
    accepted=diag()->command_sequence;
    if(kind==1)measured=0;
    if(kind==2)measured=-10;
    if(kind==3)observe=0;
    if(kind==3)tick(0,151,1);
    force_command_full(RING_EXIT,kind==4?0:1,0,kind==4?0:1,kind==5?0:1);
    CHECK(diag()->command_sequence==accepted && !diag()->exit_force_stage && !diag()->exited);
    tick(0,240,1);CHECK(beeps==1u);
}
static void interruption(int direction,int kind)
{
    unsigned n;
    force_start(direction,2902u);
    if(kind==0){ring_runtime_set_operator(0,0);apply();}
    else if(kind==1){tick(0,301,1);apply();}
    else if(kind==2){tick(-side*1000,40,1);revs+=.06f;force_command(RING_EXIT,0,0,1);}
    else if(kind==3){measured=0;tick(0,1,1);force_command(RING_EXIT,0,0,1);}
    else if(kind==4){revs+=3.6f;tick(0,20,1);force_command(RING_EXIT,0,0,1);}
    else if(kind==5) {
        for(n=0;n<110u && diag()->active;++n) {
            tick(0,20,1);revs+=.02f;force_command(RING_EXIT,0,0,1);
        }
    } else if(kind==6){detected++;force_command(RING_EXIT,1,0,1);}
    else {side=-side;force_command(RING_EXIT,1,0,1);}
    CHECK(!diag()->active && !diag()->exited && !ring_runtime_watchdog_stop());
    CHECK(speed>0.f); /* Ring failure never writes a stop target. */
}
static void already_aligned(int direction)
{
    boot(direction);advance(3601u,RING_ENTER);command(RING_EXIT,0);
    CHECK(diag()->active && !diag()->exit_force_stage && !diag()->force_active);
    tick(0,200,1);command(RING_EXIT,0);tick(0,140,1);
    CHECK(beeps==4u); /* Aligned actual EXIT announces three short pulses too. */
}
static void outer_diagnostic_isolation(void)
{
    ring_packet_t p={{0}};
    uint32_t source_sequence=0,accepted;
    unsigned n;
    boot(-1);command(RING_NORMAL,0);ordinary_turn=31;apply();
    accepted=diag()->command_sequence;
    p.word[RL_MAGIC]=RING_DIAGNOSTIC_MAGIC;p.word[RL_EPOCH]=(int32_t)epoch;
    p.word[RD_COMMAND_SEQUENCE]=(int32_t)accepted;p.word[RD_FRAME]=123;
    p.word[RD_TIMING_REJECT]=ring_diagnostic_timing(20u,0u);
    p.word[RD_FLAGS]=ring_diagnostic_flags(0,0,0,0,1,7,1,0);
    p.word[RD_POINTS]=ring_diagnostic_points(94,108);
    ring_link_write(ring_diagnostic_mailbox,&p,&source_sequence);apply();
    CHECK(diag()->diagnostic_valid && diag()->path_source==7u && diag()->branch_ok);
    CHECK(!diag()->active && turn==31.f && !diag()->maneuver_stage &&
        diag()->command_sequence==accepted && !diag()->command_age_ms);
    /* Fresh source7 packets never renew the command's age. */
    for(n=0;n<8u;++n) {
        tick(0,40,1);p.word[RD_FRAME]++;
        ring_link_write(ring_diagnostic_mailbox,&p,&source_sequence);apply();
        CHECK(diag()->diagnostic_valid && diag()->path_source==7u &&
            diag()->command_age_ms==(n+1u)*40u && !diag()->active && turn==31.f);
    }
    CHECK(diag()->command_age_ms>RING_LINK_TIMEOUT_MS && !ring_runtime_watchdog_stop());
    /* The same fresh optional source cannot prolong an owned entry. */
    boot(1);accepted=diag()->command_sequence;source_sequence=0;
    p.word[RL_EPOCH]=(int32_t)epoch;p.word[RD_COMMAND_SEQUENCE]=(int32_t)accepted;
    for(n=0;n<8u;++n) {
        tick(0,40,1);p.word[RD_FRAME]++;
        ring_link_write(ring_diagnostic_mailbox,&p,&source_sequence);apply();
        CHECK(diag()->command_sequence==accepted && diag()->command_age_ms==(n+1u)*40u);
    }
    CHECK(!diag()->active && !diag()->exited && !ring_runtime_watchdog_stop());
}
static void measured_full_lap_correction(int direction)
{
    uint32_t accepted;
    unsigned n;
    force_start(direction,3502u);
    forced_advance(95u,1);
    CHECK(diag()->maneuver_turn10>=3590 && diag()->maneuver_turn10<3600 &&
        diag()->exit_force_stage==1u);
    accepted=diag()->command_sequence;
    force_command(RING_RECOVER,1,0,1);
    CHECK(diag()->command_sequence==accepted && !diag()->exited);
    forced_advance(10u,1);
    CHECK(diag()->maneuver_turn10>=3600 && diag()->exit_force_stage==2u);
    /* A measured full lap remains proven after a small real straight-road
     * correction. Completion is not fabricated from the358-degree floor. */
    tick(-side*1000,12u,1);revs+=.02f;force_command(RING_EXIT,1,0,1);
    CHECK(diag()->active && diag()->exit_force_stage==2u &&
        diag()->maneuver_turn10>=3590 && diag()->maneuver_turn10<3600);
    force_command(RING_RECOVER,1,0,1);
    CHECK(diag()->active && diag()->phase==RING_RECOVER);
    force_command(RING_COOLDOWN,1,1,0);
    CHECK(!diag()->active && diag()->exited==1u);
    /* The correction allowance is finite, not a permission to reverse. */
    force_start(direction,3502u);
    for(n=0;n<7u && diag()->exit_force_stage==1u;++n)forced_advance(20u,1);
    CHECK(diag()->exit_force_stage==2u);
    tick(-side*1000,40u,1);revs+=.06f;force_command(RING_EXIT,1,0,1);
    CHECK(!diag()->active && !diag()->exited);
}
static void missed_initial_visual_packet(int direction)
{
    unsigned n;
    boot(direction);advance(3102u,RING_ENTER);
    /* The first path1 EXIT may be overwritten in the mailbox before the
     * controller polls. The current paired explicit request still carries
     * the vision core's finite, already-confirmed departure decision. */
    force_command(RING_EXIT,0,0,1);
    CHECK(diag()->active && diag()->phase==RING_EXIT && !diag()->path_valid &&
        diag()->exit_force_stage==1u && diag()->force_active);
    for(n=0;n<16u;++n) {
        tick(0,20u,1);revs+=.03f;force_command(RING_EXIT,0,0,1);
    }
    CHECK(diag()->active && diag()->exit_force_stage==1u && beeps==4u);
    force_command(RING_EXIT,0,0,1);tick(0,200u,1);force_command(RING_EXIT,0,0,1);
    CHECK(beeps==4u && diag()->active && !diag()->exited);
}
int main(void)
{
    int direction,kind;
    for(direction=-1;direction<=1;direction+=2) {
        complete(direction,2902u);complete(direction,3102u);complete(direction,3502u);
        for(kind=0;kind<6;++kind)rejection(direction,kind);
        for(kind=0;kind<8;++kind)interruption(direction,kind);
        already_aligned(direction);
        measured_full_lap_correction(direction);
        missed_initial_visual_packet(direction);
    }
    outer_diagnostic_isolation();
    puts("PASS: v34 measured both-side final70/50/10deg, explicit request+motion/identity guards, fresh path0 finite finish, positive-forward differential cap, no pre360 success, exactly one fast triple and success pair with normal restoration");
    return 0;
}
