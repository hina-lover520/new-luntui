/* Production control/mailbox regression: uncapped exit observation and
 * late heading correction. No motor/camera dynamics are simulated. */
#define main inherited_v33_runtime_main
#include "test_ring_v33_runtime.c"
#undef main

static void force_command(int phase,int path,unsigned exited,int requested)
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

static void observation_command(int kind)
{
    ring_packet_t p={{0}};
    int phase=kind==1?RING_ENTER:RING_CIRCLE;
    p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)epoch;
    p.word[RC_PHASE]=ring_command_versioned_phase(phase,kind==5?1:0,
        kind==12?RING_PAIR_VERSION-1u:RING_PAIR_VERSION);
    p.word[RC_CAP]=ring_command_cap_turn(-1,diag()->maneuver_turn10/10);
    p.word[RC_DETECTED]=(int32_t)(detected+(kind==9?1u:0u));
    p.word[RC_EXITED]=0;
    p.word[RC_FAULT]=ring_command_diagnostics(kind==6?1:0,0,0,
        kind==8?0:1,kind==7?1:0,kind==4?1:0,1,0,80,kind==10?side:-side);
    if(kind==2)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_TIMED_ENTRY);
    if(kind==3)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_FORCE_ENTRY);
    ring_link_write(ring_command_mailbox,&p,&sequence);apply();
}

static void uncapped_observation(int direction)
{
    unsigned n;float initial;uint32_t accepted;
    boot(direction);advance(1002u,RING_ENTER);
    ordinary_turn=137.f;measured=100.f;initial=speed;
    CHECK(initial<=90.01f);
    observation_command(0);
    CHECK(diag()->active && diag()->cap==-1 && diag()->phase==RING_CIRCLE &&
        diag()->maneuver_stage==3u && !diag()->path_valid && diag()->committed);
    CHECK(turn==137.f && !diag()->force_active);
    for(n=0;n<20u;++n) {
        tick(side*1000,20u,1);revs+=.03f;observation_command(0);
        CHECK(diag()->active && diag()->cap==-1 && turn==137.f && !diag()->force_active);
        CHECK(speed>=initial && speed<=100.001f);
        initial=speed;
    }
    CHECK(speed>99.9f && diag()->speed_goal10==1000 && diag()->held_target10==0);
    CHECK(diag()->maneuver_turn10>1390);
    accepted=diag()->command_sequence;
    /* Cached/optional traffic cannot keep this uncapped context alive. */
    tick(0,RING_LINK_TIMEOUT_MS+1u,1);apply();
    CHECK(diag()->command_sequence==accepted && !diag()->active &&
        diag()->release_reason==4u && !ring_runtime_watchdog_stop() && turn==137.f);
}
static void reject_bad_observation(int direction,int kind)
{
    uint32_t accepted;
    boot(direction);
    if(kind!=13)advance(1002u,RING_ENTER);
    if(kind==14)command(RING_NORMAL,0);
    if(kind==11)tick(0,RING_LINK_TIMEOUT_MS+1u,1);
    accepted=diag()->command_sequence;
    observation_command(kind);
    CHECK(diag()->command_sequence==accepted && (kind==14?!diag()->active:diag()->cap!=-1));
    CHECK(!diag()->exited && !diag()->exit_force_stage);
}
static void overshoot_correction(int direction,unsigned initial_ms)
{
    unsigned n,seen;uint32_t accepted;
    boot(direction);advance(initial_ms,RING_ENTER);
    force_command(RING_EXIT,1,0,1);
    CHECK(diag()->active && diag()->phase==RING_EXIT && diag()->exit_force_stage==1u &&
        diag()->exit_force_side==-side && diag()->force_active);
    CHECK(diag()->exit_force_remaining10>0 && diag()->exit_force_remaining10<=700);
    accepted=diag()->command_sequence;
    force_command(RING_RECOVER,1,0,1);
    CHECK(diag()->command_sequence==accepted && diag()->exit_force_stage==1u);
    /* Both continued positive wheel motion and opposite yaw are required. */
    for(n=0;n<45u && diag()->exit_force_stage==1u;++n) {
        tick(-side*1000,20u,1);revs+=.03f;force_command(RING_EXIT,0,0,1);
        CHECK(diag()->active && speed>0.f && !diag()->exited);
        CHECK(diag()->exit_force_side==-side);
        if(diag()->exit_force_stage==1u)CHECK(side*turn<=0.f);
    }
    CHECK(diag()->exit_force_stage==2u && diag()->maneuver_turn10>=3580 &&
        diag()->maneuver_turn10<=3600);
    force_command(RING_EXIT,1,0,1);
    tick(0,320u,1);force_command(RING_EXIT,1,0,1);
    seen=beeps;CHECK(seen==4u);
    force_command(RING_RECOVER,1,0,1);
    CHECK(diag()->active && diag()->phase==RING_RECOVER);
    ordinary_turn=-37.f;force_command(RING_COOLDOWN,1,1,0);
    CHECK(!diag()->active && diag()->exited==1u && turn==-37.f);
    tick(0,200u,1);CHECK(beeps==seen+2u);
}
static void overshoot_refusals(int direction,int kind)
{
    unsigned n;uint32_t accepted;
    boot(direction);advance(kind==0?4305u:4102u,RING_ENTER);
    accepted=diag()->command_sequence;
    force_command(RING_EXIT,1,0,1);
    if(kind==0) {
        CHECK(diag()->command_sequence==accepted && !diag()->exit_force_stage);
        return;
    }
    CHECK(diag()->exit_force_stage==1u);
    if(kind==1) {
        tick(side*1000,40u,1);revs+=.06f;force_command(RING_EXIT,1,0,1);
    } else if(kind==2) {
        tick(0,RING_LINK_TIMEOUT_MS+1u,1);apply();
    } else if(kind==3) {
        measured=0.f;tick(0,1u,1);force_command(RING_EXIT,1,0,1);
    } else {
        for(n=0;n<110u && diag()->active;++n) {
            tick(0,20u,1);revs+=.02f;force_command(RING_EXIT,0,0,1);
        }
    }
    CHECK(!diag()->active && !diag()->exited && speed>0.f);
}
static void early_exit_denied(int direction,unsigned initial_ms)
{
    uint32_t accepted;
    boot(direction);advance(initial_ms,RING_ENTER);
    CHECK(diag()->active && diag()->maneuver_stage==3u && beeps==1u);
    accepted=diag()->command_sequence;
    force_command(RING_EXIT,1,0,1);
    CHECK(diag()->command_sequence==accepted && diag()->active &&
        diag()->phase==RING_ENTER && !diag()->exit_force_stage && !diag()->exited);
    force_command(RING_COOLDOWN,1,1,0);
    CHECK(diag()->command_sequence==accepted && diag()->active &&
        diag()->phase==RING_ENTER && !diag()->exit_force_stage && !diag()->exited);
    tick(0,300u,1);CHECK(beeps==1u);
}
int main(void)
{
    int direction,kind;
    for(direction=-1;direction<=1;direction+=2) {
        uncapped_observation(direction);
        for(kind=1;kind<=14;++kind)reject_bad_observation(direction,kind);
        overshoot_correction(direction,4002u);
        overshoot_correction(direction,4102u);
        overshoot_correction(direction,4298u);
        for(kind=0;kind<5;++kind)overshoot_refusals(direction,kind);
        early_exit_denied(direction,2902u);early_exit_denied(direction,3502u);
    }
    puts("PASS: v36 both-side strict existing uncapped observation preserves ordinary steering, clears entry cap, resumes slew and tracks lap; malformed/unacquired/stale negative caps rejected; actual400/410/429deg history permits bounded correction toward original360,290/350deg EXIT/success denied, with finite motion/identity gates");
    return 0;
}

