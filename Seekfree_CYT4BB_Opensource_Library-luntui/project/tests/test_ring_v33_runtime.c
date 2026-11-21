/* Host-level control contract, using integrated gyro and real mailboxes.
 * These checks do not model motor/track dynamics or claim a field exit. */
#include "ring_runtime.h"
#include "ring_link.h"
#include "ring.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t sequence,epoch,detected,exit_sequence;
static float speed,turn,revs,measured,ordinary_turn;
static int side,observe;
static unsigned beeps;
static unsigned departure_announced;
#define CHECK(c) do { if(!(c)) { const ring_runtime_diagnostics_t *v=ring_runtime_diagnostics(); \
    fprintf(stderr,"FAIL v33 runtime:%d %s side%d phase%d man%u active%u seq%lu yaw%.1f rev%.4f turn%.2f\n", \
    __LINE__,#c,side,v->phase,v->maneuver_stage,v->active,(unsigned long)v->command_sequence, \
    v->maneuver_turn10*.1f,revs,turn);exit(1); } } while(0)

static const ring_runtime_diagnostics_t *diag(void)
{return ring_runtime_diagnostics();}
static void tick(int raw,unsigned count,int standing)
{while(count--)if(ring_runtime_tick((int16_t)raw,(uint8_t)standing)==1)++beeps;}
static void apply(void)
{
    speed=100;turn=ordinary_turn;
    if(observe)ring_runtime_observe_motion(measured,0,0,0,0);
    ring_runtime_apply(&speed,&turn,revs,3);
}
static void command_full(int phase,unsigned exited,unsigned release,uint32_t packet_epoch,
    uint32_t packet_detected,int route,int path)
{
    ring_packet_t p={{0}};
    int owned=phase>=RING_APPROACH && phase<=RING_RECOVER;
    p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)packet_epoch;
    p.word[RC_PHASE]=ring_command_versioned_phase(phase,0,RING_PAIR_VERSION);
    p.word[RC_CAP]=ring_command_cap_turn(owned?(phase==RING_APPROACH?100:90):-1,360);
    p.word[RC_DETECTED]=(int32_t)packet_detected;p.word[RC_EXITED]=(int32_t)exited;
    p.word[RC_FAULT]=ring_command_diagnostics(0,1,0,0,path,0,1,0,80,route);
    if(phase==RING_APPROACH)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|
        RING_COMMAND_FORCE_ENTRY|RING_COMMAND_TIMED_ENTRY);
    else if(phase==RING_ENTER || phase==RING_CIRCLE)
        p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_TIMED_ENTRY);
    p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|(release<<RING_COMMAND_RELEASE_SHIFT));
    ring_link_write(ring_command_mailbox,&p,&sequence);apply();
}
static void command(int phase,unsigned exited)
{command_full(phase,exited,0,epoch,detected,-side,1);}
static void boot(int direction)
{
    unsigned i;
    side=direction;sequence=exit_sequence=beeps=departure_announced=0;revs=0;measured=100;
    detected=1;ordinary_turn=(float)(side*120);observe=1;
    memset(ring_command_mailbox,0,RING_LINK_WORDS*sizeof(int32_t));
    memset(ring_motion_mailbox,0,RING_LINK_WORDS*sizeof(int32_t));
    memset(ring_exit_diagnostic_mailbox,0,RING_LINK_WORDS*sizeof(int32_t));
    ring_runtime_init(.1f);ring_runtime_set_operator(0,0);tick(0,2200,0);
    ring_runtime_set_operator(1,0);tick(0,1,1);apply();epoch=diag()->command_epoch;
    command(RING_APPROACH,0);
    for(i=0;i<RING_MANEUVER_DELAY_MS/20u+2u;++i) {
        tick(0,20,1);revs+=.03f;command(RING_APPROACH,0);
        if(diag()->maneuver_stage==2u)break;
    }
    CHECK(diag()->maneuver_stage==2u && beeps==1u);
}
static void advance(unsigned count,int phase)
{
    while(count) {
        unsigned n=count>20u?20u:count;
        tick(side*1000,n,1);revs+=.0015f*n;command(phase,0);count-=n;
    }
}
static void first_exit(void)
{
    advance(3601u,RING_ENTER);command(RING_EXIT,0);
    CHECK(diag()->active && diag()->phase==RING_EXIT && diag()->maneuver_stage==3u);
    departure_announced=1u;
}
static void no_exit_beep(void)
{tick(0,300,1);CHECK(!diag()->exited && beeps>=1u &&
    beeps<=(departure_announced?4u:1u));}
static void initial_reject(int direction,unsigned degrees_ms)
{
    uint32_t accepted;
    boot(direction);advance(degrees_ms,RING_ENTER);accepted=diag()->command_sequence;
    command(RING_EXIT,0);CHECK(diag()->command_sequence==accepted && diag()->phase==RING_ENTER);
    command(RING_COOLDOWN,1);CHECK(diag()->command_sequence==accepted && !diag()->exited);
    no_exit_beep();
}
static void initial_motion(int direction,int kind)
{
    uint32_t accepted;
    boot(direction);advance(3601u,RING_ENTER);accepted=diag()->command_sequence;
    if(kind==0)revs=-1;
    else if(kind==1)measured=0;
    else if(kind==2)measured=-10;
    else if(kind==3)measured=NAN;
    else {observe=0;tick(0,151,1);}
    command(RING_EXIT,0);CHECK(diag()->command_sequence==accepted && diag()->active);
    command(RING_COOLDOWN,1);CHECK(!diag()->exited);no_exit_beep();
}
static void continue_exit(int direction)
{
    boot(direction);first_exit();
    /* The ordinary road changes heading beyond the first 15-degree gate. */
    advance(200u,RING_EXIT);CHECK(diag()->phase==RING_EXIT && diag()->maneuver_turn10>3750);
    measured=5;command(RING_EXIT,0);
    CHECK(diag()->active && fabsf(turn)<=5.001f && diag()->entry_turn_cap10==50 && speed>0);
    ordinary_turn=0;command(RING_EXIT,0);CHECK(turn==0.f && diag()->active);
    ordinary_turn=(float)(-side*120);command(RING_RECOVER,0);
    CHECK(diag()->active && diag()->phase==RING_RECOVER &&
        side*turn<0.f && fabsf(turn)<=5.001f && diag()->entry_turn_cap10==50 && speed>0);
    ordinary_turn=-37;tick(0,20,1);revs+=.03f;command(RING_COOLDOWN,1);
    CHECK(!diag()->active && diag()->exited==1u && turn==-37.f);
    tick(0,600,1);CHECK(beeps==6u);
    command(RING_COOLDOWN,1);tick(0,200,1);CHECK(beeps==6u);
    /* Legacy vision may publish its first successful snapshot at COOLDOWN. */
    boot(direction);advance(3601u,RING_ENTER);command(RING_COOLDOWN,1);
    CHECK(!diag()->active && diag()->exited==1u);tick(0,300,1);CHECK(beeps==3u);
}
static void continuation_reject(int direction,int kind)
{
    uint32_t accepted;
    unsigned i;
    boot(direction);first_exit();accepted=diag()->command_sequence;
    if(kind==0) {advance(155u,RING_EXIT);accepted=diag()->command_sequence;
        tick(side*1000,155,1);revs+=.2f;}
    else if(kind==1) {tick(-side*1000,20,1);revs+=.03f;}
    else if(kind==2)revs-=.003f;
    else if(kind==3)measured=0;
    else if(kind==4) {ring_runtime_set_operator(0,0);ring_runtime_set_operator(1,0);}
    else if(kind==5) {tick(0,1,0);tick(0,1,1);}
    else if(kind==6) {
        /* RECOVER does not restart the four-second departure deadline. */
        for(i=0;i<195u;++i){tick(0,20,1);revs+=.03f;command(RING_EXIT,0);}
        command(RING_RECOVER,0);CHECK(diag()->active && diag()->phase==RING_RECOVER);
        tick(0,101,1);revs+=.15f;accepted=diag()->command_sequence;
    }
    else if(kind==7) {revs+=.1f;command(RING_EXIT,0);revs-=.001f;command(RING_EXIT,0);
        CHECK(diag()->active);revs-=.002f;accepted=diag()->command_sequence;}
    else if(kind==8) {
        command_full(RING_EXIT,0,0,epoch+1u,detected,-side,1);
        CHECK(!diag()->active);command(RING_COOLDOWN,1);no_exit_beep();return;
    }
    else if(kind==9) {
        command_full(RING_EXIT,0,0,epoch,detected+1u,-side,1);
        CHECK(!diag()->active);command(RING_COOLDOWN,1);no_exit_beep();return;
    }
    else if(kind==10) {
        command_full(RING_EXIT,0,0,epoch,detected,side,1);
        CHECK(!diag()->active);command(RING_COOLDOWN,1);no_exit_beep();return;
    }
    else {observe=0;tick(0,151,1);}
    command(RING_RECOVER,0);
    CHECK(!diag()->active && diag()->command_sequence==accepted && diag()->maneuver_stage==4u);
    command(RING_COOLDOWN,1);CHECK(!diag()->exited);no_exit_beep();
}
static void first_identity(int direction)
{
    uint32_t accepted;
    boot(direction);advance(3601u,RING_ENTER);accepted=diag()->command_sequence;
    command(RING_RECOVER,0);CHECK(diag()->command_sequence==accepted);
    command_full(RING_EXIT,0,0,epoch,detected+1u,-side,1);CHECK(diag()->command_sequence==accepted);
    command_full(RING_EXIT,0,0,epoch,detected,side,1);CHECK(diag()->command_sequence==accepted);
    command_full(RING_EXIT,0,0,epoch,detected,-side,0);CHECK(diag()->command_sequence==accepted);
    command(RING_EXIT,0);CHECK(diag()->active && diag()->phase==RING_EXIT);
    command_full(RING_COOLDOWN,0,2,epoch,detected,0,0);
    CHECK(!diag()->active && diag()->release_reason==2u && !diag()->exited);
}
static void circle_speed(int direction)
{
    boot(direction);advance(751u,RING_ENTER);CHECK(diag()->maneuver_stage==3u);
    measured=5;tick(0,1,1);command(RING_ENTER,0);
    CHECK(fabsf(turn)<=5.001f && diag()->entry_turn_cap10==50 && speed>0);
    /* The next handover step starts at the capped value actually sent. */
    measured=100;tick(0,1,1);command(RING_ENTER,0);CHECK(fabsf(turn)<7.f);
    tick(0,200u,1);command(RING_CIRCLE,0);tick(0,200u,1);command(RING_CIRCLE,0);
    CHECK(fabsf(turn)==90.f && diag()->entry_turn_cap10==900 && speed>0);
    measured=0;command(RING_CIRCLE,0);CHECK(turn==0 && speed>0);
    measured=-5;command(RING_ENTER,0);CHECK(turn==0 && speed>0);
    measured=NAN;command(RING_CIRCLE,0);CHECK(turn==0 && speed>0);
    measured=5;command(RING_CIRCLE,0);CHECK(fabsf(turn)<=5.001f && speed>0);
    observe=0;tick(0,151,1);command(RING_CIRCLE,0);CHECK(turn==0 && speed>0);
    observe=1;measured=5;ordinary_turn=120;command(RING_NORMAL,0);
    CHECK(turn==120.f && !diag()->active && !diag()->entry_turn_cap10);
}
static void exit_diagnostic(uint32_t packet_epoch,uint32_t command_sequence,uint32_t status)
{
    ring_packet_t p={{0}};
    p.word[RL_MAGIC]=RING_EXIT_DIAGNOSTIC_MAGIC;p.word[RL_EPOCH]=(int32_t)packet_epoch;
    p.word[RX_COMMAND_SEQUENCE]=(int32_t)command_sequence;p.word[RX_FRAME]=123;
    p.word[RX_STATUS]=(int32_t)status;p.word[RX_COUNTS_LAP]=ring_exit_diagnostic_counts(7,3,3612);
    p.word[RX_FORWARD_MREV]=800;ring_link_write(ring_exit_diagnostic_mailbox,&p,&exit_sequence);apply();
}
static void optional_diagnostic(void)
{
    uint32_t accepted;
    boot(-1);command(RING_NORMAL,0);ordinary_turn=31;apply();accepted=diag()->command_sequence;
    exit_diagnostic(epoch,accepted,(uint32_t)ring_exit_diagnostic_status(5,1,1,0));
    CHECK(diag()->exit_diagnostic_valid && diag()->exit_reject==5 && diag()->exit_candidate &&
        diag()->exit_latched && !diag()->exit_confirmed && diag()->exit_candidate_frames==7 &&
        diag()->exit_confirm_frames==3 && diag()->exit_lap10==3612 && diag()->exit_forward_mrev==800);
    CHECK(!diag()->active && turn==31.f && speed>0 && diag()->command_sequence==accepted);
    /* Invalid packets cannot replace previously valid evidence. */
    exit_diagnostic(epoch+1u,accepted,0);CHECK(diag()->exit_reject==5);
    exit_diagnostic(epoch,accepted+2u,0);CHECK(diag()->exit_reject==5);
    exit_diagnostic(epoch,accepted,0x80000000u);CHECK(diag()->exit_reject==5);
    ring_exit_diagnostic_mailbox[RL_MAGIC]=0;apply();CHECK(diag()->exit_reject==5);
    exit_diagnostic(epoch,accepted,0x10000u);CHECK(diag()->exit_diagnostic_valid);
    ring_exit_diagnostic_mailbox[RL_SEQUENCE]|=1;tick(0,301,1);apply();
    CHECK(!diag()->exit_diagnostic_valid && diag()->command_age_ms>RING_LINK_TIMEOUT_MS &&
        diag()->command_sequence==accepted && !diag()->active && turn==31.f);
    /* Even a fresh optional packet cannot keep an actual ring owner alive. */
    boot(1);accepted=diag()->command_sequence;tick(0,301,1);
    exit_diagnostic(epoch,accepted,0x70000u);
    CHECK(!diag()->active && diag()->command_age_ms>RING_LINK_TIMEOUT_MS && !diag()->exited);
}
int main(void)
{
    int direction,kind;static const unsigned rejected[]={1800u,2700u,3580u,3760u,5400u};
    unsigned i;
    CHECK(RING_MANEUVER_DELAY_MS==800u && RING_MANEUVER_TURN_DEG==75.f &&
        RING_EXIT_MIN_DEG==360.f && RING_LAP_EXIT_HEADING_TOL==15.f && RING_EXIT_CONTINUE_YAW_DEG==30.f);
    for(direction=-1;direction<=1;direction+=2) {
        for(i=0;i<sizeof(rejected)/sizeof(rejected[0]);++i)initial_reject(direction,rejected[i]);
        for(kind=0;kind<5;++kind)initial_motion(direction,kind);
        continue_exit(direction);first_identity(direction);circle_speed(direction);
        for(kind=0;kind<12;++kind)continuation_reject(direction,kind);
    }
    optional_diagnostic();
    puts("PASS: v33 both-side measured first EXIT gates, bounded same-exit continuation beyond375, unified4s and reversal/reset guards, post75 speed-scaled turn with output feedback, optional L isolation");
    return 0;
}
