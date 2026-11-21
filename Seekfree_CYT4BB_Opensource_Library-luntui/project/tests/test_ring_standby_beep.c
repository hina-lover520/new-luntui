#include "ring_runtime.h"
#include "ring_link.h"
#include "ring.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(c) do { if(!(c)) { fprintf(stderr,"FAIL standby beep line %d: %s\n",__LINE__,#c);exit(1); } } while(0)
static uint32_t command_sequence, epoch;
static unsigned beep_ons, beep_offs;
static uint8_t standing;
static float target, turn;

static int8_t tick(void)
{
    int8_t event=ring_runtime_tick(0,standing);
    if(event==1) ++beep_ons;
    if(event==0) ++beep_offs;
    return event;
}
static void ticks(unsigned count)
{ while(count--) (void)tick(); }
static void apply(int vision_state)
{
    target=123.0f;turn=17.0f;
    ring_runtime_apply(&target,&turn,0.0f,vision_state);
}
static void restart(void)
{
    ring_packet_t feedback;
    ring_runtime_init(0.1f);ring_runtime_set_operator(0,0);
    standing=0;beep_ons=beep_offs=0;command_sequence=0;
    apply(1);CHECK(ring_link_read(ring_motion_mailbox,&feedback));
    epoch=(uint32_t)feedback.word[RL_EPOCH];
}
static void command(int phase,int cap,uint32_t detected,uint32_t exited,int state)
{
    ring_packet_t packet={{0}};
    packet.word[RL_MAGIC]=RING_LINK_MAGIC;
    packet.word[RL_EPOCH]=(int32_t)epoch;
    packet.word[RC_PHASE]=ring_command_phase(phase,0);
    packet.word[RC_CAP]=ring_command_cap_turn(cap,0);
    packet.word[RC_DETECTED]=(int32_t)detected;
    packet.word[RC_EXITED]=(int32_t)exited;
    ring_link_write(ring_command_mailbox,&packet,&command_sequence);
    apply(state);
}
static void unchanged(void)
{
    CHECK(target==123.0f && turn==17.0f);
    CHECK(!ring_runtime_diagnostics()->active);
    CHECK(!ring_runtime_watchdog_stop());
}

int main(void)
{
    int i;
    restart();command(RING_NORMAL,-1,1,0,1);unchanged();
    CHECK(tick()==1 && beep_ons==1);
    for(i=0;i<8;++i) {
        /* New snapshots and duplicate reads both keep the same event count. */
        command(RING_NORMAL,-1,1,0,1);unchanged();
        apply(1);unchanged();ticks(50);
    }
    CHECK(beep_ons==1 && beep_offs==1);
    command(RING_COOLDOWN,-1,2,0,2);unchanged();ticks(400);
    CHECK(beep_ons==2 && beep_offs==2);
    CHECK(ring_runtime_diagnostics()->cap==-1);
    puts("PASS: parked NORMAL/COOLDOWN detection beeps once per new event without control ownership");

    /* A requested startup cancels an already playing preview tone ONCE.
     * Returning -1 thereafter is essential: repeated zero writes would
     * suppress the original startup buzzer in the same 1ms ISR. */
    restart();command(RING_NORMAL,-1,1,0,1);CHECK(tick()==1);
    ring_runtime_set_operator(1,0);
    CHECK(tick()==0);
    for(i=0;i<400;++i) CHECK(tick()==-1);
    CHECK(beep_ons==1 && beep_offs==1);unchanged();
    /* Startup also drops an event which was queued but never played. */
    restart();command(RING_NORMAL,-1,1,0,1);
    ring_runtime_set_operator(1,0);
    for(i=0;i<400;++i) CHECK(tick()==-1);
    standing=1;ticks(400);
    CHECK(!beep_ons && !beep_offs);
    puts("PASS: startup owns its buzzer; queued/active parked tones cannot leak into startup");

    restart();ring_runtime_set_operator(1,0);standing=1;CHECK(tick()==-1);
    /* A running confirmation arrives before state=3 handover. It must NOT
     * consume the event, or the following actual acquisition will be silent. */
    command(RING_APPROACH,100,1,0,1);unchanged();ticks(20);
    CHECK(!beep_ons);
    command(RING_APPROACH,100,1,0,3);
    CHECK(ring_runtime_diagnostics()->active && target==100.0f);
    ticks(400);CHECK(beep_ons==1 && beep_offs==1);
    command(RING_APPROACH,100,1,0,3);ticks(400);
    CHECK(beep_ons==1 && beep_offs==1);
    command(RING_COOLDOWN,-1,1,1,1);unchanged();ticks(400);
    CHECK(beep_ons==3 && beep_offs==3);
    puts("PASS: running acquisition fast1 and genuine successful exit fast2 are emitted once");
    return 0;
}
