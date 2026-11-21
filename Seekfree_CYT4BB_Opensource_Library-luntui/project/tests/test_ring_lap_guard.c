/* Control-core checks use actual integrated gyro, not an optional H/K
 * diagnostic or the vision core's claimed angle. No camera-success claim. */
#include "ring_runtime.h"
#include "ring_link.h"
#include "ring.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL lap guard:%d %s\n",__LINE__,#c);exit(1);}}while(0)
static uint32_t seq,epoch;
static float speed,turn,revs;
static void tick(int raw,unsigned n)
{while(n--)(void)ring_runtime_tick((int16_t)raw,1);}
static void apply(void)
{speed=100;turn=-33;ring_runtime_observe_motion(100,0,0,0,0);ring_runtime_apply(&speed,&turn,revs,1);}
static void command(int side,int phase,int claimed_deg,unsigned exited,unsigned release)
{
 ring_packet_t p={{0}};int owned=phase>=RING_APPROACH && phase<=RING_RECOVER;
 p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)epoch;
 p.word[RC_PHASE]=ring_command_versioned_phase(phase,0,RING_PAIR_VERSION);
 p.word[RC_CAP]=ring_command_cap_turn(owned?(phase==RING_APPROACH?100:90):-1,claimed_deg);
 p.word[RC_DETECTED]=1;p.word[RC_EXITED]=(int32_t)exited;
 p.word[RC_FAULT]=ring_command_diagnostics(0,1,0,0,1,0,1,0,80,-side);
 if(phase==RING_APPROACH)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_FORCE_ENTRY);
 if(owned)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_TIMED_ENTRY);
 p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|(release<<RING_COMMAND_RELEASE_SHIFT));
 ring_link_write(ring_command_mailbox,&p,&seq);apply();
}
static void boot(int side)
{
 unsigned i;memset(ring_command_mailbox,0,32);memset(ring_motion_mailbox,0,32);seq=0;revs=0;
 ring_runtime_init(.1f);ring_runtime_set_operator(1,0);
 for(i=0;i<2000;++i)(void)ring_runtime_tick(0,0);
 tick(0,1);apply();epoch=ring_runtime_diagnostics()->command_epoch;
 command(side,RING_APPROACH,0,0,0);
 for(i=0;i<RING_MANEUVER_DELAY_MS/20u;++i){tick(0,20);command(side,RING_APPROACH,0,0,0);}
 CHECK(ring_runtime_diagnostics()->maneuver_stage==2);
}
static void advance(int side,unsigned ms)
{
 while(ms){unsigned n=ms>20?20:ms;tick(side*1000,n);command(side,RING_ENTER,360,0,0);ms-=n;}
}
static void guard(int side)
{
 uint32_t accepted;const ring_runtime_diagnostics_t*d;
 boot(side);advance(side,2700);d=ring_runtime_diagnostics();
 CHECK(d->maneuver_stage==3 && d->active && !d->force_active && d->maneuver_turn10>=2699 && d->maneuver_turn10<=2701);
 accepted=d->command_sequence;
 command(side,RING_EXIT,360,0,0);d=ring_runtime_diagnostics();
 CHECK(d->active && d->command_sequence==accepted && d->phase==RING_ENTER && speed>0);
 command(side,RING_COOLDOWN,360,1,0);d=ring_runtime_diagnostics();
 CHECK(d->active && d->command_sequence==accepted && !d->exited);
 advance(side,880);CHECK(ring_runtime_diagnostics()->maneuver_stage==3 && !ring_runtime_diagnostics()->force_active);
 command(side,RING_COOLDOWN,360,1,0);CHECK(ring_runtime_diagnostics()->active && !ring_runtime_diagnostics()->exited);
 advance(side,30);d=ring_runtime_diagnostics();CHECK(d->maneuver_stage==3 && d->active);
 accepted=d->command_sequence;
 command(side,RING_COOLDOWN,360,1,0);d=ring_runtime_diagnostics();
 CHECK(d->active && d->command_sequence==accepted && !d->exited); /* yaw without translation */
 revs=1;
 /* Heading must still match: a previous360 does not authorize an exit
  * after rotating farther away from the original travel direction. */
 tick(side*1000,200);accepted=d->command_sequence;
 command(side,RING_COOLDOWN,360,1,0);d=ring_runtime_diagnostics();
 CHECK(d->active && d->command_sequence==accepted && !d->exited);
 /* An explicit abort releases original outputs, without an exit event. */
 command(side,RING_COOLDOWN,0,0,2);d=ring_runtime_diagnostics();
 CHECK(!d->active && speed>=90 && speed<=100 && turn==-33 && d->release_reason==2 && !d->exited);
 boot(side);advance(side,3610);ring_runtime_set_operator(0,0);apply();
 CHECK(ring_runtime_diagnostics()->maneuver_stage==4 && !ring_runtime_diagnostics()->force_active && turn==-33);
}
int main(void)
{guard(-1);guard(1);puts("PASS: controller accepts visual handoff after75, rejects claimed360 exit at270/358, in-place360 without forward travel, wrong heading; no optional diagnostic dependency; explicit abort and operator stop release unchanged ordinary outputs, no false exit");return 0;}
