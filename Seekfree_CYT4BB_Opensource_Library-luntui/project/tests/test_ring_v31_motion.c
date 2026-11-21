/* Runtime-level entry regression. No motor or track dynamics claim. */
#include "ring_runtime.h"
#include "ring_link.h"
#include "ring.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL v31 motion:%d %s f%.2f t%.2f man%u\n",__LINE__,#c,speed,turn,ring_runtime_diagnostics()->maneuver_stage);exit(1);}}while(0)
static float speed,turn,measured;static uint32_t seq,epoch;
static void tick(int raw,unsigned n,int standing)
{while(n--)(void)ring_runtime_tick((int16_t)raw,(uint8_t)standing);}
static void apply(void)
{speed=100;turn=17;ring_runtime_observe_motion(measured,0,0,0,0);
 ring_runtime_apply(&speed,&turn,1,3);}
static void command(int side,int phase)
{
 ring_packet_t p={{0}};p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)epoch;
 p.word[RC_PHASE]=ring_command_versioned_phase(phase,0,RING_PAIR_VERSION);
 p.word[RC_CAP]=ring_command_cap_turn(phase==RING_APPROACH?100:90,0);p.word[RC_DETECTED]=1;
 p.word[RC_FAULT]=ring_command_diagnostics(0,1,0,0,1,0,1,0,80,-side);
 p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_TIMED_ENTRY);
 if(phase==RING_APPROACH)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_FORCE_ENTRY);
 ring_link_write(ring_command_mailbox,&p,&seq);apply();
}
static void boot(void)
{
 memset(ring_command_mailbox,0,32);memset(ring_motion_mailbox,0,32);seq=0;
 ring_runtime_init(.1f);ring_runtime_set_operator(0,0);
 /* The old one-second window would store this -130dps hand movement.
  * Stable standby afterward must replace it before RUN freezes the zero. */
 tick(20,999,0);tick(-1300,1001,0);tick(20,1000,0);
 measured=100;apply();CHECK(ring_runtime_diagnostics()->yaw_calibrated);
 CHECK(abs(ring_runtime_diagnostics()->yaw_bias_raw-20)<=1);
 ring_runtime_set_operator(1,0);tick(20,10,1);apply();
 epoch=ring_runtime_diagnostics()->command_epoch;
 CHECK(abs(ring_runtime_diagnostics()->force_rate10)<=1);
}
static void motion(int side)
{
 unsigned i;const ring_runtime_diagnostics_t *d=ring_runtime_diagnostics();
 boot();measured=0;command(side,RING_APPROACH);
 for(i=0;i<40;++i){tick(20,50,1);command(side,RING_APPROACH);}
 CHECK(d->maneuver_stage==1 && !d->force_active && d->force_reason==2);
 measured=-20;tick(20,50,1);command(side,RING_APPROACH);
 CHECK(d->maneuver_stage==1 && !d->force_active);
 measured=100;tick(20,50,1);command(side,RING_APPROACH);
 CHECK(d->maneuver_stage==2 && d->force_active && side*turn>0);
 /* Full authority is preserved when the vehicle actually advances. */
 tick(20,50,1);command(side,RING_ENTER);
 CHECK(d->entry_turn_cap10==900 && side*turn>0 && speed>0);
 measured=5;tick(20,50,1);command(side,RING_ENTER);
 CHECK(fabsf(turn)<=5.001f && d->entry_turn_cap10==50 && speed>0);
 measured=0;tick(20,50,1);command(side,RING_ENTER);
 CHECK(turn==0 && speed>0 && d->force_reason==8);
 measured=-50;tick(20,50,1);command(side,RING_ENTER);
 CHECK(turn==0 && speed>0 && d->entry_turn_cap10==0);
 measured=100;tick(20+side*1000,100,1);command(side,RING_ENTER);
 CHECK(side*turn>0 && d->maneuver_stage==2);
 for(i=0;i<7;++i){tick(20+side*1000,100,1);command(side,RING_ENTER);}
 CHECK(d->maneuver_stage==3 && !d->force_active);
 CHECK(d->yaw_bias_raw==20); /* a true turn never becomes a new zero */
 tick(20,350,1);apply();CHECK(turn==17 && speed>0);
}
static void invalid_zero(void)
{
 unsigned i;const ring_runtime_diagnostics_t *d=ring_runtime_diagnostics();
 memset(ring_command_mailbox,0,32);memset(ring_motion_mailbox,0,32);seq=0;
 ring_runtime_init(.1f);ring_runtime_set_operator(0,0);tick(1500,3000,0);
 ring_runtime_set_operator(1,0);measured=100;tick(20,1,1);apply();
 epoch=d->command_epoch;CHECK(!d->yaw_calibrated);
 for(i=0;i<40;++i){tick(20,50,1);command(-1,RING_APPROACH);}
 CHECK(!d->active && !d->force_active && !d->maneuver_stage && turn==17 && speed==100);
}
int main(void)
{
 CHECK(RING_ENTRY_MIN_FORWARD_RPM>0 && RING_MANEUVER_TURN_DEG==75.f);
 motion(-1);motion(1);invalid_zero();
 puts("PASS: boot hand-motion bias corrected in standby; stationary/reverse cannot arm; full75 authority with forward motion; collapse suppresses axle spin without zero/negative forward request; both sides and uncalibrated isolation");
 return 0;
}
