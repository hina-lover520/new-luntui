/* Manual commands and raw gyro counts; no recognition/state preparation
 * from the implementation. Assert actual final turn, not error/boost. */
#include "ring_force.h"
#include "ring_runtime.h"
#include "ring_link.h"
#include "ring_uart_diag.h"
#include <stdio.h>
#include <stdlib.h>
stc_SCB_t uart_port;
char uart_capture[32768];unsigned uart_capture_count;
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL force:%d %s\n",__LINE__,#c);exit(1);}}while(0)
static uint32_t seq,epoch;static float speed,turn;
static void ticks(int raw,unsigned n,int ready){while(n--)(void)ring_runtime_tick((int16_t)raw,(uint8_t)ready);}
static void apply(int state,float rev){speed=100;turn=-25;ring_runtime_apply(&speed,&turn,rev,state);}
static void boot(void)
{
 memset(ring_command_mailbox,0,32);memset(ring_motion_mailbox,0,32);seq=0;
 ring_runtime_init(.1f);ring_runtime_set_operator(1,0);ticks(0,2000,0);ticks(0,1,1);apply(1,0);
 epoch=ring_runtime_diagnostics()->command_epoch;
}
static void command(int side,int phase,int force,int stop,int state,float rev)
{
 ring_packet_t p={{0}};p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)epoch;
 p.word[RC_CAP]=ring_command_cap_turn(phase==RING_NORMAL || phase==RING_APPROACH?-1:stop?0:90,0);
 p.word[RC_PHASE]=ring_command_versioned_phase(phase,0,RING_PAIR_VERSION);p.word[RC_DETECTED]=1;
 p.word[RC_FAULT]=ring_command_diagnostics(0,1,0,0,1,stop,1,side*26,80,-side);
 if(force)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_FORCE_ENTRY);
 ring_link_write(ring_command_mailbox,&p,&seq);apply(state,rev);
}
static void runtime_contract(int side)
{
 int i;float previous;const ring_runtime_diagnostics_t*d;
 boot();command(side,RING_APPROACH,0,0,1,0);CHECK(speed==100 && turn==-25);
 command(side,RING_ENTER,1,0,3,0);
 for(i=1;i<=6;++i){ticks(0,46,1);command(side,RING_ENTER,1,0,3,.02f*i);d=ring_runtime_diagnostics();
  CHECK(d->force_active && d->force_side==side && d->force_target10==side*1000);
  CHECK(side*turn>0 && side*turn<=RING_FORCE_MAX_RPM && speed==90);
 }
 previous=turn;ticks(0,46,1);command(side,RING_ENTER,0,0,3,.14f);CHECK(turn==previous);
 ticks(0,121,1);command(side,RING_CIRCLE,0,0,3,.16f);CHECK(turn==-25 && !ring_runtime_diagnostics()->force_active);
 /* Repeated commands in the same entry cannot renew elapsed-time budget. */
 boot();command(side,RING_ENTER,1,0,3,0);
 for(i=0;i<35;++i){ticks(0,46,1);command(side,RING_ENTER,1,0,3,.01f*i);}
 d=ring_runtime_diagnostics();CHECK(!d->force_active && d->force_reason==3 && turn==-25 && speed==90);
 /* Neither yaw movement nor wheels require a matched optional D packet. */
 boot();command(side,RING_ENTER,1,0,3,0);ticks(side*2000,550,1);command(side,RING_ENTER,1,0,3,.1f);
 CHECK(!ring_runtime_diagnostics()->force_active && ring_runtime_diagnostics()->force_reason==4 && turn==-25);
 boot();command(side,RING_ENTER,1,0,3,0);ticks(0,46,1);command(side,RING_ENTER,1,0,3,2.51f);
 CHECK(!ring_runtime_diagnostics()->force_active && ring_runtime_diagnostics()->force_reason==5);
 boot();command(side,RING_ENTER,1,0,3,0);ticks(0,46,1);command(side,RING_ENTER,1,0,3,.02f);
 ticks(0,301,1);apply(3,.03f);CHECK(speed==100 && turn==-25 && !ring_runtime_diagnostics()->force_active);
 CHECK(ring_runtime_diagnostics()->release_reason==4 && !ring_runtime_watchdog_stop());
 /* Ring stop requests now release ownership to the original controller. */
 boot();command(side,RING_ENTER,1,0,3,0);ticks(0,46,1);command(side,RING_ENTER,1,0,3,.02f);
 ticks(0,46,1);command(side,RING_ENTER,0,1,3,.04f);CHECK(speed==100 && turn==-25);
 CHECK(!ring_runtime_diagnostics()->active && ring_runtime_diagnostics()->release_reason==1);
 /* A force bit in waiting/NORMAL must fail command validation. */
 boot();command(side,RING_APPROACH,1,0,3,0);CHECK(speed==100 && turn==-25 && !ring_runtime_diagnostics()->active);
 command(side,RING_NORMAL,1,0,1,0);CHECK(speed==100 && turn==-25);
}
static void feedback_and_lock(void)
{
 ring_force_t a,b;float slow,fast;int side;
 for(side=-1;side<=1;side+=2){
  ring_force_begin(&a,0,side,0,0);ring_force_begin(&b,0,side,0,0);
  (void)ring_force_apply(&a,0,100,0,.1f,0,side,1,1,1);
  (void)ring_force_apply(&b,0,100,0,.1f,side*150.f,side,1,1,1);
  slow=ring_force_apply(&a,0,200,0,.2f,0,side,1,1,1);
  fast=ring_force_apply(&b,0,200,0,.2f,side*150.f,side,1,1,1);
  CHECK(slow*side>fast*side && fast*side>=0 && slow*side<=90);
  (void)ring_force_apply(&a,0,246,0,.22f,0,-side,1,1,1);CHECK(a.spent && !a.active && a.reason==7);
  CHECK(ring_force_apply(&a,0,292,0,.24f,0,side,1,1,1)==0 && !a.active);
  ring_force_begin(&a,0,side,0,0);
  (void)ring_force_apply(&a,0,1450,0,.1f,0,side,1,1,1);
  (void)ring_force_apply(&a,-side*12.f,1490,0,.12f,0,side,0,1,1);CHECK(a.releasing);
  CHECK(ring_force_apply(&a,-side*12.f,1510,0,.14f,0,side,0,1,1)==-side*12.f && !a.releasing);
 }
}
static void circle_does_not_cut_or_restart(int side)
{
 unsigned i;boot();command(side,RING_ENTER,1,0,3,0);
 for(i=0;i<6;++i){ticks(side*1000,46,1);command(side,i<2?RING_ENTER:RING_CIRCLE,1,0,3,.06f*i);}
 CHECK(ring_runtime_diagnostics()->force_active && side*turn>0);
 for(i=0;i<28;++i){ticks(0,46,1);command(side,RING_CIRCLE,1,0,3,.36f+.02f*i);}
 CHECK(!ring_runtime_diagnostics()->force_active && ring_runtime_diagnostics()->force_reason==3);
 ticks(0,46,1);command(side,RING_ENTER,1,0,3,.75f);
 CHECK(!ring_runtime_diagnostics()->force_active); /* CIRCLE -> ENTER is not a new F */
}
int main(void)
{
 const ring_runtime_diagnostics_t*d;int i;
 runtime_contract(-1);runtime_contract(1);feedback_and_lock();
 circle_does_not_cut_or_restart(-1);circle_does_not_cut_or_restart(1);
 boot();command(1,RING_ENTER,1,0,3,0);
 /* At FIFO16 the full startup schema precedes optional K records. Bound
  * its drain explicitly instead of assuming an old schema byte length. */
 for(i=0;i<256;++i){
  const char *record;
  ticks(0,46,1);command(1,RING_ENTER,i<6,0,3,.001f*i);uart_port.used=0;ring_uart_diag_poll(26,100,-100);
  uart_capture[uart_capture_count]=0;record=strstr(uart_capture,"\r\nK,");
  if(record && strstr(record+2,"\r\n"))break;
 }
 CHECK(i<256);
 uart_capture[uart_capture_count]=0;d=ring_runtime_diagnostics();(void)d;
 CHECK(strstr(uart_capture,"#RING,36,"));CHECK(strstr(uart_capture,"#K,seq,ms,epoch,cmd,active,side,target10,rate10,out10,reason,age,drop"));
 CHECK(strstr(uart_capture,"\r\nK,"));
 puts("PASS: force final-output/right-left contracts, rate feedback, locked direction, non-renewable time/yaw/travel, release, stale/stop/invalid requests, UART K");return 0;
}
