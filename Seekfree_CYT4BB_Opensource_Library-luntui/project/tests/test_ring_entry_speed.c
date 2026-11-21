/* Production command-mailbox regression; optional telemetry cannot arm. */
#include "ring_entry_speed.h"
#include "ring_runtime.h"
#include "ring_link.h"
#include "ring_uart_diag.h"
#include <stdlib.h>
#include <math.h>
stc_SCB_t uart_port;char uart_capture[32768];unsigned uart_capture_count;
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL entry speed:%d %s f%.3f turn%.3f stage%u\n",__LINE__,#c,forward,turn,ring_runtime_diagnostics()->maneuver_stage);exit(1);}}while(0)
static uint32_t seq,epoch;static float forward,turn,measured=150;static int permit=0,observe=1,path=1;
static void ticks(int raw,unsigned n,int ready){while(n--)(void)ring_runtime_tick((int16_t)raw,(uint8_t)ready);}
static void apply(float normal,int state){forward=normal;turn=17;
 if(observe)ring_runtime_observe_motion(measured,2.25f,-3.5f,-1234,2345);
 ring_runtime_apply(&forward,&turn,0,state);}
static void boot(void){
 memset(ring_command_mailbox,0,32);memset(ring_motion_mailbox,0,32);seq=0;permit=0;observe=1;path=1;measured=150;
 ring_runtime_init(.1f);ring_runtime_set_operator(1,0);ticks(0,2000,0);ticks(0,1,1);apply(150,1);
 epoch=ring_runtime_diagnostics()->command_epoch;}
static void command(int side,int phase,int cap,int timed,float normal,int state){
 ring_packet_t p={{0}};p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)epoch;
 p.word[RC_CAP]=ring_command_cap_turn(cap,0);p.word[RC_PHASE]=ring_command_versioned_phase(phase,0,RING_PAIR_VERSION);p.word[RC_DETECTED]=1;
 p.word[RC_FAULT]=ring_command_diagnostics(0,1,0,0,path,0,1,0,80,-side);
 if(phase==RING_COOLDOWN)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|(1u<<RING_COMMAND_RELEASE_SHIFT));
 if(timed)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_TIMED_ENTRY);
 if(permit && phase==RING_APPROACH)p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_FORCE_ENTRY);
 ring_link_write(ring_command_mailbox,&p,&seq);apply(normal,state);}
static void profile(int side){
 unsigned i;float previous;boot();command(side,RING_APPROACH,100,1,150,3);
 CHECK(forward==150 && turn==17);
 for(i=0;i<40;++i){previous=forward;measured=previous;ticks(0,50,1);command(side,RING_APPROACH,100,1,150,3);
  CHECK(forward<=previous && previous-forward<=4.001f && forward>0);
  CHECK(ring_runtime_diagnostics()->maneuver_stage==1 && !ring_runtime_diagnostics()->force_active);}
 CHECK(forward==100);
 permit=1;measured=175;ticks(0,50,1);command(side,RING_APPROACH,100,1,150,3);
 CHECK(ring_runtime_diagnostics()->maneuver_stage==1 && ring_runtime_diagnostics()->force_reason==2);
 observe=0;ticks(0,160,1);command(side,RING_APPROACH,100,1,150,3);CHECK(ring_runtime_diagnostics()->maneuver_stage==1);
 observe=1;measured=100;path=0;ticks(0,50,1);command(side,RING_APPROACH,100,1,150,3);
 CHECK(ring_runtime_diagnostics()->maneuver_stage==2 && side*turn>0);path=1;
 for(i=0;i<16;++i){previous=forward;measured=forward;ticks(side*1000,50,1);command(side,RING_ENTER,90,1,150,3);
  CHECK(forward<=100 && forward>0 && fabsf(forward-previous)<=4.001f);}
 CHECK(ring_runtime_diagnostics()->maneuver_stage==3);
 for(i=0;i<12;++i){measured=forward;ticks(0,50,1);command(side,RING_CIRCLE,90,0,150,3);}
 CHECK(forward==90 && turn==17);
 previous=forward;ticks(0,46,1);command(side,RING_COOLDOWN,-1,0,160,1);
 CHECK(!ring_runtime_diagnostics()->active && turn==17 && forward-previous<=1.611f && forward>=90);
 previous=forward;ticks(0,46,1);apply(160,0);CHECK(forward<=previous && turn==17);
 CHECK(!ring_runtime_diagnostics()->held_target10);
 apply(0,1);CHECK(forward==0 && turn==17);
 ring_runtime_set_operator(0,0);apply(150,1);CHECK(forward==150 && turn==17 && !ring_runtime_diagnostics()->force_active);
}
static void timeout_case(void){
 float fallback,output;boot();command(1,RING_APPROACH,100,1,150,3);ticks(0,100,1);command(1,RING_APPROACH,100,1,150,3);
 fallback=forward;output=0;ticks(0,301,1);ring_runtime_poll_timeout(&forward,&output);
 CHECK(forward==fallback && output==17);
 ticks(0,1,1);measured=fallback;apply(160,1);CHECK(forward-fallback<=3.501f && turn==17);
}
static void wire(void){
 unsigned i;char *p,*end;unsigned commas=0;boot();command(1,RING_APPROACH,100,1,150,3);
 for(i=0;i<180;++i){uart_port.used=0;ring_uart_diag_poll(0,0,0);}
 uart_capture[uart_capture_count]=0;CHECK(strstr(uart_capture,"#RING,36,") && strstr(uart_capture,"meas10,pitch100,tilt100,pwmL,pwmR,m_age"));
 p=strstr(uart_capture,"\r\nK,");CHECK(p);p+=2;end=strstr(p,"\r\n");CHECK(end);
 while(p<end)if(*p++==',')++commas;
 CHECK(commas==29);
}
int main(void){profile(-1);profile(1);timeout_case();wire();
 puts("PASS: both-side visual approach, immediate deceleration, portal/speed/observation gates, bounded75 handover, cancellation/lost-line no speed surge, zero/stop and ISR release, K29 wire");return 0;}

