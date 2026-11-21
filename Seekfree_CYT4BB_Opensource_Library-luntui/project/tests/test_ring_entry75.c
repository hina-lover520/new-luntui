#include "../code/ring.c"
#include "ring_force.h"
#include "ring_tangent_test_geometry.h"
#include <stdio.h>
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL entry-budget:%d %s phase%d force%d angle%.1f\n",__LINE__,#c,ring_result.phase,ring_result.force_entry,assist_total_yaw);exit(1);}}while(0)
static uint8_t pixels[120][188];
static ring_motion_t m;
static void step(void)
{ring_observe_track(1);ring_observe_frame_interval(20);ring_update((const uint8_t(*)[188])pixels,&m);}
static void approach(int side)
{
 int i;ring_init();m=(ring_motion_t){0,0,0,1};
 for(i=0;i<=125;++i){rtg_pose_t p=rtg_approach(i);rtg_render(pixels,side,&p);m.now_ms+=20;m.wheel_revs+=.02f;step();if(ring_result.phase==RING_ENTER)break;}
 CHECK(ring_result.phase==RING_ENTER && ring_result.force_entry);
}
static void visual_budget(int side,int kind)
{
 uint32_t started;float rev;approach(side);started=m.now_ms;rev=m.wheel_revs;
 /* This frozen mouth can support bounded steering but never fabricate
  * departure/commitment. Old 45deg/.8rev/600ms must no longer end it. */
 m.now_ms+=650;m.yaw_deg=side*50.f;m.wheel_revs=rev+1.f;step();
 CHECK(ring_result.phase==RING_ENTER && ring_result.force_entry && ring_result.path_source==5);
 CHECK(!ring_result.left_entry && !ring_result.committed && !ring_result.exited_events);
 m.now_ms=started+1100;m.yaw_deg=side*80.f;m.wheel_revs=rev+1.8f;step();
 CHECK(ring_result.phase==RING_ENTER && ring_result.force_entry && ring_result.path_source==5);
 CHECK(!ring_result.left_entry && !ring_result.committed && !ring_result.exited_events);
 m.now_ms=started+1200;
 if(kind==0)m.yaw_deg=side*101.f;
 if(kind==1)m.wheel_revs=rev+2.51f;
 if(kind==2)m.now_ms=started+1501;
 if(kind==3){ring_observe_track(1);ring_observe_frame_interval(65535u);ring_update((const uint8_t(*)[188])pixels,&m);}
 else step();
 CHECK(!ring_result.force_entry && !ring_result.committed && !ring_result.exited_events);
}
static void controller_budget(int side)
{
 ring_force_t f;float out;ring_force_begin(&f,100,side,0,0);
 out=ring_force_apply(&f,side*10,750,side*50,1.f,side*100,side,1,1,1);
 CHECK(f.active && side*out>0 && !f.spent);
 out=ring_force_apply(&f,side*10,1200,side*80,1.8f,side*100,side,1,1,1);
 CHECK(f.active && side*out>0 && !f.spent);
 out=ring_force_apply(&f,-side*10,1300,side*101,1.9f,side*100,side,1,1,1);
 CHECK(f.spent && !f.active && f.reason==4 && out==-side*10);
 (void)ring_force_apply(&f,0,1800,side*10,0,0,side,1,1,1);
 CHECK(!f.active && f.reason==4); /* first expiration reason persists */
}
int main(void)
{int side,k;for(side=-1;side<=1;side+=2){for(k=0;k<4;++k)visual_budget(side,k);controller_budget(side);}puts("PASS: paired 100deg/1500ms/2.5rev F budgets exceed both old limits, independent visual/controller expiration and unknown-time rejection, no fabricated entry or budget renewal");return 0;}
