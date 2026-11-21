/* Prepared visual-lap history tests the independent exit path only.
 * Recognition and completion of an actual moving route are covered by
 * paired fixed-world tests; prepared booleans are not field evidence. */
#include "../code/ring.c"
#include "ring_tangent_test_geometry.h"
#include <stdio.h>
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL visual exit:%d %s ph%d lap%.2f count%u\n",__LINE__,#c,ring_result.phase,ring_result.lap_deg,visual_exit_count);exit(1);}}while(0)
static uint8_t pixels[120][188];static ring_motion_t m;
static void prepare(int side,float degrees,int committed)
{
 ring_init();m=(ring_motion_t){100,side*degrees,2,1};ring_result.phase=RING_CIRCLE;
 ring_result.ring_side=(int8_t)side;ring_result.route=(int8_t)-side;
 ring_result.committed=ring_result.left_entry=(uint8_t)committed;
 ring_result.lap_valid=1;ring_result.lap_origin_yaw=0;
 ring_since=phase_since=100;timed_started=1;tangent_lap_revs=1;
 ring_result.detected_events=1;ring_result.approach_stage=RING_TANGENT_SHARED;
 rtg_straight(pixels);ring_observe_maneuver_feedback(3,(int16_t)(degrees*10));
}
static void step(void)
{m.now_ms+=20;m.wheel_revs+=.02f;ring_observe_frame_interval(20);ring_observe_track(1);ring_update((const uint8_t(*)[188])pixels,&m);}
static void accepted(int side,int shared)
{
 unsigned i;prepare(side,360.1f,1);
 if(!shared)ring_result.approach_stage=RING_TANGENT_LEFT;
 /* Prepared EXIT-only fixture: observe the new passive candidate gate
  * explicitly. End-to-end path ownership is tested by timed-integration. */
 tangent_lap(&m);ring_result.visual_dt_ms=50;
 for(i=0;i<3;++i){m.now_ms+=50;scan_geometry((const uint8_t(*)[188])pixels,&m);tangent_exit_observe(&m);}
 CHECK(ring_result.phase==RING_EXIT && ring_result.exit_latched);
 for(i=0;i<50 && !ring_result.exited_events;++i){step();if(i<11){CHECK(!ring_result.exited_events);
  CHECK(ring_result.path_valid && abs(ring_result.steer_error)<=2 && ring_result.path_source==4);
 }}
 CHECK(ring_result.exited_events==1 && !ring_owns_path() && !ring_result.stop && !ring_result.timed_entry);
 CHECK(ring_blend_error(19)==19 && ring_speed_limit()==-1);
}
static void rejected(int side,int kind)
{
 unsigned i;prepare(side,kind==0?270.f:kind==1?359.f:360.1f,kind==2?0:1);
 if(kind==3){memset(pixels,0,sizeof(pixels));}
 if(kind==4){rtg_cross(pixels);}
 if(kind==5){m.yaw_deg=side*380.f;}
 if(kind==6){m.wheel_revs=tangent_lap_revs;}
 for(i=0;i<35;++i){
  if(kind==6)m.wheel_revs=tangent_lap_revs-.02f;
  step();CHECK(!ring_result.exited_events && !ring_result.stop);
 }
}
int main(void)
{int side,k;for(side=-1;side<=1;side+=2){accepted(side,0);accepted(side,1);for(k=0;k<7;++k)rejected(side,k);}
 puts("PASS: visually committed full lap + aligned complete ordinary image exits without mandatory N/F pixels;270/359,uncommitted360,blank,cross,wrong heading,no net forward travel cannot fake exit;handoff returns original error/speed");return 0;}
