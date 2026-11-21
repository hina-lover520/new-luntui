/* Public confirmation/turn sequence plus rigid-line geometric oracle.
 * No test treats injected gyro progress alone as a completed branch/lap. */
#include "../code/ring.c"
#include "ring_tangent_test_geometry.h"
#include <stdio.h>
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL v30:%d %s ph%d count%d left%d br%d\n",__LINE__,#c,ring_result.phase,tangent_left_count,ring_result.left_entry,ring_result.branch_ok);exit(1);}}while(0)
static uint8_t pixels[120][188];
static ring_motion_t m;
static void frame(unsigned feedback,int angle){
 ring_observe_frame_interval(20);ring_observe_track(1);ring_observe_maneuver_feedback(feedback,(int16_t)angle);
 ring_update((const uint8_t(*)[188])pixels,&m);
}
static void rotated_line(void){
 int side,j;const float angles[]={0,45,75,89,90,91,105};
 for(side=-1;side<=1;side+=2)for(j=0;j<7;++j){
  float a=.23f,b=8.f,forward=40.f,distance=side*25.f;
  float x=b-a*forward+distance*sqrtf(1+a*a),theta=side*angles[j]*RTG_PI/180.f;
  ring_init();ring_result.ring_side=(int8_t)side;mouth_entry_slope=a;
  mouth_entry_offset=b-a*119.f+94.f;ring_result.lap_origin_yaw=0;
  m=(ring_motion_t){0,side*angles[j],0,1};
  ring_result.guide_x=(int16_t)lroundf(94+cosf(theta)*x-sinf(theta)*forward);
  ring_result.guide_y=(uint8_t)lroundf(119-(sinf(theta)*x+cosf(theta)*forward));
  CHECK(fabsf(tangent_entry_distance(&m)-25.f)<1.5f);
 }
}
static void confirm(int side){
 int i;ring_init();m=(ring_motion_t){0,0,0,1};
 for(i=0;i<120;++i){rtg_pose_t q=rtg_approach(i);rtg_render(pixels,side,&q);m.now_ms+=20;m.wheel_revs+=.02f;frame(0,0);if(ring_result.detected_events)break;}
 CHECK(ring_result.timed_entry && ring_result.force_entry && ring_result.ring_side==side);
}
static void turn_evidence(int side,int replacement){
 int i,seen=0;rtg_pose_t prev={0,60,0};confirm(side);
 for(i=0;i<RTG_JOIN_STEPS;++i){
  rtg_pose_t q=rtg_join(side,i);float dx=q.x-prev.x,dy=q.y-prev.y;
  m.now_ms+=20;m.wheel_revs+=sqrtf(dx*dx+dy*dy)/RTG_WHEEL_CIRCUMFERENCE;m.yaw_deg=q.heading_deg;
  if(replacement)rtg_straight(pixels);else rtg_render(pixels,side,&q);
  frame(2,(int)lroundf(side*m.yaw_deg*10));prev=q;
  CHECK(!ring_result.committed && !ring_result.exited_events);
  if(tangent_left_count>=(int)RING_BRANCH_CONFIRM)seen=1;
  if(side*m.yaw_deg>=75.f)break;
 }
 CHECK(seen==!replacement);
 m.now_ms+=20;m.wheel_revs+=.02f;frame(3,750);
 if(replacement)CHECK(!ring_result.committed && !ring_result.exited_events);
 else CHECK(ring_result.committed && ring_result.left_entry && ring_result.phase==RING_CIRCLE);
}
int main(void){rotated_line();turn_evidence(-1,0);turn_evidence(1,0);turn_evidence(-1,1);turn_evidence(1,1);
 puts("PASS: v30 finite entry retains measured branch evidence across75; plain lane+yaw cannot commit; rotated incoming line remains finite/correct at75/89/90/91/105, both sides");return 0;}
