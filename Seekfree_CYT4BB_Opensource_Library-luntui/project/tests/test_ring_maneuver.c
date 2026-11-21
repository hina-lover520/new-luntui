#include "ring_maneuver.h"
#include "ring_link.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL maneuver:%d %s\n",__LINE__,#c);exit(1);}}while(0)
static void route(int side)
{
 ring_maneuver_t s;float out;uint32_t start=100,at=start+RING_MANEUVER_DELAY_MS;
 ring_maneuver_begin(&s,start,side,12,.3f);
 out=ring_maneuver_apply(&s,side*70,600,12,.6f,0,1,0);
 CHECK(s.stage==1 && out==side*70); /* former500ms boundary is still straight */
 out=ring_maneuver_apply(&s,side*70,at-1,12,.9f,0,1,0);
 CHECK(s.stage==1 && out==side*70 && s.progress==0);
 s.armed=1;
 out=ring_maneuver_apply(&s,-side*70,at,12,1.f,0,1,0);
 CHECK(s.stage==2 && side*out>0 && s.turn_yaw==12);
 (void)ring_maneuver_apply(&s,0,at+100,12-side*10,1.1f,-side*100,1,0);
 CHECK(s.stage==2 && s.progress==-10);
 (void)ring_maneuver_apply(&s,0,at+300,12+side*70,1.2f,side*100,1,0);
 CHECK(s.stage==2 && s.target_rate*side<100 && s.progress==70);
 (void)ring_maneuver_apply(&s,0,at+350,12+side*74.9f,1.3f,side*20,1,0);
 CHECK(s.stage==2);
 out=ring_maneuver_apply(&s,-side*13,at+400,12+side*75,1.4f,side*20,1,0);
 CHECK(s.stage==3 && s.reason==4 && out==-side*13);
 out=ring_maneuver_apply(&s,7,at+600,12,1.5f,0,1,0);
 CHECK(s.stage==3 && out==7 && s.started_ms==start); /* never re-arm */
 /* After75 the supplied visual output is authoritative, including
  * zero on a straight tangent and an opposite correction if off-centre.
  * Neither270 nor360 may enable a hidden angular-position controller. */
 out=ring_maneuver_apply(&s,-side*33,at+1000,12+side*270,3,side*90,1,1);
 CHECK(s.stage==3 && out==-side*33 && s.target_rate==0 && !s.lap_reached);
 out=ring_maneuver_apply(&s,0,at+1500,12+side*359,4,side*80,1,1);
 CHECK(out==0 && s.target_rate==0 && !s.lap_reached);
 out=ring_maneuver_apply(&s,21,at+1800,12+side*360.1f,5,0,1,1);
 CHECK(out==21 && s.target_rate==0 && s.lap_reached);
 out=ring_maneuver_apply(&s,-17,at+2500,12+side*380,6,0,1,1);
 CHECK(out==-17 && s.target_rate==0); /* no heading lock after360 */
}
static void interrupted(int kind)
{
 ring_maneuver_t s;float out;
 ring_maneuver_begin(&s,0,1,0,0);s.armed=1;
 (void)ring_maneuver_apply(&s,0,RING_MANEUVER_DELAY_MS,0,.5f,0,1,0);
 if(kind==0)out=ring_maneuver_apply(&s,-25,RING_MANEUVER_DELAY_MS+2001,0,.6f,0,1,0);
 else if(kind==1)out=ring_maneuver_apply(&s,-25,RING_MANEUVER_DELAY_MS+100,0,4.01f,0,1,0);
 else out=ring_maneuver_apply(&s,-25,RING_MANEUVER_DELAY_MS+100,0,.6f,0,0,0);
 CHECK(s.stage==4 && out==-25 && s.reason==(unsigned)(kind==0?3:kind==1?5:6));
 out=ring_maneuver_apply(&s,14,3100,75,.7f,0,1,0);CHECK(s.stage==4 && out==14);
}
int main(void)
{
 ring_maneuver_t s;int side,k;int32_t packed;
 CHECK(RING_TIMED_ENTRY==1 && RING_PAIR_VERSION==36);
 for(side=-1;side<=1;side+=2)route(side);
 for(k=0;k<3;++k)interrupted(k);
 ring_maneuver_begin(&s,0,1,0,0);s.armed=1;
 CHECK(ring_maneuver_apply(&s,90,100,5,.1f,0,1,0)==90); /* heading drift corrected */
 ring_maneuver_begin(&s,0xffffff00u,1,0,0);s.armed=1;
 (void)ring_maneuver_apply(&s,0,0xffffff00u+RING_MANEUVER_DELAY_MS,0,.1f,0,1,0);CHECK(s.stage==2); /* clock wrap */
 for(k=-1000;k<=1000;k+=125){packed=ring_motion_status(3,2,k);CHECK(ring_motion_status_valid(packed));CHECK((packed&3)==3 && ring_motion_stage(packed)==2 && ring_motion_progress(packed)==k);}
 CHECK(!ring_motion_status_valid((int32_t)0x80000003u));CHECK(!ring_motion_status_valid(3|(5<<2)));
 puts("PASS: configured heading hold then bounded75deg entry, visual outputs including zero/opposite correction at270/359/360/380 pass unchanged,360 exit measurement only, both sides, wrong-way/yaw oscillation cannot finish, rate taper, finite abort without zero, immutable completion, clock wrap and packed feedback");return 0;
}
