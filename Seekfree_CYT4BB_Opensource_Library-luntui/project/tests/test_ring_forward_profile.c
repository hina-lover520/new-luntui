#include "ring_forward_profile.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL forward profile:%d %s\n",__LINE__,#x);exit(1);}}while(0)
int main(void){
 ring_forward_profile_t s={0};float out,prev;unsigned t;
 out=ring_forward_profile_apply(&s,160,0,1,0,0,1);CHECK(out==0);
 for(t=50;t<=5000;t+=50){prev=out;out=ring_forward_profile_apply(&s,160,t,1,0,prev,1);CHECK(out>=prev && out-prev<=1.751f);}
 CHECK(out==160);
 out=ring_forward_profile_apply(&s,90,5050,1,0,160,1);CHECK(out==156);
 for(t=5100;t<=7000;t+=50)out=ring_forward_profile_apply(&s,90,t,1,0,out,1);
 CHECK(out==90);
 out=ring_forward_profile_apply(&s,160,7050,1,0,90,1);CHECK(out<=91.751f);
 out=ring_forward_profile_apply(&s,160,7100,1,1,90,1);CHECK(out<=90); /* lost line cannot command160 */
 for(t=7150;t<=8500;t+=50)out=ring_forward_profile_apply(&s,160,t,1,0,out,1);
 prev=out;out=ring_forward_profile_apply(&s,160,8550,1,0,190,1);CHECK(out<=prev && s.goal<=151);
 out=ring_forward_profile_apply(&s,0,8600,0,0,190,1);CHECK(out==0 && !s.valid);
 out=ring_forward_profile_apply(&s,125,0xfffffff0u,1,0,90,1);CHECK(out==90);
 out=ring_forward_profile_apply(&s,125,34,1,0,90,1);CHECK(out>91.74f && out<91.76f);
 prev=out;out=ring_forward_profile_apply(&s,160,10034,1,0,90,0);CHECK(out-prev<=3.501f);
 puts("PASS: startup/rise/downshift/cancellation/lost-line/overspeed/stop/wrap/long-poll forward target bounds");return 0;
}

