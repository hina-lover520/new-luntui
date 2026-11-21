#include "ring_test_geometry.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL branch-follow line %d: %s ph=%d src=%u mode=%u branch=%u age=%u inner=%u assoc=%u support=%u guide=(%d,%u) rej=%u hold=%u stop=%u\n",__LINE__,#x,ring_result.phase,ring_result.path_source,ring_result.branch_mode,ring_result.branch_ok,ring_result.strong_age_ms,ring_result.inner_rows,ring_result.associated_rows,ring_result.target_support,ring_result.guide_x,ring_result.guide_y,ring_result.reject,ring_result.hold_reject,ring_result.stop);exit(1);}}while(0)
static uint8_t image[RING_H][RING_W], saved[RING_H][RING_W];
static ring_motion_t motion;
static unsigned frames;
static void step(unsigned dt)
{
    int y;
    /* The fixture expresses logical ring progress; the sensor retains the
     * actual raw-Z convention, which the core alone normalises. */
    ring_motion_t sensor=motion;
    motion.now_ms+=dt==65535u?20:dt;sensor.now_ms=motion.now_ms;
    sensor.yaw_deg=motion.yaw_deg/RING_YAW_SIGN;
    ring_observe_track(1);ring_observe_frame_interval((uint16_t)dt);
    memcpy(saved,image,sizeof(image));ring_update((const uint8_t (*)[RING_W])image,&sensor);++frames;
    CHECK(!memcmp(saved,image,sizeof(image)));
    if(ring_result.path_source==1) for(y=0;y<RING_H;++y)
        if(ring_result.mid[y]>=0) CHECK(image[y][ring_result.mid[y]]);
    CHECK((ring_result.guide_x==-1)==(ring_result.guide_y==255u));
}
static void start(int route,int advance)
{
    int i;
    ring_init();motion=(ring_motion_t){0,0,0,1};rg_entry(image,route,advance,0);
    for(i=0;i<RING_CONFIRM_FRAMES;++i) step(20);
    CHECK(ring_result.phase==RING_APPROACH && ring_result.branch_ok);
}
static void approach_orbit(int route,int last)
{
    int i;
    start(route,0);
    for(i=1;i<=last;++i) {
        rg_transition(image,route,i,12);motion.yaw_deg=route*4.0f*i;motion.wheel_revs+=0.025f;step(20);
        CHECK(ring_result.branch_ok && ring_result.path_source==1);
    }
}
static void circle(int route)
{
    int i;
    approach_orbit(route,12);rg_orbit(image,route,90);
    for(i=0;i<6;++i){motion.yaw_deg+=route*2;motion.wheel_revs+=0.01f;step(20);}
    CHECK(ring_result.phase==RING_CIRCLE && ring_result.committed);
}
static void partial_view(int route,int shift,int counterbow,int short_edge)
{
    int y,x;
    /* Keep actual pixels of two visible inner-edge patches; erase the
     * intervening island edge as it clips out of the selected corridor.
     * Each retained patch is shorter than the strong 25-row curve test. */
    for(y=24;y<=117;++y) {
        int keep=(y>=70 && y<=(short_edge?79:86)) || (y>=100 && y<=(short_edge?109:117));
        int edge=route>0?0:RING_W-1;
        if(route>0){while(edge<RING_W && !image[y][edge])++edge;}
        else{while(edge>=0 && !image[y][edge])--edge;}
        if(!keep) {
            if(route>0) for(x=0;x<94;++x) image[y][x]=255;
            else for(x=94;x<RING_W;++x) image[y][x]=255;
        } else if(edge>=0 && edge<RING_W) {
            int moved=edge+route*shift;
            if(counterbow && y>=70 && y<=86) {
                int local=y-70;
                moved-=route*(5*(8-abs(local-8))/8);
            }
            if(route>0) {
                for(x=0;x<moved;++x) image[y][x]=0;
                for(x=moved;x<edge;++x) image[y][x]=255;
            } else {
                for(x=moved+1;x<RING_W;++x) image[y][x]=0;
                for(x=edge+1;x<=moved;++x) image[y][x]=255;
            }
        }
    }
}
int main(int argc,char **argv)
{
    int route,i,guide_only=argc>1 && !strcmp(argv[1],"guide");
    if(!guide_only) for(route=-1;route<=1;route+=2) {
        int16_t old_error,held_error;
        circle(route);old_error=ring_result.steer_error;
        rg_orbit(image,route,90);partial_view(route,2,0,0);motion.yaw_deg+=route;motion.wheel_revs+=0.01f;step(20);
        /* This assertion fails v8 on actual pixels: it produces hold with
         * branch=0, rather than a path following the visible current edge. */
        CHECK(ring_result.branch_ok && ring_result.path_source==1);
        CHECK(ring_result.branch_mode==RING_BRANCH_PARTIAL && ring_result.inner_rows==35);
        CHECK(ring_result.associated_rows>=12 && ring_result.target_support==7);
        CHECK(ring_result.strong_age_ms==20 && ring_result.steer_error!=old_error);
        for(i=2;i<=5;++i) {
            rg_orbit(image,route,90);partial_view(route,2,0,0);
            motion.yaw_deg+=route;motion.wheel_revs+=0.01f;step(20);
            CHECK(ring_result.branch_mode==RING_BRANCH_PARTIAL && !ring_result.stop);
            CHECK(ring_result.strong_age_ms==(unsigned)(20*i));
        }
        held_error=ring_result.steer_error;step(20);
        CHECK(ring_result.stop && ring_result.path_source==0 && !ring_result.branch_ok);
        CHECK(ring_result.hold_reject&RING_HOLD_FRAMES_REJECT);
        CHECK(ring_result.steer_error==held_error && ring_result.phase==RING_CIRCLE);

        circle(route);
        for(i=0;i<3;++i){rg_orbit(image,route,90);partial_view(route,0,0,0);step(20);CHECK(ring_result.branch_mode==RING_BRANCH_PARTIAL);}
        rg_orbit(image,route,90);step(20);
        CHECK(ring_result.branch_mode==RING_BRANCH_STRONG && !ring_result.strong_age_ms);
        for(i=0;i<5;++i){rg_orbit(image,route,90);partial_view(route,0,0,0);step(20);CHECK(ring_result.branch_mode==RING_BRANCH_PARTIAL);}
        circle(route);
        for(i=0;i<4;++i){rg_orbit(image,route,90);partial_view(route,0,0,0);step(20);}
        held_error=ring_result.steer_error;memset(image,0,sizeof(image));step(20);
        CHECK(ring_result.branch_mode==RING_BRANCH_HOLD && ring_result.path_source==2);
        CHECK(ring_result.steer_error==held_error);step(20);
        CHECK(ring_result.stop && ring_result.path_source==0);
    }
    if(!guide_only) puts("PASS: current short-edge pixels steer; strong anchor age advances; weak and hold cannot renew or stack lifetimes");

    for(route=-1;route<=1;route+=2) {
        start(route,-17);
        /* Known pixel geometry: this island tip is row53, so row49 is
         * inside the selected branch while fixed row78 is on the bridge.
         * Check the actual steering against that measured white path point
         * BEFORE inspecting any newly added telemetry fields. */
        CHECK(ring_result.island_near==53 && ring_result.mid[49]>=0);
        CHECK(ring_result.steer_error==(ring_result.near_x-RING_W/2-1)+
              (ring_result.mid[49]-ring_result.near_x)*20/(RING_NEAR_ROW-49));
        CHECK(route*(ring_result.mid[49]-ring_result.near_x)>0);
        CHECK(ring_result.guide_y<RING_LOOK_ROW && ring_result.target_support&4u);
        CHECK(ring_result.guide_x==ring_result.mid[ring_result.guide_y]);
        CHECK(ring_result.steer_error==(ring_result.near_x-RING_W/2-1)+
              (ring_result.guide_x-ring_result.near_x)*20/(RING_NEAR_ROW-ring_result.guide_y));
        for(i=-16;i<=18;++i) {
            int16_t previous_error=ring_result.steer_error;
            rg_entry(image,route,i,0);step(20);
            CHECK(ring_result.branch_ok && ring_result.path_source==1 &&
                  ring_result.entry_flags&RING_ENTRY_MEASURED);
            CHECK((ring_result.branch_mode==RING_BRANCH_STRONG ||
                   ring_result.branch_mode==RING_BRANCH_ASSIST) && ring_result.target_support&4u);
            CHECK(abs(ring_result.steer_error-previous_error)<=RING_MAX_ROW_STEP);
        }
        CHECK(ring_result.guide_y==RING_LOOK_ROW);
        {
            int raw=(ring_result.near_x-RING_W/2-1)+
                (ring_result.look_x-ring_result.near_x)*2/3;
            int expected=raw;
            if((ring_result.entry_flags&RING_ENTRY_ASSIST) && abs(raw)<RING_ASSIST_MIN_ERROR)
                expected=raw<0?-RING_ASSIST_MIN_ERROR:RING_ASSIST_MIN_ERROR;
            CHECK(ring_result.steer_error==expected);
        }
    }
    puts("PASS: early entry guide has actual inner-edge support and converges to the unchanged fixed-look error weight");
    if(guide_only) return 0;

    for(route=-1;route<=1;route+=2) {
        uint8_t full_view[RING_H][RING_W];
        approach_orbit(route,9);memcpy(full_view,image,sizeof(image));
        CHECK(ring_result.phase==RING_ENTER && ring_result.committed);
        partial_view(route,0,0,0);motion.yaw_deg+=route*4;
        for(i=0;i<5;++i) {
            step(20);CHECK(ring_result.branch_mode==RING_BRANCH_PARTIAL);
            CHECK(ring_result.phase==RING_ENTER);
        }
        memcpy(image,full_view,sizeof(image));
        for(i=0;i<(int)RING_BRANCH_CONFIRM;++i) step(20);
        CHECK(ring_result.phase==RING_CIRCLE && ring_result.branch_mode==RING_BRANCH_STRONG);

        circle(route);rg_exit(image,route);motion.yaw_deg=route*350;motion.wheel_revs=1;
        step(20);CHECK(ring_result.phase==RING_CIRCLE);
        partial_view(route,0,0,0);motion.yaw_deg+=route*12;
        for(i=0;i<RING_EXIT_CONFIRM_FRAMES;++i) {
            step(20);CHECK(ring_result.branch_mode==RING_BRANCH_PARTIAL);
            CHECK(ring_result.phase==RING_CIRCLE && !ring_result.exited_events);
        }
        for(i=0;i<7;++i) {
            circle(route);rg_orbit(image,route,90);
            if(i==0) rg_road(image,0);
            if(i==1) rg_cross(image,0);
            if(i==2) {rg_orbit(image,-route,90);}
            if(i==3) partial_view(route,0,1,0);
            if(i==4) partial_view(route,0,0,1);
            if(i==5) {partial_view(route,0,0,0);motion.yaw_deg+=route*12.1f;}
            if(i==6) {partial_view(route,0,0,0);motion.wheel_revs+=0.401f;}
            step(20);CHECK(!ring_result.branch_ok && ring_result.branch_mode!=RING_BRANCH_PARTIAL);
        }
        circle(route);partial_view(route,0,0,0);step(251);
        CHECK(ring_result.stop && (ring_result.hold_reject&RING_HOLD_TIME));
        circle(route);partial_view(route,0,0,0);step(65535);
        CHECK(ring_result.stop && (ring_result.hold_reject&RING_HOLD_UNKNOWN_DT));
        circle(route);rg_orbit(image,route,90);
        for(i=0;i<40;++i){motion.yaw_deg+=route*2;motion.wheel_revs+=0.01f;step(20);CHECK(ring_result.branch_mode==RING_BRANCH_STRONG && !ring_result.strong_age_ms);}
    }
    puts("PASS: partial measurements supply control without advancing CIRCLE or exit confirmation");
    puts("PASS: ordinary/cross, opposite or short/reversed edge, unknown clock and original time/yaw/travel limits reject weak continuation");
    printf("PASS: %u branch-follow geometry frames (synthetic, not field replay)\n",frames);
    return 0;
}
