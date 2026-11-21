/* Cached geometry test: known commitment/lap are prepared, not inferred
 * from injected yaw. World-map pixels move with an actual camera pose. */
#include "../code/ring.c"
#include "../code/ring_exit_opening.h"
#include "ring_tangent_test_geometry.h"
#include <stdio.h>
static uint8_t picture[RING_H][RING_W];
static ring_motion_t movement;
static int direction;
static uint32_t maximum_checks;
static void annulus_only(int side,const rtg_pose_t *pose);
#define VERIFY(c) do{if(!(c)){fprintf(stderr,"FAIL v35 opening:%d %s side%d lap%.2f\n",__LINE__,#c,direction,ring_result.lap_deg);exit(1);}}while(0)
static void prepare(int side,float progress)
{
    direction=side;ring_init();ring_result.phase=RING_CIRCLE;
    ring_result.route=(int8_t)-side;ring_result.ring_side=(int8_t)side;
    ring_result.committed=ring_result.left_entry=ring_result.lap_valid=1;
    ring_result.lane_width=60;ring_result.visual_dt_ms=20;
    ring_result.lap_deg=progress;timed_started=1;maneuver_feedback=3;
    movement=(ring_motion_t){1000,side*progress,2.f,1};
}
static int probe(exit_probe_t *p)
{
    ring_result_t before;
    uint32_t scans;
    int result,y;
    scan_geometry((const uint8_t(*)[RING_W])picture,&movement);
    before=ring_result;scans=ring_scan_pixels;
    result=tangent_exit_opening(p,&movement);
    VERIFY(ring_opening_cache_checks<=48000u);
    if(ring_opening_cache_checks>maximum_checks)maximum_checks=ring_opening_cache_checks;
    VERIFY(scans==ring_scan_pixels && !memcmp(&before,&ring_result,sizeof(before)));
    if(result)for(y=0;y<RING_H;++y)if(p->mid[y]>=0)
        VERIFY(p->mid[y]<RING_W && picture[y][p->mid[y]]);
    return result;
}
static void observe_chain(int side,int first,int expect)
{
    rtg_pose_t pose=rtg_return_near(side,first);
    int frame;exit_probe_t original;
    prepare(side,side*pose.heading_deg);
    ring_since=phase_since=movement.now_ms;have_path=1;
    last_path_fault_ms=movement.now_ms;tangent_lap_revs=0.f;
    for(frame=0;frame<3;++frame) {
        pose=rtg_return_near(side,first+frame);
        ring_result.lap_deg=side*pose.heading_deg;movement.yaw_deg=pose.heading_deg;
        movement.now_ms+=50;movement.wheel_revs+=.03f;
        ++ring_result.frame_sequence;vision_ms+=50;ring_result.visual_dt_ms=50;
        if(expect==1)rtg_render(picture,side,&pose);
        else if(expect==2)annulus_only(side,&pose);
        else rtg_cross(picture);
        scan_geometry((const uint8_t(*)[RING_W])picture,&movement);
        if(expect==1)VERIFY(!tangent_exit_probe(&original,0,&movement));
        (void)tangent_exit_observe(&movement);
        if(frame<2 || expect!=1)VERIFY(ring_result.phase==RING_CIRCLE && !ring_result.exit_latched);
    }
    if(expect==1) {
        island_t island=find_island();
        VERIFY(ring_result.phase==RING_EXIT && ring_result.exit_latched &&
            ring_result.exit_turn_request && ring_result.exit_candidate_frames==3);
        VERIFY(!ring_result.path_valid && !ring_result.exited_events);
        ring_observe_exit_turn_feedback(0);
        tangent_active(&island,&movement,0);
        VERIFY(ring_result.phase==RING_EXIT && ring_result.exit_turn_request &&
            !ring_result.path_valid && !ring_result.branch_ok && !ring_result.release_reason);
    } else VERIFY(!ring_result.exit_latched && !ring_result.exit_turn_request);
}
static void world_scan(void)
{
    int side,i;
    for(side=-1;side<=1;side+=2) {
        exit_probe_t old;int have_old=0;
        for(i=24;i<=39;++i) {
            rtg_pose_t pose=rtg_return_near(side,i);exit_probe_t p;
            prepare(side,side*pose.heading_deg);rtg_render(picture,side,&pose);
            VERIFY(probe(&p));
            if(have_old)VERIFY(tangent_exit_opening_associated(&old,&p));
            old=p;have_old=1;
            VERIFY(!ring_result.path_valid && !ring_result.exited_events);
        }
    }
}
static void draw_road(float angle)
{
    int y,x;float a=angle*.01745329252f;
    for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x) {
        float d=(x-94.f)*cosf(a)-(117-y)*sinf(a);
        picture[y][x]=fabsf(d)<=29.5f?255:0;
    }
}
static void annulus_only(int side,const rtg_pose_t *pose)
{
    int y,x;float a=pose->heading_deg*.01745329252f;
    for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x) {
        float wx=pose->x+cosf(a)*(x-94.f)+sinf(a)*(119-y);
        float wy=pose->y-sinf(a)*(x-94.f)+cosf(a)*(119-y);
        float dx=wx-side*90.f,d2=dx*dx+wy*wy;
        picture[y][x]=d2>=3600.f && d2<=14400.f?255:0;
    }
}
static void negatives(void)
{
    int side,i,y,x;exit_probe_t p;
    for(side=-1;side<=1;side+=2) {
        rtg_pose_t pose=rtg_return_near(side,25);
        prepare(side,side*pose.heading_deg);rtg_render(picture,side,&pose);
        ring_result.ring_side=(int8_t)-side;VERIFY(!probe(&p));
        for(i=24;i<=39;++i) {
            pose=rtg_return_near(side,i);prepare(side,side*pose.heading_deg);
            annulus_only(side,&pose);VERIFY(!probe(&p));
        }
        for(i=0;i<3;++i) {
            prepare(side,300.f+i*20.f);draw_road(side*(60.f-i*20.f));VERIFY(!probe(&p));
            draw_road(0);VERIFY(!probe(&p));
            rtg_cross(picture);VERIFY(!probe(&p));
        }
        prepare(side,side*rtg_return_near(side,25).heading_deg);
        pose=rtg_return_near(side,25);rtg_render(picture,side,&pose);
        for(y=100;y<=106;++y)for(x=0;x<RING_W;++x)picture[y][x]=0;
        VERIFY(!probe(&p));
        rtg_render(picture,side,&pose);ring_result.committed=0;VERIFY(!probe(&p));
        ring_result.committed=1;ring_result.lap_valid=0;VERIFY(!probe(&p));
        ring_result.lap_valid=1;ring_result.visual_dt_ms=65535u;VERIFY(!probe(&p));
        ring_result.visual_dt_ms=20;ring_result.lap_deg=289.9f;VERIFY(!probe(&p));
        ring_result.lap_deg=360.f;VERIFY(!probe(&p));
    }
}
static void bounded_cache_work(void)
{
    unsigned seed=315360u;
    int trial,y,x;exit_probe_t p;
    for(trial=0;trial<90;++trial) {
        prepare(trial&1?1:-1,290.f+(trial%69));
        ring_result.lane_width=(int16_t)(18+trial%93);
        for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x) {
            seed=seed*1664525u+1013904223u;
            if(trial%3==0)picture[y][x]=(uint8_t)(((x+y/7)%16)<9?255:0);
            else if(trial%3==1)picture[y][x]=(uint8_t)((seed>>31)?255:0);
            else picture[y][x]=(uint8_t)(x>15 && x<173?255:0);
        }
        VERIFY(!probe(&p));
    }
}
int main(void)
{
    int side;
    world_scan();negatives();bounded_cache_work();
    for(side=-1;side<=1;side+=2) {
        observe_chain(side,24,1);observe_chain(side,28,1);
        observe_chain(side,24,2);observe_chain(side,28,0);
    }
    printf("maximum cached checks=%lu\n",(unsigned long)maximum_checks);
    puts("PASS: v35 cached early opening from290deg, original-heading association and mirrors; no pixel reread or path credit; wrong side, pure ring, straight, cross, disconnected road, uncommitted/stale/incomplete feedback rejected");
    return 0;
}
