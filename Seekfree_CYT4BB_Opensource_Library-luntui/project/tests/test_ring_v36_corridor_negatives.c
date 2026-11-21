/* Current cached geometry only: no yaw-fed success or fabricated branch. */
#include "../code/ring.c"
#include "ring_tangent_test_geometry.h"
#include <stdio.h>

static uint8_t frame[RING_H][RING_W];
static ring_motion_t motion;
static unsigned checks, failures, maximum_work;
static const char *case_name;
static int case_side;

static void prepare(int side,float lap)
{
    ring_init();
    ring_result.phase=RING_CIRCLE;
    ring_result.route=(int8_t)-side;ring_result.ring_side=(int8_t)side;
    ring_result.committed=ring_result.left_entry=ring_result.lap_valid=1;
    ring_result.lane_width=60;ring_result.visual_dt_ms=20;
    ring_result.lap_deg=lap;timed_started=1;maneuver_feedback=3;
    exit_net_reached=(uint8_t)(lap>=RING_EXIT_NET_MIN_DEG);
    motion=(ring_motion_t){1000,side*lap,2.f,1};
    case_side=side;
}

static void require(int ok,const char *what)
{
    ++checks;
    if(!ok) {
        fprintf(stderr,"FAIL v36 corridor negative %s side%d lap%.1f: %s\n",
            case_name,case_side,ring_result.lap_deg,what);
        ++failures;
    }
}

static void reject_current(void)
{
    exit_probe_t out;
    ring_result_t before;
    uint32_t pixel_before;
    int y,result;
    scan_geometry((const uint8_t(*)[RING_W])frame,&motion);
    before=ring_result;pixel_before=ring_scan_pixels;
    result=tangent_exit_corridor(&out,&motion);
    require(!result,"non-exit geometry was accepted");
    require(pixel_before==ring_scan_pixels,"cached probe reread image pixels");
    require(!memcmp(&before,&ring_result,sizeof(before)),"observation changed ring state");
    require(ring_corridor_cache_checks<=32000u,"cached work exceeded32000");
    if(ring_corridor_cache_checks>maximum_work)maximum_work=ring_corridor_cache_checks;
    if(result)for(y=0;y<RING_H;++y)if(out.mid[y]>=0)
        require(out.mid[y]<RING_W && frame[y][out.mid[y]],"returned target outside current white road");
}

static void annulus(int side,float lap)
{
    int y,x;
    float turn=(lap-360.f)*.01745329252f;
    rtg_pose_t pose={side*(90.f-90.f*cosf(turn)),90.f*sinf(turn),side*lap};
    float a=pose.heading_deg*.01745329252f;
    for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x) {
        float wx=pose.x+cosf(a)*(x-94.f)+sinf(a)*(119-y);
        float wy=pose.y-sinf(a)*(x-94.f)+cosf(a)*(119-y);
        float dx=wx-side*90.f,d2=dx*dx+wy*wy;
        frame[y][x]=(uint8_t)(d2>=3600.f && d2<=14400.f?255:0);
    }
}

static void wrong_heading(float angle)
{
    int x,y;float a=angle*.01745329252f;
    for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x) {
        float d=(x-94.f)*cosf(a)-(119-y)*sinf(a);
        frame[y][x]=(uint8_t)(fabsf(d)<=29.5f?255:0);
    }
}

static void geometry_negatives(void)
{
    int side,degree,x,y;
    for(side=-1;side<=1;side+=2) {
        for(degree=400;degree<=430;degree+=2) {
            prepare(side,(float)degree);annulus(side,(float)degree);
            case_name="annulus-only";reject_current();
        }
        prepare(side,400.f);rtg_cross(frame);
        case_name="cross";reject_current();
        prepare(side,400.f);rtg_straight(frame);
        for(y=45;y<=75;++y)for(x=0;x<RING_W;++x)
            if((side<0 && x<94) || (side>0 && x>94))frame[y][x]=255;
        case_name="side-T";reject_current();
        prepare(side,400.f);rtg_straight(frame);
        for(y=0;y<45;++y)memset(frame[y],0,RING_W);
        for(y=45;y<=75;++y)memset(frame[y],255,RING_W);
        case_name="facing-T";reject_current();
        prepare(side,400.f);memset(frame,255,sizeof(frame));
        case_name="all-white";reject_current();
        prepare(side,400.f);
        for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x)
            frame[y][x]=(uint8_t)(x>=12 && x<=175?255:0);
        case_name="wide";reject_current();
        prepare(side,420.f);wrong_heading(side*25.f);
        case_name="wrong-heading";reject_current();
        prepare(side,400.f);rtg_straight(frame);
        for(y=90;y<=96;++y)memset(frame[y],0,RING_W);
        case_name="disconnected";reject_current();
        prepare(side,400.f);rtg_straight(frame);
        for(y=103;y<=104;++y)memset(frame[y],0,RING_W);
        case_name="missing-near-control";reject_current();
    }
}

static void noisy_budget(void)
{
    int trial,y,x;unsigned seed=360390u;
    for(trial=0;trial<120;++trial) {
        prepare(trial&1?1:-1,400.f+(trial%31));
        ring_result.lane_width=(int16_t)(18+trial%93);
        for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x) {
            seed=seed*1664525u+1013904223u;
            if(trial%3==0)frame[y][x]=(uint8_t)(((x+y/5)%16)<8?255:0);
            else if(trial%3==1)frame[y][x]=(uint8_t)((seed>>31)?255:0);
            else frame[y][x]=(uint8_t)(((x+y/7)%17)<10?255:0);
        }
        case_name="noisy-budget";reject_current();
    }
}

int main(void)
{
    geometry_negatives();noisy_budget();
    require(maximum_work>0u,"negative maps bypassed the cached geometric probe");
    printf("v36 corridor negative checks=%u failures=%u maximum cached work=%u\n",
        checks,failures,maximum_work);
    if(failures)return 1;
    puts("PASS: v36 corridor rejects pure annulus, cross/T, wide, wrong heading, disconnected and noisy maps; cached work bounded without pixel reread or path-state changes");
    return 0;
}
