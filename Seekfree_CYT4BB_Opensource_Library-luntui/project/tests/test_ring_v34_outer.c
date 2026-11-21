/* Real saved pixels validate path construction. Prior commitment, motion
 * and history below are prepared inputs, not proof of driving a full lap. */
#include "../code/ring.c"
#include "../code/ring_outer_path.h"
#include "fixtures/v34_outer/v34_outer_snapshots.h"
#include <stdio.h>
static uint8_t picture[RING_H][RING_W],saved[RING_H][RING_W];
static ring_motion_t motion;
static int test_side,test_fixture;
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL v34 outer:%d %s side%d fixture%d lane%d frame%lu\n",__LINE__,#c,test_side,test_fixture,ring_result.lane_width,(unsigned long)ring_result.frame_sequence);exit(1);}}while(0)
static void prepare(int fixture,int side,int width)
{
    const unsigned char *bits=fixture?v34_outer_snapshot_1:v34_outer_snapshot_0;
    int y,x;test_side=side;test_fixture=fixture;ring_init();tangent_outer_reset();
    motion=(ring_motion_t){1000,side*90.f,2.f,1};vision_ms=1000;ring_since=500;
    ring_result.phase=RING_CIRCLE;ring_result.route=(int8_t)-side;ring_result.ring_side=(int8_t)side;
    ring_result.committed=ring_result.left_entry=ring_result.lap_valid=1;
    ring_result.lane_width=(uint8_t)width;ring_result.visual_dt_ms=20;
    ring_result.lap_deg=90.f;ring_result.detected_events=1;timed_started=1;maneuver_feedback=3;
    have_path=1;last_path_ms=last_path_fault_ms=1000;last_yaw=motion.yaw_deg;
    for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x){
        int bit=y*RING_W+x;picture[y][side<0?x:187-x]=(bits[bit>>3]&(128>>(bit&7)))?255:0;
    }
    memcpy(saved,picture,sizeof(picture));
}
static int frame(void)
{
    unsigned scans=ring_scan_pixels;
    ++ring_result.frame_sequence;vision_ms+=20;motion.now_ms+=20;
    scan_geometry((const uint8_t(*)[188])picture,&motion);
    CHECK(ring_scan_pixels-scans<=RING_W*RING_H*2u);
    return tangent_outer_path(&motion);
}
static void check_path(void)
{
    int y,n=0;
    CHECK(ring_result.near_x>=0 && ring_result.look_x>=0 && ring_result.guide_y<100);
    for(y=24;y<=117;++y)if(ring_result.mid[y]>=0){
        int x=ring_result.mid[y],margin=ring_result.lane_width/8;
        CHECK(picture[y][x] && picture[y][x-margin] && picture[y][x+margin]);++n;
    }
    CHECK(n>=30 && !memcmp(picture,saved,sizeof(picture)));
    {
        int k,gy=ring_result.guide_y,gx=ring_result.guide_x;
        float distance=100000.f,target=ring_result.lane_width*.5f;
        for(k=24;k<=117;++k) {
            int e=ring_result.route>0?observed_right[k]:observed_left[k];
            float dx=(float)(e-gx),dy=(float)(k-gy),d=dx*dx+dy*dy;
            if(e>2 && e<RING_W-3 && d<distance)distance=d;
        }
        CHECK(fabsf(sqrtf(distance)-target)<=4.f);
    }
}
static void snapshots(void)
{
    int fixture,side,width,i;int16_t original_inner[RING_H],original_strong[RING_H];
    for(fixture=0;fixture<2;++fixture)for(side=-1;side<=1;side+=2)for(width=50;width<=80;width+=10){
        prepare(fixture,side,width);
        for(i=0;i<RING_H;++i)previous_inner[i]=strong_inner[i]=(int16_t)(40+i%9);
        memcpy(original_inner,previous_inner,sizeof(original_inner));memcpy(original_strong,strong_inner,sizeof(original_strong));
        CHECK(frame()==0); /* First observed outside edge cannot take over. */
        CHECK(!tangent_outer_fresh(&motion));
        CHECK(frame()==3);check_path();
        tangent_store_path(&motion,0);tangent_outer_store(&motion);
        CHECK(tangent_outer_fresh(&motion));
        for(i=0;i<80;++i){
            CHECK(frame()==3);check_path();
            tangent_store_path(&motion,0);tangent_outer_store(&motion);
        }
        CHECK(last_path_ms==1000 && last_path_fault_ms==motion.now_ms);
        CHECK(!memcmp(previous_inner,original_inner,sizeof(original_inner)) &&
              !memcmp(strong_inner,original_strong,sizeof(original_strong)));
        CHECK(ring_result.committed && ring_result.left_entry && ring_result.phase==RING_CIRCLE &&
              ring_result.detected_events==1 && ring_result.exited_events==0);
    }
}
static void negative_cases(void)
{
    int y,x;
    prepare(0,-1,60);ring_result.committed=0;CHECK(frame()==0);
    prepare(0,-1,60);ring_result.left_entry=0;CHECK(frame()==0);
    prepare(0,-1,60);maneuver_feedback=2;CHECK(frame()==0);
    prepare(0,-1,60);ring_result.phase=RING_ENTER;CHECK(frame()==0);
    prepare(0,-1,60);ring_result.visual_dt_ms=65535;CHECK(frame()==0);
    prepare(0,-1,60);CHECK(frame()==0);CHECK(frame()==3);tangent_outer_store(&motion);
    motion.now_ms+=RING_PATH_HOLD_MS+1u;CHECK(!tangent_outer_fresh(&motion));CHECK(frame()==0);
    prepare(0,-1,60);CHECK(frame()==0);CHECK(frame()==3);tangent_outer_store(&motion);
    motion.yaw_deg+=13.f;CHECK(!tangent_outer_fresh(&motion));CHECK(frame()==0);
    prepare(0,-1,60);CHECK(frame()==0);CHECK(frame()==3);tangent_outer_store(&motion);
    motion.wheel_revs+=.41f;CHECK(!tangent_outer_fresh(&motion));CHECK(frame()==0);
    prepare(0,-1,60);CHECK(frame()==0);CHECK(frame()==3);tangent_outer_store(&motion);
    for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x)picture[y][x]=(x>=64 && x<=123)?255:0;
    CHECK(frame()==0);motion.yaw_deg+=45.f;CHECK(frame()==0);
    memset(picture,0,sizeof(picture));CHECK(frame()==0);
    memset(picture,255,sizeof(picture));CHECK(frame()==0);
}
static void integrated_follow(void)
{
    int fixture,side,i;
    for(fixture=0;fixture<2;++fixture)for(side=-1;side<=1;side+=2) {
        prepare(fixture,side,60);
        for(i=0;i<80;++i) {
            island_t island;
            ++ring_result.frame_sequence;vision_ms+=20;motion.now_ms+=20;
            scan_geometry((const uint8_t(*)[188])picture,&motion);island=find_island();
            ring_result.reject=0;tangent_active(&island,&motion,0);
            CHECK(ring_result.phase==RING_CIRCLE && ring_result.exited_events==0);
            if(i>=1) {
                CHECK(ring_result.branch_ok && ring_result.path_source==7);
                CHECK(last_path_fault_ms==motion.now_ms);
                check_path();
            }
        }
    }
}
int main(void)
{
    snapshots();negative_cases();integrated_follow();
    puts("PASS: v34 two real clipped-inner frames and mirrors, four widths, current outside normal path, two-frame identity, persistent measured control, preserved strong-inner history; entry/straight/blank/wide/time/yaw/travel rejected");
    return 0;
}
