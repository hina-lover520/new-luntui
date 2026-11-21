/* Frozen v31 frame geometry and history regressions.
 * History is explicitly seeded from the SAME image, so these tests do not
 * reconstruct a missing video or claim recognition / closed-loop success.
 * The test calls the actual included static implementation. */
#include "../code/ring.c"
#include "fixtures/v31_paths/v31_path_fixtures.h"
#include <stdio.h>

static uint8_t pixels[120][188],original[120][188];
static const v31_path_fixture_t *fixture;
static int mirror,width;
static unsigned observations;
#define CHECK(c) do {if(!(c)){fprintf(stderr,"FAIL v32 paths:%d %s fixture=%s mirror%d width%d route%d e%d\n",__LINE__,#c,fixture?fixture->name:"synthetic",mirror,width,ring_result.route,ring_result.steer_error);exit(1);}}while(0)

static uint32_t fixture_crc32(const uint8_t *bytes,size_t n)
{
    uint32_t crc=~0u;size_t i;int bit;
    for(i=0;i<n;++i){crc^=bytes[i];for(bit=0;bit<8;++bit)crc=(crc>>1)^(0xedb88320u&(0u-(crc&1u)));}
    return ~crc;
}

static void load(void)
{
    int i,x,y;uint8_t bits[2820];
    memcpy(bits,fixture->bits,sizeof(bits));
    CHECK(fixture_crc32(bits,sizeof(bits))==fixture->crc32);
    for(y=0;y<120;++y)for(x=0;x<188;++x){i=y*188+(mirror?187-x:x);pixels[y][x]=(uint8_t)((bits[i>>3]&(128>>(i&7)))?255:0);}
    memcpy(original,pixels,sizeof(pixels));
}

static island_t prepared_association(ring_motion_t *m)
{
    int route=(int)((fixture->flags>>16)&255u);island_t is;
    if(route==255)route=-1;
    CHECK(route==-1 || route==1);
    if(mirror)route=-route;
    load();ring_init();ring_result.route=(int8_t)route;ring_result.ring_side=(int8_t)-route;
    ring_result.phase=RING_CIRCLE;ring_result.approach_stage=RING_TANGENT_LEFT;
    ring_result.committed=1;ring_result.left_entry=1;ring_result.visual_dt_ms=18;
    timed_started=1;ring_result.lane_width=(int16_t)width;vision_ms=m->now_ms;
    scan_geometry((const uint8_t(*)[188])pixels,m);is=find_island();
    (void)build_path(&is,is.valid&&is.growing&&is.returning,0,0,0,m);
    memcpy(previous_mid,ring_result.mid,sizeof(previous_mid));memcpy(previous_inner,inner,sizeof(previous_inner));
    have_path=1;confirm_count=4;last_path_ms=last_path_fault_ms=m->now_ms;
    last_yaw=m->yaw_deg;support_last_revs=m->wheel_revs;
    seed_left=lane_left[117];seed_right=lane_right[117];seed_x=ring_result.mid[117];seed_inner=inner[117];
    trusted_near=ring_result.near_x;trusted_look=ring_result.look_x;ring_result.reject=0;
    scan_geometry((const uint8_t(*)[188])pixels,m);return find_island();
}

static void check_current_path(void)
{
    int y,points=0;
    CHECK(ring_result.path_valid && ring_result.near_x>=0);
    CHECK(ring_result.guide_y<=RING_NEAR_ROW-8 && ring_result.guide_x>=0);
    for(y=24;y<=117;++y)if(ring_result.mid[y]>=0){
        int x=ring_result.mid[y],j,white=0;
        CHECK(x>=3 && x<RING_W-3 && pixels[y][x]);
        for(j=0;j<row_count[y];++j)if(connected[y][j] && x>=row_runs[y][j].left+2 && x<=row_runs[y][j].right-2)white=1;
        CHECK(white);++points;
    }
    CHECK(points>=24);
    CHECK(!memcmp(pixels,original,sizeof(pixels)));
    CHECK(!ring_result.exited_events && !ring_result.detected_events);
}

static void partial_inner_gets_checked_tail(void)
{
    ring_motion_t m={10000,0,0,1};island_t is=prepared_association(&m);
    int y,start=-1;CHECK(build_path(&is,is.valid&&is.growing&&is.returning,0,0,0,&m)==1);
    for(y=117;y>=24;--y)if(inner[y]>2&&inner[y]<RING_W-3){start=y;break;}
    CHECK(start==93 && start<RING_NEAR_ROW);
    CHECK(tangent_normal_path());check_current_path();
    CHECK(ring_result.look_x>=0 && ring_result.guide_y<=RING_LOOK_ROW);
    CHECK(ring_result.mid[117]>=0);
    CHECK(ring_result.route*(ring_result.guide_x-RING_W/2)>0);
    /* The joining tail must be checked at every current white row. Removing
     * support in the middle must reject it, even with the original edge. */
    if(width>=85){
        memset(connected[112],0,sizeof(connected[112]));
        CHECK(!tangent_normal_path());
    }
    ++observations;
}

static void valid_opposite_correction_survives(void)
{
    ring_motion_t m={10000,0,0,1};island_t is=prepared_association(&m);int e;
    CHECK(build_path(&is,is.valid&&is.growing&&is.returning,0,0,0,&m)==1);
    CHECK(tangent_normal_path());check_current_path();e=ring_result.steer_error;
    CHECK(ring_result.look_x>=0 && ring_result.guide_y<=RING_LOOK_ROW);
    CHECK(ring_result.route*e>0); /* Opposite ring_side correction is allowed. */
    CHECK(abs(e)<=20);
    ring_result.timed_entry=0;CHECK(ring_blend_error(0)==e);
    ++observations;
}

static void steep_projection_uses_measured_guide(void)
{
    ring_motion_t m={10000,0,0,1};island_t is=prepared_association(&m);
    CHECK(build_path(&is,is.valid&&is.growing&&is.returning,0,0,0,&m)==1);
    CHECK(tangent_normal_path());check_current_path();
    /* The old n16 gate rejected thirteen source normals covering thirty
     * six actual connected image rows. A nearer measured guide is useful;
     * creating an unobserved row78 target would not be evidence. */
    CHECK(ring_result.guide_y==80 && ring_result.mid[78]<0);
    CHECK(ring_result.look_x==ring_result.guide_x);
    CHECK(ring_result.route*ring_result.steer_error>=15 && ring_result.route*ring_result.steer_error<=30);
    CHECK(ring_result.mid[117]>=0);
    ++observations;
}

static void short_projection_cannot_invent_tail_credit(void)
{
    ring_motion_t m={10000,0,0,1};island_t is=prepared_association(&m);
    CHECK(build_path(&is,is.valid&&is.growing&&is.returning,0,0,0,&m)==1);
    /* This frame supplies only 8..11 projected rows, depending on width.
     * A long vehicle join must never turn them into a measured curve. */
    CHECK(!tangent_normal_path());
    ++observations;
}

static void failed_normal_cannot_refresh(int during_entry)
{
    ring_motion_t m={10000,0,0,1};island_t is=prepared_association(&m);
    int16_t saved_mid[RING_H],saved_inner[RING_H],saved_strong[RING_H];
    uint32_t prior_ms=m.now_ms-40;int old_e;
    CHECK(build_path(&is,is.valid&&is.growing&&is.returning,0,0,0,&m)==1);
    old_e=ring_result.steer_error;CHECK(ring_result.route*old_e>20);
    CHECK(!tangent_normal_path());
    /* History markers distinguish a held prior measurement from publishing
     * the provisional row-wise build after normal projection failed. */
    trusted_steer=(int16_t)(-ring_result.route*7);
    last_path_ms=last_path_fault_ms=prior_ms;
    memcpy(saved_mid,previous_mid,sizeof(saved_mid));memcpy(saved_inner,previous_inner,sizeof(saved_inner));
    memcpy(saved_strong,strong_inner,sizeof(saved_strong));
    mouth_local_valid=0;mouth_local_ms=mouth_local_curve_ms=0;
    ring_result.reject=0;
    if(during_entry){
        ring_result.phase=RING_ENTER;ring_result.committed=0;ring_result.timed_entry=1;
        maneuver_feedback=2;phase_since=m.now_ms-100;ring_result.lap_valid=1;
        ring_result.lap_origin_yaw=0;tangent_left_count=0;
        /* Make the provisional target satisfy the independent entry-
         * departure gate. A zero-distance incoming reference would hide
         * the ignored-normal bug by rejecting this build for another reason. */
        mouth_entry_slope=0;mouth_entry_offset=(float)(RING_W/2+ring_result.route*200);
        CHECK(tangent_entry_branch(&m));
        (void)tangent_timed_entry(&is,&m,0);
    }else tangent_active(&is,&m,0);
    CHECK(!ring_result.branch_ok && !ring_result.path_valid);
    CHECK(last_path_ms==prior_ms && last_path_fault_ms==prior_ms);
    CHECK(!memcmp(saved_mid,previous_mid,sizeof(saved_mid)));
    CHECK(!memcmp(saved_inner,previous_inner,sizeof(saved_inner)));
    CHECK(!memcmp(saved_strong,strong_inner,sizeof(saved_strong)));
    CHECK(trusted_steer==-ring_result.route*7);
    CHECK(!mouth_local_valid && !mouth_local_ms && !mouth_local_curve_ms);
    CHECK(!ring_result.exited_events && !ring_result.detected_events);
    ++observations;
}

static void insufficient_real_inner_stays_untrusted(void)
{
    ring_motion_t m={10000,0,0,1};island_t is=prepared_association(&m);int y,n=0;
    CHECK(build_path(&is,is.valid&&is.growing&&is.returning,0,0,0,&m)==0);
    for(y=24;y<=117;++y)if(inner[y]>2&&inner[y]<RING_W-3)++n;
    CHECK(n<24 && !tangent_normal_path());
    ++observations;
}

static void short_near_tail_is_weak_and_bounded(void)
{
    ring_motion_t m={10000,0,0,1};island_t is=prepared_association(&m);
    int16_t saved_inner[RING_H],saved_strong[RING_H];
    uint32_t prior=m.now_ms-40;
    CHECK(build_path(&is,is.valid&&is.growing&&is.returning,0,0,0,&m)==1);
    CHECK(tangent_normal_path()==2);check_current_path();
    CHECK(ring_result.mid[117]>=0);
    have_path=0;CHECK(!tangent_normal_measurement(&m));
    have_path=1;last_path_ms=last_path_fault_ms=prior;
    CHECK(tangent_normal_measurement(&m)==2);
    memcpy(saved_inner,previous_inner,sizeof(saved_inner));memcpy(saved_strong,strong_inner,sizeof(saved_strong));
    mouth_local_valid=0;mouth_local_ms=mouth_local_curve_ms=0;
    tangent_left_count=RING_BRANCH_CONFIRM-1;ring_result.phase=RING_ENTER;ring_result.committed=0;
    ring_result.lap_valid=1;ring_result.lap_origin_yaw=-ring_result.ring_side*76.f;
    tangent_active(&is,&m,0);
    CHECK(ring_result.branch_ok && ring_result.path_valid && ring_result.branch_mode==RING_BRANCH_PARTIAL);
    CHECK(last_path_ms==prior && last_path_fault_ms==prior);
    CHECK(!memcmp(saved_inner,previous_inner,sizeof(saved_inner)) && !memcmp(saved_strong,strong_inner,sizeof(saved_strong)));
    CHECK(!ring_result.committed && tangent_left_count==0);
    CHECK(!mouth_local_curve_ms && !mouth_local_valid);
    vision_ms+=RING_PATH_HOLD_MS+1;CHECK(!tangent_normal_measurement(&m));
    CHECK(!ring_result.exited_events && !ring_result.detected_events);
    ++observations;
}

static void far_only_curve_cannot_invent_near_support(void)
{
    ring_motion_t m={10000,0,0,1};int y,route;
    fixture=NULL;width=60;
    for(route=-1;route<=1;route+=2){
        ring_init();ring_result.route=(int8_t)route;ring_result.lane_width=60;
        memset(pixels,255,sizeof(pixels));
        scan_geometry((const uint8_t(*)[188])pixels,&m);
        for(y=0;y<RING_H;++y)inner[y]=(int16_t)(y>=24&&y<=80?(route>0?50:137):-1);
        /* A long, valid normal segment entirely above row108 cannot earn
         * near evidence by joining 37 invented rows through broad white. */
        CHECK(!tangent_normal_path());
        CHECK(!have_path && !ring_result.committed && !ring_result.path_valid);
        ++observations;
    }
}

int main(void)
{
    unsigned i;
    for(i=0;i<V31_PATH_FIXTURE_COUNT;++i){fixture=&v31_path_fixtures[i];
        for(mirror=0;mirror<2;++mirror){
            if(fixture->id==100){for(width=60;width<=110;width+=25)partial_inner_gets_checked_tail();}
            else if(fixture->id==70){
                for(width=60;width<=110;width+=25)short_projection_cannot_invent_tail_credit();
                width=85;failed_normal_cannot_refresh(0);failed_normal_cannot_refresh(1);
            }
            else if(fixture->id==78){width=60;valid_opposite_correction_survives();}
            else if(fixture->id==8){width=85;steep_projection_uses_measured_guide();}
            else if(fixture->id==12){width=85;insufficient_real_inner_stays_untrusted();}
            else if(fixture->id==38){for(width=85;width<=110;width+=25)short_near_tail_is_weak_and_bounded();}
        }
    }
    far_only_curve_cannot_invent_near_support();
    printf("PASS: %u frozen-image/mirror paths; partial measured inner uses checked white tail, failed normal cannot refresh strong history, valid opposite correction survives\n",observations);
    return 0;
}
