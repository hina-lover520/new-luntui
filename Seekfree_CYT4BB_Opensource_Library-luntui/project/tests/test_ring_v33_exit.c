/* Exit state/geometry unit tests, not a reconstructed car trajectory.
 * Angles, prior commitment and wheel travel are explicitly prepared inputs.
 * Each probe still receives actual current row runs from scan_geometry().
 * Nothing here proves recognition or closed-loop driving a complete lap. */
#include "../code/ring.c"
#include <stdio.h>

static uint8_t pixels[120][188],before[120][188];
static ring_motion_t motion;
static int test_side,test_frame;
static unsigned observations;
static const char *test_name="setup";
#define CHECK(c) do {if(!(c)){fprintf(stderr,"FAIL v33 exit:%d %s case=%s side%d frame%d phase%d lap%.1f\n",__LINE__,#c,test_name,test_side,test_frame,ring_result.phase,ring_result.lap_deg);exit(1);}}while(0)

static void white_run(int y,int left,int right)
{
    int x;for(x=left;x<=right;++x)pixels[y][test_side<0?x:187-x]=255;
}

/* A 60px measured outgoing corridor. The optional side road remains
 * connected through a 16row mouth, with a dark island alongside the far
 * exit corridor. The main corridor remains available in every row.
 * An unrelated side island therefore cannot be vetoed by row_count>1. */
static void exit_picture(int side_island,int offset)
{
    int y;memset(pixels,0,sizeof(pixels));
    for(y=0;y<120;++y)white_run(y,64+offset,123+offset);
    if(side_island){
        for(y=24;y<70;++y)white_run(y,8+offset,39+offset);
        for(y=70;y<=85;++y)white_run(y,8+offset,123+offset);
    }
}

static void disconnected_picture(void)
{
    int y;memset(pixels,0,sizeof(pixels));
    for(y=98;y<120;++y)white_run(y,64,123);
    for(y=24;y<=84;++y)white_run(y,64,123);
    /* Full-width dark separator85..97 deliberately disconnects a convincing
     * distant road from the current near road. */
}

static void run_geometry(void)
{
    ring_result.visual_dt_ms=20;
    ++ring_result.frame_sequence;vision_ms+=20;
    scan_geometry((const uint8_t(*)[188])pixels,&motion);
    memcpy(before,pixels,sizeof(pixels));
}

static void prepare(int side,float degrees)
{
    int y;test_side=side;test_frame=0;ring_init();
    motion=(ring_motion_t){1000,side*degrees,1.0f,1};
    ring_result.phase=RING_CIRCLE;ring_result.route=(int8_t)-side;
    ring_result.ring_side=(int8_t)side;ring_result.entry_side=(int8_t)side;
    CHECK(side!= -1 || (ring_result.route==1 && motion.yaw_deg<0));
    ring_result.approach_stage=RING_TANGENT_LEFT;
    ring_result.committed=ring_result.left_entry=ring_result.lap_valid=1;
    ring_result.lane_width=60;ring_result.lap_deg=degrees;ring_result.lap_origin_yaw=0;
    maneuver_feedback=3;timed_started=1;lap_completed=degrees>=360;
    have_path=1;last_path_ms=vision_ms;last_path_fault_ms=motion.now_ms;
    tangent_lap_revs=0;ring_since=motion.now_ms;phase_since=motion.now_ms;
    for(y=0;y<120;++y)ring_result.mid[y]=(int16_t)(70+y%7);
}

static void set_progress(float degrees)
{
    ring_result.lap_deg=degrees;motion.yaw_deg=test_side*degrees;
    if(degrees>=360)lap_completed=1;
}

static void advance(void)
{
    ++test_frame;motion.now_ms+=50;motion.wheel_revs+=.01f;
}

static void check_probe_path(const exit_probe_t *path)
{
    int y,n=0;
    CHECK(path->near_x>=0 && path->look_x>=0);
    CHECK(path->far_rows>0 && path->near_rows>0);
    for(y=24;y<=117;++y)if(path->mid[y]>=0){
        int x=path->mid[y],j,white=0;
        CHECK(x>=2 && x<RING_W-2 && pixels[y][x]);
        for(j=0;j<row_count[y];++j)if(connected[y][j] &&
            x>=row_runs[y][j].left+2 && x<=row_runs[y][j].right-2)white=1;
        CHECK(white);++n;
    }
    CHECK(n>=35);
    CHECK(!memcmp(pixels,before,sizeof(pixels)));
}

static void probe_does_not_steer_early(int side)
{
    int16_t saved_mid[RING_H];exit_probe_t candidate;
    test_name="340 degree observation is read-only";prepare(side,340);
    exit_picture(1,0);run_geometry();memcpy(saved_mid,ring_result.mid,sizeof(saved_mid));
    CHECK(tangent_exit_probe(&candidate,0,&motion));check_probe_path(&candidate);
    CHECK(!memcmp(saved_mid,ring_result.mid,sizeof(saved_mid)));
    CHECK(ring_result.phase==RING_CIRCLE && !ring_result.exit_latched);
    CHECK(!exit_start_ms && !ring_result.exited_events);
    ++observations;
}

static void candidate_only(int side)
{
    int i;int16_t saved_mid[RING_H];
    test_name="350 degree unrelated heading cannot exit";prepare(side,350);
    memcpy(saved_mid,ring_result.mid,sizeof(saved_mid));
    for(i=0;i<6;++i){advance();exit_picture(1,i&1);run_geometry();CHECK(!tangent_exit_observe(&motion));
        CHECK(ring_result.phase==RING_CIRCLE && !ring_result.exit_latched);
        CHECK(!memcmp(saved_mid,ring_result.mid,sizeof(saved_mid)));
        CHECK(!exit_start_ms && !ring_result.exited_events);
    }
    CHECK(!exit_candidate_count && !ring_result.exit_candidate &&
        (ring_result.exit_reject&RING_EXIT_REJ_HEADING));
    ++observations;
}

static void candidate_needs_time_and_frames(int side)
{
    int i;test_name="candidate needs90ms even with enough frames";prepare(side,360);
    for(i=0;i<5;++i){
        ++test_frame;motion.now_ms+=20;motion.wheel_revs+=.002f;
        exit_picture(0,0);run_geometry();CHECK(tangent_exit_observe(&motion));
        CHECK(ring_result.phase==RING_CIRCLE && !ring_result.exit_latched);
        CHECK(!exit_start_ms && !ring_result.exited_events);
    }
    CHECK(exit_candidate_count>=RING_EXIT_CANDIDATE_FRAMES);
    ++test_frame;motion.now_ms+=20;motion.wheel_revs+=.002f;
    exit_picture(0,0);run_geometry();CHECK(tangent_exit_observe(&motion));
    CHECK(ring_result.phase==RING_EXIT && ring_result.exit_latched);
    CHECK(!ring_result.exit_confirmed && !ring_result.exited_events);
    ++observations;
}

static void rotated_exit_picture(float yaw_delta);
static void reach_exit(int side,float degrees,int side_island)
{
    int i;int16_t saved_mid[RING_H];
    prepare(side,degrees);memcpy(saved_mid,ring_result.mid,sizeof(saved_mid));
    for(i=0;i<6 && !ring_result.exit_latched;++i){
        advance();
        if(degrees>368.f)rotated_exit_picture(side*(degrees-360.f));
        else exit_picture(side_island,i&1);
        run_geometry();
        CHECK(tangent_exit_observe(&motion));
        CHECK(!memcmp(saved_mid,ring_result.mid,sizeof(saved_mid)));
        if(i<2)CHECK(ring_result.phase==RING_CIRCLE && !ring_result.exit_latched);
    }
    CHECK(ring_result.phase==RING_EXIT && ring_result.exit_latched);
    CHECK(exit_start_ms && !ring_result.exit_confirmed && !ring_result.exited_events);
    check_probe_path(&exit_locked_path);
}

static void full_lap_corridor_latches(int side,float degrees,int side_island)
{
    test_name=side_island?"side island does not veto selected exit":"full lap selected exit";
    reach_exit(side,degrees,side_island);
    tangent_exit_apply(&exit_locked_path);
    CHECK(!memcmp(ring_result.mid,exit_locked_path.mid,sizeof(ring_result.mid)));
    CHECK(ring_result.path_valid && ring_result.near_x==exit_locked_path.near_x);
    CHECK(!memcmp(pixels,before,sizeof(pixels)));
    ++observations;
}

static void initial_bad_angle_is_rejected(int side,float degrees)
{
    int i;test_name=degrees>500?"540 degrees cannot acquire first exit":"380 degrees cannot acquire first exit";
    prepare(side,degrees);
    for(i=0;i<6;++i){advance();exit_picture(1,i&1);run_geometry();
        (void)tangent_exit_observe(&motion);
        CHECK(ring_result.phase==RING_CIRCLE && !ring_result.exit_latched);
        CHECK(!exit_start_ms && !ring_result.exited_events);
    }
    ++observations;
}

/* A fixed world straight corridor seen after a measured yaw change.
 * Inverse rotation keeps current image geometry consistent with the yaw
 * used by the exit association; prescribing an unchanged bitmap at380deg
 * would accidentally test an unrelated road. */
static void rotated_exit_picture(float yaw_delta)
{
    int x,y;float radians=yaw_delta*.01745329252f,c=cosf(radians),s=sinf(radians);
    memset(pixels,0,sizeof(pixels));
    for(y=0;y<120;++y)for(x=0;x<188;++x){
        float local_x=c*(x-RING_W/2)+s*(119-y);
        if(local_x>=-30.f && local_x<30.f)pixels[y][x]=255;
    }
}

static void locked_exit_remains_current_past_heading_window(int side)
{
    float degrees;test_name="latched exit continues to380 degrees";reach_exit(side,360,0);
    for(degrees=365;degrees<=380;degrees+=5){
        advance();set_progress(degrees);rotated_exit_picture(side*(degrees-360));run_geometry();
        CHECK(tangent_exit_continue(&motion));
        CHECK(ring_result.phase==RING_EXIT && ring_result.exit_latched);
        CHECK(!ring_result.exit_confirmed && !ring_result.exited_events);
        check_probe_path(&exit_locked_path);
    }
    ++observations;
}

static void far_road_must_connect_to_vehicle(int side)
{
    exit_probe_t path;test_name="disconnected far road is rejected";prepare(side,360);
    disconnected_picture();run_geometry();
    CHECK(!tangent_exit_probe(&path,0,&motion));
    CHECK(!tangent_exit_observe(&motion));
    CHECK(ring_result.phase==RING_CIRCLE && !ring_result.exit_latched);
    CHECK(!exit_candidate_count && !exit_start_ms);
    ++observations;
}

static void old_exit_is_not_current_white(int side)
{
    exit_probe_t path;test_name="vanished current exit is not fabricated";reach_exit(side,360,1);
    advance();memset(pixels,0,sizeof(pixels));run_geometry();
    CHECK(!tangent_exit_probe(&path,1,&motion));
    CHECK(!tangent_exit_continue(&motion));
    CHECK(!ring_result.exit_confirmed && !ring_result.exited_events);
    CHECK(!memcmp(pixels,before,sizeof(pixels)));
    ++observations;
}

static void shifted_road_does_not_replace_locked_exit(int side)
{
    test_name="unrelated shifted corridor cannot replace locked exit";reach_exit(side,360,0);
    advance();exit_picture(0,32);run_geometry();
    CHECK(!tangent_exit_continue(&motion));
    CHECK(!ring_result.exit_confirmed && !ring_result.exited_events);
    CHECK(!memcmp(pixels,before,sizeof(pixels)));
    ++observations;
}

static void locked_exit_bounds_survive(int side)
{
    int mode;test_name="locked exit has yaw time and signed forward bounds";
    for(mode=0;mode<3;++mode){
        reach_exit(side,360,0);advance();
        if(mode==0){set_progress(391);rotated_exit_picture(side*31.f);}
        else if(mode==1){motion.now_ms=exit_start_ms+RING_EXIT_TIMEOUT_MS+1;exit_picture(0,0);}
        else {motion.wheel_revs=exit_start_revs-.003f;exit_picture(0,0);}
        run_geometry();CHECK(!tangent_exit_continue(&motion));
        CHECK(!ring_result.exit_confirmed && !ring_result.exited_events);
        CHECK(!memcmp(pixels,before,sizeof(pixels)));
    }
    ++observations;
}

static void entry_and_forward_gates_survive(int side)
{
    int i,mode;test_name="entry stage commitment and signed forward gates";
    for(mode=0;mode<3;++mode){
        prepare(side,360);
        if(mode==0)ring_result.committed=0;
        else if(mode==1)maneuver_feedback=2;
        else motion.wheel_revs=-1;
        for(i=0;i<6;++i){advance();exit_picture(1,i&1);run_geometry();(void)tangent_exit_observe(&motion);}
        CHECK(ring_result.phase==RING_CIRCLE && !ring_result.exit_latched);
        CHECK(!exit_start_ms && !ring_result.exited_events);
    }
    ++observations;
}

int main(void)
{
    int side;CHECK(RING_MANEUVER_TURN_DEG==75.f && RING_MANEUVER_DELAY_MS==800u);
    for(side=-1;side<=1;side+=2){
        probe_does_not_steer_early(side);candidate_only(side);candidate_needs_time_and_frames(side);
        full_lap_corridor_latches(side,360,0);full_lap_corridor_latches(side,375,0);
        full_lap_corridor_latches(side,360,1);
        initial_bad_angle_is_rejected(side,380);initial_bad_angle_is_rejected(side,540);
        locked_exit_remains_current_past_heading_window(side);
        far_road_must_connect_to_vehicle(side);old_exit_is_not_current_white(side);
        shifted_road_does_not_replace_locked_exit(side);locked_exit_bounds_survive(side);
        entry_and_forward_gates_survive(side);
    }
    printf("PASS: %u v33 exit geometry/state cases including mirrors; candidate observation never steers early, first exit requires full lap, locked current corridor survives380deg, disconnected or vanished white road is rejected\n",observations);
    return 0;
}
