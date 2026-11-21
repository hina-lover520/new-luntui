#include "../code/ring.c"
#include <stdio.h>
static uint8_t frame[RING_H][RING_W];
static ring_motion_t move;
static int side;
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL v36 corridor:%d %s side%d lap%.2f\n",__LINE__,#c,side,ring_result.lap_deg);exit(1);}}while(0)
static void road(float angle,int crop)
{
    int x,y;float a=angle*.01745329252f;
    for(y=0;y<RING_H;++y)for(x=0;x<RING_W;++x) {
        float d=(x-(94.f+(crop?side*20.f:0.f)))*cosf(a)-(108.f-y)*sinf(a);
        frame[y][x]=(uint8_t)(fabsf(d)<=29.5f?255:0);
        if(crop && y>=102 && (side>0?x<94:x>94))frame[y][x]=255;
    }
}
static void prepare(float lap)
{
    ring_init();move=(ring_motion_t){1000,side*lap,2.f,1};
    ring_result.phase=RING_CIRCLE;ring_result.route=(int8_t)-side;ring_result.ring_side=(int8_t)side;
    ring_result.committed=ring_result.left_entry=ring_result.lap_valid=1;
    ring_result.lane_width=60;ring_result.visual_dt_ms=20;ring_result.lap_deg=lap;
    timed_started=1;maneuver_feedback=3;lap_completed=lap>=360.f;
    tangent_lap(&move);
    ring_since=phase_since=last_path_fault_ms=move.now_ms;last_path_ms=vision_ms;
}
static int corridor(exit_probe_t *p)
{
    uint32_t scans;int ok,y;
    scan_geometry((const uint8_t(*)[RING_W])frame,&move);scans=ring_scan_pixels;
    ok=tangent_exit_corridor(p,&move);
    CHECK(ring_corridor_cache_checks<=32000u && ring_scan_pixels==scans);
    if(ok)for(y=0;y<RING_H;++y)if(p->mid[y]>=0)CHECK(frame[y][p->mid[y]]);
    return ok;
}
static void late_crop(float lap)
{
    int i;exit_probe_t p;
    prepare(lap);road(side*(360.f-lap),1);
    scan_geometry((const uint8_t(*)[RING_W])frame,&move);
    CHECK(!tangent_exit_probe(&p,0,&move));
    CHECK(corridor(&p));
    CHECK(p.near_x>=0 && p.look_x>=0 && !ring_result.path_valid);
    for(i=0;i<3;++i) {
        move.now_ms+=50;move.wheel_revs+=.03f;vision_ms+=50;
        (void)tangent_exit_observe(&move);
        if(i<2)CHECK(!ring_result.exit_latched);
    }
    CHECK(ring_result.exit_latched && ring_result.phase==RING_EXIT && !ring_result.exited_events);
    CHECK(ring_result.exit_turn_request);
    CHECK(tangent_exit_continue(&move));
    CHECK(ring_result.path_valid);
}
static void negatives(void)
{
    int y,x;exit_probe_t p;
    prepare(400.f);road(side*35.f,1);CHECK(!corridor(&p));
    prepare(400.f);road(side*(-40.f),1);
    for(y=88;y<96;++y)for(x=0;x<RING_W;++x)frame[y][x]=0;
    CHECK(!corridor(&p));
    prepare(350.f);road(side*10.f,1);CHECK(!corridor(&p));
    prepare(431.f);road(side*(-71.f),1);CHECK(!corridor(&p));
    prepare(400.f);memset(frame,255,sizeof(frame));CHECK(!corridor(&p));
    road(side*(-40.f),1);ring_result.committed=0;CHECK(!corridor(&p));
    ring_result.committed=1;ring_result.visual_dt_ms=65535u;CHECK(!corridor(&p));
}
static void heading_gate(void)
{
    int i;prepare(410.f);road(0.f,0);
    scan_geometry((const uint8_t(*)[RING_W])frame,&move);
    for(i=0;i<8;++i) {move.now_ms+=50;(void)tangent_exit_observe(&move);}
    CHECK(!ring_result.exit_latched && (ring_result.exit_reject&RING_EXIT_REJ_HEADING));
}
static void net_gate_history(void)
{
    int i,y;
    prepare(399.f);road(side*(-39.f),0);
    scan_geometry((const uint8_t(*)[RING_W])frame,&move);
    for(i=0;i<8;++i) {move.now_ms+=50;(void)tangent_exit_observe(&move);}
    CHECK(!exit_net_reached && !ring_result.exit_latched &&
        (ring_result.exit_reject&RING_EXIT_REJ_LAP));
    /* Reversing back and forth cannot accumulate400: it is net yaw. */
    move.yaw_deg=side*250.f;tangent_lap(&move);
    move.yaw_deg=side*399.f;tangent_lap(&move);CHECK(!exit_net_reached);
    move.yaw_deg=side*400.f;tangent_lap(&move);CHECK(exit_net_reached);
    move.yaw_deg=side*360.f;tangent_lap(&move);road(0.f,0);
    scan_geometry((const uint8_t(*)[RING_W])frame,&move);
    for(i=0;i<3;++i) {move.now_ms+=50;move.wheel_revs+=.03f;(void)tangent_exit_observe(&move);}
    CHECK(ring_result.exit_latched && !ring_result.exit_turn_request &&
        ring_result.phase==RING_EXIT && !ring_result.exited_events);
    /* Current road, matching heading, positive travel and stable handover
     * are still needed after unlocking.400 alone grants no success. */
    for(i=0;i<30 && !ring_result.exited_events;++i) {
        island_t island;move.now_ms+=20;move.wheel_revs+=.03f;
        vision_ms+=20;ring_result.visual_dt_ms=20;
        scan_geometry((const uint8_t(*)[RING_W])frame,&move);island=find_island();
        tangent_active(&island,&move,0);
        for(y=0;y<RING_H;++y)if(ring_result.mid[y]>=0)CHECK(frame[y][ring_result.mid[y]]);
    }
    CHECK(ring_result.exited_events==1u && ring_result.phase==RING_COOLDOWN &&
        ring_result.exit_confirmed && ring_speed_limit()==-1);
    ring_reset_episode();CHECK(!exit_net_reached && !ring_result.exit_latched);
}
static void forced_endpoint_crossing(void)
{
    int i;
    late_crop(400.f);
    /* One46ms step can cross the target. Existing8deg final alignment,
     * not the2deg wrong-way budget, bounds this endpoint tolerance. */
    move.yaw_deg=side*357.9f;road(side*2.1f,0);
    ring_observe_exit_turn_feedback(2u);
    for(i=0;i<30 && !ring_result.exited_events;++i) {
        island_t island;move.now_ms+=46;move.wheel_revs+=.069f;
        vision_ms+=46;ring_result.visual_dt_ms=46;
        scan_geometry((const uint8_t(*)[RING_W])frame,&move);island=find_island();
        tangent_active(&island,&move,0);
        CHECK(!ring_result.release_reason);
    }
    CHECK(ring_result.exited_events==1u && ring_result.exit_confirmed);
    late_crop(400.f);move.yaw_deg=side*351.f;road(side*9.f,0);
    ring_observe_exit_turn_feedback(2u);move.now_ms+=46;vision_ms+=46;
    scan_geometry((const uint8_t(*)[RING_W])frame,&move);
    {island_t island=find_island();tangent_active(&island,&move,0);}
    CHECK(ring_result.release_reason && !ring_result.exited_events);
}
int main(void)
{
    for(side=-1;side<=1;side+=2) {
        late_crop(400.f);late_crop(402.f);late_crop(405.f);late_crop(410.f);
        negatives();heading_gate();net_gate_history();forced_endpoint_crossing();
    }
    puts("PASS: v36 cropped current exit corridor400/402/405/410 and mirrors; wrong-heading, stale, disconnected, incomplete, all-white, out-of-window rejected; no pixel reread; cache32000 bound");
    return 0;
}
