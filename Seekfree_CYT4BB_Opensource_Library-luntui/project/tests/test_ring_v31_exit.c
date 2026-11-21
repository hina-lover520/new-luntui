/* v31 exit guards only. Prepared vision history is not a field-lap claim.
 * The paired runtime portion uses integrated gyro and counts event beeps.
 * Build with ring_runtime.c and ring_link.c; ring.c is included here. */
#include "../code/ring.c"
#include "ring_runtime.h"
#include "ring_link.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL v31 exit:%d %s side%d ph%d lap%.3f src%u\n", __LINE__, #c, test_side, ring_result.phase, ring_result.lap_deg, ring_result.path_source); exit(1); } } while (0)
static int test_side;
static uint8_t test_pixels[RING_H][RING_W];
static ring_motion_t test_motion;
static uint32_t test_sequence, test_epoch;
static unsigned test_beeps;
static float test_speed, test_turn, test_revs;

/* Ordinary road bends its image bearing away from the old entry turn.
 * Both boundaries and the lane width remain visible over the whole image. */
static void test_ordinary(int side)
{
    int x,y;
    memset(test_pixels,0,sizeof(test_pixels));
    for (y=0;y<RING_H;++y) {
        int l=66+(117-y+5)/10;
        for (x=l;x<l+60;++x)
            test_pixels[y][side<0?x:RING_W-1-x]=255;
    }
}
static void test_prepare(int side,float degrees,int shared)
{
    test_side=side;ring_init();test_ordinary(side);
    test_motion=(ring_motion_t){100,side*degrees,2,1};
    ring_result.phase=RING_CIRCLE;
    ring_result.ring_side=(int8_t)side;ring_result.route=(int8_t)-side;
    ring_result.committed=ring_result.left_entry=ring_result.lap_valid=1;
    ring_result.lap_origin_yaw=0;
    ring_result.approach_stage=(uint8_t)(shared?RING_TANGENT_SHARED:RING_TANGENT_LEFT);
    ring_result.detected_events=1;ring_since=phase_since=100;
    timed_started=1;tangent_lap_revs=1;
    ring_observe_maneuver_feedback(3,(int16_t)(degrees*10));
    CHECK((side==-1 && ring_result.route==1) || (side==1 && ring_result.route==-1));
}
static void test_vision_step(void)
{
    test_motion.now_ms+=20;test_motion.wheel_revs+=.02f;
    ring_observe_frame_interval(20);ring_observe_track(1);
    ring_update((const uint8_t(*)[RING_W])test_pixels,&test_motion);
}
static void test_vision_reject(int side,int kind)
{
    int i;
    test_prepare(side,kind==0?180.f:kind==1?270.f:360.1f,1);
    if (kind==2) {
        /* A latched prior lap cannot authorize the opposite heading. */
        tangent_lap(&test_motion);CHECK(lap_completed);
        test_motion.yaw_deg=side*540.f;
    }
    for (i=0;i<35;++i) {
        test_vision_step();
        CHECK(!ring_result.exited_events);
        CHECK(ring_result.phase!=RING_EXIT && ring_result.phase!=RING_RECOVER);
    }
}
static void test_vision_exit(int side,int shared)
{
    int i,y;
    test_prepare(side,360.1f,shared);
    tangent_lap(&test_motion);ring_result.visual_dt_ms=50;
    for (i=0;i<3;++i) {
        test_motion.now_ms+=50;
        scan_geometry((const uint8_t(*)[RING_W])test_pixels,&test_motion);
        (void)tangent_exit_observe(&test_motion);
    }
    CHECK(ring_result.phase==RING_EXIT && ring_result.exit_latched);
    for (i=0;i<50 && !ring_result.exited_events;++i) {
        test_vision_step();
        if (ring_result.phase==RING_EXIT || ring_result.phase==RING_RECOVER) {
            CHECK(ring_result.exit_latched && ring_result.path_valid && ring_result.path_source==4);
            CHECK(ring_result.steer_error*side<0);
            /* A ring-edge normal must not replace this current through
             * corridor after the full-lap exit has been latched. */
            for (y=24;y<=117;++y) {
                int centre=96+(117-y+5)/10;
                if (side>0) centre=RING_W-1-centre;
                CHECK(ring_result.mid[y]>=0);
                CHECK(abs(ring_result.mid[y]-centre)<=3);
                CHECK(test_pixels[y][ring_result.mid[y]]!=0);
            }
        }
        if (i<11) CHECK(!ring_result.exited_events);
    }
    CHECK(ring_result.exited_events==1 && !ring_owns_path());
    CHECK(!ring_result.stop && !ring_result.timed_entry && ring_speed_limit()==-1);
    CHECK(ring_blend_error(-41)==-41 && ring_blend_error(0)==0 && ring_blend_error(23)==23);
}
static void test_tick(int raw,unsigned count,int standing)
{
    while (count--) if (ring_runtime_tick((int16_t)raw,(uint8_t)standing)==1) ++test_beeps;
}
static void test_apply(void)
{
    test_speed=100;test_turn=-33;
    ring_runtime_observe_motion(100,0,0,0,0);
    ring_runtime_apply(&test_speed,&test_turn,test_revs,1);
}
static void test_command(int side,int phase,unsigned exited)
{
    ring_packet_t p={{0}};
    int owned=phase>=RING_APPROACH && phase<=RING_RECOVER;
    p.word[RL_MAGIC]=RING_LINK_MAGIC;p.word[RL_EPOCH]=(int32_t)test_epoch;
    p.word[RC_PHASE]=ring_command_versioned_phase(phase,0,RING_PAIR_VERSION);
    p.word[RC_CAP]=ring_command_cap_turn(owned?(phase==RING_APPROACH?100:90):-1,360);
    p.word[RC_DETECTED]=1;p.word[RC_EXITED]=(int32_t)exited;
    p.word[RC_FAULT]=ring_command_diagnostics(0,1,0,0,1,0,1,0,80,-side);
    if (phase==RING_APPROACH) p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_FORCE_ENTRY);
    if (owned) p.word[RC_FAULT]=(int32_t)((uint32_t)p.word[RC_FAULT]|RING_COMMAND_TIMED_ENTRY);
    ring_link_write(ring_command_mailbox,&p,&test_sequence);test_apply();
}
static void test_runtime_boot(int side)
{
    unsigned i;
    test_side=side;test_sequence=test_beeps=0;test_revs=0;
    memset(ring_command_mailbox,0,RING_LINK_WORDS*sizeof(int32_t));
    memset(ring_motion_mailbox,0,RING_LINK_WORDS*sizeof(int32_t));
    ring_runtime_init(.1f);ring_runtime_set_operator(0,0);
    test_tick(0,2200,0);ring_runtime_set_operator(1,0);
    test_tick(0,1,1);test_apply();test_epoch=ring_runtime_diagnostics()->command_epoch;
    test_command(side,RING_APPROACH,0);
    for (i=0;i<RING_MANEUVER_DELAY_MS/20u+2u;++i) {
        test_tick(0,20,1);test_revs+=.03f;test_command(side,RING_APPROACH,0);
        if (ring_runtime_diagnostics()->maneuver_stage==2) break;
    }
    CHECK(ring_runtime_diagnostics()->maneuver_stage==2 && test_beeps==1);
}
static void test_runtime_advance(int side,unsigned ms)
{
    while (ms) {
        unsigned n=ms>20u?20u:ms;
        test_tick(side*1000,n,1);test_revs+=.0015f*n;
        test_command(side,RING_ENTER,0);ms-=n;
    }
}
static void test_runtime_reject(int side,int kind)
{
    uint32_t accepted;
    const ring_runtime_diagnostics_t *d;
    test_runtime_boot(side);
    test_runtime_advance(side,kind==0?1800u:kind==1?2700u:3601u);
    if (kind==2) test_runtime_advance(side,1799u);
    d=ring_runtime_diagnostics();CHECK(d->maneuver_stage==3 && d->active);
    accepted=d->command_sequence;
    test_command(side,RING_EXIT,0);
    CHECK(d->command_sequence==accepted && d->phase==RING_ENTER);
    test_command(side,RING_COOLDOWN,1);
    CHECK(d->command_sequence==accepted && d->active && !d->exited);
    test_tick(0,100,1);
    CHECK(test_beeps==1); /* Recognition only; rejected exit emits no beep. */
}
static void test_runtime_exit(int side)
{
    test_runtime_boot(side);test_runtime_advance(side,3601u);
    CHECK(ring_runtime_diagnostics()->maneuver_stage==3);
    test_command(side,RING_COOLDOWN,1);
    CHECK(!ring_runtime_diagnostics()->active && ring_runtime_diagnostics()->exited==1);
    CHECK(test_turn==-33);
    test_tick(0,300,1);CHECK(test_beeps==3);
}
int main(void)
{
    int side,kind,shared;
    CHECK(RING_EXIT_MIN_DEG==360.f && RING_LAP_EXIT_HEADING_TOL==15.f);
    for (side=-1;side<=1;side+=2) {
        for (kind=0;kind<3;++kind) {test_vision_reject(side,kind);test_runtime_reject(side,kind);}
        for (shared=0;shared<=1;++shared) test_vision_exit(side,shared);
        test_runtime_exit(side);
    }
    puts("PASS: v31 both sides,180/270 and past360-opposite heading reject exit/beep; full360 straight exit keeps current through corridor and restores ordinary steering;360/15deg guards retained");
    return 0;
}
