#include "ring_maneuver.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(c) do { if (!(c)) { \
    fprintf(stderr,"FAIL v32-handover:%d %s\n",__LINE__,#c); exit(1); \
} } while (0)
#define NEAR(a,b) CHECK(fabsf((a)-(b))<0.001f)

static float enter_handover(ring_maneuver_t *s,int side,float normal,
    uint32_t finish,int release_ok)
{
    uint32_t turn=finish-20u,start=turn-RING_MANEUVER_DELAY_MS;
    float out;
    /* Set up the same delivered80RPM entry output for the handover unit
     * contract, independent of the user's current feed-forward tuning.
     * Real default-rate integration is exercised in the paired tests. */
    float setup_rate=side*(RING_FORCE_RATE_DPS-
        (80.f-RING_FORCE_FEED_RPM)/RING_FORCE_RATE_KP);
    ring_maneuver_begin(s,start,side,12.f,0.f);
    out=ring_maneuver_apply(s,side*80.f,turn-1u,12.f,0.1f,0.f,1,0);
    CHECK(s->stage==1); NEAR(out,side*80.f);
    s->armed=1;
    out=ring_maneuver_apply(s,-side*90.f,turn,12.f,0.1f,setup_rate,1,0);
    CHECK(s->stage==2 && s->turn_yaw==12.f); NEAR(out,side*80.f);
    out=ring_maneuver_apply(s,-side*90.f,turn,12.f+side*74.9f,0.1f,0.f,1,0);
    CHECK(s->stage==2); NEAR(out,side*80.f);
    out=ring_maneuver_apply(s,normal,finish,12.f+side*75.f,0.2f,0.f,1,release_ok);
    CHECK(s->stage==3 && s->reason==4 && s->finished_ms==finish);
    CHECK(s->target_rate==0.f); NEAR(s->output,out);
    return out;
}

static void opposite_and_expiry(int side)
{
    ring_maneuver_t s;
    uint32_t finish=1920u;
    float out;
    out=enter_handover(&s,side,-side*90.f,finish,1); NEAR(out,side*68.f);
    out=ring_maneuver_apply(&s,side*90.f,finish,12.f+side*75.f,0.2f,0.f,1,1);
    NEAR(out,side*68.f); /* A duplicate timestamp cannot move the output. */
    out=ring_maneuver_apply(&s,-side*90.f,finish+50u,12.f+side*80.f,0.3f,0.f,1,1);
    NEAR(out,side*38.f); NEAR(s.output,out);
    out=ring_maneuver_apply(&s,-side*90.f,finish+100u,12.f+side*90.f,0.4f,0.f,1,1);
    NEAR(out,side*8.f);
    out=ring_maneuver_apply(&s,-side*90.f,finish+150u,12.f+side*100.f,0.5f,0.f,1,1);
    NEAR(out,-side*22.f); /* Opposite visual steering is allowed. */
    out=ring_maneuver_apply(&s,-side*90.f,finish+200u,12.f+side*110.f,0.6f,0.f,1,1);
    NEAR(out,-side*52.f);
    out=ring_maneuver_apply(&s,-side*90.f,finish+250u,12.f+side*120.f,0.7f,0.f,1,1);
    NEAR(out,-side*82.f);
    out=ring_maneuver_apply(&s,-side*90.f,finish+299u,12.f+side*130.f,0.8f,0.f,1,1);
    NEAR(out,-side*90.f);
    out=ring_maneuver_apply(&s,side*91.f,finish+300u,12.f+side*270.f,0.9f,0.f,1,1);
    NEAR(out,side*91.f); CHECK(s.target_rate==0.f && !s.lap_reached);
    out=ring_maneuver_apply(&s,0.f,finish+400u,12.f+side*359.f,1.f,0.f,1,1);
    NEAR(out,0.f); CHECK(s.target_rate==0.f && !s.lap_reached);
    out=ring_maneuver_apply(&s,-side*17.f,finish+500u,12.f+side*360.1f,1.1f,0.f,1,1);
    NEAR(out,-side*17.f); CHECK(s.target_rate==0.f && s.lap_reached);
    out=ring_maneuver_apply(&s,side*23.f,finish+600u,12.f+side*380.f,1.2f,0.f,1,1);
    NEAR(out,side*23.f); NEAR(s.output,out); CHECK(s.stage==3 && s.target_rate==0.f);
}

static void changing_visual_target(int side)
{
    ring_maneuver_t s;
    uint32_t finish=1920u;
    float out=enter_handover(&s,side,0.f,finish,1);
    NEAR(out,side*68.f);
    out=ring_maneuver_apply(&s,0.f,finish+50u,12.f+side*80.f,0.3f,0.f,1,1);
    NEAR(out,side*38.f);
    out=ring_maneuver_apply(&s,0.f,finish+100u,12.f+side*90.f,0.4f,0.f,1,1);
    NEAR(out,side*8.f);
    out=ring_maneuver_apply(&s,0.f,finish+150u,12.f+side*100.f,0.5f,0.f,1,1);
    NEAR(out,0.f);
    out=ring_maneuver_apply(&s,-side*50.f,finish+170u,12.f+side*105.f,0.6f,0.f,1,1);
    NEAR(out,-side*12.f);
    out=ring_maneuver_apply(&s,side*50.f,finish+180u,12.f+side*110.f,0.7f,0.f,1,1);
    NEAR(out,-side*6.f);
    out=ring_maneuver_apply(&s,-side*50.f,finish+190u,12.f+side*115.f,0.8f,0.f,1,1);
    NEAR(out,-side*12.f); NEAR(s.output,out);
}

static void release_bypass(int side)
{
    ring_maneuver_t s;
    uint32_t finish=1920u;
    float out=enter_handover(&s,side,0.f,finish,1);
    NEAR(out,side*68.f);
    out=ring_maneuver_apply(&s,-side*72.f,finish+10u,12.f+side*80.f,0.3f,0.f,1,0);
    NEAR(out,-side*72.f); NEAR(s.output,out);
    out=ring_maneuver_apply(&s,-side*72.f,finish+20u,12.f+side*85.f,0.4f,0.f,1,1);
    NEAR(out,-side*72.f);
    out=ring_maneuver_apply(&s,side*72.f,finish+30u,12.f+side*90.f,0.5f,0.f,1,1);
    NEAR(out,-side*66.f);
    out=ring_maneuver_apply(&s,0.f,finish+40u,12.f+side*95.f,0.6f,0.f,1,0);
    NEAR(out,0.f); NEAR(s.output,out);
    out=ring_maneuver_apply(&s,side*72.f,finish+50u,12.f+side*100.f,0.7f,0.f,1,1);
    NEAR(out,side*6.f);
    out=enter_handover(&s,side,-side*17.f,finish,0);
    NEAR(out,-side*17.f);
    out=ring_maneuver_apply(&s,-side*17.f,finish+10u,12.f+side*80.f,0.3f,0.f,1,1);
    NEAR(out,-side*17.f);
}

static void interrupted(int side)
{
    ring_maneuver_t s;
    uint32_t finish=1920u;
    float out=enter_handover(&s,side,0.f,finish,1);
    NEAR(out,side*68.f);
    out=ring_maneuver_apply(&s,-side*31.f,finish+10u,12.f+side*80.f,0.3f,0.f,0,1);
    NEAR(out,-side*31.f); CHECK(s.stage==4 && s.reason==6 && s.target_rate==0.f);
    out=ring_maneuver_apply(&s,19.f,finish+20u,12.f+side*85.f,0.4f,0.f,1,1);
    NEAR(out,19.f); CHECK(s.stage==4 && s.target_rate==0.f);
}

static void elapsed_and_wrap(int side)
{
    ring_maneuver_t s;
    uint32_t finish=1920u;
    float out=enter_handover(&s,side,-side*90.f,finish,1);
    NEAR(out,side*68.f);
    out=ring_maneuver_apply(&s,-side*90.f,finish+250u,12.f+side*100.f,0.3f,0.f,1,1);
    NEAR(out,side*8.f); /* Existing 100ms elapsed-time cap remains active. */
    out=ring_maneuver_apply(&s,-side*90.f,finish+299u,12.f+side*110.f,0.4f,0.f,1,1);
    NEAR(out,-side*21.4f);
    out=ring_maneuver_apply(&s,side*90.f,finish+299u,12.f+side*110.f,0.4f,0.f,1,1);
    NEAR(out,-side*21.4f);
    out=ring_maneuver_apply(&s,side*90.f,finish+300u,12.f+side*115.f,0.5f,0.f,1,1);
    NEAR(out,side*90.f);
    finish=0xfffffff0u;
    out=enter_handover(&s,side,-side*90.f,finish,1); NEAR(out,side*68.f);
    out=ring_maneuver_apply(&s,-side*90.f,finish+50u,12.f+side*90.f,0.3f,0.f,1,1);
    NEAR(out,side*38.f);
    out=ring_maneuver_apply(&s,-side*90.f,finish+299u,12.f+side*100.f,0.4f,0.f,1,1);
    NEAR(out,-side*22.f);
    out=ring_maneuver_apply(&s,33.f,finish+300u,12.f+side*110.f,0.5f,0.f,1,1);
    NEAR(out,33.f);
}

int main(void)
{
    int side;
    CHECK(RING_MANEUVER_DELAY_MS==800u && RING_MANEUVER_TURN_DEG==75.f);
    CHECK(RING_ENTRY_HANDOVER_MS==300u && RING_FORCE_SLEW_RPM_S==600.f);
    for(side=-1;side<=1;side+=2){
        opposite_and_expiry(side); changing_visual_target(side);
        release_bypass(side); interrupted(side); elapsed_and_wrap(side);
    }
    NEAR(ring_maneuver_forward_limit(0.f,1),0.f);
    NEAR(ring_maneuver_forward_limit(-45.f,1),0.f);
    NEAR(ring_maneuver_forward_limit(90.f,0),0.f);
    NEAR(ring_maneuver_forward_limit(NAN,1),0.f);
    NEAR(ring_maneuver_forward_limit(INFINITY,1),0.f);
    NEAR(ring_maneuver_forward_limit(22.5f,1),22.5f);
    NEAR(ring_maneuver_forward_limit(45.f,1),45.f);
    NEAR(ring_maneuver_forward_limit(90.f,1),90.f);
    NEAR(ring_maneuver_forward_limit(135.f,1),90.f);
    puts("PASS: v32 finite 300ms visual handover, both directions, zero/opposite and changing visual targets, last delivered output, release/operator bypass, duplicate and capped time, clock wrap, current 1000ms/75deg entry and forward scale");
    return 0;
}
