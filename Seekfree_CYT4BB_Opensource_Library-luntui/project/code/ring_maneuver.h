#ifndef CODE_RING_MANEUVER_H_
#define CODE_RING_MANEUVER_H_
#include "ring.h"
#include <math.h>
#include <string.h>

/* One recognition event, one non-blocking motion. Camera loss cannot reset
 * the clock/heading or cut the turn; operator stop and stale command can.
 * Stage: 0 idle,1 approach corridor,2 signed 75deg turn,3 visual follow (NO yaw target),4 aborted. */
typedef struct {
    uint32_t started_ms,turn_ms,previous_ms,finished_ms;
    float straight_yaw,turn_yaw,previous_revs,travel,output,progress,target_rate,turn_revs;
    int side;
    unsigned stage,reason,lap_reached,net_exit_reached,armed;
} ring_maneuver_t;

static inline float ring_maneuver_clamp(float v,float lo,float hi)
{return v<lo?lo:(v>hi?hi:v);}

/* Ring-only allowance: full existing90RPM turn at90RPM forward, reduced
 * proportionally as actual forward motion falls, zero at standstill/reverse.
 * This does not write wheel targets or change the original balance PID. */
static inline float ring_maneuver_forward_limit(float measured,int valid)
{
    if(!valid || !isfinite(measured) || measured<=0.f)return 0.f;
    return RING_FORCE_MAX_RPM*ring_maneuver_clamp(measured/RING_CIRCLE_RPM,0.f,1.f);
}

static inline void ring_maneuver_begin(ring_maneuver_t *s,uint32_t now,int side,
    float yaw,float revs)
{
    memset(s,0,sizeof(*s));s->stage=1;s->side=side;
    s->started_ms=s->previous_ms=now;s->straight_yaw=yaw;s->previous_revs=revs;
}

static inline float ring_maneuver_apply(ring_maneuver_t *s,float normal,
    uint32_t now,float yaw,float revs,float rate,int eligible,int release_ok)
{
    uint32_t dt=now-s->previous_ms;
    float demand,step,limit=RING_FORCE_SLEW_RPM_S*(float)(dt>100u?100u:dt)*.001f;
    s->previous_ms=now;
    if(!s->stage)return normal;
    if(!eligible && s->stage<4){s->stage=4;s->reason=6;s->target_rate=0;return normal;}
    if(s->stage==1 && s->armed && now-s->started_ms>=RING_MANEUVER_DELAY_MS){
        s->stage=2;s->turn_ms=now;s->turn_yaw=yaw;
        s->travel=0;s->turn_revs=s->previous_revs=revs;
    }
    if(s->stage==2 || s->stage==3){
        s->progress=s->side*(yaw-s->turn_yaw);
        s->travel+=fabsf(revs-s->previous_revs);s->previous_revs=revs;
        if(s->progress>=RING_EXIT_MIN_DEG)s->lap_reached=1;
        /* This is signed NET yaw from the start of the entry turn. Keep
         * its history while the finite exit correction returns to the
         * original road; absolute yaw travel never arms this latch. */
        if(s->progress>=RING_EXIT_NET_MIN_DEG)s->net_exit_reached=1;
        if(s->stage==2 && s->progress>=RING_MANEUVER_TURN_DEG){s->stage=3;s->reason=4;s->finished_ms=now;}
        else if(s->stage==2 && now-s->turn_ms>RING_MANEUVER_TURN_MS){s->stage=4;s->reason=3;}
        else if(s->stage==2 && s->travel>RING_MANEUVER_TURN_REVS){s->stage=4;s->reason=5;}
    }
    if(s->stage==3){
        /* Follow the current visual target through one finite handover.
         * Start every step from the last delivered output, including when
         * release evidence disappears; never hold a post-entry heading. */
        s->target_rate=0;
        if(eligible && release_ok && now-s->finished_ms<RING_ENTRY_HANDOVER_MS){
            step=ring_maneuver_clamp(normal-s->output,-limit,limit);
            s->output+=step;
            return s->output;
        }
        s->output=normal;
        return normal;
    }
    if(s->stage==4){s->target_rate=0;return normal;}
    if(s->stage==1){
        /* Current incoming corridor goes through the original visual PID.
         * Never lock a skewed recognition-time heading. */
        s->target_rate=0;s->output=normal;return normal;
    } else {
        float wanted=ring_maneuver_clamp((RING_MANEUVER_TURN_DEG-s->progress)*3.f,20.f,RING_FORCE_RATE_DPS);
        s->target_rate=s->side*wanted;
        demand=s->side*ring_maneuver_clamp(RING_FORCE_FEED_RPM*(wanted/RING_FORCE_RATE_DPS)+
            RING_FORCE_RATE_KP*(wanted-s->side*rate),0.f,RING_FORCE_MAX_RPM);
    }
    step=ring_maneuver_clamp(demand-s->output,-limit,limit);s->output+=step;
    s->reason=1;return s->output;
}
#endif
