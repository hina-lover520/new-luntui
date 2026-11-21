#ifndef CODE_RING_EXIT_TURN_H_
#define CODE_RING_EXIT_TURN_H_

#include "ring.h"
#include <math.h>
#include <string.h>

/* After the signed net400 gate, correct at most70 degrees back.
 * The target is the
 * incoming-road heading plus one signed revolution, never360+70.
 * The original maneuver remains in visual-follow stage3 throughout.
 * Stage:0 idle,1 turning,2 target reached,3 aborted. */
typedef struct {
    uint32_t started_ms,previous_ms,finished_ms;
    float start_yaw,target_yaw,start_lap,progress,remaining;
    float previous_revs,travel,output,release_output,target_rate;
    int side;
    unsigned stage,reason;
} ring_exit_turn_t;

static inline float ring_exit_turn_clamp(float value,float lo,float hi)
{return value<lo?lo:(value>hi?hi:value);}

static inline int ring_exit_turn_begin(ring_exit_turn_t *s,uint32_t now,
    int side,float yaw,float target,float lap,float revs,float delivered)
{
    float signed_remaining=side*(target-yaw);
    memset(s,0,sizeof(*s));
    /* side is the lap direction; a late exit corrects back toward the
     * SAME original outgoing heading, rather than adding another arc. */
    s->side=signed_remaining<0.f?-side:side;
    s->started_ms=s->previous_ms=now;
    s->start_yaw=yaw;s->target_yaw=target;s->start_lap=lap;
    s->previous_revs=revs;s->remaining=fabsf(signed_remaining);
    if((side!=1 && side!=-1) || !isfinite(yaw) || !isfinite(target) ||
       !isfinite(lap) || !isfinite(revs) || !isfinite(delivered) ||
       /* The caller certifies the per-episode net400 history. The car
        * may already have corrected toward360 before this command. */
       lap<(RING_EXIT_NET_MIN_DEG>0.f?RING_EXIT_MIN_DEG:RING_EXIT_FORCE_START_DEG) ||
       lap>RING_EXIT_SEARCH_END_DEG ||
       s->remaining>RING_EXIT_FORCE_DEG+.01f ||
       (signed_remaining<0.f && s->remaining>RING_EXIT_SEARCH_END_DEG-RING_EXIT_MIN_DEG+.01f)) {
        s->stage=3u;s->reason=6u;return 0;
    }
    s->output=s->side*ring_exit_turn_clamp(s->side*delivered,0.f,RING_FORCE_MAX_RPM);
    if(s->remaining<=0.f) {
        s->stage=2u;s->reason=2u;s->finished_ms=now;
        s->output=s->release_output=delivered;
    } else {s->stage=1u;s->reason=1u;}
    return 1;
}

static inline float ring_exit_turn_apply(ring_exit_turn_t *s,float normal,
    uint32_t now,float yaw,float revs,float rate,int eligible)
{
    uint32_t dt=now-s->previous_ms;
    float limit=RING_FORCE_SLEW_RPM_S*(float)(dt>100u?100u:dt)*.001f;
    float wanted,demand;
    s->previous_ms=now;
    if(!s->stage || s->stage==3u)return normal;
    if(!eligible || !isfinite(yaw) || !isfinite(revs) || !isfinite(rate)) {
        s->stage=3u;s->reason=6u;s->target_rate=0.f;return normal;
    }
    s->progress=s->side*(yaw-s->start_yaw);
    s->remaining=s->side*(s->target_yaw-yaw);
    s->travel+=fabsf(revs-s->previous_revs);s->previous_revs=revs;
    if(s->stage==1u) {
        if(s->remaining<=0.f) {
            s->stage=2u;s->reason=2u;s->finished_ms=now;
            s->release_output=s->output;
        } else if(now-s->started_ms>RING_EXIT_FORCE_TIMEOUT_MS) {
            s->stage=3u;s->reason=3u;s->target_rate=0.f;return normal;
        } else if(s->travel>RING_EXIT_FORCE_REVS) {
            s->stage=3u;s->reason=4u;s->target_rate=0.f;return normal;
        } else if(s->progress< -RING_EXIT_FORCE_BACKTRACK_DEG) {
            s->stage=3u;s->reason=7u;s->target_rate=0.f;return normal;
        }
    }
    if(s->stage==2u) {
        s->target_rate=0.f;
        if(now-s->finished_ms<RING_FORCE_RELEASE_MS) {
            float weight=1.f-(float)(now-s->finished_ms)/(float)RING_FORCE_RELEASE_MS;
            s->output=normal+(s->release_output-normal)*weight;
        } else s->output=normal;
        return s->output;
    }
    wanted=ring_exit_turn_clamp(s->remaining*3.f,20.f,RING_FORCE_RATE_DPS);
    s->target_rate=s->side*wanted;
    demand=s->side*ring_exit_turn_clamp(RING_FORCE_FEED_RPM*(wanted/RING_FORCE_RATE_DPS)+
        RING_FORCE_RATE_KP*(wanted-s->side*rate),0.f,RING_FORCE_MAX_RPM);
    s->output+=ring_exit_turn_clamp(demand-s->output,-limit,limit);
    return s->output;
}

#endif
