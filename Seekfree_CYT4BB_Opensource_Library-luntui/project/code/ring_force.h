#ifndef CODE_RING_FORCE_H_
#define CODE_RING_FORCE_H_

#include "ring.h"
#include <math.h>
#include <string.h>

/* Runs only after the original controller. No integral, sensor access,
 * motor writes or waits. A single F handover has one non-renewable budget. */
typedef struct {
    uint32_t started_ms, previous_ms, released_ms;
    float previous_yaw, previous_revs, angle, travel, output;
    int side;
    unsigned begun, spent, active, releasing, reason;
} ring_force_t;

static inline void ring_force_begin(ring_force_t *s,uint32_t now,int side,
                                     float yaw,float revs)
{
    memset(s,0,sizeof(*s));s->begun=1;s->side=side;
    s->started_ms=s->previous_ms=now;
    s->previous_yaw=yaw;s->previous_revs=revs;
}

/* Reasons: 0 idle,1 forcing,2 visual release,3 time,4 angle,5 travel,
 * 6 unavailable/unsafe,7 conflicting side. A false request never re-arms. */
static inline float ring_force_apply(ring_force_t *s,float normal,uint32_t now,
    float yaw,float revs,float rate,int side,int request,int eligible,int release_ok)
{
    float result=normal;
    uint32_t dt=now-s->previous_ms;
    if(!s->begun) return normal;
    s->angle+=fabsf(yaw-s->previous_yaw);s->travel+=fabsf(revs-s->previous_revs);
    s->previous_yaw=yaw;s->previous_revs=revs;s->previous_ms=now;
    if(!eligible) {s->active=s->releasing=0;s->spent=1;s->reason=6;return normal;}
    if(!s->spent) {
        if(side!=s->side || (side!=1 && side!=-1)) {s->spent=1;s->reason=7;}
        else if(now-s->started_ms>RING_FORCE_TOTAL_MS) {s->spent=1;s->reason=3;}
        else if(s->angle>RING_FORCE_ENTRY_YAW_DEG) {s->spent=1;s->reason=4;}
        else if(s->travel>RING_FORCE_ENTRY_REVS) {s->spent=1;s->reason=5;}
    }
    if(request && !s->spent) {
        float demand=RING_FORCE_FEED_RPM+RING_FORCE_RATE_KP*(RING_FORCE_RATE_DPS-side*rate);
        float step,limit=RING_FORCE_SLEW_RPM_S*(float)(dt>100u?100u:dt)*.001f;
        if(demand<0) demand=0;
        if(demand>RING_FORCE_MAX_RPM) demand=RING_FORCE_MAX_RPM;
        if(!s->active) s->output=normal*side>0?normal:0;
        if(s->output*side>RING_FORCE_MAX_RPM) s->output=side*RING_FORCE_MAX_RPM;
        step=side*demand-s->output;
        if(step>limit)step=limit;
        if(step< -limit)step= -limit;
        s->output+=step;s->active=1;s->releasing=0;s->reason=1;
        return s->output;
    }
    if(s->active) {
        s->active=0;s->spent=1;s->released_ms=now;
        s->releasing=(unsigned)(release_ok && s->reason<=2u);
        if(s->reason==1u)s->reason=2;
    }
    if(s->releasing && (now-s->started_ms>RING_FORCE_TOTAL_MS ||
       s->angle>RING_FORCE_ENTRY_YAW_DEG || s->travel>RING_FORCE_ENTRY_REVS))
        s->releasing=0; /* a visual blend cannot extend this F budget */
    if(s->releasing && s->reason<=2u && release_ok && now-s->released_ms<RING_FORCE_RELEASE_MS) {
        float weight=1.f-(float)(now-s->released_ms)/RING_FORCE_RELEASE_MS;
        result=normal+(s->output-normal)*weight;
    } else s->releasing=0;
    return result;
}
#endif
