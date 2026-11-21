#ifndef CODE_RING_FORWARD_PROFILE_H_
#define CODE_RING_FORWARD_PROFILE_H_
#include "ring.h"
#include <math.h>
#include <string.h>

/* Continuous forward TARGET only. Original steering, speed/balance PID,
 * motor limits and stop requests retain their existing implementations. */
typedef struct {
    uint32_t previous_ms;
    float output,goal;
    unsigned valid;
} ring_forward_profile_t;

static inline float ring_forward_profile_apply(ring_forward_profile_t *s,
    float requested,uint32_t now,int running,int lost,float measured,int sample_valid)
{
    uint32_t dt;float goal,delta,limit;
    if(!running || requested<=0.f || !isfinite(requested)) {
        memset(s,0,sizeof(*s));return requested;
    }
    if(!s->valid) {
        s->valid=1;s->previous_ms=now;
        /* A live startup starts from measured motion, never a160RPM jump.
         * Missing observations cannot fabricate measured zero. */
        s->output=sample_valid?fmaxf(0.f,fminf(requested,measured)):requested;
    }
    dt=now-s->previous_ms;s->previous_ms=now;if(dt>100u)dt=100u;
    goal=requested;
    if(lost)goal=fminf(goal,fminf(s->output,(float)RING_CIRCLE_RPM));
    if(sample_valid && measured>requested+RING_FORWARD_OVERSPEED_RPM) {
        float trim=fminf(RING_FORWARD_TRIM_MAX_RPM,
            (measured-requested-RING_FORWARD_OVERSPEED_RPM)*.5f);
        goal=fmaxf(0.f,goal-trim);
    }
    /* Wheel samples are supplied by the existing main-loop observer;
     * their age is NOT a measurement of underlying encoder packet age. */
    if(sample_valid && measured>s->output+RING_FORWARD_OVERSPEED_RPM)
        goal=fminf(goal,s->output);
    s->goal=goal;delta=goal-s->output;
    limit=(delta>=0.f?RING_FORWARD_RISE_RPM_S:RING_FORWARD_FALL_RPM_S)*dt*.001f;
    if(delta>limit)delta=limit;
    if(delta< -limit)delta= -limit;
    s->output+=delta;return s->output;
}
#endif

