#ifndef CODE_TRACK_SHARED_H_
#define CODE_TRACK_SHARED_H_

#include "zf_common_headfile.h"
#include "ring_control.h"

/* One writer per cache-line-aligned region. Both firmware images must use
 * this protocol together. A consumer INVALIDATES and never cleans its input. */
#define TRACK_VISION_WORDS       16
#define TRACK_FEEDBACK_WORDS     24
#define TRACK_VISION_SEQ         3
#define TRACK_FEEDBACK_SEQ       0
#define TRACK_VISION_TIMEOUT_MS  300u
#define TRACK_EVENT_BEEP_MS      120u

/* vision @ 0x28001000 (64 bytes), written only by CM7_0:
 * 0=error, 1=0 lost/1 normal/2 cross/3 ring, 2=speed cap RPM (-1 = none), 3=sequence,
 * 4=ring phase, 5=fault, 6=route, 7=net turn degrees,
 * 8=confirmed ring event counter, 9=completed exit event counter.
 * feedback @ 0x28001040 (96 bytes), written only by CM7_1:
 * 0=sequence, 1=continuous yaw*100, 2=speed*10, 3=target RPM*10,
 * 4=speed-loop angle*10, 5/6=duty, 7/8=wheel RPM, 9=standing,
 * 10=turn*10, 11..14=servo*10, 15=milliseconds, 16=wheel revs*1000.
 */
extern int32_t m7_1_data_1[TRACK_VISION_WORDS];
extern int32_t m7_feedback[TRACK_FEEDBACK_WORDS];
extern volatile uint32_t track_control_ms;
extern volatile float track_heading_deg;
extern volatile uint32_t track_beep_until_ms;
extern volatile uint32_t track_vision_deadline_ms;
extern volatile uint8_t track_timeout_enabled;

static inline void track_publish(int32_t *destination, const int32_t *source,
                                 unsigned words, unsigned seq_index, uint32_t *sequence)
{
    unsigned i;
    uint32_t next = *sequence + 2u;
    if (!next) next = 2u;
    ((volatile int32_t *)destination)[seq_index] = (int32_t)(next - 1u);
    __DMB();
    SCB_CleanDCache_by_Addr((uint32_t *)destination, (int32_t)(words * sizeof(int32_t)));
    __DMB();
    for (i = 0; i < words; ++i)
        if (i != seq_index) ((volatile int32_t *)destination)[i] = source[i];
    __DMB();
    SCB_CleanDCache_by_Addr((uint32_t *)destination, (int32_t)(words * sizeof(int32_t)));
    __DMB();
    ((volatile int32_t *)destination)[seq_index] = (int32_t)next;
    SCB_CleanDCache_by_Addr((uint32_t *)destination, (int32_t)(words * sizeof(int32_t)));
    __DMB();
    *sequence = next;
}

/* Keep the previous snapshot when a producer is midway through publishing. */
static inline int track_snapshot(int32_t *source, int32_t *snapshot,
                                unsigned words, unsigned seq_index)
{
    int32_t copy[TRACK_FEEDBACK_WORDS];
    uint32_t before, after;
    unsigned i;
    SCB_InvalidateDCache_by_Addr((uint32_t *)source, (int32_t)(words * sizeof(int32_t)));
    __DMB();
    before = (uint32_t)((volatile int32_t *)source)[seq_index];
    if (!before || (before & 1u)) return 0;
    for (i = 0; i < words; ++i) copy[i] = ((volatile int32_t *)source)[i];
    __DMB();
    SCB_InvalidateDCache_by_Addr((uint32_t *)source, (int32_t)(words * sizeof(int32_t)));
    __DMB();
    after = (uint32_t)((volatile int32_t *)source)[seq_index];
    if (before != after) return 0;
    for (i = 0; i < words; ++i) snapshot[i] = copy[i];
    return 1;
}

#endif
