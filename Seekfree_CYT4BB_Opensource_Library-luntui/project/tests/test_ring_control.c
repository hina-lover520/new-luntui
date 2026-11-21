#include "ring_control.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x); exit(1); } } while (0)

int main(void)
{
    int state, fresh;
    /* No confirmed ring must preserve arbitrary caller speeds exactly,
     * including normal line loss, stale images, and tuning above 180 RPM. */
    for (state = 0; state <= 2; ++state)
        for (fresh = 0; fresh <= 1; ++fresh) {
            CHECK(ring_control_speed(190.0f, -1, state, 0, fresh) == 190.0f);
            CHECK(ring_control_speed(180.0f, -1, state, 0, fresh) == 180.0f);
            CHECK(ring_control_speed(237.5f, -1, state, 0, fresh) == 237.5f);
            CHECK(!ring_control_stop(-1, state, 0, fresh));
        }
    CHECK(ring_control_speed(190.0f, 90, 3, 0, 1) == 90.0f);
    CHECK(ring_control_speed(70.0f, 90, 3, 0, 1) == 70.0f);
    CHECK(ring_control_speed(190.0f, 100, 0, 0, 1) == 0.0f);
    CHECK(ring_control_speed(190.0f, 0, 3, 0, 1) == 0.0f);
    CHECK(ring_control_speed(190.0f, 90, 3, 3, 1) == 0.0f);
    CHECK(ring_control_speed(190.0f, 90, 3, 0, 0) == 0.0f);
    puts("PASS: ordinary speed unchanged; caps, faults and stale stops apply only to confirmed rings");
    return 0;
}
