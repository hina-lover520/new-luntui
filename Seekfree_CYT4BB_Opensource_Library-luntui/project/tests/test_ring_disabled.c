#include "ring.h"
#include "ring_vision.h"
#include "ring_runtime.h"
#include "ring_link.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "FAIL disabled line %d\n", __LINE__); exit(1); } } while (0)
static uint8_t image[RING_H][RING_W];
int main(void)
{
    int i, error, state;
    float target, turn;
    uint32_t seq = 0;
    ring_packet_t p = {{0}};
    CHECK(RING_ENABLE == 0);
    ring_runtime_init(0.1f); ring_vision_init(); memset(image, 255, sizeof(image));
    for (i = 0; i < 4000; ++i) CHECK(ring_runtime_tick(1000, 1) == -1);
    p.word[RL_MAGIC] = RING_LINK_MAGIC; p.word[RL_EPOCH] = 1;
    p.word[RC_CAP] = 0; p.word[RC_PHASE] = RING_FAULT; p.word[RC_FAULT] = 3;
    ring_link_write(ring_command_mailbox, &p, &seq);
    for (i = 0; i < 120; ++i) {
        error = i - 60; state = i % 3; target = 237.5f; turn = -53.0f;
        ring_vision_before_frame((const uint8_t (*)[RING_W])image);
        ring_vision_select(&error, &state);
        ring_vision_after_frame((const uint8_t (*)[RING_W])image);
        ring_runtime_apply(&target, &turn, 1.0f, state);
        CHECK(error == i - 60 && state == i % 3);
        CHECK(target == 237.5f && turn == -53.0f && !ring_runtime_watchdog_stop());
        CHECK(!ring_owns_path() && ring_speed_limit() == -1);
    }
    puts("PASS: RING_ENABLE=0 preserves ordinary error/state/speed/turn despite injected ring fault");
    return 0;
}
