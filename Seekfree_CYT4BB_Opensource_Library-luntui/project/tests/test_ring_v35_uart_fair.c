/* Exercise the real formatters/FIFO with every optional stream eligible. */
#define main original_fifo_cases
#include "test_ring_uart_fifo.c"
#undef main

static unsigned optional_total(void)
{
    return ring_diag_detail_seq+ring_diag_entry_seq+ring_diag_geometry_seq+
        ring_diag_topology_seq+ring_diag_mouth_seq+ring_diag_exit_seq+
        ring_diag_force_seq+ring_diag_exit_force_seq;
}

static void warmup(void)
{
    unsigned i;
    reset_state();test_fifo_capacity=128u;
    ring_diag_have_force=ring_diag_have_exit_force=0;
    ring_diag_force_seq=ring_diag_force_last_ms=ring_diag_force_key=0;
    ring_diag_exit_force_seq=ring_diag_exit_force_last_ms=ring_diag_exit_force_key=0;
    ring_diag_exit_force_header_line=0;
    ring_diag_optional_cursor=0;
    sample.control_version=sample.vision_version=RING_PAIR_VERSION;sample.pair_ok=1;
    for(i=0;i<100u;++i)poll_frame(i);
    uart_capture_count=0;
}

static void all_streams(unsigned with_exit_turn)
{
    unsigned i,before,bytes,d_count=0,l_count=0,d_gap=0,l_gap=0;
    uint32_t start,last_d,last_l;
    warmup();start=last_d=last_l=sample.now_ms;
    sample.phase=RING_CIRCLE;sample.active=sample.committed=sample.path_valid=1;
    sample.command_epoch=1;sample.detected=1;sample.maneuver_stage=3;
    sample.run_requested=sample.standing=1;sample.force_reason=4;
    sample.exit_force_stage=(uint8_t)with_exit_turn;
    sample.exit_force_side=with_exit_turn?-1:0;
    sample.exit_force_start10=2950;
    sample.diagnostic_valid=sample.entry_diagnostic_valid=sample.geometry_valid=1;
    sample.topology_valid=sample.mouth_valid=sample.exit_diagnostic_valid=1;
    sample.diagnostic_epoch=sample.entry_diagnostic_epoch=sample.geometry_epoch=1;
    sample.topology_epoch=sample.mouth_epoch=sample.exit_diagnostic_epoch=1;
    sample.branch_ok=1;sample.classification=2;sample.visual_dt_ms=23;
    sample.approach_stage=4;sample.left_entry=sample.lap_valid=1;
    sample.mouth_near_x=70;sample.mouth_near_y=108;
    sample.mouth_far_x=125;sample.mouth_far_y=61;
    sample.mouth_first_y=44;
    for(i=0;i<200u;++i) {
        unsigned d_before=ring_diag_detail_seq,l_before=ring_diag_exit_seq;
        sample.decision=(uint8_t)(11u+(i&1u)*2u); /* Major E event every poll. */
        sample.entry_flags=(uint16_t)((i&1u)?1024u:4096u);
        sample.path_source=(uint8_t)(6u+(i&1u));
        sample.exit_candidate=(uint8_t)(i&1u); /* L events cannot starve D. */
        sample.exit_force_turn10=(int16_t)i;
        sample.exit_force_remaining10=(int16_t)(650-(int)i);
        before=optional_total();bytes=uart_capture_count;
        poll_frame(100u+i);
        CHECK(optional_total()-before<=1u);
        CHECK(uart_capture_count-bytes<=128u);
        if(ring_diag_detail_seq!=d_before) {
            unsigned gap=(unsigned)(sample.now_ms-last_d);
            if(gap>d_gap)d_gap=gap;
            last_d=sample.now_ms;++d_count;
        }
        if(ring_diag_exit_seq!=l_before) {
            unsigned gap=(unsigned)(sample.now_ms-last_l);
            if(gap>l_gap)l_gap=gap;
            last_l=sample.now_ms;++l_count;
        }
        CHECK(sample.now_ms-last_d<=500u && sample.now_ms-last_l<=500u);
    }
    CHECK(sample.now_ms-start==9200u);
    CHECK(d_count>=19u && l_count>=19u);
    CHECK(ring_diag_entry_seq && ring_diag_geometry_seq && ring_diag_topology_seq && ring_diag_mouth_seq);
    CHECK(ring_diag_count<=2u && ring_diag_dropped==0u);
    /* M stage transitions bypass routine traffic even when D/L are due. */
    if(with_exit_turn) {
        before=ring_diag_exit_force_seq;sample.exit_force_stage=2;
        poll_frame(301u);CHECK(ring_diag_exit_force_seq==before+1u);
        before=ring_diag_exit_force_seq;sample.exit_force_stage=3;sample.exit_force_reason=2;
        poll_frame(302u);CHECK(ring_diag_exit_force_seq==before+1u);
    } else CHECK(ring_diag_exit_force_seq==0u);
    for(i=0;i<8u;++i){uart_port.used=0;ring_diag_service();}
    uart_capture[uart_capture_count]=0;
    CHECK(count_lines("\r\nD,",19u)>=19u);
    CHECK(count_lines("\r\nL,",15u)>=19u);
    CHECK(count_lines("\r\nE,",14u)>0u && count_lines("\r\nG,",14u)>0u);
    CHECK(count_lines("\r\nH,",16u)>0u && count_lines("\r\nJ,",22u)>0u);
    printf("PASS v35 UART fairness: M=%u, D=%u(max%ums), L=%u(max%ums), bytes=%u/9.2s, drop0\n",
        ring_diag_exit_force_seq,d_count,d_gap,l_count,l_gap,uart_capture_count);
}

int main(void)
{
    CHECK(RING_PAIR_VERSION==36u);
    all_streams(0u);all_streams(1u);
    return 0;
}
