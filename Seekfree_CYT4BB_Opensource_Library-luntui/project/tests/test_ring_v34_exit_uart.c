/* The production UART retains the original bounded FIFO and all layouts. */
#define main prior_fifo_cases
#include "test_ring_uart_fifo.c"
#undef main

static void measured_outer_image(void)
{
    ring_image_packet_t packet={{0},{0}};
    uint32_t sequence=0;
    unsigned before;
    reset_state();sample.command_epoch=1;
    packet.word[RL_MAGIC]=RING_IMAGE_MAGIC;packet.word[RL_EPOCH]=1;
    packet.word[RI_ID]=1;packet.word[RI_FRAME]=8;packet.word[RI_COMMAND]=2;
    packet.word[RI_FLAGS]=RING_IMAGE_HANDOVER|(RING_CIRCLE<<4)|(7<<12);
    packet.word[RI_OFFSET_LENGTH]=(int32_t)(RING_IMAGE_CHUNK_BYTES<<16);
    packet.word[RI_ROWS]=ring_diagnostic_points(-1,-1);
    packet.word[RI_RESERVED]=33|(33<<8);
    ring_link_write_image(&packet,&sequence);
    ring_diag_image_poll(&sample);CHECK(ring_diag_count==0u);
    packet.word[RI_RESERVED]=35|(34<<8);
    ring_link_write_image(&packet,&sequence);
    ring_diag_image_poll(&sample);CHECK(ring_diag_count==0u);
    packet.word[RI_RESERVED]=RING_PAIR_VERSION|(RING_PAIR_VERSION<<8);
    ring_link_write_image(&packet,&sequence);
    ring_diag_image_poll(&sample);CHECK(ring_diag_count==2u);
    before=ring_diag_image_sequence;
    ring_diag_image_poll(&sample);CHECK(ring_diag_image_sequence==before);
    /* Explicit source8 still cannot leak into the snapshot protocol. */
    ring_diag_count=0;packet.word[RI_ID]=2;
    packet.word[RI_FLAGS]=RING_IMAGE_HANDOVER|(RING_CIRCLE<<4)|(8<<12);
    ring_link_write_image(&packet,&sequence);
    ring_diag_image_poll(&sample);CHECK(ring_diag_count==0u);
}

int main(void)
{
    unsigned i,before,m_count,active_bytes;
    CHECK(RING_TRACK_MODE==1 && RING_PAIR_VERSION==36u);
    reset_state();test_fifo_capacity=128u;
    sample.control_version=sample.vision_version=RING_PAIR_VERSION;sample.pair_ok=1;
    for(i=0;i<100u;++i)poll_frame(i);
    uart_capture[uart_capture_count]=0;
    CHECK(strstr(uart_capture,"#RING,36,UART1,115200")!=NULL);
    CHECK(strstr(uart_capture,"#M,")==NULL);
    CHECK(strstr(uart_capture,"#EXIT_FORCE,")==NULL);
    CHECK(count_lines("\r\nM,",16u)==0u);
    sample.command_epoch=1;sample.detected=7;sample.phase=RING_EXIT;
    sample.active=sample.path_valid=sample.run_requested=sample.standing=1;
    sample.exit_force_stage=1;sample.exit_force_side=1;sample.exit_force_start10=2950;
    sample.exit_force_remaining10=650;sample.measured_forward10=900;
    before=ring_diag_exit_force_seq;poll_frame(100u);
    CHECK(ring_diag_exit_force_seq==before+1u);
    before=ring_diag_exit_force_seq;sample.exit_force_turn10=50;
    poll_frame(101u);CHECK(ring_diag_exit_force_seq==before);
    sample.exit_force_stage=2;sample.exit_force_remaining10=0;
    poll_frame(102u);CHECK(ring_diag_exit_force_seq==before+1u);
    before=ring_diag_exit_force_seq;sample.phase=RING_NORMAL;sample.active=0;
    sample.exit_force_stage=0;sample.exit_force_side=0;
    poll_frame(103u);CHECK(ring_diag_exit_force_seq==before+1u);
    before=ring_diag_exit_force_seq;
    for(i=0;i<100u;++i)poll_frame(104u+i);
    CHECK(ring_diag_exit_force_seq==before);
    /* M/K/L/J/E/H share a single optional admission at each 46ms poll.
     * Progress changes at camera rate cannot create an M event flood. */
    uart_capture_count=0;
    sample.phase=RING_EXIT;sample.active=1;sample.path_valid=1;
    sample.maneuver_stage=3;sample.force_reason=8;
    sample.exit_force_stage=1;sample.exit_force_side=-1;sample.exit_force_start10=2940;
    sample.exit_diagnostic_valid=1;sample.exit_diagnostic_epoch=1;
    sample.exit_candidate=sample.exit_latched=1;sample.exit_lap10=2940;
    sample.diagnostic_valid=sample.entry_diagnostic_valid=sample.geometry_valid=1;
    sample.topology_valid=sample.mouth_valid=1;
    sample.diagnostic_epoch=sample.entry_diagnostic_epoch=sample.geometry_epoch=1;
    sample.topology_epoch=sample.mouth_epoch=1;
    sample.decision=1;sample.branch_ok=1;
    for(i=0;i<200u;++i) {
        sample.exit_force_turn10=(int16_t)(i*3u);
        sample.exit_force_remaining10=(int16_t)(660u-i*3u);
        sample.exit_force_elapsed_ms=(uint16_t)(i*46u);
        sample.exit_candidate_frames=(uint8_t)i;
        poll_frame(300u+i);
    }
    for(i=0;i<10u;++i){uart_port.used=0;ring_uart_diag_poll(0,100,100);}
    uart_capture[uart_capture_count]=0;
    m_count=count_lines("\r\nM,",16u);
    printf("UART fair cadence: M=%u K=%u L=%u J=%u\n",m_count,
        count_lines("\r\nK,",29u),count_lines("\r\nL,",15u),count_lines("\r\nJ,",22u));
    CHECK(strstr(uart_capture,"#M,seq,ms,epoch,cmd,det,stage,reason,side,start10,turn10,rem10,elapsed,meas10,out10,age,drop")!=NULL);
    CHECK(strstr(uart_capture,"BEEP=3x60ms_ON_60ms_GAP;END_BEEP=2")!=NULL);
    CHECK(m_count>=20u && m_count<=42u);
    CHECK(count_lines("\r\nK,",29u)>=20u);
    CHECK(count_lines("\r\nL,",15u)>=20u);
    CHECK(count_lines("\r\nJ,",22u)>=20u);
    CHECK(ring_diag_count<=2u && ring_diag_dropped==0u);
    CHECK(uart_capture_count<30000u);active_bytes=uart_capture_count;
    /* A blocked software/physical queue never makes the poll wait; the
     * transition is delivered once space becomes available. */
    before=ring_diag_exit_force_seq;sample.exit_force_stage=3;
    sample.exit_force_reason=2;uart_port.used=test_fifo_capacity;
    ring_diag_count=RING_DIAG_QUEUE_LINES;
    ring_uart_diag_poll(0,100,100);CHECK(ring_diag_exit_force_seq==before);
    ring_diag_count=0;uart_port.used=0;
    ring_uart_diag_poll(0,100,100);CHECK(ring_diag_exit_force_seq==before+1u);
    measured_outer_image();
    printf("PASS: v34 M16 immediate state events/fair heartbeat, ordinary silent, unchanged K/L/J and bounded FIFO; %u M, %u bytes/9.2s, drop0\n",m_count,active_bytes);
    return 0;
}
