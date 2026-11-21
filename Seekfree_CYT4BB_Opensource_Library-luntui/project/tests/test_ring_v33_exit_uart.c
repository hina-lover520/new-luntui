/* Production-mode L/R/K coexistence uses the established bounded FIFO stub. */
#define main prior_fifo_cases
#include "test_ring_uart_fifo.c"
#undef main

int main(void)
{
    unsigned i,l_count,before,active_bytes;
    CHECK(RING_TRACK_MODE==1 && RING_PAIR_VERSION==36u);
    reset_state();test_fifo_capacity=128u;
    sample.control_version=sample.vision_version=RING_PAIR_VERSION;sample.pair_ok=1;
    sample.exit_diagnostic_valid=1;sample.exit_diagnostic_epoch=1;
    for(i=0;i<100u;++i)poll_frame(i);
    uart_capture_count=0;
    sample.phase=RING_CIRCLE;sample.active=1;sample.path_valid=1;
    sample.run_requested=sample.standing=1;sample.maneuver_stage=3;
    sample.force_reason=8;sample.force_target10=0;sample.force_rate10=1000;
    sample.exit_reject=4u|64u;sample.exit_lap10=3500;
    sample.diagnostic_valid=sample.entry_diagnostic_valid=sample.geometry_valid=1;
    sample.topology_valid=sample.mouth_valid=1;
    sample.diagnostic_epoch=sample.entry_diagnostic_epoch=sample.geometry_epoch=1;
    sample.topology_epoch=sample.mouth_epoch=1;
    sample.decision=1;sample.branch_ok=1;
    for(i=0;i<200u;++i) {
        sample.exit_candidate_frames=(uint8_t)i;
        sample.exit_confirm_frames=(uint8_t)(i%12u);
        if(i==80u)sample.exit_candidate=1;
        if(i==100u)sample.exit_latched=1;
        if(i==120u)sample.exit_confirmed=1;
        poll_frame(100u+i);
    }
    for(i=0;i<10u;++i){uart_port.used=0;ring_uart_diag_poll(0,100,100);}
    uart_capture[uart_capture_count]=0;
    l_count=count_lines("\r\nL,",15u);
    CHECK(l_count>=20u && l_count<=50u);
    {
        const char *line=uart_capture;
        while((line=strstr(line,"\r\nL,"))!=NULL) {
            unsigned seq,ms,ep,cmd,frame,reject;
            CHECK(sscanf(line,"\r\nL,%u,%u,%u,%u,%u,%u",
                &seq,&ms,&ep,&cmd,&frame,&reject)==6);
            CHECK(reject==68u);line+=3;
        }
    }
    CHECK(count_lines("\r\nK,",29u)>=30u);
    CHECK(count_lines("\r\nJ,",22u)>=20u);
    CHECK(count_lines("\r\nE,",14u)>=20u);
    CHECK(count_lines("\r\nH,",16u)>0u);
    CHECK(ring_diag_count<=2u && ring_diag_dropped==0u);
    CHECK(uart_capture_count<30000u);
    active_bytes=uart_capture_count;
    /* Exit evidence never adds repeated traffic to ordinary tracking. */
    sample.phase=RING_NORMAL;sample.active=0;sample.maneuver_stage=0;
    sample.run_requested=0;before=ring_diag_exit_seq;
    for(i=0;i<50u;++i)poll_frame(300u+i);
    CHECK(ring_diag_exit_seq==before);
    printf("PASS: production L15=%u with unchanged R/K/J/E/H layouts, FIFO128 pressure %u bytes/9.2s, drop0; NORMAL L silent\n",l_count,active_bytes);
    return 0;
}
