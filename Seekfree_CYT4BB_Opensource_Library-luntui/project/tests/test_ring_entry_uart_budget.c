#include <stdlib.h>
#include <string.h>
#include "zf_driver_uart.h"
static unsigned fifo_capacity=128u;
static uint32_t fifo_size(volatile stc_SCB_t*p){(void)p;return fifo_capacity;}
static void fifo_write(volatile stc_SCB_t*p,uint8_t value)
{if(p->used<fifo_capacity && uart_capture_count<32767u){++p->used;uart_capture[uart_capture_count++]=(char)value;}}
#define Cy_SCB_GetFifoSize fifo_size
#define Cy_SCB_WriteTxFifo fifo_write
#include "ring_uart_diag.h"
stc_SCB_t uart_port;char uart_capture[32768];unsigned uart_capture_count;
static ring_runtime_diagnostics_t sample;
const ring_runtime_diagnostics_t*ring_runtime_diagnostics(void){return &sample;}
#define CHECK(c) do{if(!(c)){fprintf(stderr,"FAIL entry UART:%d %s queue%u drop%lu\n",__LINE__,#c,ring_diag_count,(unsigned long)ring_diag_dropped);exit(1);}}while(0)
static unsigned records(const char*prefix,unsigned columns)
{
 const char*p=uart_capture,*end,*q;unsigned count=0;
 while((p=strstr(p,prefix))!=NULL){unsigned commas=0;p+=2;end=strstr(p,"\r\n");if(!end)break;
  for(q=p;q<end;++q)if(*q==',')++commas;
  CHECK(commas==columns);++count;p=end;
 }return count;
}
static void poll(unsigned n)
{
 unsigned before=uart_capture_count;
 sample.now_ms+=46;sample.control_dt_ms=46;sample.command_sequence+=2;
 sample.frame_sequence=sample.entry_frame_sequence=sample.geometry_frame_sequence=sample.topology_frame=sample.mouth_frame=n;
 sample.branch_ok=(uint8_t)(n&1u);sample.decision=(uint8_t)(1u+(n&1u));
 uart_port.used=0;ring_uart_diag_poll(0,180,-180);
 CHECK(uart_capture_count-before<=128u && ring_diag_count<=8u);
}
int main(void)
{
 unsigned i,before;
 sample.phase=RING_ENTER;sample.active=1;sample.maneuver_stage=2;sample.force_active=1;sample.force_side=-1;
 sample.force_target10=-1000;sample.force_rate10=-980;sample.final_turn10=-652;sample.force_reason=1;
 sample.original_target10=1500;sample.final_target10=1324;sample.held_target10=1500;sample.speed_goal10=900;
 sample.measured_forward10=-365;sample.pitch100=225;sample.speed_tilt100=-350;sample.left_pwm=-1234;sample.right_pwm=2345;
 sample.diagnostic_valid=sample.entry_diagnostic_valid=sample.geometry_valid=sample.topology_valid=sample.mouth_valid=1;
 sample.command_epoch=sample.diagnostic_epoch=sample.entry_diagnostic_epoch=sample.geometry_epoch=sample.topology_epoch=sample.mouth_epoch=1;
 sample.control_version=sample.vision_version=RING_PAIR_VERSION;sample.pair_ok=1;
 sample.candidate_detected=sample.candidate_allowed=1;sample.classification=2;
 sample.entry_flags=8191;sample.hold_reject=63;sample.inner_rows=sample.associated_rows=120;
 sample.lap_origin_mdeg=-2147483647;sample.lap_mdeg=2147483647;
 for(i=0;i<50;++i)poll(i);CHECK(ring_diag_dropped==0);
 uart_capture_count=0;
 for(i=0;i<200;++i)poll(50+i);
 uart_capture[uart_capture_count]=0;
 CHECK(ring_diag_dropped==0 && ring_diag_count<=3);
 CHECK(records("\r\nB,",11)>=5);
 CHECK(records("\r\nK,",29)>=35 && records("\r\nJ,",22)>=25);
 CHECK(records("\r\nE,",14)>=10 && records("\r\nH,",16)>=10);
 printf("PASS: default K29 + R/D/E/G/H/J, %u bytes/9200ms, queue%u, drop%lu; K>=35, per-poll<=128 FIFO writes\n",uart_capture_count,ring_diag_count,(unsigned long)ring_diag_dropped);
 /* A small/full FIFO loses telemetry instead of blocking or touching
  * runtime control. Bounded queue and service still hold under pressure. */
 before=ring_diag_dropped;fifo_capacity=16;uart_capture_count=0;
 for(i=0;i<100;++i)poll(250+i);
 CHECK(ring_diag_dropped>before && ring_diag_count<=8);
 puts("PASS: constrained FIFO backpressure remains bounded and observable");return 0;
}
