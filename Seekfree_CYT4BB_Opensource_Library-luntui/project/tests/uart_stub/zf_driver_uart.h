#ifndef TEST_UART_STUB_H_
#define TEST_UART_STUB_H_
#include <stdint.h>
typedef struct {unsigned used;} stc_SCB_t;
enum { UART_1=1 };
extern stc_SCB_t uart_port;
extern char uart_capture[32768];
extern unsigned uart_capture_count;
static inline volatile stc_SCB_t *get_scb_module(int uart) {(void)uart;return &uart_port;}
static inline uint32_t Cy_SCB_GetFifoSize(volatile stc_SCB_t *p) {(void)p;return 16u;}
static inline uint32_t Cy_SCB_GetNumInTxFifo(volatile stc_SCB_t *p) {return p->used;}
static inline void Cy_SCB_WriteTxFifo(volatile stc_SCB_t *p,uint8_t value) {
    if(p->used<16u && uart_capture_count<32767u){++p->used;uart_capture[uart_capture_count++]=(char)value;}
}
#endif
