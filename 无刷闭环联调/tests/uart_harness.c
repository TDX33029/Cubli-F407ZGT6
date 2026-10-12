/* Exercise the actual UART ISR and command queue without a physical UART. */
#include "mock_hw.h"
typedef struct {uint32_t CR1,SR,DR;} MockUART;
static MockUART uart[2];
#define USART1 (&uart[0])
#define USART2 (&uart[1])
#define USART_CR1_TXEIE 128U
#define USART_SR_TXE 128U
#define USART_SR_RXNE 32U
#define USART_SR_ORE 8U
#define USART_SR_FE 2U
#define USART_SR_NE 4U
static int ping_count,stop_reason;
void Control_Heartbeat(void) {ping_count++;}
void Control_RequestStop(uint8_t reason) {stop_reason=reason;}
#include "uart_under_test.c"
static void bytes(const char *s) {
    while(*s) {USART1->SR=USART_SR_RXNE;USART1->DR=(unsigned char)*s++;USART_Link_IRQHandler();USART1->SR=0;}
}
int main(void) {
    char line[256];int i;
    bytes("APP 0\r\nDBG 2\r\nMODE 1\r\nEN2 1\r\nM2 3\r\n");
    assert(USART_ReadLine(line)&&!strcmp(line,"APP 0"));
    assert(USART_ReadLine(line)&&!strcmp(line,"DBG 2"));
    assert(USART_ReadLine(line)&&!strcmp(line,"MODE 1"));
    assert(USART_ReadLine(line)&&!strcmp(line,"EN2 1"));
    assert(USART_ReadLine(line)&&!strcmp(line,"M2 3"));assert(!USART_ReadLine(line));
    bytes("PING\n PiNg \n");assert(ping_count==2&&!USART_ReadLine(line));
    bytes("M2 20\n stop \r\n");assert(stop_reason==1&&!USART_ReadLine(line));
    for(i=0;i<300;i++) bytes("X");bytes("EN1 1\n");assert(!USART_ReadLine(line));
    bytes("EN1 1\n");assert(USART_ReadLine(line)&&!strcmp(line,"EN1 1"));
    bytes("M1 ");USART1->SR=USART_SR_ORE;USART1->DR='3';USART_Link_IRQHandler();USART1->SR=0;
    bytes("4\n");assert(!USART_ReadLine(line));
    stop_reason=0;for(i=0;i<16;i++) bytes("STATE?\n");assert(stop_reason==4&&!USART_ReadLine(line));
    for(i=0;i<2000;i++) cubli_fputc('A',NULL);assert(USART_TxFree()==0); /* full TX never blocks */
    puts("PASS: UART bursts, priority STOP/PING, damaged/oversize lines, RX queue overflow, nonblocking TX");
    return 0;
}
