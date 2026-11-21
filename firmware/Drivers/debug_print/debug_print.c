/**
  ******************************************************************************
  * @file    debug_print.c
  * @brief   USART1 调试输出 + 系统状态回传实现
  ******************************************************************************
  */
#include "debug_print.h"
#include "usart.h"
#include "../AFE/afe.h"
#include "../W5500/Ethernet/W5500/w5500.h"
#include <string.h>
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════
 * 半主机完全禁用桩函数（嵌入 debug_print.c 确保一定链接）
 * armcc 标准 C 库启动时会 _initio → freene → _sys_open 走半主机，
 * 触发 BKPT 0xAB → OpenOCD halt。提供以下空实现彻底绕过。
 * ═══════════════════════════════════════════════════════════════ */
#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; };
FILE __stdout;

int fputc(int ch, FILE *f) { (void)f; return ch; }
void _ttywrch(int ch) { (void)ch; }

typedef int FILEHANDLE;

FILEHANDLE _sys_open(const char *name, int openmode) { return -1; }
int _sys_close(FILEHANDLE fh) { return 0; }
int _sys_write(FILEHANDLE fh, const unsigned char *buf, unsigned len, int mode) { return 0; }
int _sys_read(FILEHANDLE fh, unsigned char *buf, unsigned len, int mode) { return 0; }
int _sys_istty(FILEHANDLE fh) { return 0; }
int _sys_seek(FILEHANDLE fh, long pos) { return 0; }
long _sys_flen(FILEHANDLE fh) { return 0; }
void _sys_exit(int return_code) { while (1); }

static UART_HandleTypeDef *g_debug_uart = &huart1;
static uint32_t g_interval_ms = 50;
static uint32_t g_last_send_tick = 0;
static volatile uint8_t g_tx_busy = 0;
extern int16_t adc1_buf[];
extern int16_t adc2_buf[];
extern int16_t adc3_buf[];

void Debug_Init(void) { g_debug_uart = &huart1; }

void Debug_Print(const char *str)
{ if(g_debug_uart) HAL_UART_Transmit(g_debug_uart,(uint8_t*)str,strlen(str),HAL_MAX_DELAY); }

void Debug_Println(const char *str) { Debug_Print(str); Debug_Print("\r\n"); }

void Debug_PrintHex(const char *label, uint32_t val)
{ char b[32]; snprintf(b,32,"%s%08lX",label,(unsigned long)val); Debug_Println(b); }

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h)
{ if(h->Instance==USART1) g_tx_busy=0; }

static uint8_t afe_pack(void)
{
    uint8_t s=0;
    if(AFE_GetChannelState(1).coupling==AFE_COUPLING_AC) s|=0x80;
    if(AFE_GetChannelState(2).coupling==AFE_COUPLING_AC) s|=0x20;
    if(AFE_GetChannelState(3).coupling==AFE_COUPLING_AC) s|=0x08;
    return s;
}

void SysStatus_Init(void) {
    g_last_send_tick = 0;
    g_tx_busy = 0;
    g_interval_ms = 50;
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void SysStatus_Send(void)
{
    uint32_t t=HAL_GetTick();
    if((t-g_last_send_tick)<g_interval_ms||g_tx_busy) return;

    static char tx_buf[250];
    extern TIM_HandleTypeDef htim4,htim1;
    extern uint8_t g_w5500_state;
    extern uint8_t g_w5500_ip[4];
    extern uint8_t g_w5500_mac[6];

    uint32_t spi_sr  = SPI3->SR;
    uint32_t spi_cr1 = SPI3->CR1;

    int enc1_sw = (HAL_GPIO_ReadPin(ENC1_SW_GPIO_Port, ENC1_SW_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    int enc2_sw = (HAL_GPIO_ReadPin(ENC2_SW_GPIO_Port, ENC2_SW_Pin) == GPIO_PIN_RESET) ? 1 : 0;

#if 0 /* TODO: 待 w5500_self.c 编译后启用 */
    /* TCP Server 状态监控（仅读不写，绝不干扰硬件状态机） */
    uint8_t s0_sr = 0xFF;
    uint8_t s0_ir = 0xFF;
    static uint8_t prev_sr = 0xFF;
    extern volatile uint8_t g_system_ready;
    if (g_system_ready) {
        W5500_SPI_LOCK();                           /* 原子读取 Sn_SR + Sn_IR */
        s0_sr = W5500_ReadSnSR(0);
        s0_ir = W5500_ReadSnIR(0);
        W5500_SPI_UNLOCK();
        /* 状态变化时打印事件 → 重点监控 SYN_RECV(0x15) 和 ESTABLISHED(0x17) */
        if (s0_sr != prev_sr) {
            char evt_buf[64];
            const char *sname = "???";
            switch (s0_sr) {
                case 0x00: sname = "CLOSED";      break;
                case 0x13: sname = "INIT";        break;
                case 0x14: sname = "LISTEN";      break;
                case 0x15: sname = "SYN_RECV";    break;  /* ← 握手确认 */
                case 0x17: sname = "ESTABLISHED"; break;  /* ← TCP 已建立! */
                default:   break;
            }
            snprintf(evt_buf, sizeof(evt_buf),
                "[W5500] SR:%02X→%02X(%s) IR:0x%02X\r\n", prev_sr, s0_sr, sname, s0_ir);
            Debug_Print(evt_buf);
            prev_sr = s0_sr;
        }
        /* 仅打印 CON/DISCON/TIMEOUT 事件，不写回 IR 寄存器！ */
        if (s0_ir & 0x15) {
            char ir_buf[48];
            snprintf(ir_buf, sizeof(ir_buf),
                "[W5500] IR_event:0x%02X\r\n", s0_ir);
            Debug_Print(ir_buf);
        }
    }
#endif /* w5500_self 暂未编译 */
    uint8_t s0_sr = 0xFF;
    (void)s0_sr;

    int len = snprintf(tx_buf, sizeof(tx_buf),
        "CH1:%u CH2:%u CH3:%u | ENC1:%d ENC2:%d | SR:0x%02X\r\n",
        (uint16_t)adc1_buf[0],
        (uint16_t)adc2_buf[0],
        (uint16_t)adc3_buf[0],
        (int16_t)__HAL_TIM_GET_COUNTER(&htim4),
        (int16_t)__HAL_TIM_GET_COUNTER(&htim1),
        s0_sr);

    if (len > 0) {
        /* 若 USART1 无 DMA，降级为 IT 发送 */
        HAL_StatusTypeDef status;
        if (huart1.hdmatx != NULL) {
            status = HAL_UART_Transmit_DMA(&huart1, (uint8_t*)tx_buf, len);
        } else {
            status = HAL_UART_Transmit_IT(&huart1, (uint8_t*)tx_buf, len);
        }
        if (status == HAL_OK) {
            g_tx_busy = 1;
            g_last_send_tick = t;
        } else {
            g_tx_busy = 0;
        }

#if 0 /* TODO: W5500 UDP 推送 -- 待 w5500_self.c 编译后启用 */
        /* ── W5500 UDP 推送：当网络就绪时同步发送到上位机 ── */
        extern uint8_t g_w5500_state;
        if (g_w5500_state == 2 && W5500_ReadSnSR(0) == 0x17) {
            extern void W5500_SendData(const uint8_t *data, uint16_t len);
            W5500_SendData((const uint8_t *)tx_buf, (uint16_t)len);
        }
#endif
    }
}

uint32_t SysStatus_GetInterval(void){return g_interval_ms;}
void SysStatus_SetInterval(uint32_t ms){g_interval_ms=ms>0?ms:10;}