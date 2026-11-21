/**
  ******************************************************************************
  * @file    debug_print.h
  * @brief   USART1 调试输出 + 系统状态回传统一模块
  * @note    非阻塞 DMA 状态帧 + 阻塞调试打印
  ******************************************************************************
  */
#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════
 * 系统状态帧 (18 字节 packed)
 * ═══════════════════════════════════════════════════════════════ */

typedef struct __attribute__((packed)) {
    uint16_t header;       /* 0x5AA5 */
    uint16_t ch1_raw;      /* ADC1 */
    uint16_t ch2_raw;      /* ADC2 */
    uint16_t ch3_raw;      /* ADC5 */
    uint8_t  afe_status;   /* [7:6] CH1 [5:4] CH2 [3:2] CH3 [1:0]保留 */
    uint8_t  bit_res;      /* 14 */
    uint16_t sample_rate;  /* kSPS */
    uint16_t enc1_val;     /* TIM4 */
    uint16_t enc2_val;     /* TIM1 */
    uint8_t  checksum;
    uint8_t  tail;         /* 0xED */
} SysStatusFrame_t;

#define SYS_STATUS_HEADER  0x5AA5U
#define SYS_STATUS_TAIL    0xEDU

/* ═══════════════════════════════════════════════════════════════
 * 调试打印 API (阻塞)
 * ═══════════════════════════════════════════════════════════════ */

void Debug_Init(void);
void Debug_Print(const char *str);
void Debug_Println(const char *str);
void Debug_PrintHex(const char *label, uint32_t val);

/* ═══════════════════════════════════════════════════════════════
 * 系统状态回传 API (非阻塞 DMA)
 * ═══════════════════════════════════════════════════════════════ */

void     SysStatus_Init(void);
void     SysStatus_Send(void);
uint32_t SysStatus_GetInterval(void);
void     SysStatus_SetInterval(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_PRINT_H */