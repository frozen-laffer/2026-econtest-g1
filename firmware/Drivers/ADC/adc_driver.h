#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_BUF_SIZE    1024

/* ── [N6] 遗留 ADC 驱动总开关 ──────────────────────────────────────────
 * 置 0：ADC_Driver_Init() 编译为空函数。
 *       该函数会 HAL_ADC_DeInit(&hadc1/2/5) 并按「异步时钟 + 连续转换 +
 *       软件触发 + 16x 过采样」重建三路 ADC，与当前三路交替采集
 *       （TIM8 TRGO/TRGO2/CC1 触发 + 2.5 周期采样 + DMA NORMAL）
 *       完全冲突，一旦被调用采集立即失效。
 *       采集请统一走 Drivers/App/scope_acq.c。
 * 置 1：仅当需要复活旧的示波器菜单/LCD 显示链路时使用。 */
#define ADC_LEGACY_DRIVER_ENABLE   0

extern int16_t adc1_buf[ADC_BUF_SIZE];
extern int16_t adc2_buf[ADC_BUF_SIZE];
extern int16_t adc3_buf[ADC_BUF_SIZE];

/* ── 编码器菜单模式 ── */
/* 短按切换顺序与右侧面板显示顺序一致: Ch → V → T → Trg → Cpl → Ofs → Run */
typedef enum {
    MODE_CHANNEL = 0,
    MODE_VOLT_DIV,
    MODE_TIME_BASE,
    MODE_TRG_LEVEL,
    MODE_COUPLING,
    MODE_VERT_OFFSET,
    MODE_RUN_MODE
} EncoderMode_t;

/* ── 当前选中通道 ── */
typedef enum {
    CH_SEL_CH1 = 1,
    CH_SEL_CH2 = 2,
    CH_SEL_BOTH = 3
} ChannelSelect_t;

/* ── 运行模式 ── */
typedef enum {
    RUN_AUTO = 0,
    RUN_NORMAL,
    RUN_SINGLE
} RunMode_t;

/* ── 耦合模式 ── */
typedef enum {
    COUPLING_DC = 0,
    COUPLING_AC
} CouplingMode_t;

void ADC_Driver_Init(void);
float ADC_GetSampleRate(uint8_t ch);
void ADC_DisplayOnLCD(void);
void ADC_PrintRealtime(void);
uint8_t ADC_GetBufComplete(void);
RunMode_t ADC_GetRunMode(void);
CouplingMode_t ADC_GetCoupling(void);

/* ── 触摸菜单控制 API ── */
EncoderMode_t ADC_GetMenu(void);
void ADC_SetMenu(EncoderMode_t m);
void ADC_StepMenuUp(void);
void ADC_StepMenuDown(void);
int32_t ADC_GetTrg(void);
void ADC_ForceRedraw(void);

#ifdef __cplusplus
}
#endif

#endif