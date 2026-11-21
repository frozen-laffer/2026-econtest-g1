/**
  ******************************************************************************
  * @file    sample_clock.c
  * @brief   TIM8 三相位采样时基实现
  ******************************************************************************
  */
#include "sample_clock.h"
#include "tim.h"
#include "../debug_print/debug_print.h"
#include <stdio.h>

/* ── 档位表: 每路周期 P (tick, tick = 47.06 ns) ──
 * [CLK] 时钟探针(2026-11-14)扫 P=4..16 的结果: P=4/5 掉一半, P=6 起探针报 ~3.45MSPS。
 *       但真实采集里 P=6 仍会"隔一次触发才转换"(100kHz 被读成 200kHz),
 *       说明 256 点探针看不出溢出, 必须留足余量。取 {8,10,12,16}:
 *       单路 2.656/2.125/1.771/1.328 MSPS -> Nyquist 1.33/1.06/0.885/0.664 MHz。
 *       题3 的 1 MHz 在 gear0(1.33MHz) 不折叠。 */
static const uint16_t s_p_tbl[FS_COUNT] = { 6U, 8U, 10U, 12U };

volatile uint32_t g_fs_hz    = 0;
volatile uint8_t  g_fs_index = FS_G0;
volatile uint8_t  g_fs_mode  = SAMPLE_MODE_SINGLE;

/* ═══════════════════════════════════════════════════════════════
 *  TIM8 静态配置:
 *    MMS  = UPDATE   -> TRGO  = 每周期溢出脉冲   -> ADC1 (相位 0)
 *    MMS2 = OC2REF   -> TRGO2 = OC2REF 上升沿     -> ADC5 (相位 2)
 *    CH1  = PWM2     -> CC1 事件落在 CCR1        -> ADC2 (相位 1)
 *    CH2  = PWM2     -> OC2REF 上升沿落在 CCR2   -> 供 TRGO2
 *
 *  注意: 不调用 HAL_TIM_PWM_Start, 避免 HAL_TIM_PWM_MspInit 去配置
 *        PC6/PC7 为复用输出; 这里只置 CCER 的使能位, 让内部参考产生事件。
 * ═══════════════════════════════════════════════════════════════ */
static void tim8_static_config(void)
{
    TIM_MasterConfigTypeDef m = {0};
    TIM_OC_InitTypeDef      oc = {0};

    /* CH1 / CH2: PWM 模式 2 —— OCxREF 在 CNT < CCRx 时为低, 到 CCRx 上升 */
    oc.OCMode       = TIM_OCMODE_PWM2;
    oc.Pulse        = 1U;
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    (void)HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_ConfigChannel(&htim8, &oc, TIM_CHANNEL_2);

    /* TRGO 源 / TRGO2 源 */
    m.MasterOutputTrigger  = TIM_TRGO_UPDATE;
    m.MasterOutputTrigger2 = TIM_TRGO2_OC2REF;
    m.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    (void)HAL_TIMEx_MasterConfigSynchronization(&htim8, &m);

    /* 使能通道比较事件（不接引脚, 只产生内部 CC/OC 信号） */
    htim8.Instance->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);

    /* 高级定时器需要 MOE 才输出, 内部事件虽不依赖 MOE, 一并打开更稳 */
    __HAL_TIM_MOE_ENABLE(&htim8);
}

/* ═══════════════════════════════════════════════════════════════ */

uint32_t SampleClock_RateHz(uint8_t idx, uint8_t mode)
{
    uint32_t p, fs;

    if (idx >= FS_COUNT) return 0UL;

    p  = (uint32_t)s_p_tbl[idx];
    fs = SAMPLE_TICK_HZ / p;

    if (mode == SAMPLE_MODE_INTER3) return fs * 3UL;
    if (mode == SAMPLE_MODE_INTER2) return fs * 2UL;   /* 双 ADC 硬件交替 */
    return fs;
}

uint32_t SampleClock_EffHz(void)
{
    return SampleClock_RateHz(g_fs_index, g_fs_mode);
}

void SampleClock_SetMode(uint8_t mode)
{
    /* 0=单路 1=三路软件交织 2=双 ADC 硬件交织
       (原来这里是三元式, 只认 INTER3, INTER2 会被吃成单路,
        导致包头 fs 报单路值) */
    g_fs_mode = mode;
    g_fs_hz   = SampleClock_RateHz(g_fs_index, g_fs_mode);
}

int SampleClock_Set(uint8_t idx)
{
    uint32_t p;

    if (idx >= FS_COUNT) return -1;

    p = (uint32_t)s_p_tbl[idx];

    /* 计数器停下才能安全改 ARR/CCR */
    __HAL_TIM_DISABLE(&htim8);

    htim8.Instance->ARR  = p - 1UL;
    htim8.Instance->CCR1 = (p / 3UL) * 2UL;  /* CH1/CC1 -> ADC5, 相位 2 (2P/3) */
    htim8.Instance->CCR2 = p / 3UL;          /* CH2/OC2REF -> ADC2, 相位 1 (P/3) */
    htim8.Instance->CNT  = 0UL;
    htim8.Instance->EGR  = TIM_EGR_UG;      /* 立即装入新值 */

    g_fs_index = idx;
    g_fs_hz    = SampleClock_RateHz(idx, g_fs_mode);

    return 0;
}

uint8_t SampleClock_Next(void)
{
    uint8_t nxt = (uint8_t)((g_fs_index + 1U) % FS_COUNT);
    (void)SampleClock_Set(nxt);
    return nxt;
}

/* 直接用"每路周期 P(tick)"设定，供时钟探针逐点扫描用。
 * 与 SampleClock_Set 的区别：不查档位表、不更新 g_fs_index。 */
void SampleClock_SetP(uint16_t p)
{
    if (p < 2U) p = 2U;

    __HAL_TIM_DISABLE(&htim8);

    htim8.Instance->ARR  = (uint32_t)p - 1UL;
    htim8.Instance->CCR1 = ((uint32_t)p / 3UL) * 2UL;
    htim8.Instance->CCR2 = (uint32_t)p / 3UL;
    htim8.Instance->CNT  = 0UL;
    htim8.Instance->EGR  = TIM_EGR_UG;

    g_fs_hz = SAMPLE_TICK_HZ / (uint32_t)p;
}

void SampleClock_StopCapture(void)
{
    __HAL_TIM_DISABLE(&htim8);
    htim8.Instance->CNT = 0UL;
}

void SampleClock_StartCapture(void)
{
    /* 先让计数器停在最后一个 tick: 使能后第一拍就溢出 -> UPDATE -> ADC1 */
    htim8.Instance->CNT = htim8.Instance->ARR;
    __HAL_TIM_ENABLE(&htim8);
}

void SampleClock_Init(void)
{
    __HAL_TIM_DISABLE(&htim8);

    /* 改预分频: 170 MHz / 8 = 21.25 MHz */
    htim8.Instance->PSC = SAMPLE_PSC;
    htim8.Instance->EGR = TIM_EGR_UG;

    tim8_static_config();

    g_fs_mode  = SAMPLE_MODE_SINGLE;
    g_fs_index = FS_G0;
    (void)SampleClock_Set(FS_G0);

    /* 计数器保持停止, 只在采集时由 SampleClock_StartCapture 打开 */
    __HAL_TIM_DISABLE(&htim8);

    {
        char buf[128];
        snprintf(buf, sizeof(buf),
            "[CLK] TIM8 tick=%lu Hz PSC=%u, gears=%d, single=%lu Hz, 3ch=%lu Hz\r\n",
            (unsigned long)SAMPLE_TICK_HZ, (unsigned)SAMPLE_PSC, (int)FS_COUNT,
            (unsigned long)SampleClock_RateHz(FS_G0, SAMPLE_MODE_SINGLE),
            (unsigned long)SampleClock_RateHz(FS_G0, SAMPLE_MODE_INTER3));
        Debug_Print(buf);
    }
}
