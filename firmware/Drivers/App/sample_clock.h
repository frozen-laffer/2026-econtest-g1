/**
  ******************************************************************************
  * @file    sample_clock.h
  * @brief   采样时基 —— TIM8 三相位触发（单路 / 三路交替采样）
  *
  *  时序设计
  *  ────────
  *  TIM8 内核时钟 170 MHz, PSC = 7  ->  tick = 21.25 MHz = 47.06 ns
  *  tick 取 47.06 ns 而不是 5.88 ns 的原因:
  *     触发脉冲（UPDATE / CC1）宽度只有 1 个 timer tick,
  *     而 ADC 时钟是 42.5 MHz (23.53 ns)。若 PSC=0 则脉冲仅 5.88 ns,
  *     短于一个 ADC 时钟周期, 有被漏检的风险。
  *     PSC=7 使脉冲宽达 2 个 ADC 时钟, 可靠触发。
  *
  *  一个周期 P 个 tick, 三个相位:
  *     CNT = 0      -> UPDATE  -> TRGO  -> ADC1
  *     CNT = P/3    -> CC1     -> CC1   -> ADC2
  *     CNT = 2P/3   -> OC2REF  -> TRGO2 -> ADC5
  *  三路交替时按时间顺序合并, 有效采样率 = 3 * 21.25 MHz / P
  *
  *  单路时只用其中一个 ADC, 采样率 = 21.25 MHz / P
  ******************************************************************************
  */
#ifndef SAMPLE_CLOCK_H
#define SAMPLE_CLOCK_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 时基常数 ─────────────────────────────────────────────── */
#define SAMPLE_CLK_HZ      170000000UL
#define SAMPLE_PSC         7U
#define SAMPLE_TICK_HZ     (SAMPLE_CLK_HZ / (SAMPLE_PSC + 1UL))   /* 21.25 MHz */

/* ── 采集模式 ─────────────────────────────────────────────── */
#define SAMPLE_MODE_SINGLE 0U    /* 单路, 一路 ADC 一次触发采 1 点 */
#define SAMPLE_MODE_INTER2  2u
#define SAMPLE_MODE_INTER3 1U    /* 三路交替, 每周期 3 个等间隔点 */

/* ── 档位（按每路周期 P 定义, 单位 tick） ────────────────────
 *   P   单路 fs      三路交替 fs     三路交替 Nyquist
 *   9   2.361 MSPS   7.083 MSPS      3.542 MHz
 *  12   1.771 MSPS   5.313 MSPS      2.656 MHz
 *  15   1.417 MSPS   4.250 MSPS      2.125 MHz
 *  18   1.181 MSPS   3.542 MSPS      1.771 MHz
 *
 *  约束: P 必须能被 3 整除（三相均分）;
 *        P * 47.06 ns 必须 >= 2.5 周期转换时间 352.9 ns -> P >= 8 -> P >= 9
 */
typedef enum {
    FS_G0 = 0,      /* P=9  */
    FS_G1 = 1,      /* P=12 */
    FS_G2 = 2,      /* P=15 */
    FS_G3 = 3,      /* P=18 */
    FS_COUNT
} SampleRate_t;

/* ── 当前生效的采样率 (Hz), 已按当前模式折算 ── */
extern volatile uint32_t g_fs_hz;

/* ── 当前档位索引 / 当前模式 ── */
extern volatile uint8_t  g_fs_index;
extern volatile uint8_t  g_fs_mode;

/**
  * @brief  初始化: 配置 TIM8 的 PSC / MMS / MMS2 / CH1 / CH2, 计数器保持停止
  * @note   必须在 MX_TIM8_Init() 之后调用
  */
void SampleClock_Init(void);

/**
  * @brief  切换档位（同时更新 ARR / CCR1 / CCR2, 并停止计数器）
  * @param  idx  档位索引 0..FS_COUNT-1
  * @retval 0=成功, -1=越界
  */
int SampleClock_Set(uint8_t idx);

/**
  * @brief  切换采集模式（只影响 g_fs_hz 的折算, 不改硬件）
  */
void SampleClock_SetMode(uint8_t mode);

/**
  * @brief  下一个档位（循环）
  */
uint8_t SampleClock_Next(void);

/**
  * @brief  某档位在某模式下的实际采样率 (Hz)
  */
uint32_t SampleClock_RateHz(uint8_t idx, uint8_t mode);

/**
  * @brief  当前档位 + 当前模式下的采样率 (Hz)
  */
uint32_t SampleClock_EffHz(void);

/**
  * @brief  停止触发: CEN=0, CNT=0
  * @note   必须在 arm ADC 之前调用, 保证第一个触发一定落在 UPDATE
  */
void SampleClock_StopCapture(void);

/**
  * @brief  启动触发: CNT = ARR (= P-1), 然后 CEN=1
  * @note   必须在三路 ADC 都 arm 完成之后调用。
  *         下一拍计数器溢出 -> UPDATE -> ADC1, 再 P/3 拍 CC1 -> ADC2,
  *         再 P/3 拍 OC2REF -> ADC5, 顺序固定为 1->2->5。
  */
void SampleClock_StartCapture(void);

/**
  * @brief  用"每路周期 P(tick)"直接设定（时钟探针用）
  * @param  p 每路周期，单位 TIM8 tick(47.06 ns)
  * @note   不查档位表、不更新 g_fs_index；CCR1=2P/3、CCR2=P/3 保持不变
  */
void SampleClock_SetP(uint16_t p);

#ifdef __cplusplus
}
#endif

#endif /* SAMPLE_CLOCK_H */
