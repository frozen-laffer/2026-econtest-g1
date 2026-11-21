/**
  ******************************************************************************
  * @file    scope_acq.h
  * @brief   单次定长采集（单路 / 三路交替）
  *
  *  工作方式
  *  ────────
  *    上位机下发 CAPTURE(mode, ch, n) ->
  *    固件启动 ADC + DMA(NORMAL) 采满 n 点后自动停止 ->
  *    固件把 n 点原始差分码（int16 补码, -2047..+2047）连同 32 字节包头回传
  *
  *  两种模式
  *  ────────
  *    SCOPE_MODE_SINGLE : 只启动一路 ADC, 每周期 1 个点
  *    SCOPE_MODE_INTER3 : 三路 ADC 各错开 P/3, 每周期 3 个点, 有效速率 ×3
  *                        合并顺序严格按时间: ADC1(相位0) -> ADC2(相位1)
  *                        -> ADC5(相位2)
  ******************************************************************************
  */
#ifndef SCOPE_ACQ_H
#define SCOPE_ACQ_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 每路最大点数（8192 x 2B = 16 KB, 三路共 48 KB） ── */
#define SCOPE_MAX_M        8192u
/* ── 合并后的最大总点数 ── */
#define SCOPE_MAX_N        (3u * SCOPE_MAX_M)

/* ── 通道编号（与 ADC1/ADC2/ADC5 对应, 同时决定相位） ── */
#define SCOPE_CH1          0u      /* ADC1  PA0/PA1  相位 0 (UPDATE)  */
#define SCOPE_CH2          1u      /* ADC2  PA6/PA7  相位 1 (CC1)     */
#define SCOPE_CH5          2u      /* ADC5  PA8/PA9  相位 2 (OC2REF)  */

/* ── 采集模式 ── */
#define SCOPE_MODE_SINGLE  0u
#define SCOPE_MODE_INTER3  1u
/* 双 ADC 硬件交替（ADC1=master, ADC2=slave, ADC_DUALMODE_INTERL）
 * 单信号同时进 PA0/PA1 与 PA6/PA7（前端一分三已保证同源），
 * 硬件保证两路严格错开半个转换周期 -> 有效速率 x2，无软件时序误差。 */
#define SCOPE_MODE_INTER2  2u

/* 上电时扫一遍 P=4..16 打印实测采样率，用于核对 ADC 时钟树。
   默认关闭：会多花约 1 s 并刷 11 行日志，需要时改成 1。 */
#ifndef SCOPE_BOOT_PROBE
#define SCOPE_BOOT_PROBE   0
#endif

/* ── 状态 ── */
extern volatile uint8_t  g_scope_done;      /* 1 = 采满 */
extern volatile uint32_t g_scope_n;         /* 合并后的总点数 */
extern volatile uint8_t  g_scope_mode;      /* SCOPE_MODE_* */
extern volatile uint8_t  g_scope_sel_ch;    /* 单路模式下的通道 */
extern volatile uint8_t  g_scope_mask;
extern volatile uint32_t g_scope_cyc;      /* 本次采集耗时(DWT 周期, 170MHz) */      /* bit0..2: 已完成的通道 */

/* 调试：1 = 单端模式（ScopeAcq_Fill 输出 0..4095 的引脚电压码，不做符号扩展） */
extern volatile uint8_t  g_scope_single_ended;

/**
  * @brief  初始化（必须在 MX_ADCx_Init / SampleClock_Init 之后调用）
  */
void ScopeAcq_Init(void);

/**
  * @brief  启动一次单次采集（非阻塞）
  * @param  mode     SCOPE_MODE_SINGLE / SCOPE_MODE_INTER3
  * @param  ch       单路模式下的通道（SCOPE_CH1/CH2/CH5）
  * @param  n_total  合并后的总点数（三路时必须能被 3 整除）
  * @retval 0=成功, <0=失败
  */
int ScopeAcq_Start(uint8_t mode, uint8_t ch, uint16_t n_total);

/**
  * @brief  是否采满
  */
uint8_t ScopeAcq_Done(void);

/**
  * @brief  中止采集（幂等）
  */
void ScopeAcq_Abort(void);

/**
  * @brief  DMA 完成中断里调用（由 stm32g4xx_it.c 转发）
  */
void ScopeAcq_OnCplt(uint8_t ch);

/**
  * @brief  合并后的总点数
  */
uint32_t ScopeAcq_Total(void);

/**
  * @brief  取合并后的连续一段样本
  * @param  start   起始序号（合并后的序号）
  * @param  dst     目标缓冲（uint16, 原始差分码）
  * @param  max_pts 最多取多少点
  * @retval 实际取出的点数
  * @note   三路交替模式下现场交织, 不占用额外的合并缓冲区
  */
uint32_t ScopeAcq_Fill(uint32_t start, uint16_t *dst, uint32_t max_pts);

/* 调试：直接取 DMA 原始字（不做符号扩展），用于核对 ADC 数据格式 */
uint32_t ScopeAcq_RawPeek(uint32_t start, uint16_t *dst, uint32_t max_pts);

/**
  * @brief  三路交替自检: 实采一次, 检查三路 DMA 是否都完成
  * @retval 1=三路都通, 0=有通道未触发（阻塞, 最多 200 ms）
  */
uint8_t ScopeAcq_SelfTest(void);

/**
  * @brief  时钟探针：逐点扫描每路周期 P(4..16 tick)，用 DWT 反算真实采样率
  * @note   阻塞；结果经 Debug_Print 打到串口。
  *         用途：判定 ADC 内核时钟是否被额外二分（P 过小时 fs 会掉一半）
  */
void ScopeAcq_ClockProbe(void);

#ifdef __cplusplus
}
#endif

#endif /* SCOPE_ACQ_H */
