/**
  ******************************************************************************
  * @file    afe.h
  * @brief   AFE (Analog Front End) - 模拟前端状态机 API
  * @note    基于 CubeMX v2 GPIO 引脚分配
  ******************************************************************************
  */
#ifndef AFE_H
#define AFE_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 耦合模式 ── */
typedef enum {
    AFE_COUPLING_DC = 0,
    AFE_COUPLING_AC = 1
} AFE_Coupling_t;

/* ── 衰减模式 ── */
typedef enum {
    AFE_ATTEN_1X  = 0,   /* 直通 (1x) */
    AFE_ATTEN_20X = 1    /* 1/20 衰减 */
} AFE_Attenuation_t;

/* ── PGA 增益 ── */
typedef enum {
    AFE_GAIN_1X  = 0,    /* PGA 旁路 (×1) */
    AFE_GAIN_20X = 1     /* PGA 使能 (×20) */
} AFE_Gain_t;

/* ── 通道状态快照 ── */
typedef struct {
    AFE_Coupling_t    coupling;
    AFE_Attenuation_t attenuation;
    AFE_Gain_t        gain;
} AFE_ChannelState_t;

/* ═══════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════ */

/**
 * @brief  初始化所有模拟前端通道
 * @note   默认：DC 耦合 + 1x 衰减 + PGA 旁路(×1)
 */
void AFE_Init(void);

/**
 * @brief  统一设置通道模式
 * @param  ch    通道编号 (1/2/3)
 * @param  cpl   耦合模式
 * @param  att   衰减模式
 * @param  gain  增益模式
 */
void AFE_SetChannelMode(uint8_t ch, AFE_Coupling_t cpl,
                        AFE_Attenuation_t att, AFE_Gain_t gain);

/**
 * @brief  获取通道当前状态
 * @param  ch  通道编号 (1/2/3)
 * @return     通道状态快照
 */
AFE_ChannelState_t AFE_GetChannelState(uint8_t ch);

/**
 * @brief  计算通道总增益倍率
 * @param  ch  通道编号 (1/2/3)
 * @return     实际电压增益 (1x / 1/20x / 20x / 1x)
 *
 * 组合表：
 *   att=1x  + gain=1x  → ×1    (0 ~ ±3.0V 差分)
 *   att=1x  + gain=20x → ×20   (0 ~ ±150mV)
 *   att=20x + gain=1x  → ×1/20 (0 ~ ±60V)
 *   att=20x + gain=20x → ×1    (0 ~ ±3.0V)
 */
float AFE_GetTotalGain(uint8_t ch);

/* ── 兼容旧 API（弃用，建议使用 AFE_SetChannelMode） ── */
typedef enum {
    AFE_CH1 = 1,
    AFE_CH2 = 2,
    AFE_CH3 = 3
} AFE_Channel;  /* 保留以兼容旧代码 */

void AFE_SetCoupling(AFE_Channel ch, AFE_Coupling_t coupling);
void AFE_SetAttenuation(AFE_Channel ch, uint8_t att);
void AFE_SetGain(AFE_Channel ch, AFE_Gain_t gain);

#ifdef __cplusplus
}
#endif

#endif /* AFE_H */