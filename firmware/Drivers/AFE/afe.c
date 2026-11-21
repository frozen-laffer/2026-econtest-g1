/**
  ******************************************************************************
  * @file    afe.c
  * @brief   AFE (Analog Front End) - 模拟前端三通道状态机
  * @note    CubeMX v2 引脚分配：
  *          CH1: AC/DC→PC13, ATT→PB9, PGA_EN→PC3
  *          CH2: AC/DC→PA3,  ATT→PA2, PGA_EN→PA4
  *          CH3: AC/DC→PB0,  ATT→PA5, PGA_EN→PB1
  ******************************************************************************
  */
#include "afe.h"
#include "gpio.h"

/* ── 三通道状态 ── */
static AFE_ChannelState_t g_state[3];  /* 索引 0/1/2 → CH1/CH2/CH3 */

/* ═══════════════════════════════════════════════════════════════
 * 内部辅助：GPIO 操作表
 * ═══════════════════════════════════════════════════════════════ */
typedef struct {
    GPIO_TypeDef *port_cpl;
    uint16_t      pin_cpl;
    GPIO_TypeDef *port_att;
    uint16_t      pin_att;
    GPIO_TypeDef *port_gain;
    uint16_t      pin_gain;
} AFE_PinMap_t;

static const AFE_PinMap_t g_pinmap[3] = {
    /* CH1 */ { RELAY1_AC_DC_GPIO_Port, RELAY1_AC_DC_Pin,
                RELAY1_ATT_GPIO_Port,   RELAY1_ATT_Pin,
                PGA1_EN_GPIO_Port,      PGA1_EN_Pin },
    /* CH2 */ { RELAY2_AC_DC_GPIO_Port, RELAY2_AC_DC_Pin,
                RELAY2_ATT_GPIO_Port,   RELAY2_ATT_Pin,
                PGA2_EN_GPIO_Port,      PGA2_EN_Pin },
    /* CH3 */ { RELAY3_AC_DC_GPIO_Port, RELAY3_AC_DC_Pin,
                RELAY3_ATT_GPIO_Port,   RELAY3_ATT_Pin,
                PGA3_EN_GPIO_Port,      PGA3_EN_Pin },
};

/* ═══════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════ */

/**
 * @brief  初始化所有通道：DC 耦合 + 1x 衰减 + PGA 旁路
 */
void AFE_Init(void)
{
    uint8_t i;
    for (i = 0; i < 3; i++) {
        AFE_SetChannelMode((uint8_t)(i + 1),
                           AFE_COUPLING_DC,
                           AFE_ATTEN_1X,
                           AFE_GAIN_1X);
    }
}

/**
 * @brief  统一设置通道模式（唯一对外 API）
 * @note   [P0-6] 极性已按原理图网表核实修正
 *
 *  硬件实际（见《硬件事实基线_由网表核实.md》§3.1）:
 *    STM32 GPIO ──1kΩ──► AO3400A 栅极
 *    AO3400A: 源极接 DGND, 漏极接继电器线圈(-), 线圈(+)接 +12V
 *    续流二极管 M7 (D2~D7): 阳极→+12V, 阴极→漏极
 *
 *    => GPIO 高电平 = MOSFET 导通 = 继电器吸合  ★高=吸合★
 *    => 上电复位时 GPIO 为低 = 全部释放（安全默认态）
 *
 *  ⚠️ 切换前应先停止采集(关 TIM8 MOE), 否则会采到继电器动作尖峰。
 *     参考 W5500_Switch_Capture_Mode() 的 MOE 开关写法。
 */
void AFE_SetChannelMode(uint8_t ch, AFE_Coupling_t cpl,
                        AFE_Attenuation_t att, AFE_Gain_t gain)
{
    if (ch < 1 || ch > 3) return;

    const AFE_PinMap_t *p = &g_pinmap[ch - 1];

    /* 耦合继电器：AC 耦合 = 吸合 = 高（把隔直电容 C79 串入信号通路）
     *             DC 耦合 = 释放 = 低（直通）
     *  网表: IN1+ → K1-3(动触点); K1-4/K1-5 → C79(隔直) */
    HAL_GPIO_WritePin(p->port_cpl, p->pin_cpl,
        (cpl == AFE_COUPLING_AC) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* 衰减继电器：高=吸合。具体对应 1x 还是 20x 需实测导通后确认(基线 §10-3)
     * 此处保持原语义约定，极性统一为"高=吸合" */
    HAL_GPIO_WritePin(p->port_att, p->pin_att,
        (att == AFE_ATTEN_20X) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* 增益切换：BL1551B ENB 引脚
     *  网表: ENB(PGAx_EN) 经 U18/U21/U24 选择 INA128 的 R_G
     *        A1 → R109 = 2.61kΩ  → G = 1 + 50k/2.61k = 20.16
     *        A2 → R110 = 49.9kΩ  → G = 1 + 50k/49.9k = 2.002
     *  BL1551B 为低有效使能(ENB)，但具体哪一路选通取决于 A/B 接线，
     *  ⚠️ 需实测确认。此处按"高=GAIN_20X"约定，与 AFE_GetTotalGain 保持一致。 */
    HAL_GPIO_WritePin(p->port_gain, p->pin_gain,
        (gain == AFE_GAIN_20X) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* 更新状态快照 */
    g_state[ch - 1].coupling    = cpl;
    g_state[ch - 1].attenuation = att;
    g_state[ch - 1].gain        = gain;
}

/**
 * @brief  获取通道当前状态
 */
AFE_ChannelState_t AFE_GetChannelState(uint8_t ch)
{
    if (ch < 1 || ch > 3) {
        AFE_ChannelState_t empty = {AFE_COUPLING_DC, AFE_ATTEN_1X, AFE_GAIN_1X};
        return empty;
    }
    return g_state[ch - 1];
}

/**
 * @brief  计算总增益（INA128 × THS4521 FDA）
 * @note   [P0-10] 已按硬件实际阻值重写
 *
 *  硬件实际（见《硬件事实基线_由网表核实.md》§4）:
 *    INA128 (U19): G_ina = 1 + 50kΩ/R_G
 *        R_G = 49.9 kΩ (R110) → G_ina = 2.002
 *        R_G = 2.61 kΩ (R109) → G_ina = 20.157
 *    THS4521 FDA (U20): A_fda = Rf/Rg = 2.49k/1.27k = 1.9606
 *
 *  返回值为 INA 与 FDA 的级联增益（不含衰减继电器与 ADC 折算）
 *
 *  ⚠️ 标称值仅供参考。精确系数必须经直流标定后使用
 *     （见《树莓派任务书》§7 标定 SOP）。预期:
 *       ×2 档 总增益 = 3.925
 *       ×20 档总增益 = 39.52
 */
#define AFE_A_FDA        1.9606f
#define AFE_G_INA_X2     2.002f     /* R_G = 49.9 kΩ */
#define AFE_G_INA_X20    20.157f    /* R_G = 2.61 kΩ */

float AFE_GetTotalGain(uint8_t ch)
{
    AFE_ChannelState_t s = AFE_GetChannelState(ch);

    if (s.attenuation == AFE_ATTEN_20X) {
        /* 衰减档: 额外 1/20 分压（具体拓扑待实测, 基线 §10-3） */
        return (s.gain == AFE_GAIN_20X)
             ? (AFE_G_INA_X20 * AFE_A_FDA / 20.0f)
             : (AFE_G_INA_X2  * AFE_A_FDA / 20.0f);
    } else {
        return (s.gain == AFE_GAIN_20X)
             ? (AFE_G_INA_X20 * AFE_A_FDA)      /* 39.52 */
             : (AFE_G_INA_X2  * AFE_A_FDA);     /*  3.925 */
    }
}

/* ═══════════════════════════════════════════════════════════════
 * 兼容旧 API
 * ═══════════════════════════════════════════════════════════════ */

void AFE_SetCoupling(AFE_Channel ch, AFE_Coupling_t coupling)
{
    if (ch < AFE_CH1 || ch > AFE_CH3) return;
    /* 保持其他参数不变 */
    AFE_ChannelState_t s = g_state[ch - 1];
    AFE_SetChannelMode((uint8_t)ch, coupling, s.attenuation, s.gain);
}

void AFE_SetAttenuation(AFE_Channel ch, uint8_t att)
{
    if (ch < AFE_CH1 || ch > AFE_CH3) return;
    AFE_ChannelState_t s = g_state[ch - 1];
    AFE_SetChannelMode((uint8_t)ch, s.coupling,
        att ? AFE_ATTEN_20X : AFE_ATTEN_1X, s.gain);
}

void AFE_SetGain(AFE_Channel ch, AFE_Gain_t gain)
{
    if (ch < AFE_CH1 || ch > AFE_CH3) return;
    AFE_ChannelState_t s = g_state[ch - 1];
    AFE_SetChannelMode((uint8_t)ch, s.coupling, s.attenuation, gain);
}