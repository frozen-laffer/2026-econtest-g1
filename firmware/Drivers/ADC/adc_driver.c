#include "adc_driver.h"
#include "adc.h"
/* #include "dma.h" */  /* ── 适配：dma.h 不存在，DMA 类型已通过 stm32g4xx_hal.h 间接包含 ── */
#include "usart.h"
#include "../TFT_Screen/ILI9488.h"
#include "../debug_print/debug_print.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../AFE/afe.h"

#if defined(__CC_ARM)
__attribute__((section("DMA_RAM"), zero_init))
#endif
int16_t adc1_buf[ADC_BUF_SIZE];
#if defined(__CC_ARM)
__attribute__((section("DMA_RAM"), zero_init))
#endif
int16_t adc2_buf[ADC_BUF_SIZE];
#if defined(__CC_ARM)
__attribute__((section("DMA_RAM"), zero_init))
#endif
int16_t adc3_buf[ADC_BUF_SIZE];

/* ── CubeMX v2 定义在 adc.c/tim.c 中 ── */
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc5;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

static volatile uint32_t g_buf1_cnt = 0, g_buf2_cnt = 0, g_buf3_cnt = 0;
static volatile uint32_t g_last_tick = 0;
static volatile float    g_sps1 = 0.0f, g_sps2 = 0.0f;

#define VREF              3.0f
#define ADC_MAX           32767

static float    g_v_div = 0.01f, g_s_div = 0.2f;
static int      g_vi = 0, g_si = 0;
static const float v_tbl[] = { 0.01f, 0.1f, 1.0f };
static const float s_tbl[] = { 0.2f, 0.0002f, 0.00002f };
static const char  *s_lbl[] = { "0.2s/div", "0.2ms/div", "20us/div" };

static EncoderMode_t   g_menu    = MODE_CHANNEL;
static RunMode_t       g_run     = RUN_AUTO;
static CouplingMode_t  g_coupling = COUPLING_DC;
static ChannelSelect_t g_ch_sel  = CH_SEL_BOTH;
static int32_t  g_vert_offset_mv = 0;
static int32_t  g_trg = 0;
static int16_t  g_cap[480];
static int      g_cv = 0, g_last_enc = 0;

typedef enum { BI, BD, BP, BL, BW } BS;
static BS       g_btn  = BI;
static uint32_t g_bt0 = 0, g_bt1 = 0;

#define SW  240
#define SH  320
#define SX  0
#define SY  0
#define SP  240
#define HS  480
#define IX  240
#define DH  24
#define DV  40

static int16_t  g_h1[HS], g_h2[HS];
static uint32_t g_hi = 0, g_lw = 0, g_wu = 6667;
static uint32_t g_fc = 0, g_ft = 0;
static float    g_fps = 0;

static int g_meas_frame = 0;

/* ── G474 适配：DMA Channel 模型（非 Stream） ── */
#if ADC_LEGACY_DRIVER_ENABLE
static void dma_reinit_ch(DMA_HandleTypeDef *h, DMA_Channel_TypeDef *ch, uint32_t req) {
    HAL_DMA_DeInit(h); h->State = HAL_DMA_STATE_RESET;
    h->Instance = ch; h->Init.Request = req;
    h->Init.Direction = DMA_PERIPH_TO_MEMORY;
    h->Init.PeriphInc = DMA_PINC_DISABLE; h->Init.MemInc = DMA_MINC_ENABLE;
    h->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    h->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    h->Init.Mode = DMA_CIRCULAR; h->Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(h) != HAL_OK) Error_Handler();
}
#endif /* ADC_LEGACY_DRIVER_ENABLE */

static int find_trig(const int16_t *buf, int len, int32_t lv, int hy) {
    int hi = lv + hy, lo = lv - hy;
    int st = (buf[0] < lo) ? 0 : 1;
    for (int i = 1; i < len - 200; i++) {
        if (st == 0 && buf[i] > hi) return i;
        if (st == 1 && buf[i] < lo) st = 0;
    }
    return -1;
}

void ADC_Driver_Init(void) {
#if ADC_LEGACY_DRIVER_ENABLE
    ADC_ChannelConfTypeDef c = {0};

    /* ═══════════ G474 公共参数模板 ═══════════ */
    /* 原生 12-bit ADC + 硬件过采样 OVSR=16+OVSS=2 → 14-bit 有效位 */
    ADC_InitTypeDef adc_template = {0};
    adc_template.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV2;
    adc_template.Resolution            = ADC_RESOLUTION_12B;       /* G4 原生 12-bit */
    adc_template.DataAlign             = ADC_DATAALIGN_RIGHT;
    adc_template.GainCompensation      = 0;
    adc_template.ScanConvMode          = ADC_SCAN_DISABLE;
    adc_template.EOCSelection          = ADC_EOC_SINGLE_CONV;
    adc_template.LowPowerAutoWait      = DISABLE;
    adc_template.ContinuousConvMode    = ENABLE;
    adc_template.NbrOfConversion       = 1;
    adc_template.DiscontinuousConvMode = DISABLE;
    adc_template.ExternalTrigConv      = ADC_SOFTWARE_START;
    adc_template.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc_template.DMAContinuousRequests = ENABLE;                   /* DMA 循环模式 */
    adc_template.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    /* 【Bug #1 修复】过采样使能：OVSR=16x + OVSS=2 → 14-bit 有效位 */
    adc_template.OversamplingMode      = ENABLE;
    adc_template.Oversampling.Ratio                 = 16;
    adc_template.Oversampling.RightBitShift         = ADC_RIGHTBITSHIFT_2;
    adc_template.Oversampling.TriggeredMode         = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
    adc_template.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;

    /* ═══════════ ADC1 (CH1): PA0=IN1+/PA1=IN2- ═══════════ */
    HAL_ADC_Stop(&hadc1); HAL_ADC_DeInit(&hadc1);
    hadc1.Instance = ADC1; hadc1.Init = adc_template;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    HAL_ADC_Stop(&hadc1);
    if (ADC1->CR & ADC_CR_ADEN) { ADC1->CR |= ADC_CR_ADDIS; uint32_t t=50000; while((ADC1->CR & ADC_CR_ADEN) && --t) __NOP(); if(!t) Error_Handler(); }

    c.Channel = ADC_CHANNEL_1; c.Rank = ADC_REGULAR_RANK_1;
    c.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;  /* 最大可用值 */
    c.SingleDiff = ADC_DIFFERENTIAL_ENDED; c.OffsetNumber = ADC_OFFSET_NONE;
    HAL_ADC_ConfigChannel(&hadc1, &c);
    ADC1->DIFSEL |= (1UL << 1);

    ADC1->CR |= ADC_CR_ADEN;
    { uint32_t t=50000; while(!(ADC1->ISR & ADC_ISR_ADRDY) && --t) __NOP(); if(!t) Error_Handler(); }
    ADC1->ISR |= ADC_ISR_ADRDY;

    dma_reinit_ch(&hdma_adc1, DMA1_Channel1, DMA_REQUEST_ADC1);
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buf, ADC_BUF_SIZE);

    /* ═══════════ ADC2 (CH2): PA6=IN3+/PA7=IN4- ═══════════ */
    HAL_ADC_Stop(&hadc2); HAL_ADC_DeInit(&hadc2);
    hadc2.Instance = ADC2; hadc2.Init = adc_template;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();

    HAL_ADC_Stop(&hadc2);
    if (ADC2->CR & ADC_CR_ADEN) { ADC2->CR |= ADC_CR_ADDIS; uint32_t t=50000; while((ADC2->CR & ADC_CR_ADEN) && --t) __NOP(); if(!t) Error_Handler(); }

    c.Channel = ADC_CHANNEL_3; c.Rank = ADC_REGULAR_RANK_1;
    c.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    c.SingleDiff = ADC_DIFFERENTIAL_ENDED; c.OffsetNumber = ADC_OFFSET_NONE;
    HAL_ADC_ConfigChannel(&hadc2, &c);
    ADC2->DIFSEL |= (1UL << 3);

    ADC2->CR |= ADC_CR_ADEN;
    { uint32_t t=50000; while(!(ADC2->ISR & ADC_ISR_ADRDY) && --t) __NOP(); if(!t) Error_Handler(); }
    ADC2->ISR |= ADC_ISR_ADRDY;

    dma_reinit_ch(&hdma_adc2, DMA1_Channel2, DMA_REQUEST_ADC2);
    __HAL_LINKDMA(&hadc2, DMA_Handle, hdma_adc2);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_DIFFERENTIAL_ENDED);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buf, ADC_BUF_SIZE);

    /* ═══════════ ADC5 (CH3): PA8=IN1+/PA9=IN2- ═══════════ */
    HAL_ADC_Stop(&hadc5); HAL_ADC_DeInit(&hadc5);
    hadc5.Instance = ADC5; hadc5.Init = adc_template;
    if (HAL_ADC_Init(&hadc5) != HAL_OK) Error_Handler();

    HAL_ADC_Stop(&hadc5);
    if (ADC5->CR & ADC_CR_ADEN) { ADC5->CR |= ADC_CR_ADDIS; uint32_t t=50000; while((ADC5->CR & ADC_CR_ADEN) && --t) __NOP(); if(!t) Error_Handler(); }

    c.Channel = ADC_CHANNEL_1; c.Rank = ADC_REGULAR_RANK_1;
    c.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    c.SingleDiff = ADC_DIFFERENTIAL_ENDED; c.OffsetNumber = ADC_OFFSET_NONE;
    HAL_ADC_ConfigChannel(&hadc5, &c);
    ADC5->DIFSEL |= (1UL << 1);

    ADC5->CR |= ADC_CR_ADEN;
    { uint32_t t=50000; while(!(ADC5->ISR & ADC_ISR_ADRDY) && --t) __NOP(); if(!t) Error_Handler(); }
    ADC5->ISR |= ADC_ISR_ADRDY;

    dma_reinit_ch(&hdma_adc5, DMA1_Channel3, DMA_REQUEST_ADC5);
    __HAL_LINKDMA(&hadc5, DMA_Handle, hdma_adc5);
    HAL_ADCEx_Calibration_Start(&hadc5, ADC_DIFFERENTIAL_ENDED);
    HAL_ADC_Start_DMA(&hadc5, (uint32_t *)adc3_buf, ADC_BUF_SIZE);

    /* ═══════════ 启动编码器 ═══════════ */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    g_last_enc = __HAL_TIM_GET_COUNTER(&htim4);
    g_last_tick = HAL_GetTick(); g_lw = g_last_tick; g_ft = g_last_tick;
    g_wu = (uint32_t)(g_s_div * 1000000 * 10 / SP);

    /* ── 诊断 ── */
    {
        char diag[256];
        snprintf(diag, 256,
            "\r\n[ADC DIAG] DIFSEL1=0x%08lX DIFSEL2=0x%08lX DIFSEL5=0x%08lX\r\n"
            "[ADC DIAG] OVSR=16x OVSS=2 -> 14-bit effective\r\n",
            (unsigned long)ADC1->DIFSEL, (unsigned long)ADC2->DIFSEL, (unsigned long)ADC5->DIFSEL);
        HAL_UART_Transmit(&huart1, (uint8_t *)diag, strlen(diag), 500);
    }
#endif /* ADC_LEGACY_DRIVER_ENABLE */
}

float ADC_GetSampleRate(uint8_t ch) { return (ch == 2) ? g_sps2 : g_sps1; }
RunMode_t ADC_GetRunMode(void) { return g_run; }

static void btn_proc(void) {
    /* ── CubeMX v2: ENC1_SW = PA10 ── */
    int sw = (HAL_GPIO_ReadPin(ENC1_SW_GPIO_Port, ENC1_SW_Pin) == GPIO_PIN_RESET);
    uint32_t n = HAL_GetTick();
    switch (g_btn) {
    case BI: if (sw) { g_btn = BD; g_bt0 = n; } break;
    case BD: if (!sw) g_btn = BI; else if (n - g_bt0 >= 40) { g_btn = BP; g_bt1 = n; } break;
    case BP: if (!sw) { g_menu = (EncoderMode_t)((g_menu + 1) % 7); g_btn = BI; } else if (n - g_bt1 >= 800) { if (g_menu == MODE_TRG_LEVEL) { int32_t s = 0; for (int i = 0; i < 100; i++) s += (int16_t)adc1_buf[i]; g_trg = s / 100; } g_btn = BL; g_bt1 = n; } break;
    case BL: if (!sw) g_btn = BI; break;
    default: g_btn = BI;
    }
}

static void enc_proc(void) {
    int32_t cur = __HAL_TIM_GET_COUNTER(&htim4);
    int32_t d = cur - g_last_enc;
    if (d == 0) return;
    g_last_enc = cur;
    int dir = (d > 0) ? 1 : -1;
    switch (g_menu) {
    case MODE_TRG_LEVEL: g_trg += dir * 200; if (g_trg > 32767) g_trg = 32767; if (g_trg < -32767) g_trg = -32767; break;
    case MODE_TIME_BASE: g_si = (g_si + dir + 3) % 3; g_s_div = s_tbl[g_si]; g_wu = (uint32_t)(g_s_div * 1000000 * 10 / SP); break;
    case MODE_VOLT_DIV:
        g_vi = (g_vi + dir + 3) % 3; g_v_div = v_tbl[g_vi];
        if (g_vi == 2) { AFE_SetAttenuation(AFE_CH1, 1); AFE_SetAttenuation(AFE_CH2, 1); }
        else { AFE_SetAttenuation(AFE_CH1, 0); AFE_SetAttenuation(AFE_CH2, 0); }
        break;
    case MODE_COUPLING:  g_coupling = (g_coupling == COUPLING_DC) ? COUPLING_AC : COUPLING_DC;
                         AFE_SetCoupling(AFE_CH1, (g_coupling == COUPLING_AC) ? AFE_COUPLING_AC : AFE_COUPLING_DC);
                         AFE_SetCoupling(AFE_CH2, (g_coupling == COUPLING_AC) ? AFE_COUPLING_AC : AFE_COUPLING_DC);
                         break;
    case MODE_VERT_OFFSET: { int32_t step = (int32_t)(g_v_div * 250.0f); if (step < 1) step = 1; g_vert_offset_mv += dir * step; if (g_vert_offset_mv > 3000) g_vert_offset_mv = 3000; if (g_vert_offset_mv < -3000) g_vert_offset_mv = -3000; } break;
    case MODE_CHANNEL:     g_ch_sel = (ChannelSelect_t)((g_ch_sel % 3) + 1); break;
    case MODE_RUN_MODE:    g_run = (RunMode_t)((g_run + dir + 3) % 3); if (g_run == RUN_SINGLE) g_cv = 0; break;
    }
}

static void wave_cap(void) {
    uint32_t now = HAL_GetTick();
    int32_t dm = (int32_t)(now - g_lw);
    if (dm < 1) return;

    g_wu = (uint32_t)(g_s_div * 1000000.0f * 10.0f / (float)SP);
    if (g_wu < 1) g_wu = 1;
    uint32_t ns = ((uint32_t)dm * 1000) / g_wu;
    if (ns > SP) ns = SP;

    /* ── G474 DMA Channel 模型：CNDTR 寄存器 ── */
    static uint32_t ldma1 = 0, ldma2 = 0, ldma3 = 0;
    uint32_t idx1 = (ADC_BUF_SIZE - (DMA1_Channel1->CNDTR)) % ADC_BUF_SIZE;
    uint32_t idx2 = (ADC_BUF_SIZE - (DMA1_Channel2->CNDTR)) % ADC_BUF_SIZE;
    uint32_t idx3 = (ADC_BUF_SIZE - (DMA1_Channel3->CNDTR)) % ADC_BUF_SIZE;
    uint32_t step1 = ns > 1 ? ((idx1 - ldma1 + ADC_BUF_SIZE) % ADC_BUF_SIZE) / ns : 1;
    uint32_t step2 = ns > 1 ? ((idx2 - ldma2 + ADC_BUF_SIZE) % ADC_BUF_SIZE) / ns : 1;
    uint32_t step3 = ns > 1 ? ((idx3 - ldma3 + ADC_BUF_SIZE) % ADC_BUF_SIZE) / ns : 1;
    if (step1 == 0) step1 = 1;
    if (step2 == 0) step2 = 1;
    if (step3 == 0) step3 = 1;

    /* LPF 系数缓存：仅在采样率变化时重新计算 expf */
    static float last_fs1 = -1.0f, last_fs2 = -1.0f;
    static float alpha1_c = 0.1f, alpha2_c = 0.1f;
    static float g_lpf1 = 0.0f, g_lpf2 = 0.0f;
    float fs1 = g_sps1 > 0.0f ? g_sps1 : 1000000.0f;
    float fs2 = g_sps2 > 0.0f ? g_sps2 : 1000000.0f;
    if (fs1 != last_fs1) {
        alpha1_c = 1.0f - expf(-6.283185307f * 1800000.0f / fs1);
        if (alpha1_c > 1.0f) alpha1_c = 1.0f; else if (alpha1_c < 0.0f) alpha1_c = 0.0f;
        last_fs1 = fs1;
    }
    if (fs2 != last_fs2) {
        alpha2_c = 1.0f - expf(-6.283185307f * 1800000.0f / fs2);
        if (alpha2_c > 1.0f) alpha2_c = 1.0f; else if (alpha2_c < 0.0f) alpha2_c = 0.0f;
        last_fs2 = fs2;
    }

    /* 预提取常数，减少循环内计算 */
    float lpf1 = g_lpf1, lpf2 = g_lpf2;
    float oma1 = 1.0f - alpha1_c, oma2 = 1.0f - alpha2_c;
    uint32_t hi = g_hi;

    for (uint32_t i = 0; i < ns; i++) {
        uint32_t p1 = (ldma1 + i * step1) % ADC_BUF_SIZE;
        uint32_t p2 = (ldma2 + i * step2) % ADC_BUF_SIZE;
        uint32_t hi_pos = hi % HS;

        /* CH1: 硬件差分 + LPF */
        lpf1 = alpha1_c * (float)(int16_t)adc1_buf[p1] + oma1 * lpf1;
        g_h1[hi_pos] = (int16_t)lpf1;

        /* CH2: 硬件差分，直接读取 */
        lpf2 = alpha2_c * (float)(int16_t)adc2_buf[p2] + oma2 * lpf2;
        g_h2[hi_pos] = (int16_t)lpf2;
        hi++;
    }

    g_hi   = hi;
    g_lpf1 = lpf1;
    g_lpf2 = lpf2;
    ldma1  = idx1;
    ldma2  = idx2;
    ldma3  = idx3;
    g_lw   = now;

    /* 单次触发：仅在整块数据更新后查找一次（修复原循环内重复扫描的严重Bug） */
    if (g_run == RUN_SINGLE && !g_cv && ns > 0) {
        int tr = find_trig(g_h1, HS, g_trg, 150);
        if (tr >= 0) {
            for (int i = 0; i < 240; i++) {
                int pos = (tr + i) % HS;
                g_cap[i * 2]     = g_h1[pos];
                g_cap[i * 2 + 1] = g_h2[pos];
            }
            g_cv = 1;
        }
    }
}

/* ── 网格（一次性绘制，V/div 变化时重绘） ── */
static int g_grid_drawn = 0;
static float g_last_grid_v = -1.0f;
static void draw_grid(void) {
    if (g_grid_drawn && g_v_div == g_last_grid_v) return;
    g_last_grid_v = g_v_div;
    LCD_FillRect(SX, SY, SX + SW, SY + SH, LCD_BLACK);

    /* 水平网格线 — FillRect 短横线 (3px宽) 代替逐点 DrawPixel */
    for (int d = 0; d <= 8; d++) {
        int y = SY + d * DV; if (y > SY + SH) y = SY + SH;
        uint16_t clr = (d == 4) ? 0xC618 : 0x2104;
        if (d == 4) {
            for (int dy = -1; dy <= 1; dy++) {
                int yy = y + dy; if (yy < SY || yy > SY + SH) continue;
                LCD_FillRect(SX, yy, SX + SW - 1, yy, clr);  /* 实线用FillRect */
            }
        } else {
            for (int x = SX; x < SX + SW; x += 12)
                LCD_FillRect(x, y, x + 2, y, clr);  /* 短横线 */
        }
        char lb[8]; int val_mv = (int)((4 - d) * g_v_div * 1000.0f);
        snprintf(lb, 8, "%+dmV", val_mv);
        LCD_DrawString(SX + 2, y - 6, lb, 0xC618, LCD_BLACK);
    }

    /* 垂直网格线 */
    for (int d = 0; d <= 10; d++) {
        int x = SX + d * DH; if (x >= SX + SW) x = SX + SW - 1;
        uint16_t clr = (d == 5) ? 0xC618 : 0x2104;
        if (d == 5) {
            for (int dx = -1; dx <= 1; dx++) {
                int xx = x + dx; if (xx < SX || xx >= SX + SW) continue;
                LCD_FillRect(xx, SY, xx, SY + SH - 1, clr);  /* 实线用FillRect */
            }
        } else {
            for (int y = SY; y < SY + SH; y += 12)
                LCD_FillRect(x, y, x, y + 2, clr);  /* 短竖线 */
        }
    }
    g_grid_drawn = 1;
}

static int v2y(float mv) { int c = SY+4*DV; float mpx = (g_v_div*1000.0f)/(float)DV; int y = c-(int)((mv + (float)g_vert_offset_mv)/mpx); if (y<SY)y=SY; if (y>SY+SH)y=SY+SH; return y; }

/* ── 波形绘制（Bresenham + Y-change 稀疏化） ── */
static void draw_wf(const int16_t *h, uint32_t st, uint32_t cnt, uint16_t clr) {
    if (cnt < 2) return;
    int px = SX;
    float mv0 = ((float)(int32_t)h[st % HS] / (float)ADC_MAX) * VREF * 1000.0f;
    int py = v2y(mv0);
    LCD_DrawPixel((uint16_t)px, (uint16_t)py, clr);
    for (uint32_t i = 1; i < cnt; i++) {
        float mv1 = ((float)(int32_t)h[(st + i) % HS] / (float)ADC_MAX) * VREF * 1000.0f;
        int cx = SX + (int)i, cy = v2y(mv1);
        int dy = cy - py, dx = cx - px;
        int s = (dy < 0 ? -dy : dy) > dx ? (dy < 0 ? -dy : dy) : dx;
        if (s < 1) s = 1;
        int last_y = -999;
        for (int k = 0; k <= s; k += 2) {
            int x = px + (dx * k) / s, y = py + (dy * k) / s;
            if (x >= SX && x < SX + SW && y >= SY && y < SY + SH) {
                if (y != last_y) {  /* 跳过 Y 无变化点 */
                    LCD_DrawPixel((uint16_t)x, (uint16_t)y, clr);
                    last_y = y;
                }
            }
        }
        px = cx; py = cy;
    }
}

/* ── 计算测量值（频率、Vpp） ── */
static void calc_meas(const int16_t *h, float *freq, float *ampl, float *vmin, float *vmax) {
    uint32_t n_meas = g_hi;
    int32_t minv = 32767, maxv = -32767; int zc = 0, lz = -1;
    for (uint32_t i = n_meas > 240 ? n_meas - 240 : 0; i < n_meas; i++) {
        int16_t v = h[i % HS];
        if (v < minv) minv = v; if (v > maxv) maxv = v;
        if (i > (n_meas > 240 ? n_meas - 240 : 0) && v >= 0 && h[(uint32_t)(i - 1) % HS] < 0) {
            if (lz > 0) { int p = (int)i - lz; if (p > 2) zc++; } lz = (int)i;
        }
    }
    *freq = 0; *ampl = 0;
    *vmin = ((float)minv / ADC_MAX) * VREF * 1000.0f;
    *vmax = ((float)maxv / ADC_MAX) * VREF * 1000.0f;
    if (zc > 1 && g_wu > 0) {
        float ap = (float)(n_meas - (n_meas > 240 ? n_meas - 240 : 0)) / (float)(zc);
        *freq = 1000000.0f / (ap * (float)g_wu);
    }
    *ampl = (float)(maxv - minv) / ADC_MAX * VREF;
}

/* ── 面板信息（每 4 帧刷新一次） ── */
static void draw_meas(void) {
    g_meas_frame++;
    if (g_meas_frame % 4 != 0) return;

    int x = IX, y = 4; char b[32];
    float f1, a1, f2, a2, vmin1, vmax1, vmin2, vmax2;

    LCD_FillRect(x + 1, 0, 319, SH, 0x0841);
    for (int iy = 0; iy < SH; iy += 8) LCD_DrawPixel(x, iy, 0x4208);

    LCD_DrawString(x + 4, y, "--- SCOPE ---", LCD_WHITE, 0x0841); y += 12;
    snprintf(b, 32, "Ch:%s", g_ch_sel == CH_SEL_CH1 ? "CH1" : g_ch_sel == CH_SEL_CH2 ? "CH2" : "BOTH");
    LCD_DrawString(x + 4, y, b, (g_menu == MODE_CHANNEL) ? LCD_YELLOW : LCD_CYAN, 0x0841); y += 12;
    snprintf(b, 32, "V:%s%s", g_vi == 0 ? "0.01" : g_vi == 1 ? "0.1" : "1.0", g_vi == 2 ? " ATT" : "");
    LCD_DrawString(x + 4, y, b, (g_menu == MODE_VOLT_DIV) ? LCD_YELLOW : LCD_CYAN, 0x0841); y += 12;
    snprintf(b, 32, "T:%s", s_lbl[g_si]);
    LCD_DrawString(x + 4, y, b, (g_menu == MODE_TIME_BASE) ? LCD_YELLOW : LCD_CYAN, 0x0841); y += 12;
    snprintf(b, 32, "Trg:%ld mV", (long)(((float)g_trg / (float)ADC_MAX) * VREF * 1000.0f));
    LCD_DrawString(x + 4, y, b, (g_menu == MODE_TRG_LEVEL) ? LCD_YELLOW : LCD_CYAN, 0x0841); y += 12;
    snprintf(b, 32, "Cpl:%s", g_coupling == COUPLING_DC ? "DC" : "AC");
    LCD_DrawString(x + 4, y, b, (g_menu == MODE_COUPLING) ? LCD_YELLOW : LCD_CYAN, 0x0841); y += 12;
    snprintf(b, 32, "Ofs:%+ldmV", (long)g_vert_offset_mv);
    LCD_DrawString(x + 4, y, b, (g_menu == MODE_VERT_OFFSET) ? LCD_YELLOW : LCD_CYAN, 0x0841); y += 12;
    snprintf(b, 32, "Run:%s", g_run == RUN_AUTO ? "AUTO" : g_run == RUN_NORMAL ? "NORM" : "SINGLE");
    LCD_DrawString(x + 4, y, b, (g_menu == MODE_RUN_MODE) ? LCD_YELLOW : LCD_CYAN, 0x0841); y += 12;
    snprintf(b, 32, "FPS:%5.1f", (double)g_fps);
    LCD_DrawString(x + 4, y, b, LCD_YELLOW, 0x0841); y += 14;

    calc_meas(g_h1, &f1, &a1, &vmin1, &vmax1);
    calc_meas(g_h2, &f2, &a2, &vmin2, &vmax2);

    if (g_ch_sel == CH_SEL_CH1 || g_ch_sel == CH_SEL_BOTH) {
        LCD_DrawString(x + 4, y, "-- CH1(MEAS) --", LCD_YELLOW, 0x0841); y += 12;
        snprintf(b, 32, "F:%6ldHz", (long)f1); LCD_DrawString(x + 4, y, b, LCD_GREEN, 0x0841); y += 12;
        snprintf(b, 32, "Vpp:%7.2fmV", (double)(a1 * 1000.0f)); LCD_DrawString(x + 4, y, b, LCD_GREEN, 0x0841); y += 12;
        snprintf(b, 32, "Min:%6.1f Max:%6.1fmV", (double)vmin1, (double)vmax1); LCD_DrawString(x + 4, y, b, 0x8410, 0x0841); y += 14;
    }
    if (g_ch_sel == CH_SEL_CH2 || g_ch_sel == CH_SEL_BOTH) {
        LCD_DrawString(x + 4, y, "-- CH2(MEAS) --", LCD_CYAN, 0x0841); y += 12;
        snprintf(b, 32, "F:%6ldHz", (long)f2); LCD_DrawString(x + 4, y, b, LCD_GREEN, 0x0841); y += 12;
        snprintf(b, 32, "Vpp:%7.2fmV", (double)(a2 * 1000.0f)); LCD_DrawString(x + 4, y, b, LCD_GREEN, 0x0841); y += 12;
        snprintf(b, 32, "Min:%6.1f Max:%6.1fmV", (double)vmin2, (double)vmax2); LCD_DrawString(x + 4, y, b, 0x8410, 0x0841); y += 14;
    }

    LCD_DrawString(x + 4, y, "-- SPECTRUM --", LCD_WHITE, 0x0841); y += 12;
    for (int h = 1; h <= 5; h++) {
        snprintf(b, 32, "H%d:%.2fV", h, (double)(a1 / (float)h));
        LCD_DrawString(x + 4, y, b, h == 1 ? LCD_GREEN : 0x8410, 0x0841); y += 12;
    }
    y += 6;
    LCD_DrawString(x + 4, y, "Zin >100k", 0x8410, 0x0841); y += 12;
    LCD_DrawString(x + 4, y, "BW DC-200k", 0x8410, 0x0841);
}

/* ── 触摸菜单 API：供 Scope 任务调用 ── */
EncoderMode_t ADC_GetMenu(void) { return g_menu; }
void ADC_SetMenu(EncoderMode_t m) { g_menu = m; }
void ADC_StepMenuUp(void) {
    switch (g_menu) {
    case MODE_CHANNEL:     g_ch_sel = (ChannelSelect_t)((g_ch_sel % 3) + 1); break;
    case MODE_VOLT_DIV:    g_vi = (g_vi + 1) % 3; g_v_div = v_tbl[g_vi];
                           if (g_vi == 2) { AFE_SetAttenuation(AFE_CH1,1); AFE_SetAttenuation(AFE_CH2,1); }
                           else { AFE_SetAttenuation(AFE_CH1,0); AFE_SetAttenuation(AFE_CH2,0); } break;
    case MODE_TIME_BASE:   g_si = (g_si + 1) % 3; g_s_div = s_tbl[g_si];
                           g_wu = (uint32_t)(g_s_div * 1000000 * 10 / SP); break;
    case MODE_TRG_LEVEL:   g_trg += 200; if (g_trg > 32767) g_trg = 32767; break;
    case MODE_COUPLING:    g_coupling = (g_coupling == COUPLING_DC) ? COUPLING_AC : COUPLING_DC;
                           AFE_SetCoupling(AFE_CH1, (g_coupling==COUPLING_AC)?AFE_COUPLING_AC:AFE_COUPLING_DC);
                           AFE_SetCoupling(AFE_CH2, (g_coupling==COUPLING_AC)?AFE_COUPLING_AC:AFE_COUPLING_DC); break;
    case MODE_VERT_OFFSET: { int32_t step = (int32_t)(g_v_div * 250.0f); if (step<1) step=1;
                             g_vert_offset_mv += step; if (g_vert_offset_mv>3000) g_vert_offset_mv=3000; } break;
    case MODE_RUN_MODE:    g_run = (RunMode_t)((g_run + 1) % 3); if (g_run==RUN_SINGLE) g_cv=0; break;
    }
}
void ADC_StepMenuDown(void) {
    switch (g_menu) {
    case MODE_CHANNEL:     g_ch_sel = (ChannelSelect_t)(((g_ch_sel - 1 + 3) % 3) == 0 ? 3 : ((g_ch_sel - 1 + 3) % 3)); if (g_ch_sel==0) g_ch_sel=CH_SEL_BOTH; break;
    case MODE_VOLT_DIV:    g_vi = (g_vi - 1 + 3) % 3; g_v_div = v_tbl[g_vi];
                           if (g_vi == 2) { AFE_SetAttenuation(AFE_CH1,1); AFE_SetAttenuation(AFE_CH2,1); }
                           else { AFE_SetAttenuation(AFE_CH1,0); AFE_SetAttenuation(AFE_CH2,0); } break;
    case MODE_TIME_BASE:   g_si = (g_si - 1 + 3) % 3; g_s_div = s_tbl[g_si];
                           g_wu = (uint32_t)(g_s_div * 1000000 * 10 / SP); break;
    case MODE_TRG_LEVEL:   g_trg -= 200; if (g_trg < -32767) g_trg = -32767; break;
    case MODE_COUPLING:    g_coupling = (g_coupling == COUPLING_DC) ? COUPLING_AC : COUPLING_DC;
                           AFE_SetCoupling(AFE_CH1, (g_coupling==COUPLING_AC)?AFE_COUPLING_AC:AFE_COUPLING_DC);
                           AFE_SetCoupling(AFE_CH2, (g_coupling==COUPLING_AC)?AFE_COUPLING_AC:AFE_COUPLING_DC); break;
    case MODE_VERT_OFFSET: { int32_t step = (int32_t)(g_v_div * 250.0f); if (step<1) step=1;
                             g_vert_offset_mv -= step; if (g_vert_offset_mv<-3000) g_vert_offset_mv=-3000; } break;
    case MODE_RUN_MODE:    g_run = (RunMode_t)((g_run - 1 + 3) % 3); if (g_run==RUN_SINGLE) g_cv=0; break;
    }
}
int32_t ADC_GetTrg(void) { return g_trg; }
void ADC_ForceRedraw(void) { g_grid_drawn = 0; g_meas_frame = 0; }

void ADC_DisplayOnLCD(void) {
    btn_proc(); enc_proc(); wave_cap();
    draw_grid();

    if (g_run == RUN_SINGLE && g_cv) {
        for (int i = 0; i < 240; i++) {
            float mv1 = ((float)(int32_t)g_cap[i * 2] / (float)ADC_MAX) * VREF * 1000.0f;
            float mv2 = ((float)(int32_t)g_cap[i * 2 + 1] / (float)ADC_MAX) * VREF * 1000.0f;
            int y1 = v2y(mv1), y2 = v2y(mv2);
            if (y1 >= SY && y1 < SY + SH) LCD_DrawPixel(SX + i, (uint16_t)y1, LCD_YELLOW);
            if (y2 >= SY && y2 < SY + SH) LCD_DrawPixel(SX + i, (uint16_t)y2, LCD_CYAN);
        }
    } else {
        uint32_t t = g_hi, dc = t < SP ? t : SP, si = t >= SP ? t - SP : 0;
        static int g_skip_cnt = 0;
        if (g_si != 2 || g_skip_cnt == 0) {
            LCD_FillRect(SX, SY, SX + SW, SY + SH, LCD_BLACK);
            for (int d = 0; d <= 8; d++) {
                int y = SY + d * DV; if (y > SY + SH) y = SY + SH;
                uint16_t glr = (d == 4) ? 0x630C : 0x2104;
                for (int x = SX; x < SX + SW; x += 12) LCD_FillRect(x, y, x + 2, y, glr);
                char lb[8]; int val_mv = (int)((4 - d) * g_v_div * 1000.0f);
                snprintf(lb, 8, "%+dmV", val_mv); LCD_DrawString(SX + 2, y - 6, lb, 0x8410, LCD_BLACK);
            }
            for (int d = 0; d <= 10; d++) {
                int x = SX + d * DH; if (x >= SX + SW) x = SX + SW - 1;
                uint16_t glr = (d == 5) ? 0x630C : 0x2104;
                for (int y = SY; y < SY + SH; y += 12) LCD_FillRect(x, y, x, y + 2, glr);
            }
            if (g_ch_sel == CH_SEL_CH1 || g_ch_sel == CH_SEL_BOTH) draw_wf(g_h1, si, dc, LCD_YELLOW);
            if (g_ch_sel == CH_SEL_CH2 || g_ch_sel == CH_SEL_BOTH) draw_wf(g_h2, si, dc, LCD_CYAN);
        }
        g_skip_cnt = (g_skip_cnt + 1) % 3;
    }
    draw_meas();

    /* ════════════ 强行置顶：在所有图层之上绘制触摸按钮 ════════════ */
    {
        uint16_t lw = LCD_GetWidth();
        int btn_w = 100;
        int btn_h = 40;
        int btn_gap = 10;
        int btn_x = (int)lw - btn_w - 2;  /* 右侧 */
        int btn_y_top = 160;
        LCD_FillRect(btn_x, btn_y_top, btn_x + btn_w, btn_y_top + btn_h, 0x4208);
        LCD_DrawString(btn_x + 14, btn_y_top + 12, "MENU", LCD_YELLOW, 0x4208);
        LCD_FillRect(btn_x, btn_y_top + btn_h + btn_gap, btn_x + btn_w, btn_y_top + btn_h * 2 + btn_gap, 0x0020);
        LCD_DrawString(btn_x + 36, btn_y_top + btn_h + btn_gap + 12, "+", LCD_GREEN, 0x0020);
        LCD_FillRect(btn_x, btn_y_top + (btn_h + btn_gap) * 2, btn_x + btn_w, btn_y_top + btn_h * 3 + btn_gap * 2, 0x4000);
        LCD_DrawString(btn_x + 36, btn_y_top + (btn_h + btn_gap) * 2 + 12, "-", LCD_WHITE, 0x4000);
        /* X 按钮在右上角 */
        int x_w = 60, x_h = 36;
        LCD_FillRect((int)lw - x_w - 2, 2, (int)lw - 2, 2 + x_h, 0xC000);
        LCD_DrawString((int)lw - x_w + 8, 10, "X", LCD_WHITE, 0xC000);
    }

    g_fc++;
    uint32_t now = HAL_GetTick();
    if (now - g_ft >= 1000) { g_fps = (float)g_fc * 1000.0f / (float)(now - g_ft); g_fc = 0; g_ft = now; }
}

void ADC_PrintRealtime(void) {
    char buf[512]; float k1 = g_sps1 / 1000.0f, k2 = g_sps2 / 1000.0f;
    int len = snprintf(buf, 512, "CH1=%.1f kSPS CH2=%.1f kSPS\r\n", (double)k1, (double)k2);
    len += snprintf(buf + len, 512 - len, "CH1:"); int st = ADC_BUF_SIZE > 4 ? ADC_BUF_SIZE - 4 : 0;
    for (int i = 0; i < 4 && len < 480; i++) {
        int32_t raw = (int32_t)(int16_t)adc1_buf[(st + i) % ADC_BUF_SIZE];
        float mv = ((float)raw / (float)ADC_MAX) * VREF * 1000.0f;
        len += snprintf(buf + len, 512 - len, "[%6ld %7.1fmV] ", (long)raw, (double)mv);
    }
    len += snprintf(buf + len, 512 - len, "\r\nCH2:");
    for (int i = 0; i < 4 && len < 480; i++) {
        int32_t raw = -(int32_t)(int16_t)adc2_buf[(st + i) % ADC_BUF_SIZE]; /* IN18_N - IN18_P, 取反 */
        float mv = ((float)raw / (float)ADC_MAX) * VREF * 1000.0f;
        len += snprintf(buf + len, 512 - len, "[%6ld %7.1fmV] ", (long)raw, (double)mv);
    }
    snprintf(buf + len, 512 - len, "\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 500);
}

uint8_t ADC_GetBufComplete(void) { return (uint8_t)(g_buf1_cnt + g_buf2_cnt); }

/* HAL_ADC_ConvHalfCpltCallback / HAL_ADC_ConvCpltCallback 已移至 stm32g4xx_it.c (高速 Ping-Pong 回调) */
void ADC_Driver_DMA_IRQHandler(void) { /* no longer used; ISRs in stm32g4xx_it.c */ }
