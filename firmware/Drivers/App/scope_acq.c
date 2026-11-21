/**
  ******************************************************************************
  * @file    scope_acq.c
  * @brief   单次定长采集实现（单路 / 三路交替）
  ******************************************************************************
  */
#include "scope_acq.h"
#include "sample_clock.h"
#include "adc.h"
#include "tim.h"
#include "dma.h"
#include "../debug_print/debug_print.h"
#include <stdio.h>

/* ADC 的 DMA 句柄定义在 Core/Src/adc.c 中（adc.h 未导出） */
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc5;

/* ── 三路样本缓冲: 3 x 8192 x 2B = 48 KB ── */
static uint16_t s_buf[3][SCOPE_MAX_M];
/* INTER2: CDR 每个 32 位字含两个样本（低半=ADC1 master, 高半=ADC2 slave）
 * 直接复用 s_buf[0]+s_buf[1] 的 32KB，不额外占内存 */
#define S_BUF32        ((uint32_t *)(void *)&s_buf[0][0])
#define SCOPE_INTER2_MAX_N  (2u * SCOPE_MAX_M)

volatile uint8_t  g_scope_done   = 0;
volatile uint32_t g_scope_n      = 0;
volatile uint8_t  g_scope_mode   = SCOPE_MODE_SINGLE;
volatile uint8_t  g_scope_sel_ch = SCOPE_CH1;
volatile uint8_t  g_scope_mask   = 0;
volatile uint32_t g_scope_cyc    = 0;   /* 本次采集耗时(DWT 周期, 170MHz) */

/* 调试开关：1 = 单端模式（输出引脚绝对电压 0..4095，不做符号扩展） */
volatile uint8_t  g_scope_single_ended = 0;

static volatile uint8_t s_armed = 0;
static uint32_t         s_t0_cyc = 0;

/* ── 通道号 → HAL 句柄 ── */
static ADC_HandleTypeDef *adc_of(uint8_t ch)
{
    switch (ch) {
    case SCOPE_CH1: return &hadc1;
    case SCOPE_CH2: return &hadc2;
    case SCOPE_CH5: return &hadc5;
    default:        return NULL;
    }
}

static DMA_HandleTypeDef *dma_of(uint8_t ch)
{
    switch (ch) {
    case SCOPE_CH1: return &hdma_adc1;
    case SCOPE_CH2: return &hdma_adc2;
    case SCOPE_CH5: return &hdma_adc5;
    default:        return NULL;
    }
}

/* ── 把某一路 DMA 重新配成 NORMAL 模式 ── */
static int dma_prepare(uint8_t ch)
{
    DMA_HandleTypeDef *d = dma_of(ch);
    ADC_HandleTypeDef *h = adc_of(ch);

    if (d == NULL || h == NULL) return -1;

    (void)HAL_ADC_Stop_DMA(h);

    d->Init.Mode = DMA_NORMAL;
    /* 必须显式复位回 16 位：INTER2 曾把 ADC1 的 DMA 改成 WORD，
       不还原的话单路采集会把 32 位字当两个样本读，频谱全乱。 */
    d->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    d->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    (void)HAL_DMA_DeInit(d);
    if (HAL_DMA_Init(d) != HAL_OK) return -2;

    __HAL_LINKDMA(h, DMA_Handle, *d);
    return 0;
}

/* ── INTER2 用: ADC1 的 DMA 必须按 32 位字搬 CDR ── */
static int dma_prepare_u32(void)
{
    DMA_HandleTypeDef *d = &hdma_adc1;
    ADC_HandleTypeDef *h = &hadc1;

    (void)HAL_ADC_Stop_DMA(h);

    d->Init.Mode                 = DMA_NORMAL;
    d->Init.PeriphDataAlignment  = DMA_PDATAALIGN_WORD;
    d->Init.MemDataAlignment     = DMA_MDATAALIGN_WORD;
    (void)HAL_DMA_DeInit(d);
    if (HAL_DMA_Init(d) != HAL_OK) return -2;

    __HAL_LINKDMA(h, DMA_Handle, *d);
    return 0;
}

/* ── 停掉所有 ADC 与 DMA ── */
static void adc_stop_all(void)
{
    uint8_t i;
    for (i = 0; i < 3u; i++) {
        ADC_HandleTypeDef *h = adc_of(i);
        if (h != NULL) (void)HAL_ADC_Stop_DMA(h);
    }
}

/* ═══════════════════════════════════════════════════════════════ */

void ScopeAcq_Init(void)
{
    g_scope_done   = 0;
    g_scope_n      = 0;
    g_scope_mode   = SCOPE_MODE_SINGLE;
    g_scope_sel_ch = SCOPE_CH1;
    g_scope_mask   = 0;
    s_armed        = 0;

    /* DWT 周期计数器: 用来实测 ADC 的真实采样率 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    g_scope_cyc = 0U;

    adc_stop_all();
    SampleClock_StopCapture();

    Debug_Print("[ACQ] single-shot capture ready (single + 3ch interleave)\r\n");
}

int ScopeAcq_Start(uint8_t mode, uint8_t ch, uint16_t n_total)
{
    uint32_t m;
    uint8_t  i;

    if (s_armed) return -3;

    if (mode == SCOPE_MODE_INTER2) {
        if (ch != SCOPE_CH1) return -7;
        if ((n_total & 1u) != 0u) return -6;
        if (n_total < 16u || (uint32_t)n_total > SCOPE_INTER2_MAX_N) return -2;
        m = (uint32_t)n_total / 2u;          /* DMA 字数 = 样本数/2 */
    } else if (mode == SCOPE_MODE_INTER3) {
        if ((n_total % 3u) != 0u) return -6;
        m = (uint32_t)n_total / 3u;
        if (m == 0UL || m > SCOPE_MAX_M) return -2;
    } else {
        mode = SCOPE_MODE_SINGLE;
        if (ch >= 3u) return -1;
        m = (uint32_t)n_total;
        /* 单路只用 s_buf[0]，但 s_buf 是 3x8192 的连续内存，
           所以单路最多可以采 3*SCOPE_MAX_M 点（提高频率分辨率用）。 */
        if (m < 16UL || m > (3UL * SCOPE_MAX_M)) return -2;
    }

    /* 1) 先停触发, 保证第一个触发一定落在 UPDATE（相位 0） */
    SampleClock_StopCapture();
    adc_stop_all();

    /* 2) 把要用的几路 DMA 重新配成 NORMAL */
    if (mode == SCOPE_MODE_INTER2) {
        /* 顺序很重要: dma_prepare_u32 里的 HAL_ADC_Stop_DMA 会把 ADEN 清掉,
           双 ADC 模式寄存器(ADC12_CCR)只有 ADEN=0 时才允许改写。 */
        if (dma_prepare_u32() != 0) return -4;
        if (ADC_DualInterleavedConfig(1u) != 0) return -5;
    } else {
        ADC_DualInterleavedConfig(0u);
        if (mode == SCOPE_MODE_INTER3) {
            for (i = 0; i < 3u; i++) {
                if (dma_prepare(i) != 0) return -4;
            }
        } else {
            if (dma_prepare(ch) != 0) return -4;
        }
    }

    /* 3) 记录本次参数 */
    g_scope_mode   = mode;
    g_scope_sel_ch = ch;
    if (mode == SCOPE_MODE_INTER2)      g_scope_n = m * 2UL;
    else if (mode == SCOPE_MODE_INTER3) g_scope_n = m * 3UL;
    else                                g_scope_n = m;
    g_scope_mask   = 0;
    g_scope_done   = 0;
    s_armed        = 1;

    /* 4) arm ADC（此时计数器停着, 不会有触发丢失） */
    if (mode == SCOPE_MODE_INTER2) {
        if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)S_BUF32, m) != HAL_OK) {
            s_armed = 0;
            adc_stop_all();
            return -5;
        }
    } else if (mode == SCOPE_MODE_INTER3) {
        for (i = 0; i < 3u; i++) {
            if (HAL_ADC_Start_DMA(adc_of(i), (uint32_t *)s_buf[i], m) != HAL_OK) {
                s_armed = 0;
                adc_stop_all();
                return -5;
            }
        }
    } else {
        if (HAL_ADC_Start_DMA(adc_of(ch), (uint32_t *)s_buf[ch], m) != HAL_OK) {
            s_armed = 0;
            return -5;
        }
    }

    /* 5) 开闸: CNT=ARR, 下一拍溢出 -> UPDATE -> ADC1 */
    s_t0_cyc = DWT->CYCCNT;
    SampleClock_StartCapture();

    return 0;
}

void ScopeAcq_OnCplt(uint8_t ch)
{
    if (!s_armed)   return;
    if (ch > 2u)    return;

    g_scope_mask |= (uint8_t)(1U << ch);

    if (g_scope_mode == SCOPE_MODE_INTER2) {
        if (ch == SCOPE_CH1) {          /* 多模式下只有 master(ADC1) 进中断 */
            s_armed      = 0;
            g_scope_cyc  = DWT->CYCCNT - s_t0_cyc;
            g_scope_done = 1;
        }
    } else if (g_scope_mode == SCOPE_MODE_INTER3) {
        if (g_scope_mask == 0x07u) {
            s_armed      = 0;
            g_scope_cyc  = DWT->CYCCNT - s_t0_cyc;
            g_scope_done = 1;
        }
    } else {
        if (ch == g_scope_sel_ch) {
            s_armed      = 0;
            g_scope_cyc  = DWT->CYCCNT - s_t0_cyc;
            g_scope_done = 1;
        }
    }
}

uint8_t ScopeAcq_Done(void)
{
    return g_scope_done;
}

uint32_t ScopeAcq_Total(void)
{
    return g_scope_n;
}

void ScopeAcq_Abort(void)
{
    s_armed = 0;
    SampleClock_StopCapture();
    adc_stop_all();
}

/* 差分数值修正: G4 差分模式 12 位补码只占 DR[11:0]，高 4 位为 0，
   若直接当 uint16/int16 用会把 -1(0x0FFF) 错读成 +4095。
   这里统一按低 12 位补码做符号扩展；对硬件已做符号扩展的情况同样正确。 */
static inline uint16_t diff_fix(uint16_t w)
{
    /* 单端调试模式：不做换算，原样给出 0..4095 的引脚电压码 */
    if (g_scope_single_ended != 0u) return (uint16_t)(w & 0x0FFFu);

    /* 差分模式：G4 规则组差分结果的 12 bit 是「偏移二进制」，
       2048 = 差分零点，4095 = +满量程，0 = -满量程。
       实测（raw 0B1F/0C4B/0C99 连续单调）证实不能按补码符号扩展，
       否则 2048 以上会被翻到 -2048，出现"满幅乱跳"。 */
    return (uint16_t)(int16_t)((int32_t)(w & 0x0FFFu) - 2048);
}

uint32_t ScopeAcq_Fill(uint32_t start, uint16_t *dst, uint32_t max_pts)
{
    uint32_t n = g_scope_n;
    uint32_t j;

    if (dst == NULL || start >= n) return 0UL;
    if (max_pts > (n - start)) max_pts = n - start;

    if (g_scope_mode == SCOPE_MODE_INTER2) {
        uint32_t k    = start >> 1;
        uint32_t half = start & 1UL;

        for (j = 0; j < max_pts; j++) {
            uint32_t w = S_BUF32[k];
            dst[j] = diff_fix((uint16_t)((half != 0UL) ? (w >> 16) : (w & 0xFFFFu)));
            if (++half >= 2UL) { half = 0UL; k++; }
        }
    } else if (g_scope_mode == SCOPE_MODE_INTER3) {
        uint32_t p = start % 3UL;
        uint32_t k = start / 3UL;

        for (j = 0; j < max_pts; j++) {
            dst[j] = diff_fix(s_buf[p][k]);
            if (++p >= 3UL) { p = 0UL; k++; }
        }
    } else {
        const uint16_t *src = s_buf[g_scope_sel_ch];
        for (j = 0; j < max_pts; j++) {
            dst[j] = diff_fix(src[start + j]);
        }
    }
    return max_pts;
}

/* 调试用：直接给出 DMA 原始字（不做任何换算），用来核对 ADC 的数据格式 */
uint32_t ScopeAcq_RawPeek(uint32_t start, uint16_t *dst, uint32_t max_pts)
{
    uint32_t n = g_scope_n;
    uint32_t j;

    if (dst == NULL || start >= n) return 0UL;
    if (max_pts > (n - start)) max_pts = n - start;

    if (g_scope_mode == SCOPE_MODE_INTER2) {
        uint32_t k    = start >> 1;
        uint32_t half = start & 1UL;

        for (j = 0; j < max_pts; j++) {
            uint32_t w = S_BUF32[k];
            dst[j] = (uint16_t)((half != 0UL) ? (w >> 16) : (w & 0xFFFFu));
            if (++half >= 2UL) { half = 0UL; k++; }
        }
    } else if (g_scope_mode == SCOPE_MODE_INTER3) {
        uint32_t p = start % 3UL;
        uint32_t k = start / 3UL;

        for (j = 0; j < max_pts; j++) {
            dst[j] = s_buf[p][k];
            if (++p >= 3UL) { p = 0UL; k++; }
        }
    } else {
        const uint16_t *src = s_buf[g_scope_sel_ch];
        for (j = 0; j < max_pts; j++) dst[j] = src[start + j];
    }
    return max_pts;
}

uint8_t ScopeAcq_SelfTest(void)
{
    uint32_t t0;

    if (s_armed) return 0;

    g_scope_done = 0;
    if (ScopeAcq_Start(SCOPE_MODE_INTER3, SCOPE_CH1, 6144u) != 0) return 0;

    t0 = HAL_GetTick();
    while (!g_scope_done && (HAL_GetTick() - t0) < 200u) {
        /* 等三路 DMA 完成 */
    }

    if (!g_scope_done) {
        ScopeAcq_Abort();
        Debug_Print("[ACQ] interleave self-test FAILED (a channel never triggered)\r\n");
        return 0;
    }

    {
        /* 用 DWT 周期数反算真实采样率，并对采到的数据做一次零交叉测频。
           这样不接上位机也能判断"触发有没有被丢、频率读得对不对"。 */
        static uint16_t tbuf[256];
        char b[220];
        uint32_t off = 0UL, total = ScopeAcq_Total(), nz = 0UL, k, thr = 0UL;
        int32_t  vmin =  32767, vmax = -32768, prev = 0, have = 0, state = 0;
        uint32_t fs_meas = 0UL;

        while (off < total) {
            uint32_t cnt = ScopeAcq_Fill(off, tbuf, 256u);
            if (cnt == 0UL) break;
            for (k = 0u; k < cnt; k++) {
                int32_t x = (int32_t)(int16_t)tbuf[k];
                if (x < vmin) vmin = x;
                if (x > vmax) vmax = x;
                if (have) {
                    /* 带回环的过零计数：噪声 ±10 码不会来回抖出多次过零 */
                    if (state == 0) {
                        if (x >  (vmax / 8)) { state = 1; nz++; }
                        else if (x < -(vmax / 8)) { state = 2; nz++; }
                    } else if (state == 1) {
                        if (x < -(vmax / 8)) { state = 2; nz++; }
                    } else {
                        if (x >  (vmax / 8)) { state = 1; nz++; }
                    }
                }
                prev = x; have = 1;
            }
            off += cnt;
        }
        if (g_scope_cyc > 0UL) {
            fs_meas = (uint32_t)(((uint64_t)total * 170000000ULL) / (uint64_t)g_scope_cyc);
        }
        {
            /* 频率 = (零交叉次数 / 2) / (total / fs_meas) */
            uint32_t f_est = 0UL;
            if (fs_meas > 0UL && total > 0UL) {
                uint64_t num = (uint64_t)(nz / 2UL) * (uint64_t)fs_meas;
                f_est = (uint32_t)(num / (uint64_t)total);
            }
            snprintf(b, sizeof(b),
                     "[ACQ] self-test cyc=%lu us=%lu n=%lu fs_meas=%lu Hz | codes[%ld,%ld] nz=%lu f_est=%lu Hz\r\n",
                     (unsigned long)g_scope_cyc, (unsigned long)(g_scope_cyc / 170UL),
                     (unsigned long)total, (unsigned long)fs_meas,
                     (long)vmin, (long)vmax, (unsigned long)nz, (unsigned long)f_est);
            Debug_Print(b);
        }
        {
            /* 形状抽点：连续 256 点分 4 行打出来，PC 上直接画图看波形 */
            static uint16_t dbuf[64];
            char c[420];
            uint32_t k0, got;
            int  w, i;
            for (k0 = 0UL; k0 < 256UL; k0 += 64UL) {
                got = ScopeAcq_Fill(k0, dbuf, 64u);
                if (got == 0UL) break;
                w = snprintf(c, sizeof(c), "[ACQ] seg%lu:", (unsigned long)k0);
                for (i = 0; i < (int)got && w < (int)sizeof(c) - 8; i++) {
                    w += snprintf(c + w, sizeof(c) - (size_t)w, " %ld", (long)(int16_t)dbuf[i]);
                }
                snprintf(c + w, sizeof(c) - (size_t)w, "\r\n");
                Debug_Print(c);
            }
        }
    }
    return 1;
}

/* ── 时钟探针：逐点扫描每路周期 P，用 DWT 反算真实采样率 ──
 * 目的：判定 ADC 内核时钟到底是 PLLP(56.67MHz) 还是它的一半(28.33MHz)。
 * 若一次转换耗时 Tconv，则 P*tick < Tconv 时会出现"隔一次触发才转换"，
 * 实测 fs 会掉到标称的一半。扫描一遍就能看出阈值落在哪个 P 上。 */
void ScopeAcq_ClockProbe(void)
{
    static const uint16_t ps[] = { 4u, 5u, 6u, 7u, 8u, 9u, 10u, 11u, 12u, 14u, 16u };
    char b[160];
    uint32_t i, t0;

    if (s_armed) return;

    Debug_Print("[PROBE] P sweep start (tick=47.06ns)\r\n");

    for (i = 0u; i < (sizeof(ps) / sizeof(ps[0])); i++) {
        uint32_t nominal, cyc, nspt, fs;

        SampleClock_SetP(ps[i]);

        g_scope_done = 0;
        if (ScopeAcq_Start(SCOPE_MODE_SINGLE, SCOPE_CH1, 256u) != 0) {
            Debug_Print("[PROBE] start failed\r\n");
            continue;
        }

        t0 = HAL_GetTick();
        while (!g_scope_done && (HAL_GetTick() - t0) < 50u) {
            /* 等 DMA 完成 */
        }
        if (!g_scope_done) {
            ScopeAcq_Abort();
            snprintf(b, sizeof(b), "[PROBE] P=%u TIMEOUT\r\n", (unsigned)ps[i]);
            Debug_Print(b);
            continue;
        }

        cyc    = g_scope_cyc;
        nominal = SAMPLE_TICK_HZ / (uint32_t)ps[i];
        nspt   = (cyc == 0UL) ? 0UL
                               : (uint32_t)(((uint64_t)cyc * 1000000000ULL) / 170000000ULL / 256ULL);
        fs     = (cyc == 0UL) ? 0UL : (uint32_t)(((uint64_t)256ULL * 170000000ULL) / (uint64_t)cyc);

        snprintf(b, sizeof(b),
                 "[PROBE] P=%2u nominal=%7lu Hz  fs_meas=%7lu Hz  ns/pt=%4lu  ratio=%lu/100\r\n",
                 (unsigned)ps[i], (unsigned long)nominal, (unsigned long)fs,
                 (unsigned long)nspt,
                 (unsigned long)((nominal == 0UL) ? 0UL : (fs * 100UL / nominal)));
        Debug_Print(b);
    }

    SampleClock_Set(0u);
    Debug_Print("[PROBE] P sweep done, restored gear0\r\n");
}
