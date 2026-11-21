/**
 * @file    param_store.c
 * @brief   W25Q16 上的参数备份区（A/B 双槽 + CRC，掉电不丢）
 *
 * 注意：读槽时**不要**顺手把数据读进 s_buf —— Commit 里要拿 s_buf 当接收缓冲，
 * 判定槽有效性必须只看 16 字节头，所以分成 ps_slot_info()/ps_slot_load()。
 */
#include "param_store.h"
#include "../Flash/w25qxx.h"
#include <string.h>

static uint8_t  s_buf[PARAM_MAX_LEN];   /* 接收缓冲 + 当前有效参数，一份就好 */
static uint32_t s_len;                  /* RAM 里有效参数长度（0=无） */
static uint32_t s_seq;                  /* RAM 里有效参数的序号 */
static uint32_t s_rx_len;
static uint32_t s_rx_got;
static uint8_t  s_rx_on;
static uint8_t  s_ready;               /* W25Qxx_Init() 是否成功过 */

/* W25Q16 的 CS(PB6) 要配成 GPIO 输出、SPI1 要已初始化，否则所有读写都是
   空操作（读到 0xFF）。main.c 里没人调 W25Qxx_Init()，这里自己保证一次。 */
static uint8_t ps_ensure(void)
{
    if (s_ready) {
        return 1u;
    }
    s_ready = (W25Qxx_Init() == HAL_OK) ? 1u : 0u;
    return s_ready;
}

/* ── 小工具 ────────────────────────────────────────────────────────── */
static uint16_t ps_crc16(const uint8_t *p, uint32_t n)
{
    uint16_t c = 0xFFFFu;
    uint32_t i;
    uint8_t  b;
    for (i = 0u; i < n; i++) {
        c ^= (uint16_t)((uint16_t)p[i] << 8);
        for (b = 0u; b < 8u; b++) {
            c = (uint16_t)((c & 0x8000u) ? ((uint16_t)(c << 1) ^ 0x1021u)
                                         : (uint16_t)(c << 1));
        }
    }
    return c;
}

static uint32_t ps_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void ps_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void ps_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t ps_get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* 只读 16 字节头：magic/ver/len 合法就认为"槽里有东西"，不校验 CRC */
static uint8_t ps_slot_info(uint32_t base, uint32_t *len, uint32_t *seq)
{
    uint8_t h[PARAM_HDR_LEN];

    W25Qxx_ReadData(base, h, PARAM_HDR_LEN);
    if (h[0] != PARAM_MAGIC0 || h[1] != PARAM_MAGIC1
     || h[2] != PARAM_MAGIC2 || h[3] != PARAM_MAGIC3) {
        return 0u;
    }
    if (ps_get_u16(&h[4]) != (uint16_t)PARAM_VER) {
        return 0u;
    }
    *len = ps_get_u32(&h[6]);
    *seq = ps_get_u32(&h[10]);
    return (*len > 0u && *len <= PARAM_MAX_LEN) ? 1u : 0u;
}

/* 读头 + 数据 + 校验 CRC，成功才把数据拷进 dst */
static uint8_t ps_slot_load(uint32_t base, uint8_t *dst, uint32_t *len, uint32_t *seq)
{
    uint8_t  h[PARAM_HDR_LEN];
    uint16_t crc;
    uint32_t n, s;

    if (!ps_slot_info(base, &n, &s)) {
        return 0u;
    }
    W25Qxx_ReadData(base, h, PARAM_HDR_LEN);
    crc = ps_get_u16(&h[14]);
    W25Qxx_ReadData(base + PARAM_DATA_OFS, dst, n);
    if (ps_crc16(dst, n) != crc) {
        return 0u;
    }
    *len = n;
    *seq = s;
    return 1u;
}

/* 擦整槽（2 个扇区）→ 写头 → 按页写数据 → 回读头 + CRC 确认 */
static HAL_StatusTypeDef ps_write_slot(uint32_t base, const uint8_t *data,
                                       uint32_t len, uint32_t seq)
{
    uint8_t  h[PARAM_HDR_LEN];
    uint8_t  back[PARAM_HDR_LEN];
    uint8_t  chk[64];
    uint32_t off, k;
    uint16_t crc = ps_crc16(data, len);

    h[0] = PARAM_MAGIC0; h[1] = PARAM_MAGIC1;
    h[2] = PARAM_MAGIC2; h[3] = PARAM_MAGIC3;
    ps_put_u16(&h[4], (uint16_t)PARAM_VER);
    ps_put_u32(&h[6], len);
    ps_put_u32(&h[10], seq);
    ps_put_u16(&h[14], crc);

    for (k = 0u; k < PARAM_SLOT_SIZE; k += 4096UL) {
        W25Qxx_EraseSector(base + k);
    }
    W25Qxx_WritePage(base, h, (uint16_t)PARAM_HDR_LEN);
    for (off = 0u; off < len; off += PARAM_PAGE) {
        uint32_t n = len - off;
        if (n > PARAM_PAGE) {
            n = PARAM_PAGE;
        }
        W25Qxx_WritePage(base + PARAM_DATA_OFS + off, data + off, (uint16_t)n);
    }
    /* 头对了才算写成功；再抽查数据首尾，页对齐错了立刻能看出来 */
    W25Qxx_ReadData(base, back, sizeof(back));
    if (memcmp(back, h, PARAM_HDR_LEN) != 0) {
        return HAL_ERROR;
    }
    k = (len > sizeof(chk)) ? (uint32_t)sizeof(chk) : len;
    W25Qxx_ReadData(base + PARAM_DATA_OFS, chk, k);
    if (memcmp(chk, data, k) != 0) {
        return HAL_ERROR;
    }
    k = (len > sizeof(chk)) ? (len - (uint32_t)sizeof(chk)) : 0u;
    W25Qxx_ReadData(base + PARAM_DATA_OFS + k, chk, len - k);
    return (memcmp(chk, data + k, len - k) == 0) ? HAL_OK : HAL_ERROR;
}

/* ── 对外 API ──────────────────────────────────────────────────────── */

uint32_t ParamStore_Init(void)
{
    uint32_t la = 0u, lb = 0u, sa = 0u, sb = 0u;
    uint8_t  oka, okb;

    s_len = 0u;
    s_seq = 0u;
    s_rx_on = 0u;

    if (!ps_ensure()) {
        return 0u;                     /* 片子没认出来，当作没有参数 */
    }

    /* 先只读两个头（不占额外 RAM），再按序号决定读哪一个进 s_buf */
    oka = ps_slot_info(PARAM_SLOT_A, &la, &sa);
    okb = ps_slot_info(PARAM_SLOT_B, &lb, &sb);

    if (oka && okb) {
        if (sb > sa) {
            if (ps_slot_load(PARAM_SLOT_B, s_buf, &lb, &sb)) {
                s_len = lb; s_seq = sb;
            } else if (ps_slot_load(PARAM_SLOT_A, s_buf, &la, &sa)) {
                s_len = la; s_seq = sa;
            }
        } else {
            if (ps_slot_load(PARAM_SLOT_A, s_buf, &la, &sa)) {
                s_len = la; s_seq = sa;
            } else if (ps_slot_load(PARAM_SLOT_B, s_buf, &lb, &sb)) {
                s_len = lb; s_seq = sb;
            }
        }
    } else if (oka) {
        if (ps_slot_load(PARAM_SLOT_A, s_buf, &la, &sa)) {
            s_len = la; s_seq = sa;
        }
    } else if (okb) {
        if (ps_slot_load(PARAM_SLOT_B, s_buf, &lb, &sb)) {
            s_len = lb; s_seq = sb;
        }
    }
    return s_len;
}

uint32_t ParamStore_Size(void)      { return s_len; }
uint32_t ParamStore_Seq(void)       { return s_seq; }
const uint8_t *ParamStore_Ptr(void) { return s_buf; }

HAL_StatusTypeDef ParamStore_Begin(uint32_t len)
{
    if (len == 0u || len > PARAM_MAX_LEN) {
        return HAL_ERROR;
    }
    s_rx_len = len;
    s_rx_got = 0u;
    s_rx_on  = 1u;
    memset(s_buf, 0, PARAM_MAX_LEN);
    return HAL_OK;
}

HAL_StatusTypeDef ParamStore_Chunk(uint32_t off, const uint8_t *data, uint32_t len)
{
    if (!s_rx_on || data == NULL || len == 0u) {
        return HAL_ERROR;
    }
    if (off >= s_rx_len || len > (s_rx_len - off)) {
        return HAL_ERROR;
    }
    memcpy(s_buf + off, data, len);
    if ((off + len) > s_rx_got) {
        s_rx_got = off + len;
    }
    return HAL_OK;
}

HAL_StatusTypeDef ParamStore_Commit(void)
{
    uint32_t la = 0u, lb = 0u, sa = 0u, sb = 0u;
    uint8_t  oka, okb;
    uint32_t tgt, seq;
    HAL_StatusTypeDef st;

    if (!s_rx_on || s_rx_got != s_rx_len) {
        return HAL_ERROR;
    }
    if (!ps_ensure()) {
        s_rx_on = 0u;
        return HAL_ERROR;
    }
    oka = ps_slot_info(PARAM_SLOT_A, &la, &sa);   /* 只看头，不碰 s_buf */
    okb = ps_slot_info(PARAM_SLOT_B, &lb, &sb);

    if (!oka && !okb) {
        tgt = PARAM_SLOT_A; seq = 1u;
    } else if (oka && okb) {
        tgt = (sa <= sb) ? PARAM_SLOT_A : PARAM_SLOT_B;   /* 覆盖旧的那个 */
        seq = ((sa > sb) ? sa : sb) + 1u;
    } else if (oka) {
        tgt = PARAM_SLOT_B; seq = sa + 1u;
    } else {
        tgt = PARAM_SLOT_A; seq = sb + 1u;
    }

    st = ps_write_slot(tgt, s_buf, s_rx_len, seq);
    if (st == HAL_OK) {
        s_len = s_rx_len;
        s_seq = seq;
    }
    s_rx_on = 0u;
    return st;
}

HAL_StatusTypeDef ParamStore_Clear(void)
{
    if (!ps_ensure()) {
        return HAL_ERROR;
    }
    W25Qxx_EraseSector(PARAM_SLOT_A);
    W25Qxx_EraseSector(PARAM_SLOT_A + 4096UL);
    W25Qxx_EraseSector(PARAM_SLOT_B);
    W25Qxx_EraseSector(PARAM_SLOT_B + 4096UL);
    s_len = 0u;
    s_seq = 0u;
    s_rx_on = 0u;
    return HAL_OK;
}

uint8_t ParamStore_SelfTest(void)
{
    uint8_t  tmp[64];
    uint8_t  i;
    /* 不额外占 8KB 备份缓冲：Commit 只会覆盖"旧"的那个槽，新参数原样留在
       另一个槽里，自检完再 ParamStore_Init() 就能把真参数读回来。 */
    for (i = 0u; i < sizeof(tmp); i++) {
        tmp[i] = (uint8_t)((i * 7u) + 3u);
    }
    if (ParamStore_Begin(sizeof(tmp)) != HAL_OK) {
        return 1u;
    }
    if (ParamStore_Chunk(0u, tmp, (uint32_t)sizeof(tmp)) != HAL_OK) {
        return 2u;
    }
    if (ParamStore_Commit() != HAL_OK) {
        return 3u;
    }
    if (ParamStore_Init() != (uint32_t)sizeof(tmp)) {
        return 4u;
    }
    if (memcmp(tmp, s_buf, sizeof(tmp)) != 0) {
        return 5u;
    }
    (void)ParamStore_Init();             /* 再把真正的参数读回来 */
    return 0u;
}
