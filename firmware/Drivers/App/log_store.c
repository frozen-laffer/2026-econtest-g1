/**
 * @file    log_store.c
 * @brief   掉电日志环：固定 128 字节槽位，环形覆盖，不跨 256 字节页
 *
 * 为什么用固定槽而不是紧凑流：W25Qxx_WritePage() 是裸页编程，起始地址没
 * 256 字节对齐就会在页内回卷。128 字节槽天然对齐、单次写不跨页，最省事。
 * 8 个扇区 × 32 槽 = 256 条，够存一次上电到出问题的全过程。
 */
#include "log_store.h"
#include "../Flash/w25qxx.h"
#include "../debug_print/debug_print.h"
#include <string.h>
#include <stdio.h>

#define SLOT_SIZE   128UL
#define SLOTS_PER    (LOG_SECTOR_SIZE / SLOT_SIZE)          /* 32 */
#define SLOT_TOTAL   (LOG_SECTORS * SLOTS_PER)              /* 256 */

static uint8_t  s_ready;
static uint32_t s_seq;                 /* 最大序号（0 = 还没写过） */
static uint32_t s_newest;              /* 最新记录所在槽号，0xFFFFFFFF = 无 */

static uint8_t  s_work[SLOT_SIZE];     /* 一条记录的工作缓冲 */

static uint16_t ls_crc16(const uint8_t *p, uint32_t n)
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

static uint32_t ls_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void ls_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint32_t ls_slot_addr(uint32_t slot)
{
    uint32_t sec = slot / SLOTS_PER;
    uint32_t off = (slot % SLOTS_PER) * SLOT_SIZE;
    return LOG_BASE + sec * LOG_SECTOR_SIZE + off;
}

static uint8_t ls_ensure(void)
{
    if (s_ready) {
        return 1u;
    }
    s_ready = (W25Qxx_Init() == HAL_OK) ? 1u : 0u;
    return s_ready;
}

/* 读一个槽，校验通过就返回 1 并填 hdr/lvl/len */
static uint8_t ls_read_slot(uint32_t slot, uint32_t *seq, uint8_t *lvl,
                            uint8_t *len, char *text)
{
    uint8_t  h[LOG_HDR_LEN];
    uint32_t n;

    W25Qxx_ReadData(ls_slot_addr(slot), h, LOG_HDR_LEN);
    n = ls_get_u32(h);
    if (n == 0xFFFFFFFFu || h[5] > LOG_TEXT_MAX) {
        return 0u;
    }
    if (text != NULL && h[5] > 0u) {
        W25Qxx_ReadData(ls_slot_addr(slot) + LOG_HDR_LEN, (uint8_t *)text, h[5]);
        if (ls_crc16((const uint8_t *)text, h[5]) != (uint16_t)((uint16_t)h[6]
                | ((uint16_t)h[7] << 8))) {
            return 0u;                      /* 掉电写了半条 */
        }
    }
    *seq = n;
    *lvl = h[4];
    *len = h[5];
    return 1u;
}

uint32_t LogStore_Init(void)
{
    uint32_t i, seq, newest = 0xFFFFFFFFu, maxseq = 0u;
    uint8_t  lvl, len;

    s_seq = 0u;
    s_newest = 0xFFFFFFFFu;
    if (!ls_ensure()) {
        return 0u;
    }
    for (i = 0u; i < SLOT_TOTAL; i++) {
        if (ls_read_slot(i, &seq, &lvl, &len, NULL)) {
            if (seq >= maxseq) {            /* 同号取靠后的，环形写会更靠后 */
                maxseq = seq;
                newest = i;
            }
        }
    }
    s_seq = maxseq;
    s_newest = newest;
    return s_seq;
}

uint8_t LogStore_Put(uint8_t level, const char *text, uint8_t len)
{
    uint32_t slot, sec, sib;
    uint16_t crc;
    char     echo[LOG_TEXT_MAX + 24u];

    if (!ls_ensure()) {
        return 1u;
    }
    if (text == NULL) {
        len = 0u;
    }
    if (len > LOG_TEXT_MAX) {
        len = (uint8_t)LOG_TEXT_MAX;
    }

    s_seq++;
    slot = (s_newest == 0xFFFFFFFFu) ? 0u : ((s_newest + 1u) % SLOT_TOTAL);
    sec  = slot / SLOTS_PER;
    sib  = slot % SLOTS_PER;

    if (sib == 0u) {
        W25Qxx_EraseSector(LOG_BASE + sec * LOG_SECTOR_SIZE);
    }

    memset(s_work, 0xFF, sizeof(s_work));
    ls_put_u32(&s_work[0], s_seq);
    s_work[4] = level;
    s_work[5] = len;
    if (len > 0u) {
        memcpy(&s_work[LOG_HDR_LEN], text, len);
    }
    crc = ls_crc16((const uint8_t *)&s_work[LOG_HDR_LEN], len);
    s_work[6] = (uint8_t)(crc & 0xFFu);
    s_work[7] = (uint8_t)((crc >> 8) & 0xFFu);

    W25Qxx_WritePage(ls_slot_addr(slot), s_work, (uint16_t)SLOT_SIZE);
    s_newest = slot;

    /* 同时从调试串口打一遍：Pi 挂了也能在串口上看到最后一条 */
    (void)snprintf(echo, sizeof(echo), "[LOG] #%lu L%u %.*s\r\n",
                   (unsigned long)s_seq, (unsigned)level, (int)len,
                   (len > 0u) ? text : "");
    Debug_Print(echo);
    return 0u;
}

void LogStore_Stat(uint32_t *used_sectors, uint32_t *seq)
{
    if (used_sectors != NULL) {
        *used_sectors = (s_seq == 0u) ? 0u
                      : ((s_newest == 0xFFFFFFFFu) ? 0u : (s_newest / SLOTS_PER + 1u));
    }
    if (seq != NULL) {
        *seq = s_seq;
    }
}

/* 环形顺序 = 序号顺序：从"最新槽的下一个"开始走，天然由旧到新 */
uint32_t LogStore_Dump(uint32_t from_seq, uint8_t *buf, uint32_t cap)
{
    uint32_t used = 0u, i, slot, seq;
    uint8_t  lvl, len;
    char     text[LOG_TEXT_MAX + 1u];
    int      w;

    if (buf == NULL || cap < 24u || s_seq == 0u) {
        return 0u;
    }
    for (i = 0u; i < SLOT_TOTAL; i++) {
        slot = ((s_newest == 0xFFFFFFFFu) ? 0u : ((s_newest + 1u) % SLOT_TOTAL)) + i;
        slot %= SLOT_TOTAL;
        if (!ls_read_slot(slot, &seq, &lvl, &len, text)) {
            continue;
        }
        if (seq < from_seq) {
            continue;
        }
        text[len] = '\0';
        w = snprintf((char *)buf + used, cap - used, "%lu\tL%u\t%s\n",
                     (unsigned long)seq, (unsigned)lvl, text);
        if (w <= 0 || (uint32_t)w >= (cap - used)) {
            return used;                    /* 装不下了，剩下的下次再取 */
        }
        used += (uint32_t)w;
    }
    return used;
}

void LogStore_Clear(void)
{
    uint32_t i;

    if (!ls_ensure()) {
        return;
    }
    for (i = 0u; i < LOG_SECTORS; i++) {
        W25Qxx_EraseSector(LOG_BASE + i * LOG_SECTOR_SIZE);
    }
    s_seq = 0u;
    s_newest = 0xFFFFFFFFu;
}
