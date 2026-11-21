/**
 * @file    log_store.h
 * @brief   掉电日志环（W25Q16 上的独立区域，和参数区互不干扰）
 *
 * 用途：树莓派上的监测程序把运行日志/报错逐条发给板子，板子写进自己的
 * Flash。这样**系统卡-重刷都不会丢日志**，拔下 Pi 把板子接到串口也能看到
 * 最后一次报错（每条都会同时从调试串口打一遍）。
 *
 * 布局（8 个 4KB 扇区 = 32KB，环形覆盖）：
 *   0x00104000 + i*0x1000, i = 0..7
 *
 * 每条记录（不跨扇区）：
 *   seq u32 | level u8 | len u8 | crc16 | text[len]
 *   crc16 = CRC-16/CCITT(text)，用来判断这条记录是否写完整（掉电半条）
 */
#ifndef LOG_STORE_H
#define LOG_STORE_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LOG_BASE        0x00104000UL
#define LOG_SECTORS     8UL
#define LOG_SECTOR_SIZE 4096UL
#define LOG_HDR_LEN     8UL          /* seq u32 + level u8 + len u8 + crc u16 */
#define LOG_TEXT_MAX    120UL        /* 单条正文上限 */

/* 日志级别（和上位机侧保持一致） */
#define LOG_LEVEL_DEBUG 0u
#define LOG_LEVEL_INFO  1u
#define LOG_LEVEL_WARN  2u
#define LOG_LEVEL_ERR   3u

/** @brief 上电扫描扇区，找到最新写位置；返回当前最大序号（0 = 空） */
uint32_t LogStore_Init(void);

/** @brief 追加一条日志（同时从调试串口打印），返回 0 成功 */
uint8_t LogStore_Put(uint8_t level, const char *text, uint8_t len);

/** @brief 当前状态：已用扇区数 / 正在写的扇区 / 最大序号 */
void LogStore_Stat(uint32_t *used_sectors, uint32_t *seq);

/**
 * @brief 把 >= from_seq 的记录按文本行导出到 buf
 * @retval 写入的字节数（0 = 没有更多）；行格式 "seq<TAB>L<level><TAB>text\n"
 */
uint32_t LogStore_Dump(uint32_t from_seq, uint8_t *buf, uint32_t cap);

/** @brief 抹掉所有日志（8 个扇区全擦） */
void LogStore_Clear(void);

#ifdef __cplusplus
}
#endif

#endif /* LOG_STORE_H */
