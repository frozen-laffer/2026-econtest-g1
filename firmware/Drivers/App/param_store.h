#ifndef PARAM_STORE_H
#define PARAM_STORE_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 参数区（W25Q16，2MB，挂在 SPI1，CS=PB6）
 *
 * 目的：把上位机算出来的校准/激励参数（eq.json + config.json + calib.json +
 * excite.json 打成一个 blob）存到**板子自己**的 Flash 上，这样树莓派重刷系统
 * 或换卡都不丢参数，一条命令就能读回来。
 *
 * 布局（两个 8KB 槽轮流写，掉电时至少有一个槽完整）：
 *   0x00100000  A 槽（2 个 4KB 扇区）
 *   0x00102000  B 槽（2 个 4KB 扇区）
 *   每个槽 = 256 字节头区 + 数据（数据从第 1 页起，保证 256 字节页对齐 ——
 *   W25Qxx_WritePage() 是裸的"页编程"，起始地址没对齐就会在页内回卷写坏）
 *   头: 'L','S','P','1' | ver u16 | len u32 | seq u32 | crc16 u16 | rsv u16
 *
 * 0x000000~0x03FFFF 是示波器快照区（touch_calibration 也用了一点），
 * 0x100000 处的"测试区"已改成参数区。
 */

#define PARAM_MAGIC0        0x4Cu   /* 'L' */
#define PARAM_MAGIC1        0x53u   /* 'S' */
#define PARAM_MAGIC2        0x50u   /* 'P' */
#define PARAM_MAGIC3        0x31u   /* '1' */
#define PARAM_VER           1u

#define PARAM_SLOT_A        0x00100000UL
#define PARAM_SLOT_B        0x00102000UL
#define PARAM_SLOT_SIZE     0x00002000UL   /* 每个槽占 8KB = 2 个扇区 */
#define PARAM_HDR_LEN       16UL
#define PARAM_DATA_OFS      256UL          /* 数据从整页开始，保证页对齐 */
#define PARAM_MAX_LEN       7168UL         /* 256 + 7168 = 7424 < 8192 */
#define PARAM_PAGE          256UL

typedef struct {
    uint8_t  magic[4];
    uint16_t ver;
    uint32_t len;
    uint32_t seq;
    uint16_t crc16;
    uint16_t rsv;
} ParamHdr_t;                       /* 16 B */

/**
 * @brief  上电扫描两个槽，把最新的有效参数读进 RAM，并返回它的长度
 * @retval 0 表示没有有效参数；否则为字节数
 */
uint32_t ParamStore_Init(void);

/** @brief 当前 RAM 里有效参数的长度（0 = 没有） */
uint32_t ParamStore_Size(void);

/** @brief 当前 RAM 里有效参数的写入序号（每次 Commit +1，用来判断新旧） */
uint32_t ParamStore_Seq(void);

/** @brief 当前 RAM 里有效参数的指针（ParamStore_Size() 为 0 时不可用） */
const uint8_t *ParamStore_Ptr(void);

/**
 * @brief  开始接收一组新参数（先校验长度，再准备缓冲）
 * @param  len  这次的参数总字节数（<= PARAM_MAX_LEN）
 * @retval HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef ParamStore_Begin(uint32_t len);

/**
 * @brief  写入一段数据（按偏移，允许乱序重传）
 */
HAL_StatusTypeDef ParamStore_Chunk(uint32_t off, const uint8_t *data, uint32_t len);

/**
 * @brief  收齐后落盘：算 CRC、选旧槽擦除写入、回读校验
 */
HAL_StatusTypeDef ParamStore_Commit(void);

/** @brief 清掉（把两个槽的头都擦掉） */
HAL_StatusTypeDef ParamStore_Clear(void);

/** @brief 板子自带的自检：擦 A → 写 → 读回比对，返回 0 表示通过 */
uint8_t ParamStore_SelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* PARAM_STORE_H */
