#ifndef W25QXX_H
#define W25QXX_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* W25Q16: 2MB (16Mbit), 512 sectors of 4KB, 32 blocks of 64KB */
#define W25Q16_FLASH_SIZE       0x200000    /* 2MB */
#define W25Q16_SECTOR_SIZE      0x1000      /* 4KB */
#define W25Q16_BLOCK_SIZE       0x10000     /* 64KB */
#define W25Q16_PAGE_SIZE        256         /* 256 bytes */

/* Scope data storage layout in Flash */
#define FLASH_SCOPE_BASE        0x000000    /* Start at sector 0 */
#define FLASH_SCOPE_MAX_SECTORS 64          /* Use 64 sectors (256KB) */
#define FLASH_WAVEFORM_POINTS   240         /* Points per capture */

/*
 * Flash stored scope snapshot (one per save)
 * Layout per snapshot: 4 bytes header + 240*2 bytes CH1 + 240*2 bytes CH2 = 964 bytes
 * Per sector: 4096 / 964 ≈ 4 snapshots
 */
typedef struct {
    uint32_t timestamp;                          /* HAL_GetTick() at capture */
    int16_t  ch1_data[FLASH_WAVEFORM_POINTS];    /* CH1 waveform */
    int16_t  ch2_data[FLASH_WAVEFORM_POINTS];    /* CH2 waveform */
} ScopeSnapshot;

/* ── API ── */

/**
 * @brief  Initialize W25Q16 via SPI4
 * @return HAL_OK on success
 */
HAL_StatusTypeDef W25Qxx_Init(void);

/**
 * @brief  Read JEDEC ID (Manufacturer + Memory Type + Capacity)
 * @param id  Pointer to 3-byte buffer
 */
void W25Qxx_ReadID(uint8_t *id);

/**
 * @brief  Erase a 4KB sector
 * @param addr  Sector address (must be sector-aligned)
 */
void W25Qxx_EraseSector(uint32_t addr);

/**
 * @brief  Write page (up to 256 bytes)
 * @param addr  Address to write
 * @param data  Data buffer
 * @param len   Number of bytes (max 256, must not cross page boundary)
 */
void W25Qxx_WritePage(uint32_t addr, const uint8_t *data, uint16_t len);

/**
 * @brief  Read data from flash
 * @param addr  Start address
 * @param data  Output buffer
 * @param len   Number of bytes to read
 */
void W25Qxx_ReadData(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief  Save current scope waveform to flash
 * @param snap_idx  Snapshot index (0..255)
 */
void W25Qxx_SaveScopeSnapshot(const ScopeSnapshot *snap, uint32_t snap_idx);

/**
 * @brief  Load scope waveform from flash
 * @param snap_idx  Snapshot index to load
 * @param snap      Output buffer
 */
void W25Qxx_LoadScopeSnapshot(ScopeSnapshot *snap, uint32_t snap_idx);

/**
 * @brief  Get number of saved snapshots
 */
uint32_t W25Qxx_GetSnapshotCount(void);

/**
 * @brief  W25Q16 读写测试函数：擦除扇区 → 写入测试模式 → 读回验证 → 串口回传
 * @note   测试地址 0x100000 (1MB 处，不影响示波器存储区 0~256KB)
 *         写入 256 字节递增数据，读回后逐字节比对并打印
 *         可通过 UART1 观察完整读写过程
 */
void W25Qxx_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* W25QXX_H */