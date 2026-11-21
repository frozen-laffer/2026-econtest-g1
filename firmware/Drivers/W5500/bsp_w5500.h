#ifndef __BSP_W5500_H
#define __BSP_W5500_H

#include "main.h"
#include "spi.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── BSP 硬件抽象层：封装已验证的 SPI3 / GPIO / 中断锁 ── */

/* 单字节 SPI 读写（底层原语，不操作 CS） */
uint8_t BSP_W5500_SPI_ReadWrite(uint8_t tx_data);

/* CS 控制 + 中断锁（在 CS 选中时自动加锁） */
void BSP_W5500_Select(void);
void BSP_W5500_Deselect(void);

/* 硬件复位 */
void BSP_W5500_Reset(void);

/* SPI Burst 读写（循环调用单字节，兼容 ioLibrary） */
void BSP_W5500_ReadBurst(uint8_t *pBuf, uint16_t len);
void BSP_W5500_WriteBurst(uint8_t *pBuf, uint16_t len);

/* BSP 初始化（配置 CS 引脚为高、注册 ioLibrary 回调） */
void BSP_W5500_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_W5500_H */