/**
  * @file    bsp_w5500.c
  * @brief   W5500 BSP - DMA Burst 优化版（DMA 轮询完成标志，不与 ILI9488 的 HAL 回调冲突）
  */
#include "bsp_w5500.h"
#include "../debug_print/debug_print.h"
#include "Ethernet/wizchip_conf.h"
#include <string.h>

extern DMA_HandleTypeDef hdma_spi3_tx;

/* ═══════════════════════════════════════════════════════════════
 * 单字节 SPI（HAL 库，用于寄存器读写 — 保持可靠）
 * ═══════════════════════════════════════════════════════════════ */

uint8_t BSP_W5500_SPI_ReadWrite(uint8_t tx_data)
{
    uint8_t rx_data = 0;
    uint32_t timeout = 10000;   /* 防止死锁 */

    /* 如果 SPI 卡死，先尝试 Abort */
    if (hspi3.State == HAL_SPI_STATE_BUSY || (SPI3->SR & SPI_SR_BSY)) {
        HAL_SPI_Abort(&hspi3);
        HAL_Delay(1);
    }

    if (HAL_SPI_TransmitReceive(&hspi3, &tx_data, &rx_data, 1, 5) != HAL_OK) {
        Debug_Print("[SPI] TransmitReceive failed!\r\n");
        /* 强制复位 */
        CLEAR_BIT(SPI3->CR1, SPI_CR1_SPE);
        HAL_Delay(1);
        SET_BIT(SPI3->CR1, SPI_CR1_SPE);
    }

    /* 修改 A：读操作后增加延迟，给 W5500 MISO 输出留稳定时间 */
    if (tx_data == 0xFF) {
        for (volatile int i = 0; i < 20; i++);
    }
    return rx_data;
}

static uint8_t BSP_W5500_SPI_ReadByte(void)
{
    return BSP_W5500_SPI_ReadWrite(0xFF);
}

static void BSP_W5500_SPI_WriteByte(uint8_t wb)
{
    BSP_W5500_SPI_ReadWrite(wb);
    __NOP(); __NOP(); __NOP(); __NOP();
}

/* ═══════════════════════════════════════════════════════════════
 * DMA Burst 读写（wiz_send_data 的 3KB 数据负载走这里）
 * ═══════════════════════════════════════════════════════════════ */

/* ── Polling 版本 Burst 函数（安全回退，确保基础通信正常） ── */

void BSP_W5500_WriteBurst(uint8_t *pBuf, uint16_t len)
{
    if (len == 0) return;
    HAL_SPI_Transmit(&hspi3, pBuf, len, 100);
}

void BSP_W5500_ReadBurst(uint8_t *pBuf, uint16_t len)
{
    if (len == 0) return;
    HAL_SPI_Receive(&hspi3, pBuf, len, 100);
}

/* ── DMA 版本（暂时禁用，待基础通信确认正常后再启用优化） ── */
#if 0

/* SPI 状态诊断 */
static void BSP_W5500_CheckSPIState(void)
{
    HAL_SPI_StateTypeDef state = HAL_SPI_GetState(&hspi3);
    if (state != HAL_SPI_STATE_READY)
    {
        Debug_Print("[W5500] SPI not ready!\r\n");
    }
}

void BSP_W5500_WriteBurst_DMA(uint8_t *pBuf, uint16_t len)
{
    if (len == 0) return;

    BSP_W5500_CheckSPIState();   /* 状态诊断 */

    BSP_W5500_Select();

    __HAL_DMA_CLEAR_FLAG(&hdma_spi3_tx, DMA_FLAG_TC1);
    if (HAL_SPI_Transmit_DMA(&hspi3, pBuf, len) != HAL_OK)
    {
        Debug_Print("[W5500] DMA Transmit start failed!\r\n");
        BSP_W5500_Deselect();
        return;
    }

    /* 轮询 DMA TC1 标志位（不与 ILI9488 的 HAL_SPI_TxCpltCallback 冲突） */
    uint32_t timeout = 500000;
    while (__HAL_DMA_GET_FLAG(&hdma_spi3_tx, DMA_FLAG_TC1) == RESET && --timeout)
        { __NOP(); }
    if (timeout == 0)
    {
        Debug_Print("[W5500] DMA Timeout!\r\n");
        HAL_SPI_DMAStop(&hspi3);
    }
    __HAL_DMA_CLEAR_FLAG(&hdma_spi3_tx, DMA_FLAG_TC1);

    BSP_W5500_Deselect();
}

void BSP_W5500_ReadBurst_DMA(uint8_t *pBuf, uint16_t len)
{
    if (len == 0) return;
    BSP_W5500_CheckSPIState();   /* 第1步：状态诊断 */

    BSP_W5500_Select();

    __HAL_DMA_CLEAR_FLAG(&hdma_spi3_tx, DMA_FLAG_TC1);
    HAL_SPI_Receive_DMA(&hspi3, pBuf, len);
    uint32_t timeout = 500000;
    while (__HAL_DMA_GET_FLAG(&hdma_spi3_tx, DMA_FLAG_TC1) == RESET && --timeout)
        { __NOP(); }
    __HAL_DMA_CLEAR_FLAG(&hdma_spi3_tx, DMA_FLAG_TC1);

    BSP_W5500_Deselect();
}

#endif

/* ═══════════════════════════════════════════════════════════════
 * CS / 复位控制
 * ═══════════════════════════════════════════════════════════════ */

void BSP_W5500_Select(void)
{
    //__disable_irq();//
    for (volatile uint32_t i = 0; i < 100; i++);   /* 第3步：加长 CS 建立延迟排除时序问题 */
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
    for (volatile uint32_t i = 0; i < 100; i++);
}

void BSP_W5500_Deselect(void)
{
    for (volatile uint32_t i = 0; i < 60; i++);
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
    for (volatile uint32_t i = 0; i < 100; i++);   /* 修改 C：加长 CS 高电平间隙 */
    //__enable_irq();//
}

void BSP_W5500_Reset(void)
{
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(200);
}

/* ═══════════════════════════════════════════════════════════════
 * BSP 初始化 & ioLibrary 回调注册
 * ═══════════════════════════════════════════════════════════════ */

void BSP_W5500_Init(void)
{
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

    reg_wizchip_cs_cbfunc(BSP_W5500_Select, BSP_W5500_Deselect);
    reg_wizchip_spi_cbfunc(BSP_W5500_SPI_ReadByte, BSP_W5500_SPI_WriteByte);

    /* 回退到 Polling 模式 — 先确保基础 SPI 通信正常（TXBUF 读回 8KB） */
    WIZCHIP.IF.SPI._read_burst  = BSP_W5500_ReadBurst;
    WIZCHIP.IF.SPI._write_burst = BSP_W5500_WriteBurst;

    Debug_Print("[BSP_W5500] Initialized with Polling Burst (safe fallback)\r\n");
}