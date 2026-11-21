/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdio.h>                          /* snprintf */
#include "../../Drivers/debug_print/debug_print.h"
#include "../../Drivers/W5500/w5500_network.h"
#include "../../Drivers/AFE/afe.h"          /* [P0-6] 模拟前端控制 */
#include "../../Drivers/App/sample_clock.h"  /* [P0-5] 采样率档位 */
#include "../../Drivers/App/scope_acq.h"     /* 三路交替单次采集 */
#include "../../Drivers/App/param_store.h"
#include "../../Drivers/App/log_store.h"
#include "../../Drivers/Flash/w25qxx.h"   /* W25Q16 参数备份区 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t breath_tick = 0;
volatile uint8_t g_system_ready = 0;

/* ─── [P0-2][P0-4] 高速采集 Ping-Pong 双缓冲 ───────────────────────────
 *  三路 ADC 现为【单通道真差分】, 由 scope_acq 模块按需单次采集 N 点
 *  缓冲位于 scope_acq.c: 3 x 8192 x 2B = 48 KB
 * ──────────────────────────────────────────────────────────────────── */
volatile uint8_t g_capture_mode = 0;

/* ─── [P0-10] 增益与标定系数 (标称值, 需实测标定后替换) ───
 *  总增益 = INA128 增益 x FDA 增益
 *      ×2  档: 2.002  x 1.9606 = 3.925
 *      ×20 档: 20.157 x 1.9606 = 39.52
 *  电压换算: V_in = raw x VREF / 2047 / 总增益
 *  见《技术方案与实施细节总文档》§3.1
 * ─────────────────────────────────────────────────────────── */
#define ADC_VREF_V      3.0f
#define ADC_HALF_SCALE  2047.0f      /* 12-bit 差分满量程 (非 32767!) */
volatile float   g_total_gain   = 3.925f;   /* 当前总增益 */
volatile int32_t g_offset_code  = 0;        /* 零点校正量 (标定得到) */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Debug_Print("\r\n==================================================\r\n");
  Debug_Print("SYSTEM CRITICAL BOOT / RESET DETECTED\r\n");
  Debug_Print("==================================================\r\n\r\n");
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC5_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  /* ═══════════════════════════════════════════════════════════════
   * [P0-6] 模拟前端初始化 —— 必须在 MX_GPIO_Init() 之后、采集启动之前
   *  上电安全默认态: DC 耦合 + 1x 衰减 + ×2.00 增益
   *  （继电器极性: 高=吸合; 复位默认低=全释放）
   * ═══════════════════════════════════════════════════════════════ */
  AFE_Init();

  /* ── 启动背光 PWM ── */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	/* ── 启动编码器（TIM4/TIM1 双编码器） ── */
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	/* ── 启动系统状态回传（USART1 DMA 非阻塞发送，默认 50ms/帧） ── */
	SysStatus_Init();

    __enable_irq();

    /* ═══════════════════════════════════════════════════════════
	 * W5500 网络配置（可在此修改 IP/网关/端口）
	 * ═══════════════════════════════════════════════════════════ */
	{
		wiz_NetInfo g_net_config = {
			.mac  = {0x00, 0x08, 0xDC, 0x01, 0x02, 0x03},
			.ip   = {192, 168, 57, 200},
			.sn   = {255, 255, 255, 0},
			.gw   = {192, 168, 57, 1},
			.dns  = {8, 8, 8, 8},
			.dhcp = NETINFO_STATIC
		};
		uint8_t remote_ip[] = {192, 168, 57, 100};  /* UDP 目标 PC/树莓派 IP */

		W5500_Network_Init(&g_net_config);         /* 1. 硬件复位 + 8KB SRAM 划分 */
		W5500_HighSpeed_UDP_Init(5001, remote_ip, 8080);  /* 2. Socket1 UDP 数据泵 */
		W5500_TCP_Server_Init(5000);               /* 3. Socket0 TCP Server */

		g_net_mode = NET_MODE_TCP_SERVER;          /* 启动 TCP 命令解析模式 */
	}

	/* ═══════════════════════════════════════════════════════════
	 * [P0-5] 采样率档位初始化（TIM8 TRGO2）
	 *  TIM8 已由 MX_TIM8_Init 配置为 PSC=0 / TRGO2=UPDATE
	 *  这里设置默认档位 ar 启动计数器, MOE 由采集流程控制
	 * ═══════════════════════════════════════════════════════════ */
	SampleClock_Init();

	/* ═══════════════════════════════════════════════════════════
	 * 采集子系统初始化（命令-应答式单次采集）
	 *  三路 ADC 由 TIM8 的 UPDATE / CC1 / OC2REF 分别触发,
	 *  相位 0 / 1/3 / 2/3; 上位机下发 CAPTURE 后采 N 点再回传。
	 * ═══════════════════════════════════════════════════════════ */
	{
		ScopeAcq_Init();
		Debug_Print("[ACQ] 3-phase interleaved acquisition ready\r\n");
		/* 上电自检: 顺便用 DWT 周期数反算真实采样率, 能查出"触发被丢一半" */
		(void)ScopeAcq_SelfTest();
		/* 参数备份区: 把 W25Q16 里的 eq/config/calib/excite blob 读进 RAM，
		   上位机连上后用 PARAM_INFO/PARAM_LOAD 取回（重刷系统也不丢标定）。 */
		{
			uint8_t  wid[3] = {0, 0, 0};
			uint32_t plen;
			char     pdbg[96];
			W25Qxx_ReadID(wid);   /* 认片：EF 40 15 = W25Q16 */
			plen = ParamStore_Init();
			snprintf(pdbg, sizeof(pdbg),
			         "[PARAM] w25q %02X %02X %02X, blob = %lu bytes\r\n",
			         wid[0], wid[1], wid[2], (unsigned long)plen);
			Debug_Print(pdbg);
			{
				uint32_t lseq = LogStore_Init();
				snprintf(pdbg, sizeof(pdbg),
				         "[LOG] store ready, seq = %lu\r\n",
				         (unsigned long)lseq);
				Debug_Print(pdbg);
			}
		}
		/* 时钟探针（默认关闭，见 scope_acq.h 的 SCOPE_BOOT_PROBE）:
		   扫 P=4..16 打印实测采样率，用于核对 ADC 时钟树。 */
#if SCOPE_BOOT_PROBE
		ScopeAcq_ClockProbe();
#endif
	}

	g_system_ready = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ── 网络轮询：每 1 ms 一次 ──────────────────────────────────
     *  命令解析、采集完成回传、Socket0 保活都在 W5500_Poll() 内完成。
     *  采集本身由 TIM8 + DMA 自主完成, CPU 不参与搬运。
     * ──────────────────────────────────────────────────────────── */
    {
        static uint32_t last_poll_ms = 0;
        uint32_t now = HAL_GetTick();
        if (now != last_poll_ms)
        {
            last_poll_ms = now;
            W5500_Poll();
        }
    }

    /* ── Ring Buffer 积压包重试发送（非阻塞） ── */
    w5500_udp_retry_pending();

    /* ── ADC 状态监控（每 5 秒打印一次，3 路差分） ── */
    {
        static uint32_t last_adc_tick = 0;
        if (HAL_GetTick() - last_adc_tick >= 5000) {
            /* 三路 ADC 各 1 个差分通道, 每个样本就是一路差分值
             * 12-bit 差分: 二进制补码, 范围 -2047..+2047 */
            uint16_t tmp[3] = {0, 0, 0};
            int16_t d1 = 0, d2 = 0, d3 = 0;
            if (ScopeAcq_Total() >= 3u)
            {
                (void)ScopeAcq_Fill(0, tmp, 3);
                d1 = (int16_t)tmp[0];   /* ADC1 = CH1 (PA0/PA1) */
                d2 = (int16_t)tmp[1];   /* ADC2 = CH2 (PA6/PA7) */
                d3 = (int16_t)tmp[2];   /* ADC5 = CH3 (PA8/PA9) */
            }
            uint32_t sent = 0, drop = 0;
            uint16_t pend = 0;
            w5500_burst_stats(&sent, &drop, &pend);

            uint8_t  tcp_sr    = W5500_TcpState();
            uint8_t  peer[4]   = {0, 0, 0, 0};
            uint16_t peer_port = 0;
            W5500_TcpPeer(peer, &peer_port);

            char dbg[288];
            snprintf(dbg, sizeof(dbg),
                "[ADC] CH1:%6d CH2:%6d CH3:%6d | fs=%lu gear=%u | TCP sr=%02X peer=%u.%u.%u.%u:%u\r\n"
                "[TX ] sent=%lu drop=%lu pend=%u\r\n",
                (int)d1, (int)d2, (int)d3,
                (unsigned long)g_fs_hz, (unsigned)g_fs_index,
                (unsigned)tcp_sr,
                (unsigned)peer[0], (unsigned)peer[1], (unsigned)peer[2], (unsigned)peer[3],
                (unsigned)peer_port,
                (unsigned long)sent, (unsigned long)drop, (unsigned)pend);
            Debug_Print(dbg);
            last_adc_tick = HAL_GetTick();
        }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;   /* [CLK] 340/6 = 56.67 MHz -> ADC 异步时钟 (VCO=340MHz) */
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* 半主机禁用已移至 retarget.c */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */