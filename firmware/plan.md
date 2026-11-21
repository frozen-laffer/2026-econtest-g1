# 📋 设计方案书：Three_New 高性能数据采集与显示控制系统

**MCU**: STM32G474RET6 (Cortex-M4F, 170MHz)
**日期**: 2026-06-26
**状态**: Phase 3 推进 — TCP/UDP 双模式完成，层级整理完成，烧录配置修复

---

## 目录

1. [已完成概要](#1-已完成概要)
2. [W5500 UDP 模式开发](#2-w5500-udp-模式开发)
3. [待完成路线图](#3-待完成路线图)
4. [已知问题与风险清单](#4-已知问题与风险清单)

---

## 1. 已完成概要

### 1.1 基础设施修复 ✅

| 项目 | 状态 |
|------|------|
| `ILI9488.h` 引脚修正 (CS→PB12, DC→PB10, RST→PB2) | ✅ |
| `w5500` CS 引脚修正 (PD3→PA15/SPI3_CS) | ✅ |
| `ENC2_SW` (PC2) 改为 Input + PULLUP | ✅ |
| AFE 驱动重写 — CH1/CH2/CH3 + `AFE_SetChannelMode()` API | ✅ |

### 1.2 数据采集 Pipeline ✅

| 项目 | 状态 |
|------|------|
| `adc_driver.c` 重构 — 移除 extern DMA 句柄 + 新增 ADC5 (DMA1_Ch3) | ✅ |
| 过采样 OVSR=16 + OVSS=2 → 14-bit 有效位 | ✅ |
| Bug#1-#4 全部修复 | ✅ |

### 1.3 W5500 TCP Server ✅

| 项目 | 状态 |
|------|------|
| W5500 驱动架构整理 — `Drivers/W5500/` 为唯一权威目录，删除 16 个冗余文件 | ✅ |
| SPI3 8-bit ~5.3MHz Mode 0, PA15 软件 CS | ✅ |
| VERSIONR=0x04 硬件验证通过 | ✅ |
| TCP Server (Socket 0, Port 5000, Echo) 调通 | ✅ |
| Sn_SR 状态机自然流转 (CLOSED→INIT→LISTEN→ESTABLISHED) | ✅ |
| SPI 临界区 `__disable_irq()/__enable_irq()` | ✅ |
| SPI 写入后 NOP 延时 + listen() CR 等待机制 | ✅ |

---

## 2. W5500 UDP 模式开发

### 2.1 总体目标

在现有 TCP Server 框架基础上增加 UDP 模式支持。UDP 用于高吞吐 ADC 数据流推送，无连接、低延迟、不阻塞主循环。

### 2.2 UDP Socket 状态机

```
SOCK_CLOSED → socket(0, Sn_MR_UDP, port, 0) → SOCK_UDP (0x22)
                                                  ↓
                                            直接 sendto() 收发
                                                  ↓
                              关闭时 close(0) → SOCK_CLOSED
```

与 TCP 不同：UDP 无 LISTEN/ESTABLISHED 握手，socket() 成功后直接进入 SOCK_UDP (0x22) 即可发送。

### 2.3 双模式切换架构

```c
typedef enum {
    NET_MODE_TCP_SERVER = 0,
    NET_MODE_UDP        = 1
} NetMode_t;

extern volatile NetMode_t g_net_mode;  // 定义在 main.c 中，w5500_network.h 中 extern 声明
```

- 通过 `g_net_mode` 切换模式
- `W5500_Network_Init()` 中根据模式初始化不同协议
- `W5500_TCP_Echo_Poll()` 改为 `W5500_Poll()`，分支处理 TCP/UDP

### 2.4 待实现文件修改

| 文件 | 修改 |
|------|------|
| `w5500_network.c` | 新增 `W5500_UDP_Init(port)` / UDP 发送/接收 API / `W5500_Poll()` 改名扩展 |
| `w5500_network.h` | 新增 UDP 函数声明 |
| `main.c` | 按需调用 UDP 初始化 + 轮询 |

### 2.5 UDP 数据推送流程

```
ADC DMA 完成中断 → 数据 ready
                     ↓
主循环 W5500_Poll()
  → 检查 g_net_mode == NET_MODE_UDP
  → getSn_SR(0) == SOCK_UDP
  → sendto(0, adc_buf, len, remote_ip, remote_port)
```

---

## 3. 待完成路线图

### Phase 2: 数据采集 Pipeline（部分待完成）

- [x] P2.1 重构 `adc_driver.c`
- [ ] **P2.2** 实现 `scope_pipeline.c` — 数据流主循环调度

### Phase 3: 网络与存储（进行中）

- [x] P3.1 TCP Server 调通 (Port 5000, Echo)
- [x] **P3.1-UDP** UDP 模式开发完成
- [x] **P3.1-CLEAN** 架构层级整理完成（删除 w5500_self 死代码，IP/mode 外提至 main.c）
- [ ] **P3.2** W25Q16 非阻塞写入状态机

### Phase 4: 后续增强

- [ ] 示波器功能优化（ft6336.c 死循环重构）
- [ ] TIM2 PWM 参数调整（Prescaler=0, Period=0xFFFFFFFF）
- [ ] FT6336 旋转映射完善
- [ ] DHCP 客户端启用

---

## 4. 已知问题与风险清单

| # | 问题 | 严重度 | 状态 |
|---|------|--------|------|
| 1 | ~~`ILI9488.h` 引脚不匹配~~ | — | ✅ |
| 2 | ~~`w5500.c` CS 硬编码 PD3~~ | — | ✅ |
| 3 | ~~`ENC2_SW` PC2 OUTPUT~~ | — | ✅ |
| 4 | ~~AFE 旧驱动引脚全部失效~~ | — | ✅ |
| 5 | ~~ADC DMA 需运行时重配为 Circular~~ | — | ✅ |
| 6 | TIM2 Prescaler=0, Period=0xFFFFFFFF 需调整 PWM | 🟡 中 | 待适配 |
| 7 | W25Q16 阻塞式写入可能丢 DMA 中断 | 🟡 中 | 改为非阻塞 |
| 8 | `ft6336.c` 示波器死循环，架构混乱 | 🟡 中 | 待重构 |
| 9 | FT6336 旋转映射硬编码 270° | 🟢 低 | 待完善 |
| 10 | ~~W5500 TCP RST 根因 (状态机 CLOSED 死循环)~~ | — | ✅ 已修复 |
| 11 | OpenOCD 烧录后 CPU halted on breakpoint | 🟡 中 | ✅ 已修复 (eide.yml 添加 `reset run`) |

---

> **当前焦点**：§5 200kHz 3 通道高速 ADC → W5500 UDP 数据泵

---

## 5. 200kHz 3 通道高速 ADC → W5500 UDP 数据泵（新增）

### 5.1 硬件配置

| 外设 | 配置 | 状态 |
|------|------|------|
| TIM8 @ APB2 170MHz | PSC=0, ARR=849 → 200kHz TRGO2_UPDATE | ✅ CubeMX 已生成 |
| ADC1 | Continuous + T8_TRGO2 触发 + DMA1_Ch1 | ✅ CubeMX 已生成 |
| ADC2 | 同上 + DMA1_Ch2 | ✅ |
| ADC5 | 同上 + DMA1_Ch3 | ✅ |
| W5500 Socket 0 | TCP/UDP 控制 | ✅ |
| W5500 Socket 1 | UDP 专用于数据流 | ⏳ 代码待写 |

### 5.2 数据流动

```
TIM8_TRGO2 (200kHz)
  ├→ ADC1(DMA1_Ch1) ─┐
  ├→ ADC2(DMA1_Ch2) ─┼→ Ping-Pong Buffer[3ch × HALF_BUF × 2]
  └→ ADC5(DMA1_Ch3) ─┘      ↓
                         HT/TC ISR → g_buf_ready flag
                              ↓
                         主循环 → SPI Burst → W5500 Socket 1 UDP sendto()
```

### 5.3 性能预算

| 指标 | 值 |
|------|-----|
| 采样率 | 200kHz × 3ch |
| 数据率 | 1.2 MB/s |
| HALF_BUF_SIZE | 512 |
| 每包大小 | 3072 Bytes |
| 发包速率 | ~390 包/秒 |
| SPI 传输耗时 | ~0.58ms/包 |
| SPI 带宽占用 | ~23% |

### 5.4 实施阶段

| Phase | 文件 | 修改 | 类型 |
|-------|------|------|------|
| A+B | CubeMX | TIM8 (200kHz TRGO2) + ADC1/2/5 (T8_TRGO2 触发) | ✅ 已完成 |
| C | dma.c | USER CODE 中重配 hdma_adc1/2/5 为 DMA_CIRCULAR | 代码 |
| D | main.c | Ping-Pong 双缓冲 + g_buf_ready flag + TIM8/ADC 启动 + 主循环推送调度 | 代码 |
| E | stm32g4xx_it.c | HT/TC 回调 3 路计数满后置 g_buf_ready |= 0x01/0x02 | 代码 |
| F | w5500_network.c/h | Socket 1 UDP 初始化 + `w5500_udp_stream_push()` 零拷贝 SPI Burst | 代码 |
| G | — | 编译验证 | — |
