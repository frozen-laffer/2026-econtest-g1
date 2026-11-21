# 2026 电赛 G 题 · 方案一

> 信号源 + 四阶有源滤波前端 + STM32G474 三路交替采样 + 树莓派上位机
> 支持双信号源：**GW Instek AFG-2225**（串口）与 **RIGOL DG4202**（USB-TMC，谐波/任意波）

## 这个仓库是什么

一套可复现的「赛题装置 + 测量上位机」：

- **STM32G474 固件**（`firmware/`）：ADC 三路交替采样、AFE 档位控制、W5500 以太网 UDP 传输、Flash 参数存储、TFT/触摸界面
- **树莓派上位机**（`host/`）：波形装载、扫频标定、频响补偿、谐波测量、触摸屏操作界面
- **验收测试**（`test/`）：10 例赛题用例、160 点波形扫描、相位专项
- **文档**（`docs/`）：四卷问答体——硬件 / 固件 / 软件 / 外设驱动，入口 [`docs/00_README.md`](docs/00_README.md)
- **硬件**（`hardware/`）：490 kHz 四阶 Chebyshev 前端设计文件与 BOM

## 关键特性

| 项 | 说明 |
|---|---|
| 采样 | STM32G474 ADC 三路交替，等效采样率随档位切换（`6/8/10/12` 档配置表） |
| 前端 | 全差分二阶 FDA + 二阶 MFB 级联，490 kHz 四阶 Chebyshev（0.5 dB） |
| 传输 | W5500 UDP，10 Mbps 链路 |
| 上位机 | Python + Tk（labwc/pcmanfm 桌面），树莓派上开机自启、桌面图标 |
| 信号源 | AFG-2225 与 DG4202 双支持，抽象层 `lsa_src.py` 自动适配 |
| 谐波 | DG4202 **内置谐波模式优先**（`APPL:HARM` + `HARM:USER` 掩码），超 16 次或带干扰时自动转 ARB 合成 |

## 快速开始

### 树莓派上位机

```bash
# 依赖
pip install --user --break-system-packages numpy matplotlib pillow paramiko pyserial

# 拷贝 host/ 到 ~/lsa 后
cd ~/lsa
python3 lsa_touch.py            # 图形界面
python3 lsa_cli.py --help       # 命令行
python3 dg4202.py --idn         # DG4202 自检（走 /dev/usbtmc0）
```

DG4202 需要一条 udev 规则让普通用户能读写 `/dev/usbtmc0`：

```
SUBSYSTEM=="usbmisc", ATTRS{idVendor}=="1ab1", MODE="0666", GROUP="plugdev"
```

树莓派连接的地址与密码通过环境变量或 `~/.lsa_pi` 提供（见 `host/pi_env.py`）：

```bash
python3 pi_env.py --init          # 交互式写入 ~/.lsa_pi（600）
# 或
export LSA_PI=10.110.75.61 LSA_PI_USER=frozen-laffer LSA_PI_PASS=***
```

### STM32 固件

用 Keil MDK 打开 `firmware/MDK-ARM/Three_New.uvprojx`（器件 STM32G474xx），编译后下载。
CubeMX 工程文件是 `firmware/Three_New.ioc`。

## 验收结果

10 例赛题用例（判据 Upp/Urms/各阶幅度均 ≤5 mV）**10/10 通过**，明细见 [`test/README.md`](test/README.md) 与 [`docs/03_软件.md`](docs/03_软件.md) 的验收表。

## 目录

```
firmware/    STM32G474 固件（自写驱动 + HAL + CMSIS + MDK 工程）
host/        树莓派上位机（30 个模块）
test/        验收测试脚本与结果
docs/        四卷问答体文档（00 索引）
hardware/    前端滤波器设计文件与 BOM
```

## 许可与致谢

本仓库为参赛作品的存档，代码按现状提供。固件依赖 STMicroelectronics HAL 与 CMSIS（各自原始许可见 `firmware/Drivers/` 内头文件）；RIGOL 官方编程手册（有版权）未收录，谐波/ARB 相关结论均为实测所得。
