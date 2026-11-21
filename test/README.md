# 验收测试

四套脚本，都在**树莓派上跑**（DG4202 插在派的 USB 上，采集板在 `192.168.57.200:5000`）。

## 前置

1. 采集板在线：`Test-NetConnection 192.168.57.200 -Port 5000`（PC）或 `nc -z 192.168.57.200 5000`（派）
2. **板子固件同一时刻只服务一个连接**：`lsa_touch.py` 开着会占住 socket，所有测试脚本都会先 `pkill -f '[l]sa_touch.py'`，并在跑的过程中用守护线程反复清理
3. DG4202 走 `/dev/usbtmc0`，需 udev 规则（见仓库根 README）

## 脚本

| 文件 | 内容 | 判据 |
|---|---|---|
| `test_matrix10.py` | **10 例赛题用例**（题库原样，含谐波比例与干扰项） | `\|ΔUpp\|≤5 mV`、`\|ΔUrms\|≤5 mV`、`\|Δf1\|≤1 kHz`、逐分量 `≤5 mV` |
| `test_sweep160.py` | **160 点波形扫描**：4 种波形 × 8 频率（10k…500k）× 5 幅度（50…250 mVpp） | 同上；`--quick` 为 3 波形 × 3 频率 × 2 幅度 |
| `test_phase.py` | **相位专项**：2/3/4 次谐波、基波 50k…250k、比例 0.60/0.60/0.40、最高次相位 0/90/180/270 | `\|ΔUpp\|≤5 mV` |
| `test_phase_check.py` | 相位核对（旧版，AFG/DG 双支持） | 同上 |
| `test_dg_selftest.py` | **DG4202 驱动离线自检**（stub 传输，50 项断言，不需要硬件） | 全部断言通过 |
| `test_fulltest.py` | 原始全测试（AFG 路径，`--src {afg,dg}`） | 同上 |
| `test_matrix_run.py` | 批量矩阵运行器 | — |
| `test_gap.py` / `test_phase_honor.py` | 单点排障工具（间隙测试 / 相位是否被严格执行） | — |

## 用法

```bash
cd ~/lsa
python3 test/test_matrix10.py                     # 10 例，约 6 分钟
python3 test/test_sweep160.py --quick             # 160 点扫描的快速子集
python3 test/test_sweep160.py                     # 全量 160 点，约 50 分钟
python3 test/test_phase.py                        # 相位专项
python3 test/test_dg_selftest.py                  # 离线自检，秒级
```

## 最近一次结果

`test_matrix10.py`：**10/10 通过**（新树莓派 + DG4202，2026-11-21）

| 用例 | 路数 | ΔUpp (mV) | ΔUrms (mV) |
|---|---|---|---|
| 题1-A | 谐波 29296.9 Hz | +0.20 | −0.15 |
| 题1-B | 谐波 | +0.12 | −0.03 |
| 题1-C | 谐波 58593.75 Hz | +1.63 | +0.03 |
| 题1-D | 谐波 | +1.21 | +0.05 |
| 题2-A | 谐波 117187.5 Hz | −4.36 | −0.97 |
| 题2-B | 谐波 | −0.70 | −0.15 |
| 题2-C | 谐波 234375 Hz | −4.02 | −1.14 |
| 题2-D | ARB（17 次超谐波上限） | −0.48 | −0.11 |
| 题3-A | ARB（带 41×F0 干扰） | −2.91 | −0.50 |
| 题3-B | ARB（带 35×F0 干扰） | +1.22 | −0.18 |

`test_sweep160.py` 首轮（sine/square/triangle 120 点）：**111 通过 / 9 失败**，失败全在 50 mV 档且 `f1` 被误认成 593–632 kHz——那根固定约 600 kHz 的仪器残留被前端补偿放大所致，已通过三个测量开关（`LSA_EQ_FHI_HZ` / `LSA_EQ_LP_HZ` / `LSA_FIT_RESID_MAX`，见 `docs/06`）与半波路径修正，未复跑全量。

`results/` 里是历史数据：`sweep.csv`（扫频标定原始数据）、`full_test_afg.csv`（AFG 时期的全测试记录）。
