# 验收测试

全部在**树莓派**上跑（DG4202 插在派上，采集板在 `192.168.57.200:5000`）。

## 前置

1. 采集板在线（`192.168.57.200:5000` 可连）
2. 采集板同一时刻只服务一个连接：测试脚本会先 `pkill -f '[l]sa_touch.py'`，运行中也会反复清理
3. DG4202 走 `/dev/usbtmc0`，需要 udev 规则（见仓库根 README）

## 脚本

| 文件 | 内容 |
|---|---|
| `test_matrix10.py` | 10 例赛题用例（题库原样，含谐波比例与干扰项） |
| `test_sweep160.py` | 160 点波形扫描：4 波形 × 8 频率（10k…500k）× 5 幅度（50…250 mVpp），`--quick` 为子集 |
| `test_phase.py` | 相位专项：2/3/4 次谐波，最高次相位 0/90/180/270 |
| `test_phase_check.py` | 相位核对（旧版，支持 AFG / DG） |
| `test_dg_selftest.py` | DG4202 驱动离线自检：stub 传输、50 项断言，不需要硬件 |
| `test_fulltest.py` | 原始全测试（`--src {afg,dg}`） |
| `test_matrix_run.py` | 批量矩阵运行器 |
| `test_gap.py` / `test_phase_honor.py` | 单点排障工具 |

## 用法

```bash
cd ~/lsa
python3 test/test_matrix10.py        # 10 例验收
python3 test/test_sweep160.py --quick
python3 test/test_phase.py
python3 test/test_dg_selftest.py     # 离线自检，秒级
```

## 判据与结果

判据：`|ΔUpp| ≤ 5 mV`、`|ΔUrms| ≤ 5 mV`、`|Δf1| ≤ 1 kHz`、逐分量 `≤ 5 mV`。

`test_matrix10.py` 在新树莓派 + DG4202 上 **10/10 通过**：

| 用例 | 模式 | ΔUpp (mV) | ΔUrms (mV) |
|---|---|---:|---:|
| 题1-A | 谐波 | +0.20 | −0.15 |
| 题1-B | 谐波 | +0.12 | −0.03 |
| 题1-C | 谐波 | +1.63 | +0.03 |
| 题1-D | 谐波 | +1.21 | +0.05 |
| 题2-A | 谐波 | −4.36 | −0.97 |
| 题2-B | 谐波 | −0.70 | −0.15 |
| 题2-C | 谐波 | −4.02 | −1.14 |
| 题2-D | ARB | −0.48 | −0.11 |
| 题3-A | ARB | −2.91 | −0.50 |
| 题3-B | ARB | +1.22 | −0.18 |

`results/` 里是历史数据：`sweep.csv`（扫频标定原始数据）、`full_test_afg.csv`（AFG 时期全测试记录）。
