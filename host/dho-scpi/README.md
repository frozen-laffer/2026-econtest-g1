# dho-scpi — RIGOL DHO800/900 示波器 LAN SCPI 驱动

DHO812（固件 00.01.03）实战打磨出来的**纯标准库**驱动：不开 NI-VISA、不装 pyvisa，
只要能 TCP 到示波器的 5555 端口就能读波形、取深存储、算 SFDR/THD。

```
dho-scpi/
├─ dho_scope.py          # 驱动本体：SocketTransport + Scope + FFT/CSV 工具 + CLI
├─ find_scope.py         # 发现设备：UDP 触发 ARP → 读 arp 表 → 逐个试 5555 端口
├─ examples/
│  └─ bode_capture.py    # 用法示例：阶跃响应/扫频记录，边采边落 CSV
└─ README.md
```

## 为什么不用 VISA

DHO812 的 USB 口是 USBTMC，但本机只有类驱动（`ausbtmc` / `oem49.inf`），
没有 NI-VISA 运行时，pyvisa 与 libusb 都起不来 —— 直接 `CreateFile` 打得开、
写得进，读永远不返回。所以统一走**以太网 raw socket**（Rigol 默认端口 5555）。

## 准备

1. 给接示波器的那块网卡配同网段静态 IP（例：`192.168.1.50/24`）。
2. 示波器面板 `Utility → IO → LAN`：设固定 IP（例 `192.168.1.101`）、关 DHCP。
3. 别用 ping 判断通不通 —— 沙箱里 ICMP 常被挡。用 `find_scope.py` 或 `scan`。

```bash
python find_scope.py --subnets 192.168.1        # UDP 触发 ARP，列出活着的 5555
python dho_scope.py scan 192.168.1              # 纯 TCP 扫端口（并发 256）
```

## 命令行

```bash
export SCOPE_HOST=192.168.1.101        # 或者每条命令都带 --host

python dho_scope.py idn                        # *IDN? 认证
python dho_scope.py status                     # 时基/通道/触发状态
python dho_scope.py cmd ":MEASure:VPP? CHANnel1"
python dho_scope.py wav 1 -o ch1.csv           # 存屏幕波形（t, V）
python dho_scope.py fft 1 --fund 1e6           # 算 SFDR/THD 并过门限
python dho_scope.py setup-sine 1 --vdiv 2 --tdiv 200n
```

`fft` 的验收门限可用 `--gate-sfdr 45 --gate-thd -40` 调整（默认沿用板级验收值）。

## 作为库使用

```python
from dho_scope import SocketTransport, Scope

with SocketTransport("192.168.1.101") as t:      # timeout 默认 5 s
    sc = Scope(t)
    print(sc.idn())
    sc.channel(1, scale=2.0, offset=0.0, coupling="DC", probe=1.0, on=True)
    xs, ys = sc.waveform(1)                      # 屏幕帧
    xs, ys, fs = sc.waveform_deep(1, points=200000)   # 深存储 RAW
    print(len(ys), "pts, fs =", fs)
```

主要方法：

| 方法 | 作用 |
|---|---|
| `SocketTransport(host, port=5555, timeout=5.0)` | TCP SCPI 传输层，`query` / `write` / `query_binary` / `drain` |
| `Scope.idn()` | 认证，返回 `RIGOL TECHNOLOGIES,DHO812,<SN>,<FW>` |
| `Scope.run() / stop() / single()` | 采集控制（注意下面第 2 条坑） |
| `Scope.recover(scale=2.0)` | 采集空了（POINts=0）时把仪器拉回可测状态 |
| `Scope.channel(ch, scale, offset, coupling, probe, on)` | 通道设置 |
| `Scope.timebase(sec_per_div)` | 时基 |
| `Scope.sample_rate()` | 从时基反推采样率 |
| `Scope.status()` | 时基/通道/触发的字典快照 |
| `Scope.waveform(ch)` | 屏幕波形 → `(xs, ys)` |
| `Scope.waveform_deep(ch, points=200000, chunk=..., reuse=True)` | 深存储 RAW → `(xs, ys, fs)` |
| `save_csv(path, xs, ys, fs)` / `welch_like_fft(ys, fs)` / `report_spectrum(...)` | 落盘与频谱判据 |

## 取波形的正确顺序（错一步就是「电压恒定不变」）

```
drain()                         # 清掉上次残留的半截二进制
:TRIGger:SWEep AUTO             # 直流没有边沿，NORMal 会永远等
:WAVeform:MODE NORMal           # RAW 在运行中读会阻塞
:RUN → sleep(max(12*时基, 0.15)) → :STOP
等 :TRIGger:STATus? 变 STOP
:WAVeform:POINts? 必须 > 0      # 0 = 没有帧，后面全白做
:WAVeform:STARt 1 ; :STOP N     # 必须显式设，默认值是 0
:WAVeform:FORMat BYTE
:WAVeform:DATA?                 # 回 #<d><len><二进制>
```

### 踩过的坑（细节见 `SKILL` 文档）

1. `POINts?` 为 0 时 `DATA?` 回空块或永久阻塞 —— 取数前必须查。
2. 直接 `:STOP` 会**冻结上一帧**：之后每次测量都是旧数据，看起来像「换码值没反应」。
   永远先 `RUN` 采一帧新的再 `STOP`。
3. `*OPC?` 在这台固件上立刻回 `"0"`，不能当采集完成屏障。
4. `:SINGle` 在直流/无触发时永不完成。
5. 中断过的读取会留半截二进制，之后每个查询错位 —— 必须 `drain()`。
6. 别按对方宣告的长度精确读，用宽容读再按头部裁剪。
7. `:WAVeform:MODE RAW` 只在停止后可用。
8. `:RUNState?` 回空串，判运行状态用 `:TRIGger:STATus?`。
9. `:MEASure:VAVerage?` 回空，平均值自己从数据算。
10. 312.5 MSa/s 时 156.25 MHz 有 −44 dBc 采样器馈通，测频谱先 `:CHANnel1:BWLimit 20M`。
11. FFT 窗长必须等于变换长度，否则造出 −32 dBc 假裙边（曾把 SFDR 从 63 dBc 误判成 32 dBc）。
12. 卡死时先 `AUTO + RUN/STOP` 软恢复，不行再 `*RST`（会清通道设置）。

## 依赖

- 运行时：Python 3.8+，**只用标准库**。
- 可选：`numpy` —— 有则用 `rfft`，没有走内置纯 Python FFT（慢但结果一致）。

## 平台说明

- `find_scope.py` 解析 Windows 的 `arp -a` 输出，Linux/macOS 上改用
  `dho_scope.py scan <网段>`。
- 目前只覆盖 RIGOL DHO800/900 的 raw socket；VXI-11 未实现。
- 频谱参考值（AN9767 + 100 MSPS 前传，1 MHz 单音）：SFDR 63.6 dBc、THD −60.7 dBc。
