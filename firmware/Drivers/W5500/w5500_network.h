#ifndef __W5500_NETWORK_H
#define __W5500_NETWORK_H

#include "main.h"
#include "Ethernet/wizchip_conf.h"
#include "Ethernet/socket.h"
#include <stdint.h>

/* ═══════════════════════════════════════════════════════════
 * TCP 控制协议 + 统一数据帧格式 (V2.7)
 * TCP/串口使用此帧格式，UDP 保持原有简单格式
 * ═══════════════════════════════════════════════════════════ */

/* ── 网络模式枚举 ── */
typedef enum {
    NET_MODE_UDP        = 0,
    NET_MODE_TCP_SERVER = 1
} NetMode_t;

extern volatile NetMode_t g_net_mode;

/* ── 采集配置结构体（全局，被 main.c/stm32g4xx_it.c 读取） ── */
typedef struct {
    uint8_t  ch_mask;       /* bit0=CH1, bit1=CH2, bit2=CH3 */
    uint8_t  resolution;    /* 8/10/12, 默认12 */
    uint32_t sample_rate;   /* Hz, 默认200000 */
    uint8_t  running;       /* 0=stopped, 1=running */
} CaptureConfig_t;

extern volatile CaptureConfig_t g_cap_cfg;

/* ── TCP 命令码 ── */
#define CMD_CONFIGURE   0x01
#define CMD_START       0x02
#define CMD_STOP        0x03
#define CMD_STATUS      0x04
#define CMD_SINGLE      0x05

/* ── MACRO: 通道掩码 → 有效通道计数 ── */
static inline uint8_t ch_count(uint8_t mask) {
    uint8_t c = 0;
    if (mask & 0x01) c++;
    if (mask & 0x02) c++;
    if (mask & 0x04) c++;
    return c;
}

/* ── 统一数据帧结构体 ── */

#define FRAME_MAGIC     0x55AA
#define FRAME_TYPE_TCP  0x01
#define FRAME_TYPE_UART 0x02
#define FRAME_HDR_LEN   12

#pragma pack(1)
typedef struct {
    uint16_t magic;           /* 0x55AA 帧同步 */
    uint8_t  frame_type;      /* 0x01=TCP, 0x02=UART */
    uint8_t  flags;           /* bit0~2=CH1~3_en, bit4=half */
    uint16_t seq;             /* 帧序列号 */
    uint16_t samples;         /* 每通道样点数 */
    uint16_t sample_rate;     /* 当前采样率 Hz */
    uint8_t  resolution;      /* 当前分辨率 8/10/12 */
    uint8_t  ch_mask;         /* 通道掩码 */
    uint8_t  checksum;        /* 以上所有字节累加低8位 */
    /* 数据体跟在后面: CH1_DATA[2×samples] CH2_DATA[2×samples] CH3_DATA[2×samples] */
} DataFrame_t;
#pragma pack()

/* ── TCP 命令包 (8字节) ── */
#pragma pack(1)
typedef struct {
    uint8_t  cmd;
    uint8_t  ch_mask;
    uint8_t  resolution;
    uint16_t sample_rate;
    uint8_t  action;
    uint8_t  rsv;
    uint8_t  checksum;
} TCmdPacket_t;
#pragma pack()

/* ── TCP 状态回传包 (8字节) ── */
#pragma pack(1)
typedef struct {
    uint8_t  rsp;             /* 0x81 */
    uint8_t  status;          /* 0=idle, 1=running */
    uint8_t  ch_mask;
    uint8_t  resolution;
    uint16_t sample_rate;
    uint8_t  error;
    uint8_t  checksum;
} TStaPacket_t;
#pragma pack()

#ifdef __cplusplus
extern "C" {
#endif

/* ── 网络配置（调用 ioLibrary ctlnetwork 初始化，参数由 main.c 传入） ── */
void W5500_Network_Init(wiz_NetInfo *netconfig);

/* ── TCP Server 初始化（socket + listen） ── */
int W5500_TCP_Server_Init(uint16_t port);

/* ── UDP 模式初始化 ── */
int W5500_UDP_Init(uint16_t local_port);

/* ── 统一轮询（TCP+UDP 双模式，按 g_net_mode 分支） ── */
void W5500_Poll(void);

/* ── UDP 数据收发 ── */
int32_t W5500_UDP_SendTo(const uint8_t *data, uint16_t len, uint8_t *remote_ip, uint16_t remote_port);
int32_t W5500_UDP_RecvFrom(uint8_t *data, uint16_t max_len, uint8_t *remote_ip, uint16_t *remote_port);

/* ── 兼容层（TCP 模式） ── */
int  W5500_Net_SendData(const uint8_t *data, uint16_t len);
int  W5500_Net_RecvData(uint8_t *data, uint16_t len);
int  W5500_Net_IsConnected(void);

/* ── 采集模式枚举 ── */
typedef enum {
    CAPTURE_MODE_PURE_UDP     = 0,  /* 纯高速 UDP 数据泵（默认） */
    CAPTURE_MODE_TCP_RECALL   = 1   /* TCP 可靠回传（未来） */
} CaptureMode_t;

extern volatile uint8_t g_capture_mode;

/* ── 高速数据流 UDP 推送（Socket 1，专用） ── */
int W5500_HighSpeed_UDP_Init(uint16_t local_port, uint8_t *remote_ip, uint16_t remote_port);
void w5500_udp_stream_push(uint8_t half_sel, uint16_t *adc0_buf, uint16_t *adc1_buf, uint16_t *adc2_buf);
void w5500_udp_retry_pending(void);

/* ═══════════════════════════════════════════════════════════════
 * [P0-7] 突发采集传输 —— 22 字节新包头
 *
 *  与旧的 6 字节轻量包头不同, 新包头携带:
 *    magic       'LSMT' 用于格式识别
 *    sample_idx  绝对样本序号 ★关键★ 上位机据此拼帧与检测丢包
 *    fs_khz      实际采样率, 上位机无需配置
 *    gain_code   增益档, 用于自动幅度标定
 *    crc16       CRC-16/CCITT 防止 UDP 静默损坏
 *
 *  字节序: 小端。载荷为 int16 交错: [s0_c1,s0_c2,s0_c3, s1_c1, ...]
 *  详见《技术方案与实施细节总文档》§4.1
 * ═══════════════════════════════════════════════════════════════ */

#define HS2_MAGIC        0x4C534D54UL   /* 'LSMT' */
#define HS2_HEADER_LEN   22

/* 增益档编码 */
#define HS2_GAIN_X2      0u             /* ×2.00 档 */
#define HS2_GAIN_X20     1u             /* ×20.16 档 */

/* flags 位定义 */
#define HS2_FLAG_AC      0x01           /* AC 耦合 */
#define HS2_FLAG_TRIG    0x02           /* 触发点在本包 */
#define HS2_FLAG_SINGLE  0x04           /* 单次采集 */

/**
 * @brief  发送一次突发采集数据（三通道, 带 22 字节包头）
 * @param  adc0/adc1/adc2  三通道缓冲区基址（int16 差分码）
 * @param  count           每通道样本数（三路必须相同）
 * @param  ch_mask         通道掩码 bit0..2
 * @param  gain_code       HS2_GAIN_X2 / HS2_GAIN_X20
 * @param  flags           HS2_FLAG_*
 * @retval 实际入队的分段数; <0 表示失败
 * @note   非阻塞: 数据分段落入环形缓冲, 由 w5500_udp_retry_pending() 排空
 *         必须在采集停止后调用（缓冲不再被 DMA 改写）
 */
int w5500_burst_send(const uint16_t *adc0, const uint16_t *adc1, const uint16_t *adc2,
                     uint16_t count, uint8_t ch_mask, uint8_t gain_code, uint8_t flags);

/**
 * @brief  等待环形缓冲排空（阻塞, 带上限）
 * @param  timeout_ms  超时毫秒
 * @retval 0=已排空, -1=超时仍有积压
 */
int w5500_burst_flush(uint32_t timeout_ms);

/**
 * @brief  查询发送统计
 * @param  sent   输出: 已发送包数（可为 NULL）
 * @param  drop   输出: 溢出丢弃包数（可为 NULL）
 * @param  pend   输出: 当前积压包数（可为 NULL）
 */
void w5500_burst_stats(uint32_t *sent, uint32_t *drop, uint16_t *pend);

/* ═══════════════════════════════════════════════════════════════
 * 命令-应答式单次采集协议（上位机 / 树莓派 4）
 *
 *   命令 16 字节 (小端):
 *     0  u32 magic  = 'LSCO'
 *     4  u8  cmd
 *     5  u8  a0
 *     6  u8  a1
 *     7  u8  a2
 *     8  u8  a3
 *     9  u8  rsv
 *    10  u16 n      一次采集的合并总点数
 *    12  u32 tok    会话令牌, 原样回传
 *
 *   回复 32 字节头 + 载荷 int16:
 *     0  u32 magic  = 'LSDA'
 *     4  u8  ver    = 1
 *     5  u8  type   0=ACK 1=DATA 2=ERR
 *     6  u8  gear
 *     7  u8  mode   0=单路 1=三路交替
 *     8  u32 fs_hz  实际采样率（三路时已 ×3）
 *    12  u32 n_total
 *    16  u32 n_this （ACK/ERR 时为 0）
 *    20  u32 gain_milli
 *    24  u32 tok
 *    28  u16 flags
 *    30  u16 crc16  覆盖 0..29
 * ═══════════════════════════════════════════════════════════════ */
#define LCMD_MAGIC       0x4F43534CUL   /* 字节 'L','S','C','O' */
#define LSDA_MAGIC       0x4144534CUL   /* 字节 'L','S','D','A' */
#define LCMD_LEN         16u
#define LSDA_HDR_LEN     32u

/* 命令码 */
#define LCMD_PING        0x01u
#define LCMD_CAPTURE     0x02u
#define LCMD_STATUS      0x03u
#define LCMD_ABORT       0x04u
#define LCMD_SET_AFE     0x05u
#define LCMD_SELFTEST    0x06u
/* 参数备份区（W25Q16 @ SPI1，见 Drivers/App/param_store.h）
   上位机把 eq.json/config.json/calib.json/excite.json 打成一个 blob，
   用 BEGIN(总长) -> CHUNK(偏移+数据) x N -> COMMIT 三段式写进板子 Flash。 */
#define LCMD_PARAM_BEGIN  0x07u   /* a0..a3 = 总字节数 */ 
#define LCMD_PARAM_CHUNK  0x08u   /* a0..a3 = 偏移，16 字节头之后是数据 */
#define LCMD_PARAM_COMMIT 0x09u   /* 无参：算 CRC、落盘、回读校验 */
#define LCMD_PARAM_LOAD   0x0Au   /* a0..a3 = 偏移，n = 要多少字节 -> 回 DATA */
#define LCMD_PARAM_INFO   0x0Bu   /* 回 DATA 8 字节：len u32 | seq u32 */
#define LCMD_PARAM_CLEAR  0x0Cu   /* 清空两个槽 */

/* 掉电日志环（W25Q16 @ 0x00104000，见 Drivers/App/log_store.h）
   树莓派把运行日志/报错逐条发过来，板子写进自己的 Flash 并同时从调试
   串口打一遍 —— 系统坏了日志也还在。 */
#define LCMD_LOG_PUT      0x0Du   /* 16 字节头之后 = level u8 + 正文, n = 1+正文长 */
#define LCMD_LOG_INFO     0x0Eu   /* 回 DATA 8 字节: seq u32 | 已用扇区 u32 */
#define LCMD_LOG_READ     0x0Fu   /* a0..a3 = from_seq, n = 最多字节数 -> 回文本行 */
#define LCMD_LOG_CLEAR    0x10u   /* 擦掉全部日志 */

/* 回复类型 */
#define LSDA_TYPE_ACK    0u
#define LSDA_TYPE_DATA   1u
#define LSDA_TYPE_ERR    2u

/* 错误码（放在 header 的 type=ERR 时的 a0 位, 这里用 flags 高字节） */
#define LSDA_ERR_NONE    0u
#define LSDA_ERR_PARAM   1u
#define LSDA_ERR_BUSY    2u
#define LSDA_ERR_START   3u
#define LSDA_ERR_TIMEOUT 4u

/* flags 位 */
#define LSDA_FLAG_INTER3_OK   0x0001u    /* 三路交替自检通过 */
#define LSDA_FLAG_ERROR       0x0002u

#pragma pack(1)
typedef struct {
    uint32_t magic;
    uint8_t  ver;
    uint8_t  type;
    uint8_t  gear;
    uint8_t  mode;
    uint32_t fs_hz;
    uint32_t n_total;
    uint32_t n_this;
    uint32_t gain_milli;
    uint32_t tok;
    uint16_t flags;
    uint16_t crc16;
} ScopeHdr_t;
#pragma pack()

/**
 * @brief  阻塞发送一段原始字节到 Socket 0（自动分块）
 * @retval 0=成功, -1=失败
 */
int W5500_SendRaw(const uint8_t *data, uint32_t len);

/* 调试用：Socket0 状态与对端信息 */
uint8_t  W5500_TcpState(void);
void     W5500_TcpPeer(uint8_t ip[4], uint16_t *port);
void     W5500_HardReset(void);

/* ── 采集模式切换（动态控制 TIM8 MOE，供未来 TCP 指令调用） ── */
void W5500_Switch_Capture_Mode(CaptureMode_t new_mode);

/* ── 向后兼容宏 ── */
#define W5500_TCP_Echo_Poll()  W5500_Poll()

#ifdef __cplusplus
}
#endif

#endif /* __W5500_NETWORK_H */
