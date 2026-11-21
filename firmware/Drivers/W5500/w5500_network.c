/**
  * @file    w5500_network.c
  * @brief   TCP/UDP 双模式网络业务层（基于 ioLibrary socket API）
  */
#include "w5500_network.h"
#include "bsp_w5500.h"           /* 必须在 ioLibrary 之前：通过 main.h 拉入 CMSIS 头，防止 #define IR 污染 */
#include "../debug_print/debug_print.h"
#include "../App/sample_clock.h"
#include "../App/scope_acq.h"
#include "../App/param_store.h"
#include "../App/log_store.h"   /* W25Q16 参数备份区 */
#include "../AFE/afe.h"
#include "adc.h"                 /* ADC_ReconfigSingleEnded()（单端调试模式） */
#include <string.h>
#include <stdio.h>

static uint8_t g_eth_initialized = 0;

/* ── 网络模式（定义在 main.c 中，此处 extern 引用） ── */
volatile NetMode_t g_net_mode;

/* ── 全局采集配置 ── */
volatile CaptureConfig_t g_cap_cfg = {
    .ch_mask     = 0x07,       /* CH1+CH2+CH3 */
    .resolution  = 12,         /* 12-bit */
    .sample_rate = 200000,     /* 200kHz */
    .running     = 1           /* 默认启动 */
};

/* ── TCP 回传帧序列号 ── */
static uint16_t g_tcp_seq = 0;

/* ─── UDP 收发 ─── */
static uint8_t calc_cksum(const uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

/* ─── 远程主机信息（UDP 模式默认目标，也可动态指定） ─── */
static uint8_t  g_udp_remote_ip[4]   = {192, 168, 1, 100};
static uint16_t g_udp_remote_port    = 6000;

/* ═══════════════════════════════════════════════════════════════
 * 初始化（网络配置由 main.c 传入）
 * ═══════════════════════════════════════════════════════════════ */

void W5500_Network_Init(wiz_NetInfo *netconfig)
{
    BSP_W5500_Init();
    BSP_W5500_Reset();
    HAL_Delay(10); /* 🔴 给硬件复位电平建立留出 10ms 稳定窗口 */

    /* ── 第2步：裸机 SPI 测试 — 绕过 ioLibrary 直接发 VERSIONR 读取帧 ── */
    {
        char diag[64];
        uint8_t ver;
        BSP_W5500_Select();
        BSP_W5500_SPI_ReadWrite(0x00);  /* Control byte (Read) */
        BSP_W5500_SPI_ReadWrite(0x00);  /* Address High */
        BSP_W5500_SPI_ReadWrite(0x39);  /* Address Low (VERSIONR = 0x0039) */
        ver = BSP_W5500_SPI_ReadWrite(0xFF);  /* Dummy read */
        BSP_W5500_Deselect();
        snprintf(diag, sizeof(diag),
            "[RAW TEST] Manual VERSIONR read = 0x%02X\r\n", ver);
        Debug_Print(diag);
    }

    /* 🔴 诊断：手动写 Socket 0 TX BUF 为 2KB + 立即回读验证 */
    /* TX = 2+8 = 10KB <= 16KB OK,  RX = 2+2 = 4KB <= 16KB OK */
    {
        static uint8_t txbuf[8] = {4, 6, 0, 0, 0, 0, 0, 0};   /* Socket0 TX=4KB(数据回传), Socket1 TX=6KB */
        static uint8_t rxbuf[8] = {2, 2, 0, 0, 0, 0, 0, 0};
        wizchip_init(txbuf, rxbuf);
    }

    /* ── 第1步：最基础 VERSIONR 诊断 + 单字节写读验证 ── */
    {
        char diag[96];
        uint8_t ver = IINCHIP_READ(VERSIONR);
        snprintf(diag, sizeof(diag),
            "[DIAG] VERSIONR read = 0x%02X (MUST be 0x04 if chip OK)\r\n", ver);
        Debug_Print(diag);

    }

    ctlnetwork(CN_SET_NETINFO, (void*)netconfig);

    g_eth_initialized = 1;

    /* 先关闭 Socket 0 */
    {
        char buf[128];
        snprintf(buf, sizeof(buf),
            "[NET] SRAM: S0 TX=%dKB RX=%dKB | S1 TX=%dKB RX=%dKB | IP:%d.%d.%d.%d\r\n",
            getSn_TXBUF_SIZE(0), getSn_RXBUF_SIZE(0),
            getSn_TXBUF_SIZE(1), getSn_RXBUF_SIZE(1),
            netconfig->ip[0], netconfig->ip[1], netconfig->ip[2], netconfig->ip[3]);
        Debug_Print(buf);
    }
}

/* ═══════════════════════════════════════════════════════════════
 * TCP Server 初始化
 * ═══════════════════════════════════════════════════════════════ */

int W5500_TCP_Server_Init(uint16_t port)
{
    int ret;

    if (!g_eth_initialized)
    {
        Debug_Print("[TCP] Network not initialized!\r\n");
        return -1;
    }

    ret = socket(0, Sn_MR_TCP, port, 0);
    if (ret != 0)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "[TCP] socket() failed: %d\r\n", ret);
        Debug_Print(buf);
        return -1;
    }
    Debug_Print("[TCP] Socket created\r\n");

    ret = listen(0);
    if (ret != SOCK_OK)
    {
        uint8_t sr = getSn_SR(0);
        if (sr == SOCK_LISTEN || sr == SOCK_ESTABLISHED)
        {
            char buf[64];
            snprintf(buf, sizeof(buf),
                "[TCP] listen() returned %d but HW is %s (0x%02X) - OK\r\n",
                ret, (sr == SOCK_ESTABLISHED) ? "ESTABLISHED" : "LISTENING", sr);
            Debug_Print(buf);
            return 0;
        }
        else
        {
            char buf[48];
            snprintf(buf, sizeof(buf), "[TCP] listen() failed: %d, Sn_SR=0x%02X\r\n", ret, sr);
            Debug_Print(buf);
            return -1;
        }
    }
    {
        char buf[48];
        snprintf(buf, sizeof(buf), "[TCP] Listening on port %u\r\n", port);
        Debug_Print(buf);
    }
    return 0;
}

/* ═══════════════════════════════════════════════════════════════
 * UDP 初始化
 * ═══════════════════════════════════════════════════════════════ */

int W5500_UDP_Init(uint16_t local_port)
{
    int ret;

    if (!g_eth_initialized)
    {
        Debug_Print("[UDP] Network not initialized!\r\n");
        return -1;
    }

    /* 先关闭 Socket 0 */
    close(0);

    /* 创建 UDP Socket */
    ret = socket(0, Sn_MR_UDP, local_port, 0);
    if (ret != 0)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "[UDP] socket() failed: %d\r\n", ret);
        Debug_Print(buf);
        return -1;
    }

    /* 验证进入 SOCK_UDP (0x22) */
    {
        uint8_t sr = getSn_SR(0);
        char buf[48];
        snprintf(buf, sizeof(buf), "[UDP] Socket open, SR=0x%02X, port=%u\r\n", sr, local_port);
        Debug_Print(buf);

        if (sr != SOCK_UDP)
        {
            Debug_Print("[UDP] WARNING: SR not SOCK_UDP (0x22)\r\n");
            return -1;
        }
    }

    Debug_Print("[UDP] Ready to send/receive\r\n");
    return 0;
}

/* ═══════════════════════════════════════════════════════════════
 * 统一轮询（TCP + UDP 双模式）
 * ═══════════════════════════════════════════════════════════════ */

/* ═══════════════════════════════════════════════════════════════
 * 命令-应答式单次采集（上位机 / 树莓派）
 * ═══════════════════════════════════════════════════════════════ */

static volatile uint8_t s_cap_busy       = 0;
static uint8_t          cap_mode         = SCOPE_MODE_SINGLE;  /* a1: 0=单路 1=三路 2=双ADC */
static volatile uint8_t s_inter3_ok      = 0;
static uint32_t         s_cap_tok        = 0;
static uint32_t         s_cap_t0         = 0;
static uint32_t         s_cap_gain_milli = 0;

static uint16_t lc_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t lc_u32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static uint16_t lc_crc16(const uint8_t *d, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t  b;
    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)d[i] << 8);
        for (b = 0; b < 8u; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

int W5500_SendRaw(const uint8_t *data, uint32_t len)
{
    uint32_t off = 0;
    uint32_t t0  = HAL_GetTick();

    if (data == NULL) return -1;

    while (off < len) {
        uint32_t chunk = len - off;
        int32_t  r;

        if (chunk > 4096u) chunk = 4096u;

        r = send(0, (uint8_t *)(data + off), (uint16_t)chunk);
        if (r > 0) {
            off += (uint32_t)r;
            continue;
        }
        if (r == SOCK_BUSY) {
            if ((HAL_GetTick() - t0) > 3000u) return -1;
            continue;
        }
        return -1;
    }
    return 0;
}

static void lc_fill_hdr(ScopeHdr_t *h, uint8_t type, uint32_t n_this,
                        uint32_t tok, uint16_t flags)
{
    memset(h, 0, sizeof(*h));
    h->magic      = LSDA_MAGIC;
    h->ver        = 1u;
    h->type       = type;
    h->gear       = g_fs_index;
    h->mode       = g_scope_mode;
    h->fs_hz      = g_fs_hz;
    h->n_total    = g_scope_n;
    h->n_this     = n_this;
    h->gain_milli = s_cap_gain_milli;
    h->tok        = tok;
    h->flags      = flags;
    h->crc16      = lc_crc16((const uint8_t *)h, 30u);
}

static int lc_send_ack(uint8_t type, uint32_t tok, uint16_t flags)
{
    ScopeHdr_t h;
    lc_fill_hdr(&h, type, 0UL, tok, flags);
    return W5500_SendRaw((const uint8_t *)&h, LSDA_HDR_LEN);
}

/* 参数 blob 回传：32 字节头 + payload（一次性发完，blob 只有几 KB） */
static int lc_send_blob(const uint8_t *d, uint32_t len, uint32_t tok)
{
    ScopeHdr_t h;
    lc_fill_hdr(&h, LSDA_TYPE_DATA, len, tok, 0u);
    if (W5500_SendRaw((const uint8_t *)&h, LSDA_HDR_LEN) != 0) {
        return -1;
    }
    if (len > 0u && W5500_SendRaw(d, len) != 0) {
        return -1;
    }
    return 0;
}

static int lc_send_data(void)
{
    static uint16_t chunk[512];        /* 512 点 = 1024 B */
    ScopeHdr_t h;
    uint32_t   off   = 0;
    uint32_t   total = ScopeAcq_Total();
    char       dbg[192];

    snprintf(dbg, sizeof(dbg),
             "[TX ] DATA hdr: magic=%08lX ver=%u type=%u gear=%u mode=%u "
             "fs=%lu n=%lu gain=%lu tok=%lu\r\n",
             (unsigned long)LSDA_MAGIC, (unsigned)1u, (unsigned)LSDA_TYPE_DATA,
             (unsigned)g_fs_index, (unsigned)g_scope_mode,
             (unsigned long)g_fs_hz, (unsigned long)total,
             (unsigned long)s_cap_gain_milli, (unsigned long)s_cap_tok);
    Debug_Print(dbg);

    /* 实测采样率: DWT 周期数 -> 真实 fs，用来暴露"触发被丢一半"这类问题 */
    if (g_scope_cyc > 0UL) {
        uint64_t fs_num = (uint64_t)total * 170000000ULL;
        uint32_t fs_meas = (uint32_t)(fs_num / (uint64_t)g_scope_cyc);
        snprintf(dbg, sizeof(dbg),
                 "[CAP] cyc=%lu us=%lu n=%lu -> fs_meas=%lu Hz (nominal %lu)\r\n",
                 (unsigned long)g_scope_cyc,
                 (unsigned long)(g_scope_cyc / 170UL),
                 (unsigned long)total,
                 (unsigned long)fs_meas,
                 (unsigned long)g_fs_hz);
        Debug_Print(dbg);
    }

    lc_fill_hdr(&h, LSDA_TYPE_DATA, total, s_cap_tok, 0u);
    if (W5500_SendRaw((const uint8_t *)&h, LSDA_HDR_LEN) != 0) {
        Debug_Print("[TX ] header send FAILED\r\n");
        return -1;
    }

    while (off < total) {
        uint32_t cnt = ScopeAcq_Fill(off, chunk, 512u);
        if (cnt == 0UL) break;
        if (W5500_SendRaw((const uint8_t *)chunk, cnt * 2UL) != 0) {
            snprintf(dbg, sizeof(dbg), "[TX ] payload FAILED at off=%lu\r\n",
                     (unsigned long)off);
            Debug_Print(dbg);
            return -1;
        }
        off += cnt;
    }

    snprintf(dbg, sizeof(dbg), "[TX ] DATA sent: %lu pts, %lu bytes (payload)\r\n",
             (unsigned long)off, (unsigned long)(off * 2UL));
    Debug_Print(dbg);
    return 0;
}

static void lc_handle_cmd(const uint8_t *p, uint32_t rlen)
{
    uint8_t  cmd = p[4];
    uint8_t  a0  = p[5];
    uint8_t  a1  = p[6];
    uint8_t  a2  = p[7];
    uint8_t  a3  = p[8];
    uint16_t n   = lc_u16(p + 10);
    uint32_t tok = lc_u32(p + 12);
    char     dbg[160];

    snprintf(dbg, sizeof(dbg),
             "[CMD] cmd=%u a0=%u a1=%u a2=%u a3=%u n=%u tok=%lu\r\n",
             (unsigned)cmd, (unsigned)a0, (unsigned)a1, (unsigned)a2,
             (unsigned)a3, (unsigned)n, (unsigned long)tok);
    Debug_Print(dbg);

    switch (cmd) {
    case LCMD_PING:
        (void)lc_send_ack(LSDA_TYPE_ACK, tok, s_inter3_ok ? LSDA_FLAG_INTER3_OK : 0u);
        break;

    case LCMD_CAPTURE:
        if (s_cap_busy) {
            Debug_Print("[CAP] busy, reject\r\n");
            (void)lc_send_ack(LSDA_TYPE_ERR, tok, LSDA_ERR_BUSY);
            break;
        }
        /* a1: 0=单路, 1=三路软件交织, 2=双 ADC 硬件交织 */
    if (a1 == 1u)      { SampleClock_SetMode(SAMPLE_MODE_INTER3); cap_mode = SCOPE_MODE_INTER3; }
    else if (a1 == 2u) { SampleClock_SetMode(SAMPLE_MODE_INTER2); cap_mode = SCOPE_MODE_INTER2; }
    else               { SampleClock_SetMode(SAMPLE_MODE_SINGLE); cap_mode = SCOPE_MODE_SINGLE; }
        if (SampleClock_Set(a0) != 0) {
            Debug_Print("[CAP] bad gear\r\n");
            (void)lc_send_ack(LSDA_TYPE_ERR, tok, LSDA_ERR_PARAM);
            break;
        }
        s_cap_gain_milli = (uint32_t)(AFE_GetTotalGain((uint8_t)((a1 == 1u) ? 1u : (uint16_t)(a2 + 1u))) * 1000.0f);
        /* a3：0=差分（正常）  1=单端量「正脚」  2=单端量「负脚」（调试引脚电压用） */
        if (a3 != 0u) {
            ADC_ReconfigSingleEnded(a3);
            g_scope_single_ended = 1u;
        } else {
            ADC_ReconfigSingleEnded(0u);
            g_scope_single_ended = 0u;
        }
        if (ScopeAcq_Start(cap_mode,
                           a2, n) != 0) {
            Debug_Print("[CAP] ScopeAcq_Start FAILED\r\n");
            (void)lc_send_ack(LSDA_TYPE_ERR, tok, LSDA_ERR_START);
            break;
        }
        s_cap_tok  = tok;
        s_cap_busy = 1;
        s_cap_t0   = HAL_GetTick();
        snprintf(dbg, sizeof(dbg),
                 "[CAP] start: mode=%u gear=%u ch=%u n=%u fs=%lu gain=%lu\r\n",
                 (unsigned)((a1 == 1u) ? SCOPE_MODE_INTER3 : SCOPE_MODE_SINGLE),
                 (unsigned)a0, (unsigned)a2, (unsigned)n,
                 (unsigned long)g_fs_hz, (unsigned long)s_cap_gain_milli);
        Debug_Print(dbg);
        break;

    case LCMD_STATUS:
        (void)lc_send_ack(LSDA_TYPE_ACK, tok,
                          (uint16_t)(s_inter3_ok ? LSDA_FLAG_INTER3_OK : 0u));
        break;

    case LCMD_ABORT:
        ScopeAcq_Abort();
        s_cap_busy = 0;
        (void)lc_send_ack(LSDA_TYPE_ACK, tok, 0u);
        break;

    case LCMD_SET_AFE:
        AFE_SetChannelMode(a0,
                           (a1 != 0u) ? AFE_COUPLING_AC : AFE_COUPLING_DC,
                           (a2 != 0u) ? AFE_ATTEN_20X   : AFE_ATTEN_1X,
                           (a3 != 0u) ? AFE_GAIN_20X    : AFE_GAIN_1X);
        (void)lc_send_ack(LSDA_TYPE_ACK, tok, 0u);
        break;

    case LCMD_SELFTEST:
        s_inter3_ok = ScopeAcq_SelfTest();
        (void)lc_send_ack(LSDA_TYPE_ACK, tok,
                          (uint16_t)(s_inter3_ok ? LSDA_FLAG_INTER3_OK : 0u));
        break;

    case LCMD_PARAM_BEGIN:
    {
        uint32_t total = (uint32_t)a0 | ((uint32_t)a1 << 8)
                       | ((uint32_t)a2 << 16) | ((uint32_t)a3 << 24);
        if (ParamStore_Begin(total) == HAL_OK) {
            (void)lc_send_ack(LSDA_TYPE_ACK, tok, 0u);
        } else {
            (void)lc_send_ack(LSDA_TYPE_ERR, tok, LSDA_ERR_PARAM);
        }
        break;
    }

    case LCMD_PARAM_CHUNK:
    {
        uint32_t off = (uint32_t)a0 | ((uint32_t)a1 << 8)
                     | ((uint32_t)a2 << 16) | ((uint32_t)a3 << 24);
        uint32_t dl  = (rlen > LCMD_LEN) ? (rlen - LCMD_LEN) : 0u;
        if (dl > 0u && ParamStore_Chunk(off, p + LCMD_LEN, dl) == HAL_OK) {
            (void)lc_send_ack(LSDA_TYPE_ACK, tok, 0u);
        } else {
            (void)lc_send_ack(LSDA_TYPE_ERR, tok, LSDA_ERR_PARAM);
        }
        break;
    }

    case LCMD_PARAM_COMMIT:
        if (ParamStore_Commit() == HAL_OK) {
            (void)lc_send_ack(LSDA_TYPE_ACK, tok, 0u);
        } else {
            (void)lc_send_ack(LSDA_TYPE_ERR, tok, LSDA_ERR_PARAM);
        }
        break;

    case LCMD_PARAM_LOAD:
    {
        uint32_t off = (uint32_t)a0 | ((uint32_t)a1 << 8)
                     | ((uint32_t)a2 << 16) | ((uint32_t)a3 << 24);
        uint32_t tot = ParamStore_Size();
        uint32_t want = (uint32_t)n;
        if (off >= tot) {
            (void)lc_send_ack(LSDA_TYPE_ERR, tok, LSDA_ERR_PARAM);
        } else {
            if (want == 0u || want > (tot - off)) {
                want = tot - off;
            }
            (void)lc_send_blob(ParamStore_Ptr() + off, want, tok);
        }
        break;
    }

    case LCMD_PARAM_INFO:
    {
        uint8_t info[8];
        uint32_t sz = ParamStore_Size();
        info[0] = (uint8_t)(sz & 0xFFu);
        info[1] = (uint8_t)((sz >> 8) & 0xFFu);
        info[2] = (uint8_t)((sz >> 16) & 0xFFu);
        info[3] = (uint8_t)((sz >> 24) & 0xFFu);
        {
            uint32_t sq = ParamStore_Seq();
            info[4] = (uint8_t)(sq & 0xFFu);
            info[5] = (uint8_t)((sq >> 8) & 0xFFu);
            info[6] = (uint8_t)((sq >> 16) & 0xFFu);
            info[7] = (uint8_t)((sq >> 24) & 0xFFu);
        }
        (void)lc_send_blob(info, 8u, tok);
        break;
    }

    case LCMD_PARAM_CLEAR:
        (void)ParamStore_Clear();
        (void)lc_send_ack(LSDA_TYPE_ACK, tok, 0u);
        break;

    case LCMD_LOG_PUT:
    {
        uint32_t dl = (rlen > LCMD_LEN) ? (rlen - LCMD_LEN) : 0u;
        if (dl >= 1u) {
            uint8_t lvl = p[LCMD_LEN];
            uint32_t tl = dl - 1u;
            if (tl > 120u) {
                tl = 120u;
            }
            (void)LogStore_Put(lvl, (const char *)(p + LCMD_LEN + 1u), (uint8_t)tl);
            (void)lc_send_ack(LSDA_TYPE_ACK, tok, 0u);
        } else {
            (void)lc_send_ack(LSDA_TYPE_ERR, tok, LSDA_ERR_PARAM);
        }
        break;
    }

    case LCMD_LOG_INFO:
    {
        uint8_t info[8];
        uint32_t sec = 0u, sq = 0u;
        LogStore_Stat(&sec, &sq);
        info[0] = (uint8_t)(sq & 0xFFu);
        info[1] = (uint8_t)((sq >> 8) & 0xFFu);
        info[2] = (uint8_t)((sq >> 16) & 0xFFu);
        info[3] = (uint8_t)((sq >> 24) & 0xFFu);
        info[4] = (uint8_t)(sec & 0xFFu);
        info[5] = (uint8_t)((sec >> 8) & 0xFFu);
        info[6] = (uint8_t)((sec >> 16) & 0xFFu);
        info[7] = (uint8_t)((sec >> 24) & 0xFFu);
        (void)lc_send_blob(info, 8u, tok);
        break;
    }

    case LCMD_LOG_READ:
    {
        static uint8_t lbuf[1000];
        uint32_t from = (uint32_t)a0 | ((uint32_t)a1 << 8)
                      | ((uint32_t)a2 << 16) | ((uint32_t)a3 << 24);
        uint32_t used = LogStore_Dump(from, lbuf, sizeof(lbuf));
        if (used > 0u) {
            (void)lc_send_blob(lbuf, used, tok);
        } else {
            (void)lc_send_ack(LSDA_TYPE_ACK, tok, 0u);   /* 没有更多日志 */
        }
        break;
    }

    case LCMD_LOG_CLEAR:
        LogStore_Clear();
        (void)lc_send_ack(LSDA_TYPE_ACK, tok, 0u);
        break;

    default:
        (void)lc_send_ack(LSDA_TYPE_ERR, tok, LSDA_ERR_PARAM);
        break;
    }
}

/* ─── 采集结果调试输出 ─── */
static void lc_dbg_stats(void)
{
    static uint16_t tmp[256];
    int32_t  vmin = 32767;
    int32_t  vmax = -32768;
    int64_t  sum  = 0;
    uint32_t off  = 0UL;
    uint32_t total = ScopeAcq_Total();
    uint32_t cnt, i;
    char     dbg[192];
    int      head[8];
    int      hn = 0;

    while (off < total) {
        cnt = ScopeAcq_Fill(off, tmp, 256u);
        if (cnt == 0UL) break;
        for (i = 0UL; i < cnt; i++) {
            int16_t s = (int16_t)tmp[i];
            if (s < vmin) vmin = s;
            if (s > vmax) vmax = s;
            sum += (int64_t)s;
            if (hn < 8) head[hn++] = (int)s;
        }
        off += cnt;
    }

    if (off == 0UL) {
        Debug_Print("[CAP] done but no data\r\n");
        return;
    }

    snprintf(dbg, sizeof(dbg),
             "[CAP] done: mode=%u n=%lu ch=%u | codes[%ld,%ld] mean=%ld | "
             "head=%d,%d,%d,%d,%d,%d\r\n",
             (unsigned)g_scope_mode, (unsigned long)off,
             (unsigned)g_scope_sel_ch,
             (long)vmin, (long)vmax, (long)(sum / (int64_t)off),
             head[0], head[1], head[2], head[3], head[4], head[5]);
    Debug_Print(dbg);

    /* 原始 DMA 字（未做符号扩展），用来核对 ADC 到底是补码还是偏移码 */
    {
        static uint16_t raw[8];
        uint32_t rn = ScopeAcq_RawPeek(0UL, raw, 8u);
        snprintf(dbg, sizeof(dbg),
                 "[CAP] raw=%04X %04X %04X %04X %04X %04X %04X %04X (n=%lu)\r\n",
                 (unsigned)(rn > 0u ? raw[0] : 0u), (unsigned)(rn > 1u ? raw[1] : 0u),
                 (unsigned)(rn > 2u ? raw[2] : 0u), (unsigned)(rn > 3u ? raw[3] : 0u),
                 (unsigned)(rn > 4u ? raw[4] : 0u), (unsigned)(rn > 5u ? raw[5] : 0u),
                 (unsigned)(rn > 6u ? raw[6] : 0u), (unsigned)(rn > 7u ? raw[7] : 0u),
                 (unsigned long)rn);
        Debug_Print(dbg);
    }
}

/* ─── UDP 收发 ─── */
static void lc_service(void)
{
    if (!s_cap_busy) return;

    if (ScopeAcq_Done()) {
        s_cap_busy = 0;
        lc_dbg_stats();
        (void)lc_send_data();
        return;
    }

    if ((HAL_GetTick() - s_cap_t0) > 500u) {
        ScopeAcq_Abort();
        s_cap_busy = 0;
        Debug_Print("[LCMD] capture timeout\r\n");
        (void)lc_send_ack(LSDA_TYPE_ERR, s_cap_tok, LSDA_ERR_TIMEOUT);
    }
}

/* ═══════════════════════════════════════════════════════════════
 * 网络轮询主入口: TCP 命令解析 + UDP 数据泵保活
 * ═══════════════════════════════════════════════════════════════ */

static void w5500_poll_tcp(void);
static void w5500_poll_udp(void);

void W5500_Poll(void)
{
    if (!g_eth_initialized) return;

    /* TCP 命令解析 */
    w5500_poll_tcp();
    /* UDP Socket 1 关闭则重建 */
    if (getSn_SR(1) != SOCK_UDP && getSn_SR(1) == SOCK_CLOSED) {
        Debug_Print("[NET] UDP Socket 1 closed, re-initializing...\r\n");
        uint8_t rip[4] = {192, 168, 1, 100};
        W5500_HighSpeed_UDP_Init(5001, rip, 8080);
    }
}

/* ─── TCP 状态机 ─── */
static void w5500_poll_tcp(void)
{
    static uint8_t rxbuf[2048];
    uint8_t sr;
    int32_t len;
    static uint8_t s_ka_done = 0;   /* 本次连接是否已开启 keep-alive */
static uint32_t s_last_rx = 0;  /* 最近一次收到对端数据的时刻（空闲看门狗用） */

    sr = getSn_SR(0);

    /* CRITICAL 状态迁移监控 */
    {
        static uint8_t last_logged_sr = 0xFF;
        if (sr != last_logged_sr) {
            char buf[64];
            const char *sname = "???";
            switch (sr) {
                case 0x00: sname = "CLOSED";       break;
                case 0x13: sname = "INIT";         break;
                case 0x14: sname = "LISTEN";       break;
                case 0x15: sname = "SYNSENT";      break;
                case 0x16: sname = "SYN_RECV";     break;
                case 0x17: sname = "ESTABLISHED";  break;
                case 0x18: sname = "FIN_WAIT";     break;
                case 0x1B: sname = "TIME_WAIT";    break;
                case 0x1C: sname = "CLOSE_WAIT";   break;
                case 0x1D: sname = "LAST_ACK";     break;
                default:   break;
            }
            snprintf(buf, sizeof(buf),
                "[CRITICAL] Socket 0 Status: 0x%02X->0x%02X (%s)\r\n",
                last_logged_sr, sr, sname);
            Debug_Print(buf);
            last_logged_sr = sr;
        }
    }

    /* ── 半关闭状态看门狗 ──
       W5500 不会从 FIN_WAIT / CLOSING / LAST_ACK / TIME_WAIT 自动回到 LISTEN。
       上位机进程被杀、拔网线、或 TCP 连接被对端 RST 之后，socket 可能残留在这几个
       状态里：此时 ICMP 仍由 W5500 硬件应答（ping 通），但 5000 端口不再回 SYN-ACK，
       表现为"ping 得通、连不上"。这里超时 2 s 就强制 close 复位到 CLOSED，
       下一轮 poll 会重建 socket 并重新 listen。 */
    {
        static uint32_t s_stuck_cnt = 0;

        switch (sr)
        {
        case SOCK_CLOSED:
        case SOCK_INIT:
        case SOCK_LISTEN:
        case SOCK_SYNRECV:
        case SOCK_ESTABLISHED:
        case SOCK_CLOSE_WAIT:
            s_stuck_cnt = 0;
            break;

        default:
            if (++s_stuck_cnt > 2000U)
            {
                Debug_Print("[TCP] stuck in half-closed state, force close\r\n");
                close(0);
                s_stuck_cnt = 0;
            }
            break;
        }
    }

    switch (sr)
    {
    case SOCK_CLOSED:
        s_ka_done = 0;
        Debug_Print("[TCP] SOCK_CLOSED -> creating socket...\r\n");
        if (socket(0, Sn_MR_TCP, 5000, 0) == 0)
        {
            Debug_Print("[TCP] Socket created, now in INIT\r\n");
        }
        else
        {
            Debug_Print("[TCP] socket() failed, retry next poll\r\n");
        }
        break;

    case SOCK_INIT:
        Debug_Print("[TCP] SOCK_INIT -> starting listen...\r\n");
        if (listen(0) == SOCK_OK)
        {
            Debug_Print("[TCP] Listening on port 5000\r\n");
        }
        else
        {
            uint8_t hw_sr = getSn_SR(0);
            if (hw_sr == SOCK_LISTEN || hw_sr == SOCK_ESTABLISHED)
            {
                Debug_Print("[TCP] listen() returned error but HW is already listening\r\n");
            }
            else
            {
                Debug_Print("[TCP] listen() failed, will retry\r\n");
            }
        }
        break;

    case SOCK_LISTEN:
    case SOCK_SYNRECV:
        break;

    case SOCK_ESTABLISHED:
    {
        static TStaPacket_t s_pending;
        static uint8_t      s_has_pending = 0;

        /* 每个连接只设一次 keep-alive：空闲 5 s 后 W5500 自动发探测包。
           上位机崩溃 / 拔网线时对端不再 ACK，W5500 按 RTR*RCR 重传失败后自动关 socket
           回到 LISTEN，避免永远卡在 ESTABLISHED 让后续连接全部超时。 */
        if (!s_ka_done)
        {
            setSn_KPALVTR(0, 5);
            s_ka_done = 1;
            s_last_rx = HAL_GetTick();
            Debug_Print("[TCP] keep-alive enabled (5s)\r\n");
        }

        /* 应用层空闲看门狗：对端进程已退出、但内核不再回 RST 时，
           W5500 会一直停在 ESTABLISHED，导致新连接被 RST（表现为
           “Connection refused”）。上位机每 8 s 发一次 PING 保活，
           所以这里给到 120 s——只有真的掉线才断，不会误断正在用的连接。 */
        if (!s_cap_busy && (HAL_GetTick() - s_last_rx) > 120000u)
        {
            Debug_Print("[TCP] idle timeout 120s, force disconnect\r\n");
            disconnect(0);
            s_ka_done = 0;
            break;
        }

        /* Socket 异常关闭，尝试重建 */
        if (s_has_pending)
        {
            int32_t sret = send(0, (uint8_t*)&s_pending, sizeof(s_pending));
            if (sret > 0)
            {
                Debug_Print("[TCP-REPLY] pending sta sent ok\r\n");
                s_has_pending = 0;
            }
            else if (sret != SOCK_BUSY)
            {
                char dbg[48];
                snprintf(dbg, sizeof(dbg), "[TCP-REPLY] pending fail: %ld\r\n", (long)sret);
                Debug_Print(dbg);
                s_has_pending = 0;   /* 非 BUSY 错误，丢弃 */
            }
        }

        /* 新协议：若有单次采集在等待，完成则回传数据 */
        lc_service();

        /* Socket 异常关闭，尝试重建 */
        if (getSn_IR(0) & Sn_IR_RECV)
        {
            setSn_IR(0, Sn_IR_RECV);
            len = recv(0, rxbuf, sizeof(rxbuf));
            if (len > 0)
            {
                s_last_rx = HAL_GetTick();
                char hex_str[256];
                int i, pos = 0;
                for (i = 0; i < len && pos < (int)sizeof(hex_str) - 4; i++) {
                    pos += snprintf(hex_str + pos, sizeof(hex_str) - pos, "%02X ", rxbuf[i]);
                }
                char dbg_line[320];
                snprintf(dbg_line, sizeof(dbg_line), "[TCP-RX] %d bytes: %s\r\n", (int)len, hex_str);
                Debug_Print(dbg_line);
            }
            /* 新协议：16 字节命令（magic 'LSCO'）优先 */
            if (len >= (int32_t)LCMD_LEN && lc_u32(rxbuf) == LCMD_MAGIC)
            {
                lc_handle_cmd(rxbuf, (uint32_t)len);
                break;
            }

            if (len >= 8)
            {
                TCmdPacket_t *cmd = (TCmdPacket_t *)rxbuf;
                uint8_t cksum = calc_cksum((uint8_t *)cmd, 7);
                if (cksum != cmd->checksum) {
                    Debug_Print("[TCP] CMD checksum error, ignored\r\n");
                    break;
                }

                TStaPacket_t sta;
                memset(&sta, 0, sizeof(sta));
                sta.rsp = 0x81;

                switch (cmd->cmd) {
                case CMD_CONFIGURE:
                    if (cmd->resolution == 8 || cmd->resolution == 10 || cmd->resolution == 12)
                        g_cap_cfg.resolution = cmd->resolution;
                    if (cmd->ch_mask > 0 && cmd->ch_mask <= 0x07)
                        g_cap_cfg.ch_mask = cmd->ch_mask;
                    if (cmd->sample_rate > 0 && cmd->sample_rate <= 200000)
                        g_cap_cfg.sample_rate = cmd->sample_rate;
                    sta.status = g_cap_cfg.running ? 1 : 0;
                    Debug_Print("[TCP] CMD: CONFIGURE\r\n");
                    break;

                case CMD_START:
                    g_cap_cfg.running = 1;
                    sta.status = 1;
                    Debug_Print("[TCP] CMD: START\r\n");
                    break;

                case CMD_STOP:
                    g_cap_cfg.running = 0;
                    sta.status = 0;
                    Debug_Print("[TCP] CMD: STOP\r\n");
                    break;

                case CMD_STATUS:
                    sta.status = g_cap_cfg.running ? 1 : 0;
                    Debug_Print("[TCP] CMD: STATUS\r\n");
                    break;

                case CMD_SINGLE:
                    sta.status = 2;
                    Debug_Print("[TCP] CMD: SINGLE trigger\r\n");
                    break;

                default:
                    sta.error = 1;
                    Debug_Print("[TCP] CMD: UNKNOWN\r\n");
                    break;
                }

                sta.ch_mask     = g_cap_cfg.ch_mask;
                sta.resolution  = g_cap_cfg.resolution;
                sta.sample_rate = (uint16_t)g_cap_cfg.sample_rate;
                sta.checksum    = calc_cksum((uint8_t *)&sta, 7);

                /* 尝试发送回复，BUSY 则存为 pending */
                int32_t sret = send(0, (uint8_t*)&sta, sizeof(sta));
                if (sret > 0)
                {
                    Debug_Print("[TCP-REPLY] sta sent ok\r\n");
                }
                else
                {
                    char buf[64];
                    snprintf(buf, sizeof(buf), "[TCP-REPLY] busy/fail: %ld\r\n", (long)sret);
                    Debug_Print(buf);
                    if (sret == SOCK_BUSY)
                    {
                        memcpy(&s_pending, &sta, sizeof(sta));
                        s_has_pending = 1;
                    }
                }
            }
        }
        break;
    }

    case SOCK_CLOSE_WAIT:
        Debug_Print("[TCP] CLOSE_WAIT -> disconnect (will auto-transition to CLOSED)\r\n");
        disconnect(0);
        break;

    case SOCK_TIME_WAIT:
        break;

    default:
        break;
    }
}

/* ─── UDP 收发 ─── */
static void w5500_poll_udp(void)
{
    static uint8_t rxbuf[2048];
    static uint32_t last_status_tick = 0;
    uint8_t sr;
    int32_t len;
    uint8_t remote_ip[4];
    uint16_t remote_port;

    sr = getSn_SR(0);

    /* 状态监控 */
    {
        static uint8_t last_logged_sr = 0xFF;
        if (sr != last_logged_sr) {
            char buf[48];
            snprintf(buf, sizeof(buf),
                "[UDP] Socket 0 Status: 0x%02X->0x%02X\r\n", last_logged_sr, sr);
            Debug_Print(buf);
            last_logged_sr = sr;
        }
    }

    if (sr != SOCK_UDP)
    {
        /* Socket 异常关闭，尝试重建 */
        if (sr == SOCK_CLOSED)
        {
            Debug_Print("[UDP] Socket closed, re-creating...\r\n");
            W5500_UDP_Init(5000);
        }
        return;
    }

    /* 接收 UDP 数据 */
    if (getSn_IR(0) & Sn_IR_RECV)
    {
        setSn_IR(0, Sn_IR_RECV);
        len = recvfrom(0, rxbuf, sizeof(rxbuf), remote_ip, &remote_port);
        if (len > 0)
        {
            /* 更新对端地址（用于后续发送） */
            memcpy(g_udp_remote_ip, remote_ip, 4);
            g_udp_remote_port = remote_port;

            char buf[64];
            snprintf(buf, sizeof(buf),
                "[UDP] Recv %d bytes from %d.%d.%d.%d:%u\r\n",
                (int)len,
                remote_ip[0], remote_ip[1], remote_ip[2], remote_ip[3],
                remote_port);
            Debug_Print(buf);

            /* Echo 回传 */
            len = sendto(0, rxbuf, (uint16_t)len, remote_ip, remote_port);
            if (len > 0)
            {
                snprintf(buf, sizeof(buf), "[UDP] Echo sent %d bytes\r\n", (int)len);
                Debug_Print(buf);
            }
        }
    }

    /* 定时状态上报（每 1000ms 通过网络发送一次系统状态） */
    if (HAL_GetTick() - last_status_tick >= 1000)
    {
        extern int16_t adc1_buf[], adc2_buf[], adc3_buf[];
        extern TIM_HandleTypeDef htim4, htim1;
        char pkt[128];
        int pkt_len = snprintf(pkt, sizeof(pkt),
            "STATUS|CH1:%d|CH2:%d|CH3:%d|ENC1:%ld|ENC2:%ld\r\n",
            (int)adc1_buf[0],
            (int)adc2_buf[0],
            (int)adc3_buf[0],
            (int32_t)__HAL_TIM_GET_COUNTER(&htim4),
            (int32_t)__HAL_TIM_GET_COUNTER(&htim1));

        sendto(0, (uint8_t*)pkt, (uint16_t)pkt_len, g_udp_remote_ip, g_udp_remote_port);
        last_status_tick = HAL_GetTick();
    }
}

/* ═══════════════════════════════════════════════════════════════
 * UDP 数据收发 API
 * ═══════════════════════════════════════════════════════════════ */

int32_t W5500_UDP_SendTo(const uint8_t *data, uint16_t len, uint8_t *remote_ip, uint16_t remote_port)
{
    if (getSn_SR(0) != SOCK_UDP) return -1;
    return sendto(0, (uint8_t*)data, len, remote_ip, remote_port);
}

int32_t W5500_UDP_RecvFrom(uint8_t *data, uint16_t max_len, uint8_t *remote_ip, uint16_t *remote_port)
{
    if (getSn_RX_RSR(0) == 0) return 0;
    return recvfrom(0, data, max_len, remote_ip, remote_port);
}

/* ═══════════════════════════════════════════════════════════════
 * 兼容层（TCP 模式）
 * ═══════════════════════════════════════════════════════════════ */

int W5500_Net_SendData(const uint8_t *data, uint16_t len)
{
    if (getSn_SR(0) != SOCK_ESTABLISHED) return -1;
    return send(0, (uint8_t *)data, len);
}

int W5500_Net_RecvData(uint8_t *data, uint16_t len)
{
    if (getSn_RX_RSR(0) == 0) return 0;
    return (int)recv(0, data, len);
}

int W5500_Net_IsConnected(void)
{
    uint8_t sr = getSn_SR(0);
    if (g_net_mode == NET_MODE_UDP)
        return (sr == SOCK_UDP) ? 1 : 0;
    else
        return (sr == SOCK_ESTABLISHED) ? 1 : 0;
}

/* ═══════════════════════════════════════════════════════════════
 * Socket 1 高速 UDP 数据泵（Ring Buffer 架构）
 * ═══════════════════════════════════════════════════════════════ */

#define HS_UDP_PORT      6000
#define HS_HALF_RAW      1024                           /* 每半缓冲 raw 样本数 */
#define HS_HALF_DIFF     512                            /* 每半缓冲 差分样本数 */
#define HS_PKT_SIZE 1032                                /* 最大容纳 172×6 字节（不含包头） */
#define HS_HEADER_LEN    6                              /* 轻量包头 6 字节 */
#define HS_MAX_PKT_LEN (HS_PKT_SIZE + HS_HEADER_LEN)    /* 最大包长（含包头）1038 */

#define HS_SEG0 170
#define HS_SEG1 170
#define HS_SEG2 172      /* 170+170+172=512 差分样本完整覆盖半缓冲 */
#define RING_DEPTH 8     /* 4→8 扩容，消除溢出 margin */


static uint8_t  g_hs_remote_ip[4]   = {192, 168, 1, 100};
static uint16_t g_hs_remote_port    = HS_UDP_PORT;
static uint32_t g_hs_pkt_seq        = 0;
static uint32_t g_hs_drop_cnt       = 0;
static uint32_t g_hs_sent_cnt       = 0;
static uint32_t g_hs_ring_overflow  = 0;

/* ── Ring Buffer ── */
static __attribute__((aligned(4))) uint8_t g_ring_buf[RING_DEPTH][HS_MAX_PKT_LEN];
static              uint16_t g_ring_pkt_len[RING_DEPTH];
static volatile     uint8_t  g_ring_wr = 0;
static volatile     uint8_t  g_ring_rd = 0;
static volatile     uint8_t  g_ring_cnt = 0;
static              uint32_t g_pkt_seq = 0;          /* 数据包序号计数器 */

/* ── 写入轻量包头（6 字节） ── */
static void hs_write_header(uint8_t *dst, uint16_t seq,
                            uint8_t half_id, uint8_t seg_id, uint8_t sample_cnt)
{
    dst[0] = (uint8_t)(seq >> 8);          /* seq_id Hi */
    dst[1] = (uint8_t)(seq & 0xFF);        /* seq_id Lo */
    dst[2] = (half_id << 6) | (seg_id << 4);  /* flags: [7:6]=半缓冲, [5:4]=段号 */
    dst[3] = sample_cnt;                   /* 本包差分样本数 (170/172) */
    dst[4] = 0;                            /* reserved */
    dst[5] = 0;                            /* reserved */
}

/* ── 打包辅助：包头 + ADC raw 差分值写入指定缓冲区 ── */
static void hs_pack_segment(uint8_t *dst, uint8_t half_id, uint8_t seg_id,
                            uint16_t seq, uint16_t seg_start,
                            uint16_t *adc0, uint16_t *adc1, uint16_t *adc2,
                            uint16_t count)
{
    uint16_t i;
    uint8_t *body = dst + HS_HEADER_LEN;

    /* 写入包头 */
    hs_write_header(dst, seq, half_id, seg_id, (uint8_t)count);

    /* 写入差分数据 */
    for (i = 0; i < count; i++) {
        uint32_t raw = (uint32_t)(seg_start + i) << 1;   /* raw 编号 = 差分偏移 × 2 */
        int16_t ch1 = (int16_t)adc0[raw + 1] - (int16_t)adc0[raw];
        int16_t ch2 = (int16_t)adc1[raw + 1] - (int16_t)adc1[raw];
        int16_t ch3 = (int16_t)adc2[raw + 1] - (int16_t)adc2[raw];
        body[i*6+0] = (uint8_t)(ch1 >> 8);
        body[i*6+1] = (uint8_t)(ch1 & 0xFF);
        body[i*6+2] = (uint8_t)(ch2 >> 8);
        body[i*6+3] = (uint8_t)(ch2 & 0xFF);
        body[i*6+4] = (uint8_t)(ch3 >> 8);
        body[i*6+5] = (uint8_t)(ch3 & 0xFF);
    }
}

/* ── 尝试发送一个 ring buffer 中的数据包 ── */
static uint8_t hs_try_send_one(void)
{
    uint16_t len;

    if (g_ring_cnt == 0) return 0;

    if (getSn_CR(1)) return 0;
    len = g_ring_pkt_len[g_ring_rd];
    if (getSn_TX_FSR(1) < len) return 0;
    if (getSn_SR(1) != SOCK_UDP) return 0;

    wiz_send_data(1, g_ring_buf[g_ring_rd], len);
    setSn_CR(1, Sn_CR_SEND);

    g_ring_rd = (g_ring_rd + 1) % RING_DEPTH;
    __disable_irq();
    g_ring_cnt--;
    __enable_irq();

    g_hs_pkt_seq++;
    g_hs_sent_cnt++;
    return 1;
}

int W5500_HighSpeed_UDP_Init(uint16_t local_port, uint8_t *remote_ip, uint16_t remote_port)
{
    int ret;
    memcpy(g_hs_remote_ip, remote_ip, 4);
    g_hs_remote_port = remote_port;
    close(1);
    /* 验证进入 SOCK_UDP (0x22) */
    ret = socket(1, Sn_MR_UDP, local_port, 0);
    if (ret != 1) {
        char buf[32];
        snprintf(buf, sizeof(buf), "[HS-UDP] socket(1) failed: %d\r\n", ret);
        Debug_Print(buf);
        return -1;
    }
    setSn_DIPR(1, g_hs_remote_ip);
    setSn_DPORT(1, g_hs_remote_port);

    g_ring_wr = 0;
    g_ring_rd = 0;
    g_ring_cnt = 0;
    g_hs_drop_cnt = 0;
    g_hs_sent_cnt = 0;
    g_hs_ring_overflow = 0;

    {
        char buf[64];
        snprintf(buf, sizeof(buf),
            "[HS-UDP] Socket 1 ready, TX=%dKB -> %d.%d.%d.%d:%u\r\n",
            getSn_TXBUF_SIZE(1),
            g_hs_remote_ip[0], g_hs_remote_ip[1], g_hs_remote_ip[2], g_hs_remote_ip[3],
            g_hs_remote_port);
        Debug_Print(buf);
    }
    {
        uint8_t ver = IINCHIP_READ(VERSIONR);
        char diag[128];
        snprintf(diag, sizeof(diag),
            "[HS-UDP] VERSIONR=0x%02X | TXBUF(1)=%dKB | FSR=%d | SR=0x%02X\r\n",
            ver, getSn_TXBUF_SIZE(1), getSn_TX_FSR(1), getSn_SR(1));
        Debug_Print(diag);
    }
    return 0;
}

void w5500_udp_stream_push(uint8_t half_sel,
                           uint16_t *adc0_buf, uint16_t *adc1_buf, uint16_t *adc2_buf)
{
    uint16_t diff_offset;
    uint8_t  seg;
    uint8_t  wr;

    /* 检查 Socket 是否存活 */
    if (getSn_SR(1) != SOCK_UDP) {
        if (getSn_SR(1) == SOCK_CLOSED)
            W5500_HighSpeed_UDP_Init(HS_UDP_PORT, g_hs_remote_ip, g_hs_remote_port);
        return;
    }

    diff_offset = half_sel ? (uint16_t)HS_HALF_DIFF : (uint16_t)0;

    /* 三段分段打包: 170+170+172=512 差分（完整覆盖） */

    static const uint16_t seg_counts[3] = {HS_SEG0, HS_SEG1, HS_SEG2};
    static const uint16_t seg_lens[3]   = {HS_SEG0 * 6 + HS_HEADER_LEN,
                                           HS_SEG1 * 6 + HS_HEADER_LEN,
                                           HS_SEG2 * 6 + HS_HEADER_LEN};  /* 1026, 1026, 1038 */

    for (seg = 0; seg < 3; seg++) {
        __disable_irq();
        if (g_ring_cnt >= RING_DEPTH) {
            __enable_irq();
            g_hs_ring_overflow++;
            continue;
        }
        wr = g_ring_wr;                         /* 先保留写入位置 */
        g_ring_wr = (wr + 1) % RING_DEPTH;      /* 再推进写指针 */
        __enable_irq();

        /* 差分偏移计算：seg0=0, seg1=170, seg2=340 */
        uint16_t seg_start = diff_offset + (seg == 0 ? 0 : (seg == 1 ? HS_SEG0 : HS_SEG0 + HS_SEG1));
        hs_pack_segment(g_ring_buf[wr], half_sel, seg,
                        (uint16_t)g_pkt_seq, seg_start,
                        adc0_buf, adc1_buf, adc2_buf,
                        seg_counts[seg]);
        g_ring_pkt_len[wr] = seg_lens[seg];
        g_pkt_seq++;                            /* 每个包序号递增 */

        g_ring_cnt++;                           /* 最后再增加计数（确保包已完整写入） */
    }

    /* 立即尝试发送（非阻塞） */
    (void)hs_try_send_one();
}

/* ── 主循环调用的重试发送（非阻塞） ── */
static void hs2_drain(uint8_t max_pkts);   /* 前置声明: 定义见文件末尾 */

void w5500_udp_retry_pending(void)
{
    if (getSn_SR(1) != SOCK_UDP) return;

    /* 排空积压包（限制每次最多 4 包，保持系统实时性） */
    for (int i = 0; i < 4; i++) {
        if (!hs_try_send_one()) break;
    }

    /* 验证进入 SOCK_UDP (0x22) */
    hs2_drain(8);
}

void W5500_Switch_Capture_Mode(CaptureMode_t new_mode)
{
    extern TIM_HandleTypeDef htim8;
    if (new_mode == CAPTURE_MODE_TCP_RECALL) {
        __HAL_TIM_MOE_DISABLE(&htim8);
        g_capture_mode = CAPTURE_MODE_TCP_RECALL;
    } else {
        g_capture_mode = CAPTURE_MODE_PURE_UDP;
        __HAL_TIM_MOE_ENABLE(&htim8);
    }
}

/* ═══════════════════════════════════════════════════════════════
 * [P0-7] 突发采集传输 — 22 字节新包头实现
 *
 *  设计要点（见《技术方案与实施细节总文档》§3.1）
 *    - 22 字节包头: magic + sample_idx + seq + fs_khz + ch_mask
 *                   + gain_code + flags + samples + crc16
 *    - sample_idx 是绝对样本序号, 上位机据此拼帧与检测丢包
 *    - fs_khz 用 16 位 (kHz 单位), 避开旧 uint16 Hz 截断 bug
 *    - CRC-16/CCITT 防止 UDP 静默损坏
 *    - 载荷 int16 交错, 直接发送 ADC 差分码（不再做相邻点求差）
 *
 *  分段: 每通道 168 样本/包 -> 载荷 168 x 6 = 1008 B, 总包长 1030 B
 *        4096 样本 -> 25 包 (24 x 168 + 1 x 64)
 * ═══════════════════════════════════════════════════════════════ */

#define HS2_SAMPLES_PER_PKT   168         /* 每段每通道样本数 */
#define HS2_PAYLOAD_PER_PKT   (HS2_SAMPLES_PER_PKT * 3 * 2)   /* 1008 B */
#define HS2_MAX_PKT_LEN       (HS2_HEADER_LEN + HS2_PAYLOAD_PER_PKT)  /* 1030 B */
#define HS2_RING_DEPTH        8           /* 8 x 1030 B = 8.2 KB */

#define HS2_PORT              8080

static uint8_t  g_hs2_remote_ip[4]  = {192, 168, 1, 110};
static uint16_t g_hs2_remote_port   = HS2_PORT;

static __attribute__((aligned(4))) uint8_t g_hs2_ring[HS2_RING_DEPTH][HS2_MAX_PKT_LEN];
static uint16_t g_hs2_len[HS2_RING_DEPTH];
static volatile uint8_t  g_hs2_wr  = 0;
static volatile uint8_t  g_hs2_rd  = 0;
static volatile uint8_t  g_hs2_cnt = 0;

static uint32_t g_hs2_seq       = 0;   /* 包序号 */
static uint32_t g_hs2_sent      = 0;   /* 已发送包数 */
static uint32_t g_hs2_drop      = 0;   /* 溢出丢弃 */
static uint32_t g_hs2_sample_ix = 0;   /* 绝对样本序号 */

extern volatile uint32_t g_fs_hz;      /* sample_clock.c */

/* ── CRC-16/CCITT (poly 0x1021, init 0xFFFF) ── */
static uint16_t hs2_crc16(const uint8_t *d, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t  b;
    for (i = 0; i < len; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void hs2_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static void hs2_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

/**
 * @brief  组一个数据包: 22B 包头 + 交错 int16 载荷
 * @param  dst        目标缓冲
 * @param  adc0/1/2   三通道基址（NULL 表示该通道无效, 置 0）
 * @param  start      本段起始样本下标
 * @param  count      本段每通道样本数
 * @param  sample_idx 本段首样本的绝对序号
 * @param  ch_mask    通道掩码
 */
static void hs2_pack(uint8_t *dst,
                     const uint16_t *adc0, const uint16_t *adc1, const uint16_t *adc2,
                     uint16_t start, uint16_t count,
                     uint32_t sample_idx, uint8_t ch_mask,
                     uint8_t gain_code, uint8_t flags)
{
    uint8_t *body = dst + HS2_HEADER_LEN;
    uint16_t i;

    hs2_put_u32(dst + 0,  HS2_MAGIC);                       /* magic        */
    hs2_put_u32(dst + 4,  sample_idx);                      /* 写 sample_idx 字段 */
    hs2_put_u32(dst + 8,  g_hs2_seq);                       /* seq          */
    hs2_put_u16(dst + 12, (uint16_t)(g_fs_hz / 1000UL));    /* fs_khz       */
    dst[14] = ch_mask;                                      /* ch_mask      */
    dst[15] = gain_code;                                    /* gain_code    */
    dst[16] = flags;                                        /* flags        */
    dst[17] = 0;                                            /* reserved     */
    hs2_put_u16(dst + 18, count);                           /* samples      */
    /* 接收 UDP 数据 */

    /* 三段分段打包: 170+170+172=512 差分（完整覆盖） */
    for (i = 0; i < count; i++) {
        uint16_t idx = (uint16_t)(start + i);
        int16_t ch1 = adc0 ? (int16_t)adc0[idx] : 0;
        int16_t ch2 = adc1 ? (int16_t)adc1[idx] : 0;
        int16_t ch3 = adc2 ? (int16_t)adc2[idx] : 0;

        body[i*6 + 0] = (uint8_t)((uint16_t)ch1 & 0xFF);
        body[i*6 + 1] = (uint8_t)((uint16_t)ch1 >> 8);
        body[i*6 + 2] = (uint8_t)((uint16_t)ch2 & 0xFF);
        body[i*6 + 3] = (uint8_t)((uint16_t)ch2 >> 8);
        body[i*6 + 4] = (uint8_t)((uint16_t)ch3 & 0xFF);
        body[i*6 + 5] = (uint8_t)((uint16_t)ch3 >> 8);
    }

    /* crc16 覆盖 offset 4..19 */
    hs2_put_u16(dst + 20, hs2_crc16(dst + 4, 16));
}

/**
 * @brief  尝试发送环形缓冲中的一包（非阻塞）
 * @retval 1=已发送, 0=未发送（无积压或超时）
 */
static uint8_t hs2_try_send_one(void)
{
    uint16_t len;
    uint8_t  rd;

    if (g_hs2_cnt == 0) return 0;
    if (getSn_SR(1) != SOCK_UDP) return 0;
    if (getSn_CR(1)) return 0;                       /* 上一命令未完成 */

    rd  = g_hs2_rd;
    len = g_hs2_len[rd];
    if (getSn_TX_FSR(1) < len) return 0;             /* TX 空间不足 */

    wiz_send_data(1, g_hs2_ring[rd], len);
    setSn_CR(1, Sn_CR_SEND);

    g_hs2_rd = (uint8_t)((rd + 1) % HS2_RING_DEPTH);
    __disable_irq();
    g_hs2_cnt--;
    __enable_irq();

    g_hs2_sent++;
    return 1;
}

int w5500_burst_send(const uint16_t *adc0, const uint16_t *adc1, const uint16_t *adc2,
                     uint16_t count, uint8_t ch_mask, uint8_t gain_code, uint8_t flags)
{
    uint16_t start = 0;
    int      queued = 0;

    if (count == 0) return 0;
    if (getSn_SR(1) != SOCK_UDP) {
        if (getSn_SR(1) == SOCK_CLOSED) {
            W5500_HighSpeed_UDP_Init(HS2_PORT, g_hs2_remote_ip, g_hs2_remote_port);
        }
        return -1;
    }

    while (start < count) {
        uint16_t seg = (uint16_t)(count - start);
        uint8_t  wr;

        if (seg > HS2_SAMPLES_PER_PKT) seg = HS2_SAMPLES_PER_PKT;

        __disable_irq();
        if (g_hs2_cnt >= HS2_RING_DEPTH) {
            __enable_irq();
            g_hs2_drop++;
            return (queued > 0) ? queued : -1;       /* 环满, 放弃剩余 */
        }
        wr = g_hs2_wr;
        g_hs2_wr = (uint8_t)((wr + 1) % HS2_RING_DEPTH);
        __enable_irq();

        hs2_pack(g_hs2_ring[wr], adc0, adc1, adc2, start, seg,
                 g_hs2_sample_ix + start, ch_mask, gain_code, flags);

        g_hs2_len[wr] = (uint16_t)(HS2_HEADER_LEN + seg * 3u * 2u);

        __disable_irq();
        g_hs2_cnt++;
        __enable_irq();

        g_hs2_seq++;
        queued++;
        start = (uint16_t)(start + seg);

        /* Socket 异常关闭，尝试重建 */
        (void)hs2_try_send_one();
    }

    g_hs2_sample_ix += count;                        /* 推进绝对样本序号 */

    /* 排空积压包（限制每次最多 4 包，保持系统实时性） */
    {
        int i;
        for (i = 0; i < 32; i++) {
            if (!hs2_try_send_one()) break;
        }
    }

    return queued;
}

int w5500_burst_flush(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    while (g_hs2_cnt > 0) {
        (void)hs2_try_send_one();
        if ((HAL_GetTick() - t0) > timeout_ms) return -1;
    }
    return 0;
}

void w5500_burst_stats(uint32_t *sent, uint32_t *drop, uint16_t *pend)
{
    if (sent) *sent = g_hs2_sent;
    if (drop) *drop = g_hs2_drop;
    if (pend) *pend = (uint16_t)g_hs2_cnt;
}

/**
 * @brief  主循环调用的排空钩子（供原有 retry_pending 一并驱动）
 * @note   在 w5500_udp_retry_pending() 末尾调用
 */
static void hs2_drain(uint8_t max_pkts)
{
    uint8_t i;
    if (getSn_SR(1) != SOCK_UDP) return;
    for (i = 0; i < max_pkts; i++) {
        if (!hs2_try_send_one()) break;
    }
}

/* ═══════════════════════════════════════════════════════════════
 * 调试用访问器：把 Socket0 的真实状态与对端信息暴露给 main.c
 * ═══════════════════════════════════════════════════════════════ */

uint8_t W5500_TcpState(void)
{
    return getSn_SR(0);
}

void W5500_TcpPeer(uint8_t ip[4], uint16_t *port)
{
    uint8_t dipr[4] = {0, 0, 0, 0};

    getSn_DIPR(0, dipr);
    if (ip != 0)
    {
        ip[0] = dipr[0];
        ip[1] = dipr[1];
        ip[2] = dipr[2];
        ip[3] = dipr[3];
    }
    if (port != 0)
    {
        *port = getSn_DPORT(0);
    }
}

void W5500_HardReset(void)
{
    BSP_W5500_Reset();
    HAL_Delay(10);
}