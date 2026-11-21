/**
 * @file    retarget.c
 * @brief   MicroLIB printf 重定向 — 完整禁用半主机断点
 *
 * armcc + MicroLIB 组合下，仅重写 fputc 不足以完全关闭半主机。
 * 必须通过 #pragma import(__use_no_semihosting) 阻止链接器链接
 * 半主机库函数，并提供 _sys_exit 和 struct __FILE 定义。
 * 否则启动阶段或某些库函数调用时仍会触发 BKPT 0xAB。
 */
#include <stdio.h>

/* ── 完整禁用半主机 (Semihosting) ── */
#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; };
FILE __stdout;

/* 重定向 printf/fprintf 输出（丢弃不输出） */
int fputc(int ch, FILE *f)
{
    (void)f;
    return ch;
}

void _ttywrch(int ch)
{
    (void)ch;
}

/* ── 半主机底层 I/O 函数 — 全部空实现 ── */
/* 标准 C 库启动阶段 _initio → freopen → _sys_open 会走半主机，
 * 必须提供空实现阻止 BKPT 0xAB。 */
typedef int FILEHANDLE;

FILEHANDLE _sys_open(const char *name, int openmode)
{
    return -1;                      /* 拒绝打开任何文件 */
}

int _sys_close(FILEHANDLE fh)
{
    return 0;
}

int _sys_write(FILEHANDLE fh, const unsigned char *buf, unsigned len, int mode)
{
    return 0;                       /* 静默丢弃 */
}

int _sys_read(FILEHANDLE fh, unsigned char *buf, unsigned len, int mode)
{
    return 0;
}

int _sys_istty(FILEHANDLE fh)
{
    return 0;                       /* 不是终端 */
}

int _sys_seek(FILEHANDLE fh, long pos)
{
    return 0;
}

long _sys_flen(FILEHANDLE fh)
{
    return 0;
}

void _sys_exit(int return_code)
{
    while (1);                      /* 禁止半主机退出 */
}
