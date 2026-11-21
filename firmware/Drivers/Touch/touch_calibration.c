#include "touch_calibration.h"
#include "../TFT_Screen/ILI9488.h"
#include "../debug_print/debug_print.h"
#include "../Flash/w25qxx.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ── 全局校准数据 ── */
static TouchCalibrationData_t g_calib;
static int g_calib_loaded = 0;

#define CALIB_MAGIC  0x5443414C  /* "TCAL" */

/* ── 五点校准的 LCD 目标坐标 ──
 *  基准方向：ILI9488_ROTATION_270 (480×320 横屏)
 *  FT6336_MapTouch 输出的 mapped 坐标在此空间内 */
static const uint16_t calib_lcd_x[CALIB_POINT_COUNT] = { 40, 440, 240, 40, 440 };
static const uint16_t calib_lcd_y[CALIB_POINT_COUNT] = { 40,  40, 160, 280, 280 };

/* ── 初始化：从 Flash 加载校准数据 ── */
void TouchCalibration_Init(void)
{
    if (W25Qxx_Init() == HAL_OK) {
        W25Qxx_ReadData(CALIB_FLASH_ADDR, (uint8_t *)&g_calib, sizeof(g_calib));
        if (g_calib.magic == CALIB_MAGIC) {
            g_calib_loaded = 1;
            Debug_Print("[CALIB] Loaded from Flash\r\n");
            return;
        }
    }
    /* 默认单位矩阵 */
    memset(&g_calib, 0, sizeof(g_calib));
    g_calib.a[0] = 1.0f;
    g_calib.a[4] = 1.0f;
    g_calib.magic = 0;
    g_calib_loaded = 0;
    Debug_Print("[CALIB] Using default identity matrix\r\n");
}

int TouchCalibration_IsCalibrated(void)
{
    return g_calib_loaded;
}

void TouchCalibration_GetMatrix(float out[6])
{
    memcpy(out, g_calib.a, sizeof(g_calib.a));
}

void TouchCalibration_SetMatrix(const float mat[6])
{
    memcpy(g_calib.a, mat, sizeof(g_calib.a));
    g_calib.magic = CALIB_MAGIC;
    g_calib_loaded = 1;
    /* 跳过 SPI4 Flash 写入（FreeRTOS 任务中 SPI4 可能不可用） */
    Debug_Print("[CALIB] Matrix saved to RAM (Flash write skipped)\r\n");
}

void TouchCalibration_Apply(const TouchPoint *raw, uint16_t *lcd_x, uint16_t *lcd_y)
{
    float rx = (float)raw->x, ry = (float)raw->y;
    *lcd_x = (uint16_t)(g_calib.a[0] * rx + g_calib.a[1] * ry + g_calib.a[2]);
    *lcd_y = (uint16_t)(g_calib.a[3] * rx + g_calib.a[4] * ry + g_calib.a[5]);
}

/* ── 最小二乘法求解 5×3 超定方程组 (X/Y 分别独立求解) ── */
static int solve_5x3(const float A[5][3], const float B[5], float X[3])
{
    float N[3][4];
    int i, j, k;

    for (i = 0; i < 3; i++) {
        for (j = 0; j <= 3; j++) N[i][j] = 0.0f;
    }

    /* N = A^T * A,  rhs = A^T * B */
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            float s = 0.0f;
            for (k = 0; k < 5; k++) s += A[k][i] * A[k][j];
            N[i][j] = s;
        }
        float s = 0.0f;
        for (k = 0; k < 5; k++) s += A[k][i] * B[k];
        N[i][3] = s;
    }

    /* 高斯消元 3x3 */
    for (i = 0; i < 3; i++) {
        int pivot = i;
        float maxv = fabsf(N[i][i]);
        for (k = i + 1; k < 3; k++) {
            if (fabsf(N[k][i]) > maxv) { maxv = fabsf(N[k][i]); pivot = k; }
        }
        if (pivot != i) {
            for (j = 0; j <= 3; j++) { float t = N[i][j]; N[i][j] = N[pivot][j]; N[pivot][j] = t; }
        }
        if (fabsf(N[i][i]) < 1e-10f) return 0;
        float div = N[i][i];
        for (j = i; j <= 3; j++) N[i][j] /= div;
        for (k = 0; k < 3; k++) {
            if (k == i) continue;
            float fac = N[k][i];
            for (j = i; j <= 3; j++) N[k][j] -= fac * N[i][j];
        }
    }
    for (i = 0; i < 3; i++) X[i] = N[i][3];
    return 1;
}

/* ── 五点校准流程（阻塞式，在 Task 中调用） ── */
void TouchCalibration_Start(void)
{
    /* mapped[i] = FT6336_MapTouch(raw) 后的基准坐标 (ROTATION_270, 480×320) */
    TouchPoint raw[CALIB_POINT_COUNT];
    TouchPoint mapped[CALIB_POINT_COUNT];
    int collected = 0;
    char buf[128];
    uint16_t lw, lh;

    lw = LCD_GetWidth();
    lh = LCD_GetHeight();

    Debug_Print("\r\n========================================\r\n");
    Debug_Print("  5-Point Touch Calibration START\r\n");
    {
        char sz[32];
        snprintf(sz, sizeof(sz), "  LCD size: %u x %u\r\n", lw, lh);
        Debug_Print(sz);
    }
    Debug_Print("  Touch each cross on screen...\r\n");
    Debug_Print("========================================\r\n\r\n");

    LCD_FillColor(LCD_BLACK);
    snprintf(buf, sizeof(buf), "5-Point Calibration");
    LCD_DrawString(60, 10, buf, LCD_YELLOW, LCD_BLACK);
    snprintf(buf, sizeof(buf), "Touch each cross...");
    LCD_DrawString(80, 30, buf, LCD_CYAN, LCD_BLACK);

    while (collected < CALIB_POINT_COUNT) {
        uint16_t cx = calib_lcd_x[collected];
        uint16_t cy = calib_lcd_y[collected];

        /* 绘制十字 */
        LCD_FillRect(cx - 20, cy, cx + 20, cy, LCD_RED);
        LCD_FillRect(cx, cy - 20, cx, cy + 20, LCD_RED);
        LCD_FillRect(cx - 4, cy - 4, cx + 4, cy + 4, LCD_WHITE);

        snprintf(buf, sizeof(buf), "Point %d/%d", collected + 1, CALIB_POINT_COUNT);
        LCD_DrawString(10, lh - 20, buf, LCD_GREEN, LCD_BLACK);

        /* 串口提示 */
        {
            char hint[64];
            snprintf(hint, sizeof(hint), "[CALIB] Point %d/%d: LCD(%u,%u) — waiting touch...\r\n",
                     collected + 1, CALIB_POINT_COUNT, cx, cy);
            Debug_Print(hint);
        }

        /* 等待触摸按下，持续输出实时坐标 */
        {
            uint8_t cnt;
            uint32_t last_dbg = 0;
            int has_touch = 0;
            do {
                TouchPoint tmp[2];
                cnt = FT6336_ReadAll(tmp);
                if (cnt > 0) {
                    has_touch = 1;
                    raw[collected].x = tmp[0].x;
                    raw[collected].y = tmp[0].y;
                    FT6336_MapTouch(&tmp[0], &mapped[collected].x, &mapped[collected].y);

                    /* 每200ms输出实时触摸坐标 */
                    if (HAL_GetTick() - last_dbg >= 200) {
                        char dbg[80];
                        snprintf(dbg, sizeof(dbg), "  [TOUCH] raw(%u,%u) mapped(%u,%u)\r\n",
                                 tmp[0].x, tmp[0].y,
                                 mapped[collected].x, mapped[collected].y);
                        Debug_Print(dbg);
                        last_dbg = HAL_GetTick();
                    }
                }
                /* 有触摸后松开 → 确认采集 */
                if (has_touch && cnt == 0) {
                    HAL_Delay(50);
                    if (FT6336_GetTouchCount() == 0) {
                        break;
                    }
                    /* 还没松，继续等 */
                    has_touch = 0;
                }
                HAL_Delay(20);
            } while (1);
        }

        /* 擦除十字 */
        LCD_FillRect(cx - 20, cy, cx + 20, cy, LCD_BLACK);
        LCD_FillRect(cx, cy - 20, cx, cy + 20, LCD_BLACK);
        LCD_FillRect(cx - 4, cy - 4, cx + 4, cy + 4, LCD_BLACK);

        /* 串口回传采集结果 */
        {
            char info[128];
            snprintf(info, sizeof(info),
                     "[CALIB] Point %d COLLECTED:\r\n"
                     "  raw(%u,%u) -> mapped(%u,%u) -> target LCD(%u,%u)\r\n\r\n",
                     collected + 1,
                     raw[collected].x, raw[collected].y,
                     mapped[collected].x, mapped[collected].y,
                     cx, cy);
            Debug_Print(info);
        }
        collected++;
    }

    /* ── 最小二乘法：mapped 坐标 → LCD 目标 ── */
    Debug_Print("[CALIB] Solving affine matrix (mapped -> LCD)...\r\n");
    {
        float AX[5][3], Bx[5], By[5];
        for (int i = 0; i < CALIB_POINT_COUNT; i++) {
            AX[i][0] = (float)mapped[i].x;
            AX[i][1] = (float)mapped[i].y;
            AX[i][2] = 1.0f;
            Bx[i] = (float)calib_lcd_x[i];
            By[i] = (float)calib_lcd_y[i];
        }

        float solX[3], solY[3];
        int okX = solve_5x3(AX, Bx, solX);
        int okY = solve_5x3(AX, By, solY);

        if (!okX || !okY) {
            Debug_Print("[CALIB] ERROR: Matrix singular!\r\n");
        }

        float cal_mat[6];
        cal_mat[0] = solX[0]; cal_mat[1] = solX[1]; cal_mat[2] = solX[2];
        cal_mat[3] = solY[0]; cal_mat[4] = solY[1]; cal_mat[5] = solY[2];

        TouchCalibration_SetMatrix(cal_mat);

        /* 串口输出校准矩阵 */
        {
            char mat_info[128];
            snprintf(mat_info, sizeof(mat_info),
                     "[CALIB] Matrix: [%.4f %.4f %.2f | %.4f %.4f %.2f]\r\n",
                     (double)cal_mat[0], (double)cal_mat[1], (double)cal_mat[2],
                     (double)cal_mat[3], (double)cal_mat[4], (double)cal_mat[5]);
            Debug_Print(mat_info);
        }

        /* 验证：对每个点计算校准后坐标和误差 */
        Debug_Print("[CALIB] Verification:\r\n");
        for (int i = 0; i < CALIB_POINT_COUNT; i++) {
            float mx = (float)mapped[i].x, my = (float)mapped[i].y;
            uint16_t px = (uint16_t)(cal_mat[0]*mx + cal_mat[1]*my + cal_mat[2]);
            uint16_t py = (uint16_t)(cal_mat[3]*mx + cal_mat[4]*my + cal_mat[5]);
            char vfy[100];
            snprintf(vfy, sizeof(vfy),
                     "  P%d: mapped(%u,%u) -> lcd(%u,%u)  target(%u,%u)  err(%+d,%+d)\r\n",
                     i+1, mapped[i].x, mapped[i].y, px, py,
                     calib_lcd_x[i], calib_lcd_y[i],
                     (int)px - (int)calib_lcd_x[i], (int)py - (int)calib_lcd_y[i]);
            Debug_Print(vfy);
        }

        LCD_FillColor(LCD_BLACK);
        snprintf(buf, sizeof(buf), "Calibration Done!");
        LCD_DrawString(80, lh / 2 - 20, buf, LCD_GREEN, LCD_BLACK);
        snprintf(buf, sizeof(buf), "a:%.3f %.3f %.1f", (double)cal_mat[0], (double)cal_mat[1], (double)cal_mat[2]);
        LCD_DrawString(20, lh / 2, buf, LCD_CYAN, LCD_BLACK);
        snprintf(buf, sizeof(buf), "b:%.3f %.3f %.1f", (double)cal_mat[3], (double)cal_mat[4], (double)cal_mat[5]);
        LCD_DrawString(20, lh / 2 + 16, buf, LCD_CYAN, LCD_BLACK);
    }

    Debug_Print("[CALIB] Calibration complete!\r\n");
    Debug_Print("========================================\r\n\r\n");
    HAL_Delay(2000);
}