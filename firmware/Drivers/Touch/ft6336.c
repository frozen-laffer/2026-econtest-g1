#include "ft6336.h"
#include "i2c.h"
#include "../debug_print/debug_print.h"
#include "../TFT_Screen/ILI9488.h"
#include "../ADC/adc_driver.h"
//#include "../W5500/w5500.h"
#include <stdio.h>
#include "gpio.h"

#define FT6336_TIMEOUT   50

extern void Debug_Print(const char *str);
extern void ADC_DisplayOnLCD(void);

static HAL_StatusTypeDef ft6336_read(uint8_t reg, uint8_t *data, uint8_t len) {
    HAL_StatusTypeDef ret;
    uint8_t tx = reg;
    ret = HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)(FT6336_ADDR << 1), &tx, 1, FT6336_TIMEOUT);
    if (ret != HAL_OK) return ret;
    HAL_Delay(1);
    ret = HAL_I2C_Master_Receive(&hi2c3, (uint16_t)(FT6336_ADDR << 1), data, len, FT6336_TIMEOUT);
    return ret;
}

static void FT6336_HardwareReset(void) {
    HAL_GPIO_WritePin(T_RST_GPIO_Port, T_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(T_RST_GPIO_Port, T_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(T_RST_GPIO_Port, T_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(300);
}

HAL_StatusTypeDef FT6336_Init(void) {
    uint8_t id;
    char msg[64];
    FT6336_HardwareReset();

    if (HAL_I2C_IsDeviceReady(&hi2c3, FT6336_ADDR << 1, 3, 100) != HAL_OK) {
        snprintf(msg, sizeof(msg), "[FT6336] Device not ready at 0x%02X\r\n", FT6336_ADDR << 1);
        Debug_Print(msg);
        return HAL_ERROR;
    }

    id = FT6336_ReadChipID();
    if (id == 0x00 || id == 0xFF) {
        Debug_Print("[FT6336] Invalid Chip ID\r\n");
        return HAL_ERROR;
    }
    snprintf(msg, sizeof(msg), "[FT6336] Init OK, Chip ID: 0x%02X\r\n", id);
    Debug_Print(msg);
    return HAL_OK;
}

uint8_t FT6336_ReadChipID(void) {
    uint8_t buf[2] = {0};
    if (ft6336_read(0xA8, buf, 2) == HAL_OK && buf[0] != 0x00 && buf[0] != 0xFF) return buf[0];
    if (ft6336_read(0xA6, buf, 1) == HAL_OK && buf[0] != 0x00 && buf[0] != 0xFF) return buf[0];
    if (ft6336_read(0xA3, buf, 1) == HAL_OK && buf[0] != 0x00 && buf[0] != 0xFF) return buf[0];
    return 0;
}

uint8_t FT6336_GetTouchCount(void) {
    uint8_t status;
    ft6336_read(FT6336_REG_TD_STATUS, &status, 1);
    return status & 0x0F;
}

uint8_t FT6336_GetGesture(void) {
    uint8_t gesture;
    ft6336_read(FT6336_REG_GEST_ID, &gesture, 1);
    return gesture;
}

static void ft6336_get_touch_raw(uint8_t idx, uint8_t *buf) {
    uint8_t reg = (idx == 0) ? 0x03 : 0x09;
    ft6336_read(reg, buf, 6);
}

// ── 270° 固定映射 ──
#define CAL_COUNT 5
static const uint16_t cal_x[CAL_COUNT] = {40, 440, 240, 40, 440};
static const uint16_t cal_y[CAL_COUNT] = {40,  40, 160, 280, 280};

void FT6336_Test(void) {
    char buf[128];
    Debug_Print("\r\n=== FT6336 5-Point Calibration ===\r\n");

    LCD_Init();
    LCD_SetRotation(ILI9488_ROTATION_270);
    LCD_FillColor(LCD_BLACK);

    uint16_t raw_x[CAL_COUNT] = {0}, raw_y[CAL_COUNT] = {0};

    for (uint8_t step = 0; step < CAL_COUNT; step++) {
        uint16_t cx = cal_x[step], cy = cal_y[step];
        LCD_FillColor(LCD_BLACK);
        snprintf(buf, sizeof(buf), "Step %d/5  LCD(%d,%d)", step + 1, cx, cy);
        LCD_DrawString(10, 10, buf, LCD_YELLOW, LCD_BLACK);
        LCD_DrawString(10, 30, "Tap the cross", LCD_WHITE, LCD_BLACK);

        LCD_FillRect(cx - 20, cy, cx + 20, cy, LCD_CYAN);
        LCD_FillRect(cx, cy - 20, cx, cy + 20, LCD_CYAN);
        LCD_FillRect(cx - 3, cy - 3, cx + 3, cy + 3, LCD_RED);

        uint8_t got = 0;
        while (!got) {
            if (FT6336_GetTouchCount() > 0) {
                uint8_t reg[6];
                ft6336_get_touch_raw(0, reg);
                raw_x[step] = reg[0];
                raw_y[step] = reg[1];
                snprintf(buf, sizeof(buf), "  Regs: [0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X], touch(%u,%u)\r\n",
                         reg[0], reg[1], reg[2], reg[3], reg[4], reg[5], raw_x[step], raw_y[step]);
                Debug_Print(buf);
                got = 1;
            }
            HAL_Delay(20);
        }
        HAL_Delay(300);
    }

    LCD_FillColor(LCD_BLACK);
    LCD_DrawString(10, 10, "Calib Done!", LCD_GREEN, LCD_BLACK);
    Debug_Print("\r\n=== CALIB DATA ===\r\n");
    for (uint8_t i = 0; i < CAL_COUNT; i++) {
        snprintf(buf, sizeof(buf), "P%d: LCD(%d,%d) -> Reg(%d,%d)\r\n", i+1, cal_x[i], cal_y[i], raw_x[i], raw_y[i]);
        Debug_Print(buf);
    }

    /* 进入颜色测试 + 旋转按钮 */
    Debug_Print("\r\n[INFO] Starting color test with rotation...\r\n");
    HAL_Delay(500);

    { // block for local variables
        uint16_t colors[] = {LCD_RED, LCD_GREEN, LCD_BLUE, LCD_WHITE};
        const char *labels[] = {"RED", "GRN", "BLU", "WHT"};
        const uint16_t btn_h = 60, gap = 20;
        const uint16_t rbtn_w = 120, rbtn_h = 50;
        uint16_t bg = LCD_BLACK;

        ILI9488_Rotation rots[] = {ILI9488_ROTATION_90, ILI9488_ROTATION_180, ILI9488_ROTATION_270, ILI9488_ROTATION_0};
        const char *rot_names[] = {"90", "180", "270", "0"};
        uint8_t cur_rot = 0;

        LCD_SetRotation(rots[cur_rot]);

        uint16_t lw = LCD_GetWidth(), lh = LCD_GetHeight();

        uint16_t btn_x = (lw < 480) ? 10 : 20;
        uint16_t btn_w = (lw < 480) ? (uint16_t)(lw - 20) : 120;
        uint16_t rbtn_x = lw - 130, rbtn_y = lh - 70;

        uint16_t osc_btn_x = lw - 130, osc_btn_y = lh - 130;
        uint16_t osc_btn_w = 120, osc_btn_h = 40;

        /* 初始 UI 绘制 */
        LCD_FillColor(bg);
        { int i;
        for (i = 0; i < 4; i++) {
            uint16_t y = 30 + i * (btn_h + gap);
            LCD_FillRect(btn_x, y, (uint16_t)(btn_x + btn_w - 1), (uint16_t)(y + btn_h - 1), colors[i]);
            LCD_DrawString(btn_x + 30, y + 18, (char*)labels[i], LCD_BLACK, colors[i]);
        }}
        LCD_DrawString(160, 40, "Last: --", LCD_YELLOW, bg);
        LCD_FillRect(rbtn_x, rbtn_y, rbtn_x + rbtn_w, rbtn_y + rbtn_h, LCD_CYAN);
        LCD_DrawString(rbtn_x + 10, rbtn_y + 15, (char*)rot_names[cur_rot], LCD_BLACK, LCD_CYAN);
        LCD_DrawString(rbtn_x + 25, rbtn_y + 15, " ROT", LCD_BLACK, LCD_CYAN);
        LCD_FillRect(osc_btn_x, osc_btn_y, osc_btn_x + osc_btn_w, osc_btn_y + osc_btn_h, LCD_MAGENTA);
        LCD_DrawString(osc_btn_x + 20, osc_btn_y + 12, "OSCILLO", LCD_WHITE, LCD_MAGENTA);

        TouchPoint pts[2];
        static uint32_t osc_touch_start = 0;
        while (1) {
            uint8_t cnt = FT6336_ReadAll(pts);
            uint8_t i;
            for (i = 0; i < cnt && i < 2; i++) {
                uint16_t raw_x = pts[i].x, raw_y = pts[i].y;
                uint16_t lcd_x, lcd_y;

                switch (rots[cur_rot]) {
                    case ILI9488_ROTATION_0:
                        lcd_x = raw_x;
                        lcd_y = raw_y;
                        break;
                    case ILI9488_ROTATION_90:
                        lcd_x = raw_y;
                        lcd_y = lh - raw_x;
                        break;
                    case ILI9488_ROTATION_180:
                        lcd_x = lw - raw_x;
                        lcd_y = lh - raw_y;
                        break;
                    default:
                        lcd_x = lw - raw_y;
                        lcd_y = raw_x;
                        break;
                }

                /* 示波器按钮检测 */
                if (lcd_x >= osc_btn_x && lcd_x <= osc_btn_x + osc_btn_w &&
                    lcd_y >= osc_btn_y && lcd_y <= osc_btn_y + osc_btn_h) {
                    Debug_Print("[OSC] Entering oscilloscope mode\r\n");
                    LCD_FillColor(LCD_BLACK);
                    LCD_DrawString(10, 10, "Loading Scope...", LCD_WHITE, LCD_BLACK);

                    /* [N6] 严禁在此调用 ADC_Driver_Init()：
                     * 它会 HAL_ADC_DeInit(&hadc1/2/5) 并按扫描/连续模式重建，
                     * 直接摧毁 main.c 里三路交替（TRGO/TRGO2/CC1 触发 + 2.5 周期采样）的配置。
                     * 采集统一由 Drivers/App/scope_acq.c 负责，本分支只做显示。
                     * ADC_Driver_Init();  ← 已停用 */
                    /* W5500 已由 main.c 初始化，此处不再重复初始化，保持 Socket 连续 */

                    static uint32_t last_send = 0;
                    while (1) {
                        ADC_DisplayOnLCD();

#if 0 /* TODO: W5500 Oscilloscope TCP Send -- 待 w5500_self.c 编译后启用 */
                        if (HAL_GetTick() - last_send >= 50) {
                            extern int16_t adc1_buf[1024], adc2_buf[1024];
                            uint8_t pkt[512]; int len = 0;
                            for (int s = 0; s < 128 && len < 508; s++) {
                                uint16_t idx = (s * 8) % 1024;
                                int16_t v1 = adc1_buf[idx], v2 = adc2_buf[idx];
                                len += snprintf((char *)pkt + len, (size_t)(512 - len),
                                                "%d,%d\n", (int)v1, (int)v2);
                            }
                            W5500_SendData(pkt, (uint16_t)len);
                            last_send = HAL_GetTick();
                        }
#endif

                        if (FT6336_GetTouchCount() > 0) {
                            if (osc_touch_start == 0) osc_touch_start = HAL_GetTick();
                            else if (HAL_GetTick() - osc_touch_start >= 2000) {
                                Debug_Print("[OSC] Exiting oscilloscope mode\r\n");
                                osc_touch_start = 0;
                                break;
                            }
                        } else {
                            osc_touch_start = 0;
                        }
                        HAL_Delay(5);
                    }
                    /* 退回颜色测试界面，重绘 */
                    LCD_FillColor(bg);
                    { int j;
                    for (j = 0; j < 4; j++) {
                        uint16_t y = 30 + j * (btn_h + gap);
                        LCD_FillRect(btn_x, y, (uint16_t)(btn_x + btn_w - 1), (uint16_t)(y + btn_h - 1), colors[j]);
                        LCD_DrawString(btn_x + 30, y + 18, (char*)labels[j], LCD_BLACK, colors[j]);
                    }}
                    LCD_DrawString(160, 40, "Last: --", LCD_YELLOW, bg);
                    LCD_FillRect(rbtn_x, rbtn_y, rbtn_x + rbtn_w, rbtn_y + rbtn_h, LCD_CYAN);
                    LCD_DrawString(rbtn_x + 10, rbtn_y + 15, (char*)rot_names[cur_rot], LCD_BLACK, LCD_CYAN);
                    LCD_DrawString(rbtn_x + 25, rbtn_y + 15, " ROT", LCD_BLACK, LCD_CYAN);
                    LCD_FillRect(osc_btn_x, osc_btn_y, osc_btn_x + osc_btn_w, osc_btn_y + osc_btn_h, LCD_MAGENTA);
                    LCD_DrawString(osc_btn_x + 20, osc_btn_y + 12, "OSCILLO", LCD_WHITE, LCD_MAGENTA);
                    break;
                }

                /* 旋转按钮检测 */
                if (lcd_x >= rbtn_x && lcd_x <= rbtn_x + rbtn_w &&
                    lcd_y >= rbtn_y && lcd_y <= rbtn_y + rbtn_h) {
                    cur_rot = (cur_rot + 1) % 4;
                    LCD_SetRotation(rots[cur_rot]);

                    lw = LCD_GetWidth(); lh = LCD_GetHeight();
                    btn_x = (lw < 480) ? 10 : 20;
                    btn_w = (lw < 480) ? (uint16_t)(lw - 20) : 120;
                    rbtn_x = lw - 130; rbtn_y = lh - 70;
                    osc_btn_x = lw - 130; osc_btn_y = lh - 130;

                    LCD_FillColor(bg);
                    { int j;
                    for (j = 0; j < 4; j++) {
                        uint16_t y = 30 + j * (btn_h + gap);
                        LCD_FillRect(btn_x, y, (uint16_t)(btn_x + btn_w - 1), (uint16_t)(y + btn_h - 1), colors[j]);
                        LCD_DrawString(btn_x + 30, y + 18, (char*)labels[j], LCD_BLACK, colors[j]);
                    }}
                    LCD_DrawString(160, 40, "Last: --", LCD_YELLOW, bg);
                    LCD_FillRect(rbtn_x, rbtn_y, rbtn_x + rbtn_w, rbtn_y + rbtn_h, LCD_CYAN);
                    LCD_DrawString(rbtn_x + 10, rbtn_y + 15, (char*)rot_names[cur_rot], LCD_BLACK, LCD_CYAN);
                    LCD_DrawString(rbtn_x + 25, rbtn_y + 15, " ROT", LCD_BLACK, LCD_CYAN);
                    LCD_FillRect(osc_btn_x, osc_btn_y, osc_btn_x + osc_btn_w, osc_btn_y + osc_btn_h, LCD_MAGENTA);
                    LCD_DrawString(osc_btn_x + 20, osc_btn_y + 12, "OSCILLO", LCD_WHITE, LCD_MAGENTA);

                    { char rinfo[32]; snprintf(rinfo, sizeof(rinfo), "[ROT] Switched to %s deg\r\n", rot_names[cur_rot]); Debug_Print(rinfo); }
                    break;
                }

                /* 颜色按键检测 */
                { int b;
                for (b = 0; b < 4; b++) {
                    uint16_t by = 30 + b * (btn_h + gap);
                    if (lcd_x >= btn_x && lcd_x <= btn_x + btn_w && lcd_y >= by && lcd_y <= by + btn_h) {
                        bg = colors[b];
                        LCD_FillColor(bg);
                        { int j;
                        for (j = 0; j < 4; j++) {
                            uint16_t yy = 30 + j * (btn_h + gap);
                            LCD_FillRect(btn_x, yy, (uint16_t)(btn_x + btn_w - 1), (uint16_t)(yy + btn_h - 1), colors[j]);
                            LCD_DrawString(btn_x + 30, yy + 18, (char*)labels[j], LCD_BLACK, colors[j]);
                        }}
                        { char bi[32]; snprintf(bi, sizeof(bi), "Last: %s", labels[b]);
                          LCD_DrawString(160, 40, bi, LCD_YELLOW, bg); }
                        LCD_FillRect(rbtn_x, rbtn_y, rbtn_x + rbtn_w, rbtn_y + rbtn_h, LCD_CYAN);
                        LCD_DrawString(rbtn_x + 10, rbtn_y + 15, (char*)rot_names[cur_rot], LCD_BLACK, LCD_CYAN);
                        LCD_DrawString(rbtn_x + 25, rbtn_y + 15, " ROT", LCD_BLACK, LCD_CYAN);
                        LCD_FillRect(osc_btn_x, osc_btn_y, osc_btn_x + osc_btn_w, osc_btn_y + osc_btn_h, LCD_MAGENTA);
                        LCD_DrawString(osc_btn_x + 20, osc_btn_y + 12, "OSCILLO", LCD_WHITE, LCD_MAGENTA);
                    }
                }}
            }
            HAL_Delay(30);
        }
    }
}

uint8_t FT6336_ReadAll(TouchPoint *points) {
    uint8_t count = FT6336_GetTouchCount();
    if (count > 2) count = 2;
    for (uint8_t i = 0; i < count; i++) {
        uint8_t buf[6];
        ft6336_get_touch_raw(i, buf);
        points[i].event = buf[0] >> 6;
        points[i].x = (uint16_t)(((buf[0] & 0x0F) << 8) | buf[1]);
        points[i].y = (uint16_t)(((buf[2] & 0x0F) << 8) | buf[3]);
    }
    return count;
}

void FT6336_SetRotation(Touch_Rotation r) {}
void FT6336_SetLCDSize(uint16_t w, uint16_t h) {}
Touch_Rotation FT6336_GetRotation(void) { return TOUCH_ROTATION_0; }
void FT6336_MapTouch(const TouchPoint *raw, uint16_t *lcd_x, uint16_t *lcd_y) {
    *lcd_x = 480 - raw->y;
    *lcd_y = raw->x;
}