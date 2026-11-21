#ifndef TOUCH_CALIBRATION_H
#define TOUCH_CALIBRATION_H

#include "main.h"
#include "ft6336.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 仿射变换校准参数 (6 个 float) ── */
typedef struct {
    float a[6];           /* X_lcd = a[0]*x + a[1]*y + a[2]; Y_lcd = a[3]*x + a[4]*y + a[5] */
    uint32_t magic;       /* 校验魔术字 0x5443414C ("TCAL") */
} TouchCalibrationData_t;

/* ── 五点校准十字坐标（LCD 坐标） ── */
#define CALIB_POINT_COUNT   5

/* ── W25Q Flash 校准数据存储地址（最后一页，避免与示波器快照冲突） ── */
#define CALIB_FLASH_ADDR    0x1FF000   /* W25Q16 2MB 末尾 4KB */

/* ── 公共 API ── */
void TouchCalibration_Init(void);                             /* 初始化，从 Flash 加载 */
int  TouchCalibration_IsCalibrated(void);                     /* 是否已校准 */
void TouchCalibration_GetMatrix(float out[6]);                /* 获取当前校准矩阵 */
void TouchCalibration_SetMatrix(const float mat[6]);          /* 设置校准矩阵并保存到 Flash */
void TouchCalibration_Apply(const TouchPoint *raw, uint16_t *lcd_x, uint16_t *lcd_y);
void TouchCalibration_Start(void);                            /* 启动五点校准流程（阻塞） */

#ifdef __cplusplus
}
#endif

#endif