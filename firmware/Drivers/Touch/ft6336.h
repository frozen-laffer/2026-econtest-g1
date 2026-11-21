#ifndef FT6336_H
#define FT6336_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FT6336_ADDR         0x38
#define FT6336_REG_TD_STATUS   0x02
#define FT6336_REG_P1_XH       0x03
#define FT6336_REG_GEST_ID     0x01

typedef struct {
    uint8_t  event;
    uint16_t x, y;          /* 原始触摸坐标 */
} TouchPoint;

/* 触摸旋转枚举（与 LCD 旋转一致） */
typedef enum {
    TOUCH_ROTATION_0   = 0,
    TOUCH_ROTATION_90  = 1,
    TOUCH_ROTATION_180 = 2,
    TOUCH_ROTATION_270 = 3
} Touch_Rotation;

#define TOUCH_DEFAULT_LCD_W  480
#define TOUCH_DEFAULT_LCD_H  320

HAL_StatusTypeDef FT6336_Init(void);
uint8_t         FT6336_ReadAll(TouchPoint *points);
uint8_t         FT6336_GetTouchCount(void);
uint8_t         FT6336_GetGesture(void);
uint8_t         FT6336_ReadChipID(void);
void            FT6336_Test(void);

/* ── 统一触摸映射接口 ── */
void FT6336_SetRotation(Touch_Rotation rotation);
void FT6336_SetLCDSize(uint16_t w, uint16_t h);
void FT6336_MapTouch(const TouchPoint *raw, uint16_t *lcd_x, uint16_t *lcd_y);
Touch_Rotation FT6336_GetRotation(void);

#ifdef __cplusplus
}
#endif

#endif