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
    uint16_t x, y;
} TouchPoint;

HAL_StatusTypeDef FT6336_Init(void);
uint8_t         FT6336_ReadAll(TouchPoint *points);
uint8_t         FT6336_GetTouchCount(void);
uint8_t         FT6336_GetGesture(void);
uint8_t         FT6336_ReadChipID(void);
void            FT6336_Test(void);

#ifdef __cplusplus
}
#endif

#endif