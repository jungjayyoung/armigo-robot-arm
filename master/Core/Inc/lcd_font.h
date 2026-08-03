#ifndef __LCD_FONT_H
#define __LCD_FONT_H

#include "stm32f4xx_hal.h"

// 표준 영문/숫자/기호 문자열 출력 함수
void LCD_PutChar(uint16_t x, uint16_t y, char ch, uint16_t color, uint16_t bg_color, uint8_t scale);
void LCD_PutString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg_color, uint8_t scale);

#endif