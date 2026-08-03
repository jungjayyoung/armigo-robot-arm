#ifndef __ILI9341_H
#define __ILI9341_H

#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

// ---------------------------------------------------------------------------
// 핀 제어 매핑 (LCD 2개용 CS 분리 및 공유 핀 설정)
// ---------------------------------------------------------------------------
// LCD 1번 CS: PB0
#define LCD1_CS_PORT    GPIOB
#define LCD1_CS_PIN     GPIO_PIN_0

// LCD 2번 CS: PC5 (수정됨)
#define LCD2_CS_PORT    GPIOC
#define LCD2_CS_PIN     GPIO_PIN_5

// 공유 제어 핀 (RST: PB1, DC: PB10)
#define LCD_RST_PORT    GPIOB
#define LCD_RST_PIN     GPIO_PIN_1

#define LCD_DC_PORT     GPIOB
#define LCD_DC_PIN      GPIO_PIN_10

// CS 제어 매크로 (개별 제어)
#define LCD1_CS_LOW()   HAL_GPIO_WritePin(LCD1_CS_PORT, LCD1_CS_PIN, GPIO_PIN_RESET)
#define LCD1_CS_HIGH()  HAL_GPIO_WritePin(LCD1_CS_PORT, LCD1_CS_PIN, GPIO_PIN_SET)

#define LCD2_CS_LOW()   HAL_GPIO_WritePin(LCD2_CS_PORT, LCD2_CS_PIN, GPIO_PIN_RESET)
#define LCD2_CS_HIGH()  HAL_GPIO_WritePin(LCD2_CS_PORT, LCD2_CS_PIN, GPIO_PIN_SET)

// DC 및 RST 제어 매크로 (공유 제어)
#define LCD_DC_COMMAND() HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_DATA()    HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET)

#define LCD_RST_LOW()    HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET)
#define LCD_RST_HIGH()   HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET)

// ---------------------------------------------------------------------------
// 색상 정의 (RGB565 포맷)
// ---------------------------------------------------------------------------
#define ILI9341_BLACK       0x0000
#define ILI9341_WHITE       0xFFFF
#define ILI9341_RED         0xF800
#define ILI9341_GREEN       0x07E0
#define ILI9341_BLUE        0x001F
#define ILI9341_YELLOW      0xFFE0
#define ILI9341_ORANGE      0xFD20
#define ILI9341_CYAN        0x07FF
#define ILI9341_MAGENTA     0xF81F

// 화면 해상도
#define ILI9341_WIDTH       320
#define ILI9341_HEIGHT      240

// ---------------------------------------------------------------------------
// 주요 함수 프로토타입
// ---------------------------------------------------------------------------
void ILI9341_WriteCommand(uint8_t cmd);
void ILI9341_WriteData(uint8_t data);
void ILI9341_Init(void);
void ILI9341_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void ILI9341_FillScreen(uint16_t color);
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ILI9341_DrawPixelScaled(uint16_t x, uint16_t y, uint16_t color, uint8_t scale);

#endif /* __ILI9341_H */