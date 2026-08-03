#include "ili9341.h"

// SPI 로우 레벨 바이트 전송
static void ILI9341_SPI_Send(uint8_t data) {
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
}

// 명령 전송
void ILI9341_WriteCommand(uint8_t cmd) {
    LCD_DC_COMMAND();
    ILI9341_SPI_Send(cmd);
}

// 데이터 전송
void ILI9341_WriteData(uint8_t data) {
    LCD_DC_DATA();
    ILI9341_SPI_Send(data);
}

// 주소 창 영역 지정 (Draw 영역 설정)
void ILI9341_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    ILI9341_WriteCommand(0x2A); // Column Address Set
    ILI9341_WriteData(x0 >> 8);
    ILI9341_WriteData(x0 & 0xFF);
    ILI9341_WriteData(x1 >> 8);
    ILI9341_WriteData(x1 & 0xFF);

    ILI9341_WriteCommand(0x2B); // Page Address Set
    ILI9341_WriteData(y0 >> 8);
    ILI9341_WriteData(y0 & 0xFF);
    ILI9341_WriteData(y1 >> 8);
    ILI9341_WriteData(y1 & 0xFF);

    ILI9341_WriteCommand(0x2C); // Memory Write
}

// ILI9341 제어 레지스터 초기화
void ILI9341_Init(void) {
    // 하드웨어 리셋
    // LCD_RST_LOW();
    // HAL_Delay(20);
    // LCD_RST_HIGH();
    // HAL_Delay(120);

    // 소프트웨어 초기화 시퀀스
    ILI9341_WriteCommand(0x01); // Software Reset
    HAL_Delay(100);

    ILI9341_WriteCommand(0xCB);
    ILI9341_WriteData(0x39);
    ILI9341_WriteData(0x2C);
    ILI9341_WriteData(0x00);
    ILI9341_WriteData(0x34);
    ILI9341_WriteData(0x02);

    ILI9341_WriteCommand(0xCF);
    ILI9341_WriteData(0x00);
    ILI9341_WriteData(0xC1);
    ILI9341_WriteData(0x30);

    ILI9341_WriteCommand(0xE8);
    ILI9341_WriteData(0x85);
    ILI9341_WriteData(0x00);
    ILI9341_WriteData(0x78);

    ILI9341_WriteCommand(0xEA);
    ILI9341_WriteData(0x00);
    ILI9341_WriteData(0x00);

    ILI9341_WriteCommand(0xED);
    ILI9341_WriteData(0x64);
    ILI9341_WriteData(0x03);
    ILI9341_WriteData(0x12);
    ILI9341_WriteData(0x81);

    ILI9341_WriteCommand(0xF7);
    ILI9341_WriteData(0x20);

    ILI9341_WriteCommand(0xC0); // Power Control 1
    ILI9341_WriteData(0x23);

    ILI9341_WriteCommand(0xC1); // Power Control 2
    ILI9341_WriteData(0x10);

    ILI9341_WriteCommand(0xC5); // VCOM Control 1
    ILI9341_WriteData(0x3E);
    ILI9341_WriteData(0x28);

    ILI9341_WriteCommand(0xC7); // VCOM Control 2
    ILI9341_WriteData(0x86);

    /* Memory Access Control: landscape + 180 degree rotation for both LCDs. */
    ILI9341_WriteCommand(0x36);
    ILI9341_WriteData(0xE8);

    ILI9341_WriteCommand(0x3A); // COLMOD: Pixel Format Set (RGB565)
    ILI9341_WriteData(0x55);

    ILI9341_WriteCommand(0xB1);
    ILI9341_WriteData(0x00);
    ILI9341_WriteData(0x18);

    ILI9341_WriteCommand(0xB6); // Display Function Control
    ILI9341_WriteData(0x08);
    ILI9341_WriteData(0x82);
    ILI9341_WriteData(0x27);

    ILI9341_WriteCommand(0x11); // Exit Sleep Mode
    HAL_Delay(120);

    ILI9341_WriteCommand(0x29); // Display ON
    HAL_Delay(20);
}

// 화면 전체 채우기
void ILI9341_FillScreen(uint16_t color) {
    ILI9341_SetAddressWindow(0, 0, ILI9341_WIDTH - 1, ILI9341_HEIGHT - 1);

    LCD_DC_DATA();
    uint8_t color_high = color >> 8;
    uint8_t color_low = color & 0xFF;
    uint8_t pixels[128]; /* 64 RGB565 pixels per SPI transaction. */

    for (uint16_t i = 0U; i < sizeof(pixels); i += 2U) {
        pixels[i] = color_high;
        pixels[i + 1U] = color_low;
    }

    uint32_t bytes_remaining = (uint32_t)ILI9341_WIDTH * ILI9341_HEIGHT * 2U;
    while (bytes_remaining > 0U) {
        uint16_t length = (bytes_remaining > sizeof(pixels)) ?
                          (uint16_t)sizeof(pixels) : (uint16_t)bytes_remaining;
        (void)HAL_SPI_Transmit(&hspi1, pixels, length, HAL_MAX_DELAY);
        bytes_remaining -= length;
    }
}

// 픽셀 1개 찍기
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if (x >= ILI9341_WIDTH || y >= ILI9341_HEIGHT) return;

    ILI9341_SetAddressWindow(x, y, x, y);
    LCD_DC_DATA();
    ILI9341_SPI_Send(color >> 8);
    ILI9341_SPI_Send(color & 0xFF);
}

// 스케일링(배율) 적용 픽셀 찍기
void ILI9341_DrawPixelScaled(uint16_t x, uint16_t y, uint16_t color, uint8_t scale) {
    if (scale == 1) {
        ILI9341_DrawPixel(x, y, color);
        return;
    }

    if (x + scale > ILI9341_WIDTH || y + scale > ILI9341_HEIGHT) return;

    ILI9341_SetAddressWindow(x, y, x + scale - 1, y + scale - 1);
    LCD_DC_DATA();
    uint8_t color_high = color >> 8;
    uint8_t color_low = color & 0xFF;

    for (uint16_t i = 0; i < scale * scale; i++) {
        ILI9341_SPI_Send(color_high);
        ILI9341_SPI_Send(color_low);
    }
}
