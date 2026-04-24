#pragma once
#include <stdint.h>

/* ── LCD dimensions ── */
#define LCD_W  240
#define LCD_H  280

/* ── Colors RGB565 ── */
#define C_BLK   0x0000
#define C_WHT   0xFFFF
#define C_RED   0xF800
#define C_GRN   0x07E0
#define C_CYN   0x07FF
#define C_YEL   0xFFE0
#define C_BLU   0x001F
#define C_DKG   0x4208
#define C_LTG   0x8410
#define C_ORG   0xFD20
#define C_MAG   0xF81F

/* ── Public API ── */
void lcd_init(void);
void lcd_rect(int x, int y, int w, int h, uint16_t col);
void lcd_char(int x, int y, char ch, uint16_t fg, int sc);
void lcd_str(int x, int y, const char *s, uint16_t fg, int sc);
void lcd_bar(int x, int y, int w, int h, float val, float mx,
             uint16_t fg, uint16_t bg);
void lcd_wave(int x, int y, int w, int h, float *d, int n, uint16_t col);
void lcd_set_brightness(uint8_t level);
