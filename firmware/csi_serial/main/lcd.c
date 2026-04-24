#include "lcd.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

/* ── Pin definitions (internal) ── */
#define LCD_MOSI   GPIO_NUM_7
#define LCD_CLK    GPIO_NUM_6
#define LCD_CS     GPIO_NUM_5
#define LCD_DC     GPIO_NUM_4
#define LCD_RST    GPIO_NUM_8
#define LCD_BL     GPIO_NUM_15

static spi_device_handle_t s_spi;

/* ── SPI primitives ── */

static void lcd_cmd(uint8_t cmd) {
    gpio_set_level(LCD_DC, 0);
    spi_transaction_t t = { .length = 8, .tx_buffer = &cmd };
    spi_device_transmit(s_spi, &t);
}

static void lcd_dat(const uint8_t *d, int len) {
    if (!len) return;
    gpio_set_level(LCD_DC, 1);
    spi_transaction_t t = { .length = len * 8, .tx_buffer = d };
    spi_device_transmit(s_spi, &t);
}

static void lcd_d8(uint8_t v) { lcd_dat(&v, 1); }

static void lcd_window(int x0, int y0, int x1, int y1) {
    lcd_cmd(0x2A);
    uint8_t c[] = {x0>>8, x0&0xFF, x1>>8, x1&0xFF};
    lcd_dat(c, 4);
    lcd_cmd(0x2B);
    uint8_t r[] = {y0>>8, y0&0xFF, y1>>8, y1&0xFF};
    lcd_dat(r, 4);
    lcd_cmd(0x2C);
}

void lcd_rect(int x, int y, int w, int h, uint16_t col) {
    if (w <= 0 || h <= 0 || x >= LCD_W || y >= LCD_H) return;
    if (x + w > LCD_W) w = LCD_W - x;
    if (y + h > LCD_H) h = LCD_H - y;
    lcd_window(x, y, x + w - 1, y + h - 1);
    gpio_set_level(LCD_DC, 1);
    uint8_t row[LCD_W * 2];
    uint8_t hi = col >> 8, lo = col & 0xFF;
    for (int i = 0; i < w; i++) { row[i*2] = hi; row[i*2+1] = lo; }
    for (int r = 0; r < h; r++) {
        spi_transaction_t t = { .length = w * 16, .tx_buffer = row };
        spi_device_transmit(s_spi, &t);
    }
}

/* ── 5x7 font — minimal set ── */
static const uint8_t fnt[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* space */
    {0x00,0x00,0x5F,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00},
    {0x08,0x2A,0x1C,0x2A,0x08}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x00,0x08,0x14,0x22,0x41}, {0x14,0x14,0x14,0x14,0x14},
    {0x41,0x22,0x14,0x08,0x00}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x01,0x01},
    {0x3E,0x41,0x41,0x51,0x32}, {0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x04,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F},
    {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43},
};

void lcd_char(int x, int y, char ch, uint16_t fg, int sc) {
    int idx = 0;
    if (ch >= ' ' && ch <= 'Z') idx = ch - ' ';
    else if (ch >= 'a' && ch <= 'z') idx = ch - 'a' + ('A' - ' ');
    for (int c = 0; c < 5; c++) {
        uint8_t line = fnt[idx][c];
        for (int r = 0; r < 7; r++) {
            if (line & (1 << r))
                lcd_rect(x + c*sc, y + r*sc, sc, sc, fg);
        }
    }
}

void lcd_str(int x, int y, const char *s, uint16_t fg, int sc) {
    while (*s) { lcd_char(x, y, *s, fg, sc); x += 6*sc; s++; }
}

void lcd_bar(int x, int y, int w, int h, float val, float mx,
             uint16_t fg, uint16_t bg) {
    int f = (int)(val / (mx > 0 ? mx : 1) * w);
    if (f > w) f = w;
    if (f < 0) f = 0;
    lcd_rect(x, y, f, h, fg);
    lcd_rect(x+f, y, w-f, h, bg);
}

void lcd_wave(int x, int y, int w, int h, float *d, int n, uint16_t col) {
    if (n < 2) { lcd_rect(x, y, w, h, C_BLK); return; }
    float mn = d[0];
    float mx = d[0];
    for (int i = 1; i < n; i++) {
        if (d[i]<mn) mn=d[i];
        if (d[i]>mx) mx=d[i];
    }
    float rng = mx - mn; if (rng < 0.01f) rng = 1;
    int step = n > w ? n/w : 1;
    int cols = n/step; if (cols > w) cols = w;
    for (int i = 0; i < cols; i++) {
        int bh = (int)((d[i*step]-mn)/rng*(h-2))+1;
        lcd_rect(x+i, y, 1, h-bh, C_BLK);
        lcd_rect(x+i, y+h-bh, 1, bh, col);
    }
    if (cols < w) lcd_rect(x+cols, y, w-cols, h, C_BLK);
}

void lcd_set_brightness(uint8_t level) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, level);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

/* ── LCD init ── */
void lcd_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<LCD_DC)|(1ULL<<LCD_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    ledc_timer_config_t lt = {
        .speed_mode=LEDC_LOW_SPEED_MODE, .timer_num=LEDC_TIMER_0,
        .duty_resolution=LEDC_TIMER_8_BIT, .freq_hz=5000,
    };
    ledc_timer_config(&lt);
    ledc_channel_config_t lc = {
        .speed_mode=LEDC_LOW_SPEED_MODE, .channel=LEDC_CHANNEL_0,
        .timer_sel=LEDC_TIMER_0, .gpio_num=LCD_BL, .duty=200,
    };
    ledc_channel_config(&lc);

    spi_bus_config_t bus = {
        .mosi_io_num=LCD_MOSI, .sclk_io_num=LCD_CLK,
        .miso_io_num=-1, .quadwp_io_num=-1, .quadhd_io_num=-1,
        .max_transfer_sz=LCD_W*2*20,
    };
    spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    spi_device_interface_config_t dev = {
        .clock_speed_hz=40000000, .mode=0, .spics_io_num=LCD_CS, .queue_size=7,
    };
    spi_bus_add_device(SPI2_HOST, &dev, &s_spi);

    gpio_set_level(LCD_RST, 0); vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_RST, 1); vTaskDelay(pdMS_TO_TICKS(120));

    lcd_cmd(0x11); vTaskDelay(pdMS_TO_TICKS(120));
    lcd_cmd(0x36); lcd_d8(0x00);
    lcd_cmd(0x3A); lcd_d8(0x05);
    lcd_cmd(0x21);
    lcd_cmd(0x29); vTaskDelay(pdMS_TO_TICKS(20));

    /* Clear entire RAM including hidden areas */
    lcd_rect(0, 0, LCD_W, LCD_H, C_BLK);
}
