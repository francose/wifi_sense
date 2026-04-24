#include <string.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

static const char *TAG = "csi_sense";

#define LCD_MOSI   GPIO_NUM_7
#define LCD_CLK    GPIO_NUM_6
#define LCD_CS     GPIO_NUM_5
#define LCD_DC     GPIO_NUM_4
#define LCD_RST    GPIO_NUM_8
#define LCD_BL     GPIO_NUM_15

#define LCD_W  240
#define LCD_H  280

/* Colors RGB565 */
#define C_BLK   0x0000
#define C_WHT   0xFFFF
#define C_RED   0xF800
#define C_GRN   0x07E0
#define C_CYN   0x07FF
#define C_YEL   0xFFE0
#define C_BLU   0x001F
#define C_DKG   0x4208
#define C_LTG   0x8410

static spi_device_handle_t s_spi;

/* CSI state */
#define HIST 64
static float s_amp_hist[HIST];
static float s_phase_hist[HIST];
static int   s_hidx = 0;
static uint32_t s_fcnt = 0;
static int   s_rssi = 0, s_noise = 0, s_chan = 0;
static float s_sc_var[64];
static float s_breath = 0, s_motion = 0;
static float s_prev_amp[64];
static int   s_prev_ok = 0;
static const char *s_state = "INIT";
static float s_conf = 0;

/* ── LCD primitives ── */

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

static void lcd_rect(int x, int y, int w, int h, uint16_t col) {
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

/* 5x7 font — minimal set */
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

static void lcd_char(int x, int y, char ch, uint16_t fg, int sc) {
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

static void lcd_str(int x, int y, const char *s, uint16_t fg, int sc) {
    while (*s) { lcd_char(x, y, *s, fg, sc); x += 6*sc; s++; }
}

static void lcd_bar(int x, int y, int w, int h, float val, float mx,
                    uint16_t fg, uint16_t bg) {
    int f = (int)(val / (mx > 0 ? mx : 1) * w);
    if (f > w) f = w;
    if (f < 0) f = 0;
    lcd_rect(x, y, f, h, fg);
    lcd_rect(x+f, y, w-f, h, bg);
}

static void lcd_wave(int x, int y, int w, int h, float *d, int n, uint16_t col) {
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

/* ── LCD init ── */
static void lcd_init(void) {
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

/* ── CSI processing ── */
static void process_csi(wifi_csi_info_t *info) {
    int nsc = info->len / 2;
    if (nsc > 64) nsc = 64;
    float amp[64];
    float asum = 0, pdsum = 0;
    int pdcnt = 0;

    for (int i = 0; i < nsc; i++) {
        int8_t re = (int8_t)info->buf[2*i];
        int8_t im = (int8_t)info->buf[2*i+1];
        amp[i] = sqrtf(re*re + im*im);
        asum += amp[i];
        if (s_prev_ok) { pdsum += fabsf(amp[i] - s_prev_amp[i]); pdcnt++; }
    }

    static float savg[64]; static int sinit = 0;
    if (!sinit) { memcpy(savg, amp, sizeof(float)*nsc); sinit = 1; }
    for (int i = 0; i < nsc; i++) {
        float d = amp[i] - savg[i];
        savg[i] = savg[i]*0.95f + amp[i]*0.05f;
        s_sc_var[i] = s_sc_var[i]*0.9f + d*d*0.1f;
    }

    float ma = asum / nsc;
    s_amp_hist[s_hidx % HIST] = ma;
    if (pdcnt > 0) s_phase_hist[s_hidx % HIST] = pdsum / pdcnt;
    s_hidx++;
    memcpy(s_prev_amp, amp, sizeof(float)*nsc);
    s_prev_ok = 1;

    float tv = 0, mv = 0;
    for (int i = 0; i < nsc; i++) { tv += s_sc_var[i]; if (s_sc_var[i]>mv) mv = s_sc_var[i]; }
    s_motion = tv;
    s_breath = pdcnt > 0 ? pdsum / pdcnt : 0;
    float av = tv / nsc;

    if (av < 0.5f && s_breath < 0.5f) { s_state = "EMPTY"; s_conf = 1.0f - av; }
    else if (av > 3.0f || mv > 10.0f) { s_state = "MOTION"; s_conf = av / 10.0f; }
    else if (s_breath > 0.3f && av < 2.0f) { s_state = "STILL"; s_conf = s_breath / 2.0f; }
    else { s_state = "PRESENCE"; s_conf = av / 5.0f; }
    if (s_conf > 1.0f) s_conf = 1.0f;
}

/* ── CSI callback ── */
static void csi_cb(void *ctx, wifi_csi_info_t *info) {
    (void)ctx;
    if (!info || !info->buf || info->len <= 0) return;
    s_fcnt++;
    s_rssi = info->rx_ctrl.rssi;
    s_noise = info->rx_ctrl.noise_floor;
    s_chan = info->rx_ctrl.channel;

    static int64_t last = 0;
    int64_t now = esp_timer_get_time();
    if ((now - last) < 50000) return;
    last = now;

    process_csi(info);

    printf("CSI,%d,%d,%d,%d,", s_rssi, s_chan, s_noise, info->len);
    for (int i = 0; i < info->len; i++) printf("%02x", (uint8_t)info->buf[i]);
    printf("\n");
    fflush(stdout);
}

static void prom_cb(void *b, wifi_promiscuous_pkt_type_t t) { (void)b; (void)t; }

/* ── Display task ── */
/* Content starts at y=35 to account for the rounded bezel */
#define Y_START 35

static void lcd_task(void *arg) {
    (void)arg;
    char buf[40];

    /* One-time full clear */
    lcd_rect(0, 0, LCD_W, LCD_H, C_BLK);
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {
        int y = Y_START;

        /* Header */
        lcd_rect(0, y, LCD_W, 16, C_DKG);
        lcd_str(30, y+1, "WIFI SENSE", C_CYN, 2);
        y += 18;

        /* Stats */
        lcd_rect(0, y, LCD_W, 8, C_BLK);
        snprintf(buf, sizeof(buf), "FR:%lu CH:%d RSSI:%d SNR:%d",
                 (unsigned long)s_fcnt, s_chan, s_rssi, s_rssi - s_noise);
        lcd_str(4, y, buf, C_LTG, 1);
        y += 10;

        /* State */
        uint16_t sc = C_GRN;
        if (strcmp(s_state,"MOTION")==0) sc = C_RED;
        else if (strcmp(s_state,"STILL")==0) sc = C_YEL;
        else if (strcmp(s_state,"PRESENCE")==0) sc = C_CYN;

        lcd_rect(0, y, LCD_W, 18, C_DKG);
        lcd_str(8, y+2, s_state, sc, 2);
        snprintf(buf, sizeof(buf), "%d%%", (int)(s_conf*100));
        lcd_str(185, y+5, buf, C_WHT, 1);
        y += 20;

        /* SC variance heatmap */
        lcd_str(4, y, "SC VAR", C_LTG, 1);
        y += 9;
        float smx = 0.01f;
        for (int i=0;i<64;i++) { if (s_sc_var[i]>smx) smx=s_sc_var[i]; }
        for (int i=0;i<64;i++) {
            float n = s_sc_var[i]/smx;
            uint16_t c = C_DKG;
            if (n>=0.25f) c=C_BLU;
            if (n>=0.5f) c=C_CYN;
            if (n>=0.75f) c=C_GRN;
            if (n>0.9f) c=C_RED;
            int bx = 8+i*3, bh = (int)(n*12)+1;
            lcd_rect(bx, y+12-bh, 3, bh, c);
            lcd_rect(bx, y, 3, 12-bh, C_BLK);
        }
        y += 14;

        /* Bars */
        lcd_str(4, y, "BREATH", C_LTG, 1);
        lcd_bar(50, y, 180, 5, s_breath, 5.0f, C_GRN, C_DKG);
        y += 7;
        lcd_str(4, y, "MOTION", C_LTG, 1);
        lcd_bar(50, y, 180, 5, s_motion/64.0f, 10.0f, C_RED, C_DKG);
        y += 9;

        /* Amplitude waveform */
        lcd_str(4, y, "AMP", C_CYN, 1);
        y += 8;
        int np = s_hidx < HIST ? s_hidx : HIST;
        if (np > 2) {
            float ord[HIST];
            int st = s_hidx % HIST;
            for (int i=0;i<np;i++) ord[i] = s_amp_hist[(st+i)%HIST];
            lcd_wave(4, y, 232, 28, ord, np, C_CYN);
        } else lcd_rect(4, y, 232, 28, C_BLK);
        y += 30;

        /* Phase waveform */
        lcd_str(4, y, "PHASE", C_GRN, 1);
        y += 8;
        if (np > 2) {
            float ord[HIST];
            int st = s_hidx % HIST;
            for (int i=0;i<np;i++) ord[i] = s_phase_hist[(st+i)%HIST];
            lcd_wave(4, y, 232, 28, ord, np, C_GRN);
        } else lcd_rect(4, y, 232, 28, C_BLK);
        y += 30;

        /* Clear below */
        if (y < LCD_H) lcd_rect(0, y, LCD_W, LCD_H-y, C_BLK);

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/* ── Main ── */
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "wifi_sense CSI + LCD starting...");

    lcd_init();
    lcd_str(30, 130, "STARTING...", C_WHT, 2);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(prom_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    wifi_csi_config_t cc = {
        .lltf_en=true, .htltf_en=true, .stbc_htltf2_en=true,
        .ltf_merge_en=true, .channel_filter_en=false, .manu_scale=false,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&cc));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    ESP_LOGI(TAG, "CSI active — LCD + serial");

    lcd_rect(0, 0, LCD_W, LCD_H, C_BLK);
    xTaskCreatePinnedToCore(lcd_task, "lcd", 8192, NULL, 5, NULL, 1);

    while (1) {
        ESP_LOGI(TAG, "frames: %lu", (unsigned long)s_fcnt);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
