#include "touch.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TOUCH_SDA   GPIO_NUM_11
#define TOUCH_SCL   GPIO_NUM_10
#define TOUCH_RST   GPIO_NUM_13
#define TOUCH_ADDR  0x15
#define I2C_PORT    I2C_NUM_0

#define SWIPE_THRESH 30
#define TAP_THRESH   15

static const char *TAG = "touch";

static int s_active = 0;
static int s_sx = 0, s_sy = 0;
static int s_lx = 0, s_ly = 0;
static int64_t s_last_touch = 0;

static esp_err_t touch_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(
        I2C_PORT, TOUCH_ADDR, buf, 2, pdMS_TO_TICKS(100)
    );
}

/*
 * The CST816D goes to sleep ~5 seconds after last touch.
 * When asleep, I2C reads return NACK. A touch wakes it for ~1-2 seconds.
 * Our poll must be fast enough to catch data during that wake window.
 *
 * Strategy: poll at high frequency (every ~30ms in the task loop).
 * When we get a NACK, we know the chip is asleep — skip until next touch.
 * When we get data, read fast to track the finger movement.
 *
 * Also: periodically write DisAutoSleep (0xFE=0x01) to keep it awake
 * while actively being used.
 */

void touch_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << TOUCH_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    gpio_set_level(TOUCH_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    gpio_set_level(TOUCH_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(300));

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TOUCH_SDA,
        .scl_io_num = TOUCH_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);

    /* Disable auto-sleep */
    touch_write_reg(0xFE, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Enable gesture detection */
    touch_write_reg(0xEC, 0x07);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "CST816D touch init (fast polling, auto-sleep disabled)");
}

touch_gesture_t touch_read(void)
{
    uint8_t reg = 0x01;
    uint8_t data[6] = {0};

    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT, TOUCH_ADDR, &reg, 1, data, 6, pdMS_TO_TICKS(10)
    );

    if (err != ESP_OK) {
        /* NACK = asleep. If we had active touch, this means finger lifted */
        if (s_active) {
            s_active = 0;
            int dx = s_lx - s_sx;
            int dy = s_ly - s_sy;
            int adx = dx > 0 ? dx : -dx;
            int ady = dy > 0 ? dy : -dy;

            if (adx < TAP_THRESH && ady < TAP_THRESH) {
                ESP_LOGI(TAG, "TAP (nack)");
                return GESTURE_TAP;
            }
            if (adx > ady && adx >= SWIPE_THRESH) {
                ESP_LOGI(TAG, "SWIPE %s (nack) dx=%d", dx > 0 ? "RIGHT" : "LEFT", dx);
                return dx > 0 ? GESTURE_SWIPE_RIGHT : GESTURE_SWIPE_LEFT;
            }
            if (ady > adx && ady >= SWIPE_THRESH) {
                ESP_LOGI(TAG, "SWIPE %s (nack) dy=%d", dy > 0 ? "DOWN" : "UP", dy);
                return dy > 0 ? GESTURE_SWIPE_DOWN : GESTURE_SWIPE_UP;
            }
        }
        return GESTURE_NONE;
    }

    /* I2C responded — chip is awake */
    uint8_t gesture = data[0];
    uint8_t fingers = data[1];
    int x = ((data[2] & 0x0F) << 8) | data[3];
    int y = ((data[4] & 0x0F) << 8) | data[5];

    /* Keep chip awake by periodically re-disabling auto-sleep */
    int64_t now = esp_timer_get_time();
    if ((now - s_last_touch) > 3000000) {
        touch_write_reg(0xFE, 0x01);
    }

    /* Hardware gesture — CST816D might report these on some fw versions */
    switch (gesture) {
        case 0x01: ESP_LOGI(TAG, "HW SWIPE UP"); return GESTURE_SWIPE_UP;
        case 0x02: ESP_LOGI(TAG, "HW SWIPE DOWN"); return GESTURE_SWIPE_DOWN;
        case 0x03: ESP_LOGI(TAG, "HW SWIPE LEFT"); return GESTURE_SWIPE_LEFT;
        case 0x04: ESP_LOGI(TAG, "HW SWIPE RIGHT"); return GESTURE_SWIPE_RIGHT;
        case 0x05: ESP_LOGI(TAG, "HW TAP"); return GESTURE_TAP;
    }

    /* Software gesture from X/Y tracking */
    if (fingers > 0 && (x > 0 || y > 0)) {
        s_last_touch = now;

        if (!s_active) {
            s_active = 1;
            s_sx = x;
            s_sy = y;
        }
        s_lx = x;
        s_ly = y;
        return GESTURE_NONE;
    }

    /* fingers==0 but chip awake — finger just lifted */
    if (s_active) {
        s_active = 0;
        int dx = s_lx - s_sx;
        int dy = s_ly - s_sy;
        int adx = dx > 0 ? dx : -dx;
        int ady = dy > 0 ? dy : -dy;

        if (adx < TAP_THRESH && ady < TAP_THRESH) {
            ESP_LOGI(TAG, "TAP x=%d y=%d", s_lx, s_ly);
            return GESTURE_TAP;
        }
        if (adx > ady && adx >= SWIPE_THRESH) {
            ESP_LOGI(TAG, "SWIPE %s dx=%d", dx > 0 ? "RIGHT" : "LEFT", dx);
            return dx > 0 ? GESTURE_SWIPE_RIGHT : GESTURE_SWIPE_LEFT;
        }
        if (ady > adx && ady >= SWIPE_THRESH) {
            ESP_LOGI(TAG, "SWIPE %s dy=%d", dy > 0 ? "DOWN" : "UP", dy);
            return dy > 0 ? GESTURE_SWIPE_DOWN : GESTURE_SWIPE_UP;
        }
    }

    return GESTURE_NONE;
}
