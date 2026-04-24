#include "touch.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TOUCH_SDA   GPIO_NUM_11
#define TOUCH_SCL   GPIO_NUM_10
#define TOUCH_RST   GPIO_NUM_13
#define TOUCH_ADDR  0x15
#define I2C_PORT    I2C_NUM_0

static const char *TAG = "touch";

void touch_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << TOUCH_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(TOUCH_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(TOUCH_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

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

    ESP_LOGI(TAG, "CST816T touch initialized (I2C addr 0x%02X)", TOUCH_ADDR);
}

touch_gesture_t touch_read(void) {
    uint8_t reg = 0x01;
    uint8_t gesture = 0;
    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT, TOUCH_ADDR, &reg, 1, &gesture, 1, pdMS_TO_TICKS(50)
    );
    if (err != ESP_OK) {
        return GESTURE_NONE;
    }
    switch (gesture) {
        case 0x05: return GESTURE_TAP;
        case 0x03: return GESTURE_SWIPE_LEFT;
        case 0x04: return GESTURE_SWIPE_RIGHT;
        case 0x01: return GESTURE_SWIPE_UP;
        case 0x02: return GESTURE_SWIPE_DOWN;
        default:   return GESTURE_NONE;
    }
}
