#include "imu.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

#define IMU_ADDR    0x6B
#define I2C_PORT    I2C_NUM_0

static const char *TAG = "imu";

imu_data_t g_imu = {0};

static esp_err_t imu_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_PORT, IMU_ADDR, buf, 2, pdMS_TO_TICKS(50));
}

static esp_err_t imu_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_PORT, IMU_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(50));
}

void imu_init(void) {
    uint8_t who = 0;
    esp_err_t err = imu_read_reg(0x00, &who, 1);
    if (err != ESP_OK || who != 0x05) {
        ESP_LOGW(TAG, "QMI8658 not found (who_am_i=0x%02X, err=%d)", who, err);
        return;
    }

    imu_write_reg(0x03, 0x15);  // CTRL2: accel +-4g 119Hz
    imu_write_reg(0x04, 0x54);  // CTRL3: gyro +-512dps 119Hz
    imu_write_reg(0x08, 0x03);  // CTRL7: enable accel + gyro

    ESP_LOGI(TAG, "QMI8658 IMU initialized");
}

void imu_read(void) {
    uint8_t raw[12];
    esp_err_t err = imu_read_reg(0x35, raw, 12);
    if (err != ESP_OK) {
        return;
    }

    int16_t ax = (int16_t)(raw[0]  | (raw[1]  << 8));
    int16_t ay = (int16_t)(raw[2]  | (raw[3]  << 8));
    int16_t az = (int16_t)(raw[4]  | (raw[5]  << 8));
    int16_t gx = (int16_t)(raw[6]  | (raw[7]  << 8));
    int16_t gy = (int16_t)(raw[8]  | (raw[9]  << 8));
    int16_t gz = (int16_t)(raw[10] | (raw[11] << 8));

    g_imu.accel_x = ax * (4.0f / 32768.0f);
    g_imu.accel_y = ay * (4.0f / 32768.0f);
    g_imu.accel_z = az * (4.0f / 32768.0f);
    g_imu.gyro_x  = gx * (512.0f / 32768.0f);
    g_imu.gyro_y  = gy * (512.0f / 32768.0f);
    g_imu.gyro_z  = gz * (512.0f / 32768.0f);

    float mag = sqrtf(g_imu.accel_x * g_imu.accel_x +
                      g_imu.accel_y * g_imu.accel_y +
                      g_imu.accel_z * g_imu.accel_z);

    static float ema_mag = 1.0f;
    ema_mag = ema_mag * 0.9f + mag * 0.1f;
    g_imu.device_motion = fabsf(mag - ema_mag);

    float gyro_mag = sqrtf(g_imu.gyro_x * g_imu.gyro_x +
                           g_imu.gyro_y * g_imu.gyro_y +
                           g_imu.gyro_z * g_imu.gyro_z);
    if (g_imu.device_motion > 0.3f || gyro_mag > 30.0f) {
        g_imu.device_moving = 1;
    } else {
        g_imu.device_moving = 0;
    }
}
