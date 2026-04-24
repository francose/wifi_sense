#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "lcd.h"
#include "csi_engine.h"
#include "channel_hop.h"
#include "touch.h"
#include "imu.h"
#include "ui_state.h"
#include "ui_pages.h"

static const char *TAG = "csi_sense";

static void imu_timer_cb(void *arg)
{
    (void)arg;
    imu_read();
}

/* UI task — handles touch navigation and page rendering */
static void ui_task(void *arg)
{
    (void)arg;
    lcd_rect(0, 0, LCD_W, LCD_H, C_BLK);

    while (1)
    {
        touch_gesture_t g = touch_read();
        if (g == GESTURE_SWIPE_LEFT)
        {
            g_ui.current_page = (g_ui.current_page + 1) % UI_NUM_PAGES;
            lcd_rect(0, 0, LCD_W, LCD_H, C_BLK);
        }
        else if (g == GESTURE_SWIPE_RIGHT)
        {
            g_ui.current_page = (g_ui.current_page + UI_NUM_PAGES - 1) % UI_NUM_PAGES;
            lcd_rect(0, 0, LCD_W, LCD_H, C_BLK);
        }
        else if (g == GESTURE_TAP && g_ui.current_page == 4)
        {
            g_ui.alert_enabled = !g_ui.alert_enabled;
        }

        switch (g_ui.current_page)
        {
            case 0: ui_page_overview(); break;
            case 1: ui_page_waveforms(); break;
            case 2: ui_page_ap_list(); break;
            case 3: ui_page_imu(); break;
            case 4: ui_page_alert(); break;
            default: break;
        }
        ui_draw_nav_bar();

        /* Alert border flash */
        if (g_ui.alert_enabled && strcmp(g_csi.state, "MOTION") == 0)
        {
            g_ui.alert_triggered = 1;
            lcd_rect(0, Y_START, LCD_W, 3, C_RED);
            lcd_rect(0, 257, LCD_W, 3, C_RED);
            lcd_rect(0, Y_START, 3, 225, C_RED);
            lcd_rect(237, Y_START, 3, 225, C_RED);
        }
        else
        {
            g_ui.alert_triggered = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* Main */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "wifi_sense CSI + LCD starting...");

    lcd_init();
    lcd_str(30, 130, "STARTING...", C_WHT, 2);

    /* Touch + IMU init (touch first for I2C) */
    touch_init();
    imu_init();

    /* IMU read timer at 20Hz */
    esp_timer_handle_t imu_timer;
    esp_timer_create_args_t imu_args = {
        .callback = imu_timer_cb,
        .name = "imu",
    };
    ESP_ERROR_CHECK(esp_timer_create(&imu_args, &imu_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(imu_timer, 50000));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    uint8_t hop_channels[] = {1, 6, 11};
    channel_hop_init(hop_channels, 3, 500);
    channel_hop_start();

    csi_engine_init();

    ESP_LOGI(TAG, "CSI active — UI + serial");

    lcd_rect(0, 0, LCD_W, LCD_H, C_BLK);
    xTaskCreatePinnedToCore(ui_task, "ui", 8192, NULL, 5, NULL, 1);

    while (1)
    {
        ESP_LOGI(TAG, "frames: %lu", (unsigned long)g_csi.frame_count);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
