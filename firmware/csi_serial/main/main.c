#include <string.h>
#include <stdio.h>
#include "driver/gpio.h"
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

/*
 * UI task: polls touch at 30ms for responsiveness,
 * redraws display at ~200ms (every 7th loop).
 */
static void ui_task(void *arg)
{
    (void)arg;
    lcd_rect(0, 0, LCD_W, LCD_H, C_BLK);

    int redraw_counter = 0;

    while (1) {
        /* Fast touch poll — 30ms to catch CST816D wake window */
        touch_gesture_t g = touch_read();

        if (g == GESTURE_SWIPE_LEFT) {
            g_ui.current_page = (g_ui.current_page + 1) % UI_NUM_PAGES;
            lcd_rect(0, 0, LCD_W, LCD_H, C_BLK);
            redraw_counter = 6;  /* force immediate redraw */
        } else if (g == GESTURE_SWIPE_RIGHT) {
            g_ui.current_page = (g_ui.current_page + UI_NUM_PAGES - 1) % UI_NUM_PAGES;
            lcd_rect(0, 0, LCD_W, LCD_H, C_BLK);
            redraw_counter = 6;
        } else if (g == GESTURE_TAP) {
            if (g_ui.current_page == 4) {
                g_ui.alert_enabled = !g_ui.alert_enabled;
            }
            redraw_counter = 6;  /* force redraw on tap too */
        }

        /* Redraw display every ~210ms (7 * 30ms) */
        redraw_counter++;
        if (redraw_counter >= 7) {
            redraw_counter = 0;

            switch (g_ui.current_page) {
                case 0: ui_page_overview(); break;
                case 1: ui_page_waveforms(); break;
                case 2: ui_page_ap_list(); break;
                case 3: ui_page_imu(); break;
                case 4: ui_page_alert(); break;
                default: break;
            }
            ui_draw_nav_bar();

            /* Alert border flash */
            if (g_ui.alert_enabled && strcmp(g_csi.state, "MOTION") == 0) {
                g_ui.alert_triggered = 1;
                lcd_rect(0, Y_START, LCD_W, 3, C_RED);
                lcd_rect(0, 257, LCD_W, 3, C_RED);
                lcd_rect(0, Y_START, 3, 225, C_RED);
                lcd_rect(237, Y_START, 3, 225, C_RED);
            } else {
                g_ui.alert_triggered = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "wifi_sense CSI + LCD starting...");

    lcd_init();
    lcd_str(30, 130, "STARTING...", C_WHT, 2);

    /* Touch + IMU init */
    touch_init();
    imu_init();

    /* WiFi */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Channel hopping */
    uint8_t hop_channels[] = {1, 6, 11};
    channel_hop_init(hop_channels, 3, 500);
    channel_hop_start();

    /* CSI */
    csi_engine_init();

    /* IMU timer at 20Hz */
    esp_timer_handle_t imu_timer;
    esp_timer_create_args_t imu_args = {
        .callback = imu_timer_cb,
        .name = "imu",
    };
    ESP_ERROR_CHECK(esp_timer_create(&imu_args, &imu_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(imu_timer, 50000));

    ESP_LOGI(TAG, "CSI active — LCD + serial + touch");

    lcd_rect(0, 0, LCD_W, LCD_H, C_BLK);
    xTaskCreatePinnedToCore(ui_task, "ui", 8192, NULL, 5, NULL, 1);

    while (1) {
        ESP_LOGI(TAG, "frames: %lu page: %d",
                 (unsigned long)g_csi.frame_count, g_ui.current_page);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
