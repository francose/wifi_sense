#include <string.h>
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "channel_hop.h"

static const char *TAG = "ch_hop";

#define MAX_CHANNELS 14

static uint8_t s_channels[MAX_CHANNELS];
static int     s_count;
static int     s_index;
static uint32_t s_dwell_ms;
static esp_timer_handle_t s_timer;
static int     s_running;

static void hop_timer_cb(void *arg)
{
    (void)arg;
    s_index = (s_index + 1) % s_count;
    ESP_LOGD(TAG, "hopping to channel %d", s_channels[s_index]);
    esp_wifi_set_channel(s_channels[s_index], WIFI_SECOND_CHAN_NONE);
}

void channel_hop_init(const uint8_t *channels, int count, uint32_t dwell_ms)
{
    if (count > MAX_CHANNELS) {
        count = MAX_CHANNELS;
    }
    memcpy(s_channels, channels, (size_t)count);
    s_count    = count;
    s_index    = 0;
    s_dwell_ms = dwell_ms;
    s_running  = 0;
    s_timer    = NULL;
}

void channel_hop_start(void)
{
    if (s_count <= 1) {
        return;
    }
    if (s_running) {
        return;
    }

    esp_timer_create_args_t args = {
        .callback        = hop_timer_cb,
        .arg             = NULL,
        .name            = "ch_hop",
        .dispatch_method = ESP_TIMER_TASK,
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &s_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_timer, (uint64_t)s_dwell_ms * 1000ULL));
    s_running = 1;
    ESP_LOGI(TAG, "channel hopping started, %d channels, %lu ms dwell",
             s_count, (unsigned long)s_dwell_ms);
}

void channel_hop_stop(void)
{
    if (!s_running) {
        return;
    }
    esp_timer_stop(s_timer);
    esp_timer_delete(s_timer);
    s_timer   = NULL;
    s_running = 0;
    ESP_LOGI(TAG, "channel hopping stopped");
}

int channel_hop_current(void)
{
    return (int)s_channels[s_index];
}
