#ifndef AP_TRACKER_H
#define AP_TRACKER_H

#include <stdint.h>

#define AP_MAX_TRACKED 5

typedef struct {
    uint8_t bssid[6];
    char    ssid[33];
    int     rssi;
    int     channel;
    float   variance;
    int64_t last_seen_us;
    uint32_t frame_count;
    uint8_t active;
} tracked_ap_t;

typedef struct {
    tracked_ap_t aps[AP_MAX_TRACKED];
    int count;
} ap_tracker_t;

extern ap_tracker_t g_ap_tracker;

void ap_tracker_init(void);
void ap_tracker_update(const uint8_t *bssid, const char *ssid,
                       int rssi, int channel, float variance);
void ap_tracker_tick(void);
const tracked_ap_t *ap_tracker_strongest(void);

#endif
