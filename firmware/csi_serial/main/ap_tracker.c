#include <string.h>
#include "esp_timer.h"
#include "ap_tracker.h"

ap_tracker_t g_ap_tracker;

void ap_tracker_init(void) {
    memset(&g_ap_tracker, 0, sizeof(g_ap_tracker));
}

void ap_tracker_update(const uint8_t *bssid, const char *ssid,
                       int rssi, int channel, float variance) {
    int64_t now = esp_timer_get_time();
    int i;

    /* Check if BSSID already tracked */
    for (i = 0; i < AP_MAX_TRACKED; i++) {
        if (g_ap_tracker.aps[i].active &&
            memcmp(g_ap_tracker.aps[i].bssid, bssid, 6) == 0) {
            g_ap_tracker.aps[i].rssi         = rssi;
            g_ap_tracker.aps[i].channel      = channel;
            g_ap_tracker.aps[i].variance     = variance;
            g_ap_tracker.aps[i].last_seen_us = now;
            g_ap_tracker.aps[i].frame_count++;
            if (ssid && ssid[0] != '\0') {
                strncpy(g_ap_tracker.aps[i].ssid, ssid, 32);
                g_ap_tracker.aps[i].ssid[32] = '\0';
            }
            return;
        }
    }

    /* Not found — find an empty slot */
    if (g_ap_tracker.count < AP_MAX_TRACKED) {
        for (i = 0; i < AP_MAX_TRACKED; i++) {
            if (!g_ap_tracker.aps[i].active) {
                memcpy(g_ap_tracker.aps[i].bssid, bssid, 6);
                if (ssid && ssid[0] != '\0') {
                    strncpy(g_ap_tracker.aps[i].ssid, ssid, 32);
                    g_ap_tracker.aps[i].ssid[32] = '\0';
                } else {
                    g_ap_tracker.aps[i].ssid[0] = '\0';
                }
                g_ap_tracker.aps[i].rssi         = rssi;
                g_ap_tracker.aps[i].channel      = channel;
                g_ap_tracker.aps[i].variance     = variance;
                g_ap_tracker.aps[i].last_seen_us = now;
                g_ap_tracker.aps[i].frame_count  = 1;
                g_ap_tracker.aps[i].active       = 1;
                g_ap_tracker.count++;
                return;
            }
        }
    }

    /* Table full — replace weakest entry if new one is stronger */
    int weakest_idx = 0;
    int weakest_rssi = g_ap_tracker.aps[0].rssi;
    for (i = 1; i < AP_MAX_TRACKED; i++) {
        if (g_ap_tracker.aps[i].rssi < weakest_rssi) {
            weakest_rssi = g_ap_tracker.aps[i].rssi;
            weakest_idx  = i;
        }
    }
    if (rssi > weakest_rssi) {
        memcpy(g_ap_tracker.aps[weakest_idx].bssid, bssid, 6);
        if (ssid && ssid[0] != '\0') {
            strncpy(g_ap_tracker.aps[weakest_idx].ssid, ssid, 32);
            g_ap_tracker.aps[weakest_idx].ssid[32] = '\0';
        } else {
            g_ap_tracker.aps[weakest_idx].ssid[0] = '\0';
        }
        g_ap_tracker.aps[weakest_idx].rssi         = rssi;
        g_ap_tracker.aps[weakest_idx].channel      = channel;
        g_ap_tracker.aps[weakest_idx].variance     = variance;
        g_ap_tracker.aps[weakest_idx].last_seen_us = now;
        g_ap_tracker.aps[weakest_idx].frame_count  = 1;
        g_ap_tracker.aps[weakest_idx].active       = 1;
    }
}

void ap_tracker_tick(void) {
    int64_t now = esp_timer_get_time();
    int i;

    for (i = 0; i < AP_MAX_TRACKED; i++) {
        if (!g_ap_tracker.aps[i].active) {
            continue;
        }
        int64_t age = now - g_ap_tracker.aps[i].last_seen_us;
        if (age > 30000000) {
            /* Inactive for >30s — remove entry */
            memset(&g_ap_tracker.aps[i], 0, sizeof(tracked_ap_t));
            if (g_ap_tracker.count > 0) {
                g_ap_tracker.count--;
            }
        } else if (age > 5000000) {
            /* Inactive for >5s — mark inactive */
            g_ap_tracker.aps[i].active = 0;
        }
    }
}

const tracked_ap_t *ap_tracker_strongest(void) {
    const tracked_ap_t *best = NULL;
    int i;

    for (i = 0; i < AP_MAX_TRACKED; i++) {
        if (!g_ap_tracker.aps[i].active) {
            continue;
        }
        if (best == NULL || g_ap_tracker.aps[i].rssi > best->rssi) {
            best = &g_ap_tracker.aps[i];
        }
    }
    return best;
}
