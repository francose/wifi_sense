#ifndef CSI_ENGINE_H
#define CSI_ENGINE_H

#include <stdint.h>
#include "esp_wifi_types.h"

#define CSI_HIST   64
#define CSI_MAX_SC 64

typedef struct {
    float    amp_hist[CSI_HIST];
    float    phase_hist[CSI_HIST];
    int      hist_idx;
    uint32_t frame_count;
    int      rssi;
    int      noise_floor;
    int      channel;
    float    sc_variance[CSI_MAX_SC];
    float    breathing_power;
    float    motion_power;
    float    confidence;
    const char *state;
    uint8_t  bssid[6];
    int      bssid_rssi;
    char     ssid[33];
    int      occupancy_estimate;
    const char *motion_direction;
    float    sc_group_var[3];
} csi_state_t;

extern csi_state_t g_csi;

void csi_engine_init(void);

#endif
