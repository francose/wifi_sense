#include <string.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_timer.h"

#include "csi_engine.h"
#include "ap_tracker.h"
#include "imu.h"

/* Global CSI state */
csi_state_t g_csi = {
    .hist_idx        = 0,
    .frame_count     = 0,
    .rssi            = 0,
    .noise_floor     = 0,
    .channel         = 0,
    .breathing_power = 0,
    .motion_power    = 0,
    .confidence      = 0,
    .state           = "INIT",
    .bssid           = {0},
    .bssid_rssi      = -100,
    .ssid            = "---",
    .occupancy_estimate = 0,
    .motion_direction   = "NONE",
};

/* Minimal 802.11 header for beacon parsing */
typedef struct {
    uint16_t frame_ctrl;
    uint16_t duration;
    uint8_t  addr1[6];
    uint8_t  addr2[6];
    uint8_t  addr3[6];
    uint16_t seq_ctrl;
} __attribute__((packed)) wifi_hdr_t;

/* Internal state for process_csi */
static float s_prev_amp[CSI_MAX_SC];
static int   s_prev_ok = 0;

/* CSI processing */
static void process_csi(wifi_csi_info_t *info) {
    if (0 && g_imu.device_moving) {
        g_csi.state = "DEV MOVE";
        g_csi.confidence = 0;
        return;
    }
    int nsc = info->len / 2;
    if (nsc > CSI_MAX_SC) {
        nsc = CSI_MAX_SC;
    }
    float amp[CSI_MAX_SC];
    float asum = 0, pdsum = 0;
    int pdcnt = 0;

    for (int i = 0; i < nsc; i++) {
        int8_t re = (int8_t)info->buf[2*i];
        int8_t im = (int8_t)info->buf[2*i+1];
        amp[i] = sqrtf(re*re + im*im);
        asum += amp[i];
        if (s_prev_ok) {
            pdsum += fabsf(amp[i] - s_prev_amp[i]);
            pdcnt++;
        }
    }

    static float savg[CSI_MAX_SC];
    static int sinit = 0;
    if (!sinit) {
        memcpy(savg, amp, sizeof(float)*nsc);
        sinit = 1;
    }
    for (int i = 0; i < nsc; i++) {
        float d = amp[i] - savg[i];
        savg[i] = savg[i]*0.95f + amp[i]*0.05f;
        g_csi.sc_variance[i] = g_csi.sc_variance[i]*0.9f + d*d*0.1f;
    }

    float ma = asum / nsc;
    g_csi.amp_hist[g_csi.hist_idx % CSI_HIST] = ma;
    if (pdcnt > 0) {
        g_csi.phase_hist[g_csi.hist_idx % CSI_HIST] = pdsum / pdcnt;
    }
    g_csi.hist_idx++;
    memcpy(s_prev_amp, amp, sizeof(float)*nsc);
    s_prev_ok = 1;

    float tv = 0, mv = 0;
    for (int i = 0; i < nsc; i++) {
        tv += g_csi.sc_variance[i];
        if (g_csi.sc_variance[i] > mv) {
            mv = g_csi.sc_variance[i];
        }
    }
    g_csi.motion_power    = tv;
    g_csi.breathing_power = pdcnt > 0 ? pdsum / pdcnt : 0;
    float av = tv / nsc;

    if (av < 0.5f && g_csi.breathing_power < 0.5f) {
        g_csi.state      = "EMPTY";
        g_csi.confidence = 1.0f - av;
    } else if (av > 3.0f || mv > 10.0f) {
        g_csi.state      = "MOTION";
        g_csi.confidence = av / 10.0f;
    } else if (g_csi.breathing_power > 0.3f && av < 2.0f) {
        g_csi.state      = "STILL";
        g_csi.confidence = g_csi.breathing_power / 2.0f;
    } else {
        g_csi.state      = "PRESENCE";
        g_csi.confidence = av / 5.0f;
    }
    if (g_csi.confidence > 1.0f) {
        g_csi.confidence = 1.0f;
    }

    /* Occupancy estimation */
    int high_var_count = 0;
    float mean_var = tv / nsc;
    for (int i = 0; i < nsc; i++) {
        if (g_csi.sc_variance[i] > mean_var * 2.0f) {
            high_var_count++;
        }
    }
    if (high_var_count < 3) {
        g_csi.occupancy_estimate = 0;
    } else if (high_var_count < 8) {
        g_csi.occupancy_estimate = 1;
    } else if (high_var_count < 16) {
        g_csi.occupancy_estimate = 2;
    } else {
        g_csi.occupancy_estimate = 3;
    }

    /* Motion direction from subcarrier group variance */
    float gv[3] = {0, 0, 0};
    for (int i = 0; i < 16; i++) {
        gv[0] += g_csi.sc_variance[i];
    }
    for (int i = 16; i < 48; i++) {
        gv[1] += g_csi.sc_variance[i];
    }
    for (int i = 48; i < nsc; i++) {
        gv[2] += g_csi.sc_variance[i];
    }
    gv[0] /= 16.0f;
    gv[1] /= 32.0f;
    gv[2] /= (nsc > 48 ? (float)(nsc - 48) : 16.0f);

    static float prev_gv[3] = {0, 0, 0};
    float d0 = gv[0] - prev_gv[0];
    float d2 = gv[2] - prev_gv[2];
    prev_gv[0] = gv[0];
    prev_gv[1] = gv[1];
    prev_gv[2] = gv[2];
    g_csi.sc_group_var[0] = gv[0];
    g_csi.sc_group_var[1] = gv[1];
    g_csi.sc_group_var[2] = gv[2];

    if (fabsf(d0) < 0.1f && fabsf(d2) < 0.1f) {
        g_csi.motion_direction = "NONE";
    } else if (d0 > d2 + 0.2f) {
        g_csi.motion_direction = "LEFT";
    } else if (d2 > d0 + 0.2f) {
        g_csi.motion_direction = "RIGHT";
    } else if (d0 > 0 && d2 > 0) {
        g_csi.motion_direction = "TOWARD";
    } else {
        g_csi.motion_direction = "AWAY";
    }
}

/* CSI callback */
static void csi_cb(void *ctx, wifi_csi_info_t *info) {
    (void)ctx;

    ap_tracker_tick();

    if (!info || !info->buf || info->len <= 0) {
        return;
    }
    g_csi.frame_count++;
    g_csi.rssi        = info->rx_ctrl.rssi;
    g_csi.noise_floor = info->rx_ctrl.noise_floor;
    g_csi.channel     = info->rx_ctrl.channel;

    static int64_t last = 0;
    int64_t now = esp_timer_get_time();
    if ((now - last) < 50000) {
        return;
    }
    last = now;

    /* Track strongest source BSSID */
    if (g_csi.rssi > g_csi.bssid_rssi) {
        memcpy(g_csi.bssid, info->mac, 6);
        g_csi.bssid_rssi = g_csi.rssi;
    }
    {
        static int64_t ld = 0;
        int64_t n2 = esp_timer_get_time();
        if ((n2 - ld) > 2000000) {
            g_csi.bssid_rssi = -100;
            ld = n2;
        }
    }

    process_csi(info);

    float mean_var = 0;
    for (int i = 0; i < 64; i++) {
        mean_var += g_csi.sc_variance[i];
    }
    mean_var /= 64.0f;
    ap_tracker_update(info->mac, g_csi.ssid, g_csi.rssi, g_csi.channel, mean_var);

    printf("CSI,%d,%d,%d,%d,", g_csi.rssi, g_csi.channel, g_csi.noise_floor, info->len);
    for (int i = 0; i < info->len; i++) {
        printf("%02x", (uint8_t)info->buf[i]);
    }
    printf("\n");
    fflush(stdout);
}

/* Promiscuous callback */
static void prom_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT) {
        return;
    }
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    if (pkt->rx_ctrl.sig_len < 36) {
        return;
    }
    wifi_hdr_t *hdr = (wifi_hdr_t *)pkt->payload;
    uint16_t fc = hdr->frame_ctrl;
    /* Beacon = type 0, subtype 8: frame_ctrl & 0xFC == 0x80 */
    if ((fc & 0xFC) != 0x80) {
        return;
    }
    /* Fixed params (timestamp 8 + interval 2 + capability 2) = 12 bytes after header */
    int offset = sizeof(wifi_hdr_t) + 12;
    if (offset + 2 > pkt->rx_ctrl.sig_len) {
        return;
    }
    uint8_t *ie = pkt->payload + offset;
    int remaining = pkt->rx_ctrl.sig_len - offset - 4; /* minus FCS */
    /* Walk IEs looking for SSID (tag 0) */
    while (remaining >= 2) {
        uint8_t tag = ie[0];
        uint8_t len = ie[1];
        if (remaining < 2 + len) {
            break;
        }
        if (tag == 0 && len > 0 && len < 33) {
            /* Check if this beacon is from our tracked BSSID */
            if (memcmp(hdr->addr3, g_csi.bssid, 6) == 0) {
                memcpy(g_csi.ssid, ie + 2, len);
                g_csi.ssid[len] = 0;
            }
            break;
        }
        ie += 2 + len;
        remaining -= 2 + len;
    }
}

/* Engine init */
void csi_engine_init(void) {
    ap_tracker_init();

    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(prom_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    wifi_csi_config_t cc = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = false,
        .manu_scale        = false,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&cc));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}
