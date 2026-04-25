#include <string.h>
#include <stdio.h>
#include "lcd.h"
#include "csi_engine.h"
#include "ap_tracker.h"
#include "imu.h"
#include "channel_hop.h"
#include "ui_state.h"
#include "ui_pages.h"

ui_state_t g_ui = {0, 0, 0, 0};

/* ── Signal meter: 10 segments mapping RSSI -90..-30 ── */
void ui_draw_signal_meter(int x, int y, int rssi)
{
    int clamped = rssi;
    if (clamped < -90)
    {
        clamped = -90;
    }
    if (clamped > -30)
    {
        clamped = -30;
    }
    int filled = (clamped + 90) * 10 / 60;  /* 0..10 */
    for (int i = 0; i < 10; i++)
    {
        uint16_t col;
        if (i < filled)
        {
            if (i < 3)
            {
                col = C_RED;
            }
            else if (i < 6)
            {
                col = C_YEL;
            }
            else
            {
                col = C_GRN;
            }
        }
        else
        {
            col = C_DKG;
        }
        lcd_rect(x + i * 12, y, 10, 7, col);
    }
}

/* ── Page 0: Overview (migrated from old lcd_task) ── */
void ui_page_overview(void)
{
    char buf[48];
    int y = Y_START;

    /* Header */
    lcd_rect(0, y, LCD_W, 16, C_DKG);
    lcd_str(30, y + 1, "WIFI SENSE", C_CYN, 2);
    y += 18;

    /* SSID + BSSID */
    lcd_rect(0, y, LCD_W, 18, C_BLK);
    lcd_str(4, y, g_csi.ssid, C_YEL, 1);
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             g_csi.bssid[0], g_csi.bssid[1], g_csi.bssid[2],
             g_csi.bssid[3], g_csi.bssid[4], g_csi.bssid[5]);
    lcd_str(4, y + 9, buf, C_LTG, 1);
    y += 20;

    /* Stats */
    lcd_rect(0, y, LCD_W, 9, C_BLK);
    snprintf(buf, sizeof(buf), "CH:%d RSSI:%d SNR:%d FR:%lu",
             g_csi.channel, g_csi.rssi,
             g_csi.rssi - g_csi.noise_floor,
             (unsigned long)g_csi.frame_count);
    lcd_str(4, y + 1, buf, C_LTG, 1);
    y += 12;

    /* Signal meter */
    ui_draw_signal_meter(4, y, g_csi.rssi);
    y += 10;

    /* State box */
    uint16_t sc = C_GRN;
    if (strcmp(g_csi.state, "MOTION") == 0)
    {
        sc = C_RED;
    }
    else if (strcmp(g_csi.state, "STILL") == 0)
    {
        sc = C_YEL;
    }
    else if (strcmp(g_csi.state, "PRESENCE") == 0)
    {
        sc = C_CYN;
    }
    lcd_rect(0, y, LCD_W, 18, C_DKG);
    lcd_str(8, y + 2, g_csi.state, sc, 2);
    snprintf(buf, sizeof(buf), "%d%%", (int)(g_csi.confidence * 100));
    lcd_str(185, y + 5, buf, C_WHT, 1);
    y += 21;

    /* Direction + Occupancy */
    lcd_rect(0, y, LCD_W, 9, C_BLK);
    snprintf(buf, sizeof(buf), "DIR:%s  OCC:%d", g_csi.motion_direction, g_csi.occupancy_estimate);
    lcd_str(4, y, buf, C_LTG, 1);
    y += 10;

    /* Separator */
    lcd_rect(0, y, LCD_W, 1, C_DKG);
    y += 3;

    /* SC Variance heatmap */
    lcd_str(4, y, "SC VARIANCE", C_LTG, 1);
    y += 10;
    float smx = 0.01f;
    for (int i = 0; i < CSI_MAX_SC; i++)
    {
        if (g_csi.sc_variance[i] > smx)
        {
            smx = g_csi.sc_variance[i];
        }
    }
    for (int i = 0; i < CSI_MAX_SC; i++)
    {
        float n = g_csi.sc_variance[i] / smx;
        uint16_t c = C_DKG;
        if (n >= 0.25f)
        {
            c = C_BLU;
        }
        if (n >= 0.5f)
        {
            c = C_CYN;
        }
        if (n >= 0.75f)
        {
            c = C_GRN;
        }
        if (n > 0.9f)
        {
            c = C_RED;
        }
        int bx = 8 + i * 3;
        int bh = (int)(n * 12) + 1;
        lcd_rect(bx, y + 12 - bh, 3, bh, c);
        lcd_rect(bx, y, 3, 12 - bh, C_BLK);
    }
    y += 16;

    /* Separator */
    lcd_rect(0, y, LCD_W, 1, C_DKG);
    y += 3;

    /* Breathing bar */
    lcd_str(4, y, "BREATH", C_GRN, 1);
    lcd_bar(52, y, 178, 5, g_csi.breathing_power, 5.0f, C_GRN, C_DKG);
    y += 9;

    /* Motion bar */
    lcd_str(4, y, "MOTION", C_RED, 1);
    lcd_bar(52, y, 178, 5, g_csi.motion_power / 64.0f, 10.0f, C_RED, C_DKG);
    y += 11;

    /* Separator */
    lcd_rect(0, y, LCD_W, 1, C_DKG);
    y += 3;

    /* Amplitude waveform */
    lcd_str(4, y, "AMPLITUDE", C_CYN, 1);
    y += 9;
    int np = g_csi.hist_idx < CSI_HIST ? g_csi.hist_idx : CSI_HIST;
    if (np > 2)
    {
        float ord[CSI_HIST];
        int st = g_csi.hist_idx % CSI_HIST;
        for (int i = 0; i < np; i++)
        {
            ord[i] = g_csi.amp_hist[(st + i) % CSI_HIST];
        }
        lcd_wave(4, y, 232, 22, ord, np, C_CYN);
    }
    else
    {
        lcd_rect(4, y, 232, 22, C_BLK);
    }
    y += 25;

    /* Separator */
    lcd_rect(0, y, LCD_W, 1, C_DKG);
    y += 3;

    /* Phase rate waveform */
    lcd_str(4, y, "PHASE RATE", C_GRN, 1);
    y += 9;
    if (np > 2)
    {
        float ord[CSI_HIST];
        int st = g_csi.hist_idx % CSI_HIST;
        for (int i = 0; i < np; i++)
        {
            ord[i] = g_csi.phase_hist[(st + i) % CSI_HIST];
        }
        lcd_wave(4, y, 232, 22, ord, np, C_GRN);
    }
    else
    {
        lcd_rect(4, y, 232, 22, C_BLK);
    }
    y += 24;

    /* Clear below content */
    if (y < 258)
    {
        lcd_rect(0, y, LCD_W, 258 - y, C_BLK);
    }
}

/* ── Page 1: Waveforms ── */
void ui_page_waveforms(void)
{
    char buf[48];
    int y = Y_START;

    /* Header */
    lcd_rect(0, y, LCD_W, 16, C_DKG);
    lcd_str(50, y + 1, "WAVEFORMS", C_CYN, 2);
    y += 18;

    /* Motion intensity label */
    float motion_norm = g_csi.motion_power / 64.0f;
    lcd_rect(0, y, LCD_W, 10, C_BLK);
    if (motion_norm < 1.0f)
    {
        lcd_str(4, y + 1, "CALM", C_GRN, 1);
    }
    else if (motion_norm < 5.0f)
    {
        lcd_str(4, y + 1, "LIGHT MOTION", C_YEL, 1);
    }
    else
    {
        lcd_str(4, y + 1, "HEAVY MOTION", C_RED, 1);
    }

    /* Estimated breathing rate (zero-crossing method) */
    if (g_csi.breathing_power > 1.0f)
    {
        int np_tmp = g_csi.hist_idx < CSI_HIST ? g_csi.hist_idx : CSI_HIST;
        int crossings = 0;
        if (np_tmp > 4)
        {
            int st_tmp = g_csi.hist_idx % CSI_HIST;
            float prev = g_csi.phase_hist[st_tmp];
            for (int i = 1; i < np_tmp; i++)
            {
                float cur = g_csi.phase_hist[(st_tmp + i) % CSI_HIST];
                if ((prev < 0.0f && cur >= 0.0f) || (prev >= 0.0f && cur < 0.0f))
                {
                    crossings++;
                }
                prev = cur;
            }
        }
        /* sample rate ~10 Hz, 64 samples = ~6.4 s window */
        /* half-crossings -> cycles; BPM = (crossings/2) / (n/10) * 60 */
        int bpm = 0;
        if (crossings > 0 && np_tmp > 0)
        {
            /* bpm = crossings * 10 * 60 / (2 * np_tmp) */
            bpm = (crossings * 300) / np_tmp;
        }
        if (bpm < 4)
        {
            bpm = 4;
        }
        if (bpm > 40)
        {
            bpm = 40;
        }
        snprintf(buf, sizeof(buf), "~%d BPM", bpm);
        lcd_rect(140, y, 96, 10, C_BLK);
        lcd_str(140, y + 1, buf, C_GRN, 1);
    }
    else
    {
        lcd_rect(140, y, 96, 10, C_BLK);
        lcd_str(140, y + 1, "NO BREATHING", C_DKG, 1);
    }
    y += 12;

    int np = g_csi.hist_idx < CSI_HIST ? g_csi.hist_idx : CSI_HIST;

    /* Amplitude waveform (SIGNAL) */
    lcd_rect(0, y, LCD_W, 9, C_BLK);
    lcd_str(4, y, "SIGNAL", C_CYN, 1);

    /* Std dev of last 16 samples to classify signal */
    if (np >= 4)
    {
        int nsamp = np < 16 ? np : 16;
        int st16 = (g_csi.hist_idx - nsamp + CSI_HIST * 2) % CSI_HIST;
        float sum16 = 0.0f;
        for (int i = 0; i < nsamp; i++)
        {
            sum16 += g_csi.amp_hist[(st16 + i) % CSI_HIST];
        }
        float mean16 = sum16 / (float)nsamp;
        float var16 = 0.0f;
        for (int i = 0; i < nsamp; i++)
        {
            float d = g_csi.amp_hist[(st16 + i) % CSI_HIST] - mean16;
            var16 += d * d;
        }
        var16 /= (float)nsamp;
        /* simple integer sqrt approximation */
        float std16 = 0.0f;
        float guess = var16 / 2.0f + 0.5f;
        if (guess < 0.001f)
        {
            guess = 0.001f;
        }
        /* 3 Newton iterations */
        guess = (guess + var16 / guess) * 0.5f;
        guess = (guess + var16 / guess) * 0.5f;
        guess = (guess + var16 / guess) * 0.5f;
        std16 = guess;
        if (std16 < 1.0f)
        {
            lcd_str(150, y, "STABLE", C_GRN, 1);
        }
        else if (std16 < 5.0f)
        {
            lcd_str(138, y, "CHANGING", C_YEL, 1);
        }
        else
        {
            lcd_str(132, y, "DISRUPTED", C_RED, 1);
        }
    }
    y += 10;

    if (np > 2)
    {
        float ord[CSI_HIST];
        int st = g_csi.hist_idx % CSI_HIST;
        for (int i = 0; i < np; i++)
        {
            ord[i] = g_csi.amp_hist[(st + i) % CSI_HIST];
        }
        lcd_wave(4, y, 232, 55, ord, np, C_CYN);
    }
    else
    {
        lcd_rect(4, y, 232, 55, C_BLK);
        lcd_str(70, y + 24, "NO DATA", C_DKG, 1);
    }
    y += 57;

    /* Separator */
    lcd_rect(0, y, LCD_W, 1, C_DKG);
    y += 4;

    /* Phase rate waveform (BODY MOTION) */
    lcd_rect(0, y, LCD_W, 9, C_BLK);
    lcd_str(4, y, "BODY MOTION", C_GRN, 1);

    /* Show BREATHING label if breathing detected */
    if (g_csi.breathing_power > 1.0f)
    {
        lcd_str(152, y, "BREATHING", C_GRN, 1);
    }
    else
    {
        lcd_rect(152, y, 84, 9, C_BLK);
    }
    y += 10;

    if (np > 2)
    {
        float ord[CSI_HIST];
        int st = g_csi.hist_idx % CSI_HIST;
        for (int i = 0; i < np; i++)
        {
            ord[i] = g_csi.phase_hist[(st + i) % CSI_HIST];
        }
        lcd_wave(4, y, 232, 55, ord, np, C_GRN);
    }
    else
    {
        lcd_rect(4, y, 232, 55, C_BLK);
        lcd_str(70, y + 24, "NO DATA", C_DKG, 1);
    }
    y += 57;

    /* Separator */
    lcd_rect(0, y, LCD_W, 1, C_DKG);
    y += 4;

    /* State + direction + occupancy summary */
    lcd_rect(0, y, LCD_W, 9, C_BLK);
    snprintf(buf, sizeof(buf), "STATE:%s DIR:%s OCC:%d",
             g_csi.state, g_csi.motion_direction, g_csi.occupancy_estimate);
    lcd_str(4, y, buf, C_LTG, 1);
    y += 11;

    /* Clear below */
    if (y < 258)
    {
        lcd_rect(0, y, LCD_W, 258 - y, C_BLK);
    }
}

/* ── Page 2: AP List ── */
void ui_page_ap_list(void)
{
    char buf[48];
    int y = Y_START;

    /* Header */
    lcd_rect(0, y, LCD_W, 16, C_DKG);
    lcd_str(66, y + 1, "AP LIST", C_CYN, 2);
    y += 20;

    int shown = 0;
    for (int i = 0; i < g_ap_tracker.count; i++)
    {
        const tracked_ap_t *ap = &g_ap_tracker.aps[i];
        if (!ap->active)
        {
            continue;
        }

        /* SSID */
        lcd_rect(0, y, LCD_W, 9, C_BLK);
        lcd_str(4, y, ap->ssid[0] ? ap->ssid : "<hidden>", C_YEL, 1);
        /* Channel */
        snprintf(buf, sizeof(buf), "CH%d", ap->channel);
        lcd_str(200, y, buf, C_LTG, 1);
        y += 10;

        /* BSSID */
        lcd_rect(0, y, LCD_W, 9, C_BLK);
        snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
                 ap->bssid[0], ap->bssid[1], ap->bssid[2],
                 ap->bssid[3], ap->bssid[4], ap->bssid[5]);
        lcd_str(4, y, buf, C_LTG, 1);
        y += 10;

        /* RSSI bar */
        uint16_t rssi_col = C_GRN;
        if (ap->rssi < -70)
        {
            rssi_col = C_RED;
        }
        else if (ap->rssi < -55)
        {
            rssi_col = C_YEL;
        }
        snprintf(buf, sizeof(buf), "%ddBm", ap->rssi);
        lcd_str(4, y, buf, rssi_col, 1);
        lcd_bar(60, y, 170, 5, (float)(ap->rssi + 100), 70.0f, rssi_col, C_DKG);
        y += 8;

        /* Variance dot */
        uint16_t vc = C_GRN;
        if (ap->variance > 1.0f)
        {
            vc = C_YEL;
        }
        if (ap->variance > 5.0f)
        {
            vc = C_RED;
        }
        lcd_rect(4, y, 5, 5, vc);
        snprintf(buf, sizeof(buf), "var:%.1f", (double)ap->variance);
        lcd_str(12, y, buf, C_LTG, 1);
        y += 8;

        /* Separator */
        lcd_rect(4, y, 232, 1, C_DKG);
        y += 4;

        shown++;
        if (y > 245)
        {
            break;
        }
    }

    if (shown == 0)
    {
        lcd_rect(0, y, LCD_W, 20, C_BLK);
        lcd_str(60, y + 5, "NO APs FOUND", C_DKG, 1);
        y += 20;
    }

    /* Clear below */
    if (y < 258)
    {
        lcd_rect(0, y, LCD_W, 258 - y, C_BLK);
    }
}

/* ── Page 3: IMU ── */
void ui_page_imu(void)
{
    char buf[48];
    int y = Y_START;

    /* Header */
    lcd_rect(0, y, LCD_W, 16, C_DKG);
    lcd_str(46, y + 1, "IMU SENSOR", C_CYN, 2);
    y += 20;

    /* Accel X/Y/Z — range +-4g, centered bars */
    const char *accel_labels[] = {"AX", "AY", "AZ"};
    float accel_vals[] = {g_imu.accel_x, g_imu.accel_y, g_imu.accel_z};
    for (int i = 0; i < 3; i++)
    {
        lcd_rect(0, y, LCD_W, 14, C_BLK);
        lcd_str(4, y + 3, accel_labels[i], C_CYN, 1);
        snprintf(buf, sizeof(buf), "%+.2f", (double)accel_vals[i]);
        lcd_str(24, y + 3, buf, C_WHT, 1);

        /* Centered bar: midpoint at x=140, full range 160px for +-4g */
        int mid = 140;
        float norm = accel_vals[i] / 4.0f;  /* -1..+1 */
        if (norm > 1.0f)
        {
            norm = 1.0f;
        }
        if (norm < -1.0f)
        {
            norm = -1.0f;
        }
        int bw = (int)(norm * 80.0f);
        /* Draw background */
        lcd_rect(60, y + 2, 160, 8, C_DKG);
        /* Center line */
        lcd_rect(mid, y + 1, 1, 10, C_LTG);
        if (bw > 0)
        {
            lcd_rect(mid, y + 2, bw, 8, C_CYN);
        }
        else if (bw < 0)
        {
            lcd_rect(mid + bw, y + 2, -bw, 8, C_ORG);
        }
        y += 16;
    }

    /* Separator */
    lcd_rect(0, y, LCD_W, 1, C_DKG);
    y += 4;

    /* Gyro X/Y/Z — range +-512 dps */
    const char *gyro_labels[] = {"GX", "GY", "GZ"};
    float gyro_vals[] = {g_imu.gyro_x, g_imu.gyro_y, g_imu.gyro_z};
    for (int i = 0; i < 3; i++)
    {
        lcd_rect(0, y, LCD_W, 14, C_BLK);
        lcd_str(4, y + 3, gyro_labels[i], C_MAG, 1);
        snprintf(buf, sizeof(buf), "%+.0f", (double)gyro_vals[i]);
        lcd_str(24, y + 3, buf, C_WHT, 1);

        int mid = 140;
        float norm = gyro_vals[i] / 512.0f;
        if (norm > 1.0f)
        {
            norm = 1.0f;
        }
        if (norm < -1.0f)
        {
            norm = -1.0f;
        }
        int bw = (int)(norm * 80.0f);
        lcd_rect(60, y + 2, 160, 8, C_DKG);
        lcd_rect(mid, y + 1, 1, 10, C_LTG);
        if (bw > 0)
        {
            lcd_rect(mid, y + 2, bw, 8, C_MAG);
        }
        else if (bw < 0)
        {
            lcd_rect(mid + bw, y + 2, -bw, 8, C_ORG);
        }
        y += 16;
    }

    /* Separator */
    lcd_rect(0, y, LCD_W, 1, C_DKG);
    y += 4;

    /* Device motion magnitude bar */
    lcd_rect(0, y, LCD_W, 12, C_BLK);
    lcd_str(4, y + 2, "MOTION", C_YEL, 1);
    lcd_bar(60, y + 2, 170, 7, g_imu.device_motion, 2.0f, C_YEL, C_DKG);
    y += 16;

    /* Device status */
    lcd_rect(0, y, LCD_W, 20, C_BLK);
    if (g_imu.device_moving)
    {
        lcd_str(50, y + 3, "DEVICE MOVING", C_RED, 2);
        y += 20;
        lcd_rect(0, y, LCD_W, 12, C_BLK);
        lcd_str(60, y + 2, "CSI PAUSED", C_ORG, 1);
    }
    else
    {
        lcd_str(74, y + 3, "STABLE", C_GRN, 2);
    }
    y += 16;

    /* Clear below */
    if (y < 258)
    {
        lcd_rect(0, y, LCD_W, 258 - y, C_BLK);
    }
}

/* ── Page 4: Alert Mode ── */
void ui_page_alert(void)
{
    char buf[48];
    int y = Y_START;

    /* Header */
    lcd_rect(0, y, LCD_W, 16, C_DKG);
    lcd_str(42, y + 1, "ALERT MODE", C_CYN, 2);
    y += 24;

    /* Occupancy count from CSI engine estimate */
    int occ = g_csi.occupancy_estimate;
    g_ui.occupancy_count = occ;

    /* Large number */
    lcd_rect(0, y, LCD_W, 36, C_BLK);
    snprintf(buf, sizeof(buf), "%d", occ);
    lcd_str(105, y + 2, buf, C_WHT, 3);
    y += 36;

    /* Label */
    lcd_rect(0, y, LCD_W, 12, C_BLK);
    lcd_str(50, y, "PEOPLE DETECTED", C_LTG, 1);
    y += 20;

    /* Alert status */
    lcd_rect(0, y, LCD_W, 24, C_BLK);
    if (g_ui.alert_enabled)
    {
        lcd_str(52, y + 2, "ALERT: ON", C_GRN, 2);
    }
    else
    {
        lcd_str(48, y + 2, "ALERT: OFF", C_RED, 2);
    }
    y += 28;

    /* TAP instruction */
    lcd_rect(0, y, LCD_W, 12, C_BLK);
    lcd_str(54, y, "TAP TO TOGGLE", C_LTG, 1);
    y += 20;

    /* Separator */
    lcd_rect(0, y, LCD_W, 1, C_DKG);
    y += 5;

    /* Current state + confidence */
    lcd_rect(0, y, LCD_W, 18, C_BLK);
    uint16_t sc = C_GRN;
    if (strcmp(g_csi.state, "MOTION") == 0)
    {
        sc = C_RED;
    }
    else if (strcmp(g_csi.state, "STILL") == 0)
    {
        sc = C_YEL;
    }
    else if (strcmp(g_csi.state, "PRESENCE") == 0)
    {
        sc = C_CYN;
    }
    snprintf(buf, sizeof(buf), "%s  %d%%", g_csi.state, (int)(g_csi.confidence * 100));
    lcd_str(4, y + 4, buf, sc, 2);
    y += 22;

    /* Direction line */
    lcd_rect(0, y, LCD_W, 9, C_BLK);
    snprintf(buf, sizeof(buf), "DIR:%s", g_csi.motion_direction);
    lcd_str(4, y, buf, C_LTG, 1);
    y += 12;

    /* Clear below */
    if (y < 258)
    {
        lcd_rect(0, y, LCD_W, 258 - y, C_BLK);
    }
}

/* ── Navigation bar: 5 dots at y=262 ── */
void ui_draw_nav_bar(void)
{
    lcd_rect(0, 258, LCD_W, 22, C_BLK);
    int spacing = LCD_W / (UI_NUM_PAGES + 1);
    for (int i = 0; i < UI_NUM_PAGES; i++)
    {
        int cx = spacing * (i + 1);
        uint16_t col = (i == g_ui.current_page) ? C_WHT : C_DKG;
        /* Draw a 7x7 filled circle approximation (diamond-ish) */
        lcd_rect(cx - 1, 264, 3, 7, col);
        lcd_rect(cx - 2, 265, 5, 5, col);
        lcd_rect(cx - 3, 266, 7, 3, col);
    }
}
