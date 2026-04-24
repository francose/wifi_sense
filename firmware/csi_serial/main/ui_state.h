#ifndef UI_STATE_H
#define UI_STATE_H

#define UI_NUM_PAGES 5
#define Y_START 35

typedef struct {
    int current_page;
    int alert_enabled;
    int alert_triggered;
    int occupancy_count;
} ui_state_t;

extern ui_state_t g_ui;

#endif
