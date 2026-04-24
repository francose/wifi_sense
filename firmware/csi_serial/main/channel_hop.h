#ifndef CHANNEL_HOP_H
#define CHANNEL_HOP_H

#include <stdint.h>

void channel_hop_init(const uint8_t *channels, int count, uint32_t dwell_ms);
void channel_hop_start(void);
void channel_hop_stop(void);
int  channel_hop_current(void);

#endif
