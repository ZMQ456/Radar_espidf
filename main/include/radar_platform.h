#ifndef RADAR_PLATFORM_H
#define RADAR_PLATFORM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint64_t radar_now_ms(void);
void radar_sleep_ms(uint32_t delay_ms);
void radar_restart(void);

#ifdef __cplusplus
}
#endif

#endif
