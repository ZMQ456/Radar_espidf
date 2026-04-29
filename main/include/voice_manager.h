#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool voice_manager_init(void);
bool voice_manager_is_initialized(void);
bool voice_manager_trigger_session(void);

#ifdef __cplusplus
}
#endif
