#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MEM_TYPE_SMALL = 0,
    MEM_TYPE_LARGE,
    MEM_TYPE_PSRAM
} mem_type_t;

void *app_mem_alloc(size_t size, mem_type_t type);
void app_mem_free(void *ptr);
void app_mem_print_info(void);

#ifdef __cplusplus
}
#endif
