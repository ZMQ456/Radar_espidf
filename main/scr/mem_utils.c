#include "mem_utils.h"

#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "mem_utils";

void *app_mem_alloc(size_t size, mem_type_t type) {
    if (size == 0U) {
        return NULL;
    }

    void *ptr = NULL;
    switch (type) {
    case MEM_TYPE_SMALL:
        ptr = heap_caps_malloc(size, MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT);
        break;
    case MEM_TYPE_LARGE:
        ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (ptr == NULL) {
            ptr = heap_caps_malloc(size, MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT);
        }
        break;
    case MEM_TYPE_PSRAM:
        ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        break;
    default:
        break;
    }

    if (ptr == NULL) {
        ESP_LOGE(TAG, "memory alloc failed size=%u type=%d", (unsigned)size, (int)type);
    }
    return ptr;
}

void app_mem_free(void *ptr) {
    if (ptr != NULL) {
        heap_caps_free(ptr);
    }
}

void app_mem_print_info(void) {
    heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
    heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);
}
