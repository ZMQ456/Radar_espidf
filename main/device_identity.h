#ifndef DEVICE_IDENTITY_H
#define DEVICE_IDENTITY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t device_id;
    uint64_t device_sn;
} DeviceIdentity;

bool device_identity_init(void);
DeviceIdentity device_identity_get(void);
bool device_identity_set_device_id(uint16_t device_id);
bool device_identity_set_device_sn(uint64_t device_sn);
bool device_identity_reset_defaults(void);

#ifdef __cplusplus
}
#endif

#endif
