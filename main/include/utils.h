#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool base64_encode(const uint8_t *data, size_t len, uint8_t *out, size_t out_len, size_t *out_actual);
bool base64_decode(const uint8_t *data, size_t len, uint8_t *out, size_t out_len, size_t *out_actual);
int json_escape(const char *s, char *out, size_t out_len);
bool json_extract_string(const char *json, const char *key, char *out, size_t out_len);
bool json_extract_from_partial(const char *json, const char *key, char *out, size_t out_len);

#ifdef __cplusplus
}
#endif
