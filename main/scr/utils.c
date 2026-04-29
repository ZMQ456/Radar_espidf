#include "utils.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "mbedtls/base64.h"

static const char *TAG = "utils";

bool base64_encode(const uint8_t *data, size_t len, uint8_t *out, size_t out_len, size_t *out_actual) {
    if (data == NULL || out == NULL || out_actual == NULL) {
        ESP_LOGE(TAG, "invalid base64 encode args");
        return false;
    }

    return mbedtls_base64_encode(out, out_len, out_actual, data, len) == 0;
}

bool base64_decode(const uint8_t *data, size_t len, uint8_t *out, size_t out_len, size_t *out_actual) {
    if (data == NULL || out == NULL || out_actual == NULL) {
        ESP_LOGE(TAG, "invalid base64 decode args");
        return false;
    }

    return mbedtls_base64_decode(out, out_len, out_actual, data, len) == 0;
}

int json_escape(const char *s, char *out, size_t out_len) {
    if (s == NULL || out == NULL || out_len == 0U) {
        return -1;
    }

    size_t out_pos = 0U;
    for (size_t i = 0U; s[i] != '\0' && out_pos < out_len - 1U; ++i) {
        const char c = s[i];
        switch (c) {
        case '\\':
        case '"':
            if (out_pos + 2U > out_len) {
                out[out_pos] = '\0';
                return (int)out_pos;
            }
            out[out_pos++] = '\\';
            out[out_pos++] = c;
            break;
        case '\n':
        case '\r':
        case '\t':
            if (out_pos + 2U > out_len) {
                out[out_pos] = '\0';
                return (int)out_pos;
            }
            out[out_pos++] = '\\';
            out[out_pos++] = (c == '\n') ? 'n' : (c == '\r' ? 'r' : 't');
            break;
        default:
            out[out_pos++] = c;
            break;
        }
    }

    out[out_pos] = '\0';
    return (int)out_pos;
}

static bool json_extract_core(const char *json, const char *key, char *out, size_t out_len, bool allow_partial) {
    if (json == NULL || key == NULL || out == NULL || out_len == 0U) {
        return false;
    }

    char pattern[128] = {};
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *pos = strstr(json, pattern);
    if (pos == NULL) {
        snprintf(pattern, sizeof(pattern), "\"%s\" :", key);
        pos = strstr(json, pattern);
    }
    if (pos == NULL) {
        return false;
    }

    pos += strlen(pattern);
    while (*pos == ' ' || *pos == '\t' || *pos == '\n' || *pos == '\r') {
        ++pos;
    }
    if (*pos != '"') {
        return false;
    }

    ++pos;
    const char *start = pos;
    const char *end = strchr(pos, '"');
    if (end == NULL && allow_partial) {
        end = json + strlen(json);
    }
    if (end == NULL) {
        return false;
    }

    size_t len = (size_t)(end - start);
    if (len >= out_len) {
        len = out_len - 1U;
    }
    memcpy(out, start, len);
    out[len] = '\0';
    return true;
}

bool json_extract_string(const char *json, const char *key, char *out, size_t out_len) {
    return json_extract_core(json, key, out, out_len, false);
}

bool json_extract_from_partial(const char *json, const char *key, char *out, size_t out_len) {
    return json_extract_core(json, key, out, out_len, true);
}
