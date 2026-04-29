#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *host;
    uint16_t port;
    uint32_t timeout_ms;
    uint32_t recv_timeout_ms;
} tcp_client_config_t;

typedef void *tcp_client_handle_t;

tcp_client_handle_t tcp_client_create(const tcp_client_config_t *config);
bool tcp_client_connect(tcp_client_handle_t client);
int tcp_client_recv(tcp_client_handle_t client, char *buf, size_t len, uint32_t timeout_ms);
int tcp_client_send(tcp_client_handle_t client, const char *buf, size_t len, uint32_t timeout_ms);
int tcp_client_read_line(tcp_client_handle_t client, char *buf, size_t max_len, uint32_t timeout_ms);
void tcp_client_disconnect(tcp_client_handle_t client);
void tcp_client_destroy(tcp_client_handle_t client);

#ifdef __cplusplus
}
#endif
