#include "tcp.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

typedef struct {
    tcp_client_config_t config;
    int sockfd;
    struct sockaddr_in server_addr;
} tcp_client_t;

static const char *TAG = "tcp";

tcp_client_handle_t tcp_client_create(const tcp_client_config_t *config) {
    if (config == NULL || config->host == NULL) {
        return NULL;
    }

    tcp_client_t *client = (tcp_client_t *)malloc(sizeof(tcp_client_t));
    if (client == NULL) {
        return NULL;
    }

    memset(client, 0, sizeof(*client));
    client->config = *config;
    client->sockfd = -1;
    return client;
}

bool tcp_client_connect(tcp_client_handle_t handle) {
    if (handle == NULL) {
        return false;
    }

    tcp_client_t *client = (tcp_client_t *)handle;
    client->sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (client->sockfd < 0) {
        ESP_LOGE(TAG, "socket create failed: %s", strerror(errno));
        return false;
    }

    struct timeval timeout = {
        .tv_sec = (long)(client->config.timeout_ms / 1000U),
        .tv_usec = (long)((client->config.timeout_ms % 1000U) * 1000U),
    };
    setsockopt(client->sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(client->sockfd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    struct addrinfo hints = {};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo *res = NULL;
    const int err = getaddrinfo(client->config.host, NULL, &hints, &res);
    if (err != 0 || res == NULL) {
        ESP_LOGE(TAG, "getaddrinfo failed host=%s err=%d", client->config.host, err);
        close(client->sockfd);
        client->sockfd = -1;
        return false;
    }

    client->server_addr.sin_family = AF_INET;
    client->server_addr.sin_port = htons(client->config.port);
    memcpy(&client->server_addr.sin_addr, &((struct sockaddr_in *)res->ai_addr)->sin_addr, sizeof(struct in_addr));
    freeaddrinfo(res);

    if (connect(client->sockfd, (struct sockaddr *)&client->server_addr, sizeof(client->server_addr)) < 0) {
        ESP_LOGE(TAG, "connect failed: %s", strerror(errno));
        close(client->sockfd);
        client->sockfd = -1;
        return false;
    }

    return true;
}

int tcp_client_recv(tcp_client_handle_t handle, char *buf, size_t len, uint32_t timeout_ms) {
    if (handle == NULL || buf == NULL || len == 0U) {
        return -1;
    }

    tcp_client_t *client = (tcp_client_t *)handle;
    struct timeval timeout = {
        .tv_sec = (long)(timeout_ms / 1000U),
        .tv_usec = (long)((timeout_ms % 1000U) * 1000U),
    };
    setsockopt(client->sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    const int ret = recv(client->sockfd, buf, len, 0);
    if (ret < 0 && (errno == ETIMEDOUT || errno == EWOULDBLOCK || errno == EAGAIN)) {
        return 0;
    }
    return ret;
}

int tcp_client_send(tcp_client_handle_t handle, const char *buf, size_t len, uint32_t timeout_ms) {
    if (handle == NULL || buf == NULL || len == 0U) {
        return -1;
    }

    tcp_client_t *client = (tcp_client_t *)handle;
    struct timeval timeout = {
        .tv_sec = (long)(timeout_ms / 1000U),
        .tv_usec = (long)((timeout_ms % 1000U) * 1000U),
    };
    setsockopt(client->sockfd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    return send(client->sockfd, buf, len, 0);
}

int tcp_client_read_line(tcp_client_handle_t handle, char *buf, size_t max_len, uint32_t timeout_ms) {
    if (handle == NULL || buf == NULL || max_len == 0U) {
        return -1;
    }

    size_t pos = 0U;
    while (pos + 1U < max_len) {
        char c = '\0';
        const int ret = tcp_client_recv(handle, &c, 1U, timeout_ms);
        if (ret <= 0) {
            break;
        }
        if (c == '\n') {
            break;
        }
        buf[pos++] = c;
    }
    buf[pos] = '\0';
    return (int)pos;
}

void tcp_client_disconnect(tcp_client_handle_t handle) {
    if (handle == NULL) {
        return;
    }
    tcp_client_t *client = (tcp_client_t *)handle;
    if (client->sockfd >= 0) {
        shutdown(client->sockfd, SHUT_RDWR);
        close(client->sockfd);
        client->sockfd = -1;
    }
}

void tcp_client_destroy(tcp_client_handle_t handle) {
    if (handle == NULL) {
        return;
    }
    tcp_client_disconnect(handle);
    free(handle);
}
