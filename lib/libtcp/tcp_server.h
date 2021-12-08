#ifndef _TCP_SERVER_H
#define _TCP_SERVER_H

#define send_all(sock, buf) \
        { \
            int to_write = strlen(buf); \
            while (to_write > 0) { \
                int written = send(sock, buf, to_write, 0); \
                if (written < 0) { \
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno); \
                    break; \
                } \
                to_write -= written; \
            } \
        }


#define log_source_addr(source_addr) \
        { \
            char addr_str[128]; \
            if (source_addr.ss_family == PF_INET) { \
                inet_ntoa_r(((struct sockaddr_in*)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1); \
            } \
            ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str); \
        }

#define handle_socket_err(sock) \
        device_delay_ms(100);\
        if (sock != -1) { \
            ESP_LOGE(TAG, "Shutting down socket..."); \
            shutdown(sock, 0); \
            device_delay_ms(100); \
            close(sock); \
        }

#define CONFIG_KEEP_ALIVE 1
#define CONFIG_KEEP_IDLE 5
#define CONFIG_KEEP_COUNT 3
#define CONFIG_KEEP_INTERVAL 5

void server_init(void (*pfunc)(int));

#endif