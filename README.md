# ESP32-imu-node

## Supported command

### Connect as station

```plain
sta <ssid> <psk>
```

### Disconnect

```plain
sta - - 
```

### Set TCP partner

```plain
tcp select <address> <port>
```

### Set UDP partner

```plain
udp select <address> <port>
```

### Set mode

```plain
mode {tcp|udp}
```

### TCP Start/stop

```plain
{tcp|udp} {start|stop}
```

## 坑

### tcp_server example not able to reconnect the server after disconnect

and comment out the 2 break statements above, youll have to deal with the break logic better, I just didn't have the time.
But this makes it work. You dont want to bind again, once you have already bound.
Looks like this. （https://esp32.com/viewtopic.php?t=7791）改变while的位置
```c
static void tcp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

  

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 destAddr;
        bzero(&destAddr.sin6_addr.un, sizeof(destAddr.sin6_addr.un));
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            //break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(listen_sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            //break;
        }
        ESP_LOGI(TAG, "Socket binded");

  while (1) {

        err = listen(listen_sock, 1);
        if (err != 0) {
            ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket listening");
```
