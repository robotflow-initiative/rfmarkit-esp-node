# Manual For IMU System

## 概览

IMU测量系统是一套包括传感器、有关软件系统的测量系统。传感器基于ESP32和HI229，前者是一个低功耗的物联网微控制器 (MCU)，后者是MEMS惯性测量器件m (IMU)。此外，IMU传感器预留了一个插座用于连接LED灯珠，以实现主动定位。

## 系统设计

### 启动和连接

传感器MCU在启动后会进行自检 (LED系统、IMU系统) 并连接到预设的无线局域网。当连接失败时，传感器模块会重启并尝试连接，直到连接建立。成功联网后，传感器将会从NTP服务器获取时间。在时间同步完成后，传感器将尝试与数据记录服务器建立TCP连接。如果成功，传感器将会进入一个等待状态。否则，传感器会不断重试连接，并在一定的失败时重启。

### 控制

MCU会尝试使用DHCP客户端获得IP，一旦或得IP，用户就可以通过nc工具连接到MCU的18888端口对其进行控制。命令如下：

```console
nc <IP> 18888
```

> 目前没有很好的方案从MCU获取IP地址，因此该地址需要到路由器后台获取

MCU提供了交互式的界面。每一行命令都用回车`\n`换行并在交互式文字终端后执行命令MCU支持多种控制命令，其中控制记录开始和结束的是`start`/`stop`，这两个命令会控制MCU数据的发送的行为。

### 传输

数据记录服务器在主机上启动一个端口进行侦听。记录服务器使用多进程+select来处理来自大量MCU的数据。

部署需要的资源包括

- 传感器固件 `davidliyutong/imu-esp-node.git`
- 数据记录服务器`davidliyutong/imu-interface.git`
- OTA更新服务器
- 无线接入点
- 时间同步服务器
- 数据分析算法 `davidliyutong/imu-data-tools.git`

### 部署情况

截至2022-01-27， 时间服务器部署在`10.53.21.164`。无线接入点是`yz_sensor:yzri@1220`

## 工作计划

建立ESP32的MESH网络，改善无线传输性能
使用红外LED灯提高追踪效果
优化连接流程
当多次连接失败时，模块发出信号，提醒用户遇到了问题
当无法连接到无线局域网的时候，发射热点/开启蓝牙配网
使用StringBuffer代替写文件

  
优化传输
- 当传输中断或者出错的时候，MCU端缓冲数据并且在重连接之后发送

提高精确度
- 测试并验证时间同步算法的

## IMU

## 固件烧录

固件项目的地址在`davidliyutong/imu-esp-node.git`。使用esp-idf工具链。

## 控制

IMU使用nc命令进行控制，默认端口是18888。使用如下命令可以与IMU建立连接

```console
nc <IP> 18888
```

例如

```
nc 10.53.24.117 18888
```

## 命令

<命令> <参数>

restart / update / shutdown / id / ver

blink_set \<N\> / blink_get / blink_start / blink_stop / 

start / stop / imu_imm /

varget XXX? / varset XXX=YYY


```c
static command_reg_t s_registration[] = {
        {.name = "restart", .func = command_func_restart},
        {.name = "ping", .func = command_func_ping},
        {.name = "shutdown", .func = command_func_shutdown},
        {.name = "update", .func = command_func_update},
        {.name = "imu_cali_reset", .func = command_func_imu_cali_reset},
        {.name = "imu_cali_acc", .func = command_func_imu_cali_acc},
        {.name = "imu_cali_mag", .func = command_func_imu_cali_mag},
        {.name = "start", .func = command_func_start},
        {.name = "stop", .func = command_func_stop},
        {.name = "imu_enable", .func = command_func_imu_enable},
        {.name = "imu_disable", .func = command_func_imu_disable},
        {.name = "imu_status", .func = command_func_imu_status},
        {.name = "imu_imm", .func = command_func_imu_imm},
        {.name = "imu_setup", .func = command_func_imu_setup},
        {.name = "imu_scale",.func = command_func_imu_scale},
        {.name = "imu_debug", .func = command_func_imu_debug},
        {.name = "id", .func = command_func_id},
        {.name = "ver", .func = command_func_ver},
        {.name = "blink_set", .func = command_func_blink_set},
        {.name = "blink_get", .func = command_func_blink_get},
        {.name = "blink_start", .func = command_func_blink_start},
        {.name = "blink_stop", .func = command_func_blink_stop},
        {.name = "blink_off", .func = command_func_blink_off},
        {.name = "self_test", .func = command_func_imu_self_test},
        {.name = "always_on", .func = command_func_always_on},
        {.name = "varset", .func = command_func_varset},
        {.name = "varget", .func = command_func_varget},
        {.name = "v"CONFIG_FIRMWARE_VERSION"_shutdown", .func = command_func_shutdown}
};
```

### varget/varset

```c
mcu_var_t g_mcu_vars[] = {
        {.name = "WIFI_SSID", .type = VAR_STR},
        {.name = "WIFI_PSK", .type = VAR_STR},
        {.name = "DATA_HOST", .type=VAR_STR},
        {.name = "OTA_HOST", .type=VAR_STR},
        {.name = "NTP_HOST", .type=VAR_STR},
        {.name ="TEST", .type =VAR_INT32},
        {.name = "IMU_BAUD", .type=VAR_INT32},
        {.name = "USE_HAMMING", .type=VAR_INT32}
};
```
## 初始配置

```
ping
imu_debug AT+EOUT=0
imu_debug AT+MODE=1
imu_debug AT+SETPTL=91
imu_debug AT+BAUD=921600
imu_debug AT+ODR=200
imu_debug AT+EOUT=1
imu_debug AT+RST
varset IMU_BAUD=921600
restart
imu_imm
```

## 默认设置

IMU 默认有以下配置

- 数据记录服务器指向在`10.53.21.102` 的12900K无卡机器上。IMU会默认向这些
- 更新服务器在

### 部署情况

## IMU Mapping

这里维护了一份IMU IP 地址和设备地址之间的映射关系

| DEVICE ID    | IP/BLINK_ID |
| ------------ | ----------- |
| 78218451a7b8 | 250         |
| 78218451b56c | 17          |
| e89f6d75c800 | 228         |
| 782184507cf8 | 185         |
| e89f6d75fedc | 199         |
| 30c6f751bdc8 | 32          |
| e89f6d742a88 | 142         |
| 78218451b260 | 38          |
| 30c6f751c234 | 224         |
| 30c6f751c764 | 148         |
| 78218451b2c8 | 234         |

## 部署检查清单


### 发布OTA前检查清单

- 新的OTA固件版本是否正确设置？
- 新的OTA固件联网是否正确？
- 新的OTA固件中，更新服务器是否可达？（能否再次OTA）
- 新的OTA固件能否相应OTA指令？

### 设备初始化检查清单

- BLINK_ID 是否正确设置
- Device ID 是否正确记录
- 传感器是否已经设置200HZ输出
- 传感器是否已经启用九轴融合
- 传感器是否已经设置波特率921600
- MCU是否已经设置了波特率



## Introduction

## Environment Setup

## Configuration

## Recording

## Post-Processing

## Development

## Other