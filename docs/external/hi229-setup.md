# HI229 Setup

There are two ways to prepare/setup HI229 IMU for RFMarkIt project: offline and online

## Offline

The HI229 should be set via uart interface. First, connect HI229 to a USB to UART adapter, then connect the adapter to a computer. Use a serial terminal software, such as `minicom`, `putty`, `screen`, etc. to connect to the HI229. The baudrate should be set to 115200.

Then, send the following commands to the HI229 with `\r\n` at the end of each command:

```text
AT+EOUT=0
AT+MODE=1
AT+SETPTL=91
AT+BAUD=921600
AT+ODR=200
AT+EOUT=1
AT+RST
```

After that, the HI229 should be ready to use.

## Online

If the HI229 is already installed on the Marker, the HI229 can be set via the debug socket interface. You need a websocket client to connect to the debug socket interface. The websocket client can be a browser or a websocket client software, such as `wscat`.

First, flash the latest firmware to marker, or run OTA update. Turn on the marker and wait for registration, record its IP address.

Then, toggle the debug socket interface on the marker by running the following command:

```shell
curl -X POST http://$IP:18888/v1/imu/debug/toggle?target_state=enable
```

After that, connect to the debug socket interface with a websocket client:

```shell
wscat -c ws://$IP:18888/v1/imu/debug/socket
```

Then, send the following commands to the HI229 with `\r\n` at the end of each command:

```text
AT+EOUT=0
AT+MODE=1
AT+SETPTL=91
AT+BAUD=921600
AT+ODR=200
AT+EOUT=1
AT+RST
```
