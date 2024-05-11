# HI229 Setup

```
TODO: fix the document
```

First, flash the firmware.

Then, log IP address of imu from serial debug interface. Record the IP, such as `10.53.21.17`.

On IMU socket interface， use `blink_set <NO.>` to set blink sequence.

```text
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

```
blink_set 225
varset IMU_BAUD=921600

imu_debug AT+EOUT=0
imu_debug AT+MODE=1
imu_debug AT+SETPTL=91
imu_debug AT+BAUD=921600
imu_debug AT+ODR=200
imu_debug AT+EOUT=1
imu_debug AT+RST
restart
imu_imm
```