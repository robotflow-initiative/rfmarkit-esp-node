# ESP32-imu-node

## Get Started

First, setup esp-idf environment according to the official guide [here](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32/get-started/)

Choose the right platform:

```shell
cp sdkconfig.esp32 sdkconfig
```

Build firmware

```shell
idf.py build && idf.py flash
```

> Run `export ESPPORT=/dev/<port>` to specify port

## Code Explained


- `components` system components
- `components/apps` applications runs on RTOS
- `components/blink` operates LED
- `components/imu` operates IMU (dummy package)
- `components/sys` supporting modules for system
- `docs` documents
- `include` global headers
- `lib` common libraries
- `lib/hi229` actual imu package
- `main` entrypoint of firmware
- `scripts` helper scripts
- `tests` function tests