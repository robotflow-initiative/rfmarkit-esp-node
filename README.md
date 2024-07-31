# RFMarkIt Firmware

This is the firmware for RFMarkIt project, which is a project for a active marker system for motion capture / 6DOF tracking.

## Get Started

First, setup esp-idf environment(v4.4.4) according to the official guide [here](https://docs.espressif.com/projects/esp-idf/zh_CN/v4.4.4/esp32/get-started/)

Then, build the project by running `idf.py build` in the project root directory.

```shell
export IDF_TARGET=esp32
idf.py build
```

To flash an node, run `idf.py flash monitor` in the project root directory.

```shell
export IDF_TARGET=esp32
idf.py -p <PORT> flash monitor
```

## Developers' Guide

The project is organized as follows:

- `components` system components
  - `components/apps` applications that runs with RTOS
  - `components/blink` functions to operate LED
  - `components/ble` Bluetooth Low Energy(BLE) components
  - `components/imu` a dummy module that introduce lib/hi229 and operates IMU
  - `components/rest_controller` RESTful API controller
  - `components/sys` supporting modules for system
- `docs` documents
- `include` global headers
- `lib` common libraries
  - `lib/hi229` actual imu library
  - `lib/libudp` udp library
  - `lib/ring_buf` ring buffer library
  - `lib/spatial` spatial math library
- `main` entrypoint of firmware
- `scripts` helper scripts
- `tests` function tests

If you are using different IMU, you can add your library in `lib` directory and modify `CMakelists.txt` to use it. The interface is defined in `components/imu/imu.h`.

```cmake
# Select IMU module
set(IMU_MODULE hi229) # hi229 or bno08x
add_definitions(-DCONFIG_IMU_TYPE=IMU_TYPE_HI229) # IMU_TYPE_HI229 or IMU_TYPE_BNO08X, depending on the selected module
```
## Operators' Guide

The detailed guide can be found in [docs/manual.md](docs/manual.md)