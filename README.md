# RFMarkIt Firmware

This is the firmware for RFMarkIt project, which is a project for a active marker system for motion capture / 6DOF tracking.

## Get Started

First, setup esp-idf environment(v4.4.4) according to the official guide [here](https://docs.espressif.com/projects/esp-idf/zh_CN/v4.4.4/esp32/get-started/)

Now, you need to choose from hardware v1.0 and v2.0. v1.0 uses esp32 and hi229, while v2.0 uses esp32s3 and bno085

```shell
make set_hardware_v1
make set_hardware_v2
```

If you want to change other settings (e.g. use linear acceleration), you can run `idf.py menuconfig` in the project root directory.

```shell
idf.py menuconfig
```

```shell

Then, build the project by running `idf.py build` in the project root directory.

```shell
idf.py build
```

To flash an node, run `idf.py flash monitor` in the project root directory.

```shell
idf.py -p <PORT> flash monitor
```

## Developers' Guide

The project is organized as follows:

- `components` system components
  - `components/apps` applications that runs with RTOS
  - `components/ble` Bluetooth Low Energy(BLE) components
  - `components/blink` functions to operate LED
  - `components/imu` imu interface
  - `components/rest_controller` RESTful API controller
  - `components/sys` supporting modules for system
- `docs` documents
- `include` global headers
- `lib` common libraries
  - `lib/battery` battery
  - `lib/bno085` bno085 library
  - `lib/hi229` hi229 library
  - `lib/libtcp` tcp library
  - `lib/libudp` udp library
  - `lib/ring_buf` ring buffer library
  - `lib/spatial` spatial math library
- `main` entrypoint of firmware
- `scripts` helper scripts
- `tests` function tests

## Operators' Guide

The detailed guide can be found in [docs/manual.md](docs/manual.md)