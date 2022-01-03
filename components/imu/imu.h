#ifndef _IMU_H
#define _IMU_H

#include "settings.h"

#if CONFIG_IMU_TYPE == IMU_TYPE_GY95

#include "gy95.h"

#elif CONFIG_IMU_TYPE == IMU_TYPE_HI229
#include "hi229.h"
#endif
#endif