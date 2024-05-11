#ifndef IMU_H_
#define IMU_H_

#include "settings.h"

#if CONFIG_IMU_TYPE == IMU_TYPE_GY95
#include "gy95.h"
#elif CONFIG_IMU_TYPE == IMU_TYPE_HI229

#include "hi229.h"

#endif

// TODO: fix gy95 broken interface, make it align with hi229
#endif