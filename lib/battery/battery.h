//
// Created by liyutong on 2024/4/30.
//

#ifndef BATTERY_
#define BATTERY_

#define CONFIG_BATTERY_EN_VALUE             0

esp_err_t battery_msp_init(void);

int battery_read_level(void);

// TODO: add docs
#endif
