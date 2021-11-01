#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <esp_timer.h>
#include "driver/gpio.h"
#include "apps.h"

#define BLINK_INTERVAL_MS  40 // Blink interval in ms
#define BLINK_SEQ_LEN 24 // Blink sequence length
#define LED_GPIO_PIN 2

static char blink_seq[BLINK_SEQ_LEN];
static int blink_idx;
static ETSTimer blink_timer;

void blink_timeout () {
    if (blink_idx >= BLINK_SEQ_LEN) blink_idx = 0;
    switch (blink_seq[blink_idx]) {
        case 0:
            gpio_set_level(LED_GPIO_PIN, 0);
            printf("0");
            break;
        default:
            gpio_set_level(LED_GPIO_PIN, 1);
            printf("1");
            break;
    }
    blink_idx++;
    ets_timer_disarm(&blink_timer);
    ets_timer_arm(&blink_timer, BLINK_INTERVAL_MS, 0);
}

void app_blink(char * seq, int length) {
    blink_idx = 0;
    memcpy(blink_seq, seq, length);

    // Init GPIO
    gpio_config_t io_config = {
        .pin_bit_mask = (1ull << LED_GPIO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_config);

    // Arm timers
    ets_timer_disarm(&blink_timer);
    ets_timer_setfn(&blink_timer, blink_timeout, NULL);
    ets_timer_arm(&blink_timer, BLINK_INTERVAL_MS, 0);
}