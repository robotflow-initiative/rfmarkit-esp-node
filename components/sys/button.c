//
// Created by liyutong on 2024/5/7.
//
#include <string.h>
#include <sys/cdefs.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_sleep.h"
#include "esp_log.h"

#include "driver/gpio.h"

#include "sys.h"
#include "settings.h"

typedef enum {
    IDLE,
    PRESSED,
    RELEASED,
} button_state_t;

typedef enum {
    NONE,
    CLICK,
    LONG_PRESS,
} button_event_t;

typedef void (*button_isr_handler_t)();

static xQueueHandle button_event_queue;
static button_state_t button_state = IDLE;
static uint32_t press_timestamp;
static uint32_t release_timestamp;

static const char *TAG = "sys.button      ";

/**
 * @brief Button ISR handler
 * use a state machine to handle gpio isr events
 * @param params
**/
static void IRAM_ATTR button_fn_isr_handler(void *params) {
    switch (button_state) {
        case IDLE:
            button_state = PRESSED;
            press_timestamp = xTaskGetTickCount();
            break;
        case RELEASED:
        case PRESSED:
            button_state = RELEASED;
            release_timestamp = xTaskGetTickCount();
            break;
    }
    if (button_state == RELEASED) {
        if (button_event_queue) {
            button_event_t ev;
            if (release_timestamp - press_timestamp > CONFIG_LONG_PRESS_DURATION) {
                ev = LONG_PRESS;
            } else {
                ev = CLICK;
            }
            /** Send the event to the button event queue **/
            xQueueSendFromISR(button_event_queue, &ev, NULL);
        }
        button_state = IDLE;
    }
}

/** The button is used to control the Marker:
 *
 * 1. If the button is long-pressed, the Marker will enter deep sleep mode.
 * 2. If the button is clicked, the Marker will switch between active / standby mode.
 * 3. If the button is double clicked, the Marker will calibrate
**/
_Noreturn void button_daemon(void *params) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    button_event_t ev;
    while (1) {
        if (xQueueReceive(button_event_queue, &ev, portMAX_DELAY) == pdTRUE) {
            switch (ev) {
                case NONE:
                    break;
                case CLICK:
                    if (xQueueReceive(button_event_queue, &ev, CONFIG_DOUBLE_CLICK_DURATION) == pdTRUE) {
                        if (ev == CLICK) {
                            ESP_LOGD(TAG, "button double clicked");
                            //TODO: make use of this function
                        }
                    } else {
                        ESP_LOGD(TAG, "button clicked");
                        sys_set_operation_mode(!g_mcu.state.active);
                    }
                    break;
                case LONG_PRESS:
                    ESP_LOGD(TAG, "button long pressed");
                    power_mgmt_on_enter_deep_sleep(false);
                    break;
            }
        }
    }
}

/**
 * Initialize the button
 * 1. Create a button event queue
 * 2. Create a task to parse button events
 * 3. Install the ISR service and add the button ISR handler
 * 4. Notify the button_daemon task
**/
void sys_init_buttons() {
    int pins[] = {CONFIG_BUTTON_FN_GPIO_PIN, -1}; // -1 is used to indicate the end of the array
    button_isr_handler_t isr_handlers[] = {button_fn_isr_handler, NULL}; // NULL is used to indicate the end of the array
    button_event_queue = xQueueCreate(10, sizeof(button_event_t));

    /** A task is created to parse button events **/
    TaskHandle_t button_daemon_task;
    xTaskCreate(button_daemon, "button_daemon", 2048, NULL, 10, &button_daemon_task);

    /** Install the ISR service and add the button ISR handler **/
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    for (int i = 0; pins[i] >= 0; i++) {
        gpio_config_t io_config = {
            .pin_bit_mask = (1ull << pins[i]),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE
        };
        gpio_config(&io_config);
        gpio_isr_handler_add(pins[i], isr_handlers[i], NULL);
    }

    xTaskNotifyGive(button_daemon_task);
}

