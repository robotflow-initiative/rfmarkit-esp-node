//
// Created by liyutong on 2024/5/1.
//
#ifndef RING_BUF_H_
#define RING_BUF_H_

#include "esp_system.h"

typedef struct {
    uint16_t size;
    uint16_t item_width;

    uint8_t *buf;
    bool static_buf;

    uint16_t _head;
    int64_t head;
    SemaphoreHandle_t mutex;
} ring_buf_t;

esp_err_t ring_buf_init(ring_buf_t *rb, uint16_t size, uint16_t item_width, uint8_t *buf, bool static_buf);

esp_err_t ring_buf_reset(ring_buf_t *rb);

esp_err_t ring_buf_reset_safe(ring_buf_t *rb);

esp_err_t ring_buf_push(ring_buf_t *rb, void *item);

esp_err_t ring_buf_push_safe(ring_buf_t *rb, void *item);

esp_err_t ring_buf_peek(ring_buf_t *rb, void *item, int64_t index, int64_t *index_out);

esp_err_t ring_buf_free(ring_buf_t *rb);

#endif
