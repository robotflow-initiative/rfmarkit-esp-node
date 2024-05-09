//
// Created by liyutong on 2024/5/1.
//
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "ring_buf.h"

/**
 * @brief The ring buffer is a FIFO buffer, which is used to store data in a circular way.
 *
 * @param size The size of the ring buffer.
 * @param item_width The width of each item in the ring buffer.
 * @param buf The buffer to store the data.
 * @param static_buf Whether the buffer is static or not.
**/
esp_err_t ring_buf_init(ring_buf_t *rb, uint16_t size, uint16_t item_width, uint8_t *buf, bool static_buf) {
    if (rb == NULL || size == 0 || item_width == 0) return ESP_ERR_INVALID_ARG;
    /** Set the size of the ring buffer **/
    rb->size = size;
    rb->item_width = item_width;
    rb->head = 1;
    rb->_head = (rb->_head + 1) % rb->size;

    /** Allocate memory for the buffer **/
    if (buf == NULL) {
        // If the buffer is not static, allocate memory for the buffer
        if (static_buf) return ESP_ERR_INVALID_ARG;
        buf = (uint8_t *) malloc(size * item_width);
        if (buf == NULL) return ESP_ERR_NO_MEM;
        rb->buf = buf;
    } else {
        // If the buffer is static, the buffer should be provided by the user, user should make sure the buffer is valid and the size is correct
        if (static_buf) {
            rb->buf = buf;
        } else {
            return ESP_ERR_INVALID_ARG;
        }
    }
    rb->static_buf = static_buf;

    /** Create a mutex for the ring buffer **/
    rb->mutex = xSemaphoreCreateMutex();
    return ESP_OK;
}

/**
 * @brief Reset the ring buffer.
**/
esp_err_t ring_buf_reset(ring_buf_t *rb) {
    rb->head = 1;
    rb->_head = (rb->_head + 1) % rb->size;
    return ESP_OK;
}

/**
 * @brief Reset the ring buffer safely.
**/
esp_err_t ring_buf_reset_safe(ring_buf_t *rb) {
    if (xSemaphoreTake(rb->mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;
    esp_err_t ret = ring_buf_reset(rb);
    xSemaphoreGive(rb->mutex);
    return ret;

}


/**
 * @brief Push an item into the ring buffer.
 *
 * @param item The item to be pushed into the ring buffer.
**/
esp_err_t IRAM_ATTR ring_buf_push(ring_buf_t *rb, void *item) {
    if (rb == NULL || item == NULL) return ESP_ERR_INVALID_ARG;
    memcpy(rb->buf + (rb->_head * rb->item_width), item, rb->item_width);
    rb->_head = (rb->_head + 1) % rb->size;
    rb->head++;
    return ESP_OK;
}


/**
 * @brief Push an item into the ring buffer safely.
 *
 * @param item The item to be pushed into the ring buffer.
**/
esp_err_t IRAM_ATTR ring_buf_push_safe(ring_buf_t *rb, void *item) {
    if (xSemaphoreTake(rb->mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;
    esp_err_t ret = ring_buf_push(rb, item);
    xSemaphoreGive(rb->mutex);
    return ret;
}

/**
 * @brief Peek an item from the ring buffer.
 *
 * @param item The item to be peeked from the ring buffer.
 * @param index The index of the item to be peeked.
 *              If the index is negative or greater than the head of the ring buffer, the last item will be peeked.
 *              If the index is greater than the head of the ring buffer, ESP_ERR_INVALID_ARG will be returned.
**/
esp_err_t IRAM_ATTR ring_buf_peek(ring_buf_t *rb, void *item, int64_t index, int64_t *index_out) {
    if (rb == NULL || item == NULL) return ESP_ERR_INVALID_ARG;
    if (index >= rb->head) return ESP_ERR_INVALID_ARG;
    if (index < 0 || index <= (rb->head - rb->size)) index = rb->head - 1;
    if (index_out != NULL) *index_out = index; // Return the index of the item peeked
    int64_t place = index % rb->size;
    memcpy(item, rb->buf + (place * rb->item_width), rb->item_width);
    return ESP_OK;
}

/**
 * @brief Free the ring buffer.
**/
esp_err_t ring_buf_free(ring_buf_t *rb) {
    if (rb == NULL) return ESP_ERR_INVALID_ARG;
    if (!rb->static_buf) free(rb->buf);
    vSemaphoreDelete(rb->mutex);
    return ESP_OK;
}