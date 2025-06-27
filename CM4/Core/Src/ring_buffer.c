/*
 * ring_buffer.c
 *
 *  Created on: Jun 25, 2025
 *      Author: birdd
 */

#include "ring_buffer.h"

void ring_buffer_init(RingBuffer *rb) {
    rb->head = rb->tail = 0;
}

bool ring_buffer_push(RingBuffer *rb, uint8_t byte) {
    uint16_t next = (rb->head + 1) % RING_BUFFER_SIZE;
    if (next == rb->tail) return false;
    rb->data[rb->head] = byte;
    rb->head = next;
    return true;
}

bool ring_buffer_pop(RingBuffer *rb, uint8_t *byte) {
    if (rb->head == rb->tail) return false;
    *byte = rb->data[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
    return true;
}

bool ring_buffer_is_empty(RingBuffer *rb) {
    return rb->head == rb->tail;
}

bool ring_buffer_is_full(RingBuffer *rb) {
    return ((rb->head + 1) % RING_BUFFER_SIZE) == rb->tail;
}

