#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

#define RING_BUFFER_SIZE 256

typedef struct {
    uint8_t data[RING_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer;

void ring_buffer_init(RingBuffer *rb);
bool ring_buffer_push(RingBuffer *rb, uint8_t byte);
bool ring_buffer_pop(RingBuffer *rb, uint8_t *byte);
bool ring_buffer_is_empty(RingBuffer *rb);
bool ring_buffer_is_full(RingBuffer *rb);

#endif
