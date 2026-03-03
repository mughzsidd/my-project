#include "heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)

// Students should be provided the above code (includes and defines) and the function declarations in this file.
// They can figure out the rest.

// Allocation bitmap: 0 = free, 1 = used

// Add you code below

#define FREE  0x00
#define USED   0xFF

static uint8_t heap_memory[HEAP_SIZE];

static uint8_t block_info[BLOCK_COUNT];

void heap_init(void) {
    for (int i = 0; i < BLOCK_COUNT; i++) {
        block_info[i] = FREE;
    }
}

void* heap_alloc(size_t size) {
    if (size == 0 || size > HEAP_SIZE) return NULL;

    // Calculate how many 16-byte blocks we need
    // (size + 15) / 16 is integer math for rounding up
    size_t blocks_needed = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;

    // Search for a contiguous sequence of free blocks
    size_t consecutive_free = 0;
    int start_index = -1;

    for (int i = 0; i < BLOCK_COUNT; i++) {
        if (block_info[i] == FREE) {
            if (consecutive_free == 0) start_index = i;
            consecutive_free++;

            if (consecutive_free == blocks_needed) {
                block_info[start_index] = blocks_needed; // Head stores the size
                
                for (int j = 1; j < blocks_needed; j++) {
                    block_info[start_index + j] = USED;
                }

                return &heap_memory[start_index * BLOCK_SIZE];
            }
        } else {
            // Hit a used block, reset our search counter
            consecutive_free = 0;
        }
    }

    return NULL; 
}

void heap_free(void* ptr) {
    if (ptr == NULL) return;

    uint8_t* p = (uint8_t*)ptr;
    if (p < heap_memory || p >= (heap_memory + HEAP_SIZE)) return;

    size_t offset = p - heap_memory;
    
    if (offset % BLOCK_SIZE != 0) return; 

    int index = offset / BLOCK_SIZE;

    uint8_t blocks_to_free = block_info[index];

    if (blocks_to_free == FREE || blocks_to_free == USED) return;

    for (int i = 0; i < blocks_to_free; i++) {
        block_info[index + i] = FREE;
    }
}