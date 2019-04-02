/*
 * sharedmemory.cpp
 *
 *  Created on: Apr 2, 2019
 *      Author: quzhi
 */

#include "sharedmemory.h"

shared_memory::shared_memory::shared_memory(uint32_t size_of_queue, uint32_t num_of_queue) {
    // TODO Auto-generated constructor stub
    uint32_t num_of_bytes = size_of_queue * num_of_queue;

    buffer_length = size_of_queue;
    buffer_left = num_of_queue;
    allocated_memory = new uint8_t[num_of_bytes];
    msg_queue_ptr_queue = new os::rtos::message_queue_typed<uint8_t*> (num_of_queue);
    for (uint32_t i = 0; i < num_of_bytes; i++)
    {
        allocated_memory[i] = 0;
    }
    for (uint32_t i = 0; i < num_of_queue; i++)
    {
        uint8_t *temp = &allocated_memory[size_of_queue * i];
        msg_queue_ptr_queue->send(&temp);
    }
}

uint8_t * shared_memory::shared_memory::get_buffer(void) {
    // TODO Auto-generated constructor stub
    uint8_t *buffer_ptr;
    if (0 == msg_queue_ptr_queue->try_receive(&buffer_ptr))
    {
        buffer_left--;
        return buffer_ptr;
    }
    else
    {
        return NULL;
    }
}

uint32_t shared_memory::shared_memory::ret_buffer(uint8_t *buffer_ptr) {
    // TODO Auto-generated constructor stub
    uint8_t *buffer_ptr_reg;
    if (0 == msg_queue_ptr_queue->try_send(&buffer_ptr_reg))
    {
        buffer_left++;
        return 0;
    }
    else
    {
        return 1;
    }
}

shared_memory::shared_memory::~shared_memory() {
    // TODO Auto-generated destructor stub
    delete(allocated_memory);
    delete(msg_queue_ptr_queue);
}

