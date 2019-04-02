/*
 * sharedmemory.h
 *
 *  Created on: Apr 2, 2019
 *      Author: quzhi
 */

#include <cmsis-plus/rtos/os.h>
#ifndef SHAREDMEMORY_SRC_SHAREDMEMORY_H_
#define SHAREDMEMORY_SRC_SHAREDMEMORY_H_

namespace shared_memory {
    class shared_memory {
        private:
            os::rtos::message_queue_typed<uint8_t*> *msg_queue_ptr_queue;
            uint8_t *allocated_memory;
        public:
            uint32_t buffer_length;
            uint32_t buffer_left;
            shared_memory(uint32_t size_of_queue, uint32_t num_of_queue);
            uint8_t* get_buffer(void);
            uint32_t ret_buffer(uint8_t *buffer_ptr);
            virtual ~shared_memory();
    };
}

#endif /* SHAREDMEMORY_SRC_SHAREDMEMORY_H_ */
