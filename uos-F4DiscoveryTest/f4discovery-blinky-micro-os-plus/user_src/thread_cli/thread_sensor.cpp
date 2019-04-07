/*
 * sensor_handler.cpp
 *
 *  Created on: Apr 7, 2019
 *      Author: quzhi
 */

#include <cmsis-plus/rtos/os.h>

using namespace os;
using namespace os::rtos;


void *thread_sensor(void *args);
thread_inclusive<1024> thread_sensor_object { "thread_cli", thread_sensor, nullptr };


extern "C" {
}

void *thread_sensor(void *args)
{
    while(1)
    {
        sysclock.sleep_for (5000);
    }
    return NULL;
}
