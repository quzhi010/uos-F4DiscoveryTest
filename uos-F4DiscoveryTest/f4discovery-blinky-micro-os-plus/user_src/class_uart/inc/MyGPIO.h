/*
 * MyGPIO.h
 *
 *  Created on: Mar 31, 2019
 *      Author: quzhi
 */

#ifndef CLASS_UART_SRC_MYGPIO_H_
#define CLASS_UART_SRC_MYGPIO_H_

#include "main.h"

namespace STM32F407 {
    class MyGPIO {
        private:
            GPIO_TypeDef    *GPIO_Port;
            uint32_t        GPIO_Pin;
        public:
            MyGPIO(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
            virtual ~MyGPIO();
    };
}

#endif /* CLASS_UART_SRC_MYGPIO_H_ */
