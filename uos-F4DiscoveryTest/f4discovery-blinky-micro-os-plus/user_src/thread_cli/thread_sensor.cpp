/*
 * sensor_handler.cpp
 *
 *  Created on: Apr 7, 2019
 *      Author: quzhi
 */

#include <cmsis-plus/rtos/os.h>
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_gpio.h"

using namespace os;
using namespace os::rtos;


void *thread_sensor(void *args);
thread_inclusive<1024> thread_sensor_object { "thread_cli", thread_sensor, nullptr };


extern "C" {
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
  HAL_SPI_DMAStop(hspi);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_TxRxCpltCallback should be implemented in the user file
  */
}

extern SPI_HandleTypeDef hspi1;
uint8_t buffer_tx[2] = {3,1};
uint8_t buffer_rx[2] = {0, 0};
HAL_StatusTypeDef ret;
void *thread_sensor(void *args)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	buffer_tx[0] = 0x20;
	buffer_tx[1] = 0x97;
	ret = HAL_SPI_Transmit_DMA(&hspi1, buffer_tx, 2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    sysclock.sleep_for (1000);
    while(1)
    {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    	buffer_tx[0] = 0x20 | 0x80;
    	/*
    	ret = HAL_SPI_Transmit_DMA(&hspi1, buffer_tx, 1);
    	HAL_SPI_Receive(&hspi1, buffer_rx, 1, 50);
    	*/
    	//ret = HAL_SPI_Transmit_DMA(&hspi1, buffer_tx, 1);
    	buffer_tx[1] = 0x00;
    	HAL_SPI_TransmitReceive_DMA(&hspi1, buffer_tx, buffer_rx, 2);
    	/*
        sysclock.sleep_for (1);
    	ret = HAL_SPI_Receive_DMA(&hspi1, buffer_rx, 1);
    	buffer_tx[0] = 0x23;
    	buffer_tx[1] = 0xc8;
    	ret = HAL_SPI_Transmit_DMA(&hspi1, buffer_tx, 2);
    	*/
        trace_printf ("spi send: %d", ret);
        sysclock.sleep_for (2000);
    }
    return NULL;
}
