/*
 * thread_cli.cpp
 *
 *  Created on: Mar 28, 2019
 *      Author: quzhi
 */
#include <cmsis-plus/rtos/os.h>
#include "MyUart.h"

using namespace os;
using namespace os::rtos;

void *thread_cli(void *args);
thread_inclusive<1024> thread_cli_object { "thread_cli", thread_cli, nullptr };

uint8_t tx_test2[14] = "Hello quzhi\r\n";
uint8_t rx_test[15] = {0};
void *thread_cli(void *args)
{
    int i;
    int j;
    MyUart cli_uart(&huart2);
    cli_uart.transmit_config(tx_test2, 13);
    SET_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE);
    HAL_UART_Receive_DMA(&huart2, rx_test, 15);
    while(1)
    {
        HAL_UART_DMAPause(&huart2);
        for (i = 0; i < 15; i++)
        {
            if (rx_test[i] == '\r')
            {
                HAL_UART_DMAStop(&huart2);
                for (j = 0; j < 15; j++)
                {
                    rx_test[j] = 0;
                }
                HAL_UART_Receive_DMA(&huart2, rx_test, 15);
            }
            else
            {
                HAL_UART_DMAResume(&huart2);
            }
        }
        cli_uart.transmit();
        sysclock.sleep_for (sysclock.frequency_hz);
    }
    return NULL;
}
