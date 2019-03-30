/*
 * MyUart.cpp
 *
 *  Created on: Mar 29, 2019
 *      Author: quzhi
 */

#include "MyUart.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    ;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    int p = 3;
    p++;
}

MyUart::MyUart(UART_HandleTypeDef *uart_handle) {
    // TODO Auto-generated constructor stub
    tx_buf = NULL;
    rx_buf = NULL;
    tx_size = 0;
    rx_size = 0;
    uart_module = uart_handle;
}

void MyUart::transmit_config(uint8_t *data, uint8_t size)
{
    tx_buf = data;
    tx_size = size;
}

void MyUart::transmit(void) {
    HAL_UART_Transmit_IT(uart_module, tx_buf, tx_size);
}

void MyUart::receive_config(uint8_t *data, uint8_t size)
{
    rx_buf = data;
    rx_size = size;
}

void MyUart::receive(void) {
    HAL_UART_Transmit_IT(uart_module, tx_buf, tx_size);
}

MyUart::~MyUart() {
    // TODO Auto-generated destructor stub
}
