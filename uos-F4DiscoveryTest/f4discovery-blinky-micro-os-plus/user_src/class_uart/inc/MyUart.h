/*
 * MyUart.h
 *
 *  Created on: Mar 29, 2019
 *      Author: quzhi
 */

#ifndef CLASS_UART_MYUART_H_
#define CLASS_UART_MYUART_H_

#include "usart.h"

class MyUart {
private:
    uint8_t             *tx_buf;
    uint8_t             *rx_buf;
    uint8_t             tx_size;
    uint8_t             rx_size;
    UART_HandleTypeDef  *uart_module;
public:
    MyUart(UART_HandleTypeDef *uart_handle);
    void transmit_config(uint8_t *data, uint8_t size);
    void transmit(void);
    void receive_config(uint8_t *data, uint8_t size);
    void receive(void);
    virtual ~MyUart();
};

#endif /* CLASS_UART_MYUART_H_ */
