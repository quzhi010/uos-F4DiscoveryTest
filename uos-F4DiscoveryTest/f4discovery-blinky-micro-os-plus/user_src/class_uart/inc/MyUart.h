/*
 * MyUart.h
 *
 *  Created on: Mar 29, 2019
 *      Author: quzhi
 */

#ifndef CLASS_UART_MYUART_H_
#define CLASS_UART_MYUART_H_

#include "MyGPIO.h"
#include "MyDMA.h"
#include "stm32f4xx_hal_uart.h"

namespace STM32F407 {
    typedef struct
    {
        USART_TypeDef               *serial_instantce;
        UART_InitTypeDef            *serial_init;
        DMA_Stream_TypeDef          *rx_dma_instance;
        DMA_InitTypeDef             *rx_dma_init;
        DMA_Stream_TypeDef          *tx_dma_instance;
        DMA_InitTypeDef             *tx_dma_init;
        GPIO_TypeDef                *rx_gpio_port;
        GPIO_InitTypeDef            *rx_gpio_init;
        GPIO_TypeDef                *tx_gpio_port;
        GPIO_InitTypeDef            *tx_gpio_init;
        IRQn_Type                   tx_irq;
        IRQn_Type                   rx_irq;
        uint32_t                    uart_clck_mask;
        uint32_t                    dma_clck_mask;
        uint32_t                    gpio_clck_mask;
    }uart_instance_t;

    class MyUart {
    private:
        uart_instance_t *uart_instance;
        MyGPIO              *gpio_rx;
        MyGPIO              *gpio_tx;
        void uart_setconfig(void);
    public:
        MyDMA               *rx_dma;
        MyDMA               *tx_dma;
        MyUart(uart_instance_t *uart_inst);
        void transmit(uint8_t *data, uint8_t size);
        void receive(volatile uint8_t *data, uint8_t size);
        void dma_start_it(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
        void dma_tx_pause(void);
        void dma_rx_pause(void);
        void dma_tx_resume(void);
        void dma_rx_resume(void);
        void end_tx(void);
        void end_rx(void);
        void isr_cb(void);
        virtual ~MyUart();
    };
}

#endif /* CLASS_UART_MYUART_H_ */
