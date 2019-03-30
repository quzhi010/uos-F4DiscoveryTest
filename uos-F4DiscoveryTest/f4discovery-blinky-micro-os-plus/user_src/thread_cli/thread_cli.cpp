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

typedef enum
{
    PERIODIC_RX_CHECK = 0,
    TX_FINISHED = 1
} cli_msg_t;

DMA_InitTypeDef cli_uart_rx_init =
{
    .Channel = DMA_CHANNEL_4,
    .Direction = DMA_PERIPH_TO_MEMORY,
    .PeriphInc = DMA_PINC_DISABLE,
    .MemInc = DMA_MINC_ENABLE,
    .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
    .MemDataAlignment = DMA_MDATAALIGN_BYTE,
    .Mode = DMA_CIRCULAR,
    .Priority = DMA_PRIORITY_LOW,
    .FIFOMode = DMA_FIFOMODE_DISABLE,
    .FIFOThreshold = 0,
    .MemBurst = 0,
    .PeriphBurst = 0
};

DMA_InitTypeDef cli_uart_tx_init = {
    .Channel = DMA_CHANNEL_4,
    .Direction = DMA_MEMORY_TO_PERIPH,
    .PeriphInc = DMA_PINC_DISABLE,
    .MemInc = DMA_MINC_ENABLE,
    .PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
    .MemDataAlignment = DMA_MDATAALIGN_BYTE,
    .Mode = DMA_NORMAL,
    .Priority = DMA_PRIORITY_LOW,
    .FIFOMode = DMA_FIFOMODE_DISABLE,
    .FIFOThreshold = 0,
    .MemBurst = 0,
    .PeriphBurst = 0
};

UART_InitTypeDef cli_uart_init = {
    .BaudRate = 115200,
    .WordLength = UART_WORDLENGTH_8B,
    .StopBits = UART_STOPBITS_1,
    .Parity = UART_PARITY_NONE,
    .Mode = UART_MODE_TX_RX,
    .HwFlowCtl = UART_HWCONTROL_NONE,
    .OverSampling = UART_OVERSAMPLING_16
};

GPIO_InitTypeDef cli_gpio_rx_init =
{
    .Pin = GPIO_PIN_3,
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_PULLUP,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    .Alternate = GPIO_AF7_USART2
};

GPIO_InitTypeDef cli_gpio_tx_init =
{
    .Pin = GPIO_PIN_2,
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_PULLUP,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    .Alternate = GPIO_AF7_USART2
};

uart_instance_t cli_uart_inst =
{
    serial_instantce: USART2,
    serial_init: &cli_uart_init,
    rx_dma_instance: DMA1_Stream5,
    rx_dma_init: &cli_uart_rx_init,
    tx_dma_instance: DMA1_Stream6,
    tx_dma_init: &cli_uart_tx_init,
    rx_gpio_port: GPIOA,
    rx_gpio_init: &cli_gpio_rx_init,
    tx_gpio_port: GPIOA,
    tx_gpio_init: &cli_gpio_tx_init
};

static MyUart cli_uart(&cli_uart_inst);

message_queue_typed<cli_msg_t> cli_queue { 20 };

void *thread_cli(void *args);
thread_inclusive<1024> thread_cli_object { "thread_cli", thread_cli, nullptr };

uint8_t tx_test2[14] = "Hello quzhi\r\n";
uint8_t rx_test[15] = {0};

extern "C" {
// RX DMA full interrupt
void DMA1_Stream5_IRQHandler(void)
{
    cli_uart.rx_dma->isr_cb();
}
// TX DMA empty interrupt
void USART2_IRQHandler(void)
{
    cli_uart.isr_cb();
}
}

void func(void* args)
{
    cli_msg_t msg_periodic = PERIODIC_RX_CHECK;
    cli_queue.try_send(&msg_periodic);
}

void *thread_cli(void *args)
{
    cli_msg_t cli_msg;
    int i;
    int j;
    timer timer {func, nullptr, timer::periodic_initializer};

    SET_BIT(cli_uart_inst.serial_instantce->CR1, USART_CR1_RXNEIE);
    cli_uart.receive(rx_test, 15);
    timer.start(sysclock.frequency_hz);
    while(1)
    {
        cli_queue.receive(&cli_msg);
        cli_uart.dma_rx_pause();
        for (i = 0; i < 15; i++)
        {
            if (rx_test[i] == '\r')
            {
                cli_uart.rx_dma->abort();
                for (j = 0; j < 15; j++)
                {
                    rx_test[j] = 0;
                }
                cli_uart.receive(rx_test, 15);
            }
            else
            {
                cli_uart.dma_rx_resume();
            }
        }
        cli_uart.transmit(tx_test2, 13);
    }
    return NULL;
}
