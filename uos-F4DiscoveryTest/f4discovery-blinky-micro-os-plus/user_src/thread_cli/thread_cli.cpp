/*
 * thread_cli.cpp
 *
 *  Created on: Mar 28, 2019
 *      Author: quzhi
 */
#include <cmsis-plus/rtos/os.h>
#include "MyUart.h"
#include "string.h"
#include "globals.h"

using namespace os;
using namespace os::rtos;

typedef enum
{
    PERIODIC_RX_CHECK = 0,
    RX_RECEIVED = 1,
    TX_FINISHED = 2
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

STM32F407::uart_instance_t cli_uart_inst =
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
    tx_gpio_init: &cli_gpio_tx_init,
    tx_irq: USART2_IRQn,
    rx_irq: DMA1_Stream5_IRQn,
    uart_clck_mask: RCC_APB1ENR_USART2EN,
    dma_clck_mask: RCC_AHB1ENR_DMA1EN,
    gpio_clck_mask: RCC_AHB1ENR_GPIOAEN
};

static STM32F407::MyUart cli_uart(&cli_uart_inst);

message_queue_typed<cli_msg_t> cli_queue { 100 };

void *thread_cli(void *args);
thread_inclusive<1024> thread_cli_object { "thread_cli", thread_cli, nullptr };

uint8_t tx_test2[14] = "Hello quzhi\r\n";

extern "C" {
// RX DMA full interrupt
void DMA1_Stream5_IRQHandler(void)
{
    cli_uart.rx_dma->isr_cb();
    cli_msg_t msg_rx_received = RX_RECEIVED;
    cli_queue.try_send(&msg_rx_received);
}
// TX DMA empty interrupt
void USART2_IRQHandler(void)
{
    cli_uart.isr_cb();
    cli_msg_t msg_tx_finished = TX_FINISHED;
    cli_queue.try_send(&msg_tx_finished);
}
}

void func(void* args)
{
    cli_msg_t msg_periodic = PERIODIC_RX_CHECK;
    cli_queue.try_send(&msg_periodic);
}

void *thread_cli(void *args)
{
    /*
    char str[] = "strtok needs to be called several times to split a string";
    int init_size = strlen(str);
    char delim[] = " ";

    char *ptr = strtok(str, delim);

    while(ptr != NULL)
    {
        trace_printf("str splitted '%s'\n", ptr);
        ptr = strtok(NULL, delim);
    }

    for (int i = 0; i < init_size; i++)
    {
        trace_printf("%d ", str[i]);
    }
    trace_printf ("\n");
    */



    cli_msg_t cli_msg;
    uint8_t i;
    volatile uint8_t rx_buffer = 0;
    uint8_t command_buffer[128] = {};
    uint8_t command_index = 0;
    timer timer {func, nullptr, timer::periodic_initializer};

    //SET_BIT(cli_uart_inst.serial_instantce->CR1, USART_CR1_RXNEIE);
    cli_uart.receive(&rx_buffer, 1);
    timer.start(sysclock.frequency_hz);
    while(1)
    {
        cli_queue.receive(&cli_msg);
        if (RX_RECEIVED == cli_msg)
        {
            command_buffer[command_index] = rx_buffer;
            if ('\r' == command_buffer[command_index])
            {
                command_index++;
                cli_uart.transmit(command_buffer, command_index);
                while (TX_FINISHED != cli_msg)
                {
                    cli_queue.receive(&cli_msg);
                }
                for (i = 0; i <= command_index; i++)
                {
                    command_buffer[i] = 0;
                }
                command_index = 0;
            }
            else
            {
                command_index = (command_index + 1)%128;
            }
        }
        /*
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
        */
    }
    return NULL;
}
