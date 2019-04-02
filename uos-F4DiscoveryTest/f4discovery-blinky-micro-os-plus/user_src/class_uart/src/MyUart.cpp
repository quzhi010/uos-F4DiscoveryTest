/*
 * MyUart.cpp
 *
 *  Created on: Mar 29, 2019
 *      Author: quzhi
 */

#include "MyUart.h"

using namespace STM32F407;

MyUart::MyUart(uart_instance_t *uart_inst) {
    // TODO Auto-generated constructor stub
    uart_instance = uart_inst;

    __IO uint32_t tmpreg = 0x00U;
    RCC->AHB1ENR |= uart_inst->dma_clck_mask;
    tmpreg = RCC->AHB1ENR  & uart_inst->dma_clck_mask;
    UNUSED(tmpreg);
    RCC->APB1ENR |= uart_inst->uart_clck_mask;
    tmpreg = RCC->APB1ENR  & uart_inst->uart_clck_mask;
    UNUSED(tmpreg);
    RCC->AHB1ENR |= uart_inst->gpio_clck_mask;
    tmpreg = RCC->AHB1ENR & uart_inst->gpio_clck_mask;
    UNUSED(tmpreg);

    HAL_NVIC_SetPriority(uart_inst->rx_irq, 15, 15);
    HAL_NVIC_EnableIRQ(uart_inst->rx_irq);
    HAL_NVIC_SetPriority(uart_inst->tx_irq, 15, 15);
    HAL_NVIC_EnableIRQ(uart_inst->tx_irq);

    gpio_rx = new MyGPIO(uart_inst->rx_gpio_port, uart_inst->rx_gpio_init);
    gpio_rx = new MyGPIO(uart_inst->tx_gpio_port, uart_inst->tx_gpio_init);
    tx_dma = new MyDMA(uart_inst->tx_dma_instance, uart_inst->tx_dma_init);
    rx_dma = new MyDMA(uart_inst->rx_dma_instance, uart_inst->rx_dma_init);

    uart_instance->serial_instantce->CR1 = 0;
    uart_inst->serial_instantce->CR1 &=  ~USART_CR1_UE;
    uart_setconfig();
    CLEAR_BIT(uart_inst->serial_instantce->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
    CLEAR_BIT(uart_inst->serial_instantce->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));
    uart_inst->serial_instantce->CR1 |=  USART_CR1_UE;
}

void MyUart::transmit(uint8_t *data, uint8_t size) {
    uint32_t *tmp;

    tmp = (uint32_t*)&data;
    tx_dma->dma_start(*(uint32_t*)tmp, (uint32_t)&uart_instance->serial_instantce->DR, size);

    uart_instance->serial_instantce->SR = ~UART_FLAG_TC;

    SET_BIT(uart_instance->serial_instantce->CR3, USART_CR3_DMAT);
    SET_BIT(uart_instance->serial_instantce->CR1, USART_CR1_TCIE);
}

void MyUart::receive(volatile uint8_t *data, uint8_t size) {
    uint32_t *tmp;

    /* Enable the DMA Stream */
    tmp = (uint32_t*)&data;
    rx_dma->dma_start_it((uint32_t)&uart_instance->serial_instantce->DR, *(uint32_t*)tmp, size);

    SET_BIT(uart_instance->serial_instantce->CR3, USART_CR3_DMAR);
}

MyUart::~MyUart() {
    // TODO Auto-generated destructor stub
    delete(gpio_rx);
    delete(gpio_tx);
    delete(tx_dma);
    delete(rx_dma);

    RCC->AHB1ENR &= (   (~uart_instance->uart_clck_mask) &
                        (~uart_instance->dma_clck_mask) &
                        (~uart_instance->gpio_clck_mask)   );

    HAL_NVIC_DisableIRQ(uart_instance->tx_irq);
    HAL_NVIC_DisableIRQ(uart_instance->rx_irq);
    uart_instance->serial_instantce->CR1 = 0;
    uart_instance->serial_instantce->CR2 = 0;
    uart_instance->serial_instantce->CR3 = 0;
}

void MyUart::dma_tx_pause(void)
{
    CLEAR_BIT(uart_instance->serial_instantce->CR3, USART_CR3_DMAT);
}

void MyUart::dma_rx_pause(void)
{
    CLEAR_BIT(uart_instance->serial_instantce->CR1, USART_CR1_PEIE);
    CLEAR_BIT(uart_instance->serial_instantce->CR3, USART_CR3_EIE);

    /* Disable the UART DMA Rx request */
    CLEAR_BIT(uart_instance->serial_instantce->CR3, USART_CR3_DMAR);
}

void MyUart::end_tx(void)
{
    CLEAR_BIT(uart_instance->serial_instantce->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));
}

void MyUart::end_rx(void)
{
    CLEAR_BIT(uart_instance->serial_instantce->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
    CLEAR_BIT(uart_instance->serial_instantce->CR3, USART_CR3_EIE);
}

void MyUart::dma_tx_resume(void)
{
    SET_BIT(uart_instance->serial_instantce->CR3, USART_CR3_DMAT);
}

void MyUart::dma_rx_resume(void)
{
    /* Reenable PE and ERR (Frame error, noise error, overrun error) interrupts */
    SET_BIT(uart_instance->serial_instantce->CR1, USART_CR1_PEIE);
    SET_BIT(uart_instance->serial_instantce->CR3, USART_CR3_EIE);

    /* Enable the UART DMA Rx request */
    SET_BIT(uart_instance->serial_instantce->CR3, USART_CR3_DMAR);
}

void MyUart:: uart_setconfig(void)
{
    uint32_t tmpreg = 0x00U;

    /*-------------------------- USART CR2 Configuration -----------------------*/
    tmpreg = uart_instance->serial_instantce->CR2;

    /* Clear STOP[13:12] bits */
    tmpreg &= (uint32_t)~((uint32_t)USART_CR2_STOP);

    /* Configure the UART Stop Bits: Set STOP[13:12] bits according to huart->Init.StopBits value */
    tmpreg |= (uint32_t)uart_instance->serial_init->StopBits;

    /* Write to USART CR2 */
    WRITE_REG(uart_instance->serial_instantce->CR2, (uint32_t)tmpreg);

    /*-------------------------- USART CR1 Configuration -----------------------*/
    tmpreg = uart_instance->serial_instantce->CR1;

    /* Clear M, PCE, PS, TE and RE bits */
    tmpreg &= (uint32_t)~((uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | \
                                   USART_CR1_RE | USART_CR1_OVER8));

    /* Configure the UART Word Length, Parity and mode:
     Set the M bits according to huart->Init.WordLength value
     Set PCE and PS bits according to huart->Init.Parity value
     Set TE and RE bits according to huart->Init.Mode value
     Set OVER8 bit according to huart->Init.OverSampling value */
    tmpreg |= (uint32_t)uart_instance->serial_init->WordLength | uart_instance->serial_init->Parity | uart_instance->serial_init->Mode | uart_instance->serial_init->OverSampling;

    /* Write to USART CR1 */
    WRITE_REG(uart_instance->serial_instantce->CR1, (uint32_t)tmpreg);

    /*-------------------------- USART CR3 Configuration -----------------------*/
    tmpreg = uart_instance->serial_instantce->CR3;

    /* Clear CTSE and RTSE bits */
    tmpreg &= (uint32_t)~((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE));

    /* Configure the UART HFC: Set CTSE and RTSE bits according to huart->Init.HwFlowCtl value */
    tmpreg |= uart_instance->serial_init->HwFlowCtl;

    /* Write to USART CR3 */
    WRITE_REG(uart_instance->serial_instantce->CR3, (uint32_t)tmpreg);

    /* Check the Over Sampling */
    if(uart_instance->serial_init->OverSampling == UART_OVERSAMPLING_8)
    {
    /*-------------------------- USART BRR Configuration ---------------------*/
    if((uart_instance->serial_instantce == USART1) || (uart_instance->serial_instantce == USART6))
    {
        uart_instance->serial_instantce->BRR = UART_BRR_SAMPLING8(HAL_RCC_GetPCLK2Freq(), uart_instance->serial_init->BaudRate);
    }
    else
    {
        uart_instance->serial_instantce->BRR = UART_BRR_SAMPLING8(HAL_RCC_GetPCLK1Freq(), uart_instance->serial_init->BaudRate);
    }
    }
    else
    {
    /*-------------------------- USART BRR Configuration ---------------------*/
    if((uart_instance->serial_instantce == USART1) || (uart_instance->serial_instantce == USART6))
    {
        uart_instance->serial_instantce->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), uart_instance->serial_init->BaudRate);
    }
    else
    {
        uart_instance->serial_instantce->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), uart_instance->serial_init->BaudRate);
    }
    }
}

void MyUart::isr_cb(void)
{
    if(uart_instance->serial_instantce->SR & USART_SR_TC)
    {
        CLEAR_BIT(uart_instance->serial_instantce->SR, USART_SR_TC);
        return;
    }
}
