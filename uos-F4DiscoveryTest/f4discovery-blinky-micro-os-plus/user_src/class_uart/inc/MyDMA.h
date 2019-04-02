/*
 * MyDMA.h
 *
 *  Created on: Mar 31, 2019
 *      Author: quzhi
 */

#ifndef CLASS_UART_SRC_MYDMA_H_
#define CLASS_UART_SRC_MYDMA_H_

#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal.h"

namespace STM32F407 {
    class MyDMA {
        private:
            DMA_Stream_TypeDef   *dma_handle;
            DMA_InitTypeDef *dma_init_data;
            uint32_t StreamIndex;
            uint32_t StreamBaseAddress;
            void dma_set_config(uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
            void dma_calc_base_and_bitshift(void);
        public:
            MyDMA(DMA_Stream_TypeDef *dma_instance, DMA_InitTypeDef *dma_init);
            void dma_start_it(uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
            void dma_start(uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
            void abort(void);
            void isr_cb(void);
            void clear_intr(void);
            virtual ~MyDMA();
    };
}

#endif /* CLASS_UART_SRC_MYDMA_H_ */
