/*
 * MyDMA.cpp
 *
 *  Created on: Mar 31, 2019
 *      Author: quzhi
 */

#include "MyDMA.h"

typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

#define HAL_TIMEOUT_DMA_ABORT    ((uint32_t)5)  /* 5 ms */

MyDMA::MyDMA(DMA_Stream_TypeDef *dma_instance, DMA_InitTypeDef *dma_init) {
    dma_handle = dma_instance;
    dma_init_data = dma_init;
    dma_calc_base_and_bitshift();
    // TODO Auto-generated constructor stub
    uint32_t tmp = 0U;
    DMA_Base_Registers *regs;

    /* Disable the peripheral */
    dma_handle->CR &=  ~DMA_SxCR_EN;

    /* Get the CR register value */
    tmp = dma_handle->CR;

    /* Clear CHSEL, MBURST, PBURST, PL, MSIZE, PSIZE, MINC, PINC, CIRC, DIR, CT and DBM bits */
    tmp &= ((uint32_t)~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST | \
                        DMA_SxCR_PL    | DMA_SxCR_MSIZE  | DMA_SxCR_PSIZE  | \
                        DMA_SxCR_MINC  | DMA_SxCR_PINC   | DMA_SxCR_CIRC   | \
                        DMA_SxCR_DIR   | DMA_SxCR_CT     | DMA_SxCR_DBM));

    /* Prepare the DMA Stream configuration */
    tmp |=  dma_init_data->Channel             | dma_init_data->Direction        |
            dma_init_data->PeriphInc           | dma_init_data->MemInc           |
            dma_init_data->PeriphDataAlignment | dma_init_data->MemDataAlignment |
            dma_init_data->Mode                | dma_init_data->Priority;

    /* the Memory burst and peripheral burst are not used when the FIFO is disabled */
    if(dma_init_data->FIFOMode == DMA_FIFOMODE_ENABLE)
    {
      /* Get memory burst and peripheral burst */
      tmp |=  dma_init_data->MemBurst | dma_init_data->PeriphBurst;
    }

    /* Write to DMA Stream CR register */
    dma_handle->CR = tmp;

    /* Get the FCR register value */
    tmp = dma_handle->FCR;

    /* Clear Direct mode and FIFO threshold bits */
    tmp &= (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);

    /* Prepare the DMA Stream FIFO configuration */
    tmp |= dma_init_data->FIFOMode;

    /* Write to DMA Stream FCR */
    dma_handle->FCR = tmp;

    /* Initialize StreamBaseAddress and StreamIndex parameters to be used to calculate
       DMA steam Base Address needed by HAL_DMA_IRQHandler() and HAL_DMA_PollForTransfer() */
    regs = (DMA_Base_Registers *)StreamBaseAddress;

    /* Clear all interrupt flags */
    regs->IFCR = 0x3FU << StreamIndex;

}

MyDMA::~MyDMA() {
    // TODO Auto-generated destructor stub
    DMA_Base_Registers *regs;

    /* Disable the selected DMA Streamx */
    dma_handle->CR &=  ~DMA_SxCR_EN;

    /* Reset DMA Streamx control register */
    dma_handle->CR   = 0U;

    /* Reset DMA Streamx number of data to transfer register */
    dma_handle->NDTR = 0U;

    /* Reset DMA Streamx peripheral address register */
    dma_handle->PAR  = 0U;

    /* Reset DMA Streamx memory 0 address register */
    dma_handle->M0AR = 0U;

    /* Reset DMA Streamx memory 1 address register */
    dma_handle->M1AR = 0U;

    /* Reset DMA Streamx FIFO control register */
    dma_handle->FCR  = (uint32_t)0x00000021U;

    /* Get DMA steam Base Address */
    regs = (DMA_Base_Registers *)StreamBaseAddress;

    /* Clear all interrupt flags at correct offset within the register */
    regs->IFCR = 0x3FU << StreamIndex;
}


void MyDMA::dma_start_it(uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs = (DMA_Base_Registers *)StreamBaseAddress;

  /* Check the parameters */
  assert_param(IS_DMA_BUFFER_SIZE(DataLength));

    /* Configure the source, destination address and the data length */
    dma_set_config(SrcAddress, DstAddress, DataLength);

    /* Clear all interrupt flags at correct offset within the register */
    regs->IFCR = 0x3FU << StreamIndex;

    /* Enable Common interrupts*/
    dma_handle->CR  |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
    dma_handle->FCR |= DMA_IT_FE;

    /* Enable the Peripheral */
    dma_handle->CR |=  DMA_SxCR_EN;
}

void MyDMA::abort(void)
{
  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs = (DMA_Base_Registers *)StreamBaseAddress;

    /* Disable all the transfer interrupts */
    dma_handle->CR  &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
    dma_handle->FCR &= ~(DMA_IT_FE);

    /* Disable the stream */
    dma_handle->CR &=  ~DMA_SxCR_EN;

    /* Clear all interrupt flags at correct offset within the register */
    regs->IFCR = 0x3FU << StreamIndex;
}

void MyDMA::dma_start(uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
    abort();

    /* Configure the source, destination address and the data length */
    dma_set_config(SrcAddress, DstAddress, DataLength);

    /* Enable the Peripheral */
    dma_handle->CR |=  DMA_SxCR_EN;
}


void MyDMA::dma_set_config(uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  /* Clear DBM bit */
  dma_handle->CR &= (uint32_t)(~DMA_SxCR_DBM);

  /* Configure DMA Stream data length */
  dma_handle->NDTR = DataLength;

  /* Peripheral to Memory */
  if((dma_init_data->Direction) == DMA_MEMORY_TO_PERIPH)
  {
    /* Configure DMA Stream destination address */
    dma_handle->PAR = DstAddress;

    /* Configure DMA Stream source address */
    dma_handle->M0AR = SrcAddress;
  }
  /* Memory to Peripheral */
  else
  {
    /* Configure DMA Stream source address */
    dma_handle->PAR = SrcAddress;

    /* Configure DMA Stream destination address */
    dma_handle->M0AR = DstAddress;
  }
}

void MyDMA::dma_calc_base_and_bitshift(void)
{
    uint32_t stream_number = (((uint32_t)dma_handle & 0xFFU) - 16U) / 24U;

    /* lookup table for necessary bitshift of flags within status registers */
    static const uint8_t flagBitshiftOffset[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};
    StreamIndex = flagBitshiftOffset[stream_number];

    if (stream_number > 3U)
    {
        /* return pointer to HISR and HIFCR */
        StreamBaseAddress = (((uint32_t)dma_handle & (uint32_t)(~0x3FFU)) + 4U);
    }
    else
    {
        /* return pointer to LISR and LIFCR */
        StreamBaseAddress = ((uint32_t)dma_handle & (uint32_t)(~0x3FFU));
    }
}

void MyDMA::isr_cb(void)
{
    uint32_t tmpisr;
    DMA_Base_Registers *regs = (DMA_Base_Registers *)StreamBaseAddress;
    tmpisr = regs->ISR;
    if (tmpisr & (DMA_FLAG_TCIF0_4 << StreamIndex))
    {
        regs->IFCR = DMA_FLAG_TCIF0_4 << StreamIndex;
    }
}

