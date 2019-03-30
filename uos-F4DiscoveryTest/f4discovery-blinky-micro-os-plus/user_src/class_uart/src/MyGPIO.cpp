/*
 * MyGPIO.cpp
 *
 *  Created on: Mar 31, 2019
 *      Author: quzhi
 */

#include "MyGPIO.h"

#define GPIO_MODE             ((uint32_t)0x00000003U)
#define EXTI_MODE             ((uint32_t)0x10000000U)
#define GPIO_MODE_IT          ((uint32_t)0x00010000U)
#define GPIO_MODE_EVT         ((uint32_t)0x00020000U)
#define RISING_EDGE           ((uint32_t)0x00100000U)
#define FALLING_EDGE          ((uint32_t)0x00200000U)
#define GPIO_OUTPUT_TYPE      ((uint32_t)0x00000010U)

#define GPIO_NUMBER           ((uint32_t)16U)

MyGPIO::MyGPIO(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init) {
    // TODO Auto-generated constructor stub
    GPIO_Port = GPIOx;
    GPIO_Pin = GPIO_Init->Pin;
    uint32_t position;
    uint32_t ioposition = 0x00U;
    uint32_t iocurrent = 0x00U;
    uint32_t temp = 0x00U;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_INSTANCE(GPIO_Port));
    assert_param(IS_GPIO_PIN(GPIO_Init->Pin));
    assert_param(IS_GPIO_MODE(GPIO_Init->Mode));
    assert_param(IS_GPIO_PULL(GPIO_Init->Pull));

    /* Configure the port pins */
    for(position = 0U; position < GPIO_NUMBER; position++)
    {
      /* Get the IO position */
      ioposition = ((uint32_t)0x01U) << position;
      /* Get the current IO position */
      iocurrent = (uint32_t)(GPIO_Init->Pin) & ioposition;

      if(iocurrent == ioposition)
      {
        /*--------------------- GPIO Mode Configuration ------------------------*/
        /* In case of Alternate function mode selection */
        if((GPIO_Init->Mode == GPIO_MODE_AF_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
        {
          /* Check the Alternate function parameter */
          assert_param(IS_GPIO_AF(GPIO_Init->Alternate));
          /* Configure Alternate function mapped with the current IO */
          temp = GPIO_Port->AFR[position >> 3U];
          temp &= ~((uint32_t)0xFU << ((uint32_t)(position & (uint32_t)0x07U) * 4U)) ;
          temp |= ((uint32_t)(GPIO_Init->Alternate) << (((uint32_t)position & (uint32_t)0x07U) * 4U));
          GPIO_Port->AFR[position >> 3U] = temp;
        }

        /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
        temp = GPIO_Port->MODER;
        temp &= ~(GPIO_MODER_MODER0 << (position * 2U));
        temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2U));
        GPIO_Port->MODER = temp;

        /* In case of Output or Alternate function mode selection */
        if((GPIO_Init->Mode == GPIO_MODE_OUTPUT_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_PP) ||
           (GPIO_Init->Mode == GPIO_MODE_OUTPUT_OD) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
        {
          /* Check the Speed parameter */
          assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
          /* Configure the IO Speed */
          temp = GPIO_Port->OSPEEDR;
          temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2U));
          temp |= (GPIO_Init->Speed << (position * 2U));
          GPIO_Port->OSPEEDR = temp;

          /* Configure the IO Output Type */
          temp = GPIO_Port->OTYPER;
          temp &= ~(GPIO_OTYPER_OT_0 << position) ;
          temp |= (((GPIO_Init->Mode & GPIO_OUTPUT_TYPE) >> 4U) << position);
          GPIO_Port->OTYPER = temp;
        }

        /* Activate the Pull-up or Pull down resistor for the current IO */
        temp = GPIO_Port->PUPDR;
        temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));
        temp |= ((GPIO_Init->Pull) << (position * 2U));
        GPIO_Port->PUPDR = temp;

        /*--------------------- EXTI Mode Configuration ------------------------*/
        /* Configure the External Interrupt or event for the current IO */
        if((GPIO_Init->Mode & EXTI_MODE) == EXTI_MODE)
        {
          /* Enable SYSCFG Clock */
          __HAL_RCC_SYSCFG_CLK_ENABLE();

          temp = SYSCFG->EXTICR[position >> 2U];
          temp &= ~(((uint32_t)0x0FU) << (4U * (position & 0x03U)));
          temp |= ((uint32_t)(GPIO_GET_INDEX(GPIO_Port)) << (4U * (position & 0x03U)));
          SYSCFG->EXTICR[position >> 2U] = temp;

          /* Clear EXTI line configuration */
          temp = EXTI->IMR;
          temp &= ~((uint32_t)iocurrent);
          if((GPIO_Init->Mode & GPIO_MODE_IT) == GPIO_MODE_IT)
          {
            temp |= iocurrent;
          }
          EXTI->IMR = temp;

          temp = EXTI->EMR;
          temp &= ~((uint32_t)iocurrent);
          if((GPIO_Init->Mode & GPIO_MODE_EVT) == GPIO_MODE_EVT)
          {
            temp |= iocurrent;
          }
          EXTI->EMR = temp;

          /* Clear Rising Falling edge configuration */
          temp = EXTI->RTSR;
          temp &= ~((uint32_t)iocurrent);
          if((GPIO_Init->Mode & RISING_EDGE) == RISING_EDGE)
          {
            temp |= iocurrent;
          }
          EXTI->RTSR = temp;

          temp = EXTI->FTSR;
          temp &= ~((uint32_t)iocurrent);
          if((GPIO_Init->Mode & FALLING_EDGE) == FALLING_EDGE)
          {
            temp |= iocurrent;
          }
          EXTI->FTSR = temp;
        }
      }
    }
}

MyGPIO::~MyGPIO() {
    // TODO Auto-generated destructor stub
    uint32_t position;
    uint32_t ioposition = 0x00U;
    uint32_t iocurrent = 0x00U;
    uint32_t tmp = 0x00U;

    /* Check the parameters */
    assert_param(IS_GPIO_ALL_INSTANCE(GPIO_Port));

    /* Configure the port pins */
    for(position = 0U; position < GPIO_NUMBER; position++)
    {
      /* Get the IO position */
      ioposition = ((uint32_t)0x01U) << position;
      /* Get the current IO position */
      iocurrent = (GPIO_Pin) & ioposition;

      if(iocurrent == ioposition)
      {
        /*------------------------- GPIO Mode Configuration --------------------*/
        /* Configure IO Direction in Input Floating Mode */
        GPIO_Port->MODER &= ~(GPIO_MODER_MODER0 << (position * 2U));

        /* Configure the default Alternate Function in current IO */
        GPIO_Port->AFR[position >> 3U] &= ~((uint32_t)0xFU << ((uint32_t)(position & (uint32_t)0x07U) * 4U)) ;

        /* Configure the default value for IO Speed */
        GPIO_Port->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2U));

        /* Configure the default value IO Output Type */
        GPIO_Port->OTYPER  &= ~(GPIO_OTYPER_OT_0 << position) ;

        /* Deactivate the Pull-up and Pull-down resistor for the current IO */
        GPIO_Port->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));

        /*------------------------- EXTI Mode Configuration --------------------*/
        tmp = SYSCFG->EXTICR[position >> 2U];
        tmp &= (((uint32_t)0x0FU) << (4U * (position & 0x03U)));
        if(tmp == ((uint32_t)(GPIO_GET_INDEX(GPIO_Port)) << (4U * (position & 0x03U))))
        {
          /* Configure the External Interrupt or event for the current IO */
          tmp = ((uint32_t)0x0FU) << (4U * (position & 0x03U));
          SYSCFG->EXTICR[position >> 2U] &= ~tmp;

          /* Clear EXTI line configuration */
          EXTI->IMR &= ~((uint32_t)iocurrent);
          EXTI->EMR &= ~((uint32_t)iocurrent);

          /* Clear Rising Falling edge configuration */
          EXTI->RTSR &= ~((uint32_t)iocurrent);
          EXTI->FTSR &= ~((uint32_t)iocurrent);
        }
      }
    }
}

