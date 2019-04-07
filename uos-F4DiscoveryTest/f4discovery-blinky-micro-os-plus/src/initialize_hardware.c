/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2016 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <cmsis-plus/rtos/os-hooks.h>
#include <cmsis_device.h>
#include "stm32f4xx_hal_pwr_ex.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_cortex.h"

// ----------------------------------------------------------------------------

// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// The code to set the clock is at the end.
//
// Note1: The default clock settings assume that the HSE_VALUE is a multiple
// of 1MHz, and try to reach the maximum speed available for the
// board. It does NOT guarantee that the required USB clock of 48MHz is
// available. If you need this, please update the settings of PLL_M, PLL_N,
// PLL_P, PLL_Q to match your needs.
//
// Note2: The external memory controllers are not enabled. If needed, you
// have to define DATA_IN_ExtSRAM or DATA_IN_ExtSDRAM and to configure
// the memory banks in system/src/cmsis/system_stm32f4xx.c to match your needs.

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

// This is the application hardware initialisation routine,
// redefined to add more inits.
//
// Called early from _start(), right after data & bss init, before
// constructors.
//
// After Reset the Cortex-M processor is in Thread mode,
// priority is Privileged, and the Stack is set to Main.
//
// Warning: The HAL requires the system timer, running at 1000 Hz
// and calling HAL_IncTick().
#define USE_CUBE_MX
void
os_startup_initialize_hardware(void)
{
#ifndef USE_CUBE_MX
    __IO uint32_t tmpreg = 0x00U;
    FLASH->ACR |= FLASH_ACR_ICEN;
    FLASH->ACR |= FLASH_ACR_DCEN;
    FLASH->ACR |= FLASH_ACR_PRFTEN;
    NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0U));
    SysTick_Config(SystemCoreClock/1000U);

    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	tmpreg = RCC->APB2ENR  & RCC_APB2ENR_SYSCFGEN;
	UNUSED(tmpreg);
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	tmpreg = RCC->APB1ENR & RCC_APB1ENR_PWREN;
	UNUSED(tmpreg);
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    tmpreg = RCC->AHB1ENR & RCC_AHB1ENR_GPIOHEN;
    UNUSED(tmpreg);
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    tmpreg = RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN;
    UNUSED(tmpreg);
    PWR->CR &=   (~PWR_CR_VOS) | (PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC->CR |= RCC_CR_HSEON;
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
    	*(__IO uint32_t *) RCC_CR_PLLON_BB = DISABLE;
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET)
	RCC->PLLCFGR =  (RCC_PLLCFGR_PLLSRC_HSE_Msk |
					 4U |
					 (168U << POSITION_VAL(RCC_PLLCFGR_PLLN)) |
					 (4U << POSITION_VAL(RCC_PLLCFGR_PLLQ)));
    *(__IO uint32_t *) RCC_CR_PLLON_BB = ENABLE;
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET);
    const uint8_t APBAHBPrescTable[16] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
    *(__IO uint8_t *)ACR_BYTE0_ADDRESS = (uint8_t)(FLASH_LATENCY_5);
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_HPRE)) | (RCC_SYSCLK_DIV1);
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW)) | RCC_SYSCLKSOURCE_PLLCLK;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_SYSCLKSOURCE_STATUS_PLLCLK);
	RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PPRE1)) | RCC_HCLK_DIV4;
	RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PPRE2)) | ((RCC_HCLK_DIV2) << 3U);
	SystemCoreClock = HAL_RCC_GetSysClockFreq() >> APBAHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE)>> POSITION_VAL(RCC_CFGR_HPRE)];
	SysTick_Config(SystemCoreClock/1000U);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0 ,0U);
#else
	cube_mx_init();
#endif
}

// ----------------------------------------------------------------------------
