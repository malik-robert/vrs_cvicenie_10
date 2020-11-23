/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t intensity = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern mode led_mode;
extern int8_t intensity_set_point;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */
	if(LL_DMA_IsActiveFlag_TC6(DMA1) == SET)
	{
		USART2_CheckDmaReception();
		LL_DMA_ClearFlag_TC6(DMA1);
	}
	else if(LL_DMA_IsActiveFlag_HT6(DMA1) == SET)
	{
		USART2_CheckDmaReception();
		LL_DMA_ClearFlag_HT6(DMA1);
	}
  /* USER CODE END DMA1_Channel6_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */
	if(LL_DMA_IsActiveFlag_TC7(DMA1) == SET)
	{
		LL_DMA_ClearFlag_TC7(DMA1);

		while(LL_USART_IsActiveFlag_TC(USART2) == RESET);
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_7);
	}
  /* USER CODE END DMA1_Channel7_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	static state led_state = DOWN;

	if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
		led_state = setLedState(led_mode, led_state, intensity, intensity_set_point);
		intensity = setIntensity(led_mode, led_state, intensity, intensity_set_point);
		setDutyCycle(intensity);
	}

  LL_TIM_ClearFlag_UPDATE(TIM2);
  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXT line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	if(LL_USART_IsActiveFlag_IDLE(USART2))
	{
		USART2_CheckDmaReception();
		LL_USART_ClearFlag_IDLE(USART2);
	}

  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/* Funkcia nastavuje striedu pre PWM časovača TIM2. */
void setDutyCycle(uint8_t D) {
	if (D >= PWM_MIN && D <= PWM_MAX) {
		TIM2->CCR1 = (TIM2->ARR*D)/99u;
	}
}

/* Funkcia nastavuje zhasínanie (DOWN)
 * alebo rozsvecovanie (UP) LED.
 * Stav LED sa nastaví podľa módu ovládania,
 * žiadanej hodnoty intenzity a ohraničenia
 * minimálnej/maximálnej intenzity svietenia
 *
 * Vstupy:
 * led_mode .............. mód ovládania {MAN, AUTO}
 * led_state ............. predošlý stav {DOWN, UP}
 * intensity ............. aktuálna intenzita svietenia
 * intensity_set_point ... žiadaná hodnota intenzity svietenia */
state setLedState(mode led_mode, state led_state, uint8_t intensity, int8_t intensity_set_point) {
	switch (led_mode) {
		case MAN:
			if (intensity > intensity_set_point) {
				led_state = DOWN;
			}
			else if (intensity < intensity_set_point) {
				led_state = UP;
			}
			break;

		case AUTO:
			if (intensity >= PWM_MAX) {
				led_state = DOWN;
			}
			else if (intensity <= PWM_MIN) {
				led_state = UP;
			}
			break;
	}

	return led_state;
}

/* Funkcia nastavuje intenzitu svietenia LED.
 * Intenzita sa nastaví podľa módu ovládania,
 * žiadanej hodnoty intenzity a ohraničenia
 * minimálnej/maximálnej intenzity svietenia.
 *
 * Vstupy:
 * led_mode .............. mód ovládania {MAN, AUTO}
 * led_state ............. predošlý stav {DOWN, UP}
 * intensity ............. aktuálna intenzita svietenia
 * intensity_set_point ... žiadaná hodnota intenzity svietenia */
uint8_t setIntensity(mode led_mode, state led_state, uint8_t intensity, int8_t intensity_set_point) {
	if (led_state) {
		if (intensity < PWM_MAX && (led_mode || (intensity_set_point >= 0 && intensity < intensity_set_point))) {
			intensity += PWM_INCREMENT;
		}
	}
	else {
		if (intensity > PWM_MIN && (led_mode || (intensity_set_point >= 0 && intensity > intensity_set_point))) {
			intensity -= PWM_INCREMENT;
		}
	}

	return intensity;
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
