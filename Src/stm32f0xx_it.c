/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f0xx_it.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

extern int sistemCalisiyor;
extern int tekSeferCalistir;
extern int adc;
extern int grup1;
extern int grup2;
extern int toplam;
extern int onSaniyeSayaci;
extern uint8_t limitAsimi[];
extern uint8_t urunYok[];
extern int toplamUart;
extern int urunAsimiUyarisi;
extern int butonaBasildi;
extern int getSize(char* string);




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */ 
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
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
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
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */
	
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
	
		if(!tekSeferCalistir){
		
			if(!sistemCalisiyor){
			
				sistemCalisiyor = 1;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
				
			}
			
			tekSeferCalistir = 1;
			
		}else{
		
			if(sistemCalisiyor){
			
				tekSeferCalistir = 0;
				sistemCalisiyor = 0;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
				
			}
		
		}
	
	}

  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */
	
	if(sistemCalisiyor){
		
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)){
			
			butonaBasildi = 1;
	
			if(adc <= 32){
		
				grup1++;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				
			}else{
				
				grup2++;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			
			}
			toplam ++;
			toplamUart++;
		}
	}
	

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles ADC global interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */
	
	

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc);
  /* USER CODE BEGIN ADC1_IRQn 1 */
	
	

  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break, update, trigger and commutation interrupts.
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 0 */
	if(sistemCalisiyor){
		
		onSaniyeSayaci++;
		
		if(onSaniyeSayaci == 2){
		
			if(!butonaBasildi){
			
				HAL_UART_Transmit_IT(&huart1, (uint8_t*) urunYok, 15);
			
			}else{
			
				butonaBasildi = 0;
				
			}
		
			onSaniyeSayaci = 0;
				
		}else{
		
			if(urunAsimiUyarisi){
			
				HAL_UART_Transmit_IT(&huart1, (uint8_t*) limitAsimi, 16);
				
			}else{
			
				char grp1String[3];
				char grp2String[3];
				char toplamString[3];
				
				sprintf(grp1String,"%d",grup1);
				sprintf(grp2String,"%d",grup2);
				sprintf(toplamString,"%d",toplamUart);
				
				char gonderilenVeri[] = "<Grp1:xxx,Grp2:yyy,Toplam:zzz>\r\n";
				
				int mesajin_length = getSize(gonderilenVeri);
				int grup1_length = getSize(grp1String);
				int grup2_length =getSize(grp2String);
				int toplam_length = getSize(toplamString);
				
				int tmp_grup1_index = 0;
				int tmp_grup2_index = 0;
				int tmp_toplam_index = 0;
				
				for(int i = 0; i < mesajin_length;i++){
					if(gonderilenVeri[i] == 'x'){
							if(tmp_grup1_index < grup1_length){
									gonderilenVeri[i] = grp1String[tmp_grup1_index];
							}else{
									gonderilenVeri[i] = ' ';
							}
							tmp_grup1_index++;
					}else if(gonderilenVeri[i] == 'y'){
							if(tmp_grup2_index < grup2_length){
									gonderilenVeri[i] = grp2String[tmp_grup2_index];
							}else{
									gonderilenVeri[i] = ' ';
							}
							tmp_grup2_index++;
					}else if(gonderilenVeri[i] == 'z'){
							if(tmp_toplam_index < toplam_length){
									gonderilenVeri[i] = toplamString[tmp_toplam_index];
							}else{
									gonderilenVeri[i] = ' ';
							}
							tmp_toplam_index++;
					}
				
				}
				
				HAL_UART_Transmit_IT(&huart1, (uint8_t*) gonderilenVeri, mesajin_length);
				
			}
		
		}
	
	}
	
  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */

  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	
	if(sistemCalisiyor){
		adc = HAL_ADC_GetValue(&hadc);
	}
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	if(sistemCalisiyor){
		HAL_ADC_Start_IT(&hadc);
	}
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	
	

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
