/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

void SystemClock_Config(void);


typedef enum TRIPP_PIN_COLOR {
	TRIPP_RED = GPIO_PIN_6,
	TRIPP_BLUE = GPIO_PIN_7,
	TRIPP_ORANGE = GPIO_PIN_8,
	TRIPP_GREEN = GPIO_PIN_9
} TRIPP_PIN_COLOR;	


void enable_gpio(GPIO_TypeDef* gpio) {
	switch((uintptr_t)gpio) {
		case GPIOA_BASE:
			RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
			break;
		case GPIOB_BASE:
			RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
			break;
		case GPIOC_BASE:
			RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
			break;
		case GPIOD_BASE:
			RCC->AHBENR |= RCC_AHBENR_GPIODEN;
			break;
		case GPIOE_BASE:
			RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
			break;
		case GPIOF_BASE:
			RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
			break;
	}
}

void enable_led(TRIPP_PIN_COLOR color){
	switch(color) {
		case TRIPP_RED:
			GPIOC->MODER |= (1 << 12); // Sets pin modes to output.
			GPIOC->OTYPER &= ~(1 << 6); // Sets pin output type to push-pull.
			GPIOC->OSPEEDR &= ~((1 << 13) | (1 << 12)); // Sets speed to low.
			GPIOC->PUPDR &= ~((1 << 13) | (1 << 12)); // Sets no pull-up, pull-down.
			break;
		case TRIPP_BLUE:
			GPIOC->MODER |= (1 << 14); // Sets pin modes to output.
			GPIOC->OTYPER &= ~(1 << 7); // Sets pin output type to push-pull.
			GPIOC->OSPEEDR &= ~((1 << 15) | (1 << 14)); // Sets speed to low.
			GPIOC->PUPDR &= ~((1 << 15) | (1 << 14)); // Sets no pull-up, pull-down.
			break;
		case TRIPP_ORANGE:
			GPIOC->MODER |= (1 << 16); // Sets pin modes to output.
			GPIOC->OTYPER &= ~(1 << 8); // Sets pin output type to push-pull.
			GPIOC->OSPEEDR &= ~((1 << 17) | (1 << 16)); // Sets speed to low.
			GPIOC->PUPDR &= ~((1 << 17) | (1 << 16)); // Sets no pull-up, pull-down.
			break;
		case TRIPP_GREEN:
			GPIOC->MODER |= (1 << 18); // Sets pin modes to output.
			GPIOC->OTYPER &= ~(1 << 9); // Sets pin output type to push-pull.
			GPIOC->OSPEEDR &= ~((1 << 19) | (1 << 18)); // Sets speed to low.
			GPIOC->PUPDR &= ~((1 << 19) | (1 << 18)); // Sets no pull-up, pull-down.
			break;
	}
}

void turn_on_led(TRIPP_PIN_COLOR color){
	switch(color) {
		case TRIPP_RED:
		case TRIPP_BLUE:
		case TRIPP_ORANGE:
		case TRIPP_GREEN:
			GPIOC->BSRR |= color; // Set pin high.
			break;
	}
}

void turn_off_led(TRIPP_PIN_COLOR color){
	switch(color) {
		case TRIPP_RED:
		case TRIPP_BLUE:
		case TRIPP_ORANGE:
		case TRIPP_GREEN:
			GPIOC->BRR |= color; // Set pin high.
			break;
	}
}

void toggle_led(TRIPP_PIN_COLOR color){
	switch(color) {
		case TRIPP_RED:
		case TRIPP_BLUE:
		case TRIPP_ORANGE:
		case TRIPP_GREEN:
			GPIOC->BSRR = (((GPIOC->ODR & color) << 16)| (GPIOC->ODR ^ color));
			break;
	}
}

/**
  * @brief  The application entry point.
  * @retval int
  */

volatile int16_t adc_value;
int main(void)
{
 
  HAL_Init();
  SystemClock_Config();
	
	enable_gpio(GPIOC);
	enable_led(TRIPP_BLUE);
	enable_led(TRIPP_GREEN);
	enable_led(TRIPP_RED);
	enable_led(TRIPP_ORANGE);
	
	// Set PC0 to analog
	GPIOC->MODER |= (3 << 0);
	GPIOC->PUPDR &= ~(3 << 0);
	

	// Enable ADC1
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	// continuous, hardware triggers disabled;
	ADC1->CFGR1 |= (1 << 13);
	
	// Set channel
	ADC1->CHSELR |= (1 << 10);
	
	ADC1->SMPR = 7;
	
	// Start calibration
	ADC1->CR |= (1 << 31);
	while((ADC1->CR & (1 << 31)) > 0) {}; // Calibration in progress
		
	// Enable ADC
	ADC1->CR |= (1 << 0);
	
	while((ADC1->ISR & (1 << 0)) > 0) {}; // Wait for ready flag.
	ADC1->CR |= (1 << 2); // Start
	
  while (1)
  {
		adc_value = ADC1->DR;
  }
 
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
