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
#include <stm32f0xx_hal_adc.h>
#include <stm32f0xx_hal_adc_ex.h>

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



int fix_fft(short fr[],	short ir[], short m, short inverse);

/*Integer square root - Obtained from Stack Overflow (14/6/15):
 * http://stackoverflow.com/questions/1100090/looking-for-an-efficient-integer-square-root-algorithm-for-arm-thumb2
 * User: Gutskalk
 */
uint16_t isqrt(uint32_t x)
{
	uint16_t res=0;
	uint16_t add= 0x8000;
	int i;
	for(i=0;i<16;i++)
	{
		uint16_t temp=res | add;
		uint32_t g2=temp*temp;
		if (x>=g2)
		{
			res=temp;
		}
		add>>=1;
	}
	return res;
}

void MyError() {
	enable_gpio(GPIOC);
	enable_led(TRIPP_RED);
	turn_on_led(TRIPP_RED);

}


#define ADC_FREQ (65,536) //Hz
#define ADC_BUFFER_SIZE (1024)
#define ADC_BUFFER_SIZE_LOG2 (10)
int16_t adc_value[ADC_BUFFER_SIZE];
int16_t imaginary[ADC_BUFFER_SIZE] = {0};
volatile int16_t current;
int main(void)
{
 
  HAL_Init();
  SystemClock_Config();
	
	enable_gpio(GPIOC);
	enable_led(TRIPP_ORANGE);
	enable_led(TRIPP_BLUE);
	
	RCC->APB2ENR = RCC_APB2ENR_ADC1EN;
	
	ADC_HandleTypeDef adcHandle = {0};
	adcHandle.Instance = ADC1;
	
	adcHandle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	adcHandle.Init.Resolution = ADC_RESOLUTION_12B;
	adcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	adcHandle.Init.ScanConvMode = 0;
	adcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	adcHandle.Init.LowPowerAutoWait = DISABLE;
	adcHandle.Init.LowPowerAutoPowerOff = DISABLE;
	adcHandle.Init.ContinuousConvMode = DISABLE;
	adcHandle.Init.DiscontinuousConvMode = DISABLE;
	adcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	adcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	adcHandle.Init.DMAContinuousRequests = DISABLE;
	adcHandle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	adcHandle.Init.SamplingTimeCommon = ADC_SAMPLETIME_239CYCLES_5;
	
	adcHandle.DMA_Handle = NULL;
	
	adcHandle.Lock = HAL_UNLOCKED;
	
	adcHandle.State = 0;
	
	HAL_StatusTypeDef status = HAL_ADC_Init(&adcHandle);
	
	if(status != HAL_OK) {
		MyError();
	}
	

	
	ADC_ChannelConfTypeDef adcChannel;
	adcChannel.Channel = ADC_CHANNEL_10;
	
	status = HAL_ADC_ConfigChannel(&adcHandle, &adcChannel);
	
	if(status != HAL_OK) {
		MyError();
	}
	

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 and TIM3 
	// Set clock freq to 50000 Hz
	TIM2->ARR = 1; 
	TIM2->PSC = 121;
	 	// Enable clock interupt
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CR2 |= TIM_CR2_MMS_1;

	
	// Start the timer
	TIM2->CR1 |=  TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM2_IRQn);
	
	
	status = HAL_ADCEx_Calibration_Start(&adcHandle);
	if(status != HAL_OK) {
		MyError();
	}
	
	status = HAL_ADC_Start(&adcHandle);
	if(status != HAL_OK) {
		MyError();
	}
	
	
	HAL_Delay(500);
	int32_t resting_avg = 0;
	for(int i = 0; i < ADC_BUFFER_SIZE; ++i) {
		while((ADC1->ISR & (1 << 2)) == 0);
		resting_avg += HAL_ADC_GetValue(&adcHandle);	
	}
	
	resting_avg /= ADC_BUFFER_SIZE;
	


	turn_on_led(TRIPP_ORANGE);
	enable_led(TRIPP_GREEN);
	int index = 0;	
	volatile int16_t freq = 0;
	while(1) {
		
		while((ADC1->ISR & (1 << 2)) == 0);
		current = (HAL_ADC_GetValue(&adcHandle)) - (3450);
		
		
		adc_value[index] = current;
		imaginary[index] = 0;
		++index;
		if(index == ADC_BUFFER_SIZE) {
			
			//for(index = 0; index < ADC_BUFFER_SIZE; ++index){
			//	
			//	if(index < (ADC_BUFFER_SIZE / 2)) {
			//		adc_value[index] = ((int32_t)adc_value[index]*index) >> ADC_BUFFER_SIZE_LOG2;
			//	}
			//	else {
			//		adc_value[index] = ((int32_t)adc_value[index]*((ADC_BUFFER_SIZE / 2) - index)) >> ADC_BUFFER_SIZE_LOG2;
			//	}
			//}
			
			
			fix_fft(adc_value, imaginary, ADC_BUFFER_SIZE_LOG2, 0);
			
			int16_t i_max = 0;
			uint16_t maxMag = 0;
			for(index = 1; index < ADC_BUFFER_SIZE; ++index) {
			
				uint16_t mag = isqrt((int32_t)adc_value[index] * (int32_t)adc_value[index] + (int32_t)imaginary[index] * (int32_t)imaginary[index]);
				
				if(mag > maxMag) {
					i_max = index;
					maxMag = mag;
				}
			}
			
			int16_t tempFreq = (i_max * 64) / 2;
			
			if(tempFreq > 0 && tempFreq < 1500) {
				freq = tempFreq;
			}
			
			if(freq == 320) {
				turn_on_led(TRIPP_GREEN);
			}
			else{
				turn_off_led(TRIPP_GREEN);
			}
			
			index = 0;
			
			while((ADC1->ISR & (1 << 2)) == 0);
			current = (HAL_ADC_GetValue(&adcHandle)<< 4) - resting_avg;
		}
	}	
}

void TIM2_IRQHandler(void) {

	// Clear pending
	TIM2->SR &= ~TIM_SR_UIF;
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
