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

/// @brief Simple enum to describe LED color.
typedef enum TRIPP_PIN_COLOR {
	TRIPP_RED = GPIO_PIN_6,
	TRIPP_BLUE = GPIO_PIN_7,
	TRIPP_ORANGE = GPIO_PIN_8,
	TRIPP_GREEN = GPIO_PIN_9
} TRIPP_PIN_COLOR;	


// Simple helper functions to toggle, turn on, and turn off leds.
void enable_gpio(GPIO_TypeDef* gpio);
void enable_led(TRIPP_PIN_COLOR color);
void turn_on_led(TRIPP_PIN_COLOR color);
void turn_off_led(TRIPP_PIN_COLOR color);
void toggle_led(TRIPP_PIN_COLOR color);

/// @brief Error indication function.
void my_simple_error_indicator(void);

// FFT calculation.
int fix_fft(short fr[],	short ir[], short m, short inverse);

/*Integer square root - Obtained from Stack Overflow (14/6/15):
 * http://stackoverflow.com/questions/1100090/looking-for-an-efficient-integer-square-root-algorithm-for-arm-thumb2
 * User: Gutskalk
 */
uint16_t isqrt(uint32_t x);

int16_t calculateCurrentPassFrequency(int16_t indexOfBucket);
int16_t getIndexOfBucketWithLargestMagnitude(void);
int16_t getRollingFrequency(int16_t currentFreq);
void tuneLower(void);
void tuneHigher(void);
void setAsTuned(void);


#define ADC_FREQ (65536) //Hz
#define ADC_BUFFER_SIZE (1024)
#define ADC_BUFFER_SIZE_LOG2 (10)
static int16_t adc_value[ADC_BUFFER_SIZE];
static int16_t imaginary[ADC_BUFFER_SIZE];

int main(void)
{
 
  HAL_Init();
  SystemClock_Config();
	
	// Enable Leds.
	enable_gpio(GPIOC);
	enable_led(TRIPP_ORANGE);
	enable_led(TRIPP_BLUE);
	enable_led(TRIPP_GREEN);
	enable_led(TRIPP_RED);
	
	// Enable and configure ADC to channel 10 for PC0
	RCC->APB2ENR = RCC_APB2ENR_ADC1EN;
	
	// Set conversion on clock signal.
	ADC1->CFGR1 |= (2 << ADC_CFGR1_EXTSEL_Pos) | (1 << ADC_CFGR1_EXTEN_Pos) | ADC_CFGR1_OVRMOD;
	// Set sample rate.
	ADC1->SMPR |= 0x07;
	// Set channel.
	ADC1->CHSELR |= ADC_CHSELR_CHSEL10;
	
	// Enable TIM2 to trigger ADC conversion at 65536 Hz
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 
	// Set clock freq to 65,536 Hz
	TIM2->ARR = 1; 
	TIM2->PSC = 121;
	TIM2->CR2 |= TIM_CR2_MMS_1;
	// Start the timer
	TIM2->CR1 |=  TIM_CR1_CEN;

	
	// Calibrate and start ADC
	ADC1->CR |= ADC_CR_ADCAL;
	
	while((ADC1->CR & ADC_CR_ADCAL) > 0) {}
		
	// Enable ADC
	ADC1->CR |= ADC_CR_ADEN;
	
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0) {}; // Wait for ready flag.
	ADC1->CR |= ADC_CR_ADSTART; // Start
	
	
	int index = 0;	
	volatile int16_t freq = 0;
	while(1) {
		
		while((ADC1->ISR & (1 << 2)) == 0);
		
		adc_value[index] = (ADC1->DR) - (3450); // 3450 is the midpoint of the microphone.
		imaginary[index] = 0;
		++index;
		
		if(index == ADC_BUFFER_SIZE) {
			index = 0;

			// Preform fixed-floating point fft.
			fix_fft(adc_value, imaginary, ADC_BUFFER_SIZE_LOG2, 0);
			
			// Find index of bucket with largest magnitude.
			int16_t i_max = getIndexOfBucketWithLargestMagnitude();
			if(i_max == 0) {
				continue;
			}
			
			// Compute frequency of bucket with largest magnitude.
			int16_t tempFreq = calculateCurrentPassFrequency(i_max);
			
			freq = getRollingFrequency(calculateCurrentPassFrequency(i_max));
			
			switch(freq) {	
				// Tune to E
				case 50:
				case 51:
				case 52:
				case 53:
				case 54:
				case 55:
				case 56:
				case 57:
				case 58:
				case 59:
				case 60:
				case 61:
				case 62:
				case 63:
				case 64:
				case 65:
				case 66:
				case 67:
				case 68:
				case 69:
				case 70:
				case 71:
				case 72:
				case 73:
				case 74:
				case 75:
				case 76:
				case 77:
				case 78:
				case 79:
					tuneHigher();
					break;
				
				// Low E
				case 80:
				case 81:
				case 82:
				case 83:
				case 84:
					setAsTuned();
					break;
				
				// Tune to E
				case 85:
				case 86:
				case 87:
				case 88:
				case 89:
				case 90:
				case 91:
				case 92:
				case 93:
				case 94:
				case 95:
					tuneLower();
					break;
				
				// Tune to A
				case 96:
				case 97:
				case 98:
				case 99:
				case 100:
				case 101:
				case 102:
				case 103:
				case 104:
				case 105:
				case 106:
					tuneHigher();
					break;
				
					// A
				case 107:
				case 108:
				case 109:
				case 110:
				case 111:
				case 112:
				case 113:
					setAsTuned();
					break;
			
				// Tune to A
				case 114:
				case 115:
				case 116:
				case 117:
				case 118:
				case 119:
				case 120:
				case 121:
				case 122:
				case 123:
				case 124:
				case 125:
					tuneLower();
					break;
				
				// Tune to D
				case 126:
				case 127:
				case 128:
				case 129:
				case 130:
				case 131:
				case 132:
				case 133:
				case 134:
				case 135:
				case 136:
				case 137:
				case 138:
				case 139:
				case 140:
				case 141:
				case 142:
					tuneHigher();
					break;
				
					// D
				case 143:
				case 144:
				case 145:
				case 146:
				case 147:
				case 148:
				case 149:
				case 150:
				case 151:
				case 152:
				case 153:
					setAsTuned();
					break;
				
				// Tune to D
				case 154:
				case 155:
				case 156:
				case 157:
				case 158:
				case 159:
				case 160:
				case 161:
				case 162:
				case 163:
				case 164:
				case 165:
				case 166:
				case 167:
				case 168:
				case 169:
				case 170:
					tuneLower();
					break;
				
				// Tune to G
				case 171:
				case 172:
				case 173:
				case 174:
				case 175:
				case 176:
				case 177:
				case 178:
				case 179:
				case 180:
				case 181:
				case 182:
				case 183:
				case 184:
				case 185:
				case 186:
				case 187:
				case 188:
				case 189:
					tuneHigher();
					break;
				
				// G
				case 190:
				case 191:
				case 192:
				case 193:
				case 194:
				case 195:
				case 196:
				case 197:
				case 198:
				case 199:
				case 200:
				case 201:
				case 202:
				case 203:
				case 204:
				case 205:
				case 206:
					setAsTuned();
					break;				
				
				
				// Tune to G
				case 207:
				case 208:
				case 209:
				case 210:
				case 211:
				case 212:
				case 213:
				case 214:
				case 215:
				case 216:
				case 217:
				case 218:
				case 219:
				case 220:
				case 221:
				case 222:
					tuneLower();
					break;
				
				// Tune to B
				case 223:
				case 224:
				case 225:
				case 226:
				case 227:
				case 228:
				case 229:
				case 230:
				case 231:
				case 232:
				case 233:
				case 234:
				case 235:
				case 236:
				case 237:
				case 238:
				case 239:
					tuneHigher();
					break;
					
				// B
			  case 240:
				case 241:
				case 242:
				case 243:
				case 244:
				case 245:
				case 246:
				case 247:
				case 248:
				case 249:
				case 250:
				case 251:
				case 252:
				case 253:
				case 254:
				case 255:
					setAsTuned();
					break;
				
				// Tune to B
				case 256:
				case 257:
				case 258:
				case 259:
				case 260:
				case 261:
				case 262:
				case 263:
				case 264:
				case 265:
				case 266:
				case 267:
				case 268:
				case 269:
				case 270:
				case 271:
				case 272:
				case 273:
				case 274:
				case 275:
				case 276:
				case 277:
				case 278:
				case 279:
				case 280:
				case 281:
				case 282:
				case 283:
					tuneLower();
					break;
					
				// Tune to High E
				case 284:
				case 285:
				case 286:
				case 287:
				case 288:
				case 289:
				case 290:
				case 291:
				case 292:
				case 293:
				case 294:
				case 295:
				case 296:
				case 297:
				case 298:
				case 299:
				case 300:
				case 301:
				case 302:
				case 303:
				case 304:
				case 305:
				case 306:
				case 307:
				case 308:
				case 309:
				case 310:
				case 311:
				case 312:
				case 313:
				case 314:
				case 315:
				case 316:
				case 317:
				case 318:
				case 319:
					tuneHigher();
					break;
				
				case 320:
				case 321:
				case 322:
				case 323:
				case 324:
				case 325:
				case 326:
				case 327:
				case 328:
				case 329:
				case 330:
				case 331:
				case 332:
				case 333:
				case 334:
				case 335:
				case 336:
				case 337:
				case 338:
				case 339:
				case 340:
					setAsTuned();
					break;
				
				case 341:
				case 342:
				case 343:
				case 344:
				case 345:
				case 346:
				case 347:
				case 348:
				case 349:
				case 350:
				case 351:
				case 352:
				case 353:
				case 354:
				case 355:
				case 356:
				case 357:
				case 358:
				case 359:
				case 360:
				case 361:
				case 362:
				case 363:
				case 364:
				case 365:
				case 366:
				case 367:
				case 368:
				case 369:
				case 370:
				case 371:
				case 372:
				case 373:
				case 374:
				case 375:
				case 376:
				case 377:
				case 378:
				case 379:
				case 380:
				case 381:
				case 382:
				case 383:
				case 384:
				case 385:
				case 386:
				case 387:
				case 388:
				case 389:
				case 390:
				case 391:
				case 392:
				case 393:
				case 394:
				case 395:
				case 396:
				case 397:
				case 398:
				case 399:
				case 400:
					tuneLower();
					break;
				
				default:
					break;
			}
		
			
			// Read a value to clear the ADC conversion that may have occurred during processing of frequency.
			while((ADC1->ISR & (1 << 2)) == 0);
			ADC1->DR;
		}
	}	
}

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

void my_simple_error_indicator(void) {
	enable_gpio(GPIOC);
	enable_led(TRIPP_RED);
	turn_on_led(TRIPP_RED);
	while(1) ;
}

uint16_t isqrt(uint32_t x) {
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

int16_t calculateCurrentPassFrequency(int16_t indexOfBucket) {
	// 64 is the frequency divided by the number of items. But when dividing with the macros, results in 0.
	return (indexOfBucket * 64) / 2;
}

volatile uint16_t maxMag = 0;
int16_t getIndexOfBucketWithLargestMagnitude(void) {
	// Find bucket with largest magnitude.
	int16_t i_max = 0;
	maxMag = 0;
	for(int16_t index = 2; index < 14; ++index) { // Only need to check buckets 2 to 14 since that is within the valid frequency range.
	
		uint16_t mag = isqrt((int32_t)adc_value[index] * (int32_t)adc_value[index] + (int32_t)imaginary[index] * (int32_t)imaginary[index]);
		
		if(mag > 30) {
			maxMag = mag;
			return index;
		}
		
		if(mag > maxMag) {
			i_max = index;
			maxMag = mag;
		}
	}
	
	if(maxMag > 30) {
		return i_max;
	}
	else {
		return 0;
	}
}


#define ROLLING_BUFFER_SIZE 8
int16_t getRollingFrequency(int16_t currentFreq) {
	static int16_t averageBuffer[ROLLING_BUFFER_SIZE];
	static int currentIndex = 0;
	
	if(currentFreq > 50 && currentFreq < 500) {
		averageBuffer[currentIndex++] = currentFreq;
	}
	else {
		averageBuffer[currentIndex++] = 0;
	}
	if(currentIndex > ROLLING_BUFFER_SIZE - 1) currentIndex = 0;

	int average = 0;
	for(int i = 0; i < ROLLING_BUFFER_SIZE; ++i) {
		average += averageBuffer[i];
	}
	
	return average / ROLLING_BUFFER_SIZE;
}

void tuneHigher(void) {
	turn_on_led(TRIPP_RED);
	turn_off_led(TRIPP_BLUE);
	turn_off_led(TRIPP_GREEN);
	turn_off_led(TRIPP_ORANGE);
}

void tuneLower(void) {
	turn_off_led(TRIPP_RED);
	turn_on_led(TRIPP_BLUE);
	turn_off_led(TRIPP_GREEN);
	turn_off_led(TRIPP_ORANGE);
}

void setAsTuned(void) {
	turn_off_led(TRIPP_RED);
	turn_off_led(TRIPP_BLUE);
	turn_off_led(TRIPP_GREEN);
	turn_off_led(TRIPP_ORANGE);
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
