/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	// LED Setup 
	// set peripheral clock using RCC to the syscfg peripheral 
	// RCC->APB2ENR	|= (1<<0); // Enable peripheral clock uses RCC (shifting to the left 19 times adding zeros)
	// RCC->AHBENR |= (1<<19); // Enable peripheral clock uses RCC (shifting to the left 19 times adding zeros)
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 	
	RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN);
	
	// moder 01 for general purpose
	GPIOC->MODER &=~ ((1<<19)|(1<<17)); // clear
	GPIOC->MODER &=~ ((1<<14)|(1<<12)); // Alternate function mode 
	GPIOC->MODER |=  ((1<<18)|(1<<16)); // enable 	
	GPIOC->MODER |=  ((1<<15)|(1<<13)); // alternate function mode 
	// Otyper 0 for push pull 
	GPIOC->OTYPER &=~((1<<12)|(1<<13)|(1<<14)|(1<<15)); // clear	
	// Ospeeder x0 for low speed 
	GPIOC->OSPEEDR &=~(1<<12)|(1<<13)|(1<<14)|(1<<15)|(1<<16)|(1<<17)|(1<<18)|(1<<19); // clear
	// Pupdr 00 for no pull up/down
	GPIOC->PUPDR &=~(1<<12)|(1<<13)|(1<<14)|(1<<15)|(1<<16)|(1<<17)|(1<<18)|(1<<19); // clear
		// use alternate function TIMx_CHy
	// TIM3->CH1 set GPIOC-> afrl to 0000, 24-31
	// TIM3->CH2 
	GPIOC->AFR[0] &=~ ((1<<24)|(1<<25)|(1<<26)|(1<<27)|(1<<28)|(1<<29)|(1<<30)|(1<<31)); // should dim 6 and 7
	
  // intialize Timer 2 at 4hz using RCC
	RCC->APB1ENR |= (1<<0); // enable 
	TIM2->PSC = 0x1F3F; // set psc to 1 1111 0011 1111 = 1 f 3 f to divide by 7999 and to have 1 tick be 1ms
	TIM2->ARR = 0x00FA; // setting the ARR to 250 to count to 250ms
	TIM2->DIER |= (1<<0); // enable the update 
	TIM2->CNT |= ((1<<4)|(0<<5)|(1<<6)); // enable the clock at 001 for bits 4-6
	// set up the timer interupt handler 
	NVIC_EnableIRQ(TIM2_IRQn);
	GPIOC->ODR |= (1<<9);
	
	TIM2 -> CR1 |=1;
	
	// set timer 3 to 800hz or 1ms
	//TIM3->PSC = 0x03E7; // set to 999 to divide by 1000 
	TIM3->ARR = 9999; // 10 ms
	// set the capture compare registers CCRx for both channels to 20% of your ARR value
	// current ARR value is 10 or 0x000A, set to 2 or 0x0002
	TIM3->CCR1 = 50;
	TIM3->CCR2 = 50;
	// set capture control registers to output (clear: 0,1,8,9)
	// set OC1M to PWM2 (enable 4,5,6)
	// set OC2M to PWM1 (enable 13,14 disable 12)
	TIM3->CCMR1 &=~ ((1<<0)|(1<<1));  // clear
	TIM3->CCMR1 &=~ ((1<<8)|(1<<9));
	TIM3->CCMR1 |= ((1<<4)|(1<<5)|(1<<6));
	TIM3->CCMR1 |=  ((1<<13)|(1<<14)); // enable
	TIM3->CCMR1 &=~ (1<<12);
	
	// TIM3->CCMR2 &=~ ((1<<0)|(1<<1)|(1<<8)|(1<<9)|(1<<12));  // clear
	// TIM3->CCMR2 |=  ((1<<4)|(1<<5)|(1<<6)|(1<<13)|(1<<14)); // enable
	// set the output enable bits for channels 1 and 2 in the CCER register 
	// enable bits 0 and 4
	TIM3->CCER |= (1<<0);
	TIM3->CCER |= (1<<4);

	TIM3->CR1 |= (1<<0);

	
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

	// TIM2 handler 
void TIM2_IRQHandler(void){
	GPIOC->ODR ^= (1<<8)|(1<<9); // led color change
	TIM2->SR &= 0; // clearing the register at 0
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
