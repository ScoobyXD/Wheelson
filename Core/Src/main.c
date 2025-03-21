#include "main.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal_pwr_ex.h"

void SystemClock_Config(void);
void TurretMotors_Config(void);


int main(void)
{

  HAL_Init();
  SystemClock_Config();
  TurretMotors_Config();
  while (1)
  {
	  //HAL_Delay(1000);
	  GPIOA->BSRR |= (1 << 9U);
	  TIM1->CCR1 = 250;
	  /*
	  HAL_Delay(1000);
	  TIM1->CCR1 = 0;
	  HAL_Delay(1000);
	  GPIOA->BSRR &= ~(1 << 9U);
	  TIM1->CCR1 = 250;
	  HAL_Delay(1000);
	  TIM1->CCR1 = 0;*/
  }
  /* USER CODE END 3 */
}
void TurretMotors_Config(void){
	__IO uint32_t tmpreg;

	RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; //activate clock for port A
	tmpreg = RCC->AHB2ENR;
	UNUSED(tmpreg); //standard practice to delay after starting timer to give it time to start


	RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; //enable Tim1
	tmpreg = RCC->APB2ENR;
	UNUSED(tmpreg); //standard practice to delay after starting timer to give it time to start

	RCC->CFGR &= ~RCC_CFGR_PPRE2;
	RCC->CFGR |= (0x06UL << RCC_CFGR_PPRE2_Pos); //ensure APB2 is running at 80 MHz, APB2 Prescale = 1 (TIM1 = 80MHz) HCLK divided by 8

	//Base motor direction (push/pull)
	GPIOA->MODER &= ~GPIO_MODER_MODE9_0;
	GPIOA->MODER |= GPIO_MODER_MODE9_0; //set PA9 to output (01)

	GPIOA->OTYPER &= ~GPIO_OTYPER_OT9;
	GPIOA->OTYPER |= GPIO_OTYPER_OT9; //set output type to push/pull, which is the default one

	//Base motor power (PWM)
	GPIOA->MODER &= ~GPIO_MODER_MODE8_1;
	GPIOA->MODER |= GPIO_MODER_MODE8_1; //set PA8 to alternative function

	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8_0;
	GPIOA->AFR[1] |= GPIO_AFRH_AFSEL8_0; //set PA8 to AF1 (alternate function 1), which is TIM1_CH1

	GPIO->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_Msk; //11 very high speed
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD8_Msk; //11 neither pull up nor pull down

	TIM1->PSC = 0; //prescalar, 0 so we keep clock at 80mhz
	TIM1->ARR = 1;
	TIM1->CCR1 = 1;

	TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);  // Set PWM mode 1 on CH1
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE;            // Enable preload register

	TIM1->CCER |= TIM_CCER_CC1E;  // Enable CH1 output
	TIM1->BDTR |= TIM_BDTR_MOE;   // Main output enable (For advanced timers like TIM1)
	TIM1->CR1 |= TIM_CR1_CEN;     // Enable TIM1


}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
