#include "main.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal_pwr_ex.h"
#include "stdio.h"

void SystemClock_Config(void);
void TurretMotors_Config(void);
void USART1_Config(void);
void TurretRight(void);
void TurretLeft(void);
void TurretDoNothing(void);
__IO uint32_t tmpreg;

int main(void)
{

  HAL_Init();
  SystemClock_Config();
  TurretMotors_Config();

  char UserCommand;

  while (1) //as of now doing roughly 400 pulses every second
  {
	  //poll for user inputs
	  //logic to check commands
	  if(UserCommand == 'A'){
		  TurretLeft();
	  }
	  else if(UserCommand == 'D'){
		  TurretRight();
	  }
	  else {
		  TurretDoNothing();
	  }

	  TurretLeft();
	  TurretRight();
	  TurretLeft();
	  TurretRight();
  }
}
void TurretRight(void){
	GPIOA->BSRR = GPIO_BSRR_BS9;
	TIM1->CCR1 = 4096;
	TIM1->CR1 |= TIM_CR1_CEN;

}

void TurretLeft(void){
	GPIOA->BSRR = GPIO_BSRR_BR9;
	TIM1->CCR1 = 4096;
	TIM1->CR1 |= TIM_CR1_CEN;
}
void TurretDoNothing(void){
	TIM1->CCR1 = 0;
}

void TurretMotors_Config(void){


	RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; //activate clock for port A
	tmpreg = RCC->AHB2ENR;
	UNUSED(tmpreg); //standard practice to delay after starting timer to give it time to start

	RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; //enable Tim1
	tmpreg = RCC->APB2ENR;
	UNUSED(tmpreg); //standard practice to delay after starting timer to give it time to start

	RCC->CFGR &= ~RCC_CFGR_PPRE2;
	RCC->CFGR |= (0 << RCC_CFGR_PPRE2_Pos); // make sure PCLK2 is not divided, so HCLK not divided 0x00

	//Base motor direction (push/pull) GPIO Pin 9, STM32 pin D8
	GPIOA->MODER &= ~GPIO_MODER_MODE9_Msk; //remember each pin is 2 bits wide, so when BIC must use 0x03
	GPIOA->MODER |= GPIO_MODER_MODE9_0; //set PA9 to output (01)

	GPIOA->OTYPER &= ~GPIO_OTYPER_OT9; //Output push-pull (reset state) (default value)
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD9_Msk; //set to 0x00, neither pull up nor pull down

	//Base motor power (PWM) GPIO Pin 8, STM32 pin D7
	GPIOA->MODER &= ~GPIO_MODER_MODE8_Msk;
	GPIOA->MODER |= GPIO_MODER_MODE8_1; //set PA8 to alternative function

	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8_Msk;
	GPIOA->AFR[1] |= GPIO_AFRH_AFSEL8_0; //set PA8 to AF1 (alternate function 1), which is TIM1_CH1

	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_Msk; //11 very high speed
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT8_Msk; //0x00 for output push/pull
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD8_Msk; //0x00 neither pull up nor pull down

	TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
	TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);  // Set PWM mode 1 on CH1 (mode 1 is in upcounting, CH1 is active as long as TIM CNT < TIM CCR1
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE;            // Enable preload register. (. TIMx_CCR1 preload value is loaded in the active register at each update event)

	TIM1->PSC = 0; //so we keep clock at 80mhz and not divide it by anything. (x) x (0+1)
	TIM1->ARR = 8192; //Max value is 16 bit width 65535

	TIM1->CCER |= TIM_CCER_CC1E;  // Enable CH1 output (Capture mode enable)
	TIM1->BDTR |= TIM_BDTR_MOE;   // Main output enable (For advanced timers like TIM1/TIM8)
}

void USART1_Config(void) {
	//Port A already opened in TurretMotors_Config
	RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;	//enable usart1 clock
	tmpreg = RCC->APB2ENR;
	UNUSED(tmpreg);

	GPIOA->
	USART1->BRR =
	USART1->CR1 =


    // 2. Set PA9 (TX) as alternate function push-pull
    GPIOA->CRH &= ~(0xF << 4);         // Clear CNF9 + MODE9
    GPIOA->CRH |=  (0xB << 4);         // MODE9 = 0b11 (50 MHz), CNF9 = 0b10 (AF PP)

    // 3. Set PA10 (RX) as input floating
    GPIOA->CRH &= ~(0xF << 8);         // Clear CNF10 + MODE10
    GPIOA->CRH |=  (0x4 << 8);         // CNF10 = 0b01 (floating input), MODE10 = 0b00

    // 4. Set baud rate
    USART1->BRR = 72000000 / 9600;     // Assuming 72 MHz PCLK2

    // 5. Enable USART1: TE (transmit), RE (receive), UE (USART enable)
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void uart1_send_char(char c) {
    while (!(USART1->SR & USART_SR_TXE));  // Wait until TX buffer is empty
    USART1->DR = c;
}

void uart1_send_str(const char *s) {
    while (*s) {
        uart1_send_char(*s++);
    }
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
