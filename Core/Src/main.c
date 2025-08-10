#include "main.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal_pwr_ex.h"
#include "stdio.h"
#include "core_cm4.h"

void SystemClock_Config(void);
void TurretMotors_Config(void);
void USART2_Config(void);
void TurretFire_Config(void);
void NVIC_IRQn_Set(IRQn_Type IRQ);
void USART2_IRQHandler(void); //must be called this because written in interrupt vector table in .asm Startup.startup_stm21l476rgtx.s


void TurretUp(void);
void TurretDown(void);
void TurretLeft(void);
void TurretRight(void);
void TurretFire(void);
void TurretUpDownDoNothing(void);
void TurretLeftRightDoNothing(void);
void TurretFireDoNothing(void);

volatile uint32_t tmpreg;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  TurretMotors_Config();
  TurretFire_Config();
  USART2_Config();
}

void USART2_IRQHandler(void){ //this is a hardware interrupt, so will trigger by hardware even if function not in main.
	if((USART2->ISR & USART_ISR_RXNE) != 0){ //register will be 1 if there is data in RDR register. Will be 0 if there is nothing.
		volatile uint16_t RX_Value = USART2->RDR; // Reading RDR automatically clears the RXNE flag. Volatile because variable stores interrupt data.

		if(RX_Value == 119){ //ASCII 'w' is 119
			TurretUp();
		}
		else if(RX_Value == 115){ //ASCII 's' is 115
			TurretDown();
		}
		else if(RX_Value == 97){ //ASCII 'a' is 97
			TurretLeft();
		}
		else if(RX_Value == 100){ //ASCII 'd' is 100
			TurretRight();
		}
		else if(RX_Value == 32){ //ASCII ' ' is 32
			TurretFire();
		}
		else if(RX_Value == 121){ //ASCII 'y' is 121
			TurretUpDownDoNothing();
		}
		else if(RX_Value == 120){ //ASCII 'x' is 120
			TurretLeftRightDoNothing();
		}
		else if(RX_Value == 122){ //ASCII 'z' is 120
			TurretFireDoNothing();
		}
		USART2->TDR = RX_Value;
	}
}

void TurretUp(void){
	GPIOA->BSRR = GPIO_BSRR_BR6; //direction pin 6
	TIM3->CR1 |= TIM_CR1_CEN; //pwm TIM3 enable
}
void TurretDown(void){
	GPIOA->BSRR = GPIO_BSRR_BS6;
	TIM3->CR1 |= TIM_CR1_CEN;
}
void TurretLeft(void){
	GPIOA->BSRR = GPIO_BSRR_BR9; //direction pin 9. Value is 1, so we are resetting to 0, so i guess not outputting pulse is left
	TIM1->CR1 |= TIM_CR1_CEN; //pwm TIM1 enables
}
void TurretRight(void){
	GPIOA->BSRR = GPIO_BSRR_BS9; //we're setting 1, which means right
	TIM1->CR1 |= TIM_CR1_CEN;
}
void TurretUpDownDoNothing(void){
	TIM3->CR1 &= ~TIM_CR1_CEN;
}
void TurretLeftRightDoNothing(void){
	TIM1->CR1 &= ~TIM_CR1_CEN;
}
void TurretFire(void){
	GPIOA->BSRR = GPIO_BSRR_BS5;
}
void TurretFireDoNothing(void){
	GPIOA->BSRR = GPIO_BSRR_BR5;
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

	// Whether CFGR stays or not doesnt affect the register but I could swear the motor is smoother if this remains.
	RCC->CFGR &= ~RCC_CFGR_PPRE2;
	RCC->CFGR |= (0 << RCC_CFGR_PPRE2_Pos); //APB high-speed prescaler (APB2) make sure PCLK2 is not divided, so HCLK not divided 0x00, default already 0 for this bit position

	//Base motor direction (push/pull) GPIO Pin 9, STM32 pin D8
	GPIOA->MODER &= ~GPIO_MODER_MODE9_Msk; //remember each pin is 2 bits wide, so when BIC must use 0x03
	GPIOA->MODER |= GPIO_MODER_MODE9_0; //set PA9 to output (01)

	//GPIOA->OTYPER &= ~GPIO_OTYPER_OT9; //Output push-pull (reset state) (default value)
	//GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD9_Msk; // (reset state) set to 0x00, neither pull up nor pull down

	//Base motor direction GPIO Pin 6
	GPIOA->MODER &= ~GPIO_MODER_MODE6_Msk;
	GPIOA->MODER |= GPIO_MODER_MODE6_0; //set to output, 01


	//Base motor left/right (PWM) GPIO Pin 8, TIM1_CH1, STM32 pin D7
	GPIOA->MODER &= ~GPIO_MODER_MODE8_Msk;
	GPIOA->MODER |= GPIO_MODER_MODE8_1; //set PA8 to alternative function

	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8_Msk;
	GPIOA->AFR[1] |= GPIO_AFRH_AFSEL8_0; //set PA8 to AF1 (alternate function 1), which is TIM1_CH1. Also AF[0] are pins 0:7 and AF[1] are pins 8:15

	//GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_Msk; //11 very high speed, default is 00, you probably can't tell the difference anyways
	//GPIOA->OTYPER &= ~GPIO_OTYPER_OT8_Msk; //0x00 for output push/pull
	//GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD8_Msk; //0x00 neither pull up nor pull down

	TIM1->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
	TIM1->CCMR1 |= (6<<TIM_CCMR1_OC1M_Pos); //CCMR1 has configurations for both Ch1 and Ch2  // Set PWM mode 1 on CH1 (mode 1 is in upcounting, CH1 is active as long as TIM CNT < TIM CCR1. Configures 0110 for Ch1
	//TIM1->CCMR1 |= TIM_CCMR1_OC1PE;            // Enable preload register. (. TIMx_CCR1 preload value is loaded in the active register at each update event)

	//TIM1->PSC = 0; //so we keep clock at 4mhz (default for APB1) and not divide it by anything. (x) x (0+1)
	//left right motor
	TIM1->ARR = 65535; //Max value is 16 bit width 65535. This is not Hz, its the auto-reloader's counter. So higher this is, the longer it takes to do 1 PWM cycle peak to peak.
	TIM1->CCR1 = 8192; //keep in mind CCR1 is channel 1, so CCR2 would be channel 2
	TIM1->CCER |= TIM_CCER_CC1E;  // Enable CH1 output (Capture mode enable)
	TIM1->BDTR |= TIM_BDTR_MOE;   // Main output enable (For advanced timers like TIM1/TIM8)

	//Base motor up/down (PWM) GPIO Pin 7, TIM3_CH2
	GPIOA->MODER &= ~GPIO_MODER_MODE7_Msk;
	GPIOA->MODER |= GPIO_MODER_MODE7_1; //Alternate function for pin 12

	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL7_Msk;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL7_1; //Pin 7, AF[0] is 0010 for TIM3_CH2. Also AF[0] are pins 0:7 and AF[1] are pins 8:15

	RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM3EN_Msk;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN; //enable Tim3
	tmpreg = RCC->APB1ENR1;
	UNUSED(tmpreg); //standard practice to delay after starting timer to give it time to start

	//up down motor
	TIM3->CCMR1 |= (6<<TIM_CCMR1_OC2M_Pos); //CCMR1 has configurations for both Ch1 and Ch2, this configures 0110 for Ch2, mode 1 which is upcounting TIM CNT < TIM CCR1
	TIM3->ARR = 65535; //Auto reload value (marks the rising edge in PWM) Max value is 16 bit width, 65535
	TIM3->CCR2 = 100; //Set duty rate for Ch2
	TIM3->CCER |= TIM_CCER_CC2E; //enable ch2 output
	//TIM3 is a general timer, not advanced like TIM1 or TIM 8 so no need for BDTR and MOE

}

void USART2_Config(void) {
	//Port A already opened in TurretMotors_Config
	RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART2EN_Msk;
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; //start APB1 timer (didnt need AHB1 to open APB1)
	tmpreg = RCC->APB1ENR1;
	UNUSED(tmpreg);

	GPIOA->MODER &= ~GPIO_MODER_MODE2_Msk; //keep in mind the reset value is 11, so need to BIC the mask first
	GPIOA->MODER |= GPIO_MODER_MODE2_1; //PA_2 (TX) 10 Alternate function
	GPIOA->MODER &= ~GPIO_MODER_MODE3_Msk;
	GPIOA->MODER |= GPIO_MODER_MODE3_1; //PA_3 (RX) 10 Alternate function

	GPIOA->AFR[0] |= (0x7UL << 8U); //Alternate function 0111 (USART2) for PA_2
	GPIOA->AFR[0] |= (0x7UL << 12U); //Alternate function 0111 (USART2) for PA_3

	//USART2->CR1 |= USART_CR1_PCE //parity control (1)enable/(0)disable
	//USART2->CR1 |= USART_CR1_PS; //parity 0 even, 1 odd. This field only written when USART disabled
	//USART2->CR1 &= ~USART_CR1_M1; //I want M[1:0] to be 00: 1 Start bit, 8 data bits, n stop bits (reset value)
	//USART2->CR1 &= ~USART_CR1_M0; (reset value)
	//USART2->CR2 &= ~USART_CR2_STOP; //set n stop bits to 1 stop bit (also, keep in mind 0 for all of these are default) im just setting these here for learning reasons (reset value)
	USART2->CR1 |= USART_CR1_RE; //receiver enable
	USART2->CR1 |= USART_CR1_TE; //transmitter enable (only need if you are tx back to something, which I might do eventually)
	USART2->CR1 |= USART_CR1_RXNEIE; //allow RXNE interrupts in USART2
	USART2->CR1 |= USART_CR1_UE; //enable usart2. This is last because other USART2 stuff needs to configure first
	USART2->BRR = 4000000 / 9600; //4mHz/9600 baud. Gives 417hz/1 baud. UART frame is 1 bit every 417 APB1 clock cycles
								   //we set M[1:0] as 00 so 1 start bit, 8 data bits, and 1 end bit. 10 x 417 is 4170 clock cycles per word
								   //if 4mHz that mean each uart word should take about 1ms and each bit is 1x10^-4 s. Which is sort of slow?
	NVIC_IRQn_Set(USART2_IRQn); //set up NVIC to allow USART2 interrupts. Now the hardware takes over so whenever RDR has data, the hardware will trigger the interrupt handler
}

void NVIC_IRQn_Set(IRQn_Type IRQ){ //doesnt work with negative IRQs (which are error interrupts) so don't use negative ones
	NVIC->ISER[IRQ>>5UL] = 1<<(IRQ-(32*(IRQ>>5UL))); //ISER[1] = 1<6UL, there are 8 accessible elements and each element has 32 bits.
}

void TurretFire_Config(void){
	GPIOA->MODER &= ~GPIO_MODER_MODE5_Msk; //output mode 01. if the reset state is 11, you cannot just do |= 01, you must bic 11 first.
	GPIOA->MODER |= GPIO_MODER_MODE5_0; //output mode 01
}


void I2C1_Config(void){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; //All of the I2C1 pins are PB instead of PA, so we need to open PortB now. Also, do enable the port first before starting I2C
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN; //Enable I2C1 clock in APB1
	tmpreg = RCC->APB1ENR1; //remember standard practice to give the clock some time to start
	UNUSED(tmpreg);

	GPIOB->MODER &= ~((GPIO_MODER_MODE8_1) | (GPIO_MODER_MODE9_1)); //PB8 needs to be SCL so alternative function 10 //PB9 needs to be SDA so alternative function 10
	GPIOB->MODER |= (GPIO_MODER_MODE8_1) | (GPIO_MODER_MODE9_1); //PB8/PB9 needs to be SCL so alternative function 10

	GPIOB->AFR[1] &= ~((0x4UL << 0) | (0x4UL << 4U)); //remember [0] is for pins 0-7 and [1] is for pins 8-15
	GPIOB->AFR[1] |= (0x4UL << 0) | (0x4UL << 4U);  //when we look at the other datasheet, we find PortB PB8/9 I2C1_SCL and I2C1_SDA is in AF4, so 0100												// so in the AFR[1] register we push 0100 to bits 0-3 and 4-8 in AFRH.

	GPIOB->OTYPER |= GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9; //PB8/PB9 output type open-drain // Needed for I2C because open-drain allows multiple devices to share SDA/SCL safely.

	GPIOB->PUPDR &= ~((GPIO_PUPDR_PUPD8_0) | ~(GPIO_PUPDR_PUPD9_0)); //PB8/PB9 pull-up register. We set output type to be open-drain, which means register will always go from 1 -> 0, so we need it constantly at pull-up.
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD8_0;

	GPIOB->OSPEEDR &= ~((GPIO_OSPEEDR_OSPEED8) | (GPIO_OSPEEDR_OSPEED9));
	GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED8) | (GPIO_OSPEEDR_OSPEED9);

	I2C1->CR1 &= ~I2C_CR1_PE;
	RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST; //im not sure why its recommended to reset, but I guess if we REALLY want to make sure, we reset
	RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C1RST; //so we don't want to keep the register value as 1, or else it will constantly reset, we set it back to 0 to stop resetting.

	// Keep analog filter ON (default), digital filter DNF=0 (default)
	I2C1->CR1 &= ~I2C_CR1_ANFOFF; // 0 = analog filter enabled
	I2C1->CR1 &= ~I2C_CR1_DNF;  // DNF = 0

	I2C1->CR1 |= I2C_CR1_PE; //enable the I2C peripheral
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
