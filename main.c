/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "keypad.h"
#include "dac.h"
#include "waves.h"

#define DELAY 250
#define ARRINTERRUPT 580 // 478 ARR with a list size of 796 is perfect; 380 ARR with a 1000 size list; (USED THIS AND WORKED) 253 ARR with a 1500 size list

//volatile uint16_t samples = 660;
volatile uint16_t waveIndex = 0;
volatile uint8_t freq = 1;
volatile uint8_t dutyCycle = 5;
volatile uint8_t waveType = 9; // 6 - sine; 7 - triangle; 8 - sawtooth; 9 - square


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void delay(volatile uint32_t count) {
    while (count--) {
        __NOP();  // No operation, just wait
    }
}

void Config_PA8_GPIOclock(void) {
	// Enable MCO, select MSI (4 MHz source)
	RCC->CFGR = ((RCC->CFGR & ~(RCC_CFGR_MCOSEL)) | (RCC_CFGR_MCOSEL_0));

	// Configure MCO output on PA8
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);
	GPIOA->MODER   &= ~(GPIO_MODER_MODE8);		// alternate function mode
	GPIOA->MODER   |=  (2 << GPIO_MODER_MODE8_Pos);
	GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT8);		// Push-pull output
	GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD8);		// no resistor
	GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED8);		// high speed
	GPIOA->AFR[1]  &= ~(GPIO_AFRH_AFSEL8);		// select MCO function

}


// this function will change the value of waveIndex according to the waveType and frequency
uint16_t operate_waveforms(void) {
	switch (waveType) {
	case 6:
		return sine[waveIndex];
		break;
	case 7:
		return triangle[waveIndex];
		break;
	case 8:
		return sawtooth[waveIndex];
		break;
	case 9:
		// return square wave
		if (waveIndex < dutyCycle*0.1*670) {
			return 15994;
		} else {
			return 0;
		}
		break;
	default:
		break;
	}
	return 0;
}


void TIM2_IRQHandler(void) { // Interrupt Service Routine
	if (TIM2->SR & TIM_SR_UIF) {
		GPIOC->ODR |= (GPIO_ODR_OD10); // turn on GPIO bit
		TIM2->SR &= ~(TIM_SR_UIF); // clear interrupt flag
		//DAC_Write(DAC_volt_conv(sineWave[waveIndex])); // 2.12V to the DAC
		DAC_Write(operate_waveforms());
		// this should only be done if were are displaying a sine, sawtooth, or triangle
		if (waveType == 6 || waveType == 7 || waveType == 8) {
			if (waveIndex < 654) { // COULD BE REMOVED MAYBE
				if ((waveIndex + freq) < 654) {
					waveIndex += freq;
				} else {
					waveIndex = 0;
				}
			} else {
				waveIndex = 0;
			}
		}
		if (waveType == 9) {
			if ((waveIndex + freq) < 655) {
				waveIndex += freq;
			} else {
				waveIndex = 0;
			}
		}
		GPIOC->ODR &= ~(GPIO_ODR_OD10); // turn off GPIO bit
	} else {
	}
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN; // enable GPIO B and C
  Config_LEDs();
  Config_Keypad_Rows();
  Config_Keypad_Columns();
  DAC_init(); // initializes the SPI along with its pins

  Config_PA8_GPIOclock(); // config PA8 to display 36MHz clock

  uint8_t key = 0xFF;

  // testing for TIM2 ISR with DAC-Write()
  // Configure PC10 as output
  GPIOC->MODER &= ~(GPIO_MODER_MODE10);
  GPIOC->MODER |= (1 << GPIO_MODER_MODE10_Pos);
  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT10);
  GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEED10);
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD10);
  GPIOC->ODR &= ~(GPIO_ODR_OD10);
  GPIOC->ODR |= GPIO_ODR_OD10;

  RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN); // Enable clock for TIM2
  TIM2->ARR = ARRINTERRUPT; //  36MHz clock, count for 18000 cycles
  //TIM2->CCR1 =  ONTIME5KHZ; // use CCR for 25% duty cycle (turning LED off on signal)
  TIM2->DIER |= TIM_DIER_UIE; // enable update event interrupt in TIM2 (look at datasheet)
  //TIM2->DIER |= TIM_DIER_CC1IE; // enable compare match interrupt for CCR1
  TIM2->SR &= ~(TIM_SR_UIF); // clearing the interrupt status register for update event
  //TIM2->SR &= ~(TIM_SR_CC1IF); // clearing status register for CCR

  // start timer
  TIM2->CR1 |= TIM_CR1_CEN;
  // enable tim2 interrupt in NVIC
  NVIC->ISER[0] = (1 << TIM2_IRQn);

  __enable_irq();



  while(1) {
	  key = 0xFF;
	  key = scan_keypad();
	  if (key == 0) {
		  dutyCycle = 5;
	  } else if (key <= 5) {
		  freq = key;
	  } else if (key <= 9) {
		  waveType = key;
	  } else if (key == 10) {
		  dutyCycle -= 1;
		  //delay(DELAY);
	  } else if (key == 11) {
		  dutyCycle += 1;
		  //delay(DELAY);
	  } else {}



	  /*
	  if (key != 0xFF) {
		  switch(key) {
		  case 1:
			  freq = key;
			  break;
		  case 2:
			  freq = key;
			  break;
		  case 3:
			  freq = key;
		  	  break;
		  case 4:
			  freq = key;
		  	  break;
		  case 5:
			  freq = key;
		 	  break;
		  case 6:
			  waveType = key;
		  	  break;
		  case 7:
			  waveType = key;
		  	  break;
		  case 8:
			  waveType = key;
		  	  break;
		  case 9:
			  waveType = key;
		  	  break;
		  case 10:
			  if (dutyCycle > 1) {
				  dutyCycle -= 1;
			  }
		  	  break;
		  case 11:
			  if (dutyCycle < 10) {
				  dutyCycle += 1;
			  }
			  break;
		  default:
			  break;

		  }
	  } else {}
	  */
	  //delay(DELAY);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 19;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
