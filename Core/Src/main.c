/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
void AwaitTC(void);
void SetupI2C2_Read(void);
void SetupI2C2_Write(void);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


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
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

	RCC->AHBENR |= (1 << 18) | (1 << 19); // Enable RCC for port B and C
	
	/* LED CONFIG */
	GPIOC->MODER |= (1 << 18) | (1 << 16) | (1 << 14) | (1 << 12); // Pins 6-9 enabled
	GPIOC->OTYPER &= ~((1 << 9) | (1 << 8) | (1 << 7) | (1 << 6));
	GPIOC->OSPEEDR &= ~((1 << 18) | (1 << 16) | (1 << 14) | (1 << 12)); // Low speed
	GPIOC->PUPDR &= ~((0xFF << 12));
	/* END LED CONFIG*/
	
	/* PB11 CONFIG */
	GPIOB->MODER |= (1 << 23);
	GPIOB->MODER &= ~(1 << 22); // to AF mode
	GPIOB->OTYPER |= (1 << 11); // output open-drain
	GPIOB->AFR[1] |= (1 << 12);
	GPIOB->AFR[1] &= ~((1 << 15) | (1 << 14) | (1 << 13)); // AF1 set (0001)
	/* END PB11 CONFIG */
	
	/* PB13 CONFIG */
	GPIOB->MODER |= (1 << 27); 
	GPIOB->MODER &= ~((1 << 26));// to AF mode
	GPIOB->OTYPER |= (1 << 13); // output open-drain
	GPIOB->AFR[1] |= (1 << 22) | (1 << 20); 
	GPIOB->AFR[1] &= ~((1 << 23) | (1 << 21)); // AF5 set (0101)
	/* END PB13 CONFIG */
	
	/* PB14 CONFIG */
	GPIOB->MODER &= ~((1 << 29));
	GPIOB->MODER |= (1 << 28); // to general purpose output mode
	GPIOB->OTYPER &= ~((1 << 14)); // output push-pull
	GPIOB->ODR |= (1 << 14); // set high
	/* END PB14 CONFIG */
	
	/* PB15 CONFIG */
	GPIOB->MODER &= ~((1 << 31) | (1 << 30)); // Set to Input mode
	/* END PB15 CONFIG */
	
	/* PC0 CONFIG */
	GPIOC->MODER |= (1 << 0);
	GPIOC->MODER &= ~((1 << 1)); // General purpose output mode
	GPIOC->OTYPER &= ~((1 << 0)); // output push-pull
	GPIOC->ODR |= (1 << 0); // set high
	/* END PC0 CONFIG */
	
	/* I2C2 CONFIG */
	RCC->APB1ENR |= (1 << 22); // I2C2 RCC Enabled
	
	I2C2->TIMINGR &= ~((1 << 31) | (1 << 30) | (1 << 29));
	I2C2->TIMINGR |= (1 << 28); // PRESC to 1
	
	I2C2->TIMINGR &= ~(0xFF);
	I2C2->TIMINGR |= 0x13; // SCLL to 0x13
	
	I2C2->TIMINGR &= ~(0xFF << 8);
	I2C2->TIMINGR |= (0xF << 8); // SCLH to 0xF (shifted 8 bits)
	
	I2C2->TIMINGR &= ~((0xF << 16));
	I2C2->TIMINGR |= (0x2 << 16); // SDADEL to 0x2 (shifted 16 bits)
	
	I2C2->TIMINGR &= ~((0xF << 20));
	I2C2->TIMINGR |= (0x4 << 20); // SCLDEL to 0x4 (shifted 20 bits)
	
	I2C2->CR1 |= (1 << 0); // I2C2 ENABLED, CONFIG LOCKED
	/* END I2C2 CONFIG */
	

	/* I2C2 Comms Config */
	SetupI2C2_Write();
	/* END I2C2 Comms Config */
	
	unsigned int nackfMask = (1 << 4);
	unsigned int txisMask = (1 << 1);
	
	
	/* MAIN RUNNING LOOP */
  while (1)
  {
		HAL_Delay(500);
		
		// Establish connection
		unsigned int txis = I2C2->ISR & txisMask;
		unsigned int nackf = I2C2->ISR & nackfMask;
		if (nackf == nackfMask) // Connection Error
			GPIOC->ODR |= (1 << 6); // Turn on RED LED, indicates connection error
		if (txis != txisMask) // Slave did not respond yet
		{ 
			GPIOC->ODR ^= (1 << 8); // blink Orange LED, indicates slave yet to respond
			continue;
		}
			GPIOC->ODR |= (1 << 7); // Turn on Blue LED, indicates connection
		
		// Successful connection established.
		// Write to slave
		I2C2->TXDR |= 0x0F; // WHO_AM_I addr
		// Await TC (transfer complete)
		AwaitTC();
		
		// Read from slave
		SetupI2C2_Read();
		nackf = I2C2->ISR & nackfMask;
		unsigned int rxneMask = (1 << 2);
		unsigned int rxne = I2C2->ISR & rxneMask;
		while (nackf == nackfMask){
			HAL_Delay(250);
			GPIOC->ODR ^= (1 << 6); // Connection error, blink Red
		}
		while (rxne != rxneMask){
			HAL_Delay(250);
			rxne = I2C2->ISR & rxneMask;
			GPIOC->ODR ^= (1 << 8); // Awaiting response, blink Orange
		}
		// Data received, await full transfer
		AwaitTC();
		
		// Check received data, if match to 0xD4, green LED on!
		unsigned int rawResult = I2C2->RXDR;
		unsigned int result = rawResult & ~(0xFFFFFF00);
		if (result == 0xD3 || rawResult == 0xD3)
			GPIOC->ODR |= (1 << 9); // Green On, successful!
		else
			GPIOC->ODR |= (1 << 6); // Red on, unsuccessful
		
		I2C2->CR2 |= (1 << 14); // Release I2C bus; STOP bit to 1
		while(1);
		
  }
}

/* 
 * Waits for the TC bit in I2C2_ISR to be a 1
 * Blinks Orange LED until done
 */
void AwaitTC(void)
{
	unsigned int tcMask = (1 << 6);
	while((I2C2->ISR & tcMask) != tcMask){
			HAL_Delay(50);
			GPIOC->ODR ^= (1 << 8); // Blink Orange while waiting
		}
		GPIOC->ODR &= ~(1 << 8);	// Orange off
}

/* 
 * Sets up CR2 parameters, with RD_WRN set to read
 */
void SetupI2C2_Read(void)
{
	I2C2->CR2 &= ~(0x3FF);
	I2C2->CR2 |= (0x69 << 1); // SADD: Slave address set
	
	I2C2->CR2 &= ~(0xFF << 16);
	I2C2->CR2 |= (1 << 16); // NBYTES: 1 byte to send
	
	I2C2->CR2 |= (1 << 10); // RD_WRN: 1 -> Master requests read transfer
	
	I2C2->CR2 |= (1 << 13); // START: ENABLED
}

/*
 * Sets up CR2 parameters, with RD_WRN set to write
 */
void SetupI2C2_Write(void)
{
	I2C2->CR2 &= ~(0x3FF);
	I2C2->CR2 |= (0x69 << 1); // SADD: Slave address set
	
	I2C2->CR2 &= ~(0xFF << 16);
	I2C2->CR2 |= (1 << 16); // NBYTES: 1 byte to send
	
	I2C2->CR2 &= ~((1 << 10)); // RD_WRN: 0 -> Master requests write transfer
	
	I2C2->CR2 |= (1 << 13); // START: ENABLED
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
