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

int Get_X(void);
int Get_Y(void);

void Close_I2C2_Connection(void);
void Check_I2C2_Read_Ready(void);
void Check_I2C2_Write_Ready(void);
void AwaitTC(void);
void SetupI2C2_Read(void);
void SetupI2C2_Read_LH(void);
void SetupI2C2_Write(void);
void SetupI2C2_Write_CONFIG(void);
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
	
	
	// Begin I2C connection
	
	/* CTRL_REG1 CONFIG */
	SetupI2C2_Write_CONFIG();
	Check_I2C2_Write_Ready();
	I2C2->TXDR = 0x20; // Transmit to CTRL_REG1

	Check_I2C2_Write_Ready();
	I2C2->TXDR = 0x0b; // PD to 1, Xen to 1, Yen to 1
	AwaitTC();
	Close_I2C2_Connection();
	/* END CTRL_REG1 CONFIG */

	//GPIOC->ODR |= (1 << 9); // Green LED ON to indicate successful config
	
	/* MAIN RUNNING LOOP */
  while (1)
  {
		HAL_Delay(100);
		
		volatile short x = Get_X();
		volatile short y = Get_Y();
		
		if (x > 300)
			GPIOC->ODR |= (1 << 8); // Orange
		else
			GPIOC->ODR &= ~(1 << 8);
		if (x < -300)
			GPIOC->ODR |= (1 << 9); // Green
		else
			GPIOC->ODR &= ~(1 << 9);
		
		if (y > 300)
			GPIOC->ODR |= (1 << 6); // Red
		else
			GPIOC->ODR &= ~(1 << 6);
		if (y < -300)
			GPIOC->ODR |= (1 << 7); // Blue
		else
			GPIOC->ODR &= ~(1 << 7);
  }
}

/*
 * Retrieves the X data from the Gyroscope's OUT_X_L and OUT_X_H registers.
 * Returns a signed short representing X axis movement.
 */
int Get_X()
{
	// X_Low
	SetupI2C2_Write();
	Check_I2C2_Write_Ready();
	I2C2->TXDR |= 0xA8; // Read OUT_X_L
	AwaitTC();
	
	SetupI2C2_Read_LH(); // Read 2 bytes at a time per transaction!
	Check_I2C2_Read_Ready();
	volatile short raw_xLow = I2C2->RXDR & 0xFF;
	
	// X_Hi
	Check_I2C2_Read_Ready();
	volatile short raw_xHi = I2C2->RXDR & 0xFF;
	AwaitTC();
	Close_I2C2_Connection();
	
	volatile short x = (raw_xLow << 8) | raw_xHi;
	return x;
}

/*
 * Retrieves the Y data from the Gyroscope's OUT_Y_L and OUT_Y_H registers.
 * Returns a signed short representing Y axis movement.
 */
int Get_Y()
{
	// Y_Low
	SetupI2C2_Write();
	Check_I2C2_Write_Ready();
	I2C2->TXDR |= 0xAA; // Read OUT_Y_L
	AwaitTC();
	
	SetupI2C2_Read_LH(); // Read 2 bytes at a time per transaction!
	Check_I2C2_Read_Ready();
	unsigned int raw_yLow = I2C2->RXDR & 0xFF;
	
	// Y_Hi
	Check_I2C2_Read_Ready();
	unsigned int raw_yHi = I2C2->RXDR & 0xFF;
	AwaitTC();
	Close_I2C2_Connection();
	
	short y = (raw_yLow << 8) | raw_yHi;
	return y;
}

/*
 * Sets the STOP bit in I2C2->CR2. Indicates end of a transaction.
 */ 
void Close_I2C2_Connection(void)
{
	//GPIOC->ODR &= ~(1 << 7); // Blue LED off
	I2C2->CR2 |= (1 << 14); // Release I2C bus; STOP bit to 1
}

/*
 * A waiting function that waits for RXNE bit
 * to be set. 
 * Intended to check if RXDR is ready to be read from.
 */
void Check_I2C2_Read_Ready(void)
{
	unsigned int nackfMask = (1 << 4);
	unsigned int nackf = I2C2->ISR & nackfMask;
	unsigned int rxneMask = (1 << 2);
	unsigned int rxne = I2C2->ISR & rxneMask;
	
	while (nackf == nackfMask){
		HAL_Delay(250);
		GPIOC->ODR ^= (1 << 6); // Connection error, blink Red
	}
	while (rxne != rxneMask){
		HAL_Delay(250);
		rxne = I2C2->ISR & rxneMask;
		//GPIOC->ODR ^= (1 << 8); // Waiting for some data, blink Orange
	}
}

/*
 * A waiting function that waits for TXIS bit
 * to be set. 
 * Intended to check if TXDR is ready to be written to.
 */
void Check_I2C2_Write_Ready(void)
{
	/* I2C CONNECTION CHECK */
	unsigned int nackfMask = (1 << 4);
	unsigned int txisMask = (1 << 1);
	
	unsigned int txis = I2C2->ISR & txisMask;
	unsigned int nackf = I2C2->ISR & nackfMask;
	if (nackf == nackfMask) // Connection Error
	{
		GPIOC->ODR |= (1 << 6); // Turn on RED LED, indicates connection error
		while(1);
	}
	while (txis != txisMask) // Slave did not respond yet
	{ 
		HAL_Delay(250);
		txis = I2C2->ISR & txisMask;
		//GPIOC->ODR ^= (1 << 8); // blink Orange LED, indicates slave yet to respond
	}
	//GPIOC->ODR &= ~(1 << 8); // Orange LED off
	/* END I2C CONNECTION CHECK */
}

/* 
 * Await Transfer Complete
 * Waits for the TC bit in I2C2_ISR to be a 1
 * Blinks Orange LED until done
 */
void AwaitTC(void)
{
	unsigned int tcMask = (1 << 6);
	while((I2C2->ISR & tcMask) != tcMask){
			HAL_Delay(50);
			//GPIOC->ODR ^= (1 << 8); // Blink Orange while waiting
		}
		//GPIOC->ODR &= ~(1 << 8);	// Orange off
}

/*
 * Sets up CR2 Params, with RD_WRN set to read, and NBYTES to 2
 * Intended to setup all params and set START for a transaction.
 */ 
void SetupI2C2_Read_LH(void)
{
	HAL_Delay(1);
	I2C2->CR2 &= ~(0x3FF);
	I2C2->CR2 |= (0x69 << 1); // SADD: Slave address set
	
	I2C2->CR2 &= ~(0xFF << 16);
	I2C2->CR2 |= (2 << 16); // NBYTES: 2 bytes to receive
	
	I2C2->CR2 |= (1 << 10); // RD_WRN: 1 -> Master requests read transfer
	
	I2C2->CR2 |= (1 << 13); // START: ENABLED
	//GPIOC->ODR |= (1 << 7);
}

/* 
 * Sets up CR2 parameters, with RD_WRN set to read
 * Intended to setup all params, and set START for a transaction.
 */
void SetupI2C2_Read(void)
{
	HAL_Delay(1);
	I2C2->CR2 &= ~(0x3FF);
	I2C2->CR2 |= (0x69 << 1); // SADD: Slave address set
	
	I2C2->CR2 &= ~(0xFF << 16);
	I2C2->CR2 |= (1 << 16); // NBYTES: 1 byte to receive
	
	I2C2->CR2 |= (1 << 10); // RD_WRN: 1 -> Master requests read transfer
	
	I2C2->CR2 |= (1 << 13); // START: ENABLED
	//GPIOC->ODR |= (1 << 7);
}


/*
 * Sets up CR2 Params, with RD_WRN set to write
 * Intended to setup all params and set START for a transaction.
 */
void SetupI2C2_Write(void)
{
	HAL_Delay(1);
	I2C2->CR2 &= ~(0x3FF);
	I2C2->CR2 |= (0x69 << 1); // SADD: Slave address set
	
	I2C2->CR2 &= ~(0xFF << 16);
	I2C2->CR2 |= (1 << 16); // NBYTES: 1 byte to send
	
	I2C2->CR2 &= ~((1 << 10)); // RD_WRN: 0 -> Master requests write transfer
	
	I2C2->CR2 |= (1 << 13); // START: ENABLED
	//GPIOC->ODR |= (1 << 7);
}

/*
 * Sets up CR2 Params, with RD_WRN set to write, and NBYTES to 2
 * Intended to setup params and allow for 2 bytes to be written.
 */ 
void SetupI2C2_Write_CONFIG(void)
{
	HAL_Delay(1);
	I2C2->CR2 &= ~(0x3FF);
	I2C2->CR2 |= (0x69 << 1); // SADD: Slave address set
	
	I2C2->CR2 &= ~(0xFF << 16);
	I2C2->CR2 |= (1 << 17); // NBYTES: 2 bytes to send
	
	I2C2->CR2 &= ~((1 << 10)); // RD_WRN: 0 -> Master requests write transfer
	
	I2C2->CR2 |= (1 << 13); // START: ENABLED
	//GPIOC->ODR |= (1 << 7);
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
