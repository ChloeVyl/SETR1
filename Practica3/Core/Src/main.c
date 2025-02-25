/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "hd44780.h"
#include "LPS22.h"
#include "HTS221.h"
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
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void maximos(float T, float H, float P);
void minimos(float T, float H, float P);
void resetVG(float T, float P, float H);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rxByte;
float minP;
float minT;
float minH;
float maxP;
float maxT;
float maxH;
int modo = 4;
int unidad = 0;
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  lcd_reset();
  lcd_display_settings(1, 0, 0);
  lcd_clear();
  LPS22_Init();
  HTS221_Init();
  HAL_UART_Receive_IT(&huart1, &rxByte, 1);

 THSample ths;
 ths = HTS221_Read();
 float press;
 press = LPS22_ReadPress();

 maxH = ths.hum;
 maxT = ths.temp;
 maxP = press;

 minH = ths.hum;
 minT = ths.temp;
 minP = press;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	uint8_t str[20];

	THSample ths;
	ths = HTS221_Read();
	float press;
	press = LPS22_ReadPress();

	maximos(ths.temp, ths.hum, press);
	minimos(ths.temp, ths.hum, press);

	if(modo == 0){
		if(unidad==0){
			sprintf(str, "Pres: %.1fhPA", maxP);
			moveToXY(0, 0);
			lcd_print(str);

			sprintf(str, "T:%.1fC H:%.1f\%%", maxT, maxH);
			moveToXY(1, 0);
			lcd_print(str);
			HAL_Delay(500);

			printf("Maximos - > Pres: %.1fhPA T:%.1fC H:%.1f%%\r\n", maxP, maxT, maxH);
		}else{
			sprintf(str, "Pres: %.1fmmHg", maxP*0.750);
			moveToXY(0, 0);
			lcd_print(str);

			sprintf(str, "T:%.1fF H:%.1f\%%", (maxT*9/5)+32, maxH);
			moveToXY(1, 0);
			lcd_print(str);
			HAL_Delay(500);

			printf("Maximos - > Pres: %.1fmmHg T:%.1fF H:%.1f%%\r\n", maxP*0.750, (maxT*9/5)+32, maxH);
		}
	}else if(modo == 1){
		if(unidad == 0){
			sprintf(str, "Pres: %.1fhPA", minP);
			moveToXY(0, 0);
			lcd_print(str);

			sprintf(str, "T:%.1fC H:%.1f\%%", minT, minH);
			moveToXY(1, 0);
			lcd_print(str);
			HAL_Delay(500);

			printf("Minimos - > Pres: %.1fhPA T:%.1fC H:%.1f%%\r\n", minP, minT, minH);
		}else{
			sprintf(str, "Pres: %.1fmmHg", minP*0.750);
			moveToXY(0, 0);
			lcd_print(str);

			sprintf(str, "T:%.1fF H:%.1f\%%", (minT*9/5)+32, minH);
			moveToXY(1, 0);
			lcd_print(str);
			HAL_Delay(500);

			printf("Minimos - > Pres: %.1fmmHg T:%.1fF H:%.1f%%\r\n", minP*0.750, (minT*9/5)+32, minH);
		}
	}else if(modo == 2){
		resetVG(ths.temp, press, ths.hum);
	}else{
		if(unidad == 0){
			sprintf(str, "Pres: %.1fhPA", press);
			moveToXY(0, 0);
			lcd_print(str);

			sprintf(str, "T:%.1fC H:%.1f\%%", ths.temp, ths.hum);
			moveToXY(1, 0);
			lcd_print(str);
			HAL_Delay(500);

			printf("Pres: %.1fhPA T:%.1fC H:%.1f%%\r\n", press, ths.temp, ths.hum);
		} else {
			sprintf(str, "Pres: %.1fmmHg", press*0.750);
			moveToXY(0, 0);
			lcd_print(str);

			sprintf(str, "T:%.1fC H:%.1f\%%", (ths.temp*9/5)+32, ths.hum);
			moveToXY(1, 0);
			lcd_print(str);
			HAL_Delay(500);

			printf("Pres: %.1fmmHg T:%.1fF H:%.1f%%\r\n", press*0.750, (ths.temp*9/5)+32, ths.hum);
		}
	}
//	sprintf(str, "Pres: %.1fhPA", press);
//	moveToXY(0, 0);
//	lcd_print(str);
//
//	sprintf(str, "T:%.1fC H:%.1f\%%", ths.temp, ths.hum);
//	moveToXY(1, 0);
//	lcd_print(str);
//	HAL_Delay(500);
//
//	printf("Pres: %.1fhPA T:%.1fC H:%.1f%%\r\n", press, ths.temp, ths.hum);

//	HAL_UART_Transmit(&huart1, "hola\n\r", 6, 1000);

//	lcd_print("La presion es de:");
//	moveToXY(1, 0);
//	writeIntegerToLCD(press);
//	lcd_print("hPA");
//	HAL_Delay(5000);
//	lcd_clear();
//	lcd_print("La humedad es de:");
//	moveToXY(1, 0);
//	writeIntegerToLCD(ths.hum);
//	lcd_print("%");
//	HAL_Delay(5000);
//	lcd_clear();
//	lcd_print("La temperatura es de:");
//	moveToXY(1, 0);
//	writeIntegerToLCD(ths.temp);
//	lcd_print("C");
//	HAL_Delay(10000);
//	lcd_clear();
//	lcd_print("Borrando...");
//	HAL_Delay(400);
//	lcd_clear();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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
  RCC_OscInitStruct.PLL.PLLN = 40;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Led_LCD_Pin|D4_LCD_Pin|D7_LCD_Pin|E_LCD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D6_LCD_Pin|RS_LCD_Pin|D5_LCD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Led_LCD_Pin D4_LCD_Pin D7_LCD_Pin E_LCD_Pin */
  GPIO_InitStruct.Pin = Led_LCD_Pin|D4_LCD_Pin|D7_LCD_Pin|E_LCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D6_LCD_Pin RS_LCD_Pin D5_LCD_Pin */
  GPIO_InitStruct.Pin = D6_LCD_Pin|RS_LCD_Pin|D5_LCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
	  uint8_t c[1];
	  c[0] = ch & 0x00FF;
	  HAL_UART_Transmit(&huart1, &*c, 1, 10);
	  return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//1 - Procesar Dato
	if(rxByte == 'm'){
		modo = 0;
	} else if(rxByte == 'n'){
		modo = 1;
	} else if(rxByte == 'c'){
		modo = 2;
	} else if(rxByte == 'u'){
		if (unidad == 1){
			unidad = 0;
		}else{
			unidad = 1;
		}

	} else{
		modo = 4;
	}
//2 - Esperar siguiente dato
	HAL_UART_Receive_IT(&huart1, &rxByte, 1);
}

void maximos(float T, float H, float P){
	if(maxT< T){
		maxT = T;
	}
	if(maxH< H){
		maxH = H;
	}
	if(maxP< P){
		maxP = P;
	}
}

void minimos(float T, float H, float P){
	if(minT> T){
		minT = T;
	}
	if(minH> H){
		minH = H;
	}
	if(minP> P){
		minP = P;
	}
}

void resetVG(float T, float P, float H){
	minH = H;
	minP = P;
	minT = T;
	maxH = H;
	maxP = P;
	maxT = T;
}
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
