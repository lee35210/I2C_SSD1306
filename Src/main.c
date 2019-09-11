/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include<stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define ssd1306_Addr 0x3c

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

uint8_t ssd1306_buffer[128*8]={0};
uint8_t ssd1306_Fonts[]={0x00, 0x00, 0x00, 0xE0, 0xF8, 0x1F, 0x07, 0x1F, 0xF8, 0xE0, 0x00, 0x00, 0x00,
					0x60, 0x7C, 0x1F, 0x07, 0x06, 0x06, 0x06, 0x06, 0x06, 0x07, 0x1F, 0x7C, 0x60
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

void ssd1306_W_Command(uint8_t cmd);
void ssd1306_W_Data(uint8_t* data_buffer, uint16_t buffer_size);

void ssd1306_Init(void)
{
//	HAL_Delay(100);
//	ssd1306_W_Command(0xAE);

	ssd1306_W_Command(0xA8);	//Set Mux Ratio
	ssd1306_W_Command(0x3F);

	ssd1306_W_Command(0xD3);	//Set Display Offset
	ssd1306_W_Command(0x00);

	ssd1306_W_Command(0x40);	//Set Display Start Line
	ssd1306_W_Command(0xA1);	//Set Segment re-map
	ssd1306_W_Command(0xC8);	//Set COM Output Scan Direction

	ssd1306_W_Command(0xDA);	//Set COM Pins hardware configuration
	ssd1306_W_Command(0x12);	//COM Pins Hardware Configuration 1

	ssd1306_W_Command(0x20);	//Set Memory Addressing Mode
	ssd1306_W_Command(0x02);	//0x02 Page Addressing Mode A[1:0]

//	ssd1306_W_Command(0xB0);	//
//	ssd1306_W_Command(0x00);
//	ssd1306_W_Command(0x10);

	ssd1306_W_Command(0x81);	//Set Contrast Control
	ssd1306_W_Command(0x7F);

	ssd1306_W_Command(0xA4);	//Disable Entire Display On
	ssd1306_W_Command(0xA6);	//Set Normal Display

	ssd1306_W_Command(0xD5);	//Set Osc Frequency
	ssd1306_W_Command(0x80);

//	ssd1306_W_Command(0xD9);	//
//	ssd1306_W_Command(0x22);
//
//	ssd1306_W_Command(0xDB);	//
//	ssd1306_W_Command(0x20);

	ssd1306_W_Command(0x8D);	//Enable charge pump regulator
	ssd1306_W_Command(0x14);

	ssd1306_W_Command(0xAF);	//Display ON
}

void ssd1306_W_Command(uint8_t cmd)
{
	uint8_t buffer[2]={0};		//command+number of data
	buffer[0]=(0<<7)|(0<<6);	//Co=1 , D/C=0
	buffer[1]=cmd;

	if(HAL_I2C_Master_Transmit_DMA(&hi2c1,(uint16_t)(ssd1306_Addr)<<1,(uint8_t*)buffer,2)!= HAL_OK)
	{
		Error_Handler();
	}
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
	}
//	printf("command : %x \r\n",buffer[1]);
}

void ssd1306_W_Data(uint8_t* data_buffer, uint16_t buffer_size)
{
	if(HAL_I2C_Mem_Write_DMA(&hi2c1,(uint16_t)(ssd1306_Addr<<1),0x40,1,data_buffer,buffer_size)!= HAL_OK)
	{
		Error_Handler();
	}
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{
	}
}

void ssd1306_Update(void)
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		ssd1306_W_Command(0xB0+i);
		ssd1306_W_Command(0x00);
		ssd1306_W_Command(0x10);
	}
}

void ssd1306_Clear(void)
{
	uint8_t buffer[128*8]={0};

	ssd1306_W_Command(0xB0);	//Set Page Start Address for Page Addressing Mode [X2:0]
	ssd1306_W_Command(0x00);	//Set Lower Column
	ssd1306_W_Command(0x10);	//Set Higher Column

	for(uint8_t i=0;i<8;i++)
	{
		ssd1306_W_Command(0xB0+i);
		ssd1306_W_Command(0x00);
		ssd1306_W_Command(0x10);
		ssd1306_W_Data(&buffer[i*128],128*8);
	}
}


void ssd1306_Fill_Screen(uint8_t data)
{
	for(uint16_t i=0;i<128;i++)
	{
		if((i%2)==0)
		{
			ssd1306_buffer[i]=0xAA;
		}
		else
		{
			ssd1306_buffer[i]=0x55;
		}
//		printf("%d %x\r\n",i,ssd1306_buffer[i]);
	}

	for(uint8_t i=0;i<8;i++)
	{
		ssd1306_W_Command(0xB0+i);
		ssd1306_W_Command(0x00);
		ssd1306_W_Command(0x10);
		ssd1306_W_Data(ssd1306_buffer,128);
	}
}

void ssd1306_Update_Screen(void)
{
	for(uint8_t i=0;i<8;i++)
	{
		ssd1306_W_Command(0xB0+i);
		ssd1306_W_Command(0x00);
		ssd1306_W_Command(0x10);
		ssd1306_W_Data(&ssd1306_buffer[128*i],128);
	}
}

void ssd1306_W_Fonts(uint8_t page, uint8_t column)	//page, column, font,
{
//	uint8_t fonts[]={0x00, 0x00, 0x00, 0xE0, 0xF8, 0x1F, 0x07, 0x1F, 0xF8, 0xE0, 0x00, 0x00, 0x00,
//			0x60, 0x7C, 0x1F, 0x07, 0x06, 0x06, 0x06, 0x06, 0x06, 0x07, 0x1F, 0x7C, 0x60};
////	uint8_t font_B[]={0x00, 0x00, 0x7F, 0x49, 0x49, 0x37};
//	uint8_t font_B_S[]={0x60, 0x18, 0x16, 0x11, 0x16, 0x18, 0x60};
//
//
////	ssd1306_W_Command(0xB0+page);
//
//	for(uint8_t i=0;i<13;i++)
//	{
//		for(uint8_t j=0;j<8;j++)
//		{
//			ssd1306_buffer[i+16*j]=font_A[i];
//			ssd1306_buffer[i+16*j+128]=font_A[i+13];
//		}
//	}
//	for(uint8_t i=0;i<7;i++)
//	{
//		ssd1306_buffer[128*2+i]=font_B_S[i];
//	}
//
//	ssd1306_W_Command(0xB0);
//	ssd1306_W_Data(ssd1306_buffer,128);
//	ssd1306_W_Command(0xB1);
//	ssd1306_W_Data(&ssd1306_buffer[128],128);
//	ssd1306_W_Command(0xB2);
//	ssd1306_W_Data(&ssd1306_buffer[128*2],128);
}

void ssd1306_W_Char(uint8_t character_Code, uint8_t page)
{
	uint8_t char_Buffer[26]={0};
	for(uint8_t i=0;i<26;i++)
	{
		char_Buffer[i]=ssd1306_Fonts[(character_Code-65)+i];
	}


	ssd1306_W_Command(0xB0+page);
	ssd1306_W_Data(&char_Buffer[0],13);
	ssd1306_W_Command(0xB0+page+1);
	ssd1306_W_Data(&char_Buffer[13],13);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
int _write(int file,uint8_t*ptr,int len)
{
  HAL_UART_Transmit(&huart1,ptr,len,1000);
  return len;
}
#elif (__ICCARM__||__CC_ARM)
int fputc(int c,FILE*stream)
{
  HAL_UART_Transmit(&huart1,(uint8_t*)&c,1,1000);
  return c;
}
#endif

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  ssd1306_Init();
  ssd1306_Clear();

//  ssd1306_W_Fonts(0,0);

  ssd1306_W_Char('A',0);

  int i=0;
  uint8_t line=0;

//  ssd1306_Fill_Screen(0xFF);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  for(i=0;i<8;i++)
//	  {
//		  line=(line<<1)|1;
//		  ssd1306_Fill_Screen(line);
//		  HAL_Delay(1000);
//	  }
//	  line=0;
//	  ssd1306_Fill_Screen(line);
//	  HAL_Delay(2000);
//	  ssd1306_Clear();
//	  HAL_Delay(1000);
//	  ssd1306_W_Fonts(0,0);
//	  HAL_Delay(1000);

//	  for(i=0;i<8;i++)
//	  {
//		  line=(line<<1)|1;
//		  ssd1306_Fill_Screen(line);
////		  printf("%d : %x\r\n",i, line);
//		  HAL_Delay(1000);
//	  }
//	  line=0;
//	  ssd1306_Fill_Screen(line);
//	  HAL_Delay(1000);


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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Macro to configure the PLL multiplication factor 
  */
  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV1);
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_MSI);
  /** Configure LSE Drive Capability 
  */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLLSAI1.PLLN = 24;
  PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_USBCLK;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00100413;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /** I2C Enable Fast Mode Plus 
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.Parity = UART_PARITY_ODD;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|LD3_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD3_Pin LD1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD3_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B3_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
