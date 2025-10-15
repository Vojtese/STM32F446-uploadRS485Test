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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART_BUS	UART4	// USART pro ProfiBUS
#define LL_DMA_CHANNEL_TX_BUS	LL_DMA_STREAM_4	// Vysilaci kanal DMA pro ProfiBUS

#define DE_BUS_Pin 				DE_Pin			// Pin pro DriverEnable
#define DE_BUS_GPIO_Port 		DE_GPIO_Port	// GPIO port pro DriverEnable
#define DE_BUS_Polarity			GPIO_PIN_SET		// GPIO_PIN_SET = pozitivni logika | GPIO_PIN_RESET = negace

#define RS485_RYCHLOST			38400
#define	DE_TX_START_DELAY		3000				// zpozdeni mezi povolenim budice a spustenim vysilani [us]
#define	DE_TX_END_DELAY			2200				// zpozdeni mezi povolenim budice a spustenim vysilani [us]
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t Presence = 0;
uint8_t Temp_byte1;
uint8_t Temp_byte2;
uint16_t TEMP;
uint16_t val;
uint8_t buff[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void profibus_send_string(uint8_t adresa, uint8_t FC, uint8_t* str, uint8_t len);
void PB_DE_Set(void);
void PB_DE_Reset(void);
volatile uint8_t usart_tx_dma_transfer=0;
void delay(uint16_t time);
uint32_t timeDiff(uint32_t stampA, uint32_t stampB);
uint8_t usart_start_tx_dma_transfer(void);
void DS18B20_Write (uint8_t data);
void delay2 (uint32_t us);
uint8_t DS18B20_Start (void);
uint8_t read (void);
void gpio_set_input(void);
void gpio_set_output(void);
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

	uint8_t strbuff1[16];
	uint8_t strbuff2[14];
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
  MX_UART4_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LL_DMA_ClearFlag_HT4(DMA1);
  LL_DMA_ClearFlag_TC4(DMA1);
  LL_DMA_ClearFlag_TE4(DMA1);


  //LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_TIM_Base_Start(&htim6);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Receive(&huart1, buff, 4, 100u);
	  	  for (int i = 0; i<4;i++)
	  	  {
	  		  if (buff[i] == 0xff)
	  		  {
	  			  switch(i){
	  			  case 0:
	  				  if(buff[i+1] == 0xff){
	  					  break;
	  				  }
	  				  val = ((uint16_t)buff[i+1] << 8) | buff[i+2];
	  				  break;
	  			  case 1:
	  				  if(buff[i+1] == 0xff){
	  					  break;
	  				  }
	  				  else if(buff[i-1] == 0xff){
	  					  break;
	  				  }
	  				  val = ((uint16_t)buff[i+1] << 8) | buff[i+2];
	  				  break;
	  			  case 2:
	  				  if(buff[i+1] == 0xff){
	  					  break;
	  				  }
	  				  else if(buff[i-1] == 0xff){
	  					  break;
	  				  }
	  				  val = ((uint16_t)buff[i+1] << 8) | buff[0];
	  				  break;
	  			  case 3:
	  				  if(buff[i+1] == 0xff){
	  					  break;
	  				  }
	  				  else if(buff[i-1] == 0xff){
	  					  break;
	  				  }
	  				  val = ((uint16_t)buff[0] << 8) | buff[i+1];
	  				  break;
	  			  default:
	  				  break;

	  			  }
	  		  }
	  	  }
	  	  sprintf(strbuff1 ,"distance: %ld\n\r",val);
	  	  profibus_send_string(1, 0, strbuff1, 16);
	  	  Presence = DS18B20_Start ();
	  	  HAL_Delay (1);
	  	  DS18B20_Write (0xCC);  // skip ROM
	  	  DS18B20_Write (0x44);  // convert t
	  	  HAL_Delay (800);

	  	  Presence = DS18B20_Start ();
	  	  HAL_Delay(1);
	  	  DS18B20_Write (0xCC);  // skip ROM
	  	  DS18B20_Write (0xBE);  // Read Scratch-pad

	  	  Temp_byte1 = read();
	  	  Temp_byte2 = read();
	  	  TEMP = (Temp_byte2<<8) | Temp_byte1;


	  	  sprintf(strbuff2,"Temp (%d.%d)\n\r",TEMP/16, TEMP%16);
	  	  profibus_send_string(1, 0, strbuff2, 14);
	  	  HAL_Delay(2000u);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 71;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**UART4 GPIO Configuration
  PA0-WKUP   ------> UART4_TX
  PA1   ------> UART4_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* UART4 DMA Init */

  /* UART4_TX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_4, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_4, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_4, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_4);

  /* USER CODE BEGIN UART4_Init 1 */
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_4, (uint32_t)&UART4->DR);

  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_TX_BUS);

  LL_USART_EnableDMAReq_TX(USART_BUS);
  /* USER CODE END UART4_Init 1 */
  USART_InitStruct.BaudRate = 38400;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(UART4, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(UART4);
  LL_USART_Enable(UART4);
  /* USER CODE BEGIN UART4_Init 2 */



  /* USER CODE END UART4_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ON_Pin|DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ON_Pin DE_Pin */
  GPIO_InitStruct.Pin = ON_Pin|DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * \brief           Sestavi paket a spusti prenos na sbernici
 * \param[in]       adresa: Cilova adresa paketu
 * \param[in]       FC: Funkcni kod
 * \param[in]       str: retezec k odeslani
 * \param[in]       len: delka retezce
 */
void profibus_send_string(uint8_t adresa, uint8_t FC, uint8_t* str, uint8_t len) {
	while(usart_tx_dma_transfer)
		;
	usart_tx_dma_transfer=1; // přenos probíhá
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_4, (uint32_t)str);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, len);	// nastavíme délku přenosu
	PB_DE_Set();	// Zapne vystup DriverEnable
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
	//}
    //usart_process_data(str, strlen(str));
}

/**
 * \brief	Nastavi DriverEnable a vypne prijimac
 * todo 	Zpozdeni urcovat podle timeru místo poctu pruchodu (v pripade preruseni se doba definovanych pruchodu prodlouzi)
 */
void PB_DE_Set(void){
	LL_USART_DisableDirectionRx(USART_BUS); //vypnu prijimac, aby neprijimal to, co odesila (nejednoznacne urovne na RX vystupu z budice behem zapnuteho DE)
	//LL_GPIO_SetPinMode(RX_BUS_GPIO_Port, RX_BUS_Pin, LL_GPIO_MODE_ANALOG);	//funguje pouze na F334, kde je vnitrne negovany Rx pin
	HAL_GPIO_WritePin(DE_BUS_GPIO_Port, DE_BUS_Pin, DE_BUS_Polarity);
	delay(DE_TX_START_DELAY);
	//for(uint16_t delay = DE_TX_START_DELAY;delay>0;--delay);	// zpozdeni mezi DE a spustenim vysilani (pri kratkem zpozdeti vysila prvni znaky chybne)
}

/**
 * \brief	Resetuje DriverEnable a zapne prijimac
 * todo 	Zpozdeni urcovat podle timeru místo poctu pruchodu (v pripade preruseni se doba definovanych pruchodu prodlouzi)
 */
void PB_DE_Reset(void){
	delay(DE_TX_END_DELAY);
	//HAL_Delay(4);
	//for(uint16_t delay = DE_TX_END_DELAY;delay>0;--delay);	// zpozdeni mezi ukoncenim vysilani a skoncenim DE
	HAL_GPIO_WritePin(DE_BUS_GPIO_Port, DE_BUS_Pin, !DE_BUS_Polarity);	//opet zapnu prijimac
	//LL_GPIO_SetPinMode(RX_BUS_GPIO_Port, RX_BUS_Pin, RX_BUS_Mode);	//funguje pouze na F334, kde je vnitrne negovany Rx pin
	//LL_USART_EnableDirectionRx(USART_BUS);
}

/**
 * Pocka alespon zadany pocet mikrosekund (min. 3us).
 * Mohlo by fungovat do asi 65ms.
 */
void delay(uint16_t time)
{
  uint32_t start = SysTick->VAL;   // doba zacatku cekani
  uint32_t now;                    // aktualni cas
  int32_t delay = 170 * ( (time>2)?(time-2):0 );    // kolik cyklu oscilatoru cekat

  while (delay > 0)
  {
    now = SysTick->VAL;    // aktualni cas systemoveho citace
    delay -= timeDiff(start, now); // odecteni doby od posledniho cyklu
    start = now;           // aktualizace doby zacatku cekani
  }
}
/**
 * Vrati casovy rozdil dvou casovych znacek
 * stampA - drivejsi udalost
 * stampB - pozdejsi udalost
 */
uint32_t timeDiff(uint32_t stampA, uint32_t stampB)
{
  if (stampA > stampB)
    return stampA - stampB;
  else
    return stampA - stampB + 170000;
}

void delay2 (uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim6,0);
    while ((__HAL_TIM_GET_COUNTER(&htim6))<us);
}

void DS18B20_Write (uint8_t data)
{
	gpio_set_output();

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			gpio_set_output();
			HAL_GPIO_WritePin (GPIOC, DS18B20_Pin, 0);  // pull the pin LOW
			delay2 (1);  // wait for 1 us

			gpio_set_input();
			delay2 (60);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			gpio_set_output();
			HAL_GPIO_WritePin (GPIOC, DS18B20_Pin, 0);  // pull the pin LOWi
			delay2 (60);  // wait for 60 us

			gpio_set_input();
		}
	}
}
uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	gpio_set_output();
	HAL_GPIO_WritePin (GPIOC, DS18B20_Pin, 0);  // pull the pin low
	delay2 (480);   // delay according to datasheet

	gpio_set_input();
	delay2 (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (GPIOC, DS18B20_Pin))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	delay2 (400); // 480 us delay totally.

	return Response;
}
uint8_t read (void)
{
	uint8_t value=0;
	gpio_set_input ();

	for (int i=0;i<8;i++)
	{
		gpio_set_output ();   // set as output

		HAL_GPIO_WritePin (GPIOC, DS18B20_Pin, 0);  // pull the data pin LOW
		delay2 (2);  // wait for 2 us

		gpio_set_input ();  // set as input
		if (HAL_GPIO_ReadPin (GPIOC, DS18B20_Pin))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay2 (60);  // wait for 60 us
	}
	return value;
}
void gpio_set_input(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DS18B20_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
void gpio_set_output(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DS18B20_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
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
