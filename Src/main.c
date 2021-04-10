/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

#include "stm_nrf24_arduinoforked_library.h"
#include "stdio.h"


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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE_PUT int __io_putchar(int ch)
#define PUTCHAR_PROTOTYPE_GET int __io_getchar(int ch)
#else
#define PUTCHAR_PROTOTYPE_PUT int fputc(int ch, FILE *f)
#define PUTCHAR_PROTOTYPE_GET int fgetc(FILE *f)
#endif 

PUTCHAR_PROTOTYPE_PUT
{
  HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFF);
  if(ch == '\n')
  {
    ch = '\r';
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFF);
  }
  return ch;
}

PUTCHAR_PROTOTYPE_GET
{
  uint8_t ch;
  while( HAL_UART_Receive(&huart1, (uint8_t*)&ch, 1, 0xFFFF) != HAL_OK);
  return ch;
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint32_t radio_transferred_amount = 0;
uint32_t radio_transferred_amount_past = 0;

uint32_t radio_dropped_amount = 0;
uint32_t radio_dropped_amount_past = 0;

uint8_t incoming_pipeno = 0, nrf24_status = 0;
#define RADIO_ADR_WIDTH 5
#define PAYLOAD_LEN 32
uint8_t radio_payload[PAYLOAD_LEN+1] = {0};

uint8_t NRF24_RADIO_ADDRESS[RADIO_ADR_WIDTH] = {0x12, 0x34, 0xAB, 0xCD, 0xEF};

volatile uint8_t timer_elapsed = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == htim4.Instance)
  {
    timer_elapsed = 1;
  }
}

volatile uint8_t nrf_irq_pin_active = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_0)
  {
    nrf_irq_pin_active = 1;
  }
}

#define MASTER_TRANSFER_ROLE

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  
  NRF24L01_Init();
  nrf24_setAutoAck(1);
  nrf24_enableAckPayload();
  setPALevel(RF24_PA_MAX);
  openReadingPipe(1, NRF24_RADIO_ADDRESS, RADIO_ADR_WIDTH, PAYLOAD_LEN);
  openWritingPipe(NRF24_RADIO_ADDRESS, RADIO_ADR_WIDTH,PAYLOAD_LEN);  
  
  
  
#ifdef MASTER_TRANSFER_ROLE
  
  printf("!!!!!!!!! MASTER ROLE !!!!!!!!!\n");
  stopListening();
  
  nrf_irq_pin_active = 0;
  nrf24_write_IT(radio_payload, PAYLOAD_LEN);
      
#else
  printf("!!!!!!!!! SLAVE ROLE !!!!!!!!!\n");
  startListening(NRF24_RADIO_ADDRESS, RADIO_ADR_WIDTH);
#endif
  
  
  
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef MASTER_TRANSFER_ROLE
    
    if(timer_elapsed == 1)
    {
      timer_elapsed = 0;
      
      printf("\rTX:%5d\tdrop:%5d\t",
             radio_transferred_amount - radio_transferred_amount_past,
             radio_dropped_amount - radio_dropped_amount_past);
      
      radio_transferred_amount_past = radio_transferred_amount;
      radio_dropped_amount_past = radio_dropped_amount;
      
//      static uint8_t rotate = 0;
//      rotate++;
//      if(rotate >= 26) rotate = 0;
//      
//      for(int i=0;i<32;i++)
//        radio_payload[i] = 'A'+((i+rotate)%26);
    }
    
    
    if(nrf_irq_pin_active == 1)
    {
      nrf_irq_pin_active = 0;
      
      nrf24_status = get_status();
      
      if(nrf24_status & (1<<TX_DS))
      {
//        printf("data sent %s\n", radio_payload);
        
        while(nrf24_rxfifo_available())
        {
          nrf24_read_payload(radio_payload, PAYLOAD_LEN);
//          printf("receive:%s\n", radio_payload);
          
          HAL_GPIO_WritePin(PIN_OUT_GPIO_Port, PIN_OUT_Pin, (GPIO_PinState)radio_payload[10]);
        }
        
        nrf24_clear_flags();
        
        radio_transferred_amount+=PAYLOAD_LEN;
      }
      else if(nrf24_status & (1<<MAX_RT))
      {
//        printf("data send failed\n");
        
        nrf24_flush_rx();
        nrf24_flush_tx();
        
        radio_dropped_amount+=PAYLOAD_LEN;
      }
      
      nrf24_clear_flags();
      
      radio_payload[10] = HAL_GPIO_ReadPin(BTN_IN_GPIO_Port, BTN_IN_Pin);
      nrf_irq_pin_active = 0;
      nrf24_write_IT(radio_payload, PAYLOAD_LEN);
    }
    
#else
    
    if(timer_elapsed == 1)
    {
      timer_elapsed = 0;
      
      printf("\rRX:%5d\t",
             radio_transferred_amount - radio_transferred_amount_past);
      
      radio_transferred_amount_past = radio_transferred_amount;
    }
    
    
    if(nrf_irq_pin_active == 1)
    {
      nrf_irq_pin_active = 0;
      
      nrf24_status = get_status();
      
      if(nrf24_status & (1<<RX_DR))
      {
        while(nrf24_rxfifo_available())
        {
          incoming_pipeno = get_incoming_pipeno();
          nrf24_read_payload(radio_payload,PAYLOAD_LEN);
          
          uint8_t temp[32];
//          for(uint8_t i=0;i<32;i++)
//          {
//            temp[i] = radio_payload[32-i-1];
//          }
          
          
          HAL_GPIO_WritePin(PIN_OUT_GPIO_Port, PIN_OUT_Pin, (GPIO_PinState)radio_payload[10]);
          temp[10] = HAL_GPIO_ReadPin(BTN_IN_GPIO_Port, BTN_IN_Pin);
          
          nrf24_ack_write_IT(temp, PAYLOAD_LEN,incoming_pipeno);
          radio_transferred_amount+=32;
        }
      }
    }
    
#endif
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
    Error_Handler();
  }  
  /* USER CODE END TIM4_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_NRF_CE_Pin|SPI1_NRF_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PIN_OUT_GPIO_Port, PIN_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_NRF_CE_Pin SPI1_NRF_CSN_Pin */
  GPIO_InitStruct.Pin = SPI1_NRF_CE_Pin|SPI1_NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_IN_Pin */
  GPIO_InitStruct.Pin = BTN_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_OUT_Pin */
  GPIO_InitStruct.Pin = PIN_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN_OUT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
  
  printf("\n\n");  
  uint8_t step = 0;
  char msg[] = "ERRORHANDLER";
  while(1)
  {
    step++;
    if(step >= 12) step = 0;
    msg[step] += 32;
    printf("\r%s", msg);
    msg[step] -= 32;
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    HAL_Delay(200);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
