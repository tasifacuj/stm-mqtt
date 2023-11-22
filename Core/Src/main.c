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
//#include "main.h"
#include "stm32wbxx_hal.h"
#include "w5500_spi.h"
#include "wizchip_conf.h"
#include "socket.h"

#include "MQTTClient.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>


/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_MEMORYMAP_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void UWriteData(const char data);
static void PHYStatusCheck(void);
static void PrintPHYConf(void);

wiz_NetInfo gWIZNETINFO = {
	.mac = { 0x80, 0x34, 0x28, 0x74, 0xA5, 0xCB },
	.ip = { 192,168,2,112 },
	.sn = { 255, 255, 255, 0 },
	.gw = { 192, 168, 2, 1 },
	.dns = { 8,8,8,8 },
	.dhcp = NETINFO_STATIC
};

uint8_t destination_ip[] = { 192, 168, 3, 111 };
uint16_t destination_port = 5000;
uint8_t buf[100];

#define LISTEN_PORT 5000
#define RECEIVE_BUFF_SIZE 128
uint8_t receive_buff[RECEIVE_BUFF_SIZE];//to receive data from client

#define BUFFER_SIZE	2048
uint8_t tempBuffer[BUFFER_SIZE];

uint16_t mqtt_push_counter = 0;
uint8_t mqtt_flag = 0;
uint16_t mes_id = 0;

static void tcp_client(){
	if( socket(1, Sn_MR_TCP, 0, 0) == 1 ){
		  printf( "\r\nSocket created" );
	  }else{
		  printf( "\r\nSocket creation failed" );
		  while( 1 );
	  }

	  printf("\r\nConnecting to server: %d.%d.%d.%d @ TCP Port: %d",destination_ip[0],destination_ip[1],destination_ip[2],destination_ip[3],destination_port);

	  if(connect(1, destination_ip, destination_port)==SOCK_OK){
		  printf("\r\nConnected with server.");
	  }else{
		  //failed
		  printf("\r\nCannot connect with server!");
		  while(1);
	  }

	  while (1){
		  if(send(1, "pipisanto futuro\r\n", sizeof( "pipisanto futuro\r\n" ))<=SOCK_ERROR){
			  printf("\r\nSending Failed!");
			  while(1);

		  }else{
			  printf("\r\nSending Success!");
		  }

		  HAL_Delay(1000);
	  }
}

static void tcp_echo_server(){
	printf( "\r\n***************************** TCP SERVER*****************************\r\n" );

	  while( 1 ){
		  printf( "\r\nInitializing server socket\r\n" );
		  const uint8_t SOCK_FD = 1;

		  if( socket( SOCK_FD, Sn_MR_TCP, LISTEN_PORT, 0 ) != SOCK_FD ){
			  printf( "Failed to create serve socket\r\n" );
	//		  asm( "" )
			  while(1);
		  }

		  printf( "Socket created successfully\r\n" );
		  uint8_t mode = SOCK_IO_BLOCK;
		  ctlsocket( SOCK_FD, CS_SET_IOMODE, &mode );
		  printf( "start listening on port %d\r\n", LISTEN_PORT );
		  printf( "Waiting for the client connection\r\n" );

		  if( listen( SOCK_FD ) != SOCK_OK ){
			  printf( "Failed to listen socket\r\n" );
			  while(1);
		  }

		  uint8_t sr = 0x00;

		  do{
			  sr = getSn_SR( SOCK_FD );
		  }while( sr != 0x17 && sr != 0x00 );

		  if( sr == 0x00 ){
			  printf( "Internal server error, please restart\r\n" );
			  while(1);
		  }

		  if( sr == 0x17 ){// incoming connection
			  printf( "Client connected, waiting for data\r\n" );

			  while( 1 ){
				  int len = recv( SOCK_FD, receive_buff, RECEIVE_BUFF_SIZE );

				  if( len == SOCKERR_SOCKSTATUS ){
					  // client disconnected
					  printf( "Client disconnected\r\n" );
					  break;
				  }

				  receive_buff[ len ] = '\0';
				  printf( "Received %s (%i)\r\n", receive_buff, len );
				  // echo data back
				  send( SOCK_FD, ( uint8_t* )"[", 1 );
				  send( SOCK_FD, receive_buff, len );
				  send( SOCK_FD, ( uint8_t* )"]", 1 );

				  if( strcmp( ( char const* )receive_buff, "QUIT" ) == 0 ){
					  printf( "Received QUIT command\r\n" );
					  disconnect( SOCK_FD );
					  break;
				  }
			  }// read loop
		  }// incoming conection
	  }// outer loop
}

static void messageArrived(MessageData* md)
{
	MQTTMessage* message = md->message;

	  for (uint8_t i = 0; i < md->topicName->lenstring.len; i++)
	    putchar(*(md->topicName->lenstring.data + i));

	  printf(" (%.*s)\r\n", (int32_t)message->payloadlen, (char*)message->payload);
}

static void MX_TIM2_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */

static int8_t str_printf(char *StrBuff, uint8_t BuffLen, const char *args, ...)
{
  va_list ap;
  va_start(ap, args);
  int8_t len = vsnprintf(StrBuff, BuffLen, args, ap);
  va_end(ap);
  return len;
}

int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  setbuf(stdout, NULL);
  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_MEMORYMAP_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();

  printf("My First W5500 Application!\r\n");
//  printf("HCLK=%d\n", HAL_RCC_GetHCLKFreq());
//
//  printf("APB1=%d\n", HAL_RCC_GetPCLK1Freq());
//
//  printf("APB2=%d\n", HAL_RCC_GetPCLK2Freq());
  HAL_TIM_Base_Start_IT(&htim2);
  W5500Init();
  ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);

  wiz_PhyConf phyConf;
  phyConf.by = PHY_CONFBY_SW;
  phyConf.duplex = PHY_DUPLEX_FULL;
  phyConf.speed = PHY_SPEED_10;
  phyConf.mode = PHY_MODE_AUTONEGO;
  ctlwizchip( CW_SET_PHYCONF, &phyConf );

  PHYStatusCheck();
  PrintPHYConf();

  Network n;
  MQTTClient c;
  const uint8_t SOCK_FD = 1;
  uint8_t targetIP[4] = {192, 168, 2, 102};
  uint16_t targetPort = 1883;
  int rc = 0;

  NewNetwork(&n, SOCK_FD);
  ConnectNetwork(&n, targetIP, targetPort);
  MQTTClientInit(&c,&n,1000,buf,100,tempBuffer,2048);

  	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  	  data.willFlag = 0;
  	  data.MQTTVersion = 3;//3;
  	  data.clientID.cstring = (char*)"w5500-client";
  	  data.username.cstring = "username";
  	  data.password.cstring = "";
  	  data.keepAliveInterval = 60;
  	  data.cleansession = 1;

  	rc = MQTTConnect(&c, &data);
  	printf("Connected %d\r\n", rc);


  	printf("Subscribing to %s\r\n", "hello/wiznet");
  	rc = MQTTSubscribe(&c, "hello/wiznet", QOS0, messageArrived);
  	printf("Subscribed %d\r\n", rc);

      while(1){
    	  if( mqtt_flag ){
    		  mqtt_flag = 0;
    		  char message[16];
			int8_t len = str_printf(message, sizeof(message), "%d.%d.%d.%d", gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2], gWIZNETINFO.ip[3]);

			if (len > 0){
			  MQTTMessage pubMessage;
			  pubMessage.qos = QOS0;
			  pubMessage.id = mes_id++;
			  pubMessage.payloadlen = len;
			  pubMessage.payload = message;
			  rc = MQTTPublish(&c, "/w5500_stm32_client", &pubMessage);

			  if( rc != MQTT_SUCCESS ){
				  printf( "Failed to send pub\r\n" );
			  }
			}
    	  }
      	MQTTYield(&c, data.keepAliveInterval);
      }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
//    printf( "Timer alarm\r\n" );
	MilliTimer_Handler();

	if (++mqtt_push_counter >= 1000){
	  mqtt_push_counter = 0;
	  mqtt_flag = 1;
	}
}

void UWriteData(const char data)
{
	while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TXE)==RESET);

	huart1.Instance->TDR=data;

}

int __io_putchar(int ch)
{
	UWriteData(ch);
	return ch;
}

void PHYStatusCheck(void)
{
	uint8_t tmp;

	do
	{
		printf("\r\nChecking Ethernet Cable Presence ...");
		ctlwizchip(CW_GET_PHYLINK, (void*) &tmp);

		if(tmp == PHY_LINK_OFF)
		{
			printf("NO Cable Connected!");
			HAL_Delay(1500);
		}
	}while(tmp == PHY_LINK_OFF);

	printf("Good! Cable got connected!");
}

void PrintPHYConf(void)
{
	wiz_PhyConf phyconf;

	ctlwizchip(CW_GET_PHYCONF, (void*) &phyconf);

	if(phyconf.by==PHY_CONFBY_HW)
	{
		printf("\n\rPHY Configured by Hardware Pins");
	}
	else
	{
		printf("\n\rPHY Configured by Registers");
	}

	if(phyconf.mode==PHY_MODE_AUTONEGO)
	{
		printf("\n\rAutonegotiation Enabled");
	}
	else
	{
		printf("\n\rAutonegotiation NOT Enabled");
	}

	if(phyconf.duplex==PHY_DUPLEX_FULL)
	{
		printf("\n\rDuplex Mode: Full");
	}
	else
	{
		printf("\n\rDuplex Mode: Half");
	}

	if(phyconf.speed==PHY_SPEED_10)
	{
		printf("\n\rSpeed: 10Mbps");
	}
	else
	{
		printf("\n\rSpeed: 100Mbps");
	}
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief MEMORYMAP Initialization Function
  * @param None
  * @retval None
  */
static void MX_MEMORYMAP_Init(void)
{

  /* USER CODE BEGIN MEMORYMAP_Init 0 */

  /* USER CODE END MEMORYMAP_Init 0 */

  /* USER CODE BEGIN MEMORYMAP_Init 1 */

  /* USER CODE END MEMORYMAP_Init 1 */
  /* USER CODE BEGIN MEMORYMAP_Init 2 */

  /* USER CODE END MEMORYMAP_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  __HAL_SPI_ENABLE(&hspi1);
  /* USER CODE END SPI1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void){

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 64;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
