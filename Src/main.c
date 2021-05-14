/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "lwip/apps/httpd.h"
#include "ethernetif.h"
#include "stm32f4xx_hal.h"
#include "lwip/tcp.h"
#include "ADS8688.h"
#include "FLASH_SECTOR.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_DEBUG 1
#define ETH_TCP_DEBUG 1

#define TCP_DEBUG_PORT 6000
#define startport 5000
#define numofports 10
#define numofclients 3

#define default_interval 500
#define default_msg1_interval 100
#define default_msg2_interval 100


#define BYTE_TO_BIN_PAT "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BIN(byte)		\
	(byte & 0x80 ? '1' : '0'),	\
	(byte & 0x40 ? '1' : '0'),	\
	(byte & 0x20 ? '1' : '0'),	\
	(byte & 0x10 ? '1' : '0'),	\
	(byte & 0x08 ? '1' : '0'),	\
	(byte & 0x04 ? '1' : '0'),	\
	(byte & 0x02 ? '1' : '0'),	\
	(byte & 0x01 ? '1' : '0')

#define ADC_BUF_LEN 2

// --------------------------------------------------------------------------- //
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
CAN_FilterTypeDef FilterConfig0;

CAN_RxHeaderTypeDef RxHeader1;
CAN_TxHeaderTypeDef TxHeader1;
CAN_RxHeaderTypeDef RxHeader2;
CAN_TxHeaderTypeDef TxHeader2;
uint8_t r1[8];
uint8_t r2[8];
// Can messages
uint8_t msg1[8] = {0,0,0,0,0,0,0,0};
uint8_t msg2[8] = {0,0,0,0,0,0,0,0};

char buf[100];

// Timing Variables
unsigned int current=0, previous=0, interval=default_interval;
uint32_t my_ip=0, my_net=0, my_gw=0, phyreg = 0U;

// General variables
uint8_t red_led=0, blue_led=0, green_led=0, dhcp_en=1, request_static=0, button_pressed=0;
uint16_t adc_buf[ADC_BUF_LEN];

// flash variables
uint32_t flash_data[] = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000};
__IO uint32_t flash_rx_data[5];


// ADS variables
ADS8688 ads;
uint16_t ads_data[8];
float volt_helper = 0;
int volt[8] = {0};

// tcp variables
struct tcp_pcb *pcb[numofports][numofclients];
volatile uint8_t accepted_pcb[numofports][numofclients];

#if ETH_TCP_DEBUG
struct tcp_pcb *debug_pcb;
uint8_t debug_port_accepted=0;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void print(char *msg, ...);
extern uint32_t get_my_ip();
extern uint32_t get_my_netmask();
extern uint32_t get_my_gateway();

err_t my_tcp_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
void my_tcp_init(struct tcp_pcb *pcb, uint16_t port);
void tcp_send_all();

void Print_PHY_Registers();

void CAN1_Tx(uint32_t ID, uint8_t dlc, uint8_t* data);
void CAN1_Rx();
void CAN2_Tx(uint32_t ID, uint8_t dlc,  uint8_t* data);
void CAN2_Rx();

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// CGI & SSI Related Variables and functions
// every time you want a new tag in HTML you have to increase the numSSItags and add the tag string inside theSSItags array
#define numSSItags 5
char const *theSSItags[numSSItags] = {"dhcp_status", "ip", "mask", "gw", "dhcp_box"};

// every time you want a new CGI handler, you have to increase the numCGIhandlers, create an new tCGI variable that states the .cgi and the handler function
// Also don't forget to add the new tCGI variable to theCGItable table inside myCGIinit function
#define numCGIhandlers 1
tCGI theCGItable[numCGIhandlers];
const char* NetCGIhandler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const tCGI NetCGI = { "/net.cgi", NetCGIhandler };
void myCGIinit(void);
u16_t mySSIHandler(int iIndex, char *pcInsert, int iInsertLen);
void mySSIinit(void);
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_Delay(200);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);

  httpd_init();
  myCGIinit();
  mySSIinit();


  Flash_Read_Data(START_F_ADDRESS_IP, flash_rx_data, 5);
  print("Flash variables:\n");
  for(uint8_t i=0; i<5; i++) {
	  print("%lu\n",flash_rx_data[i]);
	  flash_data[i] = flash_rx_data[i];
  }
  flash_data[0]++;
//  dhcp_en = (0 ? (flash_data[1] & 0x0001)==1 : 1);
  if((flash_data[1] & 0x0001) == 1) {
	  dhcp_en = 0;
  }
  Flash_Write_Data(START_F_ADDRESS_IP, flash_data, 5);

  for(uint8_t i=0; i<numofports; i++) {
	  for(uint8_t j=0; j<numofclients; j++) {
		  my_tcp_init(pcb[i][j], startport + i);
	  }
  }
#if ETH_TCP_DEBUG
  my_tcp_init(debug_pcb, TCP_DEBUG_PORT);
#endif

  ADS8688_Init(&ads, &hspi3, SPI3_CS_GPIO_Port, SPI3_CS_Pin);

  HAL_GPIO_WritePin(E6_GPIO_Port, E6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(WARN_GPIO_Port, WARN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IP_GPIO_Port, IP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RUN_GPIO_Port, RUN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ERR_GPIO_Port, ERR_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(E6_GPIO_Port, E6_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(WARN_GPIO_Port, WARN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IP_GPIO_Port, IP_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RUN_GPIO_Port, RUN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ERR_GPIO_Port, ERR_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  current = HAL_GetTick();
	  MX_LWIP_Process();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(previous<current) {
		  if(get_my_ip() != 0) {
			  HAL_GPIO_WritePin(IP_GPIO_Port, IP_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_TogglePin(RUN_GPIO_Port, RUN_Pin);
		  }
		  else {
			  HAL_GPIO_TogglePin(IP_GPIO_Port, IP_Pin);
		  }

		  my_ip = get_my_ip();
		  print("IP = %lu.%lu.%lu.%lu\n",(my_ip & 0xff), ((my_ip >> 8) & 0xff), ((my_ip >> 16) & 0xff), (my_ip >> 24));
		  my_net = get_my_netmask();
		  print("SUBNET = %lu.%lu.%lu.%lu\n",(my_net & 0xff), ((my_net >> 8) & 0xff), ((my_net >> 16) & 0xff), (my_net >> 24));
		  my_gw = get_my_gateway();
		  print("GATE = %lu.%lu.%lu.%lu\n",(my_gw & 0xff), ((my_gw >> 8) & 0xff), ((my_gw >> 16) & 0xff), (my_gw >> 24));

//		  Print_PHY_Registers();

		  ADS_Read_All_Raw(&ads, ads_data);
		  print("-----------------------------------------------------------\n");
		  for(int i=0; i<8; i++) {;
			  volt_helper = ((float)ads_data[i])*10.0/4095.0;
			  volt[i] = (int)(volt_helper*100000000);
			  print("CHN_%d: %d.%d volt\n", i, volt[i]/100000000, volt[i]%100000000);

		  }
		  print("-----------------------------------------------------------\n");

		  tcp_send_all();
		  previous = current;
		  previous+=interval;
	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 2;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ERR_Pin|WARN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IP_Pin|RUN_Pin|E6_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RS485_EN_Pin|SPI3_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ERR_Pin WARN_Pin */
  GPIO_InitStruct.Pin = ERR_Pin|WARN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : IP_Pin RUN_Pin E6_Pin */
  GPIO_InitStruct.Pin = IP_Pin|RUN_Pin|E6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : D_IN1_Pin */
  GPIO_InitStruct.Pin = D_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(D_IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : D_IN2_Pin */
  GPIO_InitStruct.Pin = D_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(D_IN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_EN_Pin */
  GPIO_InitStruct.Pin = RS485_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void tcp_send_all() {
	  for(uint8_t i=0; i<numofports; i++) {
		  char buf_out[30];
		  if(i<8) {
			  sprintf(buf_out, "%d.%d\n", volt[i]/100000000, volt[i]%100000000);
		  }
		  else {
			  sprintf(buf_out, "this is port: %d\n", 5009);
		  }
		  for(uint8_t j=0; j<numofclients; j++) {
			  if(accepted_pcb[i][j]) {

				  if( pcb[i][j] != NULL && pcb[i][j]->state == ESTABLISHED) {
					  if(tcp_write(pcb[i][j], &buf_out, strlen(buf_out)+1, TCP_WRITE_FLAG_COPY) != ERR_OK) {
						  print("----------- FAIL: write did not return ok\n");
					  }
					  else {
						  tcp_output(pcb[i][j]);
						  print(buf, "+++++++   WRITING on port: %d conn %d\n", startport + i, j+1);
					  }
				  }
				  else {
					  print("----------- Connection closed on port: %d conn %d\n", startport + i, j+1);
					  tcp_close(pcb[i][j]);
					  accepted_pcb[i][j] = 0;
				  }
			  }
		  }

	  }
}

const char* NetCGIhandler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]) {
	int offset = 0;
	if (strcmp(pcParam[0], "dhcp") == 0) {
		offset = 1;
		if (strcmp(pcValue[0], "1") == 0) { // submitted checked
			if(!dhcp_en) { 	// this means that dhcp was disabled and we want to enable it
				print("---------------------------------------------------\n");
				print(" -Got request to enable DHCP\n");
				flash_data[1] = 0;
				Flash_Write_Data(START_F_ADDRESS_IP, flash_data, 5);
				NVIC_SystemReset();
			}
		}
	}
	if(strcmp(pcParam[0+offset], "ip_1") == 0 && !offset) {
		if(iNumParams >= 12) {																	// NOT SURE IF NEDDED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			print("---------------------------------------------------\n");
			print(" -Got request to change network setting to:\n");

			for(int i = 0; i < iNumParams-offset; i++) {
				print("%s: %s\n", pcParam[i+offset], pcValue[i+offset]);
			}
			print("---------------------------------------------------\n");

			flash_data[1] = 1;
			flash_data[2] = (uint32_t)(atoi(pcValue[0+offset])<<0) + (uint32_t)(atoi(pcValue[1+offset])<<8) + (uint32_t)(atoi(pcValue[2+offset])<<16) + (uint32_t)(atoi(pcValue[3+offset])<<24);
			flash_data[3] = (uint32_t)(atoi(pcValue[4+offset])<<0) + (uint32_t)(atoi(pcValue[5+offset])<<8) + (uint32_t)(atoi(pcValue[6+offset])<<16) + (uint32_t)(atoi(pcValue[7+offset])<<24);
			flash_data[4] = (uint32_t)(atoi(pcValue[8+offset])<<0) + (uint32_t)(atoi(pcValue[9+offset])<<8) + (uint32_t)(atoi(pcValue[10+offset])<<16) + (uint32_t)(atoi(pcValue[11+offset])<<24);
			Flash_Write_Data(START_F_ADDRESS_IP, flash_data, 5);
			NVIC_SystemReset();
		}
		else {
			print(" -Something went wrong with the parameters...they are less than 11\n");
		}

	}
	return "/index.shtml";
}

void myCGIinit(void) {
    //add LED control CGI to the table
    theCGItable[0] = NetCGI;
    //give the table to the HTTP server
    http_set_cgi_handlers(theCGItable, numCGIhandlers);
}

u16_t mySSIHandler(int iIndex, char *pcInsert, int iInsertLen) {
	char *tmp;
	tmp = "";
	if (iIndex == 0) {// dhcp status
		if(dhcp_en) {
			tmp = "ON";
		}
		else {
			tmp = "OFF";
			request_static = 1;
		}
	}
	else if (iIndex == 1) {	// ip address
		my_ip = get_my_ip();
		char tmp1[50];
		sprintf(tmp1, "%lu.%lu.%lu.%lu",(my_ip & 0xff), ((my_ip >> 8) & 0xff), ((my_ip >> 16) & 0xff), (my_ip >> 24));
		strcpy(pcInsert, tmp1);
		return strlen(tmp1);
	}
	else if (iIndex == 2) {	// subnet mask
		my_ip = get_my_netmask();
		char tmp1[50];
		sprintf(tmp1, "%lu.%lu.%lu.%lu",(my_net & 0xff), ((my_net >> 8) & 0xff), ((my_net >> 16) & 0xff), (my_net >> 24));
		strcpy(pcInsert, tmp1);
		return strlen(tmp1);
	}
	else if (iIndex == 3) {	// default gateway
		my_ip = get_my_gateway();
		char tmp1[50];
		sprintf(tmp1, "%lu.%lu.%lu.%lu",(my_gw & 0xff), ((my_gw >> 8) & 0xff), ((my_gw >> 16) & 0xff), (my_gw >> 24));
		strcpy(pcInsert, tmp1);
		return strlen(tmp1);
	}
	else if (iIndex == 4) {	// dhcp checkbox
		if (dhcp_en) {
			tmp = "<input value=\"1\" name=\"dhcp\" type=\"checkbox\" checked>";
		}
		else {
			tmp ="<input value=\"1\" name=\"dhcp\" type=\"checkbox\">";
		}
	}
//	else if (iIndex == 5) {	// static ip menu
//		if (request_static) {
////			HAL_Delay(600);
//			tmp = "  Select your settings and press Submit order to get static address<br><form method=\"get\" action=\"/net.cgi\"> IP Address: <input type=\"text\" name=\"ip_1\" value=\"192\" maxlength=\"3\" size=\"1\">. <input type=\"text\" name=\"ip_2\" value=\"168\" maxlength=\"3\" size=\"1\">. <input type=\"text\" name=\"ip_3\" value=\"1\" maxlength=\"3\" size=\"1\">. <input type=\"text\" name=\"ip_4\" value=\"20\" maxlength=\"3\" size=\"1\"><br> Subnet Mask: <input type=\"text\" name=\"net_1\" value=\"255\" maxlength=\"3\" size=\"1\">. <input type=\"text\" name=\"net_2\" value=\"255\" maxlength=\"3\" size=\"1\">. <input type=\"text\" name=\"net_3\" value=\"255\" maxlength=\"3\" size=\"1\">. <input type=\"text\" name=\"net_4\" value=\"0\" maxlength=\"3\" size=\"1\"><br> Default Gateway: <input type=\"text\" name=\"gw_1\" value=\"192\" maxlength=\"3\" size=\"1\">. <input type=\"text\" name=\"gw_2\" value=\"168\" maxlength=\"3\" size=\"1\">. <input type=\"text\" name=\"gw_3\" value=\"1\" maxlength=\"3\" size=\"1\">. <input type=\"text\" name=\"gw_4\" value=\"1\" maxlength=\"3\" size=\"1\"><br> <input value=\"Submit\" type=\"submit\"></form>";
//		}
//		else {
//			tmp = "";
//		}
//	}
	else {
		tmp = "";
	}
	strcpy(pcInsert, tmp);
	return strlen(tmp);
}

void mySSIinit(void) {
	http_set_ssi_handler(mySSIHandler, (char const**) theSSItags,numSSItags);
}

void CAN1_Tx(uint32_t ID, uint8_t dlc, uint8_t* data) {

	uint32_t TxMailbox;

	TxHeader1.DLC = dlc;
	TxHeader1.StdId = ID;
	TxHeader1.IDE = CAN_ID_STD;
	TxHeader1.RTR = CAN_RTR_DATA;

	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, data, &TxMailbox) != HAL_OK) {
		//Error_Handler();
		print("Failed to Add Message can 1\n");
	}
	print("Sending Message to can 1\n");
	while(HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox));
	print("Message Sent to can 1\n");

}

void CAN2_Tx(uint32_t ID, uint8_t dlc,  uint8_t* data) {

	uint32_t TxMailbox;

	TxHeader2.DLC = dlc;
	TxHeader2.StdId = ID;
	TxHeader2.IDE = CAN_ID_STD;
	TxHeader2.RTR = CAN_RTR_DATA;

	if(HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, data, &TxMailbox) != HAL_OK) {
		//Error_Handler();
		print("Failed to Add Message can 2\n");
	}
	print("Sending Message to can 2\n");
	while(HAL_CAN_IsTxMessagePending(&hcan2, TxMailbox));
	print("Message Sent to can 2\n");
}

void CAN1_Rx() {
	 HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader1, r1);

	 //dont forget to add and enable filters

	 switch(RxHeader1.StdId) {

	 case 0x000 :

		 break;
	 }

}

void CAN2_Rx() {
	 HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader2, r2);

	 //dont forget to add and enable filters

	 switch(RxHeader2.StdId) {

	 case 0x000 :

		 break;
	 }

}
err_t my_tcp_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(err);

    tcp_setprio(newpcb, TCP_PRIO_NORMAL);
    tcp_recv(newpcb, NULL);
    tcp_err(newpcb, NULL);
    tcp_poll(newpcb, NULL, 4);
    uint16_t port = newpcb->local_port;
    print("+++++ ACCEPTED  on port: %u", port);
#if ETH_TCP_DEBUG
    if(port == TCP_DEBUG_PORT && !debug_port_accepted) {
    	debug_port_accepted = 1;
    	debug_pcb = newpcb;
    	return ERR_OK;
    }
#endif

    uint16_t index = port % startport;
    for(uint8_t i=0; i<numofclients; i++) {
    	if(!accepted_pcb[index][i]) {
    		pcb[index][i] = newpcb;
    		accepted_pcb[index][i] = 1;
    		return ERR_OK;
    	}
    }
    print("Something went wrong while connecting on port: %u",  port);
    print("Max 3 connections at a time per port are allowed..Ignoring request\n");
    return ERR_OK;
}

void my_tcp_init(struct tcp_pcb *pcb, uint16_t port) {
/* create new tcp pcb */
	pcb = tcp_new();
	if (pcb != NULL) {
		err_t err;
		/* bind echo_pcb to port 7 (ECHO protocol) */
		err = tcp_bind(pcb, IP_ADDR_ANY, port);
		if (err == ERR_OK) {
			/* start tcp listening for echo_pcb */
			pcb = tcp_listen(pcb);
			/* initialize LwIP tcp_accept callback function */
			tcp_accept(pcb, my_tcp_accept);

			// check for tcp_accepted()
		}
		else {
			/* deallocate the pcb */
			memp_free(MEMP_TCP_PCB, pcb);
		}
	}
}

void Print_PHY_Registers() {
	print("-----------------------------------------------------------\n");
	HAL_ETH_ReadPHYRegister(&heth, PHY_BCR, &phyreg);
	print("BMCR: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &phyreg);
	print("BMSR: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	HAL_ETH_ReadPHYRegister(&heth, 0x04, &phyreg);
	print("ANAR: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	HAL_ETH_ReadPHYRegister(&heth, 0x05, &phyreg);
	print("ANLPAR: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	HAL_ETH_ReadPHYRegister(&heth, 0x06, &phyreg);
	print("ANER: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	HAL_ETH_ReadPHYRegister(&heth, 0x07, &phyreg);
	print("ANNPTR: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	HAL_ETH_ReadPHYRegister(&heth, PHY_SR, &phyreg);
	print("PHYSTS(SR): "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	HAL_ETH_ReadPHYRegister(&heth, 0x14, &phyreg);
	print("FCSCR: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	HAL_ETH_ReadPHYRegister(&heth, 0x15, &phyreg);
	print("RECR: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	HAL_ETH_ReadPHYRegister(&heth, 0x16, &phyreg);
	print("PCSR: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	HAL_ETH_ReadPHYRegister(&heth, 0x17, &phyreg);
	print("RBR: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	HAL_ETH_ReadPHYRegister(&heth, 0x18, &phyreg);
	print("LEDCR: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	HAL_ETH_ReadPHYRegister(&heth, 0x19, &phyreg);
	print("PHYCR: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	HAL_ETH_ReadPHYRegister(&heth, 0x1A, &phyreg);
	print("10BTSCR: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	HAL_ETH_ReadPHYRegister(&heth, 0x1B, &phyreg);
	print("CDCTRL1: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	HAL_ETH_ReadPHYRegister(&heth, 0x1D, &phyreg);
	print("EDCR: "BYTE_TO_BIN_PAT" "BYTE_TO_BIN_PAT"      %#.4X\n", BYTE_TO_BIN(phyreg>>8), BYTE_TO_BIN(phyreg), phyreg);
	print("-----------------------------------------------------------\n");
}
void print(char *msg, ...) {

	char buff[100];
	va_list args;
	va_start(args, msg);
	vsprintf(buff,msg,args);
#if USB_DEBUG
	HAL_UART_Transmit(&huart2, (uint8_t *)buff, strlen(buff), 10);
#endif
#if ETH_TCP_DEBUG
	 if(debug_port_accepted) {
		  if( debug_pcb != NULL && debug_pcb->state == ESTABLISHED) {
			  if(tcp_write(debug_pcb, &buff, strlen(buff)+1, TCP_WRITE_FLAG_COPY) != ERR_OK) {
				  // something went wrong...
			  }
			  else {
				  tcp_output(debug_pcb);
			  }
		  }
		  else {
			  tcp_close(debug_pcb);
			  debug_port_accepted = 0;
		  }
	 }
#endif
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
