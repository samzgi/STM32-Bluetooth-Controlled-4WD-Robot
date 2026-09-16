/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define WAYPOINT_COUNT 5

#define PWM_MIN 200
#define PWM_MAX 950
#define PWM_STEP 50

#define GPS_BUFFER_SIZE 128

#define KP_HEADING  2.0f      // Dönüş hassasiyeti
#define BASE_SPEED  300       // Otonom temel hız
#define TURN_LIMIT  250       // Maks dönüş farkı

float current_heading = 0.0f;
float last_valid_heading = 0.0f;

#define HEADING_LOCK_SPEED 3.0f


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

typedef enum {
    MODE_MANUAL,
    MODE_AUTO
} ControlMode;

typedef enum {
    AUTO_IDLE,
    AUTO_ROTATE,
    AUTO_FORWARD,
    AUTO_ARRIVED
} AutoState;

float waypoints[WAYPOINT_COUNT][2] = {
    {38.396147f, 27.162712f},   // WP1
    {38.396300f, 27.163000f},   // WP2
    {38.396450f, 27.163200f},   // WP3
    {38.396600f, 27.163350f},   // WP4
    {38.396750f, 27.163500f}    // WP5
};


uint8_t current_wp_index = 0;

volatile ControlMode mode = MODE_MANUAL;
volatile AutoState auto_state = AUTO_IDLE;

// Temel sürüş değişkenleri
volatile uint8_t ileri_basili = 0;
volatile uint8_t geri_basili = 0;
volatile uint16_t current_speed = 500;
char msg_buffer[250]; // Global buffer

#define BT_BUFFER_SIZE 64
char bt_buffer[BT_BUFFER_SIZE];
uint8_t bt_index = 0;
uint8_t bt_rx_char;

// Gps değişkenleri
volatile uint8_t gps_rx_char;
char gps_buffer[GPS_BUFFER_SIZE];
volatile uint8_t gps_index = 0;
volatile uint8_t gps_line_ready = 0;

float latitude = 0.0f;
float longitude = 0.0f;
uint8_t gps_fix = 0;

float speed_kmh = 0.0f;
float course_deg = 0.0f;
uint8_t rmc_valid = 0;

// Auto Mode değişkenleri
float target_lat = 0.0f;
float target_lon = 0.0f;
uint8_t target_set = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

void Motor_Stop(void);
void Motor_Forward(void);
void Motor_Backward(void);
void Motor_Left(void);
void Motor_Right(void);
void Motor_Speed_Up(void);
void Motor_Speed_Down(void);
void GPS_GetLocation(void);
void GPS_Parse(void);
float convert_to_decimal(float raw);
void Parse_BT_Command(char *cmd);
float calculate_distance(float lat1, float lon1, float lat2, float lon2);
float calculate_bearing(float lat1, float lon1, float lat2, float lon2);
void Auto_Navigate(void);
void Motor_Forward_Diff(int left_pwm, int right_pwm);
void Merhaba(void);
void Hatali_giris(void);
void GPS_Save_Config(void);
void GPS_Enable_SBAS(void);

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_UART_Receive_IT(&huart2, &bt_rx_char, 1);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&gps_rx_char, 1);

  HAL_Delay(1000);          // GPS açılması için bekle
  GPS_Enable_SBAS();        // SBAS aç
  HAL_Delay(200);
  GPS_Save_Config();        // Flash'a kaydet
  HAL_Delay(200);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

	    if (gps_line_ready)
	    {
	        gps_line_ready = 0;
	        GPS_Parse();
	    }

	    if (mode == MODE_AUTO)
	    {
	        Auto_Navigate();
	    }

	    HAL_Delay(20);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|arka_sol_in2_Pin|arka_sag_in2_Pin|on_sol_in2_Pin
                          |on_sag_in2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |on_sag_in1_Pin|on_sol_in1_Pin|arka_sag_in1_Pin|arka_sol_in1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin arka_sol_in2_Pin arka_sag_in2_Pin on_sol_in2_Pin
                           on_sag_in2_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|arka_sol_in2_Pin|arka_sag_in2_Pin|on_sol_in2_Pin
                          |on_sag_in2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           on_sag_in1_Pin on_sol_in1_Pin arka_sag_in1_Pin arka_sol_in1_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |on_sag_in1_Pin|on_sol_in1_Pin|arka_sag_in1_Pin|arka_sol_in1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        char cmd[2];
        cmd[0] = bt_rx_char;
        cmd[1] = '\0';

        Parse_BT_Command(cmd);

        HAL_UART_Receive_IT(&huart2, &bt_rx_char, 1);
    }

    else if (huart->Instance == USART1)
    {
        if (gps_rx_char == '\n')
        {
            gps_buffer[gps_index] = '\0';
            gps_index = 0;
            gps_line_ready = 1;
        }
        else
        {
            if (gps_index < GPS_BUFFER_SIZE - 1)
                gps_buffer[gps_index++] = gps_rx_char;
            else
                gps_index = 0;
        }

        HAL_UART_Receive_IT(&huart1, (uint8_t *)&gps_rx_char, 1);
    }
}


void GPS_Parse(void)
{
    /* ----------- GGA ----------- */
	if (strncmp(gps_buffer, "$GPGGA", 6) == 0 ||
	    strncmp(gps_buffer, "$GNGGA", 6) == 0)
	{
	    char temp_buffer[GPS_BUFFER_SIZE];
	    strcpy(temp_buffer, gps_buffer);

	    char *token;
	    uint8_t field = 0;

	    float raw_lat = 0.0f, raw_lon = 0.0f;
	    char lat_dir = 'N', lon_dir = 'E';

	    uint8_t local_fix = 0;   // artık gerçekten kullanacağız

	    token = strtok(temp_buffer, ",");

	    while (token != NULL)
	    {
	        field++;

	        if (field == 3) raw_lat = atof(token);
	        if (field == 4) lat_dir = token[0];
	        if (field == 5) raw_lon = atof(token);
	        if (field == 6) lon_dir = token[0];

	        if (field == 7 && token[0] != '0')
	            local_fix = 1;   // sadece burada set et

	        token = strtok(NULL, ",");
	    }

	    if (local_fix)
	    {
	        gps_fix = 1;

	        latitude  = convert_to_decimal(raw_lat);
	        longitude = convert_to_decimal(raw_lon);

	        if (lat_dir == 'S') latitude = -latitude;
	        if (lon_dir == 'W') longitude = -longitude;
	    }
	}


    /* ----------- RMC ----------- */
	else if (strncmp(gps_buffer, "$GPRMC", 6) == 0 ||
	         strncmp(gps_buffer, "$GNRMC", 6) == 0)
    {
        char temp_buffer[GPS_BUFFER_SIZE];
        strcpy(temp_buffer, gps_buffer);

        char *token;
        uint8_t field = 0;

        uint8_t local_rmc_valid = 0;   // GEÇİCİ

        token = strtok(temp_buffer, ",");

        while (token != NULL)
        {
            field++;

            if (field == 3 && token[0] == 'A')
            	local_rmc_valid = 1;   // sadece burada set et

            if (field == 8)
                speed_kmh = atof(token) * 1.852f;

            if (field == 9)
            {
                float raw_course = atof(token);

                if (speed_kmh >= HEADING_LOCK_SPEED)
                {
                    current_heading = raw_course;
                    last_valid_heading = raw_course;
                }
                else
                {
                    current_heading = last_valid_heading;
                }
            }


            token = strtok(NULL, ",");
        }

        // PARSE BITTIKTEN SONRA GUNCELLE
        if (local_rmc_valid)
            rmc_valid = 1;
    }
}

void Parse_BT_Command(char *cmd)
{
    /* ========== EMERGENCY STOP (HER ZAMAN) ========== */
    if (cmd[0] == 'S')
    {
        Motor_Stop();
        mode = MODE_MANUAL;
        auto_state = AUTO_IDLE;
        target_set = 0;

        HAL_UART_Transmit(&huart2,
            (uint8_t*)"ACIL DURDURMA!\r\n", 15, 100);
        return;
    }

    // AUTO moddayken diger manuel komutlari dinleme
    if (mode == MODE_AUTO)
        return;

    /* ========== AUTO MODE KOMUTLARI ========== */

    // === WAYPOINT 1 ===
    // === SAYI ILE WAYPOINT SECIMI ===
    if (cmd[0] >= '1' && cmd[0] <= '5')
    {
        uint8_t index = cmd[0] - '1';   // '1' -> 0, '2' -> 1 ...

        if (index < WAYPOINT_COUNT)
        {
            current_wp_index = index;

            target_lat = waypoints[index][0];
            target_lon = waypoints[index][1];

            target_set = 1;
            mode = MODE_AUTO;

            char msg[40];
            int len = sprintf(msg, "WP%d'e Gidiliyor\r\n", index + 1);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
        }

        return;
    }

    /* ========== MANUAL MODE KOMUTLARI ========== */
    if (mode != MODE_MANUAL)
        return;   // AUTO moddaysak manueli dinleme

    switch (cmd[0])
    {
        case 'F':
            Motor_Forward();
            break;

        case 'B':
            Motor_Backward();
            break;

        case 'L':
            Motor_Left();
            break;

        case 'R':
            Motor_Right();
            break;

        case 'U':
            Motor_Speed_Up();
            break;

        case 'D':
            Motor_Speed_Down();
            break;

        case 'G':
            GPS_GetLocation();
            break;

        case 'M':
			Merhaba();
        	break;

        default:
        	Hatali_giris();
            break;
    }
}


float calculate_distance(float lat1, float lon1, float lat2, float lon2)
{
    float R = 6371000.0f; // metre
    float dLat = (lat2 - lat1) * M_PI / 180.0f;
    float dLon = (lon2 - lon1) * M_PI / 180.0f;

    lat1 *= M_PI / 180.0f;
    lat2 *= M_PI / 180.0f;

    float a = sin(dLat/2)*sin(dLat/2) +
              cos(lat1)*cos(lat2) *
              sin(dLon/2)*sin(dLon/2);

    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

float calculate_bearing(float lat1, float lon1, float lat2, float lon2)
{
    float y = sin((lon2 - lon1) * M_PI / 180.0f) * cos(lat2 * M_PI / 180.0f);
    float x = cos(lat1 * M_PI / 180.0f) * sin(lat2 * M_PI / 180.0f) -
              sin(lat1 * M_PI / 180.0f) * cos(lat2 * M_PI / 180.0f) *
              cos((lon2 - lon1) * M_PI / 180.0f);

    float brng = atan2(y, x) * 180.0f / M_PI;
    if (brng < 0) brng += 360.0f;
    return brng;
}

void Auto_Navigate(void)
{
	if (!gps_fix || !rmc_valid)
	{
	    if (mode == MODE_AUTO)
	        Motor_Stop();
	    return;
	}

    if (!target_set)
        return;

    float distance = calculate_distance(
        latitude, longitude,
        target_lat, target_lon);

    // === HEDEFE ULASILDI ===
    if (distance < 2.5f)
    {
        Motor_Stop();
        target_set = 0;
        mode = MODE_MANUAL;

        char msg[40];
        int len = sprintf(msg, "WP%d Ulasildi\r\n", current_wp_index + 1);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
        return;
    }


    // === HEDEF YÖNÜ ===
    float target_bearing = calculate_bearing(
        latitude, longitude,
        target_lat, target_lon);

    // === AÇI HATASI ===
    float diff = target_bearing - current_heading;

    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;

    // === P KONTROL ===
    float turn = KP_HEADING * diff;

    // Sınırla
    if (turn > TURN_LIMIT)  turn = TURN_LIMIT;
    if (turn < -TURN_LIMIT) turn = -TURN_LIMIT;

    // === DIFERANSIYEL HIZ ===
    int dynamic_speed = BASE_SPEED;

    if (distance < 10.0f)
        dynamic_speed = 200;

    if (distance < 5.0f)
        dynamic_speed = 150;

    int left_speed  = dynamic_speed - turn;
    int right_speed = dynamic_speed + turn;

    // === ARAÇ GİBİ İLERLE ===
    Motor_Forward_Diff(left_speed, right_speed);
}

float convert_to_decimal(float raw)
{
    int deg = (int)(raw / 100);
    float min = raw - (deg * 100);
    return deg + (min / 60.0f);
}

void Motor_Stop(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

	// Tüm yön pinlerini sıfırla
	HAL_GPIO_WritePin(GPIOD, on_sag_in1_Pin | on_sol_in1_Pin | arka_sag_in1_Pin | arka_sol_in1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, on_sag_in2_Pin | on_sol_in2_Pin | arka_sag_in2_Pin | arka_sol_in2_Pin, GPIO_PIN_RESET);
}

void Motor_Forward(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current_speed);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, current_speed);

	// Sol ve Sağ motorlar İLERİ
	HAL_GPIO_WritePin(GPIOD, on_sag_in1_Pin | on_sol_in1_Pin | arka_sag_in1_Pin | arka_sol_in1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, on_sag_in2_Pin | on_sol_in2_Pin | arka_sag_in2_Pin | arka_sol_in2_Pin, GPIO_PIN_RESET);
}

void Motor_Backward(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current_speed);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, current_speed);
	// Sol ve Sağ motorlar GERİ
	HAL_GPIO_WritePin(GPIOD, on_sag_in1_Pin | on_sol_in1_Pin | arka_sag_in1_Pin | arka_sol_in1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, on_sag_in2_Pin | on_sol_in2_Pin | arka_sag_in2_Pin | arka_sol_in2_Pin, GPIO_PIN_SET);
}

void Motor_Left(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current_speed);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, current_speed);
	// Sağ motorlar İLERİ
	HAL_GPIO_WritePin(GPIOD, on_sag_in1_Pin | arka_sag_in1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, on_sag_in2_Pin | arka_sag_in2_Pin, GPIO_PIN_RESET);
	// Sol motorlar DUR
	HAL_GPIO_WritePin(GPIOD, on_sol_in1_Pin | arka_sol_in1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, on_sol_in2_Pin | arka_sol_in2_Pin, GPIO_PIN_RESET);
}

void Motor_Right(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current_speed);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, current_speed);
	// Sol motorlar İLERİ
	HAL_GPIO_WritePin(GPIOD, on_sol_in1_Pin | arka_sol_in1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, on_sol_in2_Pin | arka_sol_in2_Pin, GPIO_PIN_RESET);
	// Sağ motorlar DUR
	HAL_GPIO_WritePin(GPIOD, on_sag_in1_Pin | arka_sag_in1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, on_sag_in2_Pin | arka_sag_in2_Pin, GPIO_PIN_RESET);
}

void Motor_Speed_Up(void)
{
	if (current_speed + PWM_STEP <= PWM_MAX)
		current_speed += PWM_STEP;
	else
		current_speed = PWM_MAX;
	int display_speed = current_speed / 10;
	// Mesaj formatını sadeleştirelim
	memset(msg_buffer, 0, sizeof(msg_buffer)); // Buffer'ı temizle
	int len = sprintf(msg_buffer, "Hız: %d%%\r\n", display_speed);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buffer, len, 100);
}

void Motor_Speed_Down(void)
{
	if (current_speed >= PWM_MIN + PWM_STEP)
		current_speed -= PWM_STEP;
	else
		current_speed = PWM_MIN;
	int display_speed = current_speed / 10;
	memset(msg_buffer, 0, sizeof(msg_buffer));
	int len = sprintf(msg_buffer, "Hız: %d%%\r\n", display_speed);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_buffer, len, 100);
}

void GPS_GetLocation(void)
{
	 if (gps_fix && rmc_valid)
	    {
	        int len = sprintf(msg_buffer,
	            "Lat: %.6f\r\n"
	            "Lon: %.6f\r\n"
	            "Hiz: %.2f km/h\r\n"
	            "Yon: %.1f deg\r\n\r\n"
	            "https://maps.google.com/?q=%.6f,%.6f\r\n",
	            latitude, longitude,
				speed_kmh, current_heading,
	            latitude, longitude);

	        HAL_UART_Transmit(&huart2,
	            (uint8_t*)msg_buffer,
	            len,
	            300);
	    }
	    else
	    {
	        HAL_UART_Transmit(&huart2,
	            (uint8_t*)"GPS VERISI YOK\r\n",
	            15,
	            200);
	    }
}

void Motor_Forward_Diff(int left_pwm, int right_pwm)
{
    // Limitler
    if (left_pwm < 0) left_pwm = 0;
    if (right_pwm < 0) right_pwm = 0;
    if (left_pwm > PWM_MAX) left_pwm = PWM_MAX;
    if (right_pwm > PWM_MAX) right_pwm = PWM_MAX;

    // İLERİ yön
    HAL_GPIO_WritePin(GPIOD,
        on_sag_in1_Pin | on_sol_in1_Pin |
        arka_sag_in1_Pin | arka_sol_in1_Pin,
        GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOE,
        on_sag_in2_Pin | on_sol_in2_Pin |
        arka_sag_in2_Pin | arka_sol_in2_Pin,
        GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, left_pwm);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, right_pwm);
}

void Merhaba(void)
{
	HAL_UART_Transmit(&huart2,
	            (uint8_t*)"Merhaba!\r\n",
	            12, 200);
}

void Hatali_giris(void)
{
	HAL_UART_Transmit(&huart2,
	            (uint8_t*)"Hatalı giriş yaptınız! Lütfen tekrar deneyiniz.\r\n",
	            50, 200);
}

void GPS_Enable_SBAS(void)
{
    uint8_t sbas_cmd[] = {
        0xB5,0x62,0x06,0x16,0x08,0x00,
        0x01,0x03,0x03,0x00,
        0x51,0x08,0x00,0x00,
        0x8F,0x4A
    };

    HAL_UART_Transmit(&huart1, sbas_cmd, sizeof(sbas_cmd), 200);
}

void GPS_Save_Config(void)
{
    uint8_t save_cmd[] = {
        0xB5,0x62,0x06,0x09,0x0D,0x00,
        0x00,0x00,0x00,0x00,
        0xFF,0xFF,0x00,0x00,
        0x00,0x00,0x00,0x00,
        0x17,0x31
    };

    HAL_UART_Transmit(&huart1, save_cmd, sizeof(save_cmd), 200);
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
