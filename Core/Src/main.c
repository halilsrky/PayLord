/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for SkyLord2 Flight Computer
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

/**
  ******************************************************************************
  * SkyLord2 Flight Computer - Main Application
  * 
  * This application implements a comprehensive flight computer system for rocket
  * telemetry and control, featuring:
  * 
  * SENSORS:
  * - BME280: Environmental sensor (temperature, humidity, pressure)
  * - BMI088: 6-axis IMU (3-axis accelerometer + 3-axis gyroscope)
  * - HMC1021: Single-axis magnetometer for magnetic field measurement
  * - L86 GPS/GNSS: Position and navigation data
  * 
  * COMMUNICATION:
  * - E22 LoRa module: Long-range wireless telemetry transmission
  * - UART: Debug and telemetry output
  * 
  * PROCESSING:
  * - Real-time sensor fusion using quaternion-based algorithms
  * - Kalman filtering for noise reduction
  * - Flight state estimation and control
  * 
  * TIMING:
  * - 100ms periodic operations for data acquisition
  * - 1s periodic operations for magnetometer calibration
  * - Continuous sensor monitoring and data fusion
  * 
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/*==================== PROJECT CONFIGURATION ====================*/
#include "configuration.h"

/*==================== STANDARD LIBRARIES ====================*/
#include <string.h>
#include <stdio.h>
#include <math.h>

/*==================== SENSOR LIBRARIES ====================*/
#include "bme280.h"          // Environmental sensor (temp, humidity, pressure)
#include "bmi088.h"          // 6-axis IMU (accelerometer + gyroscope)
#include "l86_gnss.h"        // GPS/GNSS module

/*==================== COMMUNICATION LIBRARIES ====================*/
#include "lora.h"            // LoRa wireless communication

/*==================== ALGORITHM LIBRARIES ====================*/
#include "queternion.h"      // Quaternion mathematics
#include "sensor_fusion.h"   // Sensor fusion algorithms

/*==================== MATHEMATICAL CONSTANTS ====================*/
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define IMU_I2C_HNDLR	hi2c1 //put your I2C handler
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */

/*==================== SENSOR STRUCTURES ====================*/
// Environmental sensor (BME280) - temperature, humidity, pressure
static BME_280_t BME280_sensor;
BME_parameters_t bme_params;

// Inertial measurement unit (BMI088) - accelerometer and gyroscope
bmi088_struct_t BMI_sensor;

// Sensor fusion output data structure
sensor_fusion_t sensor_output;

// GPS/GNSS data structure
gps_data_t gnss_data;

// LoRa communication module structure
static lorastruct e22_lora;

/*==================== COMMUNICATION BUFFERS ====================*/
// UART communication buffers
uint8_t usart1_rx_buffer[36];
static char uart_buffer[128];

/*==================== TIMING AND STATUS FLAGS ====================*/
// Timer flags for periodic operations
volatile uint8_t tx_timer_flag_100ms = 0;
volatile uint8_t tx_timer_flag_1s = 0;

// Communication status flags
volatile uint8_t usart1_packet_ready = 0;
volatile uint16_t usart1_packet_size = 0;
volatile uint8_t usart1_tx_busy = 0;

// ADC conversion status flags
volatile uint8_t adc_conversion_complete = 0;
volatile uint8_t adc_timer_flag_100ms = 0;
volatile uint8_t sr_in_timer_flag_1s = 0;

/*==================== SENSOR STATUS VARIABLES ====================*/
// Sensor initialization and health status
int is_BME_ok = 0;
int is_BMI_ok = 0;
int bmi_status_ok = 0;
uint32_t lastUpdate = 0;

// External configuration variables
extern uint8_t Gain;
extern uint8_t gyroOnlyMode;

/*==================== ADC BUFFERS AND MAGNETOMETER DATA ====================*/
// General ADC buffers
uint16_t adc1_buffer[1];
uint16_t adc2_buffer[1];
float current_mA = 0.0f;
float voltage_V = 0.0f;

// HMC1021 magnetometer (single axis) data
volatile uint16_t hmc1021_adc_buffer[1];
float hmc1021_voltage = 0.0f;
float hmc1021_gauss = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
static void bme280_begin();
uint8_t bmi_imu_init(void);
static void loraBegin();
void read_value();
void read_ADC(void);
void trigger_sr_in_pulse(void);
static void L86_GPIO_Init(void);             // Initialize L86 GPS GPIO pins

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

	/*==================== TIMER AND INTERRUPT CONFIGURATION ====================*/
	// Initialize and start timer for periodic operations (100ms intervals)
	MX_TIM2_Init();
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	// Configure external interrupt priorities for sensor data ready signals
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 1);
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 1);

	// Enable external interrupts for sensor data ready signals
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);


	/* ==== SENSOR INITIALIZATION ==== */
	// Initialize BME280 sensor (temperature, humidity, pressure)
	bme280_begin();
	HAL_Delay(1000);
	bme280_config();
	bme280_update();

	// Initialize BMI088 IMU (accelerometer and gyroscope)
	bmi_imu_init();
	bmi088_config(&BMI_sensor);
	//get_offset(&BMI_sensor);

	// Initialize HMC1021 ADC DMA for single-axis magnetometer readings
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)hmc1021_adc_buffer, 1);


	/*==================== SENSOR FUSION INITIALIZATION ====================*/
	// Initialize quaternion-based sensor fusion
	getInitialQuaternion();
	sensor_fusion_init(&BME280_sensor);

	/* ==== LORA COMMUNICATION SETUP ==== */
	lora_deactivate();
	loraBegin();
	lora_activate();

	/* ==== GPS/GNSS INITIALIZATION ==== */
	// Initialize UART5 and DMA for GPS communication
	L86_GPIO_Init();
	HAL_Delay(50);
	HAL_DMA_Init(&hdma_usart6_rx);
	L86_GNSS_Init(&huart6, BAUD_RATE_9600);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/*CONTINUOUS SENSOR UPDATES*/
		bmi088_update(&BMI_sensor);		// Update IMU sensor data (accelerometer + gyroscope) - High frequency sampling
		bme280_update(); 		// Update barometric pressure sensor data for altitude estimation
		//read_value();	// Transmit current sensor readings


		/*PERIODIC OPERATIONS (100ms)*/
		// Execute operations every 100ms
		if (tx_timer_flag_100ms >= 1) {
		  tx_timer_flag_100ms = 0;

		  // Read magnetometer ADC values
		  //read_ADC();

		  // Update sensor readings and transmit data
		  //read_value();

		  // Update GPS/GNSS data
		  L86_GNSS_Update(&gnss_data);
		  L86_GNSS_Print_Info(&gnss_data,&huart2);
		}

		/*PERIODIC OPERATIONS (1 SECOND)*/
		// Execute operations every 1 second (10 * 100ms)
		if (tx_timer_flag_1s >= 10) {
		  tx_timer_flag_1s = 0;

		  // Trigger magnetometer set/reset pulse for calibration
		  //trigger_sr_in_pulse();

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
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  huart6.Init.BaudRate = 9600;
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
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RF_M0_Pin|RF_M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : RF_M0_Pin RF_M1_Pin */
  GPIO_InitStruct.Pin = RF_M0_Pin|RF_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Initialize BME280 environmental sensor
 * @note Configures BME280 with predefined settings for normal operation
 */
void bme280_begin()
{
  BME280_sensor.device_config.bme280_filter = BME280_FILTER_8;
  BME280_sensor.device_config.bme280_mode = BME280_MODE_NORMAL;
  BME280_sensor.device_config.bme280_output_speed = BME280_OS_8;
  BME280_sensor.device_config.bme280_standby_time = BME280_STBY_20;
  bme280_init(&BME280_sensor, &hi2c3);
}

/**
 * @brief Initialize BMI088 IMU sensor
 * @return Initialization status
 * @note Configures both accelerometer and gyroscope with optimal settings
 */
uint8_t bmi_imu_init(void)
{
  // Accelerometer configuration
  BMI_sensor.device_config.acc_bandwith = ACC_BWP_OSR4;
  BMI_sensor.device_config.acc_outputDateRate = ACC_ODR_200;
  BMI_sensor.device_config.acc_powerMode = ACC_PWR_SAVE_ACTIVE;
  BMI_sensor.device_config.acc_range = ACC_RANGE_24G;

  // Gyroscope configuration
  BMI_sensor.device_config.gyro_bandWidth = GYRO_BW_116;
  BMI_sensor.device_config.gyro_range = GYRO_RANGE_2000;
  BMI_sensor.device_config.gyro_powerMode = GYRO_LPM_NORMAL;

  // Interrupt and I2C configuration
  BMI_sensor.device_config.acc_IRQ = EXTI3_IRQn;
  BMI_sensor.device_config.gyro_IRQ = EXTI4_IRQn;
  BMI_sensor.device_config.BMI_I2c = &IMU_I2C_HNDLR;
  BMI_sensor.device_config.offsets = NULL; // Offset data stored in backup SRAM

  return bmi088_init(&BMI_sensor);
}

/**
 * @brief Initialize LoRa communication module
 * @note Configures E22 LoRa module with communication parameters
 */
void loraBegin()
{
  HAL_Delay(100);

  // Set LoRa module to configuration mode
  HAL_GPIO_WritePin(RF_M0_GPIO_Port, RF_M0_Pin, RESET);
  HAL_GPIO_WritePin(RF_M1_GPIO_Port, RF_M1_Pin, SET);
  HAL_Delay(100);

  // Configure LoRa parameters
  e22_lora.baudRate = LORA_BAUD_115200;
  e22_lora.airRate = LORA_AIR_RATE_2_4k;
  e22_lora.packetSize = LORA_SUB_PACKET_64_BYTES;
  e22_lora.power = LORA_POWER_37dbm;
  e22_lora.loraAddress.address16 = 0x0000;
  e22_lora.loraKey.key16 = 0x0000;
  e22_lora.channel = ROCKET_TELEM_FREQ;

  lora_configure(&e22_lora);
  HAL_Delay(1000);
}

/**
 * @brief Initialize L86 GPS/GNSS GPIO pins
 * @note Configures UART5 pins for GPS communication
 */
static void L86_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct_UART6_TX;
  GPIO_InitTypeDef GPIO_InitStruct_UART6_RX;

  // Configure UART5 TX pin
  GPIO_InitStruct_UART6_TX.Pin = L86_TX_Pin;
  GPIO_InitStruct_UART6_TX.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct_UART6_TX.Pull = GPIO_NOPULL;
  GPIO_InitStruct_UART6_TX.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct_UART6_TX.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(L86_TX_GPIO_Port, &GPIO_InitStruct_UART6_TX);

  // Configure UART5 RX pin
  GPIO_InitStruct_UART6_RX.Pin = L86_RX_Pin;
  GPIO_InitStruct_UART6_RX.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct_UART6_RX.Pull = GPIO_NOPULL;
  GPIO_InitStruct_UART6_RX.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct_UART6_RX.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(L86_RX_GPIO_Port, &GPIO_InitStruct_UART6_RX);
}

/**
 * @brief Read and transmit sensor values via UART
 * @note Formats and sends IMU orientation data and system parameters
 */
void read_value(){
  float yaw = BMI_sensor.datas.yaw;
  float pitch = BMI_sensor.datas.pitch;
  float roll = BMI_sensor.datas.roll;
  float yaw1 = BMI_sensor.datas.yaw1;
  float pitch1 = BMI_sensor.datas.pitch1;
  float roll1 = BMI_sensor.datas.roll1;

  // Transmit primary orientation data
  sprintf(uart_buffer, "A1 %.2f %.2f %.2f\r", yaw, pitch, roll);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

  // Transmit secondary orientation data
  sprintf(uart_buffer, "A2 %.2f %.2f %.2f\r\n", yaw1, pitch1, roll1);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

  // Transmit system gain parameter
  sprintf(uart_buffer, "G %d\r", Gain);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

  // Transmit gyro-only mode status
  sprintf(uart_buffer, "M %d\r", gyroOnlyMode);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
}

/**
 * @brief Read HMC1021 magnetometer ADC values
 * @note Converts ADC readings to magnetic field strength and transmits data
 */
void read_ADC(void)
{
  if(adc_conversion_complete)
  {
    adc_conversion_complete = 0;
    
    // Convert ADC value to voltage (3.3V reference, 12-bit ADC)
    hmc1021_voltage = (float)hmc1021_adc_buffer[0] * 3.3f / 4095.0f;
    
    // Convert voltage to magnetic field (±1 Gauss range with 1V/Gauss sensitivity)
    // Assuming 1.65V is zero field (VCC/2)
    hmc1021_gauss = (hmc1021_voltage - 1.65f) / 1.0f;  // 1V/Gauss sensitivity
    
    // Send magnetometer data via UART (single axis)
    sprintf(uart_buffer, "MAG %.3f %.3f\r\n", hmc1021_gauss, hmc1021_voltage);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
    
    // Restart ADC DMA for next conversion
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)hmc1021_adc_buffer, 1);
  }
}

/**
 * @brief Generate SR_IN trigger pulse for HMC1021 magnetometer
 * @note Creates 1µs HIGH pulse for magnetometer set/reset calibration
 */
void trigger_sr_in_pulse(void)
{
  // Set SR_IN pin HIGH (PB12)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  HAL_Delay(1);
  // Set SR_IN pin LOW (PB12)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

/**
 * @brief GPIO external interrupt callback
 * @param GPIO_Pin The pin that triggered the interrupt
 * @note Handles BMI088 accelerometer and gyroscope data ready interrupts
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_4)
  {
    // Accelerometer data ready interrupt
    bmi088_set_accel_INT(&BMI_sensor);
  }
  if(GPIO_Pin == GPIO_PIN_3)
  {
    // Gyroscope data ready interrupt
    bmi088_set_gyro_INT(&BMI_sensor);
  }
}

/**
 * @brief Timer period elapsed callback
 * @param htim Timer handle
 * @note Increments timing flags for periodic operations
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    tx_timer_flag_100ms++;   // 100ms flag
    tx_timer_flag_1s++;      // 1s flag (counts to 10)
  }
}

/**
 * @brief ADC DMA conversion complete callback
 * @param hadc ADC handle
 * @note Sets flag when magnetometer ADC conversion is complete
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1)
  {
    adc_conversion_complete = 1;
  }
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
