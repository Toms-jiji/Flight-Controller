/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
/*#########################################################################################
 * Author:
 * =======
 * Toms Jiji Varghese
 * MTech ESE, DESE, IISc
 * Email: tomsjiji@iisc.ac.in
 * Ph.: 9925679264
 *
 *
 * Connections:
 * ===========
 *
 * RX_PPM 	-> PE3			-> TIM5
 *
 * MPU6050
 * -------
 * SCL		-> PB6
 * SDA		-> PB7
 *
 * PWM OUTPUT
 * ----------
 * M1		-> LT -> PB5 	-> TIM3_CH2
 * M2		-> LB -> PB3 	-> TIM2_CH2
 * M3		-> RT -> PE9 	-> TIM1_CH1
 * M4		-> RB -> PE11	-> TIM1_CH2
 *
 * Remaining
 * ---------
 * ADC Read
 * Interrupt Priorities
 * PID tuning channel mapping			----
 * Kalman/Complementary Filter			----
 * PID									----
 * Timer for PID						----
 * Calibrate Motor before starting		----
 * Compensate for battery voltage drop
 *
 * Make sure the receiver does not get plugged out	---
 *
 *
 * ALL LEDS CONSTANT ON - RX CALIBRATION STARTED
 * ALL LEDS BLINKING VERY FAST - RX CALIBRATION VARIFICATION GOING ON
 * ALL LEDS BLINKING NORMALLY  - WAITING TO ARM THE QUADCOPTER
 * LED ROTATES CW ONCE - MPU6050 INITIALIZATION DONE
 * LED ROTATES CW CONTINOUSLY - MPU6050 INITIALIZATION PROBLEM - REMOVE AND RECONNECT POWER
 *
 * Joystick special controls
 * =========================
 * RX Calibration verification
 * 			|-> left joystick to top right and right joystick to bottom  right
 * 			|-> left joystick to bottom left and right joystick to top  left
 * Both Joystick bottom and in opposite directions -> Initial ARM
 * Bottom - LEFT -> ARM/ UNARM
 * 			|-> Top-Right and then bottom -> all motors controlled directly via throttle
 * 				|-> bottom - left -> get out
 *
 * CH5 -> P Gain
 * CH6 -> D Gain
 *#########################################################################################
 */

// Interrupt priorities


#include "MPU6050.h"
#include "DEBUG_UART.h"
#include "ESP01.h"
#include <string.h>
#include <stdio.h>
#include "Quad.h"
#include "PWM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define PPM_READ
//#define PPM_READ_DEBUG
//#define LED_TEST
//#define LED_ROTATE_TEST
#define MPU6050__Testing
//#define MPU6050_MEMS
//#define PWM_testing
//#define QUAD_ARM_TEST
//#define ADC_TEST

//#define QUADCOPTER


#ifdef MPU6050__Testing
	#define PRINT_GYRO
	#define PRINT_ACC
#endif


#ifdef QUAD_ARM_TEST
	#ifndef PPM_READ
		#define PPM_READ
	#endif
#endif

#ifdef QUADCOPTER
	#ifndef PPM_READ
		#define PPM_READ
	#endif
#endif
//#define ESP01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
volatile Quad_PID_DATA QUAD_PID;
volatile QUAD_ORIENTATION QUAD;
volatile Struct_Receiver RX_DATA;
#ifdef PPM_READ

	volatile uint16_t t=0;
	volatile int pulse=0;

	volatile uint32_t IC_Val1 = 0;
	volatile uint32_t IC_Val2 = 0;
	volatile uint32_t Difference = 0;
	volatile int Is_First_Captured = 0;

#endif

Struct_MPU6050 MPU6050;
uint8_t uart2_rx_data = 0;
uint8_t uart2_rx_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM12_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
/*Function prototype for delay and UART2 configuration functions */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef PPM_READ
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
		#ifdef PPM_READ_DEBUG
		uint8_t message[100] = {'\0'};
		#endif
		if(GPIO_Pin == GPIO_PIN_3) // (E3 Pin)
		{
			t = __HAL_TIM_GET_COUNTER(&htim5)/4;
			__HAL_TIM_SET_COUNTER(&htim5, 0);
			if(t>3000){
				pulse = 1;
			}
			else{
				if(RX_DATA.Calibration_status == 1){
					switch(pulse)
					{
					case 1:
						RX_DATA.CH1 = (t-RX_DATA.CH1_offset)*RX_DATA.CH1_scale;
						if(RX_DATA.CH1 < 1000) RX_DATA.CH1 = 1000;
						else if(RX_DATA.CH1 > 2000) RX_DATA.CH1 = 2000;
						break;
					case 2:
						RX_DATA.CH2 = (t-RX_DATA.CH2_offset)*RX_DATA.CH2_scale;
						if(RX_DATA.CH2 < 1000) RX_DATA.CH2 = 1000;
						else if(RX_DATA.CH2 > 2000) RX_DATA.CH2 = 2000;
						break;
					case 3:
						RX_DATA.CH3 = (t-RX_DATA.CH3_offset)*RX_DATA.CH3_scale;
						if(RX_DATA.CH3 < 1000) RX_DATA.CH3 = 1000;
						else if(RX_DATA.CH3 > 2000) RX_DATA.CH3 = 2000;
						break;
					case 4:
						RX_DATA.CH4 = (t-RX_DATA.CH4_offset)*RX_DATA.CH4_scale;
						if(RX_DATA.CH4 < 1000) RX_DATA.CH4 = 1000;
						else if(RX_DATA.CH4 > 2000) RX_DATA.CH4 = 2000;
						break;
					case 5:
						RX_DATA.CH5 = t;
						if(RX_DATA.CH5 < 1000) RX_DATA.CH5 = 1000;
						else if(RX_DATA.CH5 > 2000) RX_DATA.CH5 = 2000;
						break;
					case 6:
						RX_DATA.CH6 = t;
						if(RX_DATA.CH6 < 1000) RX_DATA.CH6 = 1000;
						else if(RX_DATA.CH6 > 2000) RX_DATA.CH6 = 2000;
						break;
					default:
						break;
					}
				}
				else{
					switch(pulse)
					{
					case 1:
						RX_DATA.CH1 = t;
						break;
					case 2:
						RX_DATA.CH2 = t;
						break;
					case 3:
						RX_DATA.CH3 = t;
						break;
					case 4:
						RX_DATA.CH4 = t;
						break;
					case 5:
						RX_DATA.CH5 = t;
						break;
					case 6:
						RX_DATA.CH6 = t;
						break;
					default:
						break;
					}
				}

				pulse++;
				if (pulse>8){
					pulse = 0;
					if(t>100)
						RX_DATA.RX_Valid = 1;
					#ifdef PPM_READ_DEBUG
						sprintf(message, "%u\t-\t%u\t-\t%u\t-\t%u\t-\t%u\t-\t%u\r\n",RX_DATA.CH1, RX_DATA.CH2, RX_DATA.CH3, RX_DATA.CH4, RX_DATA.CH5, RX_DATA.CH6);
						HAL_UART_Transmit(&huart6, message, sizeof(message), 100);
					#endif
				}
			}
		}
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
	char message[300] = {'\0'};
 	RX_DATA.Calibration_status = 0;
 	RX_DATA.RX_Valid = 0;
 	RX_DATA.CH1_max = 0;
 	RX_DATA.CH2_max = 0;
 	RX_DATA.CH3_max = 0;
 	RX_DATA.CH4_max = 0;

 	RX_DATA.CH1_min = 2000;
 	RX_DATA.CH2_min = 2000;
 	RX_DATA.CH3_min = 2000;
 	RX_DATA.CH4_min = 2000;

 	RX_DATA.CH1_offset = 0;
 	RX_DATA.CH2_offset = 0;
 	RX_DATA.CH3_offset = 0;
 	RX_DATA.CH4_offset = 0;

 	RX_DATA.CH1_scale = 1;
 	RX_DATA.CH2_scale = 1;
 	RX_DATA.CH3_scale = 1;
 	RX_DATA.CH4_scale = 1;
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
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

#ifdef PPM_READ
  HAL_TIM_Base_Start(&htim5);
#endif

#ifdef PWM_testing
  PWM_Start();
#endif

#ifdef MPU6050_MEMS
  MPU6050_Initialization();
  Calibrate_Gyro(&MPU6050);
#endif

#ifdef MPU6050__Testing
  MPU6050_Initialization();
  Calibrate_Gyro(&MPU6050);
  while(1){
	  MPU6050_Data(&MPU6050);

#ifdef PRINT_ACC
#ifndef PRINT_GYRO
	  sprintf(message, "AX:%f\t AY:%f\t AZ:%f\n", MPU6050.acc_x, MPU6050.acc_y, MPU6050.acc_z);
	  HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), 100);
#endif
#endif

#ifdef PRINT_GYRO
#ifndef PRINT_ACC
	  sprintf(message, "Gpitch:%f\t GRoll:%f\t GYaw:%f\n", MPU6050.gyro_pitch, MPU6050.gyro_roll, MPU6050.gyro_yaw);
	  HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), 100);
#endif
#endif

#ifdef PRINT_GYRO
#ifdef PRINT_ACC
	  sprintf(message, "AX:%f\tAY:%f\tAZ:%f\tGpitch:%f\tGRoll:%f\tGYaw:%f\r\n", MPU6050.acc_x, MPU6050.acc_y, MPU6050.acc_z, MPU6050.gyro_pitch, MPU6050.gyro_roll, MPU6050.gyro_yaw);
	  HAL_UART_Transmit(&huart6, (uint8_t *)message, strlen(message), 100);
#endif
#endif
  }
#endif

#ifdef ESP01
  ESP_Init();
#endif

#ifdef  QUAD_ARM_TEST
  ARM_Quad();
#endif

#ifdef QUADCOPTER
  HAL_TIM_Base_Start(&htim12);
  quad_init();
#endif

#ifdef ADC_TEST
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc3);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef ADC_TEST
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_Start(&hadc2);
	  HAL_ADC_Start(&hadc3);
	  HAL_ADC_PollForConversion(&hadc1, 1);
	  HAL_ADC_PollForConversion(&hadc2, 1);
	  HAL_ADC_PollForConversion(&hadc3, 1);

	  uint8_t x = HAL_ADC_GetValue(&hadc1);
	  uint8_t y = HAL_ADC_GetValue(&hadc2);
	  uint8_t z = HAL_ADC_GetValue(&hadc3);
	  sprintf(message, "x:%u \t y:%u \t z:%u\r\n",x,y,z);
	  HAL_UART_Transmit(&huart6, message, sizeof(message), 100);
#endif

#ifdef PWM_testing
	  int16_t dutyCycle = 1000;
	  while(dutyCycle < 2000)
	 	        {
		  	  	  	SET_PWM_Value(dutyCycle,dutyCycle,dutyCycle,dutyCycle);
					dutyCycle += 100;
	 	            HAL_Delay(1000);
	 	        }
	 	        while(dutyCycle > 1000)
	 	        {
	 	        	SET_PWM_Value(dutyCycle,dutyCycle,dutyCycle,dutyCycle);
	 	            dutyCycle -= 100;
	 	            HAL_Delay(1000);
	 	        }
#endif

#ifdef LED_TEST
	  set_Leds(1,0,0,0);
	  HAL_Delay(1000);

	  set_Leds(0,1,0,0);
	  HAL_Delay(1000);

	  set_Leds(0,0,1,0);
	  HAL_Delay(1000);

	  set_Leds(0,0,0,1);
	  HAL_Delay(1000);
#endif

#ifdef LED_ROTATE_TEST
	  Led_Rotate_CW(100,5);
	  Led_Rotate_CCW(100,5);
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#ifdef QUADCOPTER
	  quad_fly();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
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
  sConfig.Channel = ADC_CHANNEL_13;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_8B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_8B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  htim1.Init.Prescaler = 168-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3000;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 21-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 84-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

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
  huart6.Init.BaudRate = 921600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15
                           PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
