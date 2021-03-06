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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <canard.h>
#include <uavcan/si/sample/pressure/Scalar_1_0.h>
#include <uavcan/si/sample/temperature/Scalar_1_0.h>
#include "uavcan.h"

#include "bmp280.h"

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

/* USER CODE BEGIN PV */

BMP280_HandleTypedef bmp280;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void ProcessReceivedTransfer(CanardState* const state, const CanardTransfer* const transfer);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void PublishSensorTemperature(CanardState* const state, UavcanMessage* const msg, float kelvin, const CanardMicrosecond currentTime)
{
	uavcan_si_sample_temperature_Scalar_1_0 sample = {
			.kelvin = kelvin,
			.timestamp.microsecond = currentTime
	};

	sample.timestamp.microsecond = HAL_GetTick()*1000;

	// Serialize and publish
		uint8_t serialized[uavcan_si_sample_temperature_Scalar_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
		size_t serialized_size = sizeof(serialized);
		const int8_t err = uavcan_si_sample_temperature_Scalar_1_0_serialize_(&sample, &serialized[0], &serialized_size);
		assert(err >= 0);
		if (err >= 0)
		{
			const CanardTransfer transfer = {
					.timestamp_usec = currentTime + msg->timeout,
					.priority       = msg->priority,
					.transfer_kind  = CanardTransferKindMessage,
					.port_id        = msg->portId,
					.remote_node_id = CANARD_NODE_ID_UNSET,
					.transfer_id    = (CanardTransferID) msg->transferId++,  // Increment!
					.payload_size   = serialized_size,
					.payload        = &serialized[0],
			};
			(void) canardTxPush(&state->canard, &transfer);
		}
}

static void PublishSensorPressure(CanardState* const state, UavcanMessage* const msg, float pascal, const CanardMicrosecond currentTime)
{
	uavcan_si_sample_pressure_Scalar_1_0 sample = {
			.pascal = pascal,
			.timestamp.microsecond = currentTime
	};

	// Serialize and publish pressure
	uint8_t serialized[uavcan_si_sample_pressure_Scalar_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
	size_t serialized_size = sizeof(serialized);
	const int8_t err = uavcan_si_sample_pressure_Scalar_1_0_serialize_(&sample, &serialized[0], &serialized_size);
	assert(err >= 0);
	if (err >= 0)
	{
		const CanardTransfer transfer = {
				.timestamp_usec = currentTime + msg->timeout,
				.priority       = msg->priority,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = msg->portId,
				.remote_node_id = CANARD_NODE_ID_UNSET,
				.transfer_id    = (CanardTransferID) msg->transferId++,  // Increment!
				.payload_size   = serialized_size,
				.payload        = &serialized[0],
		};
		(void) canardTxPush(&state->canard, &transfer);
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	CanardState state = {0};
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_1;
	bmp280.i2c = &hi2c1;

	while (!bmp280_init(&bmp280, &bmp280.params)) {
		HAL_Delay(2000);
	}

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* CAN1 clock enable */
	__HAL_RCC_CAN1_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**CAN1 GPIO Configuration
      PA11     ------> CAN1_RX
      PA12     ------> CAN1_TX
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	uavcanInit(&state, CAN_NODE_ID, CAN_BITRATE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	UavcanMessage pressureMessageInfo = {
		.portId 				= uavcan_si_sample_pressure_Scalar_1_0_PORT_ID,
		.timeout				= 1000000,
		.priority				= CanardPriorityNominal,
		.transferId				= 0
	};

	UavcanMessage temperatureMessageInfo = {
		.portId 				= uavcan_si_sample_temperature_Scalar_1_0_PORT_ID,
		.timeout				= 1000000,
		.priority				= CanardPriorityNominal,
		.transferId				= 0
	};

	//uint64_t next2HzIteration = state.startedAt + 250000;
	uint64_t next1HzIteration = state.startedAt + 1000000;

	while (1)
	{
		//Do scheduler stuff
		uint64_t microseconds = HAL_GetTick()*1000;
		/*
		if(microseconds >= next2HzIteration)
		{
			next2HzIteration += 500000;
			PublishSensorPressure(&state, &pressure, microseconds);
			Blink();
		}
		*/
		if(microseconds >= next1HzIteration)
		{
			next1HzIteration += 1000000;
			float sensorTemperature;
			float sensorPressure;
			bmp280_read_float(&bmp280, &sensorTemperature, &sensorPressure, NULL);
			//PublishHeartbeat(&state, microseconds);
			PublishSensorTemperature(&state, &temperatureMessageInfo, sensorTemperature + 273.15, microseconds);
			PublishSensorPressure(&state, &pressureMessageInfo, sensorPressure, microseconds);
			Blink();
		}

		// Transmit pending frames from the prioritized TX queue managed by libcanard.
		canardProcessTx(&state);

		// Process received frames by feeding them from bxCAN driver to libcanard.
		// This function will invoke the "process received" handler specified during init.
		canardProcessRx(&state);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

static void ProcessReceivedTransfer(CanardState* const state, const CanardTransfer* const transfer)
{
	if (transfer->transfer_kind == CanardTransferKindMessage)
	{
		size_t size = transfer->payload_size;
	}
	else if (transfer->transfer_kind == CanardTransferKindRequest)
	{

	}
	else
	{
		assert(false);  // Seems like we have set up a port subscription without a handler -- bad implementation.
	}
}

void Blink(void)
{
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
