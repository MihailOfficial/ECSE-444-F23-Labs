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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_psensor.h"
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01_gyro.h"
#include "stm32l4s5i_iot01_magneto.h"
#include "stm32l4s5i_iot01_qspi.h"

#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

UART_HandleTypeDef huart1;

osThreadId button_controlHandle;
osThreadId transmit_dataHandle;
osThreadId read_dataHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_OCTOSPI1_Init(void);
void buttonControl(void const *argument);
void transmitData(void const *argument);
void readData(void const *argument);

/* USER CODE BEGIN PFP */
osMutexId mutex;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int shouldPrint = 0;
int selectedSensor = 0;
int iterT = 0;
int iterH = 0;
int iterP = 0;
int iterA = 0;
int iterG = 0;
int iterM = 0;
float data_to_write[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float data_to_writeA[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float data_to_writeB[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float data_to_writeC[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

int hold = 0;
float samplesG = 0;
float meanG = 0;
float varianceG = 0;
float samplesGA = 0;
float meanGA = 0;
float varianceGA = 0;
float samplesGB = 0;
float meanGB = 0;
float varianceGB = 0;
float samplesGC = 0;
float meanGC = 0;
float varianceGC = 0;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART1_UART_Init();
	MX_I2C2_Init();
	MX_OCTOSPI1_Init();
	/* USER CODE BEGIN 2 */

	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_TSENSOR_Init();
	BSP_HSENSOR_Init();
	BSP_MAGNETO_Init();
	BSP_PSENSOR_Init();

	BSP_QSPI_Init();

	uint8_t buffer[100];

	if (BSP_QSPI_Erase_Block(0x20) == QSPI_OK) {
		memset(buffer, 0, sizeof(buffer));
		sprintf(buffer, "Erased memory! \r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 300000);
	}


	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	osMutexDef(osMutex);
	mutex = osMutexCreate(osMutex(osMutex));

	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of button_control */
	osThreadDef(button_control, buttonControl, osPriorityHigh, 0, 128);
	button_controlHandle = osThreadCreate(osThread(button_control), NULL);

	/* definition and creation of transmit_data */
	osThreadDef(transmit_data, transmitData, osPriorityNormal, 0, 256);
	transmit_dataHandle = osThreadCreate(osThread(transmit_data), NULL);

	/* definition and creation of read_data */
	osThreadDef(read_data, readData, osPriorityNormal, 0, 256);
	read_dataHandle = osThreadCreate(osThread(read_data), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST)
			!= HAL_OK) {
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
	RCC_OscInitStruct.PLL.PLLN = 60;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x307075B1;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief OCTOSPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_OCTOSPI1_Init(void) {

	/* USER CODE BEGIN OCTOSPI1_Init 0 */

	/* USER CODE END OCTOSPI1_Init 0 */

	OSPIM_CfgTypeDef OSPIM_Cfg_Struct = { 0 };

	/* USER CODE BEGIN OCTOSPI1_Init 1 */

	/* USER CODE END OCTOSPI1_Init 1 */
	/* OCTOSPI1 parameter configuration*/
	hospi1.Instance = OCTOSPI1;
	hospi1.Init.FifoThreshold = 1;
	hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
	hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
	hospi1.Init.DeviceSize = 32;
	hospi1.Init.ChipSelectHighTime = 1;
	hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
	hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
	hospi1.Init.ClockPrescaler = 1;
	hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
	hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
	hospi1.Init.ChipSelectBoundary = 0;
	hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
	if (HAL_OSPI_Init(&hospi1) != HAL_OK) {
		Error_Handler();
	}
	OSPIM_Cfg_Struct.ClkPort = 1;
	OSPIM_Cfg_Struct.NCSPort = 1;
	OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
	if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct,
	HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN OCTOSPI1_Init 2 */

	/* USER CODE END OCTOSPI1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : PIN_BUTTON_Pin */
	GPIO_InitStruct.Pin = PIN_BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PIN_BUTTON_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void calculate(float *data, float size) {

	samplesG = 0;
	meanG = 0;
	varianceG = 0;

	float sum = 0, mean, variance = 0;

	// Calculate sum of the samples
	for (int i = 0; i < size; ++i) {
		sum += data[i];
	}

	// Calculate mean
	mean = (float) sum / size;

	// Calculate sum of squared differences from the mean
	for (int i = 0; i < size; ++i) {
		int diff = data[i] - mean;
		variance += diff * diff;
	}

	// Finalize variance calculation
	variance /= (size - 1);

	samplesG = 10;
	meanG = mean;
	varianceG = variance;

}

void calculateMulti(float *dataA, float *dataB, float *dataC, float size) {

	samplesGA = 0;
	meanGA = 0;
	varianceGA = 0;

	samplesGB = 0;
	meanGB = 0;
	varianceGB = 0;

	samplesGC = 0;
	meanGC = 0;
	varianceGC = 0;

	float sum = 0;
	float mean, variance = 0;

	// Calculate sum of the samples
	for (int i = 0; i < size; ++i) {
		sum += dataA[i];
	}

	// Calculate mean
	mean = (float) sum / size;

	// Calculate sum of squared differences from the mean
	for (int i = 0; i < size; ++i) {
		float diff = (float) dataA[i] - mean;
		variance += diff * diff;
	}

	// Finalize variance calculation
	variance /= (size - 1);

	samplesGA = 10;
	meanGA = mean;
	varianceGA = variance;

	sum = 0;
	mean = 0;
	variance = 0;

	// Calculate sum of the samples
	for (int i = 0; i < size; ++i) {
		sum += dataB[i];
	}

	// Calculate mean
	mean = (float) sum / size;

	// Calculate sum of squared differences from the mean
	for (int i = 0; i < size; ++i) {
		float diff =(float) dataB[i] - mean;
		variance += diff * diff;
	}

	// Finalize variance calculation
	variance /= (size - 1);

	samplesGB = 10;
	meanGB = mean;
	varianceGB = variance;

	sum = 0;
	mean = 0;
	variance = 0;

	// Calculate sum of the samples
	for (int i = 0; i < size; ++i) {
		sum += dataC[i];
	}

	// Calculate mean
	mean = (float) sum / size;

	// Calculate sum of squared differences from the mean
	for (int i = 0; i < size; ++i) {
		float diff = (float) dataC[i] - mean;
		variance += diff * diff;
	}

	// Finalize variance calculation
	variance /= (size - 1);

	samplesGC = 10;
	meanGC = mean;
	varianceGC = variance;

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_buttonControl */
/**
 * @brief  Function implementing the button_control thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_buttonControl */
void buttonControl(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(100);

		if (HAL_GPIO_ReadPin(PIN_BUTTON_GPIO_Port, PIN_BUTTON_Pin)
				== GPIO_PIN_RESET) {

			osMutexWait(mutex, osWaitForever);

			uint8_t buffer[100];
			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer, "- Button Press \r\n");
			HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer), HAL_MAX_DELAY);

			selectedSensor = selectedSensor + 1;
			shouldPrint = 1;

			if (selectedSensor > 7) {
				selectedSensor = 0;
				shouldPrint = 0;
			}

			while (HAL_GPIO_ReadPin(PIN_BUTTON_GPIO_Port, PIN_BUTTON_Pin) == GPIO_PIN_RESET) {
				//stall
			}

			osMutexRelease(mutex);

		}

	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_transmitData */
/**
 * @brief Function implementing the transmit_data thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_transmitData */
void transmitData(void const *argument) {
	/* USER CODE BEGIN transmitData */
	/* Infinite loop */
	for (;;) {

		osDelay(100);

		if (shouldPrint == 1) {

			osMutexWait(mutex, osWaitForever);
			float readingFloat;
			float readingsFloat[3];
			int16_t readings16t[3];
			uint8_t buffer[100];


			if (selectedSensor == 1) {

				memset(buffer, 0, sizeof(buffer));
				readingFloat = (float) BSP_TSENSOR_ReadTemp();
				sprintf(buffer, "Temperature: %f \r\n", readingFloat);
				HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
				HAL_MAX_DELAY);

				if (iterT < 10) {
					data_to_write[iterT] = readingFloat;
					iterT++;
				}

				else if (iterT == 10) {
					iterT++;

					if (BSP_QSPI_Write(data_to_write, 0,
							(10 * sizeof(float))) != QSPI_OK) {
						memset(buffer, 0, sizeof(buffer));
						sprintf(buffer, "Error write! \r\n");
						HAL_UART_Transmit(&huart1, buffer,
								strlen((char*) buffer), HAL_MAX_DELAY);
					} else {
						memset(buffer, 0, sizeof(buffer));
						sprintf(buffer, "Full array -> Stored to flash\r\n");
						HAL_UART_Transmit(&huart1, buffer,
								strlen((char*) buffer), HAL_MAX_DELAY);

					}

				}

			} else if (selectedSensor == 2) {
				memset(buffer, 0, sizeof(buffer));
				readingFloat = (float) BSP_HSENSOR_ReadHumidity();
				sprintf(buffer, "Humidity: %f \r\n", readingFloat);
				HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
						strlen((char*) buffer), HAL_MAX_DELAY);


				if (iterH < 10) {
					data_to_write[iterH] = readingFloat;
					iterH++;
				}

				else if (iterH == 10) {
					iterH++;

					if (BSP_QSPI_Write(data_to_write, 10 * sizeof(float) * 1,
							10 * sizeof(float)) != QSPI_OK) {
						memset(buffer, 0, sizeof(buffer));
						sprintf(buffer, "Error write! \r\n");
						HAL_UART_Transmit(&huart1, buffer,
								strlen((char*) buffer), HAL_MAX_DELAY);

					} else {

						memset(buffer, 0, sizeof(buffer));
						sprintf(buffer, "Full array -> Stored to flash\r\n");
						HAL_UART_Transmit(&huart1, buffer,
								strlen((char*) buffer), HAL_MAX_DELAY);

					}

				}

			} else if (selectedSensor == 3) {
				memset(buffer, 0, sizeof(buffer));
				readingFloat = (float) BSP_PSENSOR_ReadPressure();
				sprintf(buffer, "Pressure: %f \r\n", readingFloat);
				HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
						strlen((char*) buffer), HAL_MAX_DELAY);

				if (iterP < 10) {
					data_to_write[iterP] = readingFloat;
					iterP++;
				}

				else if (iterP == 10) {
					iterP++;


					if (BSP_QSPI_Write(data_to_write, 10 * sizeof(float) * 2,
							10 * sizeof(float)) != QSPI_OK) {
						memset(buffer, 0, sizeof(buffer));
						sprintf(buffer, "Error write! \r\n");
						HAL_UART_Transmit(&huart1, buffer,
								strlen((char*) buffer), HAL_MAX_DELAY);
					} else {
						memset(buffer, 0, sizeof(buffer));
						sprintf(buffer, "Full array -> Stored to flash\r\n");
						HAL_UART_Transmit(&huart1, buffer,
								strlen((char*) buffer), HAL_MAX_DELAY);

					}
				}

			} else if (selectedSensor == 4) {
				memset(buffer, 0, sizeof(buffer));
				BSP_ACCELERO_AccGetXYZ(readings16t);
				sprintf(buffer, "Acceleration: X = %d, Y = %d, Z = %d \r\n",
						readings16t[0], readings16t[1], readings16t[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
						strlen((char*) buffer), HAL_MAX_DELAY);

				if (iterA < 10) {
					data_to_writeA[iterA] = readings16t[0];
					data_to_writeB[iterA] = readings16t[1];
					data_to_writeC[iterA] = readings16t[2];
					iterA++;
				}

				else if (iterA == 10) {
					iterA++;

					if (BSP_QSPI_Write(data_to_writeA, 10 * sizeof(float) * 3,
							10 * sizeof(float)) != QSPI_OK) {
						memset(buffer, 0, sizeof(buffer));
						sprintf(buffer, "Error write! \r\n");
						HAL_UART_Transmit(&huart1, buffer,
								strlen((char*) buffer), HAL_MAX_DELAY);
					}


					if (BSP_QSPI_Write(data_to_writeB, 10 * sizeof(float) * 4,
							10 * sizeof(float)) != QSPI_OK) {
						memset(buffer, 0, sizeof(buffer));
						sprintf(buffer, "Error write! \r\n");
						HAL_UART_Transmit(&huart1, buffer,
								strlen((char*) buffer), HAL_MAX_DELAY);
					}


					if (BSP_QSPI_Write(data_to_writeC, 10 * sizeof(float) * 5,
							10 * sizeof(float)) != QSPI_OK) {
						memset(buffer, 0, sizeof(buffer));
						sprintf(buffer, "Error write! \r\n");
						HAL_UART_Transmit(&huart1, buffer,
								strlen((char*) buffer), HAL_MAX_DELAY);
					}

					memset(buffer, 0, sizeof(buffer));
					sprintf(buffer, "Full arrays -> Stored to flash\r\n");
					HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
					HAL_MAX_DELAY);

				}

			} else if (selectedSensor == 5) {
				memset(buffer, 0, sizeof(buffer));
				BSP_GYRO_GetXYZ(readingsFloat);
				sprintf(buffer, "Gyroscope: X = %f, Y = %f, Z = %f \r\n",
						readingsFloat[0], readingsFloat[1], readingsFloat[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
						strlen((char*) buffer), HAL_MAX_DELAY);

				if (iterG < 10) {
					data_to_writeA[iterG] = readingsFloat[0];
					data_to_writeB[iterG] = readingsFloat[1];
					data_to_writeC[iterG] = readingsFloat[2];
					iterG++;
				}

				else if (iterG == 10) {
					iterG++;

					if (BSP_QSPI_Write(data_to_writeA, 10 * sizeof(float) * 6,
							10 * sizeof(float)) != QSPI_OK) {
						memset(buffer, 0, sizeof(buffer));
						sprintf(buffer, "Error write! \r\n");
						HAL_UART_Transmit(&huart1, buffer,
								strlen((char*) buffer), HAL_MAX_DELAY);
					}

					if (BSP_QSPI_Write(data_to_writeB, 10 * sizeof(float) * 7,
							10 * sizeof(float)) != QSPI_OK) {
						memset(buffer, 0, sizeof(buffer));
						sprintf(buffer, "Error write! \r\n");
						HAL_UART_Transmit(&huart1, buffer,
								strlen((char*) buffer), HAL_MAX_DELAY);
					}

					if (BSP_QSPI_Write(data_to_writeC, 10 * sizeof(float) * 8,
							10 * sizeof(float)) != QSPI_OK) {
						memset(buffer, 0, sizeof(buffer));
						sprintf(buffer, "Error write! \r\n");
						HAL_UART_Transmit(&huart1, buffer,
								strlen((char*) buffer), HAL_MAX_DELAY);
					}

					memset(buffer, 0, sizeof(buffer));
					sprintf(buffer, "Full arrays -> Stored to flash\r\n");
					HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
					HAL_MAX_DELAY);

				}

			} else if (selectedSensor == 6) {
				memset(buffer, 0, sizeof(buffer));
				BSP_MAGNETO_GetXYZ(readings16t);
				sprintf(buffer, "Magnetometer: X = %d, Y = %d, Z = %d \r\n",
						readings16t[0], readings16t[1], readings16t[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
						strlen((char*) buffer), HAL_MAX_DELAY);

				if (iterM < 10) {
					data_to_writeA[iterM] = readings16t[0];
					data_to_writeB[iterM] = readings16t[1];
					data_to_writeC[iterM] = readings16t[2];
					iterM++;
				}

				else if (iterM == 10) {
					iterM++;

					if (BSP_QSPI_Write(data_to_writeA, 10 * sizeof(float) * 9,
							10 * sizeof(float)) != QSPI_OK) {
						memset(buffer, 0, sizeof(buffer));
						sprintf(buffer, "Error write! \r\n");
						HAL_UART_Transmit(&huart1, buffer,
								strlen((char*) buffer), HAL_MAX_DELAY);
					}

					if (BSP_QSPI_Write(data_to_writeB, 10 * sizeof(float) * 10,
							10 * sizeof(float)) != QSPI_OK) {
						memset(buffer, 0, sizeof(buffer));
						sprintf(buffer, "Error write! \r\n");
						HAL_UART_Transmit(&huart1, buffer,
								strlen((char*) buffer), HAL_MAX_DELAY);
					}

					if (BSP_QSPI_Write(data_to_writeC, 10 * sizeof(float) * 11,
							10 * sizeof(float)) != QSPI_OK) {
						memset(buffer, 0, sizeof(buffer));
						sprintf(buffer, "Error write! \r\n");
						HAL_UART_Transmit(&huart1, buffer,
								strlen((char*) buffer), HAL_MAX_DELAY);
					}

					memset(buffer, 0, sizeof(buffer));
					sprintf(buffer, "Full arrays -> Stored to flash\r\n");
					HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
					HAL_MAX_DELAY);

				}
			}
			osMutexRelease(mutex);
		}
	}
	/* USER CODE END transmitData */
}

/* USER CODE BEGIN Header_readData */
/**
 * @brief Function implementing the read_data thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_readData */
void readData(void const *argument) {
	/* USER CODE BEGIN readData */
	/* Infinite loop */
	for (;;) {
		osDelay(200);

		if (selectedSensor == 7 && hold == 0) {
			osMutexWait(mutex, osWaitForever);


			uint8_t buffer[100];

			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer, " \r\n	| Statistics:  \r\n");
			HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
			HAL_MAX_DELAY);


			memset(data_to_write, 0, sizeof(data_to_write)); // Clear the read buffer
			if (BSP_QSPI_Read(data_to_write, 0, 10 * sizeof(float)) != QSPI_OK) {
				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "Error read! \r\n");
				HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
				HAL_MAX_DELAY);
			}

			calculate(data_to_write, 10);

			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer,
					"\r\n	| Stats Temperature: Num Sample: %f Mean: %f Variance %f\r\n",
					samplesG, meanG, varianceG);
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
					strlen((char*) buffer), HAL_MAX_DELAY);

			memset(data_to_write, 0, sizeof(data_to_write));
			if (BSP_QSPI_Read(data_to_write, 10 * sizeof(float), 10 * sizeof(float)) != QSPI_OK) {
				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "Error read! \r\n");
				HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
				HAL_MAX_DELAY);
			}

			calculate(data_to_write, 10);

			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer,
					"	| Stats Pressure: Num Sample: %f Mean: %f Variance %f\r\n",
					samplesG, meanG, varianceG);
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
					strlen((char*) buffer), HAL_MAX_DELAY);

			memset(data_to_write, 0, sizeof(data_to_write));
			if (BSP_QSPI_Read(data_to_write, 10 * sizeof(float) * 2, 10 * sizeof(float)) != QSPI_OK) {
				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "Error read! \r\n");
				HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
				HAL_MAX_DELAY);
			}

			calculate(data_to_write, 10);

			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer,
					"	| Stats Humidity: Num Sample: %f Mean: %f Variance %f\r\n",
					samplesG, meanG, varianceG);
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
					strlen((char*) buffer), HAL_MAX_DELAY);


			memset(data_to_writeA, 0, sizeof(data_to_writeA));
			memset(data_to_writeB, 0, sizeof(data_to_writeB));
			memset(data_to_writeC, 0, sizeof(data_to_writeC));

			if (BSP_QSPI_Read(data_to_writeA,  10 * sizeof(float) * 3, 10 * sizeof(float)) != QSPI_OK) {
				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "Error read! \r\n");
				HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
				HAL_MAX_DELAY);
			}

			if (BSP_QSPI_Read(data_to_writeB,  10 * sizeof(float) * 4, 10 * sizeof(float)) != QSPI_OK) {
				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "Error read! \r\n");
				HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
				HAL_MAX_DELAY);
			}

			if (BSP_QSPI_Read(data_to_writeC,  10 * sizeof(float) * 5, 10 * sizeof(float)) != QSPI_OK) {
				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "Error read! \r\n");
				HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
				HAL_MAX_DELAY);
			}

			calculateMulti(data_to_writeA, data_to_writeB, data_to_writeC, 10);

			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer,
					" \r\n	| Stats Acc X: Num Sample: %f Mean: %f Variance %f\r\n",
					samplesGA, meanGA, varianceGA);
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
					strlen((char*) buffer), HAL_MAX_DELAY);

			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer,
					"	| Stats Acc Y: Num Sample: %f Mean: %f Variance %f\r\n",
					samplesGB, meanGB, varianceGB);
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
					strlen((char*) buffer), HAL_MAX_DELAY);

			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer,
					"	| Stats Acc Z: Num Sample: %f Mean: %f Variance %f\r\n",
					samplesGC, meanGC, varianceGC);
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
					strlen((char*) buffer), HAL_MAX_DELAY);


			memset(data_to_writeA, 0, sizeof(data_to_writeA));
			memset(data_to_writeB, 0, sizeof(data_to_writeB));
			memset(data_to_writeC, 0, sizeof(data_to_writeC));

			if (BSP_QSPI_Read(data_to_writeA,  10 * sizeof(float) * 6, 10 * sizeof(float)) != QSPI_OK) {
				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "Error read! \r\n");
				HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
				HAL_MAX_DELAY);
			}

			if (BSP_QSPI_Read(data_to_writeB,  10 * sizeof(float) * 7, 10 * sizeof(float)) != QSPI_OK) {
				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "Error read! \r\n");
				HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
				HAL_MAX_DELAY);
			}

			if (BSP_QSPI_Read(data_to_writeC,  10 * sizeof(float) * 8, 10 * sizeof(float)) != QSPI_OK) {
				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "Error read! \r\n");
				HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
				HAL_MAX_DELAY);
			}

			calculateMulti(data_to_writeA, data_to_writeB, data_to_writeC, 10);

			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer,
					" \r\n	| Stats Gyro X: Num Sample: %f Mean: %f Variance %f\r\n",
					samplesGA, meanGA, varianceGA);
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
					strlen((char*) buffer), HAL_MAX_DELAY);

			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer,
					"	| Stats Gyro Y: Num Sample: %f Mean: %f Variance %f\r\n",
					samplesGB, meanGB, varianceGB);
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
					strlen((char*) buffer), HAL_MAX_DELAY);

			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer,
					"	| Stats Gyro Z: Num Sample: %f Mean: %f Variance %f\r\n",
					samplesGC, meanGC, varianceGC);
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
					strlen((char*) buffer), HAL_MAX_DELAY);


			memset(data_to_writeA, 0, sizeof(data_to_writeA));
			memset(data_to_writeB, 0, sizeof(data_to_writeB));
			memset(data_to_writeC, 0, sizeof(data_to_writeC));

			if (BSP_QSPI_Read(data_to_writeA,  10 * sizeof(float) * 9, 10 * sizeof(float)) != QSPI_OK) {
				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "Error read! \r\n");
				HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
				HAL_MAX_DELAY);
			}

			if (BSP_QSPI_Read(data_to_writeB,  10 * sizeof(float) * 10, 10 * sizeof(float)) != QSPI_OK) {
				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "Error read! \r\n");
				HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
				HAL_MAX_DELAY);
			}

			if (BSP_QSPI_Read(data_to_writeC,  10 * sizeof(float) * 11,  10 * sizeof(float)) != QSPI_OK) {
				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "Error read! \r\n");
				HAL_UART_Transmit(&huart1, buffer, strlen((char*) buffer),
				HAL_MAX_DELAY);
			}

			calculateMulti(data_to_writeA, data_to_writeB, data_to_writeC, 10);

			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer,
					" \r\n	| Stats Magn X: Num Sample: %f Mean: %f Variance %f\r\n",
					samplesGA, meanGA, varianceGA);
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
					strlen((char*) buffer), HAL_MAX_DELAY);

			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer,
					"	| Stats Magn Y: Num Sample: %f Mean: %f Variance %f\r\n",
					samplesGB, meanGB, varianceGB);
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
					strlen((char*) buffer), HAL_MAX_DELAY);

			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer,
					"	| Stats Magn Z: Num Sample: %f Mean: %f Variance %f\r\n",
					samplesGC, meanGC, varianceGC);
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
					strlen((char*) buffer), HAL_MAX_DELAY);

			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer, " \r\n	--> Press again to restart \r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
					strlen((char*) buffer), HAL_MAX_DELAY);

			hold = 0;
			shouldPrint = 0;
			selectedSensor = 0;

			iterT = 0;
			iterH = 0;
			iterP = 0;
			iterA = 0;
			iterG = 0;
			iterM = 0;

			if (BSP_QSPI_Erase_Block(0x20) == QSPI_OK) {
					memset(buffer, 0, sizeof(buffer));
					sprintf(buffer, "Erased memory! \r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 300000);
			}


			osMutexRelease(mutex);

		}

	}

	/* USER CODE END readData */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();

	uint8_t buffer[100];

	memset(buffer, 0, sizeof(buffer));
	sprintf(buffer, "CRITICAL ERROR");
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen((char*) buffer),
	HAL_MAX_DELAY);

	while (1) {
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
