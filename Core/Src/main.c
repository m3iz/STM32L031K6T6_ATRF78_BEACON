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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define RSSI_THRESHOLD       4    // порог для RADIO CLOSE
#define RSSI_ON_COUNT        50   // сколько подтверждений нужно для включения
#define RSSI_OFF_COUNT       150   // сколько подтверждений нужно для выключения
#define RSSI_THRESHOLD_LOW   1
#define RSSI_THRESHOLD_CLOSE 6    // порог для RADIO NEAR

/* USER CODE END PTD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Перечисление возможных состояний связи
typedef enum {
	STATE_LOST, STATE_CLOSE, STATE_NEAR
} connection_state_t;

volatile uint32_t last_connected_change_ms = 0;

/* counters for debounce */
static uint8_t rssi_ok_count = 0;   // счётчик для CLOSE (RSSI >=4)
static uint8_t rssi_near_count = 0;   // счётчик для NEAR (RSSI >=6)
static uint8_t rssi_low_count = 0;   // счётчик для LOST

static connection_state_t current_state = STATE_LOST;
static uint8_t led_state = 0;     // 1 – есть связь (CLOSE или NEAR), 0 – потеря

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == IRQ_Pin) {
		last_connected_change_ms = HAL_GetTick();

		uint8_t irq = readRegister(AT86RF2XX_REG__IRQ_STATUS);

		if (irq & AT86RF2XX_IRQ_STATUS_MASK__TRX_END) {
			uint8_t rssi_raw = readRegister(0x07);

			// ---- Обработка уровней RSSI с антидребезгом ----
			if (rssi_raw >= RSSI_THRESHOLD_CLOSE) {
				// NEAR (RSSI >= 6)
				rssi_near_count++;
				rssi_ok_count = 0;
				rssi_low_count = 0;

				if (rssi_near_count >= RSSI_ON_COUNT) {
					// достигнут порог для NEAR
					if (current_state != STATE_NEAR) {
						// меняем состояние и отправляем сообщение
						current_state = STATE_NEAR;
						led_state = 1;
						HAL_UART_Transmit(&huart2, (uint8_t*) "RADIO NEAR\r\n",
								12, HAL_MAX_DELAY);
					}
				}
			} else if (rssi_raw >= RSSI_THRESHOLD) {
				// CLOSE (4 <= RSSI < 6)
				rssi_ok_count++;
				rssi_near_count = 0;
				rssi_low_count = 0;

				if (rssi_ok_count >= RSSI_ON_COUNT) {
					if (current_state != STATE_CLOSE) {
						current_state = STATE_CLOSE;
						led_state = 1;
						HAL_UART_Transmit(&huart2, (uint8_t*) "RADIO CLOSE\r\n",
								13, HAL_MAX_DELAY);
					}
				}
			} else if (rssi_raw <= RSSI_THRESHOLD_LOW) {
				// LOST
				rssi_low_count++;
				rssi_ok_count = 0;
				rssi_near_count = 0;

				if (rssi_low_count >= RSSI_OFF_COUNT) {
					if (current_state != STATE_LOST) {
						current_state = STATE_LOST;
						led_state = 0;
						HAL_UART_Transmit(&huart2, (uint8_t*) "RADIO LOST\r\n",
								12, HAL_MAX_DELAY);
					}
				}
			} else {
				// RSSI в промежуточной зоне (между 2 и 3) – сбрасываем все счётчики, но состояние не меняем
				rssi_ok_count = 0;
				rssi_near_count = 0;
				rssi_low_count = 0;
			}
		}
	}
}

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
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	at86rf233_init(AT_RX);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t last_status_send_ms = HAL_GetTick();
	while (1) {
		uint32_t now = HAL_GetTick();

		// fail-safe: если давно не было прерываний и связь считалась установленной – сбрасываем в LOST
		if ((current_state != STATE_LOST)
				&& (now - last_connected_change_ms) > 5000
				&& now > last_connected_change_ms) {
			current_state = STATE_LOST;
			led_state = 0;
			rssi_ok_count = 0;
			rssi_near_count = 0;
			rssi_low_count = 0;
			for (int i = 0; i < 4; i++) {
				HAL_UART_Transmit(&huart2, (uint8_t*) "RADIO LOST\r\n", 12,
						HAL_MAX_DELAY);
				HAL_Delay(100);
			}
		}
		if ((now - last_status_send_ms) >= 2000) {
			last_status_send_ms = now;
			char msg[64];
			const char *state_str;
			switch (current_state) {
			case STATE_NEAR:
				state_str = "RADIO_NEAR";
				break;
			case STATE_CLOSE:
				state_str = "RADIO_CLOSE";
				break;
			default:
				state_str = "RADIO_LOST";
				break;
			}

			HAL_UART_Transmit(&huart2, (uint8_t*) state_str, strlen(state_str),
					HAL_MAX_DELAY);

		}

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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
