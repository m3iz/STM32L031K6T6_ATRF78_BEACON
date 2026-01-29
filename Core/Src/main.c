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

uint8_t led_state = 0;
static uint32_t last_rx_ms = 0;

#define RSSI_WINDOW_SIZE        16      // сколько пакетов усредняем
#define RSSI_NEAR_THRESHOLD    10      // подобрать экспериментально
#define RSSI_FAR_THRESHOLD      7
#define STATE_HOLD_TIME_MS    500      // минимальное время удержания состояния
#define RX_SILENCE_TIMEOUT_MS   1500
typedef enum {
    RADIO_FAR = 0,
    RADIO_NEAR
} radio_state_t;

static uint8_t  rssi_buf[RSSI_WINDOW_SIZE];
static uint8_t  rssi_idx = 0;
static uint8_t  rssi_count = 0;
static uint32_t rssi_sum = 0;

static radio_state_t radio_state = RADIO_FAR;
static uint32_t last_state_change_ms = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* counters for debounce */

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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != IRQ_Pin)
        return;
    last_rx_ms = HAL_GetTick();
    uint8_t irq = readRegister(AT86RF2XX_REG__IRQ_STATUS);

    if (!(irq & AT86RF2XX_IRQ_STATUS_MASK__TRX_END))
        return;

    uint8_t rssi = readRegister(0x07) & 0x1F;

    // кольцевой буфер
    if (rssi_count < RSSI_WINDOW_SIZE)
    {
        rssi_buf[rssi_idx++] = rssi;
        rssi_sum += rssi;
        rssi_count++;
    }
    else
    {
        rssi_sum -= rssi_buf[rssi_idx];
        rssi_buf[rssi_idx] = rssi;
        rssi_sum += rssi;
        rssi_idx++;
    }

    if (rssi_idx >= RSSI_WINDOW_SIZE)
        rssi_idx = 0;
}

void radio_proximity_process(void)
{
    uint32_t now = HAL_GetTick();

    /* 1. Потеря сигнала */
    if ((now - last_rx_ms) > RX_SILENCE_TIMEOUT_MS)
    {
        if (radio_state != RADIO_FAR)
        {
            radio_state = RADIO_FAR;
            led_state = 0;
            rssi_count = 0;
            rssi_sum = 0;
            HAL_UART_Transmit(&huart2,
                              (uint8_t *)"RADIO LOST\r\n",
                              12,
                              HAL_MAX_DELAY);
        }
        return;
    }

    /* 2. Недостаточно данных */
    if (rssi_count < RSSI_WINDOW_SIZE)
        return;

    uint8_t rssi_avg = rssi_sum / RSSI_WINDOW_SIZE;

    /* 3. FSM */
    if (radio_state == RADIO_FAR)
    {
        if (rssi_avg >= RSSI_NEAR_THRESHOLD &&
            now - last_state_change_ms >= STATE_HOLD_TIME_MS)
        {
            radio_state = RADIO_NEAR;
            last_state_change_ms = now;
            led_state = 1;
            HAL_UART_Transmit(&huart2,
                              (uint8_t *)"RADIO NEAR\r\n",
                              12,
                              HAL_MAX_DELAY);
        }
    }
    else
    {
        if (rssi_avg <= RSSI_FAR_THRESHOLD &&
            now - last_state_change_ms >= STATE_HOLD_TIME_MS)
        {
            radio_state = RADIO_FAR;
            last_state_change_ms = now;
            led_state = 0;
            HAL_UART_Transmit(&huart2,
                              (uint8_t *)"RADIO FAR\r\n",
                              11,
                              HAL_MAX_DELAY);
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
	while (1) {
		radio_proximity_process();
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
