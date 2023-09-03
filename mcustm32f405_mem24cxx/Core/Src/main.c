/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 Lars Boegild Thomsen <lbthomsen@gmail.com>
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
#include <string.h>
#include <m24cxx.h>
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
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

M24CXX_HandleTypeDef m24cxx;

uint8_t buf[256];
uint8_t do_action = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Send printf to uart1
int _write(int fd, char *ptr, int len) {
    HAL_StatusTypeDef hstatus;

    if (fd == 1 || fd == 2) {
        hstatus = HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len,
        HAL_MAX_DELAY);
        if (hstatus == HAL_OK)
            return len;
        else
            return -1;
    }
    return -1;
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
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_CRC_Init();
    /* USER CODE BEGIN 2 */

    DBG("\n\n\n--------\nStarting");

    // Wait a few ms to get ready
    HAL_Delay(10);

    DBG("Scanning I2C bus:");
    // Go through all possible i2c addresses
    for (uint8_t i = 0; i < 128; i++) {

        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t) (i << 1), 3, 100) == HAL_OK) {
            // We got an ack
            printf("%2x ", i);
        } else {
            printf("-- ");
        }

        if (i > 0 && (i + 1) % 16 == 0)
            printf("\n");

    }

    printf("\n");

    DBG("Initializing %s - %u kB EEPROM", M24CXX_TYPE, M24CXX_SIZE / 1024);

    if (m24cxx_init(&m24cxx, &hi2c1, 0x50) != M24CXX_Ok) {
        DBG("M24CXX Failed to initialize");
        Error_Handler();
    }

    uint8_t buf[M24CXX_WRITE_PAGE_SIZE];
    uint32_t crc = 0;
    uint32_t start_time;

    DBG("Erasing all");
    start_time = HAL_GetTick();
    if (m24cxx_erase_all(&m24cxx) != M24CXX_Ok) {
        DBG("Erase all failed");
        Error_Handler();
    }
    DBG("Erase all took - %lu s", (HAL_GetTick() - start_time) / 1000);

    start_time = HAL_GetTick();
    for (int i = 0; i < M24CXX_SIZE / sizeof(buf); ++i) {
        if (m24cxx_read(&m24cxx, i * sizeof(buf), (uint8_t*) &buf, sizeof(buf)) != M24CXX_Ok) {
            DBG("Read Error");
            Error_Handler();
        }
        if (i == 0)
            crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) &buf, sizeof(buf) / 4);
        else
            crc = HAL_CRC_Accumulate(&hcrc, (uint32_t*) &buf, sizeof(buf) / 4);
    }
    DBG("Read all took - %lu s - CRC = 0x%08lx", (HAL_GetTick() - start_time) / 1000, crc);

    memset(buf, 0x00, sizeof(buf));
    DBG("Writing all 0x00");
    start_time = HAL_GetTick();
    for (int i = 0; i < M24CXX_SIZE / sizeof(buf); ++i) {
        if (m24cxx_write(&m24cxx, i * sizeof(buf), (uint8_t*) &buf, sizeof(buf)) != M24CXX_Ok) {
            DBG("Write Error");
            Error_Handler();
        }
        if (i == 0)
            crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) &buf, sizeof(buf) / 4);
        else
            crc = HAL_CRC_Accumulate(&hcrc, (uint32_t*) &buf, sizeof(buf) / 4);
    }
    DBG("Write all took - %lu s - CRC = 0x%08lx", (HAL_GetTick() - start_time) / 1000, crc);

    start_time = HAL_GetTick();
    for (int i = 0; i < M24CXX_SIZE / sizeof(buf); ++i) {
        if (m24cxx_read(&m24cxx, i * sizeof(buf), (uint8_t*) &buf, sizeof(buf)) != M24CXX_Ok) {
            DBG("Read Error");
            Error_Handler();
        }
        if (i == 0)
            crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) &buf, sizeof(buf) / 4);
        else
            crc = HAL_CRC_Accumulate(&hcrc, (uint32_t*) &buf, sizeof(buf) / 4);
    }
    DBG("Read all took - %lu s - CRC = 0x%08lx", (HAL_GetTick() - start_time) / 1000, crc);

    memset(buf, 0x55, sizeof(buf));
    DBG("Writing all 0x55");
    start_time = HAL_GetTick();
    for (int i = 0; i < M24CXX_SIZE / sizeof(buf); ++i) {
        if (m24cxx_write(&m24cxx, i * sizeof(buf), (uint8_t*) &buf, sizeof(buf)) != M24CXX_Ok) {
            DBG("Write Error");
            Error_Handler();
        }
        if (i == 0)
            crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) &buf, sizeof(buf) / 4);
        else
            crc = HAL_CRC_Accumulate(&hcrc, (uint32_t*) &buf, sizeof(buf) / 4);
    }
    DBG("Write all took - %lu s - CRC = 0x%08lx", (HAL_GetTick() - start_time) / 1000, crc);

    start_time = HAL_GetTick();
    for (int i = 0; i < M24CXX_SIZE / sizeof(buf); ++i) {
        if (m24cxx_read(&m24cxx, i * sizeof(buf), (uint8_t*) &buf, sizeof(buf)) != M24CXX_Ok) {
            DBG("Read Error");
            Error_Handler();
        }
        if (i == 0)
            crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) &buf, sizeof(buf) / 4);
        else
            crc = HAL_CRC_Accumulate(&hcrc, (uint32_t*) &buf, sizeof(buf) / 4);
    }
    DBG("Read all took - %lu s - CRC = 0x%08lx", (HAL_GetTick() - start_time) / 1000, crc);

    memset(buf, 0xaa, sizeof(buf));
    DBG("Writing all 0xaa");
    start_time = HAL_GetTick();
    for (int i = 0; i < M24CXX_SIZE / sizeof(buf); ++i) {
        if (m24cxx_write(&m24cxx, i * sizeof(buf), (uint8_t*) &buf, sizeof(buf)) != M24CXX_Ok) {
            DBG("Write Error");
            Error_Handler();
        }
        if (i == 0)
            crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) &buf, sizeof(buf) / 4);
        else
            crc = HAL_CRC_Accumulate(&hcrc, (uint32_t*) &buf, sizeof(buf) / 4);
    }
    DBG("Write all took - %lu s - CRC = 0x%08lx", (HAL_GetTick() - start_time) / 1000, crc);

    start_time = HAL_GetTick();
    for (int i = 0; i < M24CXX_SIZE / sizeof(buf); ++i) {
        if (m24cxx_read(&m24cxx, i * sizeof(buf), (uint8_t*) &buf, sizeof(buf)) != M24CXX_Ok) {
            DBG("Read Error");
            Error_Handler();
        }
        if (i == 0)
            crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) &buf, sizeof(buf) / 4);
        else
            crc = HAL_CRC_Accumulate(&hcrc, (uint32_t*) &buf, sizeof(buf) / 4);
    }
    DBG("Read all took - %lu s - CRC = 0x%08lx", (HAL_GetTick() - start_time) / 1000, crc);

    DBG("Done Testing - entering main loop");

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    uint32_t now = 0, last_blink = 0;

    while (1) {

        now = HAL_GetTick();

        if (now - last_blink >= 500) {

            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

            last_blink = now;

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
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

    /* USER CODE BEGIN CRC_Init 0 */

    /* USER CODE END CRC_Init 0 */

    /* USER CODE BEGIN CRC_Init 1 */

    /* USER CODE END CRC_Init 1 */
    hcrc.Instance = CRC;
    if (HAL_CRC_Init(&hcrc) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN CRC_Init 2 */

    /* USER CODE END CRC_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

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
    huart1.Init.BaudRate = 921600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
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
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, LED_Pin | EEPROM_WP_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : LED_Pin */
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : EEPROM_WP_Pin */
    GPIO_InitStruct.Pin = EEPROM_WP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EEPROM_WP_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
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
