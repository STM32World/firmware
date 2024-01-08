/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STM32World <lth@stm32world.com>
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Define some IDs to use on the CAN bus
#define CAN_ID_NOW 0b11000000000
#define CAN_ID_RND 0b10000000000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

RNG_HandleTypeDef hrng;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint32_t TxMailbox;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t msg_count = 0;
uint16_t delay = 500;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN2_Init(void);
static void MX_RNG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Send printf to uart1
int _write(int fd, char *ptr, int len) {
    HAL_StatusTypeDef hstatus;

    if (fd == 1 || fd == 2) {
        hstatus = HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, HAL_MAX_DELAY);
        if (hstatus == HAL_OK)
            return len;
        else
            return -1;
    }
    return -1;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    DBG("HAL_CAN_TxMailbox0CompleteCallback");
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    DBG("HAL_CAN_TxMailbox1CompleteCallback");
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    DBG("HAL_CAN_TxMailbox2CompleteCallback");
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) {
    DBG("HAL_CAN_TxMailbox0AbortCallback");
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) {
    DBG("HAL_CAN_TxMailbox1AbortCallback");
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) {
    DBG("HAL_CAN_TxMailbox2AbortCallback");
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

    //DBG("HAL_CAN_RxFifo0MsgPendingCallback");

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
        Error_Handler();

    if (hcan->Instance == CAN1) {
        if (RxHeader.RTR == CAN_RTR_REMOTE) {
            if (RxHeader.StdId == CAN_ID_NOW) {

                DBG("CAN1 got NOW request");

                uint32_t now = HAL_GetTick() / 1000;

                TxHeader.DLC = 4;
                TxHeader.ExtId = 0;
                TxHeader.IDE = CAN_ID_STD;
                TxHeader.RTR = CAN_RTR_DATA;
                TxHeader.StdId = CAN_ID_NOW;

                if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (uint8_t*) &now, &TxMailbox) != HAL_OK)
                    Error_Handler();

            } else if (RxHeader.StdId == CAN_ID_RND) {

                DBG("CAN1 got RND request");

                uint32_t rnd;
                if (HAL_RNG_GenerateRandomNumber(&hrng, &rnd) != HAL_OK)
                    Error_Handler();

                TxHeader.DLC = 4;
                TxHeader.ExtId = 0;
                TxHeader.IDE = CAN_ID_STD;
                TxHeader.RTR = CAN_RTR_DATA;
                TxHeader.StdId = CAN_ID_RND;

                if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (uint8_t*) &rnd, &TxMailbox) != HAL_OK)
                    Error_Handler();

            } else {
                DBG("CAN1 Unknown ID");
            }
        } else {
            DBG("CAN1 Received data");
        }
    } else if (hcan->Instance == CAN2) {
        if (RxHeader.RTR == CAN_RTR_DATA) {
            if (RxHeader.StdId == CAN_ID_NOW) {

                uint32_t *now = (uint32_t*) &RxData[0];

                DBG("CAN2 Got NOW data!  now = %lu", *now);

            } else if (RxHeader.StdId == CAN_ID_RND) {

                uint32_t *rnd = (uint32_t*) &RxData[0];
                delay = (uint16_t) *rnd % 1000;

                DBG("CAN2 Got RND data!  rnd = 0x%08lx new delay = %u", *rnd, 500 + delay);

            } else {
                DBG("CAN2 Unknown ID");
            }
        }
    } else {
        DBG("Unknown CAN Instance");
    }

    msg_count++;
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
    DBG("HAL_CAN_RxFifo0FullCallback");
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    DBG("HAL_CAN_RxFifo1MsgPendingCallback");
}

void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) {
    DBG("HAL_CAN_RxFifo1FullCallback");
}

void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan) {
    DBG("HAL_CAN_SleepCallback");
}

void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan) {
    DBG("HAL_CAN_WakeUpFromRxMsgCallback");
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    DBG("HAL_CAN_ErrorCallback");
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
    MX_CAN1_Init();
    MX_USART1_UART_Init();
    MX_CAN2_Init();
    MX_RNG_Init();
    /* USER CODE BEGIN 2 */

    DBG("\n\n\n\n\n------------\nCAN Starting");

    CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 12;  // anything between 0 to SlaveStartFilterBank
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    //canfilterconfig.FilterIdHigh = 0x103<<5;
    canfilterconfig.FilterIdHigh = 0x0000;
    canfilterconfig.FilterIdLow = 0x0000;
    //canfilterconfig.FilterMaskIdHigh = 0x1<<13;
    canfilterconfig.FilterMaskIdHigh = 0x0;
    canfilterconfig.FilterMaskIdLow = 0x0;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1

    HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

    canfilterconfig.FilterBank = 13;

    HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);

    HAL_CAN_Start(&hcan1);

    HAL_CAN_ActivateNotification(
            &hcan1,
            //CAN_IT_TX_MAILBOX_EMPTY |
            CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_IT_RX_FIFO0_FULL |
            CAN_IT_RX_FIFO0_OVERRUN |
            CAN_IT_RX_FIFO1_MSG_PENDING |
            CAN_IT_RX_FIFO1_FULL |
            CAN_IT_RX_FIFO1_OVERRUN |
            CAN_IT_WAKEUP |
            CAN_IT_SLEEP_ACK |
            CAN_IT_ERROR_WARNING |
            CAN_IT_ERROR_PASSIVE |
            CAN_IT_BUSOFF |
            CAN_IT_LAST_ERROR_CODE |
            CAN_IT_ERROR
            );

    HAL_CAN_Start(&hcan2);

    HAL_CAN_ActivateNotification(
            &hcan2,
            //CAN_IT_TX_MAILBOX_EMPTY |
            CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_IT_RX_FIFO0_FULL |
            CAN_IT_RX_FIFO0_OVERRUN |
            CAN_IT_RX_FIFO1_MSG_PENDING |
            CAN_IT_RX_FIFO1_FULL |
            CAN_IT_RX_FIFO1_OVERRUN |
            CAN_IT_WAKEUP |
            CAN_IT_SLEEP_ACK |
            CAN_IT_ERROR_WARNING |
            CAN_IT_ERROR_PASSIVE |
            CAN_IT_BUSOFF |
            CAN_IT_LAST_ERROR_CODE |
            CAN_IT_ERROR
            );

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    uint32_t now = 0, last_blink = 0, last_now = 0, last_rnd = 0;

    while (1) {

        now = HAL_GetTick();

        if (now - last_now >= 1000) {

            //TxHeader.DLC = 4;
            TxHeader.ExtId = 0;
            TxHeader.IDE = CAN_ID_STD;
            TxHeader.RTR = CAN_RTR_REMOTE;
            TxHeader.StdId = CAN_ID_NOW;
            //TxHeader.TransmitGlobalTime = DISABLE;

            if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, (uint8_t*) &TxData, &TxMailbox) != HAL_OK)
                    {
                Error_Handler();
            }

            last_now = now;
        }

        if (now - last_rnd >= (500 + delay)) {
            //TxHeader.DLC = 4;
            TxHeader.ExtId = 0;
            TxHeader.IDE = CAN_ID_STD;
            TxHeader.RTR = CAN_RTR_REMOTE;
            TxHeader.StdId = CAN_ID_RND;
            //TxHeader.TransmitGlobalTime = DISABLE;

            if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, (uint8_t*) &TxData, &TxMailbox) != HAL_OK)
                    {
                Error_Handler();
            }

            last_rnd = now;
        }

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
void SystemClock_Config(void)
{
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
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
            {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
    hcan1.Init.Prescaler = 3;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
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
    hcan2.Init.Prescaler = 3;
    hcan2.Init.Mode = CAN_MODE_NORMAL;
    hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan2.Init.TimeTriggeredMode = DISABLE;
    hcan2.Init.AutoBusOff = DISABLE;
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
 * @brief RNG Initialization Function
 * @param None
 * @retval None
 */
static void MX_RNG_Init(void)
{

    /* USER CODE BEGIN RNG_Init 0 */

    /* USER CODE END RNG_Init 0 */

    /* USER CODE BEGIN RNG_Init 1 */

    /* USER CODE END RNG_Init 1 */
    hrng.Instance = RNG;
    if (HAL_RNG_Init(&hrng) != HAL_OK)
            {
        Error_Handler();
    }
    /* USER CODE BEGIN RNG_Init 2 */

    /* USER CODE END RNG_Init 2 */

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
    huart1.Init.BaudRate = 921600;
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : LED_Pin */
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
