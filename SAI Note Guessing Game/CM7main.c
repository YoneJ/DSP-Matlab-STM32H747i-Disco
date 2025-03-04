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
#define ARM_MATH_CM7
#include "arm_math.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#define AUDIO_BUFFER_SIZE 128
#define TARGET_FREQUENCY 330.0f
#define SAMPLE_RATE 48000.0f
#define RX_BUFFER_SIZE 64

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SAI_HandleTypeDef hsai_BlockB2;
DMA_HandleTypeDef hdma_sai2_b;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char uart_buffer[128];
int16_t audioBuffer[AUDIO_BUFFER_SIZE];
float32_t floatBuffer[AUDIO_BUFFER_SIZE]; // Buffer for converted float data
float32_t fftOutput[AUDIO_BUFFER_SIZE];
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_data;
volatile bool command_received = false;
bool game_running = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SAI2_Init(void);
/* USER CODE BEGIN PFP */
void TestSAI_Polling(void);
void process_command(uint8_t *command);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint16_t rx_index = 0;

    if (huart->Instance == USART1)
    {
        // Debugging: Echo each received character
        HAL_UART_Transmit(&huart1, (uint8_t *)"RX: ", 4, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, &rx_data, 1, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

        if (rx_data == '\n' || rx_data == '\r')
        {
            rx_buffer[rx_index] = '\0'; // Null-terminate the string
            rx_index = 0;               // Reset the index for the next command
            command_received = true;   // Set the flag to process the command

            // Debugging: Print the complete received command
            HAL_UART_Transmit(&huart1, (uint8_t *)"Command: ", 9, HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart1, rx_buffer, strlen((char *)rx_buffer), HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
        }
        else
        {
            if (rx_index < RX_BUFFER_SIZE - 1)
            {
                rx_buffer[rx_index++] = rx_data; // Store the received byte
            }
        }

        // Restart UART reception in interrupt mode
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}


void process_command(uint8_t *command)
{
    // Echo the command back to the sender
    HAL_UART_Transmit(&huart1, command, strlen((char *)command), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

    // Check and process the command
    if (strcmp((char *)command, "START") == 0)
    {
        game_running = true;
        HAL_UART_Transmit(&huart1, (uint8_t *)"Game Started\r\n", 14, HAL_MAX_DELAY);
    }
    else if (strcmp((char *)command, "STOP") == 0)
    {
        game_running = false;
        HAL_GPIO_WritePin(GPIOI, GPIO_PIN_All, GPIO_PIN_SET); // Turn off all LEDs
        HAL_UART_Transmit(&huart1, (uint8_t *)"Game Stopped\r\n", 14, HAL_MAX_DELAY);
    }
}
void ProcessAudio(void);
void SetLEDs(float frequencyDifference);


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SAI2_Init();
  /* USER CODE BEGIN 2 */
  /* Test SAI polling functionality */
  TestSAI_Polling();
  HAL_UART_Receive_IT(&huart1, &rx_data, 1); // Start UART reception in interrupt mode
  // Start SAI DMA reception
  HAL_SAI_Receive_DMA(&hsai_BlockB2, (uint8_t *)audioBuffer, AUDIO_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        if (command_received)
        {
            process_command(rx_buffer); // Process the received command
            command_received = false;  // Reset the flag
        }

        if (game_running)
        {
            if (HAL_SAI_Receive(&hsai_BlockB2, (uint8_t *)audioBuffer, AUDIO_BUFFER_SIZE, HAL_MAX_DELAY) == HAL_OK)
            {
                HAL_UART_Transmit(&huart1, (uint8_t *)"SAI Polling Success\r\n", 22, HAL_MAX_DELAY);
            }
            else
            {
                HAL_UART_Transmit(&huart1, (uint8_t *)"SAI Polling Failed\r\n", 21, HAL_MAX_DELAY);
            }

            // Additional audio processing logic can go here if needed.
            HAL_Delay(1000);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 6;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockB2.Instance = SAI2_Block_B;
  hsai_BlockB2.Init.AudioMode = SAI_MODESLAVE_RX;
  hsai_BlockB2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */

  /* USER CODE END SAI2_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GREEN_Pin|ORANGE_Pin|RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GREEN_Pin ORANGE_Pin RED_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin|ORANGE_Pin|RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TestSAI_Polling(void)
{
    int16_t testBuffer[16];  // Minimal test buffer
    HAL_UART_Transmit(&huart1, (uint8_t *)"Testing SAI Polling...\r\n", 25, HAL_MAX_DELAY);

    HAL_StatusTypeDef status = HAL_SAI_Receive(&hsai_BlockB2, (uint8_t *)testBuffer, 16, 1000);

    if (status == HAL_OK)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)"SAI Polling Success\r\n", 22, HAL_MAX_DELAY);
        for (int i = 0; i < 16; i++)
        {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "Data[%d]: %d\r\n", i, testBuffer[i]);
            HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
        }
    }
    else
    {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "SAI Polling Failed with status: %d\r\n", status);
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
    }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
    if (hsai->Instance == SAI2_Block_B)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)"DMA Transfer Complete\r\n", 25, HAL_MAX_DELAY);
    }
}

void ProcessAudio(void)
{

    // Convert int16_t data to float32_t
    for (int i = 0; i < AUDIO_BUFFER_SIZE; i++) {
        floatBuffer[i] = (float32_t)audioBuffer[i];
    }

    // Perform FFT
    arm_rfft_fast_instance_f32 fftInstance;
    arm_rfft_fast_init_f32(&fftInstance, AUDIO_BUFFER_SIZE);
    arm_rfft_fast_f32(&fftInstance, floatBuffer, fftOutput, 0);

    // Calculate magnitude of FFT output
    float32_t magnitude[AUDIO_BUFFER_SIZE / 2];
    for (int i = 0; i < AUDIO_BUFFER_SIZE / 2; i++) {
        magnitude[i] = sqrtf(fftOutput[2 * i] * fftOutput[2 * i] + fftOutput[2 * i + 1] * fftOutput[2 * i + 1]);
    }

    // Find the peak frequency
    uint32_t maxIndex;
    float32_t maxValue;
    arm_max_f32(magnitude, AUDIO_BUFFER_SIZE / 2, &maxValue, &maxIndex);

    float32_t peakFrequency = (SAMPLE_RATE / AUDIO_BUFFER_SIZE) * maxIndex;

    // Calculate the difference from the target frequency
    float32_t frequencyDifference = fabsf(peakFrequency - TARGET_FREQUENCY);

    // Print frequencyDifference to the console
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "FreqDiff: %.2f Hz\r\n", frequencyDifference);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    // Set LEDs based on the frequency difference
    SetLEDs(frequencyDifference);
}


void SetLEDs(float frequencyDifference)
{
    if (frequencyDifference > 200.0f) {
        // Far from the target frequency: Turn ON Red, OFF others
        HAL_GPIO_WritePin(GPIOI, GREEN_Pin, GPIO_PIN_SET);   // Turn OFF green
        HAL_GPIO_WritePin(GPIOI, ORANGE_Pin, GPIO_PIN_SET);  // Turn OFF orange
        HAL_GPIO_WritePin(GPIOI, RED_Pin, GPIO_PIN_RESET);   // Turn ON red
    } else if (frequencyDifference > 17.0f) {
        // Close to the target frequency: Turn ON Orange, OFF others
        HAL_GPIO_WritePin(GPIOI, GREEN_Pin, GPIO_PIN_SET);   // Turn OFF green
        HAL_GPIO_WritePin(GPIOI, ORANGE_Pin, GPIO_PIN_RESET);// Turn ON orange
        HAL_GPIO_WritePin(GPIOI, RED_Pin, GPIO_PIN_SET);     // Turn OFF red
    } else {
        // Very close to the target frequency: Turn ON Green, OFF others
        HAL_GPIO_WritePin(GPIOI, GREEN_Pin, GPIO_PIN_RESET); // Turn ON green
        HAL_GPIO_WritePin(GPIOI, ORANGE_Pin, GPIO_PIN_SET);  // Turn OFF orange
        HAL_GPIO_WritePin(GPIOI, RED_Pin, GPIO_PIN_SET);     // Turn OFF red
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
