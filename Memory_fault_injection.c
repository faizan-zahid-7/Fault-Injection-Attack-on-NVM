/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */
//for MRAM
//void FRAM_Bulk_Write(uint32_t startAddress, uint16_t numBytes, uint8_t data)
// uint8_t writeCmd[3] = {0x02, (startAddress >> 8) & 0x7F, startAddress & 0xFF};
//void FRAM_Bulk_Read(uint32_t startAddress, uint32_t numBytes, uint8_t* buffer) {
//    uint8_t readCmd[4] = {0x03, (startAddress >> 16) & 0xFF, (startAddress >> 8) & 0xFF, startAddress & 0xFF};
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */
#define FRAM_SIZE 30000
uint8_t readBuffer[FRAM_SIZE];
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
 // Buffer to store read data
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#include "stm32f10x.h" // Adjust as needed
#define START_ADDRESS  0x0000    // Starting address to write in FRAM
#define NUM_BYTES      30000// Total number of bytes to write
//#define DATA_BYTE      0x77     // The data byte to write (can be any value)
#define CHUNK_SIZE  100  // Safe for STM32F103 (adjustable)
#define TOTAL_BYTES 30000 // 30KB
////To add less delay
void DWT_Delay_us(uint32_t us) {
    uint32_t cycles = (SystemCoreClock / 1000000L) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable cycle counter
}


/////////////////Working fine can attack any of the half chunk you want/////////////////////
////////////////////////////////////////////////////////////////////////////////////
void FRAM_Bulk_Write(uint16_t startAddress, uint16_t numBytes, uint8_t data) {
    uint8_t writeEnableCmd = 0x06;  // WREN command
    uint8_t writeCmd[3];
    uint8_t dataBuf[CHUNK_SIZE];
    memset(dataBuf, data, CHUNK_SIZE);

    uint16_t bytesWritten = 0;
    uint8_t chunkIndex = 0;

    while (bytesWritten < numBytes) {
        // Enable write
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, &writeEnableCmd, 1, 100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);

        // Begin write command
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        writeCmd[0] = 0x02;
        writeCmd[1] = ((startAddress + bytesWritten) >> 8) & 0x7F;
        writeCmd[2] = (startAddress + bytesWritten) & 0xFF;
        HAL_SPI_Transmit(&hspi1, writeCmd, 3, 100);

        uint16_t currentChunk = (numBytes - bytesWritten > CHUNK_SIZE) ? CHUNK_SIZE : (numBytes - bytesWritten);

        if (chunkIndex == 100) {
            // Fault injection only on the second chunk ///Chunks 0-5
            int glitchBytes[] = {currentChunk / 2 - 1, currentChunk / 2, currentChunk / 2 + 1};
            int glitchCount = sizeof(glitchBytes) / sizeof(glitchBytes[0]);

            for (int i = 0; i < currentChunk; i++) {
                int isGlitchByte = 0;
                for (int j = 0; j < glitchCount; j++) {
                    if (i == glitchBytes[j]) {
                        isGlitchByte = 1;
                        break;
                    }
                }

                if (isGlitchByte) {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);     // Trigger impulse
                    DWT_Delay_us(100);  // Short pulse delay (adjust as needed)
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);   // Stop impulse
                }

                HAL_SPI_Transmit(&hspi1, &dataBuf[i], 1, 100);  // Send one byte
            }
        } else {
            // Normal write
            HAL_SPI_Transmit(&hspi1, dataBuf, currentChunk, 100);
        }

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);

        bytesWritten += currentChunk;
        chunkIndex++;
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)"Write Complete\r\n", 16, 100);
}

///////////////////////////////////////////////////////////////////
/////////////////////FRAM WRITE ONE BYTE//////////////////////////
//void FRAM_Bulk_Write_OneByte(uint16_t address, uint8_t dataByte) {
//    char uartMsg[100];
//    uint8_t writeEnableCmd = 0x06;  // WREN
//    uint8_t writeCmd[3];
//    uint8_t delay;
//
//    for (delay = 0; delay <= 20; delay += 2) {
//        // Write Enable
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
//        HAL_SPI_Transmit(&hspi1, &writeEnableCmd, 1, 100);
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
//        HAL_Delay(1);
//
//        // Prepare Write Command
//        writeCmd[0] = 0x02;
//        writeCmd[1] = (address >> 8) & 0x7F;
//        writeCmd[2] = address & 0xFF;
//
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
//        HAL_SPI_Transmit(&hspi1, writeCmd, 3, 100);
//
//        // Simulate Fault by triggering glitch during data transmission
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
//        DWT_Delay_us(delay);  // Adjustable delay before glitch
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
////        DWT_Delay_us(100);  // Pulse width
////        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
////
//        // Transmit data byte (with glitch)
//        HAL_SPI_Transmit(&hspi1, &dataByte, 1, 100);
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
//        HAL_Delay(1);
//    }
//
//    // Final confirmation message with address and data
//    snprintf(uartMsg, sizeof(uartMsg),
//             "Single Byte Write Glitch Sweep Done: Addr=0x%04X, Data=0x%02X\r\n",
//             address, dataByte);
//    HAL_UART_Transmit(&huart2, (uint8_t *)uartMsg, strlen(uartMsg), 100);
//}
//////working fine use for reading the whole memory///////////////////////
/////////////////////////////////////////////////////////////////////////
///////////////uint24_t startAddress for Mram and 16 for FRAM
void FRAM_Bulk_Read(uint16_t startAddress, uint16_t numBytes, uint8_t* buffer) {
    uint8_t readBuf[3];  // Buffer to store command and start address

    // Send READ command (0x03)
    readBuf[0] = 0x03;  // READ command
    readBuf[1] = (startAddress >> 8) & 0xFF;  // High byte of the address
    readBuf[2] = startAddress & 0xFF;  // Low byte of the address

    // Enable FRAM (CS low)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // Pull CS low

    // Transmit READ command and start address
    HAL_SPI_Transmit(&hspi1, readBuf, 3, 100);  // Send read command with address

    // Receive multiple bytes in one go
    HAL_SPI_Receive(&hspi1, buffer, numBytes, 1000);  // Receive numBytes of data

    // Disable FRAM (CS high)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // Pull CS high
}


///////////////////////////FOR ATTACKING ON READ//////////////////////////
//////////////////////////////////////////////////////////////////////
//void FRAM_Bulk_Read(uint16_t startAddress, uint16_t numBytes, uint8_t* buffer) {
//    uint8_t readBuf[3];
//
//    // Prepare read command
//    readBuf[0] = 0x03;
//    readBuf[1] = (startAddress >> 8) & 0xFF;
//    readBuf[2] = startAddress & 0xFF;
//
//    // Start communication
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // CS LOW
//
//    // Send read command
//    HAL_SPI_Transmit(&hspi1, readBuf, 3, 100);
//    HAL_SPI_Receive(&hspi1, buffer, numBytes, 1000);  // Receive numBytes of data
////Comment from here to remove pulse generator
//    for (uint16_t i = 0; i < numBytes; i++) {
//        // Trigger fault on specific bytes, e.g. byte 2 and 5
//        if (i == 2 || i == 3) {
//            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);  // Start fault
////            __NOP(); __NOP(); __NOP(); __NOP();                  // Short pulse
////            HAL_Delay(10); // Fault duration (adjust as needed)
//            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);  // End fault
//        }
//
//        // Read one byte at a time
//        HAL_SPI_Receive(&hspi1, &buffer[i], 1, 100);
//    }
////Comment till this point to make pin low of pulse generator
//    // End communication
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // CS HIGH
//}

//////////////////////////////////////////////////////////
/////////READ ONLY ONE BYTE /////////////////////////
//void FRAM_Read_OneByte(uint16_t startAddress, uint8_t* buffer) {
//    uint8_t readBuf[3];  // Buffer to store command and start address
//
//    // Send READ command (0x03)
//    readBuf[0] = 0x03;  // READ command
//    readBuf[1] = (startAddress >> 8) & 0xFF;  // High byte of the address
//    readBuf[2] = startAddress & 0xFF;  // Low byte of the address
//
//    // Enable FRAM (CS low)
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // Pull CS low
//
//    // Transmit READ command and start address
//    HAL_SPI_Transmit(&hspi1, readBuf, 3, 100);  // Send read command with address
//
//    // Receive only one byte
//    HAL_SPI_Receive(&hspi1, buffer, 1, 1000);  // Receive 1 byte of data
//
//    // Disable FRAM (CS high)
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // Pull CS high
//}

void FRAM_Print_Data() {
    char uartBuffer[50];

    // Print all data at once
    HAL_UART_Transmit(&huart2, (uint8_t *)"FRAM Read Data:\r\n", 17, 100);
    for (uint16_t i = 0; i < 10000; i++) {
        snprintf(uartBuffer, sizeof(uartBuffer), "Address 0x%04X: 0x%02X\r\n", i, readBuffer[i]);
        HAL_UART_Transmit(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer), 100);
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)"FRAM Read Completed\r\n", 22, 100);
}


/////////////Working fine help to inject fault on any chunk we want/////////////
///////////////////////////////////////////////////////////////////////////////
void FRAM_Test() {
    static uint8_t chunk[CHUNK_SIZE];  // Static buffer (global memory)
    uint32_t bytesVerified = 0;

    // Ensure pins are in correct state before starting
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // PB5 = VCC
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // PA0 = WP
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // PA1 = HOLD
    HAL_Delay(100); // short delay to stabilize
    // 1. Write test pattern
    for (int i = 0; i <= 100; i++)
    {
    	uint16_t currentAddress = START_ADDRESS;
    	for (uint16_t dataByte = 0x00; dataByte <= 0xFF; dataByte++) {
    	    char msg[50];
    	    snprintf(msg, sizeof(msg), "Writing byte: 0x%02X\r\n", dataByte);
    	    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    	    FRAM_Bulk_Write(currentAddress, NUM_BYTES, dataByte);

    	    currentAddress += NUM_BYTES; // Move to the next block of memory for next data byte
    	    HAL_Delay(100); // Optional: short delay between writes
    	}

    }

    //FRAM_Bulk_Write(START_ADDRESS, NUM_BYTES, DATA_BYTE);
////comment this 2nd part to remove the Read operation after the Write
//    // 2. Initial read and print
//    HAL_UART_Transmit(&huart2, (uint8_t*)"Initial Read:\r\n", 15, 100);
//    for (uint32_t addr = 0; addr < TOTAL_BYTES; addr += CHUNK_SIZE) {
//        uint32_t bytesLeft = TOTAL_BYTES - addr;
//        uint32_t chunkSize = (bytesLeft > CHUNK_SIZE) ? CHUNK_SIZE : bytesLeft;
//
//        FRAM_Bulk_Read(addr, chunkSize, chunk);
//
//        for (uint32_t i = 0; i < chunkSize; i++) {
//            char hexStr[7];
//            snprintf(hexStr, sizeof(hexStr), "0x%02X ", chunk[i]);
//            HAL_UART_Transmit(&huart2, (uint8_t*)hexStr, strlen(hexStr), 100);
//
//            if ((i + 1) % 16 == 0)
//                HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
//        }
//
//        bytesVerified += chunkSize;
//
//        char progress[32];
//        snprintf(progress, sizeof(progress), "\r\nVerified %u/%u bytes\r\n",
//                 (unsigned int)bytesVerified, (unsigned int)TOTAL_BYTES);
//        HAL_UART_Transmit(&huart2, (uint8_t*)progress, strlen(progress), 100);
//    }
//

    // 3. Simulate power/signal drop for fault injection
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nSimulating fault...\r\n", 24, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // PB5 low (VCC off)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // PA0 low (WP)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // PA1 low (HOLD)
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // CS low
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // SCK low
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // MISO low
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // MOSI low
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS low

    HAL_Delay(2000); // Wait 2 seconds

    // 4. Restore pins
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(100); // Short delay to stabilize
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nPins restored. Reading again...\r\n", 34, 100);

    // 5. Second read and print
    bytesVerified = 0;
    for (uint32_t addr = 0; addr < TOTAL_BYTES; addr += CHUNK_SIZE) {
        uint32_t bytesLeft = TOTAL_BYTES - addr;
        uint32_t chunkSize = (bytesLeft > CHUNK_SIZE) ? CHUNK_SIZE : bytesLeft;

        FRAM_Bulk_Read(addr, chunkSize, chunk);

        for (uint32_t i = 0; i < chunkSize; i++) {
            char hexStr[7];
            snprintf(hexStr, sizeof(hexStr), "0x%02X ", chunk[i]);
            HAL_UART_Transmit(&huart2, (uint8_t*)hexStr, strlen(hexStr), 100);

            if ((i + 1) % 16 == 0)
                HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
        }

        bytesVerified += chunkSize;

        char progress[32];
        snprintf(progress, sizeof(progress), "\r\nVerified %u/%u bytes\r\n",
                 (unsigned int)bytesVerified, (unsigned int)TOTAL_BYTES);
        HAL_UART_Transmit(&huart2, (uint8_t*)progress, strlen(progress), 100);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)"Done.\r\n", 7, 100);
}


////////To test only one byte in write operation
//#define TEST_ADDRESS 0x55F0     // Example test address
//#define TEST_DATA_BYTE 0xCC     // Example data byte to write
//
//void FRAM_Test_OneByte() {
//    uint8_t readData = 0;
//
//    // Ensure pins are in correct state before starting
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // PB5 = VCC
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // PA0 = WP
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // PA1 = HOLD
//    HAL_Delay(100); // short delay to stabilize
//
//    // 1. Write one byte at TEST_ADDRESS
//    FRAM_Bulk_Write_OneByte(TEST_ADDRESS, TEST_DATA_BYTE);
//
//    // 2. Simulate power/signal drop for fault injection
//    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nSimulating fault...\r\n", 24, 100);
//
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // PB5 low (VCC off)
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // PA0 low (WP)
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // PA1 low (HOLD)
//
//    HAL_Delay(2000); // Wait 2 seconds to simulate fault
//
//    // 3. Restore pins to normal
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//    HAL_Delay(100); // Short delay to stabilize
//
//    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nPins restored. Reading one byte...\r\n", 36, 100);
//
//    // 4. Read back one byte from TEST_ADDRESS
//    FRAM_Read_OneByte(TEST_ADDRESS, &readData);
//
//    // 5. Print the read data over UART
//    char hexStr[20];
//    snprintf(hexStr, sizeof(hexStr), "Read data: 0x%02X\r\n", readData);
//    HAL_UART_Transmit(&huart2, (uint8_t*)hexStr, strlen(hexStr), 100);
//
//    HAL_UART_Transmit(&huart2, (uint8_t*)"Test completed.\r\n", 16, 100);
//}


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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  DWT_Init();
  /* USER CODE END 2 */
  // Print info if needed

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //FRAM_Test_OneByte();
  FRAM_Test();
//  FRAM_Bulk_Write(START_ADDRESS, NUM_BYTES, DATA_BYTE);
//  for (int i =0; i<=100; i++)
//  {
//
//  }

  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
//  __disable_irq();
//
//  uint16_t errorCount = 0;
//  char buffer[64];
//
//  // Replace with actual length of data
//  uint16_t dataLength = YOUR_DATA_LENGTH;
//
//  // Replace with your actual write/read buffers
//  extern uint8_t expectedData[];
//  extern uint8_t readData[];
//
//  for (uint16_t i = 0; i < dataLength; i++) {
//    if (expectedData[i] != readData[i]) {
//      errorCount++;
//    }
//  }
//
//  // Send error count over UART
//  snprintf(buffer, sizeof(buffer), "Total byte mismatches: %u\r\n", errorCount);
//  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 100);
//
//  while (1) {
//    // Optionally blink an LED or stay stuck here
//  }
//
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
