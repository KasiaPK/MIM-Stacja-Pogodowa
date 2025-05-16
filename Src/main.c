/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "bme280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BME280_ADDR 0x76 // lub 0x77 zależnie od konfiguracji czujnika
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
struct bme280_dev dev;
uint8_t dev_addr = BME280_ADDR;

// Dodatkowa struktura dla ustawień BME280
struct bme280_settings sensor_settings;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
void BME280_ReadValues(float* temperature, float* humidity, float* pressure);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Funkcje opakowujące HAL dla BME280
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    return (HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg_addr,
                             I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY) == HAL_OK) ? 0 : -1;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*)intf_ptr;
    return (HAL_I2C_Mem_Write(&hi2c1, dev_addr << 1, reg_addr,
                              I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, len, HAL_MAX_DELAY) == HAL_OK) ? 0 : -1;
}

// Funkcja opóźnienia dla BME280
void user_delay_us(uint32_t period, void *intf_ptr) {
    HAL_Delay(period / 1000 + 1); // Konwersja us -> ms z zaokrągleniem w górę
}

// Funkcja odczytu temperatury, wilgotności i ciśnienia z BME280
void BME280_ReadValues(float* temperature, float* humidity, float* pressure) {
    struct bme280_data data;
    int8_t rslt = bme280_get_sensor_data(BME280_ALL, &data, &dev);
    if (rslt == BME280_OK) {
        *temperature = data.temperature;
        *humidity = data.humidity;
        *pressure = data.pressure / 100.0f;  // Pa -> hPa
    }else {
        // W przypadku błędu odczytu, ustawiamy wartości domyślne
        *temperature = 0.0f;
        *humidity = 0.0f;
        *pressure = 0.0f;

        // Wyświetl kod błędu - do debugowania
        char error_msg[32];
        sprintf(error_msg, "BME280 Error: %d", rslt);
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString(error_msg, Font_7x10, White);
        ssd1306_UpdateScreen();
        HAL_Delay(1000);
    }
}

void Debug_Printf(const char* format, ...) {
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
}

void I2C_scan(void)
{
  uint8_t devices_found = 0;
  for (uint8_t i = 1; i < 128; i++)  // Przeszukiwanie możliwych adresów I2C
  {
    if (HAL_I2C_IsDeviceReady(&hi2c3, i << 1, 3, 10) == HAL_OK)
    {
      printf("Found device at address: 0x%02X\n", i);
      devices_found++;
    }
  }
  if (devices_found == 0)
  {
    printf("No devices found on I2C bus\n");
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
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  Debug_Printf("System initialized\r\n");
  // Inicjalizacja OLED
  ssd1306_Init();
  //I2C_scan();
  //ssd1306_Clear();
  ssd1306_SetCursor(0, 0);
  ssd1306_Fill(Black);
  ssd1306_WriteString("Hello", Font_7x10, White);

  ssd1306_SetCursor(5, 0);
  ssd1306_WriteString("OLED Test", Font_7x10, White);
  ssd1306_SetCursor(5, 15);
  ssd1306_WriteString("Line 2", Font_7x10, White);
  ssd1306_SetCursor(5, 30);
  ssd1306_WriteString("Line 3", Font_7x10, White);
  ssd1306_UpdateScreen();
  HAL_Delay(2000);
  //ssd1306_SetCursor(0, 0);
  //ssd1306_WriteString("BME280 Init!", Font_7x10, White);
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();

  // Inicjalizacja BME280
  int8_t rslt;

  // Konfiguracja interfejsu komunikacyjnego
  dev.intf_ptr = &dev_addr;
  dev.intf = BME280_I2C_INTF;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;
  dev.delay_us = user_delay_us;

  // Daj czas na stabilizację po włączeniu zasilania
  HAL_Delay(100);

  // Sprawdź czy można nawiązać komunikację z czujnikiem przed inicjalizacją
  uint8_t chip_id = 0;
  if (HAL_I2C_Mem_Read(&hi2c1, (BME280_ADDR << 1), 0xD0, I2C_MEMADD_SIZE_8BIT, &chip_id, 1, HAL_MAX_DELAY) != HAL_OK) {
      ssd1306_SetCursor(5, 10);
      ssd1306_WriteString("I2C comm error!", Font_7x10, White);
      ssd1306_UpdateScreen();
      HAL_Delay(2000);
  } else {
      // Wyświetl ID czujnika, powinno być 0x60 dla BME280
      char id_msg[32];
      sprintf(id_msg, "Chip ID: 0x%02X", chip_id);
      ssd1306_SetCursor(5, 10);
      ssd1306_WriteString(id_msg, Font_7x10, White);
      ssd1306_UpdateScreen();
      HAL_Delay(1000);
  }

  rslt = bme280_init(&dev);
  if (rslt != BME280_OK) {
      // Error handling
      char error_msg[32];
      sprintf(error_msg, "BME280 init err: %d", rslt);
      ssd1306_SetCursor(5, 20);
      ssd1306_WriteString(error_msg, Font_7x10, White);
      ssd1306_UpdateScreen();
      HAL_Delay(2000);
  } else {
      ssd1306_SetCursor(5, 20);
      ssd1306_WriteString("BME280 OK!", Font_7x10, White);
      ssd1306_UpdateScreen();
      HAL_Delay(1000);
  }

  // Konfiguracja trybu pomiaru
  HAL_Delay(100); // Daj czujnikowi czas po inicjalizacji




  // Wyświetl komunikat o konfiguracji
  ssd1306_Fill(Black);
  ssd1306_SetCursor(5, 10);
  ssd1306_WriteString("Config BME280...", Font_7x10, White);
  ssd1306_UpdateScreen();

  // Konfiguracja ustawień czujnika
  sensor_settings.osr_h = BME280_OVERSAMPLING_1X;
  sensor_settings.osr_p = BME280_OVERSAMPLING_16X;
  sensor_settings.osr_t = BME280_OVERSAMPLING_2X;
  sensor_settings.filter = BME280_FILTER_COEFF_16;
  sensor_settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

  // Ustawienie konfiguracji czujnika
  rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &sensor_settings, &dev);
  if (rslt != BME280_OK) {
        char error_msg[32];
        sprintf(error_msg, "Settings err: %d", rslt);
        ssd1306_SetCursor(5, 10);
        ssd1306_WriteString(error_msg, Font_7x10, White);
        ssd1306_UpdateScreen();
        HAL_Delay(2000);
  } else {
        ssd1306_SetCursor(5, 10);
        ssd1306_WriteString("Settings OK!", Font_7x10, White);
        ssd1306_UpdateScreen();
  }

  // Ustawienie trybu pracy czujnika
  HAL_Delay(100);
  rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev);
  if (rslt != BME280_OK) {
      char error_msg[32];
      sprintf(error_msg, "Mode err: %d", rslt);
      ssd1306_SetCursor(5, 20);
      ssd1306_WriteString(error_msg, Font_7x10, White);
      ssd1306_UpdateScreen();
      HAL_Delay(2000);
  } else {
      ssd1306_SetCursor(5, 20);
      ssd1306_WriteString("Mode OK!", Font_7x10, White);
      ssd1306_UpdateScreen();
  }

  // Odczekaj czas potrzebny na pierwszą konwersję
  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float temperature, humidity, pressure;
  char buffer[32];

  while (1)
  {
	  BME280_ReadValues(&temperature, &humidity, &pressure);
	  // After OLED initialization, add:
	  Debug_Printf("OLED initialization complete\r\n");

	  // After BME280 chip ID read, add:
	  Debug_Printf("BME280 Chip ID: 0x%02X\r\n", chip_id);

	  // After BME280 initialization, add:
	  Debug_Printf("BME280 init result: %d\r\n", rslt);

	  // In your main loop, add:
	  Debug_Printf("Temp: %.2f C, Humid: %.1f %%, Press: %.1f hPa\r\n",
	               temperature, humidity, pressure);

	  ssd1306_Fill(Black);

	  sprintf(buffer, "Temp: %.2f C", temperature);
	  ssd1306_SetCursor(5, 12);
	  ssd1306_WriteString(buffer, Font_7x10, White);

	  sprintf(buffer, "Wilg: %.1f %%", humidity);
	  ssd1306_SetCursor(5, 24);
	  ssd1306_WriteString(buffer, Font_7x10, White);

	  sprintf(buffer, "Cisn: %.1f hPa", pressure);
	  ssd1306_SetCursor(5, 36);
	  ssd1306_WriteString(buffer, Font_7x10, White);

	  ssd1306_UpdateScreen();
	  HAL_Delay(1000);

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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
