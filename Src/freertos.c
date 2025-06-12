/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "cmsis_os.h"
#include "ssd1306.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306_fonts.h"
#include "FreeRTOSConfig.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    float temperature;
    float humidity;
    float pressure;
} SensorData_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
osThreadId_t BME280TaskHandle;
osThreadId_t DisplayTaskHandle;
osThreadId_t UARTTaskHandle;

QueueHandle_t sensorDataQueue;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
void StartBME280_Task(void *argument);
void StartDisplay_Task(void *argument);
void StartUART_Task(void *argument);

/* USER CODE BEGIN FunctionPrototypes */
extern void BME280_ReadValues(float* temperature, float* humidity, float* pressure);
extern void Debug_Printf(const char* format, ...);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    /* Create the queue */
    sensorDataQueue = xQueueCreate(5, sizeof(SensorData_t));
    if (sensorDataQueue == NULL) {
        Error_Handler();
    }

    /* Create the thread(s) */
    const osThreadAttr_t bme280Task_attributes = {
        .name = "BME280_Task",
        .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 256 * 4
    };
    BME280TaskHandle = osThreadNew(StartBME280_Task, NULL, &bme280Task_attributes);

    const osThreadAttr_t displayTask_attributes = {
        .name = "Display_Task",
        .priority = (osPriority_t) osPriorityBelowNormal,
        .stack_size = 384 * 4
    };
    DisplayTaskHandle = osThreadNew(StartDisplay_Task, NULL, &displayTask_attributes);

    const osThreadAttr_t uartTask_attributes = {
        .name = "UART_Task",
        .priority = (osPriority_t) osPriorityLow,
        .stack_size = 256 * 4
    };
    UARTTaskHandle = osThreadNew(StartUART_Task, NULL, &uartTask_attributes);
}

/* Task Definitions */
void StartBME280_Task(void *argument) {
    float temp, hum, press;
    SensorData_t data;
    for (;;) {
        BME280_ReadValues(&temp, &hum, &press);
        data.temperature = temp;
        data.humidity = hum;
        data.pressure = press;
        xQueueSend(sensorDataQueue, &data, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void StartDisplay_Task(void *argument) {
    SensorData_t data;
    char buffer[32];
    for (;;) {
        if (xQueueReceive(sensorDataQueue, &data, portMAX_DELAY) == pdPASS) {
            ssd1306_Fill(Black);
            ssd1306_SetCursor(0, 0);

            sprintf(buffer, "Temp: %.1f C", data.temperature);
            ssd1306_WriteString(buffer, Font_7x10, White);
            ssd1306_SetCursor(0, 12);

            sprintf(buffer, "Hum: %.1f %%", data.humidity);
            ssd1306_WriteString(buffer, Font_7x10, White);
            ssd1306_SetCursor(0, 24);

            sprintf(buffer, "Pres: %.1f hPa", data.pressure);
            ssd1306_WriteString(buffer, Font_7x10, White);

            ssd1306_UpdateScreen();
        }
    }
}

void StartUART_Task(void *argument) {
    SensorData_t data;
    for (;;) {
        if (xQueuePeek(sensorDataQueue, &data, portMAX_DELAY) == pdPASS) {
            Debug_Printf("T:%.1fC H:%.1f%% P:%.1fhPa\r\n", data.temperature, data.humidity, data.pressure);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
