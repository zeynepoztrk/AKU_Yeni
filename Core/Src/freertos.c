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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LoRa.h"
#include "BMI088.h"
#include "bmp5_defs.h"
#include "bmp5.h"
#include "common.h"
#include "i2c.h"  // Bu dosyada hi2c3 tanımlıdır
#include "spi.h"
#include <string.h>
#include "fatfs.h"
#include "ff.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	SemaphoreHandle_t spiMutex;
	SemaphoreHandle_t sdMutex;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
BMI088 imu;
FIL file;
LoRa myLoRa;
uint8_t LoRa_stat = 0;
uint8_t buffer[128];
FRESULT res;
UINT bytesWritten;

float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
char logBuf[256]={0};

struct bmp5_sensor_data sensor_data;
extern float pressure, temperature;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId YRT_BMI088Handle;
osThreadId YRT_BMP581Handle;
osThreadId YRT_TRANSMITHandle;
osThreadId YRT_SDCARDHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void BMI088_fonk(void const * argument);
void BMP581_fonk(void const * argument);
void Transmit_veri(void const * argument);
void SDfonk(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
__weak void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

	spiMutex = xSemaphoreCreateMutex();
	sdMutex = xSemaphoreCreateMutex();
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of YRT_BMI088 */
  osThreadDef(YRT_BMI088, BMI088_fonk, osPriorityBelowNormal, 0, 512);
  YRT_BMI088Handle = osThreadCreate(osThread(YRT_BMI088), NULL);

  /* definition and creation of YRT_BMP581 */
  osThreadDef(YRT_BMP581, BMP581_fonk, osPriorityNormal, 0, 512);
  YRT_BMP581Handle = osThreadCreate(osThread(YRT_BMP581), NULL);

  /* definition and creation of YRT_TRANSMIT */
  osThreadDef(YRT_TRANSMIT, Transmit_veri, osPriorityAboveNormal, 0, 512);
  YRT_TRANSMITHandle = osThreadCreate(osThread(YRT_TRANSMIT), NULL);

  /* definition and creation of YRT_SDCARD */
  osThreadDef(YRT_SDCARD, SDfonk, osPriorityNormal, 0, 512);
  YRT_SDCARDHandle = osThreadCreate(osThread(YRT_SDCARD), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_BMI088_fonk */
/**
* @brief Function implementing the YRT_BMI088 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BMI088_fonk */
void BMI088_fonk(void const * argument)
{
  /* USER CODE BEGIN BMI088_fonk */
	 BMI088_Init_I2C(&imu, &hi2c3, 0x18 << 1, 0x69 << 1);
  /* Infinite loop */
  for(;;)
  {
	  BMI088_ReadAccelerometer(&imu);
	 	 	  BMI088_ReadGyroscope(&imu);
	 	 	  rollpitchyaw(&imu);
	 vTaskDelay(pdMS_TO_TICKS(50));
  }
  /* USER CODE END BMI088_fonk */
}

/* USER CODE BEGIN Header_BMP581_fonk */
/**
* @brief Function implementing the YRT_BMP581 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BMP581_fonk */
void BMP581_fonk(void const * argument)
{
  /* USER CODE BEGIN BMP581_fonk */
  /* Infinite loop */
  for(;;)
  {
	  BMP581();
	  vTaskDelay(pdMS_TO_TICKS(100));
  }
  /* USER CODE END BMP581_fonk */
}

/* USER CODE BEGIN Header_Transmit_veri */
/**
* @brief Function implementing the YRT_TRANSMIT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Transmit_veri */
void Transmit_veri(void const * argument)
{
  /* USER CODE BEGIN Transmit_veri */

	    // SPI ile Lora işlem
	 myLoRa = configureLoRa();
	 if(LoRa_init(&myLoRa)== LORA_OK){
	    LoRa_stat = 1;
	    }
	   LoRa_startReceiving(&myLoRa);

	   snprintf(logBuf,sizeof(logBuf), "ivme_x: %.3f,\nivme_y: %.3f,\nivme_z: %.3f,\nroll: %.3f,\npitch: %.3f,\nyaw: %.3f\n\r", imu.acc_mps2[0], imu.acc_mps2[1], imu.acc_mps2[2],
	   	 		  roll, pitch, yaw);

	   LoRa_transmit(&myLoRa, (uint8_t*)logBuf, strlen(logBuf), 10);
  /* Infinite loop */
  for(;;)
  {
	  snprintf(logBuf,sizeof(logBuf), "ivme_x: %.3f,\nivme_y: %.3f,\nivme_z: %.3f,\nroll: %.3f,\npitch: %.3f,\nyaw: %.3f\n\r", imu.acc_mps2[0], imu.acc_mps2[1], imu.acc_mps2[2],
	 	   	 		  roll, pitch, yaw);
	  if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {

	 	   LoRa_transmit(&myLoRa, (uint8_t*)logBuf, strlen(logBuf), 10);
    osDelay(1);
    xSemaphoreGive(spiMutex); // işi bitince bırak
    vTaskDelay(pdMS_TO_TICKS(200));
   }
  }
  /* USER CODE END Transmit_veri */
}

/* USER CODE BEGIN Header_SDfonk */
/**
* @brief Function implementing the YRT_SDCARD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SDfonk */
void SDfonk(void const * argument)
{
  /* USER CODE BEGIN SDfonk */

	bool isCardInserted = true;
	extern SD_HandleTypeDef hsd;
	  if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
	FATFS fs;
	FIL file;
	UINT bw;
	FRESULT res;
	 HAL_Delay(100);  // Kartı taktıktan sonra SDIO stabil hale gelsin

	    if (HAL_SD_Init(&hsd) != HAL_OK) {
	        printf("SD Init ERROR!\r\n");
	        vTaskDelete(NULL);
	    }

	    if (f_mount(&fs, "0:", 1) != FR_OK) {
	        printf("f_mount failed\r\n");
	        vTaskDelete(NULL);
	    }

	    if (f_open(&file, "test.txt", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
	        f_write(&file, "Hello SD Card!\r\n", 17, &bw);
	        f_close(&file);
	        printf("SD Write Success\r\n");
	    } else {
	        printf("f_open failed\r\n");
	    }

  /* Infinite loop */
  for(;;)
  {


	  	             // Dosyayı aç (yoksa oluştur)
	  	             res = f_open(&file, "zeynep.txt", FA_OPEN_APPEND | FA_WRITE);
	  	             if(res==FR_OK)
	  	             {
	  	            	f_write(&file, logBuf, strlen(logBuf), &bytesWritten);
	  	            	 f_close(&file);
	  	  	           xSemaphoreGive(sdMutex);

	  	             }
	  	         }
    osDelay(1);
  }
  /* USER CODE END SDfonk */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
