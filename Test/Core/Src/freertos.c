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
#include "tim.h"
#include "dual_wheel_control.h"
#include "queue.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_PITCH_ANGLE   20.0f
#define MAX_YAW_ANGLE     20.0f
#define MAX_SPEED         100.0f
#define DEAD_ZONE_PITCH   3.0f
#define DEAD_ZONE_YAW     3.0f
#define SPEED_SMOOTHING   0.15f

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Pid_ControlHandle;
osThreadId ReciveHandle;
osThreadId Mpu_6050Handle;
osThreadId SafetyHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
QueueHandle_t uart_rx_queue;
extern uint8_t  buff_data;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

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
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	 uart_rx_queue = xQueueCreate(128, sizeof(UartFrame_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Pid_Control */
  osThreadDef(Pid_Control, StartDefaultTask, osPriorityHigh, 0, 128);
  Pid_ControlHandle = osThreadCreate(osThread(Pid_Control), NULL);

  /* definition and creation of Recive */
  osThreadDef(Recive, StartTask02, osPriorityRealtime, 0, 128);
  ReciveHandle = osThreadCreate(osThread(Recive), NULL);

  /* definition and creation of Mpu_6050 */
  osThreadDef(Mpu_6050, StartTask04, osPriorityBelowNormal, 0, 128);
  Mpu_6050Handle = osThreadCreate(osThread(Mpu_6050), NULL);

  /* definition and creation of Safety */
  osThreadDef(Safety, StartTask05, osPriorityRealtime, 0, 128);
  SafetyHandle = osThreadCreate(osThread(Safety), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
	uart
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	
	
	UartFrame_t frame1;
	// 用于滤波缓存上一次速度值
  float last_left_speed = 0;
  float last_right_speed = 0;
  
	
	/* Infinite loop */
  for(;;)
  {
		
		 if (xQueueReceive(uart_rx_queue, &frame1, portMAX_DELAY) == pdPASS) 
			 {
							
					 float pitch = frame1.f1;
					 float yaw = frame1.f2;
					 
					 //俯仰角控制前进后退速度
					 
					 float pitch_speed = 0.0f;
					 if(fabsf(pitch)>DEAD_ZONE_PITCH)
					 {
							pitch_speed = (pitch/MAX_PITCH_ANGLE)*MAX_SPEED;
						 //边界值
						 if (pitch_speed >  MAX_SPEED) pitch_speed =  MAX_SPEED;
						 if (pitch_speed < -MAX_SPEED) pitch_speed = -MAX_SPEED;
					 }
					 //偏航角控制左右差速
					 
					 float yaw_adjust = 0.0f;
					if (fabsf(yaw) > DEAD_ZONE_YAW)
					{
						yaw_adjust = (yaw / MAX_YAW_ANGLE) * MAX_SPEED;
						if (yaw_adjust >  MAX_SPEED) yaw_adjust =  MAX_SPEED;
						if (yaw_adjust < -MAX_SPEED) yaw_adjust = -MAX_SPEED;
					}
						// ---------------- 差速合成 ----------------
					float left_speed  = pitch_speed - yaw_adjust;
					float right_speed = pitch_speed + yaw_adjust;
				 
					 // 限幅
					if (left_speed >  MAX_SPEED) left_speed = MAX_SPEED;
					if (left_speed < -MAX_SPEED) left_speed = -MAX_SPEED;
					if (right_speed >  MAX_SPEED) right_speed = MAX_SPEED;
					if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;

					// ---------------- 一阶滤波平滑 ----------------
					left_speed  = SPEED_SMOOTHING * left_speed  + (1 - SPEED_SMOOTHING) * last_left_speed;
					right_speed = SPEED_SMOOTHING * right_speed + (1 - SPEED_SMOOTHING) * last_right_speed;

					last_left_speed  = left_speed;
					last_right_speed = right_speed;

						// ----- 输出PWM -----
					set_left_pwm((int16_t)left_speed);
					set_right_pwm((int16_t)right_speed);
					
					printf("Target L: %.1f, R: %.1f | Last L: %.1f, R: %.1f\r\n",
							 left_speed, right_speed,
							 last_left_speed, last_right_speed);
						 
			 }
			 
		}
	
  
    osDelay(1);
  
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
   PID
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Mpu_6050 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the Safety thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {  
    osDelay(1);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
