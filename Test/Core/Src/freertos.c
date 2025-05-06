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
#define MAX_PITCH_ANGLE   25.0f
#define MAX_YAW_ANGLE     30.0f
#define MAX_SPEED         200.0f
#define DEAD_ZONE_PITCH   3.0f
#define DEAD_ZONE_YAW     3.0f
#define SPEED_SMOOTHING   0.15f
#define SENSOR_FILTER_ALPHA 0.1f
#define MAX_DELTA_SPEED 10.0f
#define MAX_PITCH_JUMP 10.0f   // pitch 跳变保护阈值
#define MAX_YAW_JUMP   10,.0f   // yaw 跳变保护阈值
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
volatile uint32_t last_frame_tick = 0;
float last_left_speed = 0;
float last_right_speed = 0;

static float filtered_pitch = 0.0f;
static float filtered_yaw = 0.0f;

// 用于异常跳变保护
static float last_raw_pitch = 0.0f;
static float last_raw_yaw = 0.0f;

// 添加一个复位函数
void reset_state(void) {
    last_left_speed = 0.0f;
    last_right_speed = 0.0f;
    filtered_pitch = 0.0f;
    filtered_yaw = 0.0f;
    last_raw_pitch = 0.0f;
    last_raw_yaw = 0.0f;
}

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
		

	

		for (;;)
		{
				if (xQueueReceive(uart_rx_queue, &frame1, pdMS_TO_TICKS(5)) == pdPASS)
				{
						float raw_pitch = frame1.f1;
						float raw_yaw   = frame1.f2;
						last_frame_tick = xTaskGetTickCount(); // 更新时间戳
						// 异常跳变保护：pitch/yaw 跳变过大直接丢弃
						if (fabsf(raw_pitch - last_raw_pitch) > MAX_PITCH_JUMP ||
								fabsf(raw_yaw   - last_raw_yaw)   > MAX_YAW_JUMP)
						{
								printf("[WARN] Data jump too large: ΔPitch=%.1f, ΔYaw=%.1f → Discarded\r\n",
											 raw_pitch - last_raw_pitch, raw_yaw - last_raw_yaw);
								continue; // 丢弃此帧
						}

						// 更新上一帧原始数据
						last_raw_pitch = raw_pitch;
						last_raw_yaw   = raw_yaw;

						// 传感器输入滤波
						filtered_pitch = SENSOR_FILTER_ALPHA * raw_pitch + (1.0f - SENSOR_FILTER_ALPHA) * filtered_pitch;
						filtered_yaw   = SENSOR_FILTER_ALPHA * raw_yaw   + (1.0f - SENSOR_FILTER_ALPHA) * filtered_yaw;

						float pitch_speed = 0.0f;
						if (fabsf(filtered_pitch) > DEAD_ZONE_PITCH)
						{
								pitch_speed = (filtered_pitch / MAX_PITCH_ANGLE) * MAX_SPEED;
								pitch_speed = fminf(fmaxf(pitch_speed, -MAX_SPEED), MAX_SPEED);
						}

						float yaw_adjust = 0.0f;
						if (fabsf(filtered_yaw) > DEAD_ZONE_YAW)
						{
								yaw_adjust = (filtered_yaw / MAX_YAW_ANGLE) * MAX_SPEED;
								yaw_adjust = fminf(fmaxf(yaw_adjust, -MAX_SPEED), MAX_SPEED);
						}

						float target_left_speed  = pitch_speed - yaw_adjust;
						float target_right_speed = pitch_speed + yaw_adjust;

						float delta_l = target_left_speed - last_left_speed;
						float delta_r = target_right_speed - last_right_speed;

						if (fabsf(delta_l) > MAX_DELTA_SPEED)
								target_left_speed = last_left_speed + MAX_DELTA_SPEED * ((delta_l > 0) ? 1 : -1);
						if (fabsf(delta_r) > MAX_DELTA_SPEED)
								target_right_speed = last_right_speed + MAX_DELTA_SPEED * ((delta_r > 0) ? 1 : -1);

						float left_speed = SPEED_SMOOTHING * target_left_speed + (1 - SPEED_SMOOTHING) * last_left_speed;
						float right_speed = SPEED_SMOOTHING * target_right_speed + (1 - SPEED_SMOOTHING) * last_right_speed;

						last_left_speed = left_speed;
						last_right_speed = right_speed;

						set_left_pwm((int16_t)left_speed);
						set_right_pwm((int16_t)right_speed);

						printf("Target L: %.1f, R: %.1f | FilteredPitch: %.1f, Raw: %.1f\r\n",
									 left_speed, right_speed, filtered_pitch, raw_pitch);
				}
				osDelay(1);
		}


  
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
  {//
		
		//读取一个mpu6050的加速度x y加速度值
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
				uint32_t now = xTaskGetTickCount();
        if ((now - last_frame_tick) > pdMS_TO_TICKS(1000)) // 超过1s未更新
        {
            set_left_pwm(0);
            set_right_pwm(0);
            reset_state(); // 重置状态
        }
        osDelay(1);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
