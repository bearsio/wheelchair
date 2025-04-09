#ifndef DUAL_WHEEL_CONTROL_H
#define DUAL_WHEEL_CONTROL_H

#include "tim.h"
#include "gpio.h"
#include "main.h"
// 左轮PWM & 使能引脚（PE口）
#define LEFT_PWM_CHANNEL TIM_CHANNEL_2  // PE11
#define LEFT_EN_GPIO_Port GPIOE
#define LEFT_EN_Pin GPIO_PIN_12  // PE12 使能
#define LEFT_PWM_REV_CHANNEL TIM_CHANNEL_4  // PE14 反转
#define LEFT_EN_REV_GPIO_Port GPIOE
#define LEFT_EN_REV_Pin GPIO_PIN_13  // PE13 反转使能

// 右轮PWM & 使能引脚（PC口）
#define RIGHT_PWM_CHANNEL TIM_CHANNEL_4  // PC9
#define RIGHT_EN_GPIO_Port GPIOC
#define RIGHT_EN_Pin GPIO_PIN_8  // PC8 使能
#define RIGHT_PWM_REV_CHANNEL  TIM_CHANNEL_2 // PC7 反转
#define RIGHT_EN_REV_GPIO_Port GPIOC
#define RIGHT_EN_REV_Pin GPIO_PIN_6  // PC6 反转使能

void dual_wheel_init(void);
void set_motor_pwm(int16_t speed);
void smooth_acceleration_dual(int16_t target_left_speed, int16_t target_right_speed, uint16_t delay_ms) ;
void smooth_deceleration(uint16_t delay_ms);
void set_left_pwm(int16_t speed);
void set_right_pwm(int16_t speed);
#endif // DUAL_WHEEL_CONTROL_H
