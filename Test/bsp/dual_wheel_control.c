#include "dual_wheel_control.h"

// 初始化电机驱动
void dual_wheel_init(void) {
    HAL_GPIO_WritePin(LEFT_EN_GPIO_Port, LEFT_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LEFT_EN_REV_GPIO_Port, LEFT_EN_REV_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RIGHT_EN_GPIO_Port, RIGHT_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RIGHT_EN_REV_GPIO_Port, RIGHT_EN_REV_Pin, GPIO_PIN_RESET);
}

// 设置左轮 PWM 速度（正数前进，负数后退）
void set_left_pwm(int16_t speed) {
    if (speed > 0) { // 正转
        HAL_GPIO_WritePin(LEFT_EN_GPIO_Port, LEFT_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LEFT_EN_REV_GPIO_Port, LEFT_EN_REV_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, LEFT_PWM_CHANNEL, speed);
        __HAL_TIM_SET_COMPARE(&htim1, LEFT_PWM_REV_CHANNEL, 0);
    } else if (speed < 0) { // 反转
        HAL_GPIO_WritePin(LEFT_EN_GPIO_Port, LEFT_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEFT_EN_REV_GPIO_Port, LEFT_EN_REV_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, LEFT_PWM_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(&htim1, LEFT_PWM_REV_CHANNEL, -speed);
    } else { // 停止
        HAL_GPIO_WritePin(LEFT_EN_GPIO_Port, LEFT_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LEFT_EN_REV_GPIO_Port, LEFT_EN_REV_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, LEFT_PWM_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(&htim1, LEFT_PWM_REV_CHANNEL, 0);
    }
}

// 设置右轮 PWM 速度（正数前进，负数后退）
void set_right_pwm(int16_t speed) {
    if (speed > 0) { // 正转
         HAL_GPIO_WritePin(RIGHT_EN_GPIO_Port, RIGHT_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RIGHT_EN_REV_GPIO_Port, RIGHT_EN_REV_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim8, RIGHT_PWM_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(&htim8, RIGHT_PWM_REV_CHANNEL, speed);
    } else if (speed < 0) { // 反转
			  HAL_GPIO_WritePin(RIGHT_EN_GPIO_Port, RIGHT_EN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RIGHT_EN_REV_GPIO_Port, RIGHT_EN_REV_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim8, RIGHT_PWM_CHANNEL, -speed);
        __HAL_TIM_SET_COMPARE(&htim8, RIGHT_PWM_REV_CHANNEL, 0);
    } else { // 停止
        HAL_GPIO_WritePin(RIGHT_EN_GPIO_Port, RIGHT_EN_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RIGHT_EN_REV_GPIO_Port, RIGHT_EN_REV_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim8, RIGHT_PWM_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(&htim8, RIGHT_PWM_REV_CHANNEL, 0);
    }
}

// 平滑加速
void smooth_acceleration_dual(int16_t target_left_speed, int16_t target_right_speed, uint16_t delay_ms) {
    // 读取当前左轮速度（带方向）
    int16_t left_forward = __HAL_TIM_GET_COMPARE(&htim1, LEFT_PWM_CHANNEL);
    int16_t left_backward = __HAL_TIM_GET_COMPARE(&htim1, LEFT_PWM_REV_CHANNEL);
    int16_t current_left_speed = (left_forward > 0) ? left_forward : -left_backward;

    // 读取当前右轮速度（带方向）
    int16_t right_forward = __HAL_TIM_GET_COMPARE(&htim8, RIGHT_PWM_REV_CHANNEL);  // 正转是REV通道
    int16_t right_backward = __HAL_TIM_GET_COMPARE(&htim8, RIGHT_PWM_CHANNEL);     // 反转是正通道
    int16_t current_right_speed = (right_forward > 0) ? right_forward : -right_backward;

    // 循环直到左右轮都到达目标速度
    while (current_left_speed != target_left_speed || current_right_speed != target_right_speed) {
        // 左轮加速/减速
        if (current_left_speed < target_left_speed) {
            current_left_speed++;
        } else if (current_left_speed > target_left_speed) {
            current_left_speed--;
        }

        // 右轮加速/减速
        if (current_right_speed < target_right_speed) {
            current_right_speed++;
        } else if (current_right_speed > target_right_speed) {
            current_right_speed--;
        }

        // 更新PWM输出
        set_left_pwm(current_left_speed);
        set_right_pwm(current_right_speed);

        // 延时
        HAL_Delay(delay_ms);
    }
}

// 平滑减速（从 PWM 寄存器读取当前速度）
void smooth_deceleration(uint16_t delay_ms) {
    int16_t left_forward = __HAL_TIM_GET_COMPARE(&htim1, LEFT_PWM_CHANNEL);
    int16_t left_backward = __HAL_TIM_GET_COMPARE(&htim1, LEFT_PWM_REV_CHANNEL);
    int16_t right_forward = __HAL_TIM_GET_COMPARE(&htim8, RIGHT_PWM_CHANNEL);
    int16_t right_backward = __HAL_TIM_GET_COMPARE(&htim8, RIGHT_PWM_REV_CHANNEL);

    int16_t left_speed = (left_forward > 0) ? left_forward : left_backward;
    int16_t right_speed = (right_forward > 0) ? right_forward : right_backward;

    while (left_speed != 0 || right_speed != 0) {
        // 左轮减速
        if (left_speed > 0) {
            left_speed--;
        } else if (left_speed < 0) {
            left_speed++;
        }

        // 右轮减速  
				  
        if (right_speed > 0) {
            right_speed--;
        } else if (right_speed < 0) {
            right_speed++;
        }

        set_left_pwm(left_speed);
        set_right_pwm(right_speed);
        HAL_Delay(delay_ms);
    }
}
