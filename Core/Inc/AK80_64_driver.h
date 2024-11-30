/**
 * @file AK80_64_driver.h
 * @author Antoni Wielgus
 * @brief This structure and functions are responsible for control motor driver.
 * @date 12-10-2024
 */

#ifndef AK80_64_DRIVER_H
#define AK80_64_DRIVER_H


#include "main.h"
#include "can_handler.h"


void startEngine(CAN_HandleTypeDef* hcan_, CanHandler* can_handler);
void stopEngine(CAN_HandleTypeDef* hcan_, CanHandler* can_handler);
void engineDataSendRequest(CAN_HandleTypeDef* hcan_, CanHandler* can_handler);

int32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
void set_motor_possition(uint8_t* buffer, float p_des, float v_des, float kp, float kd, float t_ff);
void setPossition(uint8_t* buffer, float possition, float speed, float KP, float KD, float torque);




#endif