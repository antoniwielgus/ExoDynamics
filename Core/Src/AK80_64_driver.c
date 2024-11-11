/**
 * @file AK80_64_driver.c
 * @author Antoni Wielgus
 * @date 12-10-2024
 */

#include "AK80_64_driver.h"
#include "main.h"


uint8_t startMotor[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
uint8_t stopMotor[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
uint8_t motorData[9] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t przod[8] = {158, 183, 127, 240, 24, 51, 55, 255};
uint8_t tyl[8] = {127, 255, 127, 240, 24, 51, 55, 255};


union float_uint8_t
{
	float float_num;
	uint8_t uint8_array[4];
} floatTOuint;

union float_uint16t
{
	float float_num;
	uint16_t uint16_array[2];
} floatTOint;


void startEngine(CAN_HandleTypeDef* hcan_, CanHandler* can_handler)
{
    //in reserve send this twice
    sendCanFrame(hcan_, can_id, can_handler, startMotor);
    sendCanFrame(hcan_, can_id, can_handler, startMotor);
}

void stopEngine(CAN_HandleTypeDef* hcan_, CanHandler* can_handler)
{
    sendCanFrame(hcan_, can_id, can_handler, stopMotor);
    sendCanFrame(hcan_, can_id, can_handler, stopMotor);
    sendCanFrame(hcan_, can_id, can_handler, stopMotor);
}

void engineDataSendRequest(CAN_HandleTypeDef* hcan_, CanHandler* can_handler)
{
    sendCanFrame(hcan_, can_id, can_handler, motorData);
}

int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
	float span = x_max - x_min;

	if (x < x_min)
		x = x_min;
	else if (x > x_max)
		x = x_max;

	return (int)((x - x_min) * ((float)((1 << bits) / span)));
}

void set_motor_possition(uint8_t* buffer, float p_des, float v_des, float kp, float kd, float t_ff)
{
	float P_MIN = -12.5f;
	float P_MAX = 12.5f;
	float V_MIN = -45.0f;
	float V_MAX = 45.0f;
	float T_MIN = -6.0f;
	float T_MAX = 6.0f;
	float Kp_MIN = 0.0f;
	float Kp_MAX = 500.0f;
	float Kd_MIN = 0.0f;
	float Kd_MAX = 5.0f;
	float Test_Pos = 0.0f;

	p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
	v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
	kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
	kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
	t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);

	int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
	int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
	int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
	int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
	int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

	buffer[0] = p_int>>8; 						// Position High 8
	buffer[1] = p_int&0xFF; 					// Position Low 8
	buffer[2] = v_int>>4; 						// Speed High 8 bits
	buffer[3] = ((v_int&0xF)<<4)|(kp_int>>8); 	// Speed Low 4 bits KP High 4 bits
	buffer[4] = kp_int&0xFF; 					// KP Low 8 bits
	buffer[5] = kd_int>>4; 						// Kd High 8 bits
	buffer[6] = ((kd_int&0xF)<<4)|(t_int>>8); 	// KP Low 4 bits Torque High 4 bits
	buffer[7] = t_int&0xff; 					// Torque Low 8 bits
}

void setPossition(uint8_t* buffer, float possition, float speed, float KP, float KD, float torque)
{
	for (int i = 0; i < 8; ++i)
		buffer[i] = 0;

	//possition
	floatTOint.float_num = possition;
	buffer[0] = floatTOint.uint16_array[0] >> 8; //position 8 higher
	buffer[1] = floatTOint.uint16_array[0] & 0xFF; //position 8 lower

	//speed
	floatTOuint.float_num = speed;
	buffer[2] = floatTOint.uint16_array[0] >> 4;
	buffer[3] = ((floatTOint.uint16_array[0] & 0xFF) << 4);

	//KP
	floatTOint.float_num = KP;
	buffer[3] = buffer[3] | (floatTOint.uint16_array[0] >> 8);
	buffer[4] = floatTOint.uint16_array[0] & 0xFF; //KP 8 bit lower

	//KD
	floatTOint.float_num = KD;
	buffer[5] = floatTOint.uint16_array[0] >> 4;
	buffer[6]  = (floatTOint.uint16_array[0] & 0xF) << 4;

	//current value
	floatTOint.float_num = KP;
	buffer[6] = buffer[6] | (floatTOint.uint16_array[0]  >> 8);
	floatTOint.float_num = torque;
	buffer[7] = floatTOint.uint16_array[0] & 0xFF;
}