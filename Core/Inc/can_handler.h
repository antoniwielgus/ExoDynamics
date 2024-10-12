/**
 * @file can_handler.h
 * @author Antoni Wielgus
 * @brief initialization can
 * @date 12-10-2024
 */

#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H


#include "main.h"
#include "stm32f4xx_hal_can.h"


typedef struct 
{
    CAN_TxHeaderTypeDef txHeader;
    CAN_RxHeaderTypeDef rxHeader;
    uint32_t txMailbox;
    uint8_t txData[8];
    uint8_t rxData[8];
} CanHandler;


//functions 
void defaultInitialization(CanHandler* can_handler);
void initCanHandler(CanHandler* can_handler, uint32_t dlc, uint32_t ext_id, uint32_t ide, uint32_t rtr, uint32_t std_id, FunctionalState transmit_global_time);
void canStart(CAN_HandleTypeDef* hcan_);
void sendCanFrame(CAN_HandleTypeDef* hcan_, CanHandler* can_handler, uint8_t* frame);


#endif