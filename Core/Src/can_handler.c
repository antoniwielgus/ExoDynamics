/**
 * @file can_config.c
 * @author Antoni Wielgus 
 * @date 12-10-2024
 */

#include "can_handler.h"


CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;
uint32_t txMailbox;

//TODO: default initialization
// void defaultInitialization(CanHandler* can_handler)
// {
    
// }

void initCanHandler(CanHandler* can_handler, uint32_t dlc, uint32_t ext_id, uint32_t ide, uint32_t rtr, uint32_t std_id, FunctionalState transmit_global_time)
{
    can_handler->txHeader.DLC = dlc;
    can_handler->txHeader.ExtId = ext_id;
    can_handler->txHeader.IDE = ide;
    can_handler->txHeader.RTR = rtr;
    can_handler->txHeader.StdId = std_id;
    can_handler->txHeader.TransmitGlobalTime = transmit_global_time;
}


void sendCanFrame(CAN_HandleTypeDef* hcan_, uint8_t can_id, CanHandler* can_handler, uint8_t* frame)
{
    can_handler->txHeader.ExtId = can_id;
    can_handler->txHeader.StdId = can_id;

    HAL_CAN_AddTxMessage(hcan_, &can_handler->txHeader, frame, &can_handler->txMailbox);
}


