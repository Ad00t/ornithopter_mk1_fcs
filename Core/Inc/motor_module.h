/*
 * encoder_motor.h
 *
 *  Created on: Aug 17, 2025
 *      Author: adhit
 */

#ifndef MOTOR_MODULE_H
#define MOTOR_MODULE_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Encoder */
#define MME_PPR							20.0f
#define MME_GEAR_RATIO 					25.0f
#define MME_CPR 						(MME_PPR * MME_GEAR_RATIO)
#define MME_ANGLE_DEG					360.0f
#define MME_ANGLE_RAD					2.0f * 3.14159265359f

/* Driver Commands */
#define MMD_CMD_GET_FIRMARE_VERSION  	0x87
#define MMD_CMD_SET_SPEED_NORMAL 		0xD1
#define MMD_CMD_SET_SPEED_NOW 			0xD2
#define MMD_CMD_SET_SPEED_BUFFERED 		0xD4
#define MMD_CMD_RESET					0x99
#define MMD_CMD_REINITIALIZE			0x96
#define MMD_CMD_GET_VARIABLES			0x9A
#define MMD_CMD_CLEAR_FLAGS				0xA9

/* Driver Variable Offsets */
#define MMD_VAR_STATUS_FLAGS 			1

/* Configuration structure */
typedef struct {
    TIM_HandleTypeDef *htim;
    I2C_HandleTypeDef *hi2c;
    uint16_t i2c_addr;
    uint8_t	motor_num;
} MMD_Config;

/* Instance */
typedef struct {
    MMD_Config cfg;
    int last_encoder_counts;
    uint32_t last_encoder_time;
    float angle;
    float rpm;
    uint32_t counter_period;
} MotorModule;

/* General */
void MM_Init(MotorModule *m);

/* Encoder */
void MME_Update(MotorModule *m);

/* Driver */
void MMD_Log_Firmware_Version(MotorModule *m);
void MMD_Log_Status_Flags(MotorModule *m);
void MMD_Reset(MotorModule *m);
void MMD_Reinitialize(MotorModule *m);
void MMD_Clear_Reset(MotorModule *m);
void MMD_Set_Speed(MotorModule *m, int16_t speed, uint8_t mode);

#endif /* MOTOR_MODULE_H */
