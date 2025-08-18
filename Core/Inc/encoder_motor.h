/*
 * encoder_motor.h
 *
 *  Created on: Aug 17, 2025
 *      Author: adhit
 */

#ifndef ENCODER_MOTOR_H
#define ENCODER_MOTOR_H

#include "stm32f4xx_hal.h"

/* Commands */
#define EM_CMD_GET_FIRMARE_VERSION  	0x87
#define EM_CMD_SET_SPEED_NORMAL 		0xD1
#define EM_CMD_SET_SPEED_NOW 			0xD2
#define EM_CMD_SET_SPEED_BUFFERED 		0xD4
#define EM_CMD_RESET					0x99
#define EM_CMD_REINITIALIZE				0x96
#define EM_CMD_GET_VARIABLES			0x9A
#define EM_CMD_CLEAR_FLAGS				0xA9

/* Variable offsets */
#define EM_VAR_STATUS_FLAGS 	1

/* Configuration structure */
typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint16_t i2c_addr;
    uint8_t	motor_num;
} EM_Config;

/* Instance */
typedef struct {
    EM_Config cfg;
    uint8_t initialized;
} EncoderMotor;

/* API */
void EM_Init(EncoderMotor* m);
void EM_Log_Firmware_Version(EncoderMotor* m);
void EM_Log_Status_Flags(EncoderMotor* m);
void EM_Reset(EncoderMotor* m);
void EM_Reinitialize(EncoderMotor* m);
void EM_Clear_Reset(EncoderMotor* m);
void EM_Set_Speed(EncoderMotor* m, int16_t speed, uint8_t mode);

#endif /* ENCODER_MOTOR_H */
