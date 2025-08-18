/*
 * encoder_motor_20d.c
 *
 *  Created on: Aug 17, 2025
 *      Author: adhit
 */

#include "encoder_motor.h"
#include "utils.h"
#include <string.h>

const char* status_flag_labels[] = {
    "p_err", "crc_err", "cmd_time_l", "m_fault_l", "no_pow_l", "uart_err", "N/A", "N/A", "N/A", "rst", "cmd_time", "m_faulting", "no_pow", "err_active", "m_out_en", "m_driving"
};

/* I2C helpers */
uint8_t get_crc(uint8_t* message, uint8_t length) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < length; i++) {
		crc ^= message[i];
		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 1) crc ^= 0x91;
			crc >>= 1;
		}
	}
	return crc;
}

HAL_StatusTypeDef i2c_master_tx(EncoderMotor* m, uint8_t* buf, uint16_t size) {
	uint8_t to_tx[size+1];
	memcpy(to_tx, buf, size);
	to_tx[size] = get_crc(buf, size);
	HAL_StatusTypeDef hstat = HAL_I2C_Master_Transmit(m->cfg.hi2c, m->cfg.i2c_addr, to_tx, size+1, HAL_MAX_DELAY);

	if (LOG_LEVEL >= LL_DEBUG) {
		LOG(LL_DEBUG, "MCU => 0x%02X -- [ ", m->cfg.i2c_addr);
		for (size_t i = 0; i < size+1; i++)
			printf("0x%02X ", to_tx[i]);
		printf("]\r\n");
	}
	return hstat;
}

HAL_StatusTypeDef i2c_master_rx(EncoderMotor* m, uint8_t* buf, uint16_t size) {
	HAL_StatusTypeDef hstat = HAL_I2C_Master_Receive(m->cfg.hi2c, m->cfg.i2c_addr, buf, size, HAL_MAX_DELAY);
	uint8_t calcrc = get_crc(buf, size-1);

	if (LOG_LEVEL >= LL_DEBUG) {
		LOG(LL_DEBUG, "0x%02X => MCU -- [ ", m->cfg.i2c_addr);
		for (size_t i = 0; i < size; i++)
			printf("0x%02X ", buf[i]);
		printf("] -- CALCRC=0x%02X\r\n", calcrc);
	}
	return hstat;
}

/* API */
void EM_Init(EncoderMotor* m) {
	m->initialized = 1;
	EM_Reset(m);
	EM_Reinitialize(m);
	EM_Log_Firmware_Version(m);
	EM_Clear_Reset(m);
}

void EM_Log_Firmware_Version(EncoderMotor* m) {
	uint8_t cmd[1] = { EM_CMD_GET_FIRMARE_VERSION };
	HAL_StatusTypeDef tx_hstat = i2c_master_tx(m, cmd, 1);
	uint8_t buf[5];
	HAL_StatusTypeDef rx_hstat = i2c_master_rx(m, buf, 5);
	LOGLN(LL_INFO, "EM1 Driver -- 0x%04X v%u.%u CRC=0x%02X -- tx_hstat=0x%02X -- rx_hstat=0x%02X", (buf[1] << 8) | buf[0], HEX_TO_BCD(buf[3]), HEX_TO_BCD(buf[2]), buf[4], tx_hstat, rx_hstat);
}

void EM_Log_Status_Flags(EncoderMotor* m) {
	uint8_t cmd[4] = { EM_CMD_GET_VARIABLES, 0, EM_VAR_STATUS_FLAGS, 2 };
	HAL_StatusTypeDef tx_hstat = i2c_master_tx(m, cmd, 4);
	uint8_t buf[3];
	HAL_StatusTypeDef rx_hstat = i2c_master_rx(m, buf, 3);
	uint16_t flag_word = ((uint16_t)buf[1] << 8) | buf[0];

	if (LOG_LEVEL >= LL_INFO) {
		LOG(LL_INFO, "Status Flags -- ");
		for (size_t i = 0; i < 16; i++) {
			if (strcmp(status_flag_labels[i], "N/A") == 0)
				continue;
			printf("%s=%u ", status_flag_labels[i], (flag_word >> i) & 1);
		}
		printf("-- tx_hstat=0x%02X -- rx_hstat=0x%02X\r\n", tx_hstat, rx_hstat);
	}
}

void EM_Reset(EncoderMotor* m) {
	uint8_t cmd[1] = { EM_CMD_RESET };
	HAL_StatusTypeDef hstat = i2c_master_tx(m, cmd, 1);
	LOGLN(LL_INFO, "Reset -- hstat=0x%02X", hstat);
	HAL_Delay(10);
}

void EM_Reinitialize(EncoderMotor* m) {
	uint8_t cmd[1] = { EM_CMD_REINITIALIZE };
	HAL_StatusTypeDef hstat = i2c_master_tx(m, cmd, 1);
	LOGLN(LL_INFO, "Reinitialize -- hstat=0x%02X", hstat);
}

void EM_Clear_Reset(EncoderMotor* m) {
	uint8_t cmd[3] = { EM_CMD_CLEAR_FLAGS, 0x00, 0x04 };
	HAL_StatusTypeDef hstat = i2c_master_tx(m, cmd, 3);
	LOGLN(LL_INFO, "Clear Rst -- hstat=0x%02X", hstat);
}

void EM_Set_Speed(EncoderMotor* m, int16_t speed, uint8_t mode) {
	uint8_t cmd[4] = { mode, m->cfg.motor_num & 0x7F, speed & 0x7F, (speed >> 7) & 0x7F };
	HAL_StatusTypeDef hstat = i2c_master_tx(m, cmd, 4);
	LOGLN(LL_INFO, "Set Speed -- hstat=0x%02X", hstat);
}
