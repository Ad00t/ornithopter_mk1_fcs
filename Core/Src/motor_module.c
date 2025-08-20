/*
 * encoder_motor_20d.c
 *
 *  Created on: Aug 17, 2025
 *      Author: adhit
 */

#include "motor_module.h"
#include "utils.h"
#include "logger.h"
#include <string.h>
#include <math.h>

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

HAL_StatusTypeDef i2c_master_tx(MotorModule* m, uint8_t* buf, uint16_t size) {
	uint8_t to_tx[size+1];
	memcpy(to_tx, buf, size);
	to_tx[size] = get_crc(buf, size);
	HAL_StatusTypeDef hstat = HAL_I2C_Master_Transmit(m->cfg.hi2c, m->cfg.i2c_addr, to_tx, size+1, HAL_MAX_DELAY);

	if (LOG_LEVEL >= LL_DEBUG) {
		char lbuf[MAX_LOG_LEN];
		int n = snprintf(lbuf, MAX_LOG_LEN, "MCU => 0x%02X -- [ ", m->cfg.i2c_addr);
		for (size_t i = 0; i < size+1; i++)
		    n += snprintf(lbuf + n, MAX_LOG_LEN - n, "0x%02X ", to_tx[i]);
		snprintf(lbuf + n, MAX_LOG_LEN - n, "] \r\n");
		log_debug(&dfl_logger, "%s", lbuf);
	}
	return hstat;
}

HAL_StatusTypeDef i2c_master_rx(MotorModule* m, uint8_t* buf, uint16_t size) {
	HAL_StatusTypeDef hstat = HAL_I2C_Master_Receive(m->cfg.hi2c, m->cfg.i2c_addr, buf, size, HAL_MAX_DELAY);
	uint8_t calcrc = get_crc(buf, size-1);

	if (LOG_LEVEL >= LL_DEBUG) {
		char lbuf[MAX_LOG_LEN];
		int n = snprintf(lbuf, MAX_LOG_LEN, "0x%02X => MCU -- [ ", m->cfg.i2c_addr);
		for (size_t i = 0; i < size+1; i++)
		    n += snprintf(lbuf + n, MAX_LOG_LEN - n, "0x%02X ", buf[i]);
		snprintf(lbuf + n, MAX_LOG_LEN - n, "] -- calcrc=0x%02X \r\n", calcrc);
		log_debug(&dfl_logger, "%s", lbuf);
	}
	return hstat;
}

/* General */

void MM_Init(MotorModule* m) {
	// Encoder init
	m->encoder_counts = 0;
	m->last_encoder_counts = 0;
	m->last_encoder_time = 0;
	m->angle = 0;
	m->rpm = 0;
	m->counter_period = __HAL_TIM_GET_AUTORELOAD(m->cfg.htim);
	HAL_TIM_Encoder_Start(m->cfg.htim, TIM_CHANNEL_ALL);

	// Driver init
	MMD_Reset(m);
	MMD_Reinitialize(m);
	MMD_Log_Firmware_Version(m);
	MMD_Clear_Reset(m);

	m->initialized = 1;
}

/* Encoder */

void MME_Update(MotorModule* m) {
	uint32_t t_ms = HAL_GetTick();
    m->encoder_counts = (int) __HAL_TIM_GET_COUNTER(m->cfg.htim);

    if (m->encoder_counts != m->last_encoder_counts) {
        m->angle = fmodf(MME_ANGLE_DEG * m->encoder_counts / MME_CPR, 360.0f);
        float dcount = m->encoder_counts - m->last_encoder_counts;
        float dt_s = (t_ms - m->last_encoder_time) / 1000.0f;
        m->rpm = (dcount / MME_CPR) / (dt_s / 60.0f);

        m->last_encoder_counts = m->encoder_counts;
        m->last_encoder_time = t_ms;
        log_debug(&dfl_logger, "ENCODER -- count=%u -- angle=%.3f -- rpm=%.3f \r\n", m->encoder_counts, m->angle, m->rpm);
    }
}

/* Driver */

void MMD_Log_Firmware_Version(MotorModule* m) {
	uint8_t cmd[1] = { MMD_CMD_GET_FIRMARE_VERSION };
	HAL_StatusTypeDef tx_hstat = i2c_master_tx(m, cmd, 1);
	uint8_t buf[5];
	HAL_StatusTypeDef rx_hstat = i2c_master_rx(m, buf, 5);
	log_info(&dfl_logger, "MM 0x%02X #%u DRIVER FIRMWARE -- 0x%04X v%u.%u CRC=0x%02X -- tx_hstat=0x%02X -- rx_hstat=0x%02X \r\n", m->cfg.i2c_addr, m->cfg.motor_num, (buf[1] << 8) | buf[0], hex_to_bcd(buf[3]), hex_to_bcd(buf[2]), buf[4], tx_hstat, rx_hstat);
}

void MMD_Log_Status_Flags(MotorModule* m) {
	uint8_t cmd[4] = { MMD_CMD_GET_VARIABLES, 0, MMD_VAR_STATUS_FLAGS, 2 };
	HAL_StatusTypeDef tx_hstat = i2c_master_tx(m, cmd, 4);
	uint8_t buf[3];
	HAL_StatusTypeDef rx_hstat = i2c_master_rx(m, buf, 3);
	uint16_t flag_word = ((uint16_t)buf[1] << 8) | buf[0];

	if (LOG_LEVEL >= LL_INFO) {
		char lbuf[MAX_LOG_LEN];
		int n = snprintf(lbuf, MAX_LOG_LEN, "DRIVER STATUS FLAGS -- ");
		for (size_t i = 0; i < 16; i++) {
			if (strcmp(status_flag_labels[i], "N/A") == 0)
				continue;
			n += snprintf(lbuf + n, MAX_LOG_LEN - n, "%s=%u ", status_flag_labels[i], (flag_word >> i) & 1);
		}
		snprintf(lbuf + n, MAX_LOG_LEN - n, "-- tx_hstat=0x%02X -- rx_hstat=0x%02X \r\n", tx_hstat, rx_hstat);
		log_info(&dfl_logger, "%s", lbuf);
	}
}

void MMD_Reset(MotorModule* m) {
	uint8_t cmd[1] = { MMD_CMD_RESET };
	HAL_StatusTypeDef hstat = i2c_master_tx(m, cmd, 1);
	log_info(&dfl_logger, "DRIVER RESET -- hstat=0x%02X \r\n", hstat);
	HAL_Delay(10);
}

void MMD_Reinitialize(MotorModule* m) {
	uint8_t cmd[1] = { MMD_CMD_REINITIALIZE };
	HAL_StatusTypeDef hstat = i2c_master_tx(m, cmd, 1);
	log_info(&dfl_logger, "DRIVER REINITIALIZE -- hstat=0x%02X \r\n", hstat);
}

void MMD_Clear_Reset(MotorModule* m) {
	uint8_t cmd[3] = { MMD_CMD_CLEAR_FLAGS, 0x00, 0x04 };
	HAL_StatusTypeDef hstat = i2c_master_tx(m, cmd, 3);
	log_info(&dfl_logger, "DRIVER CLEAR RESET -- hstat=0x%02X \r\n", hstat);
}

void MMD_Set_Speed(MotorModule* m, int16_t speed, uint8_t mode) {
	uint8_t cmd[4] = { mode, m->cfg.motor_num & 0x7F, speed & 0x7F, (speed >> 7) & 0x7F };
	HAL_StatusTypeDef hstat = i2c_master_tx(m, cmd, 4);
	log_debug(&dfl_logger, "DRIVER SET SPEED -- hstat=0x%02X \r\n", hstat);
}
