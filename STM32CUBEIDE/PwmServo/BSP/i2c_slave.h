/**
 * @file i2c_slave.h
 * @brief Header file for I2C slave functionality on STM32F1.
 * @author turu
 * @date Sep 30, 2025
 */

#ifndef I2C_SLAVE_H_
#define I2C_SLAVE_H_

/* Includes */
#include "bsp.h"
#include "stm32f1xx_hal.h"

/* Externs */
extern I2C_HandleTypeDef hi2c2;
extern RingBuffer rxRingBuffer;
/* Function Prototypes */
void I2C_Slave_Init(void);
uint8_t I2C_Data_Ready(void);
uint8_t I2C_Get_Next_Command(Command_t *cmd);
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t dir, uint16_t addrMatchCode);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

#endif /* I2C_SLAVE_H_ */
