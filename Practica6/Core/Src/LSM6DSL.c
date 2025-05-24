/*
 * LM6DSL.c
 *
 *  Created on: Mar 3, 2025
 *      Author: paco-pepe
 */

#include "stm32l4xx_hal.h"
#include "LSM6DSL.h"
extern I2C_HandleTypeDef hi2c2;

void LSM6DSL_Init(){
	uint8_t buffer[1];

		buffer[0] = 0x40;

		HAL_I2C_Mem_Write(&hi2c2, 0xD4, 0x10, I2C_MEMADD_SIZE_8BIT, buffer, 1, 1000);
}

accel LSM6DSL_Read_All(){
	accel salida;
	uint8_t buffer[6];
	HAL_I2C_Mem_Read(&hi2c2, 0xD4, 0x28, I2C_MEMADD_SIZE_8BIT, buffer, 6, 1000);

	salida.Xaxis = ((int16_t)buffer[1]<<8|buffer[0])*0.061f;
	salida.Yaxis = ((int16_t)buffer[3]<<8|buffer[2])*0.061f;
	salida.Zaxis = ((int16_t)buffer[5]<<8|buffer[4])*0.061f;

	return salida;
}

int16_t LSM6DSL_ReadAccel(uint8_t axxis){
	int16_t salida;
	uint8_t buffer[2];

	HAL_I2C_Mem_Read(&hi2c2, 0xD4, 0x28+2*axxis, I2C_MEMADD_SIZE_8BIT, buffer, 2, 1000);
	salida = ((int16_t)(buffer[1]<<8) | buffer[0])*0.061f;


	return salida;
}
