
#include "stm32l4xx_hal.h" //Libreria HAL para Read/Write
extern I2C_HandleTypeDef hi2c2; //Handle del puerto I2C

void LPS22_Init(){

	uint8_t buffer[1];

	buffer[0] = 0x42;

	HAL_I2C_Mem_Write(&hi2c2, 0xBA, 0x10, I2C_MEMADD_SIZE_8BIT, buffer, 1, 1000);
}

float LPS22_ReadPress(){
	float press;

	uint8_t buffer[3]; //El sensor guarda la presión en 3 posiciones de memoria, por eso el buffer mas grande (se necesitan 24 bits)

	HAL_I2C_Mem_Read(&hi2c2, 0xBA, 0x28, I2C_MEMADD_SIZE_8BIT, buffer, 3, 1000); //Lectura de 3 registros seguidos

	uint32_t press_raw = (buffer[2]<<16 | buffer[1]<<8) | buffer[0]; //mascara para volcar la lectura en una sola variable

	press = press_raw/4096.0f; //El valor de la división va dado por el fabricante del sensor en la datasheet

	return press;
}
