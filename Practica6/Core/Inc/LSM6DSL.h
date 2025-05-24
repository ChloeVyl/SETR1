/*
 * LSM6DSL.h
 *
 *  Created on: Mar 3, 2025
 *      Author: paco-pepe
 */

#ifndef INC_LSM6DSL_H_
#define INC_LSM6DSL_H_

typedef struct{
	int16_t Xaxis;
	int16_t Yaxis;
	int16_t Zaxis;
}accel;

void LSM6DSL_Init();

accel LSM6DSL_Read();

#endif /* INC_LSM6DSL_H_ */
