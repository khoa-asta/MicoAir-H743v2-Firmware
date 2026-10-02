/*
 * qmc5883l.h
 *
 *      Author: Viết Khoa
 */

#ifndef DEVICE_QMC5883L_QMC5883L_H_
#define DEVICE_QMC5883L_QMC5883L_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "main.h"
#include <stdint.h>

/* Exported types */
typedef struct
{
	int16_t x_raw;
	int16_t y_raw;
	int16_t z_raw;

	float x_uT;
	float y_uT;
	float z_uT;
} QMC5883L_Data_t;

HAL_StatusTypeDef QMC5883L_Init(void);
HAL_StatusTypeDef QMC5883L_Read(QMC5883L_Data_t *data);


#ifdef __cplusplus
}
#endif

#endif /* DEVICE_QMC5883L_QMC5883L_H_ */
