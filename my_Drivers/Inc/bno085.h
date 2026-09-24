/*
 * bno085.h
 *
 *  Created on: 10 Eyl 2026
 *      Author: furkan
 */

#ifndef INC_BNO085_H_
#define INC_BNO085_H_

#include "main.h"
#include "spi.h"
#include <stdint.h>


#define BNO085_ROTATION_VECTOR_INTERVAL_US 			2500U // 2500us = 400 Hz

#define BNO085_SHTP_HEADER_SIZE 					4U

typedef enum
{
	BNO085_STATUS_OK = 0,
	BNO085_STATUS_BUSY,
	BNO085_STATUS_TIMEOUT,
	BNO085_STATUS_INVALID_PACKET,
	BNO085_STATUS_ERROR

}BNO085_Status_t;


typedef enum
{
	BNO085_ACCURACY_UNRELIABLE = 0,
	BNO085_ACCURACY_LOW,
	BNO085_ACCURACY_MEDIUM,
	BNO085_ACCURACY_HIGH

}BNO085_Accuracy_t;


typedef struct
{
	float x;
	float y;
	float z;
	float w;

	BNO085_Accuracy_t accuracy;
}BNO085_Quaternion_t;


typedef struct
{
	float roll;
	float pitch;
	float yaw;
}BNO085_Euler_t;


/*
 * BNO085 PUBLIC API's
 */

BNO085_Status_t BN085_Init(void);

void BNO085_Process(void);

BNO085_Status_t BNO085_EnableRotationVector(uint32_t interval_us);

BNO085_Status_t BNO085_GetQuaternion(BNO085_Quaternion_t *quaternion);

void BNO085_QuaternionToEuler(const BNO085_Quaternion_t *quaternion, BNO085_Euler_t *euler);

BNO085_Status_t BNO085_StartCalibration(void);

BNO085_Status_t BNO085_SaveCalibration(void);

BNO085_Status_t BNO085_RequestProductID(void);

#endif /* INC_BNO085_H_ */
