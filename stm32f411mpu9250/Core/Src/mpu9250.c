/*
 * mpu9250.c
 *
 *  Created on: Dec 21, 2022
 *      Author: erdog
 */
#include "mpu9250.h"

void mpu9250Init(void){
	HAL_I2C_Mem_Read(
				&hi2c1,
				mpu9250addr,
				whoAmIReg,
				1,
				&whoAreYou,
				1,
				100
				);
}

void mpu9250powerOn(void){
	MemData = 0x00;
	HAL_I2C_Mem_Write(
			&hi2c1,
			mpu9250addr,
			powerManagmentReg,
			1,
			&MemData,
			1,
			100
			);
}

void mpu9250Sampling(void){
	MemData = 0x07;
	HAL_I2C_Mem_Write(
			&hi2c1,
			mpu9250addr,
			sampleRateDiv,
			1,
			&MemData,
			1,
			100
			);
}

void mpu9250GyroScale(gyroScale_t scale){
	MemData = 0x00 | (scale << 3);
		HAL_I2C_Mem_Write(
			&hi2c1,
			mpu9250addr,
			gyroConf,
			1,
			&MemData,
			1,
			100
			);
}

void mpu9250AccelScale(accelScale_t scale){
	MemData = 0x00 | (scale << 3);
		HAL_I2C_Mem_Write(
			&hi2c1,
			mpu9250addr,
			accelConf,
			1,
			&MemData,
			1,
			100
			);
}

void mpu9250Config(void){
	// is valid Condition true 0x68
	mpu9250Init();

	if(validCondition1){
	// power on
		mpu9250powerOn();
	// sampling data ratio
		mpu9250Sampling();
	// gyro scale   (RAW)
		mpu9250GyroScale(degS250);
	// accel scale  (RAW)
		mpu9250AccelScale(g2);
	}
}

void mpu9250AccelRead(void){
	uint8_t gyroData[6];
	HAL_I2C_Mem_Read(
			&hi2c1,
			mpu9250addr,
			gyroMeasure,
			1,
			gyroData,
			6,
			100
			);

	RAWgyroX = (uint16_t) (gyroData[0] << 8 | gyroData[1]);
	RAWgyroY = (uint16_t) (gyroData[2] << 8 | gyroData[3]);
	RAWgyroZ = (uint16_t) (gyroData[4] << 8 | gyroData[5]);

	Gx = RAWgyroX/131.0;
	Gy = RAWgyroY/131.0;
	Gz = RAWgyroZ/131.0;
}

void mpu9250GyroRead(void){
	uint8_t gyroData[6];
	HAL_I2C_Mem_Read(
			&hi2c1,
			mpu9250addr,
			gyroMeasure,
			1,
			gyroData,
			6,
			100
			);

	RAWgyroX = (uint16_t) (gyroData[0] << 8 | gyroData[1]);
	RAWgyroY = (uint16_t) (gyroData[2] << 8 | gyroData[3]);
	RAWgyroZ = (uint16_t) (gyroData[4] << 8 | gyroData[5]);

	Gx = RAWgyroX/131.0;
	Gy = RAWgyroY/131.0;
	Gz = RAWgyroZ/131.0;
}
