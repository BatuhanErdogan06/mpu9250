/*
 * mpu9250.h
 *
 *  Created on: Dec 21, 2022
 *      Author: erdog
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_
#include "stm32f4xx_hal.h"

#define mpu9250           0x68
#define mpu9250addr       mpu9250 << 1
#define whoAmIReg         0x75 //0x68 dene
#define powerManagmentReg 0x6B
#define sampleRateDiv 	  0x19 // sampleRate = gyroRate / (1 + sampleDiv)
#define gyroConf		  0x1B
#define accelConf		  0x1C

#define accelMeasure      0x3B
#define gyroMeasure       0x43


#define validCondition1 (whoAreYou == mpu9250)

I2C_HandleTypeDef hi2c1;
//UART_HandleTypeDef huart2;

uint8_t whoAreYou;
uint8_t MemData;

typedef enum{
	degS250  = 0,
	degS500  = 1,
	degS1000 = 2,
	degS2000 = 3
}gyroScale_t;

typedef enum{
	g2  = 0,
	g4  = 1,
	g8  = 2,
	g16 = 3
}accelScale_t;

int16_t RAWgyroX;
int16_t RAWgyroY;
int16_t RAWgyroZ;

int16_t RAWaccelX;
int16_t RAWaccelY;
int16_t RAWaccelZ;

float Ax, Ay, Az;
float Gx, Gy, Gz;

void mpu9250Config(void);
void mpu9250Init(void);
void mpu9250powerOn(void);
void mpu9250Sampling(void);
void mpu9250GyroScale(gyroScale_t scale);
void mpu9250AccelScale(accelScale_t scale);

void mpu9250GyroRead(void);
void mpu9250AccelRead(void);




#endif /* INC_MPU9250_H_ */
