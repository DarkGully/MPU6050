/*
library name: 	MPU6050 6 axis module
written by: 		T.Jaber
Date Written: 	25 Mar 2019
Last Modified: 	20 April 2019 by Mohamed Yaqoob
Description: 		MPU6050 Module Basic Functions Device Driver library that use HAL libraries.
References:			
								- MPU6050 Registers map: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
								- Jeff Rowberg MPU6050 library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
								
* Copyright (C) 2019 - T. Jaber
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.
	
   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
*/

//Header files
#include "MPU6050.h"

//Library Variable
//1- I2C Handle 
static I2C_HandleTypeDef i2cHandler;  
//2- Accel & Gyro Scaling Factor
static float accelScalingFactor, gyroScalingFactor;  
//3- Bias varaibles
static float A_X_Bias = 0.0f;     
static float A_Y_Bias = 0.0f;
static float A_Z_Bias = 0.0f;

static int16_t GyroRW[3];

CaliMPU6050Data_Def Cali_Data;
ScaledData_Def Cali_Scaled_Data_Gyro, Cali_Scaled_Data_Accel;

float accelerationx_nacteni[64];
float accelerationy_nacteni[64];
float accelerationz_nacteni[64];

float gyrox_nacteni[64];
float gyroy_nacteni[64];
float gyroz_nacteni[64];

float Cali_Buffer_Gyro_xmin[Pocet_Testu];
float Cali_Buffer_Gyro_ymin[Pocet_Testu];
float Cali_Buffer_Gyro_zmin[Pocet_Testu];

float Cali_Buffer_Gyro_xmax[Pocet_Testu];
float Cali_Buffer_Gyro_ymax[Pocet_Testu];
float Cali_Buffer_Gyro_zmax[Pocet_Testu];

float Cali_Buffer_Accel_xmin[Pocet_Testu];
float Cali_Buffer_Accel_ymin[Pocet_Testu];
float Cali_Buffer_Accel_zmin[Pocet_Testu];

float Cali_Buffer_Accel_xmax[Pocet_Testu];
float Cali_Buffer_Accel_ymax[Pocet_Testu];
float Cali_Buffer_Accel_zmax[Pocet_Testu];

float accelerationx, accelerationy, accelerationz;
float gyrox, gyroy, gyroz;
		
//Fucntion Definitions
//1- i2c Handler 
void MPU6050_Init(I2C_HandleTypeDef *I2Chnd)
{
	//Copy I2C CubeMX handle to local library
	memcpy(&i2cHandler, I2Chnd, sizeof(*I2Chnd));               
}

//2- i2c Read
void I2C_Read(uint8_t ADDR, uint8_t *i2cBif, uint8_t NofData)        
{
	uint8_t i2cBuf[2];
	uint8_t MPUADDR;
	//Need to Shift address to make it proper to i2c operation
	MPUADDR = (MPU_ADDR<<1);
	i2cBuf[0] = ADDR;
	HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cBuf, 1, 10);
	HAL_I2C_Master_Receive(&i2cHandler, MPUADDR, i2cBif, NofData, 100);
}

//3- i2c Write                                                
void I2C_Write8(uint8_t ADDR, uint8_t data)
{
	uint8_t i2cData[2];
	i2cData[0] = ADDR;
	i2cData[1] = data;
	uint8_t MPUADDR = (MPU_ADDR<<1);
	HAL_I2C_Master_Transmit(&i2cHandler, MPUADDR, i2cData, 2,100);
}

//4- MPU6050 Initialaztion Configuration 
void MPU6050_Config(MPU_ConfigTypeDef *config)
{
	uint8_t Buffer = 0;
	//Clock Source 
	//Reset Device
	I2C_Write8(PWR_MAGT_1_REG, 0x80);
	HAL_Delay(100);
	Buffer = config ->ClockSource & 0x07; //change the 7th bits of register
	Buffer |= (config ->Sleep_Mode_Bit << 6) &0x40; // change only the 7th bit in the register
	I2C_Write8(PWR_MAGT_1_REG, Buffer);
	HAL_Delay(100); // should wait 10ms after changeing the clock setting.
	
	//Set the Digital Low Pass Filter
	Buffer = 0;
	Buffer = config->CONFIG_DLPF & 0x07;
	I2C_Write8(CONFIG_REG, Buffer);
	
	//Select the Gyroscope Full Scale Range
	Buffer = 0;
	Buffer = (config->Gyro_Full_Scale << 3) & 0x18;
	I2C_Write8(GYRO_CONFIG_REG, Buffer);
	
	//Select the Accelerometer Full Scale Range 
	Buffer = 0; 
	Buffer = (config->Accel_Full_Scale << 3) & 0x18;
	I2C_Write8(ACCEL_CONFIG_REG, Buffer);
	//Set SRD To Default
	MPU6050_Set_SMPRT_DIV(0x04);
	
	
	//Accelerometer Scaling Factor, Set the Accelerometer and Gyroscope Scaling Factor
	switch (config->Accel_Full_Scale)
	{
		case AFS_SEL_2g:
			accelScalingFactor = (2000.0f/32768.0f);
			break;
		
		case AFS_SEL_4g:
			accelScalingFactor = (4000.0f/32768.0f);
				break;
		
		case AFS_SEL_8g:
			accelScalingFactor = (8000.0f/32768.0f);
			break;
		
		case AFS_SEL_16g:
			accelScalingFactor = (16000.0f/32768.0f);
			break;
		
		default:
			break;
	}
	//Gyroscope Scaling Factor 
	switch (config->Gyro_Full_Scale)
	{
		case FS_SEL_250:
			gyroScalingFactor = 250.0f/32768.0f;
			break;
		
		case FS_SEL_500:
				gyroScalingFactor = 500.0f/32768.0f;
				break;
		
		case FS_SEL_1000:
			gyroScalingFactor = 1000.0f/32768.0f;
			break;
		
		case FS_SEL_2000:
			gyroScalingFactor = 2000.0f/32768.0f;
			break;
		
		default:
			break;
	}
	
}

//5- Get Sample Rate Divider
uint8_t MPU6050_Get_SMPRT_DIV(void)
{
	uint8_t Buffer = 0;
	
	I2C_Read(SMPLRT_DIV_REG, &Buffer, 1);
	return Buffer;
}

//6- Set Sample Rate Divider
void MPU6050_Set_SMPRT_DIV(uint8_t SMPRTvalue)
{
	I2C_Write8(SMPLRT_DIV_REG, SMPRTvalue);
}

//7- Get External Frame Sync.
uint8_t MPU6050_Get_FSYNC(void)
{
	uint8_t Buffer = 0;
	
	I2C_Read(CONFIG_REG, &Buffer, 1);
	Buffer &= 0x38; 
	return (Buffer>>3);
}

//8- Set External Frame Sync. 
void MPU6050_Set_FSYNC(enum EXT_SYNC_SET_ENUM ext_Sync)
{
	uint8_t Buffer = 0;
	I2C_Read(CONFIG_REG, &Buffer,1);
	Buffer &= ~0x38;
	
	Buffer |= (ext_Sync <<3); 
	I2C_Write8(CONFIG_REG, Buffer);
	
}

//9- Get Accel Raw Data
void MPU6050_Get_Accel_RawData(RawData_Def *rawDef)
{
	uint8_t i2cBuf[2];
	uint8_t AcceArr[6], GyroArr[6];
	HAL_I2C_Mem_Read(&i2cHandler, (MPU_ADDR<<1), INT_STATUS_REG, 1, &i2cBuf[1], 1, 10);
	//I2C_Read(INT_STATUS_REG, &i2cBuf[1],1);
	if((i2cBuf[1]&&0x01))
	{
		//I2C_Read(ACCEL_XOUT_H_REG, AcceArr,6);
		HAL_I2C_Mem_Read(&i2cHandler, (MPU_ADDR<<1), ACCEL_XOUT_H_REG, 1, AcceArr, 6, 10);
		
		//Accel Raw Data
		rawDef->x = ((AcceArr[0]<<8) + AcceArr[1]); // x-Axis
		rawDef->y = ((AcceArr[2]<<8) + AcceArr[3]); // y-Axis
		rawDef->z = ((AcceArr[4]<<8) + AcceArr[5]); // z-Axis
		//Gyro Raw Data
		I2C_Read(GYRO_XOUT_H_REG, GyroArr,6);
		GyroRW[0] = ((GyroArr[0]<<8) + GyroArr[1]);
		GyroRW[1] = (GyroArr[2]<<8) + GyroArr[3];
		GyroRW[2] = ((GyroArr[4]<<8) + GyroArr[5]);
	}
}

//10- Get Accel scaled data (g unit of gravity, 1g = 9.81m/s2)
void MPU6050_Get_Accel_Scale(ScaledData_Def *scaledDef)
{

	RawData_Def AccelRData;
	MPU6050_Get_Accel_RawData(&AccelRData);
	//Accel Scale data 
	scaledDef->x = ((AccelRData.x+0.0f)*accelScalingFactor);
	scaledDef->y = ((AccelRData.y+0.0f)*accelScalingFactor);
	scaledDef->z = ((AccelRData.z+0.0f)*accelScalingFactor);
}

//11- Get Accel calibrated data
void MPU6050_Get_Accel_Cali(ScaledData_Def *CaliDef)
{
	ScaledData_Def AccelScaled;
	MPU6050_Get_Accel_Scale(&AccelScaled);
	
	//Accel Scale data 
	CaliDef->x = (AccelScaled.x) - A_X_Bias; // x-Axis
	CaliDef->y = (AccelScaled.y) - A_Y_Bias;// y-Axis
	CaliDef->z = (AccelScaled.z) - A_Z_Bias;// z-Axis
}
//12- Get Gyro Raw Data
void MPU6050_Get_Gyro_RawData(RawData_Def *rawDef)
{
	
	//Accel Raw Data
	rawDef->x = GyroRW[0];
	rawDef->y = GyroRW[1];
	rawDef->z = GyroRW[2];
	
}

//13- Get Gyro scaled data
void MPU6050_Get_Gyro_Scale(ScaledData_Def *scaledDef)
{
	RawData_Def myGyroRaw;
	MPU6050_Get_Gyro_RawData(&myGyroRaw);

	//Gyro Scale data 
	scaledDef->x = (myGyroRaw.x)*gyroScalingFactor; // x-Axis
	scaledDef->y = (myGyroRaw.y)*gyroScalingFactor; // y-Axis
	scaledDef->z = (myGyroRaw.z)*gyroScalingFactor; // z-Axis
}

//14- Accel Calibration
void _Accel_Cali(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
	//1* X-Axis calibrate
	A_X_Bias		= (x_max + x_min)/2.0f;
	
	//2* Y-Axis calibrate
	A_Y_Bias		= (y_max + y_min)/2.0f;
	
	//3* Z-Axis calibrate
	A_Z_Bias		= (z_max + z_min)/2.0f;
}

//15- MPU Calibration
void MPU6050_Cali(void)
{	
	uint16_t Gyro_min_x = 0;
	uint16_t Gyro_min_y = 0;
	uint16_t Gyro_min_z = 0;
	
	uint16_t Gyro_max_x = 0;
	uint16_t Gyro_max_y = 0;
	uint16_t Gyro_max_z = 0;

	uint16_t Accel_min_x = 0;
	uint16_t Accel_min_y = 0;
	uint16_t Accel_min_z = 0;
	
	uint16_t Accel_max_x = 0;
	uint16_t Accel_max_y = 0;
	uint16_t Accel_max_z = 0;
	
	accelerationx = 0;
	accelerationy = 0;
	accelerationz = 0;
	
	gyrox = 0;
	gyrox = 0;
	gyrox = 0;
	
//Stare nacitani hodnot pro kalibraci	
//	for (uint16_t i = 0; i <Pocet_Testu; i++)
//	{
//		MPU6050_Get_Accel_RawData(&Cali_Raw_Data_Accel);
//		MPU6050_Get_Gyro_RawData(&Cali_Raw_Data_Gyro);

//		Cali_Buffer_Gyro_xmin[i] = Cali_Raw_Data_Gyro.x;
//		Cali_Buffer_Gyro_ymin[i] = Cali_Raw_Data_Gyro.y;
//		Cali_Buffer_Gyro_zmin[i] = Cali_Raw_Data_Gyro.z;
//		
//		Cali_Buffer_Gyro_xmax[i] = Cali_Raw_Data_Gyro.x;
//		Cali_Buffer_Gyro_ymax[i] = Cali_Raw_Data_Gyro.y;
//		Cali_Buffer_Gyro_zmax[i] = Cali_Raw_Data_Gyro.z;
//		
//		Cali_Buffer_Accel_xmin[i] = Cali_Raw_Data_Accel.x;
//		Cali_Buffer_Accel_ymin[i] = Cali_Raw_Data_Accel.y;
//		Cali_Buffer_Accel_zmin[i] = Cali_Raw_Data_Accel.z;
//		
//		Cali_Buffer_Accel_xmax[i] = Cali_Raw_Data_Accel.x;
//		Cali_Buffer_Accel_ymax[i] = Cali_Raw_Data_Accel.y;
//		Cali_Buffer_Accel_zmax[i] = Cali_Raw_Data_Accel.z;	
//	}

	for (uint16_t i = 0; i <Pocet_Testu; i++)
	{
		Prumer_hodnot();
		Cali_Buffer_Gyro_xmin[i] = gyrox;
		Cali_Buffer_Gyro_ymin[i] = gyroy;
		Cali_Buffer_Gyro_zmin[i] = gyroz;
		
		Cali_Buffer_Gyro_xmax[i] = gyrox;
		Cali_Buffer_Gyro_ymax[i] = gyroy;
		Cali_Buffer_Gyro_zmax[i] = gyroz;
		
		Cali_Buffer_Accel_xmin[i] = accelerationx;
		Cali_Buffer_Accel_ymin[i] = accelerationy;
		Cali_Buffer_Accel_zmin[i] = accelerationz;
		
		Cali_Buffer_Accel_xmax[i] = accelerationx;
		Cali_Buffer_Accel_ymax[i] = accelerationy;
		Cali_Buffer_Accel_zmax[i] = accelerationz;	
	}
		
	for (uint16_t i = 0; i <Pocet_Testu; i++)
	{
    if (Cali_Buffer_Gyro_xmin[i] < Cali_Buffer_Gyro_xmin[Gyro_min_x])
			Gyro_min_x = i;		
    if (Cali_Buffer_Gyro_ymin[i] < Cali_Buffer_Gyro_ymin[Gyro_min_y])
			Gyro_min_y = i;	
    if (Cali_Buffer_Gyro_zmin[i] < Cali_Buffer_Gyro_zmin[Gyro_min_z])
			Gyro_min_z = i;
		
    if (Cali_Buffer_Gyro_xmax[i] > Cali_Buffer_Gyro_xmax[Gyro_max_x])
			Gyro_max_x = i;		
    if (Cali_Buffer_Gyro_ymax[i] > Cali_Buffer_Gyro_ymax[Gyro_max_y])
			Gyro_max_y = i;	
    if (Cali_Buffer_Gyro_zmax[i] > Cali_Buffer_Gyro_zmax[Gyro_max_z])
			Gyro_max_z = i;
		
		if (Cali_Buffer_Accel_xmin[i] < Cali_Buffer_Accel_xmin[Accel_min_x])
			Accel_min_x = i;			
    if (Cali_Buffer_Accel_ymin[i] < Cali_Buffer_Accel_ymin[Accel_min_y])
			Accel_min_y = i;	
    if (Cali_Buffer_Accel_zmin[i] < Cali_Buffer_Accel_zmin[Accel_min_z])
			Accel_min_z = i;
		
    if (Cali_Buffer_Accel_xmax[i] > Cali_Buffer_Accel_xmax[Accel_max_x])
			Accel_max_x = i;			
    if (Cali_Buffer_Accel_ymax[i] > Cali_Buffer_Accel_ymax[Accel_max_y])
			Accel_max_y = i;	
    if (Cali_Buffer_Accel_zmax[i] > Cali_Buffer_Accel_zmax[Accel_max_z])
			Accel_max_z = i;
  }
	
	Cali_Data.Gyro_x_min = Cali_Buffer_Gyro_xmax[Gyro_min_x];
	Cali_Data.Gyro_y_min = Cali_Buffer_Gyro_ymax[Gyro_min_y];
	Cali_Data.Gyro_z_min = Cali_Buffer_Gyro_zmax[Gyro_min_z];
	
	Cali_Data.Gyro_x_max = Cali_Buffer_Gyro_xmax[Gyro_max_x];
	Cali_Data.Gyro_y_max = Cali_Buffer_Gyro_ymax[Gyro_max_y];
	Cali_Data.Gyro_z_max = Cali_Buffer_Gyro_zmax[Gyro_max_z];

	Cali_Data.Accel_x_min = Cali_Buffer_Accel_xmax[Accel_min_x];
	Cali_Data.Accel_y_min = Cali_Buffer_Accel_ymax[Accel_min_y];
	Cali_Data.Accel_z_min = Cali_Buffer_Accel_zmax[Accel_min_z];
	
	Cali_Data.Accel_x_max = Cali_Buffer_Accel_xmax[Accel_max_x];
	Cali_Data.Accel_y_max = Cali_Buffer_Accel_ymax[Accel_max_y];
	Cali_Data.Accel_z_max = Cali_Buffer_Accel_zmax[Accel_max_z];
	
//Stare podminky	
//		if ((myGyroScaled.x <= 0) && (myGyroScaled.x >= Cali_Data.Gyro_x_max))
//			myGyroScaled.x = 0;
//		if ((myGyroScaled.y <= Cali_Data.Gyro_y_max) && (myGyroScaled.y >= 0))
//			myGyroScaled.y = 0;
//		if ((myGyroScaled.z <= 0) && (myGyroScaled.z >= Cali_Data.Gyro_z_max))
//			myGyroScaled.z = 0;
		
//		if ((myAccelScaled.x <= Cali_Data.Accel_x_max) && (myAccelScaled.x >= 0))
//			myAccelScaled.x = 0;
//		if ((myAccelScaled.y <= 0) && (myAccelScaled.y >= Cali_Data.Accel_y_max))
//			myAccelScaled.y = 0;
//		if ((myAccelScaled.z <= Cali_Data.Accel_z_max) && (myAccelScaled.z >= 0))
//			myAccelScaled.z = 0;

	//_Accel_Cali(Cali_Data.Accel_x_min, Cali_Data.Accel_x_max, Cali_Data.Accel_y_min, Cali_Data.Accel_y_max, Cali_Data.Accel_z_min, Cali_Data.Accel_z_max);
}

void Prumer_hodnot(void)
{	
		for (uint16_t i = 0; i <10; i++)
	{
		MPU6050_Get_Accel_Scale(&Cali_Scaled_Data_Accel);
		MPU6050_Get_Gyro_Scale(&Cali_Scaled_Data_Gyro);
		
		accelerationx_nacteni[i] = Cali_Scaled_Data_Accel.x;
		accelerationy_nacteni[i] = Cali_Scaled_Data_Accel.y;
		accelerationz_nacteni[i] = Cali_Scaled_Data_Accel.z;
		
		gyrox_nacteni[i] = Cali_Scaled_Data_Gyro.x;
		gyroy_nacteni[i] = Cali_Scaled_Data_Gyro.y;
		gyroz_nacteni[i] = Cali_Scaled_Data_Gyro.z;
	}
	
	for (uint16_t i = 0; i <10; i++)
	{
		accelerationx = accelerationx + accelerationx_nacteni[i];
		accelerationy = accelerationy + accelerationy_nacteni[i];
		accelerationz = accelerationz + accelerationz_nacteni[i];
		
		gyrox = gyrox + gyrox_nacteni[i];
		gyroy = gyroy + gyroy_nacteni[i];
		gyroz = gyroz + gyroz_nacteni[i];
	}
	
	accelerationx = accelerationx/10; 
	accelerationy = accelerationy/10;
	accelerationz = accelerationz/10;
	
	gyrox = gyrox / 10;
	gyroy = gyroy / 10;
	gyroz = gyroz / 10;
}

void Cali_Podminky(void)
{

	
		//Kalibrace osy x Gyroskopu
		if ((Cali_Data.Gyro_x_min <= 0) && (Cali_Data.Gyro_x_max <= 0)) 
		{
			if ((gyrox <= 0) && (gyrox >= Cali_Data.Gyro_x_min))
				gyrox = 0;
		}
		if ((Cali_Data.Gyro_x_min >= 0) && (Cali_Data.Gyro_x_max >= 0)) 
		{
			if ((gyrox >= 0) && (gyrox <= Cali_Data.Gyro_x_max))
				gyrox = 0;
		}
		if ((Cali_Data.Gyro_x_min <= 0) && (Cali_Data.Gyro_x_max >= 0))
		{
			if ((gyrox <= Cali_Data.Gyro_x_max) && (gyrox >= Cali_Data.Gyro_x_min))
				gyrox = 0;
		}
		
		//Kalibrace osy y Gyroskopu
		if ((Cali_Data.Gyro_y_min <= 0) && (Cali_Data.Gyro_y_max <= 0)) 
		{
			if ((gyroy <= 0) && (gyroy >= Cali_Data.Gyro_y_min))
				gyroy = 0;
		}
		if ((Cali_Data.Gyro_y_min >= 0) && (Cali_Data.Gyro_y_max >= 0))
		{
			if ((gyroy >= 0) && (gyroy <= Cali_Data.Gyro_y_max))
			{
				gyroy = 0;
			}
		}
		if ((Cali_Data.Gyro_y_min <= 0) && (Cali_Data.Gyro_y_max >= 0))
		{
			if ((gyroy <= Cali_Data.Gyro_y_max) && (gyroy >= Cali_Data.Gyro_y_min))
				gyroy = 0;
		}
		
		//Kalibrace osy z Gyroskopu
		if ((Cali_Data.Gyro_z_min <= 0) && (Cali_Data.Gyro_z_max <= 0)) 
		{
			if ((gyroz <= 0) && (gyroz >= Cali_Data.Gyro_z_min))
				gyroz = 0;
		}
		if ((Cali_Data.Gyro_z_min >= 0) && (Cali_Data.Gyro_z_max >= 0)) 
		{
			if ((gyroz >= 0) && (gyroz <= Cali_Data.Gyro_z_max))
				gyroz = 0;
		}
		if ((Cali_Data.Gyro_z_min <= 0) && (Cali_Data.Gyro_z_max >= 0))
		{
			if ((gyroz <= Cali_Data.Gyro_z_max) && (gyroz >= Cali_Data.Gyro_z_min))
				gyroz = 0;
		}

		//Kalibrace osy x Akcelerometru
		if ((Cali_Data.Accel_x_min  <= 0) && (Cali_Data.Accel_x_max <= 0)) 
		{
			if ((accelerationx <= 0) && (accelerationx >= Cali_Data.Accel_x_min))
				accelerationx = 0;
		}
		if ((Cali_Data.Accel_x_min >= 0) && (Cali_Data.Accel_x_max >= 0)) 
		{
			if ((accelerationx >= 0) && (accelerationx <= Cali_Data.Accel_x_max))
				accelerationx = 0;
		}
		if ((Cali_Data.Accel_x_min <= 0) && (Cali_Data.Accel_x_max >= 0))
		{
			if ((accelerationx <= Cali_Data.Accel_x_max) && (accelerationx >= Cali_Data.Accel_x_min))
				accelerationx = 0;
		}
		
		//Kalibrace osy y Akcelerometru
		if ((Cali_Data.Accel_y_min <= 0) && (Cali_Data.Accel_y_max <= 0)) 
		{
			if ((accelerationy <= 0) && (accelerationy >= Cali_Data.Accel_y_min))
				accelerationy = 0;
		}
		if ((Cali_Data.Accel_y_min >= 0) && (Cali_Data.Accel_y_max >= 0)) 
		{
			if ((accelerationy >= 0) && (accelerationy <= Cali_Data.Accel_y_max))
				accelerationy = 0;
		}
		if ((Cali_Data.Accel_y_min <= 0) && (Cali_Data.Accel_y_max >= 0))
		{
			if ((accelerationy <= Cali_Data.Accel_y_max) && (accelerationy >= Cali_Data.Accel_y_min))
				accelerationy = 0;
		}
		
		//Kalibrace osy z Akcelerometru
		if ((Cali_Data.Accel_z_min <= 0) && (Cali_Data.Accel_z_max <= 0)) 
		{
			if ((accelerationz <= 0) && (accelerationz >= Cali_Data.Accel_z_min))
				accelerationz = 0;
		}
		if ((Cali_Data.Accel_z_min >= 0) && (Cali_Data.Accel_z_max >= 0)) 
		{
			if ((accelerationz >= 0) && (accelerationz <= Cali_Data.Accel_z_max))
				accelerationz = 0;
		}
		if ((Cali_Data.Accel_z_min <= 0) && (Cali_Data.Accel_z_max >= 0))
		{
			if ((accelerationz <= Cali_Data.Accel_z_max) && (accelerationz >= Cali_Data.Accel_z_min))
				accelerationz = 0;
		}					
}
