/*
 * imu.h
 *
 *  Created on: Jan 14, 2023
 *      Author: Bastien
 */

#ifndef IMU_H_
#define IMU_H_

#include <IMU/imu_quaternionFilter.h>
#include <stdint.h>

#include "i2c.h"

typedef enum ACCEL_FS_SEL_e {
    A2G,
    A4G,
    A8G,
    A16G
} ACCEL_FS_SEL_t;

typedef enum GYRO_FS_SEL_e {
    G250DPS,
    G500DPS,
    G1000DPS,
    G2000DPS
}GYRO_FS_SEL_t;

typedef enum FIFO_SAMPLE_RATE_e {
    SMPL_1000HZ,
    SMPL_500HZ,
    SMPL_333HZ,
    SMPL_250HZ,
    SMPL_200HZ,
    SMPL_167HZ,
    SMPL_143HZ,
    SMPL_125HZ,
}FIFO_SAMPLE_RATE_t;

typedef enum MAG_OUTPUT_BITS_e {
    M14BITS,
    M16BITS
}MAG_OUTPUT_BITS_t;

typedef enum GYRO_DLPF_CFG_e {
    GYRO_DLPF_250HZ,
	GYRO_DLPF_184HZ,
	GYRO_DLPF_92HZ,
	GYRO_DLPF_41HZ,
	GYRO_DLPF_20HZ,
	GYRO_DLPF_10HZ,
	GYRO_DLPF_5HZ,
	GYRO_DLPF_3600HZ,
}GYRO_DLPF_CFG_t;

typedef enum ACCEL_DLPF_CFG_e {
	ACC_DLPF_218HZ_0,
	ACC_DLPF_218HZ_1,
	ACC_DLPF_99HZ,
	ACC_DLPF_45HZ,
	ACC_DLPF_21HZ,
    ACC_DLPF_10HZ,
	ACC_DLPF_5HZ,
	ACC_DLPF_420HZ,
}ACCEL_DLPF_CFG_t;


typedef struct IMUSettings_s {
	ACCEL_FS_SEL_t accel_fs_sel;
	GYRO_FS_SEL_t gyro_fs_sel;
	MAG_OUTPUT_BITS_t mag_output_bits;
	FIFO_SAMPLE_RATE_t fifo_sample_rate;
	uint8_t gyro_fchoice;
	GYRO_DLPF_CFG_t gyro_dlpf_cfg;
	uint8_t accel_fchoice;
	ACCEL_DLPF_CFG_t accel_dlpf_cfg;
}IMUSettings_t;

typedef struct imuHandle_s {
	//settings
	uint8_t MAG_MODE; // 0x02 for 8 Hz, 0x06 for 100 Hz continuous magnetometer data read
    float acc_resolution;
    float gyro_resolution;
    float mag_resolution;
    IMUSettings_t settings;

	//IMU Values
    double a[3];
    double g[3];
    double m[3];
    double rpy[3];

    // Calculation Values
    quaternionFilter_t quarternion;
    double lin_acc[3];  // linear acceleration (acceleration with gravity component subtracted)

    // Calibration Parameters
    double acc_bias[3];  // acc calibration value in ACCEL_FS_SEL: 2g
    double gyro_bias[3]; // gyro calibration value in GYRO_FS_SEL: 250dps
    double mag_bias_factory[3];
    double mag_bias[3];  // mag calibration value in MAG_OUTPUT_BITS: 16BITS
    double mag_scale[3];
    double magnetic_declination;  // Japan, 24th June

    //Other
    float temperature;  //Internal temperature of imu
}imuHandle_t;


void StartImuTask(void const * argument);

uint8_t imu_init(I2C_HandleTypeDef* i2c, imuHandle_t* imuH);
void imu_initMPU9250(imuHandle_t* imuH);
void imu_initAK8963(imuHandle_t* imuH);

void imu_getMotion6(imuHandle_t* imuH);
void imu_getMotion9(imuHandle_t* imuH);

void imu_updateQuat(imuHandle_t* imuH);
void imu_updateRPY(imuHandle_t* imuH); // simpler and working way
void imu_updateRPY_from_quat(imuHandle_t* imuH); // better way but not working


#endif /* IMU_H_ */
