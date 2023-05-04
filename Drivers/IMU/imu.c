/*
 * imu.c
 *
 *  Created on: Jan 14, 2023
 *      Author: Bastien
 */
#include <cmsis_os.h>
#include <math.h>

#include "i2c.h"
#include "trace/trace.h"
#include "IMU/imu.h"
#include "IMU/imu_RegisterMap.h"
#include "IMU/imuSerial.h"
#include "IMU/micros.h"

/*** Private functions prototypes ***/
uint8_t _isConnected(void);
uint8_t _isConnectedMPU9250(void);
uint8_t _isConnectedAK8963(void);

float _get_acc_resolution(const ACCEL_FS_SEL_t accel_af_sel);
float _get_gyro_resolution(const GYRO_FS_SEL_t gyro_fs_sel);
float _get_mag_resolution(const MAG_OUTPUT_BITS_t mag_output_bits);

const uint16_t CALIB_GYRO_SENSITIVITY = 131;     // LSB/degrees/sec
const uint16_t CALIB_ACCEL_SENSITIVITY = 16384;  // LSB/g

imuHandle_t imuHandle;

/*** public functions declarations ***/

/*
 * IMU periodic task
 * SDA - PB7 - D4
 * SCL - PB6 - D5
 */
void StartImuTask(void const * argument)
{
	PRINT("Init IMU...");

	taskENTER_CRITICAL();
	if(imu_init(&hi2c1, &imuHandle) == 0)
		PRINT("done\n\n");
	else
	{
		PRINT("ERROR!\n");
		while(1);
	}
	taskEXIT_CRITICAL();

	for(;;){
		imu_getMotion9(&imuHandle);
		imu_updateQuat(&imuHandle);
		imu_updateRPY(&imuHandle);
//		imu_updateRPY_from_quat(&imuHandle);

		PRINT("Accelero - x: %+1.4f  y: %+1.4f  z: %+1.4f\n", imuHandle.a[0], imuHandle.a[1], imuHandle.a[2]);
		//PRINT("Gyro     - x: %+3.2f  y: %+3.2f  z: %+3.2f\n", imuHandle.g[0], imuHandle.g[1], imuHandle.g[2]);
		PRINT("Magneto  - x: %+3.2f  y: %+3.2f  z: %+3.2f\n", imuHandle.m[0], imuHandle.m[1], imuHandle.m[2]);

		PRINT("RPY - roll: %+2.2f  pitch: %+2.2f  yaw:  %+2.2f\n\n", imuHandle.rpy[0], imuHandle.rpy[1], imuHandle.rpy[2]);

		osDelay(1000);
	}
}

uint8_t imu_init(I2C_HandleTypeDef* i2c, imuHandle_t* imuH)
{
	DWT_Init();

	imuSerial_Init(i2c);

	if(!_isConnectedMPU9250())
		return 2;

//*** IMU HANDLE Init
	//RAZ Values
	imuH->quarternion.q[0] = 1.0f;			 //Quaternion
	for(uint8_t i=1 ; i<4 ; i++)
		imuH->quarternion.q[i] = 0.f;
	for(uint8_t i=0 ; i<3 ; i++) //Accelero
		imuH->a[i] = 0.f;
	for(uint8_t i=0 ; i<3 ; i++) //Gyroscope
		imuH->g[i] = 0.f;
	for(uint8_t i=0 ; i<3 ; i++) //Magneto
		imuH->m[i] = 0.f;
	for(uint8_t i=0 ; i<3 ; i++) //Raw Pitch Yaw
		imuH->rpy[i] = 0.f;
	for(uint8_t i=0 ; i<3 ; i++) //Linear acceleration
		imuH->lin_acc[i] = 0.f;
	for(uint8_t i=0 ; i<3 ; i++) //Mag scale
		imuH->mag_scale[i] = 1.f;

	quatF_init(&imuH->quarternion);
	imuH->magnetic_declination = -7.51;
	imuH->temperature = 0.0f;

	imuH->settings.accel_fs_sel = A16G;
	imuH->settings.gyro_fs_sel = G2000DPS;
	imuH->settings.mag_output_bits = M16BITS;
	imuH->settings.fifo_sample_rate = SMPL_200HZ;
	imuH->settings.gyro_fchoice = 0x03;
	imuH->settings.gyro_dlpf_cfg = GYRO_DLPF_41HZ;
	imuH->settings.accel_fchoice = 0x01;
	imuH->settings.accel_dlpf_cfg = ACC_DLPF_45HZ;

//*** Init physical chips
	imu_initMPU9250(imuH);
	imu_initAK8963(imuH);

	return 0;
}

// Init Accelero and gyro
void imu_initMPU9250(imuHandle_t* imuH)
{
	imuH->MAG_MODE = 0x06;
	imuH->acc_resolution = _get_acc_resolution(A16G);   //settings
	imuH->gyro_resolution = _get_gyro_resolution(G2000DPS);
	imuH->mag_resolution = _get_mag_resolution(M16BITS);

	// reset device
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
	osDelay(100);

	// wake up device
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x00);  // Clear sleep mode bit (6), enable all sensors
	osDelay(100);                                  // Wait for all registers to reset

	// get stable time source
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	osDelay(200);

	uint8_t c = 0;

	// Set accelerometer full-scale range configuration
	imuSerial_readReg(MPU9250_ADDR, MPU9250_ACCEL_CONFIG, &c);     // get current ACCEL_CONFIG register value
	c = c & ~0xE0;                                 // Clear self-test bits [7:5]
	c = c & ~0x18;                                 // Clear ACCEL_FS_SEL bits [4:3]
	c = c | (uint8_t)(imuH->settings.accel_fs_sel << 3);  // Set full scale range for the accelerometer
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_ACCEL_CONFIG, c);     // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	imuSerial_readReg(MPU9250_ADDR, MPU9250_ACCEL_CONFIG_2, &c);// get current ACCEL_CONFIG2 register value
    c = c & ~0x0F;                                     // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | (~(imuH->settings.accel_fchoice << 3) & 0x08);    // Set accel_fchoice_b to 1
    c = c | (uint8_t)(imuH->settings.accel_dlpf_cfg & 0x07);  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    imuSerial_writeReg(MPU9250_ADDR, MPU9250_ACCEL_CONFIG_2, c);        // Write new ACCEL_CONFIG2 register value

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_INT_PIN_CFG, 0x22);
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	osDelay(100);
}

// Init Magneto
void imu_initAK8963(imuHandle_t* imuH)
{
    // First extract the factory calibration for each magnetometer axis
    uint8_t raw_data[3];                            // x/y/z gyro calibration data stored here
    imuSerial_writeReg(AK8963_ADDR, AK8963_CNTL, 0x00);  // Power down magnetometer
    osDelay(10);
    imuSerial_writeReg(AK8963_ADDR, AK8963_CNTL, 0x0F);  // Enter Fuse ROM access mode
    osDelay(10);
    imuSerial_readRegBurst(AK8963_ADDR, AK8963_ASAX, 3, &raw_data[0]);  // Read the x-, y-, and z-axis calibration values
    imuH->mag_bias_factory[0] = (float)(raw_data[0] - 128) / 256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
    imuH->mag_bias_factory[1] = (float)(raw_data[1] - 128) / 256. + 1.;
    imuH->mag_bias_factory[2] = (float)(raw_data[2] - 128) / 256. + 1.;
    imuSerial_writeReg(AK8963_ADDR, AK8963_CNTL, 0x00);  // Power down magnetometer
    osDelay(10);

    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition MAG_MODE (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    imuSerial_writeReg(AK8963_ADDR, AK8963_CNTL, (uint8_t)imuH->settings.mag_output_bits << 4 | imuH->MAG_MODE);  // Set magnetometer data resolution and sample ODR
    osDelay(10);

    PRINT("\n");
	PRINT("Mag Factory Calibration Values: \n");
	PRINT("X-Axis sensitivity offset value %f\n", imuH->mag_bias_factory[0]);
	PRINT("Y-Axis sensitivity offset value %f\n", imuH->mag_bias_factory[1]);
	PRINT("Z-Axis sensitivity offset value %f\n", imuH->mag_bias_factory[2]);
}

void imu_updateQuat(imuHandle_t* imuH)
{
	// Madgwick function needs to be fed North, East, and Down direction like
	// (AN, AE, AD, GN, GE, GD, MN, ME, MD)
	// Accel and Gyro direction is Right-Hand, X-Forward, Z-Up
	// Magneto direction is Right-Hand, Y-Forward, Z-Down
	// So to adopt to the general Aircraft coordinate system (Right-Hand, X-Forward, Z-Down),
	// we need to feed (ax, -ay, -az, gx, -gy, -gz, my, -mx, mz)
	// but we pass (-ax, ay, az, gx, -gy, -gz, my, -mx, mz)
	// because gravity is by convention positive down, we need to invert the accel data

	quatF_update(&imuH->quarternion, -(imuH->a[0]), imuH->a[1], imuH->a[2], imuH->g[0], -(imuH->g[1]), -(imuH->g[2]), imuH->m[1], -(imuH->m[0]), imuH->m[2]);

//	quatF_update(&imuH->quarternion, imuH->a[0], imuH->a[1], imuH->a[2], imuH->g[0], imuH->g[1], imuH->g[2], imuH->m[0], imuH->m[1], imuH->m[2]);
}

void imu_updateRPY(imuHandle_t* imuH) {
	imuH->rpy[0] = atan2((double)imuH->a[1], (double)imuH->a[2]);				//roll
	imuH->rpy[1] = asin((double)imuH->a[0]);							//pitch
	imuH->rpy[2] = atan2((double)imuH->m[0], (double)imuH->m[1]) * 180 / M_PI;	//yaw
//	imuH->rpy[2] += imuH->magnetic_declination; //add magnetic to yaw to find the 'TRUE' heading
}

void imu_updateRPY_from_quat(imuHandle_t* imuH) {
	// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
	// In this coordinate system, the positive z-axis is down toward Earth.
	// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
	// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	// applied in the correct order which for this configuration is yaw, pitch, and then roll.
	// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.

	double a12, a22, a31, a32, a33;  // rotation matrix coefficients for Euler angles and gravity components
	// short name local variable for readability
	double qw = imuH->quarternion.q[0], qx = imuH->quarternion.q[1], qy = imuH->quarternion.q[2], qz = imuH->quarternion.q[3];
	double *roll = &imuH->rpy[0], *pitch = &imuH->rpy[1], *yaw = &imuH->rpy[2];

	a12 = 2.0f * (qx * qy + qw * qz);
	a22 = qw * qw + qx * qx - qy * qy - qz * qz;
	a31 = 2.0f * (qw * qx + qy * qz);
	a32 = 2.0f * (qx * qz - qw * qy);
	a33 = qw * qw - qx * qx - qy * qy + qz * qz;
	*roll = atan2f(a31, a33);
	*pitch = -asinf(a32);
	*yaw = atan2f(a12, a22);
	*roll *= 180.0f / M_PI;
	*pitch *= 180.0f / M_PI;
	*yaw *= 180.0f / M_PI;
	*yaw += imuH->magnetic_declination;
	if (*yaw >= +180.f)
		*yaw -= 360.f;
	else if (*yaw < -180.f)
		*yaw += 360.f;

	imuH->lin_acc[0] = imuH->a[0] + a31;
	imuH->lin_acc[1] = imuH->a[1] + a32;
	imuH->lin_acc[2] = imuH->a[2] - a33;
}


// ACCEL_*OUT_* registers

/** Get raw 9-axis motion sensor readings (accel/gyro/compass).
    FUNCTION NOT FULLY IMPLEMENTED YET.
    @param ax 16-bit signed integer container for accelerometer X-axis value
    @param ay 16-bit signed integer container for accelerometer Y-axis value
    @param az 16-bit signed integer container for accelerometer Z-axis value
    @param gx 16-bit signed integer container for gyroscope X-axis value
    @param gy 16-bit signed integer container for gyroscope Y-axis value
    @param gz 16-bit signed integer container for gyroscope Z-axis value
    @param mx 16-bit signed integer container for magnetometer X-axis value
    @param my 16-bit signed integer container for magnetometer Y-axis value
    @param mz 16-bit signed integer container for magnetometer Z-axis value
    @see getMotion6()
    @see getAcceleration()
    @see getRotation()
    @see getDirection()
*/
void imu_getMotion9(imuHandle_t* imuH) {
    //get accel and gyro
    imu_getMotion6(imuH);

    //Wait data ready flag
    uint8_t ST1;
	do
	{
		imuSerial_readReg(AK8963_ADDR, AK8963_ST1, &ST1);
	} while (!(ST1 & 0x01));

	//Get raw data
    int16_t raw_mag_data[3] = {0, 0, 0};
	uint8_t buffer[7];

	imuSerial_readRegBurst(AK8963_ADDR, AK8963_HXH, 7, buffer);
	if (imuH->MAG_MODE == 0x02 || imuH->MAG_MODE == 0x04 || imuH->MAG_MODE == 0x06) {  // continuous or external trigger read mode
//		if ((ST1 & 0x02) != 0)	// check if data is not skipped
//			return;             // this should be after data reading to clear DRDY register
	}
	raw_mag_data[0] = (((int16_t)buffer[0]) << 8) | buffer[1];
	raw_mag_data[1] = (((int16_t)buffer[2]) << 8) | buffer[3];
	raw_mag_data[2] = (((int16_t)buffer[4]) << 8) | buffer[5];

	// Calculate the magnetometer values in milliGauss
	// Include factory calibration per data sheet and user environmental corrections
	// mag_bias is calculated in 16BITS
	float bias_to_current_bits = imuH->mag_resolution / _get_mag_resolution(M16BITS);
	imuH->m[0] = (float)(raw_mag_data[0] * imuH->mag_resolution * imuH->mag_bias_factory[0] - imuH->mag_bias[0] * bias_to_current_bits) * imuH->mag_scale[0];  // get actual magnetometer value, this depends on scale being set
	imuH->m[1] = (float)(raw_mag_data[1] * imuH->mag_resolution * imuH->mag_bias_factory[1] - imuH->mag_bias[1] * bias_to_current_bits) * imuH->mag_scale[1];
	imuH->m[2] = (float)(raw_mag_data[2] * imuH->mag_resolution * imuH->mag_bias_factory[2] - imuH->mag_bias[2] * bias_to_current_bits) * imuH->mag_scale[2];
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
    Retrieves all currently available motion sensor values.
    @param ax 16-bit signed integer container for accelerometer X-axis value
    @param ay 16-bit signed integer container for accelerometer Y-axis value
    @param az 16-bit signed integer container for accelerometer Z-axis value
    @param gx 16-bit signed integer container for gyroscope X-axis value
    @param gy 16-bit signed integer container for gyroscope Y-axis value
    @param gz 16-bit signed integer container for gyroscope Z-axis value
    @see getAcceleration()
    @see getRotation()
*/
void imu_getMotion6(imuHandle_t* imuH) {
    int16_t raw_acc_gyro_data[7];

    //Get raw
	uint8_t buffer[14];
	imuSerial_readRegBurst(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 14, buffer);

	//Get accelerometer
	raw_acc_gyro_data[0] = (((int16_t)buffer[0]) << 8) | buffer[1];
    imuH->a[0] = (float)raw_acc_gyro_data[0] * imuH->acc_resolution;
	raw_acc_gyro_data[1] = (((int16_t)buffer[2]) << 8) | buffer[3];
    imuH->a[1] = (float)raw_acc_gyro_data[1] * imuH->acc_resolution;
	raw_acc_gyro_data[2] = (((int16_t)buffer[4]) << 8) | buffer[5];
    imuH->a[2] = (float)raw_acc_gyro_data[2] * imuH->acc_resolution;

    raw_acc_gyro_data[3] = (float)((((int16_t)buffer[6]) << 8) | buffer[7])  / 333.87 + 21.0;  // Read the adc values
    imuH->temperature = ((float)raw_acc_gyro_data[3]) / 333.87 + 21.0;  // Temperature in degrees Centigrade

	//Get gyroscope
	raw_acc_gyro_data[4] = (((int16_t)buffer[8]) << 8) | buffer[9];
    imuH->g[0] = (float)raw_acc_gyro_data[4] * imuH->gyro_resolution;
	raw_acc_gyro_data[5] = (((int16_t)buffer[10]) << 8) | buffer[11];
    imuH->g[1] = (float)raw_acc_gyro_data[5] * imuH->gyro_resolution;
	raw_acc_gyro_data[6] = (((int16_t)buffer[12]) << 8) | buffer[13];
    imuH->g[2] = (float)raw_acc_gyro_data[6] * imuH->gyro_resolution;
}

/*** Private functions ***/

/**
 * Test response of module
 */
uint8_t _isConnected(void) {
	uint8_t has_connected = _isConnectedMPU9250() && _isConnectedAK8963();
	return has_connected;
}

/**
 * Test response of mpu9250
 */
uint8_t _isConnectedMPU9250() {
	uint8_t c = 0;
	imuSerial_readReg(MPU9250_ADDR, MPU9250_WHO_AM_I, &c);
	uint8_t b = (c == MPU9250_WHO_AM_I_RESULT);
	return b;
}

/**
 * Test response of AK8963
 */
uint8_t _isConnectedAK8963() {
	uint8_t c = 0;
	imuSerial_readReg(AK8963_ADDR, AK8963_WIA, &c);
	uint8_t b = (c == AK8963_WHO_AM_I_RESULT);
	return b;
}


void _set_acc_gyro_to_calibration() {
	// reset device
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x01);
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x00);
	osDelay(10);

	// Configure device for bias calculation
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_INT_ENABLE, 0x00); // Disable all interrupts
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_FIFO_EN, 0x00); // Disable fifo
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_FIFO_EN, 0x00); // Disable fifo
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x00); // Turn on internal clock source
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_I2C_MST_CTRL, 0x00); // Disable I2C master
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_USER_CTRL, 0x00); // Disable FIFO and I2C master modes
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_USER_CTRL, 0x0C); // Reset FIFO and DMP

	// Configure MPU6050 gyro and accelerometer for bias calculation
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_CONFIG, 0x01); // Set low-pass filter to 188 Hz
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_SMPLRT_DIV, 0x00); // Set sample rate to 1 kHz
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_GYRO_CONFIG, 0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_USER_CTRL, 0x40); // Enable FIFO
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_FIFO_EN, 0x78); // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	osDelay(40);  // accumulate 40 samples in 40 milliseconds = 480 bytes
}

float _get_acc_resolution(const ACCEL_FS_SEL_t accel_af_sel) {
	switch (accel_af_sel) {
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case A2G:
			return 2.0 / 32768.0;
		case A4G:
			return 4.0 / 32768.0;
		case A8G:
			return 8.0 / 32768.0;
		case A16G:
			return 16.0 / 32768.0;
		default:
			return 0.;
	}
}

float _get_gyro_resolution(const GYRO_FS_SEL_t gyro_fs_sel) {
	switch (gyro_fs_sel) {
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case G250DPS:
			return 250.0 / 32768.0;
		case G500DPS:
			return 500.0 / 32768.0;
		case G1000DPS:
			return 1000.0 / 32768.0;
		case G2000DPS:
			return 2000.0 / 32768.0;
		default:
			return 0.;
	}
}

float _get_mag_resolution(const MAG_OUTPUT_BITS_t mag_output_bits) {
	switch (mag_output_bits) {
		// Possible magnetometer scales (and their register bit settings) are:
		// 14 bit resolution (0) and 16 bit resolution (1)
		// Proper scale to return milliGauss
		case M14BITS:
			return 10. * 4912. / 8190.0;
		case M16BITS:
			return 10. * 4912. / 32760.0;
		default:
			return 0.;
	}
}
