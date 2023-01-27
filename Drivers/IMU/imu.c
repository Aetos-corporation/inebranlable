/*
 * imu.c
 *
 *  Created on: Jan 14, 2023
 *      Author: Bastien
 */

#include "imu.h"
#include "imuSerial.h"
#include "imu_RegisterMap.h"

#include "i2c.h"
#include "trace.h"

#include <cmsis_os.h>

/*** Private functions prototypes ***/
uint8_t _isConnected(void);
uint8_t _isConnectedMPU9250(void);
uint8_t _isConnectedAK8963(void);

const uint16_t CALIB_GYRO_SENSITIVITY = 131;     // LSB/degrees/sec
const uint16_t CALIB_ACCEL_SENSITIVITY = 16384;  // LSB/g


/*** public functions declarations ***/

void StartImuTask(void const * argument)
{
	PRINT("Init IMU...");
	if(imu_init(&hi2c1) == 0)
		PRINT("done\n");
	else
	{
		PRINT("ERROR!\n");
		while(1);
	}

	for(;;)
	{
		int16_t accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ;
		imu_getMotion9(&accX, &accY, &accZ, &gyrX, &gyrY, &gyrZ, &magX, &magY, &magZ);

		PRINT("Accelero - x: %6d  y: %6d  z: %6d\n", accX, accY, accZ);
		PRINT("Gyro     - x: %6d  y: %6d  z: %6d\n", gyrX, gyrY, gyrZ);
		PRINT("Magneto  - x: %6d  y: %6d  z: %6d\n\n", magX, magY, magZ);

		osDelay(1000);
	}
}

uint8_t imu_init(I2C_HandleTypeDef* i2c)
{
	imuSerial_Init(i2c);

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_INT_PIN_CFG, 0x22);
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

	//Slave addr 0 to 0x0C
	uint8_t data = 0;
	imuSerial_readReg(MPU9250_ADDR, MPU9250_I2C_SLV0_ADDR, &data);
	PRINT("slv0_addr = %d\n", data);
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_I2C_SLV0_ADDR, 0x0C);

	if(!_isConnectedMPU9250())
		return 2;

//*** MPU9250
	// get stable time source
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else

	//TODO calibration


//*** AK8963
	//TODO calibration

	return 0;
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
void imu_getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx,
                         int16_t* my, int16_t* mz) {
    //get accel and gyro
    imu_getMotion6(ax, ay, az, gx, gy, gz);

    //get magneto
    imu_getDirection(mx, my, mz);
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
void imu_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
	uint8_t buffer[14];
	imuSerial_readRegBurst(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 14, buffer);
    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    //skip temp values
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

/** Get 3-axis accelerometer readings.
    These registers store the most recent accelerometer measurements.
    Accelerometer measurements are written to these registers at the Sample Rate
    as defined in Register 25.

    The accelerometer measurement registers, along with the temperature
    measurement registers, gyroscope measurement registers, and external sensor
    data registers, are composed of two sets of registers: an internal register
    set and a user-facing read register set.

    The data within the accelerometer sensors' internal register set is always
    updated at the Sample Rate. Meanwhile, the user-facing read register set
    duplicates the internal register set's data values whenever the serial
    interface is idle. This guarantees that a burst read of sensor registers will
    read measurements from the same sampling instant. Note that if burst reads
    are not used, the user is responsible for ensuring a set of single byte reads
    correspond to a single sampling instant by checking the Data Ready interrupt.

    Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
    (Register 28). For each full scale setting, the accelerometers' sensitivity
    per LSB in ACCEL_xOUT is shown in the table below:

    <pre>
    AFS_SEL | Full Scale Range | LSB Sensitivity
    --------+------------------+----------------
    0       | +/- 2g           | 8192 LSB/mg
    1       | +/- 4g           | 4096 LSB/mg
    2       | +/- 8g           | 2048 LSB/mg
    3       | +/- 16g          | 1024 LSB/mg
    </pre>

    @param x 16-bit signed integer container for X-axis acceleration
    @param y 16-bit signed integer container for Y-axis acceleration
    @param z 16-bit signed integer container for Z-axis acceleration
*/
void imu_getAcceleration(int16_t* x, int16_t* y, int16_t* z) {
	uint8_t buffer[6];
	imuSerial_readRegBurst(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** Get X-axis accelerometer reading.
    @return X-axis acceleration measurement in 16-bit 2's complement format
    @see getMotion6()
*/
int16_t imu_getAccelerationX() {
	uint8_t buffer[2];
	imuSerial_readRegBurst(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Y-axis accelerometer reading.
    @return Y-axis acceleration measurement in 16-bit 2's complement format
    @see getMotion6()
*/
int16_t imu_getAccelerationY() {
	uint8_t buffer[2];
	imuSerial_readRegBurst(MPU9250_ADDR, MPU9250_ACCEL_YOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Z-axis accelerometer reading.
    @return Z-axis acceleration measurement in 16-bit 2's complement format
    @see getMotion6()
*/
int16_t imu_getAccelerationZ() {
	uint8_t buffer[2];
	imuSerial_readRegBurst(MPU9250_ADDR, MPU9250_ACCEL_ZOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// GYRO_*OUT_* registers

/** Get 3-axis gyroscope readings.
    These gyroscope measurement registers, along with the accelerometer
    measurement registers, temperature measurement registers, and external sensor
    data registers, are composed of two sets of registers: an internal register
    set and a user-facing read register set.
    The data within the gyroscope sensors' internal register set is always
    updated at the Sample Rate. Meanwhile, the user-facing read register set
    duplicates the internal register set's data values whenever the serial
    interface is idle. This guarantees that a burst read of sensor registers will
    read measurements from the same sampling instant. Note that if burst reads
    are not used, the user is responsible for ensuring a set of single byte reads
    correspond to a single sampling instant by checking the Data Ready interrupt.

    Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
    (Register 27). For each full scale setting, the gyroscopes' sensitivity per
    LSB in GYRO_xOUT is shown in the table below:

    <pre>
    FS_SEL | Full Scale Range   | LSB Sensitivity
    -------+--------------------+----------------
    0      | +/- 250 degrees/s  | 131 LSB/deg/s
    1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
    2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
    3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
    </pre>

    @param x 16-bit signed integer container for X-axis rotation
    @param y 16-bit signed integer container for Y-axis rotation
    @param z 16-bit signed integer container for Z-axis rotation
    @see getMotion6()
*/
void imu_getRotation(int16_t* x, int16_t* y, int16_t* z) {
	uint8_t buffer[6];
	imuSerial_readRegBurst(MPU9250_ADDR, MPU9250_GYRO_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** Get X-axis gyroscope reading.
    @return X-axis rotation measurement in 16-bit 2's complement format
    @see getMotion6()
*/
int16_t imugetRotationX() {
	uint8_t buffer[2];
	imuSerial_readRegBurst(MPU9250_ADDR, MPU9250_GYRO_XOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Y-axis gyroscope reading.
    @return Y-axis rotation measurement in 16-bit 2's complement format
    @see getMotion6()
*/
int16_t imugetRotationY() {
	uint8_t buffer[2];
	imuSerial_readRegBurst(MPU9250_ADDR, MPU9250_GYRO_YOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Z-axis gyroscope reading.
    @return Z-axis rotation measurement in 16-bit 2's complement format
    @see getMotion6()
*/
int16_t imugetRotationZ() {
	uint8_t buffer[2];
	imuSerial_readRegBurst(MPU9250_ADDR, MPU9250_GYRO_ZOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}


/** Get 3-axis magnetometer readings.

    @param x 16-bit signed integer container for X-axis rotation
    @param y 16-bit signed integer container for Y-axis rotation
    @param z 16-bit signed integer container for Z-axis rotation
    @see getMotion9()
*/
void imu_getDirection(int16_t* x, int16_t* y, int16_t* z)
{
	uint8_t ST1;
	imuSerial_writeReg(AK8963_ADDR, 0x0A, 0x01);
	do
	{
		imuSerial_readReg(AK8963_ADDR, 0x02, &ST1);
	} while (!(ST1 & 0x01));

	uint8_t buffer[6];
	imuSerial_readRegBurst(AK8963_ADDR, AK8963_HXH, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

int16_t imu_getDirectionX()
{
	uint8_t ST1;
	imuSerial_writeReg(AK8963_ADDR, 0x0A, 0x01);
	do
	{
		imuSerial_readReg(AK8963_ADDR, 0x02, &ST1);
	} while (!(ST1 & 0x01));

	uint8_t buffer[2];
	imuSerial_readRegBurst(AK8963_ADDR, AK8963_HXH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

int16_t imu_getDirectionY()
{
	uint8_t ST1;
	imuSerial_writeReg(AK8963_ADDR, 0x0A, 0x01);
	do
	{
		imuSerial_readReg(AK8963_ADDR, 0x02, &ST1);
	} while (!(ST1 & 0x01));

	uint8_t buffer[2];
	imuSerial_readRegBurst(AK8963_ADDR, AK8963_HYH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

int16_t imu_getDirectionZ()
{
	uint8_t ST1;
	imuSerial_writeReg(AK8963_ADDR, 0x0A, 0x01);
	do
	{
		imuSerial_readReg(AK8963_ADDR, 0x02, &ST1);
	} while (!(ST1 & 0x01));

	uint8_t buffer[2];
	imuSerial_readRegBurst(AK8963_ADDR, AK8963_HZH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
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

void collect_acc_gyro_data_to(float* a_bias, float* g_bias) {
	// At end of sample accumulation, turn off FIFO sensor read
	uint8_t data[12];                                    // data array to hold accelerometer and gyro x, y, z, data
	imuSerial_writeReg(MPU9250_ADDR, MPU9250_FIFO_EN, 0x00); // Disable gyro and accelerometer sensors for FIFO
	imuSerial_readRegBurst(MPU9250_ADDR, MPU9250_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count

	uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
	uint16_t packet_count = fifo_count / 12;  // How many sets of full gyro and accelerometer data for averaging

	for(uint16_t ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		imuSerial_readRegBurst(MPU9250_ADDR, MPU9250_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
		imuSerial_readRegBurst(MPU9250_ADDR, MPU9250_FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
		gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
		gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
		gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

		a_bias[0] += (float)accel_temp[0];  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		a_bias[1] += (float)accel_temp[1];
		a_bias[2] += (float)accel_temp[2];
		g_bias[0] += (float)gyro_temp[0];
		g_bias[1] += (float)gyro_temp[1];
		g_bias[2] += (float)gyro_temp[2];
	}
	a_bias[0] /= (float)packet_count;  // Normalize sums to get average count biases
	a_bias[1] /= (float)packet_count;
	a_bias[2] /= (float)packet_count;
	g_bias[0] /= (float)packet_count;
	g_bias[1] /= (float)packet_count;
	g_bias[2] /= (float)packet_count;

	if (a_bias[2] > 0L) {
		a_bias[2] -= (float)CALIB_ACCEL_SENSITIVITY;
	}  // Remove gravity from the z-axis accelerometer bias calculation
	else {
		a_bias[2] += (float)CALIB_ACCEL_SENSITIVITY;
	}
}
