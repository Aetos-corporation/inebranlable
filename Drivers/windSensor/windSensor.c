/*
 * windSensor.c
 *
 *  Created on: Jan 12, 2023
 *  Author  : Thibault CAPEL
 *  Purpose : Ce script utilise les fonctions de la bibliothèque HAL
 *  pour configurer les broches du codeur incrémental en tant qu'entrées
 *  et utilise les interruptions pour détecter les changements de niveau.
 *  Il utilise également une boucle infinie pour afficher l'angle calculé
 *  en utilisant les impulsions comptées.
 */


#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include <math.h>
#include <windSensor/windSensor.h>
#include <i2c.h>
#include <log/log.h>
#include <system/system.h>

#define AS5600_ADDR 0x36
#define AS5600_REG_RAW_ANGLE_HIGH 0x0E // Utilisez cette adresse pour lire la partie haute de l'angle brut
#define AS5600_REG_RAW_ANGLE_LOW 0x0F // Utilisez cette adresse pour lire la partie basse de l'angle brut

uint8_t I2C_Error = 0;
float windAngle = 0;

void windSensorTask(void * argument);
void windSensor_Init(void);
uint16_t AS5600_ReadAngle(void);
//void I2C_Scan(I2C_HandleTypeDef *hi2c);
//uint8_t AS5600_ReadVersion(void);

void StartWindSensorTask(void * argument)
{
    for(;;)
    {
        uint16_t raw_angle = AS5600_ReadAngle();
        float angle = ((float)raw_angle / 4096.0f) * 360.0f; // Calculate angle
        angle = fabs(angle);
        angle = fmodf(angle, 360.0f); // Wrap angle between 0 and 360
        LOG_GIROUETTE(angle);

        windAngle = angle;
		osDelay(SENSOR_UPDATE_RATE);
    }
}

//void windSensor_Init()
//{
//    osThreadId_t windSensorHandle;
//    const osThreadAttr_t windSensor_attributes = {
//        .name = "windSensor",
//        .stack_size = 128 * 4,
//        .priority = (osPriority_t) osPriorityHigh,
//    };
//
//    windSensorHandle = osThreadNew(windSensorTask, NULL, &windSensor_attributes);
//}

//void I2C_Scan(I2C_HandleTypeDef *hi2c) {
//    // printf("Scanning I2C bus:\r\n");
//    HAL_StatusTypeDef res;
//    for (uint16_t i = 0; i < 128; i++) {
//        res = HAL_I2C_IsDeviceReady(hi2c, i << 1, 1, 10);
//        if (res == HAL_OK) {
//            //printf("0x%02X", i);
//        } else {
//            //printf(".");
//        }
//    }
//    // printf("\r\n");
//}

//uint8_t AS5600_ReadVersion(void) {
//    uint8_t version;
//    HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR << 1, 0x0F, I2C_MEMADD_SIZE_8BIT, &version, 1, HAL_MAX_DELAY);
//    return version;
//}


uint16_t AS5600_ReadAngle(void)
{
    uint8_t high_byte[1], low_byte[1];
    uint16_t pos = 0;

    HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR << 1, AS5600_REG_RAW_ANGLE_HIGH, I2C_MEMADD_SIZE_8BIT, high_byte, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR << 1, AS5600_REG_RAW_ANGLE_LOW, I2C_MEMADD_SIZE_8BIT, low_byte, 1, HAL_MAX_DELAY);

    pos = ((uint16_t)high_byte[0] << 8)| low_byte[0];

    //printf("Valeur de pos: %d\n", pos);
    return pos;
}

float getWindAngle(){
	return windAngle;
}
