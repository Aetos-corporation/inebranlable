/*
 * cmd_servo.h
 *
 *  Created on: Jan 23, 2023
 *      Author: clemn
 */
#ifndef CMD_SERVO_CMD_SERVO_H_
#define CMD_SERVO_CMD_SERVO_H_

#include <tim.h>
#include "cmsis_os.h"

void startPWMTask(void const * argument);

void set_PWM_value_voile(int val);
void set_PWM_value_safran(int val);
void execute_exemple_voile();
void execute_exemple_safran();

#endif /* CMD_SERVO_CMD_SERVO_H_ */
