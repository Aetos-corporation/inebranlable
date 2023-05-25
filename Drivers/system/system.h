/*
 * system.h
 *
 *  Created on: May 23, 2023
 *      Author: Bastien
 */

#ifndef SYSTEM_SYSTEM_H_
#define SYSTEM_SYSTEM_H_

#define SYS_MASK_GPS  0x1
#define SYS_MASK_XBEE 0x2
#define SYS_MASK_IMU  0x4
#define SYS_MASK_PWM  0x8
#define SYS_MASK_GIR  0x10

void sys_setInitFlag(uint8_t mask);
void sys_resetInitFlag(uint8_t mask);
uint8_t sys_testInitFlag(uint8_t mask);
uint8_t sys_isInit();

#endif /* SYSTEM_SYSTEM_H_ */
