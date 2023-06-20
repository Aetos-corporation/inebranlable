/*
 * system.c
 *
 *  Created on: May 23, 2023
 *      Author: Bastien
 */

#include "main.h"

uint8_t initFlags;

void sys_setInitFlag(uint8_t mask)
{
	initFlags |= mask;
}

void sys_resetInitFlag(uint8_t mask)
{
	initFlags &= ~(mask);
}

uint8_t sys_testInitFlag(uint8_t mask)
{
	return (initFlags & mask);
}

uint8_t sys_isInit()
{
	return (initFlags == 0x1F);
}
