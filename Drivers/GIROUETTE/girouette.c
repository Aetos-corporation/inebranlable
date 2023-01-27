/*
 * girouette.c
 *
 *  Created on: Jan 23, 2023
 *      Author: Arthur R-P
 */

#include "girouette.h"

void startGirouetteTask(void const * argument){
	volatile int pos = 0;

	for(;;){
		int A = HAL_GPIO_ReadPin(GIR_A_GPIO_Port, GIR_A_Pin);
		int B = HAL_GPIO_ReadPin(GIR_B_GPIO_Port, GIR_B_Pin);

		if(A == B){
			pos = pos + 1;
		}
		else
		{
			pos = pos - 1;
		}
		PRINT("%d", pos);
		osDelay(1000);
	}
}
