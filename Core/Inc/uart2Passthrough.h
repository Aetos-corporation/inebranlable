/*
 * uart2Passthrough.h
 *
 *  Created on: Jan 12, 2023
 *      Author: Bastien
 */

#ifndef SRC_UART2PASSTHROUGH_H_
#define SRC_UART2PASSTHROUGH_H_

void u2pt_StartReceive(void);
void u2pt_StopReceive(void);
void u2pt_rxCallback(void);

#endif /* SRC_UART2PASSTHROUGH_H_ */
