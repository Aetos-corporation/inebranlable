/*
 * frame.h
 *
 *  Created on: 21 mars 2023
 *      Author: Bastien
 */

#ifndef LOG_FRAME_H_
#define LOG_FRAME_H_

#include <assert.h>

#include "main.h"

#define MAX_DATA_SIZE  253

typedef struct __attribute__((packed))
{
	uint8_t  codeFunc;
	uint8_t  mode;
	uint8_t  dataSize;
	uint8_t*  data;  //WARING: This is a pointer! It needs to be allocated.
} genericFrame_t ;

void frameCreate(genericFrame_t* frame);
void frameDelete(genericFrame_t* frame);

void parse(uint8_t* rawBuffer, uint32_t rawBufferSize, genericFrame_t* frame);
void serialize(const genericFrame_t frame, uint8_t* rawBuffer, uint32_t* rawBufferSize);

#endif /* LOG_FRAME_H_ */
