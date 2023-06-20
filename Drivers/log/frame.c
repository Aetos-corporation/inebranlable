/*
 * frame.c
 *
 *  Created on: 21 mars 2023
 *      Author: Bastien
 */
#include <string.h>
#include <stdlib.h>

#include "log/frame.h"
#include "main.h"


void frameCreate(genericFrame_t* frame)
{
	frame->codeFunc = 0;
	frame->mode = 0;
	frame->dataSize = 0;
	frame->data = malloc(MAX_DATA_SIZE);
	memset(frame->data, 0x0, MAX_DATA_SIZE);
}

void frameDelete(genericFrame_t* frame)
{
	free(frame->data);
}

void parse(uint8_t* rawBuffer, uint32_t rawBufferSize, genericFrame_t* frame)
{
	memcpy(rawBuffer, (uint8_t*) &frame, rawBuffer[3] + 3);
}

void serialize(const genericFrame_t frame, uint8_t* rawBuffer, uint32_t* rawBufferSize)
{
	rawBuffer[0] = frame.codeFunc;
	rawBuffer[1] = frame.mode;
	rawBuffer[2] = frame.dataSize;
	memcpy(&rawBuffer[3], (uint8_t*) frame.data, frame.dataSize);
	*rawBufferSize = 3 + frame.dataSize;
}
