/*
 * fifo.c
 *
 *  Created on: Dec 14, 2022
 *      Author: bastienJ
 */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <xbee/fifo.h>

#include "log/frame.h"

static uint16_t FifoNext( Fifo_t *fifo, uint16_t index )
{
    return ( index + 1 ) % fifo->fifoSize;
}

/*!
 * Initializes the FIFO structure.
 * Dynamically allocate needed space, based on given parameters
 *
 * \param [IN] fifo   	 Pointer to the FIFO handler
 * \param [IN] dataSize  Size of each element stored
 * \param [IN] fifoSize  Number of element to store in fifo
 */
void FifoInit( Fifo_t *fifo, const size_t dataSize, const size_t fifoSize )
{
	//Allocate memory
	fifo->dataArray = (void**) malloc(fifoSize * sizeof(void*));
	for(uint8_t i=0 ; i<fifoSize ; i++)
		fifo->dataArray[i] = (void*) malloc(dataSize);

    fifo->head = 0;
    fifo->tail = 0;
    fifo->dataSize = dataSize;
    fifo->fifoSize = fifoSize;
}

/*!
 *  Free all the memory (not the handle object)
 *  And reset fifo parameters
 */
void FifoDeInit( Fifo_t *fifo )
{
	//Free memory array
	for(uint32_t i=0 ; i<(fifo->fifoSize) ; i++)
		free(fifo->dataArray[i]);
	free(fifo->dataArray);

	//Flush handler
	FifoFlush(fifo);
}

/*!
 * Pushes data in the FIFO
 * copy content of input to storage
 *
 * \param [IN] fifo Pointer to the FIFO object
 * \param [IN] data Data to be pushed into the FIFO
 */
void FifoPush( Fifo_t *fifo, const void *in )
{
    fifo->tail = FifoNext( fifo, fifo->tail );
    memcpy(fifo->dataArray[fifo->tail], in, fifo->dataSize);
}

/*!
 * Pops data out of the FIFO
 * copy content of storage to output
 *
 * \param [IN] fifo Pointer to the FIFO object
 * \retval data     Data popped from the FIFO
 */
void FifoPop( Fifo_t *fifo, void *out )
{
    memcpy(out, fifo->dataArray[FifoNext( fifo, fifo->head )], fifo->dataSize);
    fifo->head = FifoNext( fifo, fifo->head );
}

/*!
 * Flushes the FIFO
 *
 * \param [IN] fifo   Pointer to the FIFO object
 */
void FifoFlush( Fifo_t *fifo )
{
    fifo->head = 0;
    fifo->tail = 0;
}

/*!
 * Returns the number of elements stored
 *
 * \param [IN] fifo   Pointer to the FIFO object
 * \retval count
 */
uint32_t FifoCount( Fifo_t *fifo )
{
	return ( fifo->head - fifo->tail ) %  fifo->fifoSize;
}

/*!
 * Return true if the FIFO is empty
 *
 * \param [IN] fifo   Pointer to the FIFO object
 * \retval isEmpty    true: FIFO is empty, false FIFO is not empty
 */
bool IsFifoEmpty( Fifo_t *fifo )
{
    return ( fifo->head == fifo->tail );
}

/*!
 * Checks if the FIFO is full
 *
 * \param [IN] fifo   Pointer to the FIFO object
 * \retval isFull     true: FIFO is full, false FIFO is not full
 */
bool IsFifoFull( Fifo_t *fifo )
{
    return ( FifoNext( fifo, fifo->tail ) == fifo->head );
}
