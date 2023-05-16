/*
 * fifo.h
 *
 *  Created on: Jan 6, 2023
 *      Author: Bastien
 */
#ifndef __FIFO_H__
#define __FIFO_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

/*!
 * FIFO structure
 */
typedef struct Fifo_s
{
    uint16_t head;
    uint16_t tail;
    void*	 *dataArray;
    uint32_t dataSize;
    uint32_t fifoSize;
}Fifo_t;

/*!
 * Initializes the FIFO structure.
 * Dynamically allocate needed space, based on given parameters
 *
 * \param 	   fifo   	 Pointer to the FIFO handler
 * \param [IN] dataSize  Size of each element stored
 * \param [IN] fifoSize  Number of element to store in fifo
 */
void FifoInit( Fifo_t *fifo, const size_t dataSize, const size_t fifoSize );

/*!
 *  Free all the memory (not the handle object)
 *  And reset fifo parameters
 */
void FifoDeInit( Fifo_t *fifo );

/*!
 * Pushes data in the FIFO
 *
 * \param [IN] fifo Pointer to the FIFO object
 * \param [IN] data Data to be pushed into the FIFO
 */
void FifoPush( Fifo_t *fifo, const void *in );

/*!
 * Pops data out of the FIFO
 *
 * \param [IN] fifo Pointer to the FIFO object
 * \retval data     Data popped from the FIFO
 */
void FifoPop( Fifo_t *fifo, void *out );

/*!
 * Flushes the FIFO
 *
 * \param [IN] fifo   Pointer to the FIFO object
 */
void FifoFlush( Fifo_t *fifo );

/*!
 * Returns the number of elements stored
 *
 * \param [IN] fifo   Pointer to the FIFO object
 * \retval count
 */
uint32_t FifoCount( Fifo_t *fifo );

/*!
 * Checks if the FIFO is empty
 *
 * \param [IN] fifo   Pointer to the FIFO object
 * \retval isEmpty    true: FIFO is empty, false FIFO is not empty
 */
bool IsFifoEmpty( Fifo_t *fifo );

/*!
 * Checks if the FIFO is full
 *
 * \param [IN] fifo   Pointer to the FIFO object
 * \retval isFull     true: FIFO is full, false FIFO is not full
 */
bool IsFifoFull( Fifo_t *fifo );

#ifdef __cplusplus
}
#endif

#endif // __FIFO_H__
