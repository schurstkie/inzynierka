/*
 * fifo.c
 *
 *  Created on: May 16, 2016
 *      Author: kamil
 */


#include "fifo.h"

void Fifo_Init(fifo_t* fifo)
{
	fifo->queue_in = 0;
	fifo->queue_out = 0;
}

uint32_t Fifo_Put(fifo_t* fifo, unsigned char item)
{
	if(fifo->queue_in == ((fifo->queue_out - 1 + FIFO_SIZE) % FIFO_SIZE))
	{
		return FIFO_ERROR_OVERRUN; // FIFO full
	}
	fifo->queue.byte[fifo->queue_in] = item;
	fifo->queue_in = (fifo->queue_in + 1) % FIFO_SIZE;
	return FIFO_SUCCESS;
}

uint32_t Fifo_Get(fifo_t* fifo, unsigned char* item)
{

	if(fifo->queue_in == fifo->queue_out)
	{
		return FIFO_ERROR_OVERRUN; //FIFO empty
	}
	*item = fifo->queue.byte[fifo->queue_out];
	if(*item == 177)
		__NOP();
	fifo->queue_out = (fifo->queue_out + 1) % FIFO_SIZE;
	return FIFO_SUCCESS;
}
