
/*
 * fifo.h
 *
 *  Created on: May 16, 2016
 *      Author: kamil
 */

#ifndef FIFO_H_INCLUDED
#define FIFO_H_INCLUDED

#include <stdint.h>


#define FIFO_SIZE  512
#define FIFO_ELEMENTS  (FIFO_SIZE - 1)

#define FIFO_ERROR_OVERRUN 	(1)
#define FIFO_SUCCESS 		(0)
typedef union{
	 unsigned char  byte[FIFO_SIZE];
	 uint32_t 		word[FIFO_SIZE/4];
}queue_u;

typedef struct{
	 queue_u queue;
	 int queue_in;
	 int queue_out;
}fifo_t;


/** \brief Function to init FIFO
 *
 * \param fifo - pointer to queue
 */
void Fifo_Init(fifo_t* fifo);

/** \brief Function to put item into FIFO
 *
 * \param fifo - pointer to queue
 * \param item - item to put
 */
uint32_t Fifo_Put(fifo_t* fifo, unsigned char item);

/** \brief Function to get item from FIFO
 *
 * \param fifo - pointer to queue
 * \param item - pointer to where get item
 */
uint32_t Fifo_Get(fifo_t* fifo, unsigned char* item);




#endif /* FIFO_H_INCLUDED */
