/*
 * debug_port.h
 *
 *  Created on: 10 Eyl 2026
 *      Author: furkan
 */

#ifndef INC_DEBUG_PORT_H_
#define INC_DEBUG_PORT_H_

#include "usart.h"
#include <stdint.h>

#define DEBUG_TX_BUFFER_SIZE	512U // Ring Buffer Size



/*
 * @def_group BufferStatus_t = Defines buffer status.
 */
typedef enum
{
	BUFFER_STATUS_OK = 0,
	BUFFER_STATUS_FULL,
	BUFFER_STATUS_EMPTY

}BufferStatus_t;

typedef enum
{
	DEBUG_STATUS_OK =0,
	DEBUG_STATUS_BUSY,
	DEBUG_STATUS_ERROR

}DebugStatus_t;

typedef enum
{
	DMA_STATUS_IDLE =0,
	DMA_STATUS_BUSY,
	DMA_STATUS_ERROR

}DMAStatus_t;
typedef struct
{
	uint8_t data[DEBUG_TX_BUFFER_SIZE];

	volatile uint16_t head;
	volatile uint16_t tail;

}TxRingBuffer_t;

BufferStatus_t RingBufferWrite(TxRingBuffer_t *ringBuffer,const uint8_t *data, uint16_t length);

DebugStatus_t DebugSend_DMA(TxRingBuffer_t *debugData);



#endif /* INC_DEBUG_PORT_H_ */
