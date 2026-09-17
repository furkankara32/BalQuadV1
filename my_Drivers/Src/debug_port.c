/*
 * debug_port.c
 *
 *  Created on: 10 Eyl 2026
 *      Author: furkan
 */

#include "debug_port.h"

static TxRingBuffer_t * volatile active_buffer = NULL;

static volatile uint16_t active_dma_length = 0;
static volatile DMAStatus_t dma_state = DMA_STATUS_IDLE;

/**
  * @brief  This function writes data to the ring buffer.
  * @param  TxRingBuffer_t *ringBuffer = Ring buffer struct
  * @param  const uint8_t *data = Data which is written buffer.
  * @param  uint16_t length = length of the data
  * @retval @def_group BufferStatus_t
  */
BufferStatus_t RingBufferWrite(TxRingBuffer_t *ringBuffer,const uint8_t *data, uint16_t length)
{
	uint16_t used_space = 0;
	uint16_t empty_space = 0;
	uint16_t write_index = 0;


	used_space = ( (ringBuffer->head) + DEBUG_TX_BUFFER_SIZE - (ringBuffer->tail) ) % DEBUG_TX_BUFFER_SIZE;
	empty_space =  DEBUG_TX_BUFFER_SIZE - (used_space + 1);

	if(empty_space < length)
	{
		return BUFFER_STATUS_FULL;
	}
	write_index = ringBuffer->head;


	for(uint16_t i = 0; i < length; i++)
	{
		ringBuffer->data[write_index]  = *(data + i);

		write_index++;

		if(write_index >= DEBUG_TX_BUFFER_SIZE)
		{
			write_index = 0;
		}
	}

	ringBuffer->head = write_index;

	return BUFFER_STATUS_OK;
}

DebugStatus_t DebugSend_DMA(TxRingBuffer_t *debugData)
{

	uint16_t contiguous_length;
	HAL_StatusTypeDef debug_state;

	if(dma_state == DMA_STATUS_BUSY)
	{
		return DEBUG_STATUS_BUSY;
	}
	if(debugData->head == debugData->tail)
	{
		return DEBUG_STATUS_OK;
	}

	if(debugData->tail > debugData->head)
	{
		contiguous_length = (DEBUG_TX_BUFFER_SIZE ) - debugData->tail;

	}else
	{
		contiguous_length = debugData->head - debugData->tail;
	}

	active_buffer = debugData;
	active_dma_length = contiguous_length;
	dma_state = DMA_STATUS_BUSY;

	debug_state = HAL_UART_Transmit_DMA(&huart3, &(debugData->data[debugData->tail] ), active_dma_length);

	if(debug_state == HAL_OK)
	{
		return DEBUG_STATUS_OK;

	}else if(debug_state == HAL_BUSY)
	{
		active_buffer = NULL;
		active_dma_length = 0;
		dma_state = DMA_STATUS_IDLE;
		return DEBUG_STATUS_BUSY;

	}else
	{
		active_buffer = NULL;
		active_dma_length = 0;
		dma_state = DMA_STATUS_IDLE;
		return DEBUG_STATUS_ERROR;
	}


	return DEBUG_STATUS_OK;

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

  if( (huart->Instance == USART3 ) && (active_buffer != NULL) )
  {
	  active_buffer->tail += active_dma_length;
	  if(active_buffer->tail == DEBUG_TX_BUFFER_SIZE)
	  {
		  active_buffer->tail = 0;
	  }
	  active_dma_length = 0;
	  dma_state = DMA_STATUS_IDLE;

	  if(active_buffer->tail != active_buffer->head)
	  {
		  DebugSend_DMA(active_buffer);
	  }else
	  {
		  active_buffer = NULL;
	  }

  }
}
