/*
 * bno085.c
 *
 *  Created on: 10 Eyl 2026
 *      Author: furkan
 */


#include "bno085.h"

typedef enum
{
	BNO085_SPI_IDLE = 0,
	BNO085_SPI_RX_HEADER,
	BNO085_SPI_WAIT_PACKET,
	BNO085_SPI_RX_PACKET,
	BNO085_SPI_ERROR

}BNO085_SPI_State_t;

typedef struct
{
	uint16_t length;
	uint8_t channel;
	uint8_t sequence;
	uint8_t continuation;
}BNO085_SHTP_Header_t;


#define BNO085_RESET_TIMEOUT_MS			200U		//MAX RESET TIMEOUT VALUE
#define BNO085_RX_BUFFER_SIZE 			512U

static volatile BNO085_SPI_State_t spi_state = BNO085_SPI_IDLE;
static volatile uint8_t bno085_int_flag = 0U;

static BNO085_SHTP_Header_t BNO085_Header =  {0};
static uint8_t shtp_header_rx[BNO085_SHTP_HEADER_SIZE];
static uint8_t shtp_header_tx[BNO085_SHTP_HEADER_SIZE] = {0};
static volatile uint8_t bno085_header_ready = 0U;

static BNO085_SHTP_Header_t BNO085_PacketHeader =  {0};
static uint8_t shtp_packet_rx[BNO085_RX_BUFFER_SIZE];
static uint8_t shtp_packet_tx[BNO085_RX_BUFFER_SIZE] = {0};
static volatile uint8_t bno085_packet_ready = 0U;


/*
 * STATIC FUNCTION PROTOTYPES
 */

static BNO085_Status_t BNO085_HardwareReset(void);
static BNO085_Status_t BNO085_StartHeaderRead(void);
static BNO085_Status_t BNO085_ParseHeader(const uint8_t *raw_header, BNO085_SHTP_Header_t *header);
static BNO085_Status_t BNO085_StartPacketRead(void);

/*
 * ******************************************STATIC FUNCTIONS****************************************************
 */
static BNO085_Status_t BNO085_HardwareReset(void)
{
	// 1. CS = High
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

	// 2. P0/WAKE = High
	HAL_GPIO_WritePin(SPI2_WAKE_GPIO_Port, SPI2_WAKE_Pin, GPIO_PIN_SET);

	//3. RST = LOW
	HAL_GPIO_WritePin(SPI2_RST_GPIO_Port, SPI2_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);

	//4.RST = HIGH
	HAL_GPIO_WritePin(SPI2_RST_GPIO_Port, SPI2_RST_Pin, GPIO_PIN_SET);

	//Wait BNO085 reboot and INT low in max 200ms
	uint32_t start_tick = HAL_GetTick();

	while(HAL_GPIO_ReadPin(SPI2_INT_GPIO_Port, SPI2_INT_Pin) == GPIO_PIN_SET)
	{
		if( (HAL_GetTick() - start_tick ) >= BNO085_RESET_TIMEOUT_MS)
		{
			return BNO085_STATUS_TIMEOUT;
		}
	}

	return BNO085_STATUS_OK;

}

static BNO085_Status_t BNO085_StartHeaderRead(void)
{
	//1. Check SPI state
	if(spi_state != BNO085_SPI_IDLE)
	{
		return BNO085_STATUS_BUSY;
	}

	//2. Check INT pin
	if(HAL_GPIO_ReadPin(SPI2_INT_GPIO_Port, SPI2_INT_Pin) == GPIO_PIN_SET)
	{
		return BNO085_STATUS_BUSY;
	}

	//3. Start communication-> CS = LOW
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

	//4. Change state to header
	spi_state = BNO085_SPI_RX_HEADER;


	//5. Start DMA for header
	if(HAL_SPI_TransmitReceive_DMA(&hspi2, shtp_header_tx, shtp_header_rx, BNO085_SHTP_HEADER_SIZE) != HAL_OK)
	{
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
		spi_state = BNO085_SPI_IDLE;
		return BNO085_STATUS_ERROR;
	}


	return BNO085_STATUS_OK;

}


static BNO085_Status_t BNO085_ParseHeader(const uint8_t *raw_header, BNO085_SHTP_Header_t *header)
{
	uint16_t raw_length;
	uint16_t length;


	if(raw_header == NULL ||  header == NULL)
	{
		return BNO085_STATUS_ERROR;
	}

	raw_length = ((uint16_t)raw_header[1] << 8 ) | ((uint16_t)raw_header[0]);

	if(raw_length == 0xFFFFU) // RAW LENGTH CAN NOT BE 0XFFFF-> IT MEANS ERROR
	{
		return BNO085_STATUS_INVALID_PACKET;
	}


	length = raw_length & 0x7FFFU;

	if(length < 4U)
	{
		return BNO085_STATUS_INVALID_PACKET;
	}

	header->continuation = (raw_length & (0x8000U)) ? 1U : 0U; // Check 15th bit

	header->length = length;

	header->channel = raw_header[2];

	header->sequence = raw_header[3];


	return BNO085_STATUS_OK;

}

static BNO085_Status_t BNO085_StartPacketRead(void)
{
	if(spi_state != BNO085_SPI_WAIT_PACKET)
	{
		return BNO085_STATUS_BUSY;
	}
	if(HAL_GPIO_ReadPin(SPI2_INT_GPIO_Port, SPI2_INT_Pin) != GPIO_PIN_RESET)
	{
		return BNO085_STATUS_BUSY;
	}

	if(BNO085_Header.length > BNO085_RX_BUFFER_SIZE)
	{
		return BNO085_STATUS_INVALID_PACKET;
	}

	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	spi_state = BNO085_SPI_RX_PACKET;

	if( (HAL_SPI_TransmitReceive_DMA(&hspi2, shtp_packet_tx, shtp_packet_rx, BNO085_Header.length) ) != HAL_OK)
	{
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
		spi_state = BNO085_SPI_WAIT_PACKET;
		return BNO085_STATUS_ERROR;
	}

	return BNO085_STATUS_OK;

}
/*
 **************************************** PUBLIC API'S *****************************************************************
 */
BNO085_Status_t BN085_Init(void)
{
	BNO085_Status_t status;

	spi_state = BNO085_SPI_IDLE;

	BNO085_Header.length = 0U;
	BNO085_Header.channel = 0U;
	BNO085_Header.sequence = 0U;
	BNO085_Header.continuation = 0U;

	bno085_int_flag = 0U;
	bno085_header_ready = 0U;

	status = BNO085_HardwareReset();
	if(status != BNO085_STATUS_OK)
	{
		return status;
	}

	return BNO085_STATUS_OK;
}


void BNO085_Process(void)
{
	BNO085_Status_t status;

	if(bno085_header_ready == 1U)
	{
		status = BNO085_ParseHeader(shtp_header_rx, &BNO085_Header);
		bno085_header_ready = 0U;
		if(status != BNO085_STATUS_OK)
		{
			spi_state = BNO085_SPI_ERROR;
			return;
		}
		spi_state = BNO085_SPI_WAIT_PACKET;
		return;
	}
	else if(bno085_packet_ready == 1U)
	{
		status = BNO085_ParseHeader(shtp_packet_rx, &BNO085_PacketHeader);
		if(status != BNO085_STATUS_OK)
		{
			spi_state = BNO085_SPI_ERROR;
			return;
		}
		__NOP();
		return;
	}


	if( (spi_state == BNO085_SPI_IDLE) && ( (bno085_int_flag == 1U) || (HAL_GPIO_ReadPin(SPI2_INT_GPIO_Port, SPI2_INT_Pin) == GPIO_PIN_RESET) ) )
	{
		status = BNO085_StartHeaderRead();
		if(status == BNO085_STATUS_OK)
		{
			bno085_int_flag = 0U;
		}
	}else if( spi_state == BNO085_SPI_WAIT_PACKET )
	{
		if(HAL_GPIO_ReadPin(SPI2_INT_GPIO_Port, SPI2_INT_Pin) == GPIO_PIN_RESET)
		{
			status = BNO085_StartPacketRead();
			if(status == BNO085_STATUS_OK)
			{
				bno085_int_flag = 0U;
			}
			return;
		}
	}else if(spi_state != BNO085_SPI_IDLE)
	{
		return;
	}


}



/*
 *********************************************** CALLBACK FUNCTIONS **********************************************
 */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance != SPI2)
	{
		return;
	}
	if(spi_state == BNO085_SPI_RX_HEADER)
	{
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

		spi_state = BNO085_SPI_IDLE;
		bno085_header_ready = 1U;
	}
	else if(spi_state == BNO085_SPI_RX_PACKET)
	{
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
		bno085_packet_ready = 1U;
		spi_state = BNO085_SPI_IDLE;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == SPI2_INT_Pin)
	{
		bno085_int_flag = 1U;
	}
}
