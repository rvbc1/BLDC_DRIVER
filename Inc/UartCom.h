/*
 * UartCom.h
 *
 *  Created on: Apr 14, 2019
 *      Author: rvbc-
 */

#ifndef UARTCOM_H_
#define UARTCOM_H_

#define RX_BUFFER_SIZE 5
#define TX_BUFFER_SIZE 4

#include "stm32f3xx_hal.h"

class UartCom {
private:
	struct dataTX_s{
		uint8_t start_transsmision_byte;
		uint16_t angle;
		uint8_t end_transsmision_byte;
	} __attribute__ ((__packed__));

	union frameTX_u{
		uint8_t *bytesTX;
		struct dataTX_s *values;
	}frameTX;

	//uint8_t *dataBufferTX;
	uint8_t *bytesRX;
	uint16_t *dataToSend;

	UART_HandleTypeDef * uart;

	void init(UART_HandleTypeDef *uart, uint16_t *dataToSend);
	void prepareData();
	void updateData();
public:
	UartCom(UART_HandleTypeDef *uart, uint16_t *dataToSend);

	void sendData();

	virtual ~UartCom();
};

#endif /* UARTCOM_H_ */
