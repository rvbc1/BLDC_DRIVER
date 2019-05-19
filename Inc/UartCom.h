/*
 * UartCom.h
 *
 *  Created on: Apr 14, 2019
 *      Author: rvbc-
 */

#ifndef UARTCOM_H_
#define UARTCOM_H_

#include "stm32f3xx_hal.h"

#define DATA_FRAME_TX_SIZE 6
#define DATA_FRAME_RX_SIZE 7

#define START_CODE 0x40
#define END_CODE 0x80

struct dataFrameRX{
	uint8_t start_transsmision_byte;
	uint8_t code;
	uint16_t angle;
	uint16_t torque;
	uint8_t end_transsmision_byte;
} __attribute__ ((__packed__));

struct dataFrameTX{
	uint8_t start_transsmision_byte;
	uint16_t angle;
	int16_t real_angle;
	uint8_t end_transsmision_byte;
} __attribute__ ((__packed__));

class UartCom {
private:
	union frameRX_u{
		uint8_t *bytes;
		struct dataFrameRX *data;
	}frameRX;

	union frameTX_u{
		uint8_t *bytes;
		struct dataFrameTX *data;
	}frameTX;

	uint8_t send_next_data;
	uint8_t recieve_next_data;

	uint16_t *data_handler;

	UART_HandleTypeDef * uart_handler;

	void init(UART_HandleTypeDef *uart_handler, uint16_t *data_handler);
	void initFrameRX();
	void initFrameTX();
	void updateFrameTX();
	void updateData();

	void goodDataLoad();
	void badDataLoad();
	uint8_t isRecieveDataCorrect();
public:
	UartCom(UART_HandleTypeDef *uart_handler, uint16_t *data_handler);

	void sendData();
	void recieveNextData();
	void stopSendData();
	void startSendData();
	void stopRecieveData();
	void startRecieveData();

	uint16_t getData();
	uint16_t getAngle();

	virtual ~UartCom();

	int16_t real_angle;
};

#endif /* UARTCOM_H_ */
