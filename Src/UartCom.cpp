/*
 * UartCom.cpp
 *
 *  Created on: Apr 14, 2019
 *      Author: rvbc-
 */

#include "UartCom.h"

void UartCom::init(UART_HandleTypeDef *uart, uint16_t *dataToSend){
	this->uart = uart;
	this->dataToSend = dataToSend;
	this->frameTX.bytesTX = new uint8_t[TX_BUFFER_SIZE];
	prepareData();
}

UartCom::UartCom(UART_HandleTypeDef *uart, uint16_t *dataToSend) {
	init(uart, dataToSend);
}

void UartCom::prepareData(){
	frameTX.values->start_transsmision_byte = 0x40;
	frameTX.values->angle = *dataToSend;
	frameTX.values->end_transsmision_byte = 0x80;
}

void UartCom::updateData(){
	frameTX.values->angle = *dataToSend;
}

void UartCom::sendData(){
	updateData();
	HAL_UART_Transmit_IT(uart, (uint8_t*)frameTX.bytesTX, TX_BUFFER_SIZE);
}

UartCom::~UartCom() {
	// TODO Auto-generated destructor stub
}

