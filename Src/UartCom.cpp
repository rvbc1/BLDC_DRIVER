/*
 * UartCom.cpp
 *
 *  Created on: Apr 14, 2019
 *      Author: rvbc-
 */

#include "UartCom.h"

void UartCom::init(UART_HandleTypeDef *uart_handler, uint16_t *data_handler){
	this->uart_handler = uart_handler;
	this->data_handler = data_handler;
	this->frameRX.bytes = new uint8_t[DATA_FRAME_RX_SIZE];
	this->frameTX.bytes = new uint8_t[DATA_FRAME_TX_SIZE];

	initFrameRX();
	initFrameTX();

	startRecieveData();
	startSendData();
}

UartCom::UartCom(UART_HandleTypeDef *uart_handler, uint16_t *data_handler) {
	init(uart_handler, data_handler);
}

void UartCom::initFrameRX(){
	frameRX.data->start_transsmision_byte = START_CODE;
	frameRX.data->end_transsmision_byte = END_CODE;
}

void UartCom::initFrameTX(){
	frameTX.data->start_transsmision_byte = START_CODE;
	frameTX.data->end_transsmision_byte = END_CODE;
}

void UartCom::updateFrameTX(){
	frameTX.data->angle = *data_handler / 22.75;
	frameTX.data->real_angle = *data_handler;
}

void UartCom::updateData(){
	frameTX.data->angle = *data_handler;
}

void UartCom::sendData(){
	updateFrameTX();
	HAL_UART_Transmit_DMA(uart_handler, (uint8_t*)frameTX.bytes, DATA_FRAME_TX_SIZE);
}

void UartCom::recieveNextData(){
	if(isRecieveDataCorrect()){
		goodDataLoad();
		if(recieve_next_data) HAL_UART_Receive_DMA(uart_handler, frameRX.bytes, DATA_FRAME_RX_SIZE);
	} else {
		badDataLoad();
		for(int i = 1; i < DATA_FRAME_RX_SIZE; i++){
			if(frameRX.bytes[i] == START_CODE){
				for(int j = 0; j < DATA_FRAME_RX_SIZE - i; j++){
					frameRX.bytes[j] = frameRX.bytes[j + i];
				}
				if(recieve_next_data) HAL_UART_Receive_DMA(uart_handler, frameRX.bytes + DATA_FRAME_RX_SIZE - i, i);
				goto loop_end;
			}
		}
		if(recieve_next_data) HAL_UART_Receive_DMA(uart_handler, frameRX.bytes, DATA_FRAME_RX_SIZE);
	}
	loop_end:;
}

void UartCom::goodDataLoad(){

}
void UartCom::badDataLoad(){
}

uint8_t UartCom::isRecieveDataCorrect(){
	if((frameRX.data->start_transsmision_byte == START_CODE) && (frameRX.data->end_transsmision_byte == END_CODE)){
		return true;
	}
	return false;
}

void UartCom::stopSendData(){
	send_next_data = false;
}
void UartCom::startSendData(){
	send_next_data = true;
	sendData();
}
void UartCom::stopRecieveData(){
	recieve_next_data = false;
}
void UartCom::startRecieveData(){
	recieve_next_data = true;
	HAL_UART_Receive_DMA(uart_handler, frameRX.bytes, DATA_FRAME_RX_SIZE);
}

uint16_t UartCom::getData(){
	return frameRX.data->torque;
}

uint16_t UartCom::getAngle(){
	return frameRX.data->angle;
}

UartCom::~UartCom() {
	// TODO Auto-generated destructor stub
}

