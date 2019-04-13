/*
 * LedDriver.cpp
 *
 *  Created on: Apr 13, 2019
 *      Author: rvbc-
 */

#include "LedDriver.h"

void Led_Driver::init(GPIO_TypeDef* port, uint16_t start_pin, uint16_t end_pin){
	this->port = port;
	this->start_pin = start_pin;
	this->end_pin = end_pin;

	this->pin = this->start_pin;
}

Led_Driver::Led_Driver(GPIO_TypeDef* port, uint16_t start_pin, uint16_t end_pin) {
	init(port, start_pin, end_pin);
}

void Led_Driver::setAll(GPIO_PinState state){
	for(pin = start_pin; pin <= end_pin; pin <<= 1){
		HAL_GPIO_WritePin(port, pin, state);
	}
}

void Led_Driver::allOn(){
	setAll(GPIO_PIN_SET);
}

void Led_Driver::allOff(){
	setAll(GPIO_PIN_RESET);
}

void Led_Driver::allToggle(){
	for(pin = start_pin; pin <= end_pin; pin <<= 1){
		HAL_GPIO_TogglePin(port, pin);
	}
}

void Led_Driver::blink(uint32_t duration_ms){
	allOn();

	HAL_Delay(duration_ms);

	allOff();
}

void Led_Driver::turnLeftSET(uint16_t start_led_pin, uint32_t duration_ms){
	uint32_t old_pin;
	for(pin = start_led_pin; pin >= start_pin; old_pin = pin, pin >>= 1){
		if(old_pin <= end_pin) HAL_GPIO_WritePin(port, old_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
		HAL_Delay(duration_ms);
	}

	for(pin = end_pin; pin >= start_led_pin; old_pin = pin,pin >>= 1){
		if(old_pin <= end_pin) HAL_GPIO_WritePin(port, old_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
		HAL_Delay(duration_ms);
	}
	HAL_GPIO_WritePin(port, start_led_pin, GPIO_PIN_RESET);
}

void Led_Driver::turnRightSET(uint16_t start_led_pin, uint32_t duration_ms){
	uint32_t old_pin;
	for(pin = start_led_pin; pin <= end_pin; old_pin = pin, pin <<= 1){
		if(old_pin >= start_pin) HAL_GPIO_WritePin(port, old_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
		HAL_Delay(duration_ms);
	}

	for(pin = start_pin; pin <= start_led_pin; old_pin = pin,pin <<= 1){
		if(old_pin >= start_pin) HAL_GPIO_WritePin(port, old_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
		HAL_Delay(duration_ms);
	}
	HAL_GPIO_WritePin(port, start_led_pin, GPIO_PIN_RESET);
}

uint16_t Led_Driver::circleLeft(uint16_t start_led_pin, uint32_t duration_ms, GPIO_PinState state){
	for(pin = start_led_pin; pin >= start_pin; pin >>= 1){
		HAL_GPIO_WritePin(port, pin, state);
		HAL_Delay(duration_ms);
	}

	for(pin = end_pin; pin >= start_led_pin; pin >>= 1){
		HAL_GPIO_WritePin(port, pin, state);
		HAL_Delay(duration_ms);
	}
	return pin;
}

uint16_t Led_Driver::circleRight(uint16_t start_led_pin, uint32_t duration_ms, GPIO_PinState state){
	for(pin = start_led_pin; pin <= end_pin; pin <<= 1){
		HAL_GPIO_WritePin(port, pin, state);
		HAL_Delay(duration_ms);
	}

	for(pin = start_pin; pin <= start_led_pin; pin <<= 1){
		HAL_GPIO_WritePin(port, pin, state);
		HAL_Delay(duration_ms);
	}
	return pin;
}

void Led_Driver::circleRightToggle(uint16_t start_led_pin, uint32_t duration_ms){
	for(pin = start_led_pin; pin <= end_pin; pin <<= 1){
		HAL_GPIO_TogglePin(port, pin);
		HAL_Delay(duration_ms);
	}

	for(pin = start_pin; pin <= start_led_pin; pin <<= 1){
		HAL_GPIO_TogglePin(port, pin);
		HAL_Delay(duration_ms);
	}
}

void Led_Driver::circleRightBack(uint16_t start_led_pin, uint32_t duration_ms){
	circleRight(start_led_pin, duration_ms, GPIO_PIN_SET);

	if((start_led_pin >> 1) < end_pin) circleLeft(start_led_pin >> 1, duration_ms, GPIO_PIN_RESET);
	else circleLeft(end_pin, duration_ms, GPIO_PIN_RESET);
}

void Led_Driver::circleRightFront(uint16_t start_led_pin, uint32_t duration_ms){
	circleRight(start_led_pin, duration_ms, GPIO_PIN_SET);

	circleRight(start_led_pin, duration_ms, GPIO_PIN_RESET);
}

void Led_Driver::showSET(uint16_t led_pin){
	allOff();
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

void Led_Driver::showSET(uint8_t pos){
	allOff();
	pos %= 8;
	HAL_GPIO_WritePin(port, start_pin << pos, GPIO_PIN_SET);
}

void Led_Driver::showRESET(uint16_t led_pin){
	allOn();
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void Led_Driver::showRESET(uint8_t pos){
	allOn();
	pos %= 8;
	HAL_GPIO_WritePin(port, start_pin << pos, GPIO_PIN_RESET);
}

Led_Driver::~Led_Driver() {
	// TODO Auto-generated destructor stub
}

