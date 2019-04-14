/*
 * LedDriver.h
 *
 *  Created on: Apr 13, 2019
 *      Author: rvbc-
 */

#ifndef LEDDRIVER_H_
#define LEDDRIVER_H_

#include "stm32f3xx_hal.h"

class Led_Driver {
private:
	uint32_t pin;
	uint16_t start_pin;
	uint16_t end_pin;
	GPIO_TypeDef* port;

	void init(GPIO_TypeDef* port, uint16_t start_pin, uint16_t end_pin);
public:
	Led_Driver(GPIO_TypeDef* port, uint16_t start_pin, uint16_t end_pin);

	void setAll(GPIO_PinState state);
	void allOn();
	void allOff();
	void allToggle();

	void blink(uint32_t duration_ms);
	void showIntro();

	void turnRightSET(uint16_t start_led_pin, uint32_t duration_ms);
	void turnLeftSET(uint16_t start_led_pin, uint32_t duration_ms);

	uint16_t circleLeft(uint16_t start_led_pin, uint32_t duration_ms, GPIO_PinState state);
	uint16_t circleRight(uint16_t start_led_pin, uint32_t duration_ms, GPIO_PinState state);

	void circleRightToggle(uint16_t start_led_pin, uint32_t duration_ms);

	void circleRightBack(uint16_t start_led_pin, uint32_t duration_ms);

	void circleRightFront(uint16_t start_led_pin, uint32_t duration_ms);

	void showSET(uint16_t led_pin);
	void showSET(uint8_t pos);
	void showRESET(uint16_t led_pin);
	void showRESET(uint8_t pos);

	virtual ~Led_Driver();

};

#endif /* LEDDRIVER_H_ */
