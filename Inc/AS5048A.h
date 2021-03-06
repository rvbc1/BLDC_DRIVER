/*
 * AS5048A.h
 *
 *  Created on: Apr 3, 2019
 *      Author: rvbc-
 */

#ifndef AS5048A_H_
#define AS5048A_H_

#include "stm32f3xx_hal.h"
#include "vector"

#define READ_DATA_BIT 0b0100000000000000

#define DATA_BITS 0b0011111111111111

#define PARITY_BIT 0b1000000000000000
#define PARITY_BIT_POS 15

#define TRANSMISSION_ERROR_BIT 0b0100000000000000
#define TRANSMISSION_ERROR_BIT_POS 14


#define AMOUNT_OF_UINT16_BYTES 2

#define MAX_ENCODER_VALUE 16384
#define MAX_DISPERSION_VALUE 200


#define SENDING_FLAG_BIT 7
#define ERROR_TRANSSMISION_FLAG_BIT 6
#define ERROR_PARITY_FLAG_BIT 5

#define MAX_VECTOR_SIZE 100


class AS5048A {
private:
	uint8_t flags;

	uint16_t dataBufferRX;
	uint16_t dataBufferTX;

	uint16_t rotations;
	uint16_t prev_read_angle;
	uint16_t angle;

	std::vector<uint16_t> angle_vector;

	GPIO_TypeDef *nss_port;
	uint16_t nss_pin;
	SPI_HandleTypeDef *SPI;

	int8_t checkRotation();
	uint8_t isOnRange(int32_t lower_limit, int32_t higher_limit);


	uint8_t spiCalcEvenParity(uint16_t value);
	uint16_t reverseBytes16(uint16_t &nValue);

	void setFlag(uint8_t flag);
	void resetFlag(uint8_t flag);
	uint8_t getFlag(uint8_t flag);

	void setErrorParityFlag();
	void setErrorTransmissionFlag();

	void addAngleVector(uint16_t angle);
public:
	AS5048A(GPIO_TypeDef *nss_port, uint16_t nss_pin, SPI_HandleTypeDef *SPI);
	virtual ~AS5048A();

	uint32_t getAngle();
	uint16_t readRawAngle();
	uint16_t readData(uint16_t registerAddress);


	uint8_t getErrorReadFlag();
	uint8_t getErrorParityFlag();
	uint16_t *getBufferRX();

	std::vector<uint16_t> getAngleVector();

	void clearAngleVector();

	void startSendData();
	void sendNextData();
	void stopSendData();
	void resetAllFlags();
};

#endif /* AS5048A_H_ */
