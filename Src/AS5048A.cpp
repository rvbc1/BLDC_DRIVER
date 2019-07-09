/*
 * AS5048A.cpp
 *
 *  Created on: Apr 3, 2019
 *      Author: rvbc-
 */

#include "AS5048A.h"

const uint16_t AS5048A_CLEAR_ERROR_FLAG              = 0x0001;
const uint16_t AS5048A_PROGRAMMING_CONTROL           = 0x0003;
const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_HIGH    = 0x0016;
const uint16_t AS5048A_OTP_REGISTER_ZERO_POS_LOW     = 0x0017;
const uint16_t AS5048A_DIAG_AGC                      = 0x3FFD;
const uint16_t AS5048A_MAGNITUDE                     = 0x3FFE;
const uint16_t AS5048A_ANGLE                         = 0x3FFF;



AS5048A::AS5048A(GPIO_TypeDef *nss_port, uint16_t nss_pin, SPI_HandleTypeDef *SPI) {
	this->nss_port = nss_port;
	this->nss_pin = nss_pin;
	this->SPI = SPI;
	this->angle = 0;
	this->prev_read_angle = 0;
	resetAllFlags();
}

AS5048A::~AS5048A() {
	// TODO Auto-generated destructor stub
}

uint32_t AS5048A::getAngle(){
	return angle + prev_read_angle;
}

uint16_t AS5048A::readRawAngle(){
	uint16_t read_angle = readData(AS5048A_ANGLE);

	checkRotation();

	prev_read_angle = read_angle;
	return read_angle;
}

uint16_t AS5048A::readData(uint16_t registerAddress){

	uint16_t answer;

	uint16_t command = READ_DATA_BIT;
	command = command | registerAddress;

	command |= ((uint16_t)spiCalcEvenParity(command) << PARITY_BIT_POS);

	reverseBytes16(command);

	HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(SPI, (uint8_t *)(&command), (uint8_t *)(&answer), AMOUNT_OF_UINT16_BYTES, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_SET);

	reverseBytes16(answer);

	uint16_t readData = answer & DATA_BITS;
	uint8_t transmissionErrorBit = (answer >> TRANSMISSION_ERROR_BIT_POS) & 0x01;
	uint8_t parityBit = (answer >> PARITY_BIT_POS) & 0x01;

	if(transmissionErrorBit != 0x00) setErrorTransmissionFlag();

	if(parityBit != spiCalcEvenParity(answer)){
		setErrorParityFlag();
		return 0;
	} else return readData;

}

void AS5048A::startSendData(){
	setFlag(SENDING_FLAG_BIT);
	uint16_t registerAddress = AS5048A_ANGLE;
	uint16_t command = READ_DATA_BIT;
	command = command | registerAddress;

	command |= ((uint16_t)spiCalcEvenParity(command) << PARITY_BIT_POS);

	reverseBytes16(command);

	dataBufferTX = command;
	HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(SPI, (uint8_t *)(&dataBufferTX),
			(uint8_t *)(&dataBufferRX), AMOUNT_OF_UINT16_BYTES);

}

void AS5048A::sendNextData(){
	HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_SET);

	reverseBytes16(dataBufferRX);

	uint16_t readData = dataBufferRX & 0b0011111111111111;
	uint8_t transmissionErrorBit = (dataBufferRX >> TRANSMISSION_ERROR_BIT_POS) & 0x01;
	uint8_t parityBit = (dataBufferRX >> PARITY_BIT_POS) & 0x01;

	if(transmissionErrorBit != 0x00) setErrorTransmissionFlag();

	if(parityBit != spiCalcEvenParity(dataBufferRX)){
		setErrorParityFlag();
	}

	angle = readData;
	addAngleVector(angle);

	HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_RESET);
	if(getFlag(SENDING_FLAG_BIT))
		HAL_SPI_TransmitReceive_DMA(SPI, (uint8_t *)(&dataBufferTX),
				(uint8_t *)(&dataBufferRX), AMOUNT_OF_UINT16_BYTES);

}

void AS5048A::stopSendData(){
	resetFlag(SENDING_FLAG_BIT);
}

int8_t AS5048A::checkRotation(){
	int read_angle;
	if((read_angle < MAX_DISPERSION_VALUE) &&
			(prev_read_angle > MAX_ENCODER_VALUE - MAX_DISPERSION_VALUE))
		rotations++;
	else if ((prev_read_angle < MAX_DISPERSION_VALUE) &&
			(read_angle > MAX_ENCODER_VALUE - MAX_DISPERSION_VALUE))
		rotations--;
}

uint8_t AS5048A::isOnRange(int32_t lower_limit, int32_t higher_limit){

}

uint8_t AS5048A::spiCalcEvenParity(uint16_t value){
	uint8_t cnt = 0;
	uint8_t i;

	for (i = 0; i < 15; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
}

uint16_t AS5048A::reverseBytes16(uint16_t &nValue){
	return nValue = (nValue >> 8) | (nValue << 8);
}

void AS5048A::setFlag(uint8_t flag){
	flags |= 1UL << flag;
}

void AS5048A::resetFlag(uint8_t flag){
	flags &= ~(1UL << flag);
}

uint8_t AS5048A::getFlag(uint8_t flag){
	return (flags >> flag) & 1U;
}

void AS5048A::setErrorParityFlag(){
	setFlag(ERROR_PARITY_FLAG_BIT);
}

void AS5048A::setErrorTransmissionFlag(){
	setFlag(ERROR_TRANSSMISION_FLAG_BIT);
}

void AS5048A::resetAllFlags(){
	flags = 0;
}

uint16_t * AS5048A::getBufferRX(){
	return &angle;
}

void AS5048A::addAngleVector(uint16_t angle){
	if(angle_vector.size() < MAX_VECTOR_SIZE)
		angle_vector.push_back(angle);
}

std::vector<uint16_t> AS5048A::getAngleVector(){
	return angle_vector;
}

void AS5048A::clearAngleVector(){
	angle_vector.clear();
}


