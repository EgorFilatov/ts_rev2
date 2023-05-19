#include "Ts.h"

Ts::Ts() {

}

Ts::Pin::Pin() {
	this->port = GPIOA;
	this->pin = 0;
}

void Ts::Pin::setVal(GPIO_TypeDef *port, uint8_t pin) {
	this->port = port;
	this->pin = pin;
}

GPIO_TypeDef* Ts::Pin::getPort() {
	return this->port;
}
uint8_t Ts::Pin::getPin() {
	return this->pin;
}

void Ts::setSw (GPIO_TypeDef* port0, uint8_t pin0, GPIO_TypeDef* port1, uint8_t pin1, GPIO_TypeDef* port2, uint8_t pin2, GPIO_TypeDef* port3, uint8_t pin3) {
	this->sw[0].setVal(port0, pin0);
	this->sw[1].setVal(port1, pin1);
	this->sw[2].setVal(port2, pin2);
	this->sw[3].setVal(port3, pin3);
}

void Ts::setOut (GPIO_TypeDef* port, uint8_t pin) {
	this->out.setVal(port, pin);
}

void Ts::setPinNum(uint8_t num) {
	this->pinNum = num;
}

void Ts::select() {
	switch (this->pinNum) {
	case 1:
		this->sw[0].getPort()->BSRR = (1 << this->sw[0].getPin());
		break;
	case 2:
		this->sw[1].getPort()->BSRR = (1 << this->sw[1].getPin());
		break;
	case 3:
		this->sw[0].getPort()->BSRR = (1 << this->sw[0].getPin());
		this->sw[1].getPort()->BSRR = (1 << this->sw[1].getPin());
		break;
	case 4:
		this->sw[2].getPort()->BSRR = (1 << this->sw[2].getPin());
		break;
	case 5:
		this->sw[0].getPort()->BSRR = (1 << this->sw[0].getPin());
		this->sw[2].getPort()->BSRR = (1 << this->sw[2].getPin());
		break;
	case 6:
		this->sw[1].getPort()->BSRR = (1 << this->sw[1].getPin());
		this->sw[2].getPort()->BSRR = (1 << this->sw[2].getPin());
		break;
	case 7:
		this->sw[0].getPort()->BSRR = (1 << this->sw[0].getPin());
		this->sw[1].getPort()->BSRR = (1 << this->sw[1].getPin());
		this->sw[2].getPort()->BSRR = (1 << this->sw[2].getPin());
		break;
	case 8:
		this->sw[3].getPort()->BSRR = (1 << this->sw[3].getPin());
		break;
	case 9:
		this->sw[0].getPort()->BSRR = (1 << this->sw[0].getPin());
		this->sw[3].getPort()->BSRR = (1 << this->sw[3].getPin());
		break;
	case 10:
		this->sw[1].getPort()->BSRR = (1 << this->sw[1].getPin());
		this->sw[3].getPort()->BSRR = (1 << this->sw[3].getPin());
		break;
	case 11:
		this->sw[0].getPort()->BSRR = (1 << this->sw[0].getPin());
		this->sw[1].getPort()->BSRR = (1 << this->sw[1].getPin());
		this->sw[3].getPort()->BSRR = (1 << this->sw[3].getPin());
		break;
	case 12:
		this->sw[2].getPort()->BSRR = (1 << this->sw[2].getPin());
		this->sw[3].getPort()->BSRR = (1 << this->sw[3].getPin());
		break;
	case 13:
		this->sw[0].getPort()->BSRR = (1 << this->sw[0].getPin());
		this->sw[2].getPort()->BSRR = (1 << this->sw[2].getPin());
		this->sw[3].getPort()->BSRR = (1 << this->sw[3].getPin());
		break;
	case 14:
		this->sw[1].getPort()->BSRR = (1 << this->sw[1].getPin());
		this->sw[2].getPort()->BSRR = (1 << this->sw[2].getPin());
		this->sw[3].getPort()->BSRR = (1 << this->sw[3].getPin());
		break;
	case 15:
		this->sw[0].getPort()->BSRR = (1 << this->sw[0].getPin());
		this->sw[1].getPort()->BSRR = (1 << this->sw[1].getPin());
		this->sw[2].getPort()->BSRR = (1 << this->sw[2].getPin());
		this->sw[3].getPort()->BSRR = (1 << this->sw[3].getPin());
		break;
	}
}

void Ts::unSelect() {
	switch (this->pinNum) {
	case 1:
		this->sw[0].getPort()->BRR = (1 << this->sw[0].getPin());
		break;
	case 2:
		this->sw[1].getPort()->BRR = (1 << this->sw[1].getPin());
		break;
	case 3:
		this->sw[0].getPort()->BRR = (1 << this->sw[0].getPin());
		this->sw[1].getPort()->BRR = (1 << this->sw[1].getPin());
		break;
	case 4:
		this->sw[2].getPort()->BRR = (1 << this->sw[2].getPin());
		break;
	case 5:
		this->sw[0].getPort()->BRR = (1 << this->sw[0].getPin());
		this->sw[2].getPort()->BRR = (1 << this->sw[2].getPin());
		break;
	case 6:
		this->sw[1].getPort()->BRR = (1 << this->sw[1].getPin());
		this->sw[2].getPort()->BRR = (1 << this->sw[2].getPin());
		break;
	case 7:
		this->sw[0].getPort()->BRR = (1 << this->sw[0].getPin());
		this->sw[1].getPort()->BRR = (1 << this->sw[1].getPin());
		this->sw[2].getPort()->BRR = (1 << this->sw[2].getPin());
		break;
	case 8:
		this->sw[3].getPort()->BSRR = (1 << this->sw[3].getPin());
		break;
	case 9:
		this->sw[0].getPort()->BRR = (1 << this->sw[0].getPin());
		this->sw[3].getPort()->BRR = (1 << this->sw[3].getPin());
		break;
	case 10:
		this->sw[1].getPort()->BRR = (1 << this->sw[1].getPin());
		this->sw[3].getPort()->BRR = (1 << this->sw[3].getPin());
		break;
	case 11:
		this->sw[0].getPort()->BRR = (1 << this->sw[0].getPin());
		this->sw[1].getPort()->BRR = (1 << this->sw[1].getPin());
		this->sw[3].getPort()->BRR = (1 << this->sw[3].getPin());
		break;
	case 12:
		this->sw[2].getPort()->BRR = (1 << this->sw[2].getPin());
		this->sw[3].getPort()->BRR = (1 << this->sw[3].getPin());
		break;
	case 13:
		this->sw[0].getPort()->BRR = (1 << this->sw[0].getPin());
		this->sw[2].getPort()->BRR = (1 << this->sw[2].getPin());
		this->sw[3].getPort()->BRR = (1 << this->sw[3].getPin());
		break;
	case 14:
		this->sw[1].getPort()->BRR = (1 << this->sw[1].getPin());
		this->sw[2].getPort()->BRR = (1 << this->sw[2].getPin());
		this->sw[3].getPort()->BRR = (1 << this->sw[3].getPin());
		break;
	case 15:
		this->sw[0].getPort()->BRR = (1 << this->sw[0].getPin());
		this->sw[1].getPort()->BRR = (1 << this->sw[1].getPin());
		this->sw[2].getPort()->BRR = (1 << this->sw[2].getPin());
		this->sw[3].getPort()->BRR = (1 << this->sw[3].getPin());
		break;
	}
}

uint8_t Ts::getPinNum() {
	return this->pinNum;
}

void Ts::read(uint8_t *buff, uint8_t tsNum) {
	this->select();

	if (this->out.getPort()->IDR & (1 << this->out.getPin())) {
		*buff &= ~(1 << tsNum);
	} else {
		*buff |= (1 << tsNum);
	}

	this->unSelect();
}
