#ifndef SRC_TS_H_
#define SRC_TS_H_

#include "main.h"

class Ts {
	class Pin {
	private:
		GPIO_TypeDef* port;
		uint8_t pin;
	public:
		Pin();
		void setVal(GPIO_TypeDef *port, uint8_t pin);
		GPIO_TypeDef* getPort();
		uint8_t getPin();
	};

private:
	Pin sw[4];
	Pin out;
	uint8_t pinNum;

public:
	Ts();
	void setSw(GPIO_TypeDef* port0, uint8_t pin0, GPIO_TypeDef* port1, uint8_t pin1, GPIO_TypeDef* port2, uint8_t pin2, GPIO_TypeDef* port3, uint8_t pin3);
	void setOut(GPIO_TypeDef* port, uint8_t pin);
	void setPinNum(uint8_t num);
	void select();
	void unSelect();
	void read(uint8_t *buff, uint8_t tsNum);
	uint8_t getPinNum();
};

#endif
