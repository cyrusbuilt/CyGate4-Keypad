#include "ButtonpadDriver.h"

// We are using the Adafruit 4x4 Matrix keypad (PID 3844)
#define KEYPAD_PID3844

// Row and column pins
#define R1    9
#define R2    10
#define R3    11
#define R4    12
#define C1    5
#define C2    6
#define C3    7
#define C4    8

#include "buttonpad_config.h"

ButtonpadDriverClass::ButtonpadDriverClass() {
	this->_keypad = new Adafruit_Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
	this->_sender = new KeypadEvent { 0, ButtonpadButtonState::RELEASED };
}

void ButtonpadDriverClass::begin(void (*eventHandler)(KeypadEvent *sender)) {
	this->onStateChange = eventHandler;
	this->_keypad->begin();
}

void ButtonpadDriverClass::loop() {
	this->_keypad->tick();
	while (this->_keypad->available()) {
		keypadEvent e = this->_keypad->read();
		this->_sender->key = (char)e.bit.KEY;
		this->_sender->state = (ButtonpadButtonState)e.bit.EVENT;
		if (this->onStateChange != NULL) {
			this->onStateChange(this->_sender);
			delay(50);
		}
	}
}

void ButtonpadDriverClass::clear() {
	this->_keypad->clear();
}

ButtonpadDriverClass ButtonpadDriver;