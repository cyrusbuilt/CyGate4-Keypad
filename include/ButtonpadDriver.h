#ifndef _BUTTONPADDRIVER_H
#define _BUTTONPADDRIVER_H

#include <Arduino.h>
#include "Adafruit_Keypad.h"

enum class ButtonpadButtonState : uint8_t
{
	PRESSED = KEY_JUST_PRESSED,
	RELEASED = KEY_JUST_RELEASED
};

struct KeypadEvent
{
	char key;
	ButtonpadButtonState state;
};

class ButtonpadDriverClass
{
public:
	ButtonpadDriverClass();
	void begin(void (*eventHandler)(KeypadEvent *sender));
	void loop();
	void clear();

private:
	Adafruit_Keypad *_keypad;
	KeypadEvent *_sender;
	void (*onStateChange)(KeypadEvent *sender);
};

extern ButtonpadDriverClass ButtonpadDriver;

#endif