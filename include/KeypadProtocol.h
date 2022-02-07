#ifndef _KEYPADPROTOCOL_H
#define _KEYPADPROTOCOL_H

#include <Arduino.h>

// Commands
#define KEYPAD_INIT 0xDA
#define KEYPAD_DETECT 0xDB
#define KEYPAD_BAD_CODE 0xDC
#define KEYPAD_DETECT_ACK 0xDD
#define KEYPAD_GET_CMD_DATA 0xDE
#define KEYPAD_GET_AVAILABLE 0xDF

// Sizes
#define KEYPAD_DATA_BUFFER_SIZE 4

struct KeypadData {
	uint8_t size;
	uint8_t command;
	byte data[KEYPAD_DATA_BUFFER_SIZE];
};

enum class KeypadCommands : uint8_t {
	DISARM = 0x01,
	ARM_STAY = 0x02,
	ARM_AWAY = 0x03,
	UNLOCK = 0x04,
	LOCK = 0x05,
	CODE_CHECK = 0x06
};

#endif