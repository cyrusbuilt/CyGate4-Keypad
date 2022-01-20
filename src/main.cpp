#include <Arduino.h>
#include <Wire.h>
#include "CrystalFontz632.h"
#include "ButtonpadDriver.h"
#include "KeypadProtocol.h"

#define FIRMWARE_VERSION "1.0"

#define PIN_DISPLAY_TX 13
#define PIN_ADDRESS_A0 A0
#define PIN_ADDRESS_A1 A1
#define PIN_ADDRESS_A2 A2

#define DEBUG_BAUD_RATE 9600
#define ADDRESS_BASE 0x10

#define ARMING_COUNT_DOWN_START 30

enum class Screen : uint8_t
{
	HOME = 0,
	MENU = 1,
	ARMING = 2,
	ARMED = 3,
	UNLOCK = 4,
	DISARMING = 5
};

enum class ArmState : uint8_t {
	DISARMED = 0,
	ARMED = 1
};

CrystalFontz632 display(PIN_DISPLAY_TX);
String armCode;
String unlockCode;
KeypadData data;
uint8_t count = ARMING_COUNT_DOWN_START;
ArmState armState = ArmState::DISARMED;
unsigned long previousMillis = 0;
volatile byte command = 0xFF;
volatile bool processCommand = false;
volatile Screen currentScreen = Screen::HOME;
const short addrPins[3] = {
	PIN_ADDRESS_A0,
	PIN_ADDRESS_A1,
	PIN_ADDRESS_A2
};

void clearKeypadData() {
	data.command = 0;
	data.size = 0;
	for (uint8_t i = 0; i < KEYPAD_DATA_BUFFER_SIZE; i++) {
		data.data[i] = 0;
	}
}

void sendKeyCode() {
	uint8_t *buffer;
	buffer[0] = KEYPAD_GET_CMD_DATA;
	buffer[1] = data.command;
	buffer[2] = data.size;
	for (size_t i = 0; i < data.size; i++) {
		buffer[i + 3] = data.data[i];
	}

	Wire.write(buffer, data.size + 3);
	clearKeypadData();
	delete[] buffer;
}

void commBusReceiveHandler(int byteCount) {
	// NOTE: We *should* only be recieving single-byte commands.
	command = Wire.read();
	processCommand = true;
}

void commBusRequestHandler() {
	if (!processCommand) {
		return;
	}

	switch (command)
	{
	case KEYPAD_INIT:
		// TODO: What init/re-init actions should we take here?
		// For now, just ACK the init request, but we should probably return some
		// type of status code.
		Wire.write(KEYPAD_INIT);
		break;
	case KEYPAD_DETECT:
		Wire.write(KEYPAD_DETECT_ACK);
		break;
	case KEYPAD_GET_CMD_DATA:
		sendKeyCode();
		break;
	default:
		break;
	}

	processCommand = false;
}

void homeScreen() {
	Serial.println(F("INFO: Home screen"));
	display.clearDisplay();
	display.print(F("Ready"));
	display.crlf();
	display.print(F("D = Menu"));
	display.hideCursor();
	currentScreen = Screen::HOME;
}

void menuScreen() {
	Serial.println(F("INFO: Menu screen"));
	display.clearDisplay();
	display.print(F("* = Back A = Arm"));
	display.carriageReturn();
	display.print(F("C = UNLOCK"));
	display.hideCursor();
	currentScreen = Screen::MENU;
}

void armingScreen() {
	Serial.println(F("INFO: Arm screen"));
	display.clearDisplay();
	display.print(F("ARM Code: "));
	display.crlf();
	display.showBlockCursor();
	armCode = "";
	currentScreen = Screen::ARMING;
}

void unlockScreen() {
	Serial.println(F("INFO: Unlock screen"));
	display.clearDisplay();
	display.print(F("Access code - #:"));
	display.carriageReturn();
	display.showBlockCursor();
	unlockCode = "";
	currentScreen = Screen::UNLOCK;
}

void armedScreen() {
	Serial.println(F("WARN: *** ARMED ***"));
	display.clearDisplay();
	display.hideCursor();
	display.print(F("*** ARMED ***"));
	display.setCursorPos(0, 1);
	display.print(F("EXIT IN "));
	display.print(count);
	currentScreen = Screen::ARMED;
	count = ARMING_COUNT_DOWN_START;
}

void disarmingScreen() {
	Serial.println(F("INFO: Disarming screen"));
	display.clearDisplay();
	display.print(F("Disarm code - #:"));
	display.carriageReturn();
	display.showBlockCursor();
	currentScreen = Screen::DISARMING;
}

bool isArmCodeValid() {
	// TODO Send the arm code to host via I2C for validation
	return true;
}

void maskAccessCode(char key) {
	if (key != 'A' && key != 'B' && key != 'C' && key != 'D') {
		armCode += key;
		display.print(F("*"));
	}
}

void keypadHandler(KeypadEvent *event) {
	if (event->state == ButtonpadButtonState::RELEASED) {
		return;
	}

	switch (currentScreen) {
		case Screen::HOME:
			if (event->key == 'D') {
				menuScreen();
			}
			break;
		case Screen::MENU:
			switch (event->key) {
				case '*':
					homeScreen();
					break;
				case 'A':
					armingScreen();
					break;
				case 'C':
					unlockScreen();
					break;
				default:
					break;
			}
			break;
		case Screen::ARMING:
			switch (event->key) {
				case '*':
					homeScreen();
					break;
				case '#':
					if (armCode.length() == 4) {
						if (isArmCodeValid()) {
							armedScreen();
						}
						else {

						}
					}
					break;
				default:
					maskAccessCode(event->key);
					break;
			}
			break;
		case Screen::UNLOCK:
			switch (event->key) {
				case '*':
					homeScreen();
					break;
				case '#':

					break;
				default:
					maskAccessCode(event->key);
					break;
			}
		case Screen::ARMED:
			if (event->key == '#' && armState == ArmState::ARMED) {
				disarmingScreen();
			}
			else if (event->key == '*' && armState == ArmState::DISARMED) {
				// TODO Indicate cancelled first?
				homeScreen();
			}
			break;
		case Screen::DISARMING:
			switch (event->key) {
				case '*':
					// TODO Clear input?
					break;
				case '#':
					// TODO Disarm logic?
					homeScreen();
					break;
				default:
					maskAccessCode(event->key);
					break;
			}
		default:
			break;
	}
}

void initSerial() {
	Serial.begin(DEBUG_BAUD_RATE);
	while (!Serial) {
		delay(1);
	}

	Serial.print(F("INIT: CyGate4-Keypad v"));
	Serial.print(FIRMWARE_VERSION);
	Serial.println(F(" booting..."));
}

void initDisplay() {
	Serial.print(F("INIT: Initializing LCD display... "));
	display.begin(DT_SIXTEENBYTWO, DisplayBaudRate::BAUD_9600);
	display.rebootDisplay();
	display.clearDisplay();
	display.enableBacklight();
	display.hideCursor();
	display.print(F("CG4-Keypad v"));
	display.print(FIRMWARE_VERSION);
	display.crlf();
	display.print(F("booting..."));
	Serial.println(F("DONE"));
	delay(500);
}

void initKeypad() {
	Serial.print(F("INIT: Initializing keypad... "));
	display.clearDisplay();
	display.print(F("Keypad init..."));
	ButtonpadDriver.begin(keypadHandler);
	Serial.println(F("DONE"));
}

void initCommBus() {
	Serial.print(F("INIT: Initializing I2C comm bus... "));

	byte addressOffset = 0;
	for (uint8_t i = 0; i < 3; i++) {
		pinMode(addrPins[i], INPUT_PULLUP);
		delay(1);

		addressOffset <<= 1;
		if (digitalRead(addrPins[i]) == HIGH) {
			addressOffset |= 0x01;
		}
	}

	int busAddress = ADDRESS_BASE + addressOffset;
	Wire.begin(busAddress);
	Wire.onReceive(commBusReceiveHandler);
	Wire.onRequest(commBusRequestHandler);
	
	Serial.println(F("DONE"));
	Serial.print(F("INIT: I2C bus address: "));
	Serial.println(busAddress, HEX);
}

void setup() {
	initSerial();
	initDisplay();
	initKeypad();
	initCommBus();
	homeScreen();
	Serial.println(F("INIT: Boot sequence complete"));
}

void loop() {
	ButtonpadDriver.loop();

	unsigned long currentMillis = millis();
	switch (currentScreen) {
		case Screen::ARMED:
			if (count > 0 && (currentMillis - previousMillis) >= 1000) {
				count--;
				previousMillis = currentMillis;
				display.setCursorPos(8, 1);
				display.print(count);
				if (count < 10) {
					display.print(F(" "));	
				}
			}

			if (count == 0) {
				display.setCursorPos(0, 1);
				display.print(F("          "));
				armState = ArmState::ARMED;
			}
			break;
		case Screen::DISARMING:
			// TODO probably need to have some kind of timer where if the user doesn't
			// enter a valid code (in say 30 seconds?), the alarm triggers.
			break;
		default:
			break;
	}
}