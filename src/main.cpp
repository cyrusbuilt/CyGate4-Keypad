#include <Arduino.h>
#include <Wire.h>
#include "Buzzer.h"
#include "CrystalFontz632.h"
#include "ButtonpadDriver.h"
#include "KeypadProtocol.h"
#include "LED.h"

#define FIRMWARE_VERSION "1.0"

// Pin definitions
#define PIN_DISPLAY_TX 13
#define PIN_ADDRESS_A0 A0
#define PIN_ADDRESS_A1 A1
#define PIN_ADDRESS_A2 A2
#define PIN_PWR_LED 6
#define PIN_ACT_LED 7
#define PIN_PIEZO 5

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
	DISARMING = 5,
	INVALID_CODE = 6
};

enum class ArmState : uint8_t {
	DISARMED = 0,
	ARMED = 1
};

CrystalFontz632 display(PIN_DISPLAY_TX);
LED pwrLED(PIN_PWR_LED, NULL);
LED actLED(PIN_ACT_LED, NULL);
Buzzer buzzer(PIN_PIEZO, NULL, NULL);
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
		data.data[i] = 0x00;
	}
}

void sendKeyCode() {
	uint8_t buffer[3 + data.size];
	buffer[0] = KEYPAD_GET_CMD_DATA;
	buffer[1] = data.command;
	buffer[2] = data.size;
	for (uint8_t i = 0; i < data.size; i++) {
		buffer[i + 3] = data.data[i];
	}

	Wire.write(buffer, data.size + 3);
	clearKeypadData();
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
	case KEYPAD_BAD_CODE:
		// TODO Set bad code and go to invalid code screen?
		// TODO probably also flash LED and beep?
		break;
	default:
		break;
	}

	command = 0xFF;
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
	data.command = (uint8_t)KeypadCommands::ARM_AWAY;
	const char* charBuf = armCode.c_str();
	for (uint8_t i = 0; i < KEYPAD_DATA_BUFFER_SIZE; i++) {
		data.data[i] = (byte)charBuf[i];
	}

	Serial.print(F("INFO: Arm code: "));
	Serial.println(armCode);
	Serial.println(F("WARN: *** ARMED ***"));
	display.clearDisplay();
	display.hideCursor();
	display.print(F("*** ARMED ***"));
	display.setCursorPos(0, 1);
	display.print(F("EXIT IN "));
	display.print(count);
	currentScreen = Screen::ARMED;
	count = ARMING_COUNT_DOWN_START;
	delete[] charBuf;
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
	if (armCode.length() == 4) {
		// TODO Send the arm code to host via I2C for validation
		return true;
	}
	return false;
}

bool isUnlockCodeValid() {
	if (unlockCode.length() == 4) {
		// TODO Send the arm code to host via I2C for validation
		return true;
	}
	return false;
}

void maskAccessCode(char key) {
	if (key != 'A' && key != 'B' && key != 'C' && key != 'D') {
		display.print(F("*"));
	}
}

void invalidCodeScreen() {
	Serial.println(F("WARN: Invalid access code."));
	display.clearDisplay();
	display.hideCursor();
	display.print(F("INVALID CODE"));
	currentScreen = Screen::INVALID_CODE;
	delay(3000);
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
					Serial.print(F("DEBUG: Arm code = '"));
					Serial.print(armCode);
					Serial.println(F("'"));
					if (isArmCodeValid()) {
						armedScreen();
					}
					else {
						invalidCodeScreen();
						armingScreen();
					}
					break;
				default:
					if (armCode.length() < 8) {
						maskAccessCode(event->key);
						armCode += event->key;
					}
					break;
			}
			break;
		case Screen::UNLOCK:
			switch (event->key) {
				case '*':
					homeScreen();
					break;
				case '#':
					if (isUnlockCodeValid()) {
						// TODO briefly indicate unlocked?
					}
					else {
						invalidCodeScreen();
						unlockScreen();
					}
					break;
				default:
					if (unlockCode.length() < 5) {
						maskAccessCode(event->key);
						unlockCode += event->key;
					}
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

void initOutputs() {
	Serial.print(F("INIT: Initializing outputs... "));
	pwrLED.init();
	pwrLED.on();

	actLED.init();
	actLED.on();

	buzzer.init();
	buzzer.off();
	Serial.println(F("DONE"));
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
	display.carriageReturn();
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
	initOutputs();
	initDisplay();
	initKeypad();
	initCommBus();
	homeScreen();
	Serial.println(F("INIT: Boot sequence complete"));
	actLED.off();
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