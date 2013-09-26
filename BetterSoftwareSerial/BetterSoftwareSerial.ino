#include <FastSerial.h>
//#include <BetterSoftwareSerial.h>
#include <SoftwareSerial.h>

// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "wiring.h"
#endif

#define TELEMETRY_SPEED 57600  // How fast our MAVLink telemetry is coming 
//BetterSoftwareSerial MavPortAPM(10, 11); // TX, RX
//BetterSoftwareSerial MavPortModem(8, 9); // TX, RX

SoftwareSerial MavPortAPM(11, 10); // RX, TX
SoftwareSerial MavPortModem(9, 8); // RX, TX

FastSerialPort0(Serial);

void setup() {
	Serial.begin(TELEMETRY_SPEED);
	MavPortAPM.begin(TELEMETRY_SPEED);
	MavPortModem.begin(TELEMETRY_SPEED);
	
	delay(500);
	Serial.print("Booted ...\n");
}

void loop() {
	
	while (MavPortModem.available() > 0) { 
		uint8_t c = MavPortModem.read();
		MavPortAPM.print(c);
	}
	
	while (MavPortAPM.available() > 0) { 
		uint8_t c = MavPortAPM.read();
		MavPortModem.print(c);
	}
	
	Serial.print("end\n");
	delay(1000);
}
