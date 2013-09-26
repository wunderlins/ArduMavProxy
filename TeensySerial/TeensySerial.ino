/* UART Example, any character received on either the real
   serial port, or USB serial (or emulated serial to the
   Arduino Serial Monitor when using non-serial USB types)
   is printed as a message to both ports.

   This example code is in the public domain.
*/

// This line defines a "Uart" object to access the serial port
//HardwareSerial Uart = HardwareSerial();

#define TELEMETRY_SPEED 57600
#define ARDUINO_TELEMETRY_SPEED 58824

void setup() {
	Serial.begin(TELEMETRY_SPEED);
	Serial1.begin(ARDUINO_TELEMETRY_SPEED);
}

void loop() {
	int incomingByte;
  /*      
	if (Serial.available() > 0) {
		incomingByte = Serial.read();
		Serial.print("USB received: ");
		Serial.println(incomingByte, DEC);
		Serial1.print("USB received:");
		Serial1.println(incomingByte, DEC);
	}
	*/
	if (Serial1.available() > 0) {
		incomingByte = Serial1.read();
		Serial.print("UART received: ");
		Serial.println(incomingByte, DEC);
//		Serial1.print("UART received:");
//		Serial1.println(incomingByte, DEC);
	}
	
	Serial.println("end loop");
	delay(1000);
}

