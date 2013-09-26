/* UART Example, any character received on either the real
   serial port, or USB serial (or emulated serial to the
   Arduino Serial Monitor when using non-serial USB types)
   is printed as a message to both ports.

   This example code is in the public domain.
*/

// This line defines a "Uart" object to access the serial port
//HardwareSerial Uart = HardwareSerial();

#define SERIAL_SPEED 57600

void setup() {
	Serial.begin(SERIAL_SPEED);  // USB debug console
	Serial1.begin(SERIAL_SPEED); // BlueSmirf, pin 0 -> RX-I, pin 1 -> TX-O
	Serial2.begin(SERIAL_SPEED); // 3DR Modem, pin 9 -> TX, pin 10 -> RX
}

void loop() {
	char incomingByte;
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
		Serial.print("> 0x");
		Serial.println(incomingByte, HEX);
		Serial2.print(incomingByte);
	}
	
	if (Serial2.available() > 0) {
		incomingByte = Serial2.read();
		Serial.print("< 0x");
		Serial.println(incomingByte, HEX);
		Serial1.print(incomingByte);
	}
	
	//Serial.println("end loop");
	//delay(10);
}

