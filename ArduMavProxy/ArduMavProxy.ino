#include <GCS_MAVLink.h>
#include "ArduMavProxy.h"

// #define DBG

// message structs
static mavlink_message_t msg1;
static mavlink_message_t msg2;
static mavlink_message_t msg3;
static mavlink_status_t status1;
static mavlink_status_t status2;
static mavlink_status_t status3;

// Serial devices
static comm_t s_src     = {"", 0, &Serial1, msg1, status1, 0, 1};
static comm_t s_modem   = {"", 0, &Serial2, msg2, status2, 0, 2};
static comm_t s_ext = {"", 0, &Serial,  msg3, status3, 0, 3};

void setup() {
 	Serial.begin(TELEMETRY_SPEED);
 	Serial1.begin(TELEMETRY_SPEED);
 	Serial2.begin(TELEMETRY_SPEED); // FIXME: s_modem.serial->begin() doesn't work
 	//Serial3.begin(TELEMETRY_SPEED);
 	
 	// set pins to default state
 	pinMode(PIN_ARM, OUTPUT);
 	pinMode(PIN_AUTO, OUTPUT);
 	digitalWrite(PIN_ARM, LOW);
 	digitalWrite(PIN_AUTO, LOW);
}

void loop() {
	
	// No passthrough to modem so we queue src packages
	uint8_t ret1 = read_packet(&s_src, &s_modem, false);
	
	// TODO: check for comm_t.has_packet
	if (ret1) { // we got a complete message from the source
		route_packet(&s_src, &s_modem);
		flush_packet(&s_src);
		
		#ifdef DBG
			Serial.print("Sats: ");
			Serial.print(gps_satellites_visible);
			Serial.print(", fix: ");
			Serial.print(gps_fix_type);
			Serial.print("\t");
			Serial.print(motor_armed, HEX);
			Serial.print("\t");
			Serial.println(base_mode, BIN);
		#endif
		
		digitalWrite(PIN_ARM, (motor_armed) ? HIGH : LOW);
		digitalWrite(PIN_AUTO, (mode_auto) ? HIGH : LOW);
	}
	
	// read mavlink package from modem
	uint8_t ret2 = read_packet(&s_modem, &s_src, true);
	if (ret2) { // we got a complete message from the source
		// TODO: implement fast passthrough for 2 channels
		route_packet(&s_modem, &s_ext);
		flush_packet(&s_modem);
	}
	
	#ifndef DBG
	// No passthrough to modem so we queue src packages
	uint8_t ret3 = read_packet(&s_ext, &s_modem, false);
	if (ret2) { // we got a complete message from the source
		route_packet(&s_ext, &s_modem);
		flush_packet(&s_ext);
	}
	#endif
}

