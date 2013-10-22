#include <GCS_MAVLink.h>
#include "ArduMavProxy.h"

// #define DBG

#ifdef DBG
#include "message_types.h"
#endif

// message structs
static mavlink_message_t msg1;
static mavlink_message_t msg2;
static mavlink_message_t msg3;
static mavlink_status_t status1;
static mavlink_status_t status2;
static mavlink_status_t status3;

// Serial devices
static comm_t s_src     = {"", 0, &Serial1, msg1, status1, 0, 1 /*, 0, 0*/};
static comm_t s_modem   = {"", 0, &Serial2, msg2, status2, 0, 2 /*, 0, 0*/};
static comm_t s_ext     = {"", 0, &Serial,  msg3, status3, 0, 3 /*, 0, 0 */};

/**
 * User setup hook
 *
 * This method is called fro mthe setup function, there shouldn't be a need to 
 * change the setup, do it here.
 */
void on_setup() {
	#define PIN_ARM 13
	#define PIN_AUTO 14
	
 	pinMode(PIN_ARM, OUTPUT);
 	pinMode(PIN_AUTO, OUTPUT);
 	digitalWrite(PIN_ARM, LOW);
 	digitalWrite(PIN_AUTO, LOW);
}

/**
 * user functions for serial 1
 * 
 * Add your hooks here. No need to change the main loop if you just want to 
 * sniff or manipulate a message. You may not use passthrough in the main loop 
 * if you want to alter a packet before it is sent out to antoher 
 * serial link.
 */
uint8_t base_mode, motor_armed, mode_auto, mission_current = 0;
//static uint32_t num_packets = 0;
void on_serial1(comm_t *message) {
	
	#ifdef DBG
	Serial.print("1: ");
	Serial.print(message->msg.msgid);
	Serial.print(" ");
	Serial.println(msgtypes[message->msg.msgid]);
	/*
	if (message->msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
		base_mode = mavlink_msg_heartbeat_get_base_mode(&(s_src.msg));
		Serial.println(base_mode);
	}
	*/
	#endif


	if (message->msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
		base_mode = mavlink_msg_heartbeat_get_base_mode(&(s_src.msg));
	
		// is armed?
		if(getBit(base_mode, 7)) 
			motor_armed = 1;
		else 
			motor_armed = 0;

		// is auto ?
		if(getBit(base_mode, 3)) 
			mode_auto = 1;
		else 
			mode_auto = 0;
	
		digitalWrite(PIN_ARM, (motor_armed) ? HIGH : LOW);
		digitalWrite(PIN_AUTO, (mode_auto) ? HIGH : LOW);
	}
	
	if (message->msg.msgid == MAVLINK_MSG_ID_MISSION_CURRENT) {
		mission_current = (uint8_t) mavlink_msg_mission_current_get_seq(&(s_src.msg));
		
//		Serial.print("WP no: ");
//		Serial.println(mission_current);
//		Serial.print(" num pkg: ");
//		Serial.print(num_packets);
//		Serial.print(" rx: ");
//		Serial.print(message->rx);
//		Serial.print(" tx: ");
//		Serial.print(message->tx);
//		Serial.print(" droped: ");
//		int32_t dropped = message->rx - message->tx;
//		Serial.println(dropped);
		
	} 
	
}

/**
 * user functions for serial 2
 * 
 * Add your hooks here. No need to change the main loop if you just want to 
 * sniff or manipulate a message. You may not use passthrough in the main loop 
 * if you want to alter a packet before it is sent out to antoher 
 * serial link.
 */
void on_serial2(comm_t *message) {
	#ifdef DBG
	Serial.print("2: ");
	Serial.print(message->msg.msgid);
	Serial.print(" ");
	Serial.println(msgtypes[message->msg.msgid]);
	/*
	if (message->msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
		base_mode = mavlink_msg_heartbeat_get_base_mode(&(s_src.msg));
		Serial.println(base_mode);
	}
	*/
	#endif
}

/**
 * user functions for serial usb
 * 
 * Add your hooks here. No need to change the main loop if you just want to 
 * sniff or manipulate a message. You may not use passthrough in the main loop 
 * if you want to alter a packet before it is sent out to antoher 
 * serial link.
 */
void on_serial(comm_t *message) {
	// do something useful
	;
}

void setup() {
 	Serial.begin(115200);
 	Serial1.begin(57600);
 	Serial2.begin(TELEMETRY_SPEED); // FIXME: s_modem.serial->begin() doesn't work
 	
 	on_setup();
}

void loop() {
	
	// No passthrough to modem so we queue src packages
	passthrough = true;
	uint8_t ret1 = read_packet(&s_src, &s_modem, passthrough);
	
        /*
	if (ret1) { // we got a complete message from the source
		
		on_serial1(&s_src);
		
		if (passthrough == false) {
			//num_packets++;
			route_packet(&s_src, &s_modem);
			flush_packet(&s_src);
		}
	}
        */
	
	// read mavlink package from modem
	//passthrough = false;
	uint8_t ret2 = read_packet(&s_modem, &s_src, passthrough);

        /*
	if (ret2) { // we got a complete message from the source
		// TODO: implement fast passthrough for 2 channels
		on_serial2(&s_modem);
		
		if (passthrough == false) {
			route_packet(&s_modem, &s_src);
			#ifndef DBG
			route_packet(&s_modem, &s_ext);
			#endif
			flush_packet(&s_modem);
		}
	}
	
	#ifndef DBG
	// No passthrough to modem so we queue src packages
	passthrough = false;
        */
	uint8_t ret3 = read_packet(&s_ext, &s_modem, passthrough);

        /*
	if (ret3) { // we got a complete message from the source
		
		on_serial(&s_ext);
		
		if (passthrough == false) {
			route_packet(&s_ext, &s_modem);
			flush_packet(&s_ext);
		}
	}
	#endif
        */
}

