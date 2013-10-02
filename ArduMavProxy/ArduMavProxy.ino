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
		
		// route raw buffer input from src to target
		/*
		for (int i=0; i <= s_src.buffer_count; i++)
			s_modem.serial->write(s_src.buffer[i]);
		s_src.buffer_count = 0;
		s_src.buffer[0] = '\0';
		*/
		route_packet(&s_src, &s_modem);
		flush_packet(&s_src);
		
		// basic MAV information
		/*
		if (s_src.msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
			mavbeat = 1;
			apm_mav_system    = s_src.msg.sysid;
			apm_mav_component = s_src.msg.compid;
			apm_mav_type      = mavlink_msg_heartbeat_get_type(&(s_src.msg));
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
		}
		*/
		
		/*
		// system inormation
		if (s_src.msg.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
			apm_mav_sensor_present = mavlink_msg_sys_status_get_onboard_control_sensors_present(&(s_src.msg));
			apm_mav_sensor_enabled = mavlink_msg_sys_status_get_onboard_control_sensors_enabled(&(s_src.msg));
			apm_mav_sensor_health  = mavlink_msg_sys_status_get_onboard_control_sensors_health(&(s_src.msg));
		}
		
		if (s_src.msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
			gps_alt = mavlink_msg_gps_raw_int_get_alt(&(s_src.msg));
			gps_vel = mavlink_msg_gps_raw_int_get_vel(&(s_src.msg));
			gps_cog = mavlink_msg_gps_raw_int_get_cog(&(s_src.msg));
			gps_time = mavlink_msg_gps_raw_int_get_time_usec(&(s_src.msg));
			gps_lat = mavlink_msg_gps_raw_int_get_lat(&(s_src.msg)) / 10000000.0f;
			gps_lon = mavlink_msg_gps_raw_int_get_lon(&(s_src.msg)) / 10000000.0f;
			gps_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&(s_src.msg));
			gps_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&(s_src.msg));
		}
		*/
		
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
		/*
		for (int i=0; i <= s_modem.buffer_count; i++)
			s_src.serial->write(s_modem.buffer[i]);
		*/
		
		// TODO: implement fast passthrough for 2 channels
		route_packet(&s_modem, &s_ext);
		//route_packet(&s_modem, &s_src);
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

