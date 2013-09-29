#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

#include <GCS_MAVLink.h>
#include "mavlinktest.h"

void setup() {
 	//Serial.begin(TELEMETRY_SPEED);
 	
	// setup mavlink port
	//mavlink_comm_0_port = s_src.serial;
	//mavlink_comm_1_port = s_modem.serial;

	mavlink_comm_0_port = s_src.serial;
	mavlink_comm_1_port = s_modem.serial;
	//mavlink_comm_2_port = &Serial3;
	
 	Serial1.begin(TELEMETRY_SPEED);
 	Serial2.begin(TELEMETRY_SPEED);
 	//Serial3.begin(TELEMETRY_SPEED);
 	
 	// set pins to default state
 	pinMode(PIN_ARM, OUTPUT);
 	pinMode(PIN_AUTO, OUTPUT);
 	digitalWrite(PIN_ARM, LOW);
 	digitalWrite(PIN_AUTO, LOW);
}

void loop() {
	//read_mavlink();
	//uint8_t ret1 = read_packet_old(&msg1, &status1, ser_src, ser_modem);
	//uint8_t ret2 = read_packet_old(&msg2, &status2, ser_modem, ser_src);
	uint8_t ret1 = read_packet(&s_src, &s_modem);
	uint8_t ret2 = read_packet(&s_modem, &s_src);
	
	if (ret1) { // we got a complete message from the source
		
		// basic MAV information
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
		
		// system inormation
		if (msg1.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
			apm_mav_sensor_present = mavlink_msg_sys_status_get_onboard_control_sensors_present(&(s_src.msg));
			apm_mav_sensor_enabled = mavlink_msg_sys_status_get_onboard_control_sensors_enabled(&(s_src.msg));
			apm_mav_sensor_health  = mavlink_msg_sys_status_get_onboard_control_sensors_health(&(s_src.msg));
		}
		
		if (msg1.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
			gps_alt = mavlink_msg_gps_raw_int_get_alt(&(s_src.msg));
			gps_vel = mavlink_msg_gps_raw_int_get_vel(&(s_src.msg));
			gps_cog = mavlink_msg_gps_raw_int_get_cog(&(s_src.msg));
			gps_time = mavlink_msg_gps_raw_int_get_time_usec(&(s_src.msg));
			gps_lat = mavlink_msg_gps_raw_int_get_lat(&(s_src.msg)) / 10000000.0f;
			gps_lon = mavlink_msg_gps_raw_int_get_lon(&(s_src.msg)) / 10000000.0f;
			gps_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&(s_src.msg));
			gps_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&(s_src.msg));
		}

		
		Serial.print(gps_satellites_visible);
		Serial.print("\t");
		Serial.print(motor_armed, HEX);
		Serial.print("\t");
		Serial.println(base_mode, BIN);
		
		digitalWrite(PIN_ARM, (motor_armed) ? HIGH : LOW);
		digitalWrite(PIN_AUTO, (mode_auto) ? HIGH : LOW);
	}

	if (ret2) { // we got a complete message from the source
		if (s_modem.msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
			;
		}
	}
	
}


