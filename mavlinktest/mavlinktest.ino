#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

#include <GCS_MAVLink.h>

//Will come from APM telem port
static uint8_t      base_mode=0;
static bool         motor_armed = 0;

//MAVLink session control
static boolean      mavbeat = 0;
//static float        lastMAVBeat = 0;
//static boolean      waitingMAVBeats = 1;
static uint8_t      apm_mav_type;
static uint8_t      apm_mav_system; 
static uint8_t      apm_mav_component;
//static boolean      enable_mav_request = 0;

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;
static uint8_t crlf_count = 0;

static int packet_drops = 0;
static int parse_error = 0;

static uint32_t apm_mav_sensor_present;
static uint32_t apm_mav_sensor_enabled;
static uint32_t apm_mav_sensor_health;

static uint32_t gps_alt = 0;
static uint16_t gps_vel = 0;
static uint16_t gps_cog = 0;
static uint64_t gps_time = 0;
static int32_t gps_lat = 0;
static int32_t gps_lon = 0;
static uint8_t gps_fix_type = 0;
static uint8_t gps_satellites_visible = 0;

#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial 
#define PIN_ARM 13
#define PIN_AUTO 14
#define MAVLINK_FRAME_LENGTH 263

// sessage structs
mavlink_message_t msg1;
mavlink_status_t status1;
mavlink_message_t msg2;
mavlink_status_t status2;
mavlink_message_t msg3;
mavlink_status_t status3;

// routing message buffers (stram only)
char buffer1[MAVLINK_FRAME_LENGTH + 1] = "";
char buffer2[MAVLINK_FRAME_LENGTH + 1] = "";
char buffer3[MAVLINK_FRAME_LENGTH + 1] = "";
int buffer1_count = 0;
int buffer2_count = 0;
int buffer3_count = 0;

// Serial aliases for better reading in code
HardwareSerial *ser_src   = &Serial1;
HardwareSerial *ser_modem = &Serial2;
HardwareSerial *ser_ext   = &Serial3;

uint8_t mode_auto = 0;

void setup() {
 	Serial.begin(TELEMETRY_SPEED);
 	
	// setup mavlink port
	mavlink_comm_0_port = &Serial1;
	mavlink_comm_1_port = &Serial2;
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
	uint8_t ret1 = read_packet(&msg1, &status1, ser_src, ser_modem);
	uint8_t ret2 = read_packet(&msg2, &status2, ser_modem, ser_src);
	
	if (ret1) { // we got a complete message from the source
		
		// basic MAV information
		if (msg1.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
			mavbeat = 1;
			apm_mav_system    = msg1.sysid;
			apm_mav_component = msg1.compid;
			apm_mav_type      = mavlink_msg_heartbeat_get_type(&msg1);
			base_mode = mavlink_msg_heartbeat_get_base_mode(&msg1);
			
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
			apm_mav_sensor_present = mavlink_msg_sys_status_get_onboard_control_sensors_present(&msg1);
			apm_mav_sensor_enabled = mavlink_msg_sys_status_get_onboard_control_sensors_enabled(&msg1);
			apm_mav_sensor_health  = mavlink_msg_sys_status_get_onboard_control_sensors_health(&msg1);
		}
		
		if (msg1.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
			gps_alt = mavlink_msg_gps_raw_int_get_alt(&msg1);
			gps_vel = mavlink_msg_gps_raw_int_get_vel(&msg1);
			gps_cog = mavlink_msg_gps_raw_int_get_cog(&msg1);
			gps_time = mavlink_msg_gps_raw_int_get_time_usec(&msg1);
			gps_lat = mavlink_msg_gps_raw_int_get_lat(&msg1) / 10000000.0f;
			gps_lon = mavlink_msg_gps_raw_int_get_lon(&msg1) / 10000000.0f;
			gps_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg1);
			gps_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg1);
		}

		
		if (!apm_mav_sensor_enabled)
			Serial.print("00000000");
		else
			Serial.print(gps_satellites_visible);
		Serial.print("\t");
		Serial.print(motor_armed, HEX);
		Serial.print("\t");
		Serial.println(base_mode, HEX);
		
		digitalWrite(PIN_ARM, (motor_armed) ? HIGH : LOW);
		digitalWrite(PIN_AUTO, (mode_auto) ? HIGH : LOW);
	}

	if (ret2) { // we got a complete message from the source
		if (msg2.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
			;
		}
	}
	
}


