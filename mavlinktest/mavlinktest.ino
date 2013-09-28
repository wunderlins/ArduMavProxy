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

// Legacy OSD code, can be removed?
boolean getBit(byte Reg, byte whichBit) {
    boolean State;
    State = Reg & (1 << whichBit);
    return State;
}

byte setBit(byte &Reg, byte whichBit, boolean stat) {
    if (stat) {
        Reg = Reg | (1 << whichBit);
    } 
    else {
        Reg = Reg & ~(1 << whichBit);
    }
    return Reg;
}

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;
static uint8_t crlf_count = 0;

static int packet_drops = 0;
static int parse_error = 0;

/*
void request_mavlink_rates()
{
    const int  maxStreams = 6;
    const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RAW_SENSORS,
        MAV_DATA_STREAM_EXTENDED_STATUS,
        MAV_DATA_STREAM_RC_CHANNELS,
        MAV_DATA_STREAM_POSITION,
        MAV_DATA_STREAM_EXTRA1, 
        MAV_DATA_STREAM_EXTRA2};
    const uint16_t MAVRates[maxStreams] = {0x02, 0x02, 0x05, 0x02, 0x05, 0x02};
    for (int i=0; i < maxStreams; i++) {
        mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
            apm_mav_system, apm_mav_component,
            MAVStreams[i], MAVRates[i], 1);
    }
}
*/

void read_mavlink() {
    mavlink_message_t msg; 
    mavlink_status_t status;

    //grabing data 
    while(Serial1.available() > 0) { 
        uint8_t c = Serial1.read();
        Serial2.write(c);

        /* allow CLI to be started by hitting enter 3 times, if no
        heartbeat packets have been received
        if (mavlink_active == 0 && millis() < 20000 && millis() > 5000) {
            if (c == '\n' || c == '\r') {
                crlf_count++;
            } else {
                crlf_count = 0;
            }
            if (crlf_count == 3) {
                //uploadFont();
                ;
            }
        }
         */
        //trying to grab msg  
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            //mavlink_active = 1;
            //handle msg
            switch(msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    mavbeat = 1;
                    apm_mav_system    = msg.sysid;
                    apm_mav_component = msg.compid;
                    apm_mav_type      = mavlink_msg_heartbeat_get_type(&msg);            
                 //   osd_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
                    ///osd_mode = (uint8_t)mavlink_msg_heartbeat_get_custom_mode(&msg);
                    //Mode (arducoper armed/disarmed)
                    base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
                    if(getBit(base_mode,7)) motor_armed = 1;
                    else motor_armed = 0;
                    /*

                    ///osd_nav_mode = 0;          
                    lastMAVBeat = millis();
                    if(waitingMAVBeats == 1){
                        enable_mav_request = 1;
                    }
                    */
                }
                break;
            /*
            case MAVLINK_MSG_ID_SYS_STATUS:
                {

                    osd_vbat_A = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f); //Battery voltage, in millivolts (1 = 1 millivolt)
                    osd_curr_A = mavlink_msg_sys_status_get_current_battery(&msg); //Battery current, in 10*milliamperes (1 = 10 milliampere)         
                    osd_battery_remaining_A = mavlink_msg_sys_status_get_battery_remaining(&msg); //Remaining battery energy: (0%: 0, 100%: 100)
                    //osd_mode = apm_mav_component;//Debug
                    //osd_nav_mode = apm_mav_system;//Debug
                }
                break;

            case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                    osd_lat = mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0f;
                    osd_lon = mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0f;
                    osd_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
                    osd_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
                }
                break; 
            case MAVLINK_MSG_ID_VFR_HUD:
                {
                    osd_airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
                    osd_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
                    osd_heading = mavlink_msg_vfr_hud_get_heading(&msg); // 0..360 deg, 0=north
                    osd_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
                    //if(osd_throttle > 100 && osd_throttle < 150) osd_throttle = 100;//Temporary fix for ArduPlane 2.28
                    //if(osd_throttle < 0 || osd_throttle > 150) osd_throttle = 0;//Temporary fix for ArduPlane 2.28
                    osd_alt = mavlink_msg_vfr_hud_get_alt(&msg);
                    osd_climb = mavlink_msg_vfr_hud_get_climb(&msg);
                }
                break;
            case MAVLINK_MSG_ID_ATTITUDE:
                {
                    osd_pitch = ToDeg(mavlink_msg_attitude_get_pitch(&msg));
                    osd_roll = ToDeg(mavlink_msg_attitude_get_roll(&msg));
                    osd_yaw = ToDeg(mavlink_msg_attitude_get_yaw(&msg));
                }
                break;
            case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
                {
                  nav_roll = mavlink_msg_nav_controller_output_get_nav_roll(&msg);
                  nav_pitch = mavlink_msg_nav_controller_output_get_nav_pitch(&msg);
                  nav_bearing = mavlink_msg_nav_controller_output_get_nav_bearing(&msg);
                  wp_target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(&msg);
                  wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
                  alt_error = mavlink_msg_nav_controller_output_get_alt_error(&msg);
                  aspd_error = mavlink_msg_nav_controller_output_get_aspd_error(&msg);
                  xtrack_error = mavlink_msg_nav_controller_output_get_xtrack_error(&msg);
                }
                break;
            case MAVLINK_MSG_ID_MISSION_CURRENT:
                {
                    wp_number = (uint8_t)mavlink_msg_mission_current_get_seq(&msg);
                }
                break;
            case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                {
                    chan1_raw = mavlink_msg_rc_channels_raw_get_chan1_raw(&msg);
                    chan2_raw = mavlink_msg_rc_channels_raw_get_chan2_raw(&msg);
                    osd_chan5_raw = mavlink_msg_rc_channels_raw_get_chan5_raw(&msg);
                    osd_chan6_raw = mavlink_msg_rc_channels_raw_get_chan6_raw(&msg);
                    osd_chan7_raw = mavlink_msg_rc_channels_raw_get_chan7_raw(&msg);
                    osd_chan8_raw = mavlink_msg_rc_channels_raw_get_chan8_raw(&msg);
                    osd_rssi = mavlink_msg_rc_channels_raw_get_rssi(&msg);
                }
                break;
            case MAVLINK_MSG_ID_WIND:
                {
                    osd_winddirection = mavlink_msg_wind_get_direction(&msg); // 0..360 deg, 0=north
                    osd_windspeed = mavlink_msg_wind_get_speed(&msg); //m/s
                    osd_windspeedz = mavlink_msg_wind_get_speed_z(&msg); //m/s
                }
                break;
            */
            default:
                //Do nothing
                break;
            }
        }
        //delayMicroseconds(138);
        //next one
    }
    // Update global packet drops counter
    packet_drops += status.packet_rx_drop_count;
    parse_error += status.parse_error;

}

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

typedef struct comm_src {
	mavlink_message_t msg = msg1;
	mavlink_status_t status = status1;
	char *buffer = buffer1;
	int *buffer_count = &buffer1_count;
	HardwareSerial *serial = ser_src;
} comm_src_t;

typedef struct comm_modem {
	mavlink_message_t msg = msg2;
	mavlink_status_t status = status2;
	char *buffer = buffer2;
	int *buffer_count = &buffer2_count;
	HardwareSerial *serial = ser_modem;
} comm_modem_t;

typedef struct comm_ext {
	mavlink_message_t msg = msg3;
	mavlink_status_t status = status3;
	char *buffer = buffer2;
	int *buffer_count = &buffer3_count;
	HardwareSerial *serial = ser_ext;
} comm_ext_t;

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

uint8_t read_packet(mavlink_message_t *msg, 
                 mavlink_status_t *status, 
                 HardwareSerial *source, 
                 HardwareSerial *target ) {
	//grabing data 
	while(source->available() > 0) { 
		uint8_t c = source->read();
		target->write(c);

		//trying to grab msg  
		if(mavlink_parse_char(MAVLINK_COMM_0, c, msg, status)) {
			return 1;
		}
	}
	return 0;
}

uint8_t read_packet_new(struct comm_src *comm) {
	/*
	//grabing data 
	while(source->available() > 0) { 
		uint8_t c = source->read();
		target->write(c);

		//trying to grab msg  
		if(mavlink_parse_char(MAVLINK_COMM_0, c, msg, status)) {
			return 1;
		}
	}
	*/
	return 0;
}

void loop() {
	//read_mavlink();
	uint8_t ret1 = read_packet(&msg1, &status1, ser_src, ser_modem);
	uint8_t ret2 = read_packet(&msg2, &status2, ser_modem, ser_src);
	
	if (ret1) { // we got a complete message from the source
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
		
		if (!base_mode)
			Serial.print("00000000");
		else
			Serial.print(base_mode, BIN);
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
	
	//delay(50);
	
	/*
  if(Serial2.available() > 0) { 
		uint8_t c = Serial2.read();
		Serial1.write(c);
	}
	*/
	
}


