// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "wiring.h"
#endif

#include <GCS_MAVLink.h>
#include "include/mavlink/v1.0/mavlink_types.h"


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

typedef struct comm_t {
	char buffer[MAVLINK_FRAME_LENGTH + 1];
	int buffer_count;
	HardwareSerial *serial;
	mavlink_message_t msg;
	mavlink_status_t status;
};

static comm_t s_src =   {"", 0, &Serial1, msg1, status1};
static comm_t s_modem = {"", 0, &Serial2, msg2, status2};
static comm_t s_ext =   {"", 0, &Serial3, msg3, status3};

uint8_t mode_auto = 0;

