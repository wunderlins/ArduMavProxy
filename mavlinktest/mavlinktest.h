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
static uint8_t      apm_mav_type;
static uint8_t      apm_mav_system; 
static uint8_t      apm_mav_component;

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

typedef struct comm_t {
	char buffer[MAVLINK_FRAME_LENGTH + 1];
	int buffer_count;
	HardwareSerial *serial;
	mavlink_message_t msg;
	mavlink_status_t status;
	bool has_message;
	uint8_t chan;
};

typedef struct comm_usb_t {
	char buffer[MAVLINK_FRAME_LENGTH + 1];
	int buffer_count;
	usb_serial_class *serial;
	mavlink_message_t msg;
	mavlink_status_t status;
	bool has_message;
	uint8_t chan;
};

uint8_t mode_auto = 0;

