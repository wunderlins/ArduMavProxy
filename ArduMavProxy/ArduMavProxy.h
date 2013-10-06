// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "wiring.h"
#endif

#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial 
#define MAVLINK_FRAME_LENGTH 263

#include <GCS_MAVLink.h>
#include "include/mavlink/v1.0/mavlink_types.h"
bool passthrough = false;

typedef struct comm_t {
	char buffer[MAVLINK_FRAME_LENGTH + 1];
	int buffer_count;
	Stream *serial;
	mavlink_message_t msg;
	mavlink_status_t status;
	bool has_message;
	uint8_t chan;
	//uint32_t rx;
	//uint32_t tx;
};

