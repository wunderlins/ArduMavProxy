/*

*/

/// Legacy, remove?
#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 

#undef PSTR 
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 

// AVR Includes
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <math.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "wiring.h"
#endif
#include <EEPROM.h>
#include <SimpleTimer.h>
#include <GCS_MAVLink.h>

#ifdef membug
#include <MemoryFree.h>
#endif

//#include <SoftwareSerial.h>
#include <BetterSoftwareSerial.h>
BetterSoftwareSerial MavPortAPM(10, 11); // TX, RX
BetterSoftwareSerial MavPortModem(2, 3); // TX, RX

// Configurations
#include "vars.h"

#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port
#define BOOTTIME         2000   // Time in milliseconds that we show boot loading bar and wait user input

// Objects and Serial definitions
//FastSerialPort0(Serial);

SimpleTimer  mavlinkTimer;

void setup() 
{
#ifdef ArduCAM328
    pinMode(10, OUTPUT); // USB ArduCam Only
#endif

    //Serial.begin(TELEMETRY_SPEED);
    // setup mavlink port
    //mavlink_comm_0_port = &Serial;
    mavlink_comm_0_port = &MavPortModem;

#ifdef membug
    Serial.println(freeMem());
#endif

    //delay(500);

    // Startup MAVLink timers  
    mavlinkTimer.Set(&OnMavlinkTimer, 120);

    mavlinkTimer.Enable();

} // END of setup();


void loop() 
{

    if(enable_mav_request == 1){//Request rate control
        for(int n = 0; n < 3; n++){
            request_mavlink_rates();//Three times to certify it will be readed
            delay(50);
        }
        enable_mav_request = 0;
        delay(2000);
        waitingMAVBeats = 0;
        lastMAVBeat = millis();//Preventing error from delay sensing
    }

    read_mavlink();
    mavlinkTimer.Run();
}

/* *********************************************** */
/* ******** functions used in main loop() ******** */
void OnMavlinkTimer() {
	;
}

/// Legacy, remove?
void unplugSlaves(){
	;
}
