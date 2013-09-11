//Will come from APM telem port
static uint8_t      base_mode=0;
static bool         motor_armed = 0;

//MAVLink session control
static boolean      mavbeat = 0;
static float        lastMAVBeat = 0;
static boolean      waitingMAVBeats = 1;
static uint8_t      apm_mav_type;
static uint8_t      apm_mav_system; 
static uint8_t      apm_mav_component;
static boolean      enable_mav_request = 0;

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

