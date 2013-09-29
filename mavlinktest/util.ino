
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

uint8_t read_packet_old(mavlink_message_t *msg, 
                 mavlink_status_t *status, 
                 HardwareSerial *source, 
                 HardwareSerial *target) {
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

uint8_t read_packet(comm_t *src, comm_t *target) {
	//grabing data 
	while(src->serial->available() > 0) { 
		uint8_t c = src->serial->read();
		target->serial->write(c);

		//trying to grab msg  
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &(src->msg), &(src->status))) {
			return 1;
		}
	}
	return 0;
}


