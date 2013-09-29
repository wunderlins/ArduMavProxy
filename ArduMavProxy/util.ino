
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

void flush_packet(comm_t *src) {
	src->buffer_count = 0;
	src->buffer[0] = '\0';
}

void route_packet(comm_t *src, comm_t *target) {
	for (int i=0; i <= src->buffer_count; i++)
		target->serial->write(src->buffer[i]);
	flush_packet(src);
}

/**
 * read a mavlink packet
 *
 * returns 1 if we got a complete packet. returns 0 if we need to read 
 * more into the stream.
 * 
 * passthrough is for minimal latency. Best used when only sniffing
 *
 */
uint8_t read_packet(comm_t *src, comm_t *target, bool passthrough) {
	//grabing data 
	while(src->serial->available() > 0) { 
		src->has_message = false;
		char c = src->serial->read();
		
		// buffer the received character
		src->buffer[src->buffer_count] = c;
		(src->buffer_count)++;
		
		// buffer overflow protection
		if (src->buffer_count == MAVLINK_FRAME_LENGTH) {
			// flush stream buffer if full
			//src->buffer_count = 0;
			//src->buffer[0] = '\0';
			flush_packet(src);
		}
		
		if (passthrough)
			target->serial->write(c);

		// try to grab message, decode if complete
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &(src->msg), &(src->status))) {
			src->has_message = true;
			return 1;
		}
	}
	return 0;
}


