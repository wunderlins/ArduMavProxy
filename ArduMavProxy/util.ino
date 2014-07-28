// utility methods

/**
 * get a bit from one byte
 */
boolean getBit(byte Reg, byte whichBit) {
	boolean State;
	State = Reg & (1 << whichBit);
	return State;
}

/**
 * set a bit in a byte
 */
byte setBit(byte &Reg, byte whichBit, boolean stat) {
	if (stat)
		Reg = Reg | (1 << whichBit);
	else
		Reg = Reg & ~(1 << whichBit);
	return Reg;
}

/**
 * flush input buffer
 *
 * the user must make sure that the buffered packet was used before flusing.
 */
void flush_packet(comm_t *src) {
	src->buffer_count = 0;
	src->buffer[0] = '\0';
}

/**
 * write buffer to serial
 * 
 * writes a buffer and decoded incoming mavlink serial packet to another 
 * serial port.
 */
void route_packet(comm_t *src, comm_t *target) {
	int i=0;
	for (i=0; i < src->buffer_count; i++) {
		target->serial->write(src->buffer[i]);
	}
	// TODO: check if this helps with more than one source,
	//       http://www.pjrc.com/teensy/td_serial.html
	target->serial->flush();
	
	//src->tx += i;
	//flush_packet(src);
}

/**
 * read a mavlink packet
 *
 * returns 1 if we got a complete packet. returns 0 if we need to read 
 * more into the stream.
 * 
 * passthrough is for minimal latency. Best used for sniffing or routing only.
 */
uint8_t read_packet(comm_t *src, comm_t *target, bool passthrough) {
	
	// the packet should have been used, flush it to prevent buffer overflows
	if (src->has_message) {
		src->has_message = false;
		flush_packet(src);
	}
		
	//grabing data 
	while(src->serial->available() > 0) { 
		
		char c = src->serial->read();
		//(src->rx)++;
		
		// fast passthough for low latency. If you use this you cant modify 
		// packages before sending. this is for packet sniffing only.
		if (passthrough) {
			target->serial->write(c);
			//(src->tx)++;
			// FIXME: might add a break here while in passthrough?
		}

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
		
		// try to grab message, decode if complete
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &(src->msg), &(src->status))) {
			src->has_message = true;
			return 1;
		}
	}
	
	return 0;
}


