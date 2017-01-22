struct TR{
 	// 2 bytes
	uint16_t 	id;
	// 1 byte
	uint8_t  	num_slots;
    // 1 byte
	uint8_t		crc
} __attribute__((packed));  // total is 4 bytes
