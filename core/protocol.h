struct dqn_tr{
 	// 2 bytes
	uint16_t 	    id;
	// 1 byte
    uint8_t  	    num_slots;
    // 1 byte
	uint8_t		    crc
} __attribute__((packed));  // total is 4 bytes


struct dqn_data{
    // 30 bytes
    uint8_t[30]     data
} __attribute__((packed));  // total is 30 bytes


struct feedback{
    // 2 bytes
    uint16_t        dtq_length;
    // 2 bytes
    uint16_t        crq_length;
    // 8 bytes
    uint64_t        result; // this is bloom filter result
} __attribute__((packed));  // total is 12 bytes


// this is used to hash an integer value into
// to create a new bloom filter, pass 0 as the hash_value to initialize it
uint64_t bloom_hash(uint64_t hash_value, int value);

bool bloom_test(int value);
