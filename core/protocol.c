#define BLOOM_SIZE 64
#include "protocol.h"

// function protocols here
uint64_t hash(uint64_t);


uint64_t bloom_hash(uint64_t hash_value, int value){
    uint64_t hash_value_temp = hash(value);
    return hash_value | (1 << (hash_value_temp % BLOOM_SIZE));	
}


bool bloom_test(uint64_t hash_value, int value){
    uint64_t hash_value_temp = hash(value);
    int bit_pos = hash_value_temp % BLOOM_SIZE;
    return !(!(hash_value & (1 << bit_pos)));
}

// taken from http://stackoverflow.com/a/12996028
uint64_t hash(uint64_t x) {
    x = (x ^ (x >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    x = (x ^ (x >> 27)) * UINT64_C(0x94d049bb133111eb);
    x = x ^ (x >> 31);
    return x;
}

// this is a blocking method
void send_data(char* data){
}

uint8_t get_crc8(char *data, int len){
    return 0;
}
