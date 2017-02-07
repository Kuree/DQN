#define BLOOM_SIZE 64
#include <math.h>
#include "protocol.h"

// CRC8 implementation is adapted from 
// http://www.rajivchakravorty.com/source-code/uncertainty/multimedia-sim/html/crc8_8c-source.html

#define GP  0x107   /* x^8 + x^2 + x + 1 */
#define DI  0x07

static unsigned char crc8_table[256];     /* 8-bit table */
static int made_table=0;

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

static void init_crc8(){
    if (!made_table) {
        int i,j;
        unsigned char crc;
        for (i=0; i<256; i++) {
            crc = i;
            for (j=0; j<8; j++)
                crc = (crc << 1) ^ ((crc & 0x80) ? DI : 0);
            crc8_table[i] = crc & 0xFF;
            /* printf("table[%d] = %d (0x%X)\n", i, crc, crc); */
        }
        made_table=1;
    }
}

uint8_t get_crc8(char *data, int len){
    if(!made_table)
        init_crc8();
    uint8_t crc = 0;
    for(int i = 0; i < len; i++){
        char m = data[i];
        crc = crc8_table[(crc) ^ m];
        crc &= 0xFF;
    }
    return crc;
}

uint32_t get_transmission_time(int8_t rssi){
    // quick formula from http://electronics.stackexchange.com/a/83356
    // assume it's free space
    int a = 0; // TODO: calibrate this value
    double raw = (rssi - a) / (-10);
    double distance = pow(10, raw);
    double time = distance / 299792458 * 1000;
    return (uint32_t)time;
}

int* get_rate_config(uint8_t rate, int *values){
    int cr = rate & DQN_CR_MASK;
    int sf = rate & DQN_SF_MASK;
    values[0] = cr >> 4;
    values[1] = sf;
    return values;
}
