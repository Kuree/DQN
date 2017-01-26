#define DQN_M 4
#define DQN_N 8
#define DQN_LENGTH 400
#define DQN_MINI_SLOT_LENGTH 200
#define DQN_IDLE 0
#define DQN_SUCCESS 1
#define DQN_CONTEND 2
#define DQN_FB_LENGTH DQN_M * 2 / 8

#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <stdint.h>
#include <stdbool.h>

struct dqn_tr{
    // 2 bytes
    uint16_t        id;
    // 1 byte
    // NOTICE: since we don't want to a device requests 0 data slots
    // everything is added by 1. e.g. if the this attribute is 1, the device is
    // actually requesting for 2 data slots. Hence the range is 1-N
    uint8_t  	    num_slots;
    // 1 byte
    uint8_t		    crc;
} __attribute__((packed));  // total is 4 bytes


struct dqn_data{
    // 30 bytes
    uint8_t     data[30];
} __attribute__((packed));  // total is 30 bytes


struct  dqn_feedback{
    // 2 bytes
    uint16_t        dtq_length;
    // 2 bytes
    uint16_t        crq_length;
    // 8 bytes
    // NOT USED
    // uint64_t        result; // this is bloom filter result
    uint8_t         slots[DQN_FB_LENGTH];
    // TODO: add crc in feedback
} __attribute__((packed));  // total is 12 bytes


/* Bloom filter usage in feedback slot
 * there are three status for a mini slot: idle, requested, contended:
 * for an idle slot, 0 is assigned;
 * for requested slot, the number of data slots in assigned, up to N;
 * for a contended slot, N+1 is assigned.
 * then the status of mini slot is appened to the slot number. To avoid
 * conflicts, mini slots are counted from 1 and padded 0 zeros before status code.
 * For instance, suppose N=16, mini slot 1 idle will be 1000, 
 * mini slot 2 contended will be 20017, and mini slot 3 requested 5 slots will
 * be 3005.
 *
 * After obtain a number, a hash function is applied to the number and hash to a bit position.
 * Thus it follows the Bloom filter
 *
 *
 * Once the device recieves the Bloom filter result, it will reverse the hashing process by trying
 * out all the possibilities to see if they're in the Bloom filter. 
 * For instance, it will try mini slot 1 idle, requested 1 slot, requested 2 slots, etc. 
 * Then it will use this information to reconstruct the original feedback information.
 * The runtime will be O(mN), where m is the number of mini slots and N will be number
 * of data slots.
 */



// this is used to hash an integer value into
// to create a new bloom filter, pass 0 as the hash_value to initialize it
uint64_t bloom_hash(uint64_t hash_value, int value);

bool bloom_test(uint64_t hash_value, int value);

// used by the client
void send_data(char* data);

// used by the server
// may not be used
int receive_data(char* buffer);

// used to get feedback from the base station
// can be used to synchronize the clock
void get_feedback(struct feedback *fb);

uint8_t get_crc8(char *data, int len);
#endif
