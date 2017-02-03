#ifndef PROTOCOL_H
#define PROTOCOL_H

// define radio config
#define LORA_HEADER 4

#define DQN_M 4
#define DQN_N 4 // changed to a small number to make debug easier
#define DQN_LENGTH 400
#define DQN_MINI_SLOT_LENGTH 200
#define DQN_MINI_SLOT_FRAME (DQN_M * DQN_MINI_SLOT_LENGTH)
#define DQN_OVERHEAD (1 + DQN_MINI_SLOT_FRAME / DQN_LENGTH)
#define DQN_PREAMBLE 6
#define DQN_AVAILABLE_RATES 2
#define DQN_RATE_0 256
#define DQN_RATE_1 20524 

#define FEEDBACK_TIME 280
#define TR_TIME 215 

#define DQN_RECV_WINDOW 12 // 12ms delay time. measured by echo program
#define DQN_GUARD 15 // 15ms guard time this is used for frequency changing
// device only
#define DQN_IDLE 0
#define DQN_SYNC 1
#define DQN_TRAN 2
#define DQN_CRQ  3
#define DQN_DTQ  4
#define DQN_REQ  5
#define DQN_ADJT 6

#define DQN_MTU (DQN_RATE_1 * DQN_LENGTH / 8000)
#define DQN_MAX_PACKET (DQN_MTU * DQN_N)

#include <stdint.h>
#include <stdbool.h>


struct dqn_tr{
    // 1 byte
    // NOTICE: we don't want to a device requests 0 data slots
    uint8_t  	    num_slots;
    // this defines which rate the device wants to transmit
    uint8_t         rate; 
    // 1 byte
    uint8_t		    crc;
} __attribute__((packed));  // total is 3 bytes


// UNUSED
struct dqn_data{
    // 30 bytes
    uint8_t     data[30];
} __attribute__((packed));  // total is 30 bytes


struct  dqn_feedback{
    // 2 bytes
    uint16_t        dtq_length;
    // 2 bytes
    uint16_t        crq_length;
    uint8_t         slots[DQN_M];
    uint8_t         crc;
} __attribute__((packed));  // total is 5  + DQN_M bytes


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

// compute crc8
uint8_t get_crc8(char *data, int len);

// returns in milli seconds
uint32_t get_transmission_time(int8_t rssi);
#endif
