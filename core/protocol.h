#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
// include the RF95
#include <RH_RF95.h>
#include <time.h>
#include <stdarg.h>

// define DQN parameters
#define DQN_M 32
#define DQN_N 128 // changed to a small number to make debug easier

// define DQN timing
#define DQN_GUARD 20
#define DQN_MINI_SLOT_LENGTH 150
#define DQN_FEEDBACK 412
#define DQN_ACK_LENGTH 347
#define DQN_MINI_SLOT_FRAME (DQN_M * DQN_MINI_SLOT_LENGTH)
#define DQN_OVERHEAD (DQN_M * DQN_MINI_SLOT_LENGTH + DQN_GUARD * 3 + DQN_FEEDBACK + DQN_ACK_LENGTH)
#define DQN_PREAMBLE 6

// define DQN encodings
#define DQN_SLOW_CRC rf95.Bw500Cr48Sf4096HeaderCRC
#define DQN_FAST_CRC rf95.Bw500Cr45Sf4096HeaderCRC
#define DQN_SLOW_NOCRC rf95.Bw500Cr48Sf4096NoHeaderNoCRC

// device only
#define DQN_IDLE 0
#define DQN_SYNC 1
#define DQN_TRAN 2
#define DQN_CRQ  3
#define DQN_DTQ  4
#define DQN_REQ  5
#define DQN_ADJT 6
#define DQN_SENT 7

// define DQN meta
#define DQN_VERSION 				0x27
#define DQN_MESSAGE_TR				0x80 // need to use mask
#define DQN_MESSAGE_FEEDBACK 		0x01
#define DQN_MESSAGE_TR_JOIN 		0x90 // need to use mask
#define DQN_MESSAGE_JOIN_REQ		0xa0
#define DQN_MESSAGE_JOIN_RESP		0xa1
#define DQN_MESSAGE_MASK			0x0f


// define hardware information
#define HW_ADDR_LENGTH      6

// for testing only
#define DQN_MTU 20
#define DQN_MAX_PACKET (DQN_MTU * 4)

struct dqn_tr{
    uint8_t         version;
    uint8_t         messageid;
    uint16_t        nodeid;  // upstream only. otherwise ignored
    // 1 byte
    uint8_t		    crc;
} __attribute__((packed));  // total is 5 bytes


struct  dqn_feedback{
    uint8_t         version;
    uint8_t         messageid;
    uint32_t        networkid;
    uint32_t        timestamp;
    uint16_t        crq_length;
    uint16_t        dtq_length;
    uint16_t        data_length;
    uint8_t         slots[DQN_M / 4];
    //uint8_t         crc; let the radio to add this
} __attribute__((packed));  // total is 24 bytes

struct dqn_ack{
    uint8_t         version;
    uint8_t         messageid;
    uint8_t         data_acks[DQN_N / 8];
} __attribute__((packed));  // total is 18 bytes

struct dqn_join_req{
    uint8_t         version;
    uint8_t         messageid;
    uint8_t         hw_addr[HW_ADDR_LENGTH];
} __attribute__((packed));  // total is 8 bytes


struct dqn_join_resp{
    uint8_t         version;
    uint8_t         messageid;
    uint8_t         hw_addr[HW_ADDR_LENGTH];
    uint16_t        nodeid;
} __attribute__((packed)); // total is 10 bytes


struct dqn_feedback* make_feedback(
        struct dqn_feedback* feedback,
        uint32_t        networkid,
        uint16_t        crq_length,
        uint16_t        dtq_length,
        uint8_t         *slots);

struct dqn_tr* make_tr(
        struct          dqn_tr* tr,
        uint8_t         num_of_slots,
        bool            high_rate,
        uint16_t        nodeid);

struct dqn_tr* make_tr_join(
        struct          dqn_tr* tr,
        bool            high_rate);

struct dqn_join_req* make_join_req(
        struct dqn_join_req* req,
        uint8_t         *hw_addr);

struct dqn_join_resp* make_join_resp(
        struct dqn_join_resp* resp,
        uint8_t  *hw_addr,
        uint16_t nodeid);


void dqn_send(RH_RF95 *rf95, 
        void* data, 
        size_t size);

uint8_t dqn_recv(RH_RF95 *rf95, 
        uint8_t* buf, 
        uint32_t wait_time, 
        RH_RF95::ModemConfigChoice choice,
        uint32_t received_time);

RH_RF95* setup_radio(RH_RF95 *rf95);

// compute crc8
uint8_t get_crc8(char *data, int len);

// message printing
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO)
void mprint(const char *format, ...);
#endif



#endif
