#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
// include the RF95
#include <RH_RF95.h>
#include <time.h>
#include <stdarg.h>
#include <stdlib.h>

#if (RH_PLATFORM == RH_PLATFORM_ARDUINO)
#undef max      // Arduino toolchain will report error if standard max macro is around
#undef min
#include <queue.h>
using namespace etl;
#else
#include <queue>
using namespace std;
#endif

// define DQN parameters
#define DQN_M 32
#define DQN_N 128 // changed to a small number to make debug easier

// define DQN timing
#define DQN_GUARD 15
#define DQN_MINI_SLOT_LENGTH 150
#define DQN_FEEDBACK 412
#define DQN_ACK_LENGTH 347
#define DQN_MINI_SLOT_FRAME (DQN_M * DQN_MINI_SLOT_LENGTH)
#define DQN_OVERHEAD (DQN_M * DQN_MINI_SLOT_LENGTH + DQN_GUARD * 3 + DQN_FEEDBACK + DQN_ACK_LENGTH)
#define DQN_PREAMBLE 6

// define DQN encodings
#define DQN_RATE_FEEDBACK Bw500Cr48Sf4096
#define DQN_FAST_CRC rf95->Bw500Cr45Sf4096HeaderCRC
#define DQN_SLOW_NOCRC Bw500Cr48Sf4096NoHeadNoCrc

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
#define DQN_VERSION                 0x27
#define DQN_MESSAGE_TR              0x80 // need to use mask
#define DQN_MESSAGE_FEEDBACK        0x01
#define DQN_MESSAGE_TR_JOIN         0x90 // need to use mask
#define DQN_MESSAGE_JOIN_REQ        0xa0
#define DQN_MESSAGE_JOIN_RESP       0xa1
#define DQN_MESSAGE_MASK            0x0f

// define sync
#define DQN_SYNC_INTERVAL           36000000    // 10 hours

// define hardware information
#define HW_ADDR_LENGTH      6

// radio configuration
// arduino
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO)
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define VBATPIN A7 
#endif

// raspberry pi
#define RF95_RESET_PIN 0  // this is BCM pin 17, physical pin 11.
#define RF95_INT_PIN 7    // this is BCM pin 4, physical pin 7.
#define RF95_CS_PIN 10    // this is BCM pin 8, physical pin 24
// wiringPi pin numbers
#define TX_PIN 4
#define RX_PIN 5


// limitation of arduino-based server
#define DQN_DEVICE_QUEUE_SIZE 255 

// for testing only
#define DQN_MTU 20
#define DQN_MAX_PACKET (DQN_MTU * 4)

struct dqn_tr{
    uint8_t         version;
    uint8_t         messageid;
    uint16_t        nodeid;  // upstream only. otherwise ignored
    // 1 byte
    uint8_t         crc;
} __attribute__((packed));  // total is 5 bytes


struct  dqn_feedback{
    uint8_t         version;        // 1
    uint8_t         messageid;      // 1
    uint32_t        networkid;      // 4
    uint32_t        timestamp;      // 4
    uint16_t        crq_length;     // 2
    uint16_t        dtq_length;     // 2
    uint16_t        frame_param;    // 2
    uint8_t         slots[DQN_M / 4]; // this is not accurate for node 
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



struct dqn_data_request{
    // this is only used by server
    // assume that it is pretty good memory management
    //
    uint8_t messageid;
    uint16_t nodeid;
};

struct dqn_feedback* dqn_make_feedback(
        struct dqn_feedback* feedback,
        uint32_t        networkid,
        uint16_t        crq_length,
        uint16_t        dtq_length,
        uint8_t         *slots);

struct dqn_tr* dqn_make_tr(
        struct          dqn_tr* tr,
        uint8_t         num_of_slots,
        bool            high_rate,
        uint16_t        nodeid);

struct dqn_tr* dqn_make_tr_join(
        struct          dqn_tr* tr,
        bool            high_rate);

struct dqn_join_req* dqn_make_join_req(
        struct dqn_join_req* req,
        uint8_t         *hw_addr);

struct dqn_join_resp* dqn_make_join_resp(
        struct dqn_join_resp* resp,
        uint8_t  *hw_addr,
        uint16_t nodeid);


void dqn_send(
        RH_RF95 *rf95, 
        const void* data, 
        size_t size);

void dqn_send(
        RH_RF95 *rf95,
        const void* data,
        size_t size,
        RH_RF95::ModemConfigChoice choice);

uint8_t dqn_recv(
        RH_RF95 *rf95, 
        uint8_t* buf, 
        uint32_t wait_time, 
        RH_RF95::ModemConfigChoice choice,
        uint32_t *received_time);

uint8_t dqn_recv(                        
         RH_RF95 *rf95,                   
         uint8_t* buf,                    
         uint32_t wait_time,              
         uint32_t *received_time);

uint8_t dqn_recv(  
        RH_RF95 *rf95,
        uint8_t* buf,
        uint32_t wait_time);


RH_RF95* setup_radio(RH_RF95 *rf95);

// compute crc8
uint8_t get_crc8(char *data, int len);

// message printing
void mprint(const char *format, ...);



// defining the base class for both server and device
// a base class wrapper for all DQN methods
class RadioDevice{
    private:
        uint8_t get_power(uint32_t number);
    protected:
        // how long each data slot is
        uint16_t data_length;
        RH_RF95 *rf95;
        uint8_t hw_addr[HW_ADDR_LENGTH];
        uint8_t recv_buf[255];
        uint16_t trf;
        uint16_t num_data_slot;

        void parse_frame_param(
                struct dqn_feedback *feedback,
                uint16_t *trf);
    public:
        void setup();
        void set_hw_addr(const uint8_t *hw_addr);
        uint16_t get_frame_param();
};


class Node: public RadioDevice{
    private:
        uint32_t time_offset;
        uint16_t nodeid = 0;
        bool has_sync = false;
        uint32_t last_sync_time;
        bool fast_rate = false;
        bool has_joined = false;

        void sync();
        void check_sync();
        bool determine_rate();

        // old C++ doesn't have delegating constructors
        // I miss C#
        void ctor(uint8_t *hw_addr);
    public:
        // this will generate a fixed hardware addresss
        Node();
        Node(uint8_t *hw_addr);
        uint32_t receive_feedback(struct dqn_feedback *feedback);
        uint32_t send();
        uint32_t send(bool *ack);
        void add_data_queue(void *data, size_t size);
        void request_nodeid();
        uint32_t recv();
        void sleep(uint32_t time);
        bool join();
};

class Server: public RadioDevice{
    private:
        uint32_t networkid;
        uint32_t crq;
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO)
        queue<dqn_data_request, DQN_DEVICE_QUEUE_SIZE> dtq;
#else
        queue<dqn_data_request> dtq;
#endif

        // function call backs
        void (*on_receive)(uint8_t *data, size_t size);
        void (*on_download)(uint8_t *hw_addr, uint8_t *data, size_t *size);
        
        void send_feedback();

    public:
        Server(uint32_t, 
                void (*on_receive)(uint8_t*, size_t), 
                void (*on_download)(uint8_t*, uint8_t*, size_t*));
        // this is a blocking method
        void run();
};

#endif
