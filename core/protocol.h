#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "bloom.h"


// define DQN timing
#define DQN_GUARD 15
#define DQN_TR_LENGTH 150
#define DQN_PREAMBLE 6

// define DQN encodings
// TODO: fix all the rates
#define DQN_RATE_FEEDBACK Bw500Cr48Sf4096
#define DQN_SLOW_CRC Bw500Cr48Sf4096
#define DQN_FAST_CRC Bw500Cr45Sf4096
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
#ifdef ARDUINO
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define VBATPIN A7 

#include <SPI.h>
#include <RH_RF95.h>

#undef max      // Arduino toolchain will report error if standard max macro is around
#undef min
#include <queue.h>
#include <map.h>

using namespace etl;

#else
// raspberry pi
#define RF95_RESET_PIN 0  // this is BCM pin 17, physical pin 11.
#define RF95_INT_PIN 7    // this is BCM pin 4, physical pin 7.
#define RF95_CS_PIN 10    // this is BCM pin 8, physical pin 24
// wiringPi pin numbers
#define TX_PIN 4
#define RX_PIN 5

#include <wiringPi.h>
#include <RH_RF95.h>
#include <queue>
using namespace std;

#include <map>
#endif

#define RF95_FREQ 915.0

// frame config
#define DQN_BF_ERROR 0.01
#define DQN_FRAME_SF 12
#define DQN_FRAME_BW 500
#define DQN_FRAME_CRC true
#define DQN_FRAME_FIXED_LEN false
#define DQN_FRAME_CR 4
#define DQN_FRAME_LOW_DR false

// limitation of arduino-based server
#define DQN_DEVICE_QUEUE_SIZE 255 
#define DQN_SERVER_MAX_TR 256
#define DQN_SERVER_MAX_BLOOM 256
#define DQN_NODE_CAPACITY 256 // may increase this size later
#define DQN_MESSAGE_QUEUE_SIZE 10

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
    uint8_t         data[255 - 16]; // this is a placeholder. actual size needs to be computed 
} __attribute__((packed));  // total is 24 bytes for DQN_M = 32

struct dqn_ack{
    uint8_t         version;
    uint8_t         messageid;
    uint8_t         data_acks[32]; // this is a buf
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
} __attribute__((packed));


struct dqn_node_message{
    uint8_t* data;
    uint8_t size;
} __attribute__((packed));

class SendFunction;
class RadioDevice;
class Node;
class Server;

uint16_t dqn_make_feedback(
        struct dqn_feedback* feedback,
        uint32_t        networkid,
        uint16_t        crq_length,
        uint16_t        dtq_length,
        uint16_t         frame_param,
        uint8_t         *slots,
        uint16_t        num_of_slots,
        struct bloom    *bloom);

struct dqn_tr* dqn_make_tr(
        struct          dqn_tr* tr,
        uint8_t         num_of_slots,
        bool            high_rate,
        uint16_t        nodeid);

struct dqn_tr* dqn_make_tr_join(
        struct          dqn_tr *tr,
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

// for debugging and printing info only
void print_feedback(struct dqn_feedback* feedback);

// defining the base class for both server and device
// a base class wrapper for all DQN methods
class RadioDevice{
    private:
        uint8_t get_power(uint32_t number);
        uint8_t _rf95_buf[sizeof(RH_RF95)];

    protected:
        // how long each data slot is
        uint16_t data_length;
        uint16_t feedback_length;
        uint16_t ack_length;
        uint32_t frame_length;

        uint8_t hw_addr[HW_ADDR_LENGTH];
        uint8_t recv_buf[255];
        uint16_t num_tr;
        uint16_t num_data_slot;
        double bf_error;
        uint16_t max_payload;
        uint8_t _msg_buf[255];

        void parse_frame_param(
                struct dqn_feedback *feedback);
        uint32_t get_feedback_length();

    public:
        RH_RF95 *rf95;
        void setup();
        void set_hw_addr(const uint8_t *hw_addr);
        uint16_t get_frame_param();
        uint16_t get_lora_air_time(uint32_t bw, uint32_t sf, uint32_t pre, 
                uint32_t packet_len, bool crc, bool fixed_len, uint32_t cr, bool low_dr);
        uint32_t get_frame_length();
        void print_frame_info();
};

class Node: public RadioDevice{
    private:
        uint32_t time_offset;
        uint16_t nodeid;
        bool has_sync;
        uint32_t last_sync_time;
        uint32_t base_station_offset; // debugging purpose
        bool fast_rate;
        bool has_joined;

        bool determine_rate();
        void enter_crq(uint32_t);


        // message queue
        uint8_t _queue_buf[DQN_MESSAGE_QUEUE_SIZE * 30];
#ifdef ARDUINO
        queue<struct dqn_node_message, DQN_MESSAGE_QUEUE_SIZE> message_queue;
#else
        queue<struct dqn_node_message> message_queue;
#endif

        // old C++ doesn't have delegating constructors
        // I miss C#
        void ctor(uint8_t *hw_addr);

        // this is for all the communication requests to the base station
        // TODO: refactor the code for sync, send, and join
        void send_request(struct dqn_tr *tr, uint8_t num_of_slots, 
                void (*on_feedback_received)(struct dqn_feedback *), uint8_t send_command);

        void send_data(int index);
        void join_data(int index);
        void receive_data(int index);
    public:
        // this will generate a fixed hardware addresss
        Node();
        Node(uint8_t *hw_addr);
        void sync();
        uint32_t receive_feedback(struct dqn_feedback *feedback);
        uint32_t send();
        uint32_t send(bool *ack);
        void add_data_queue(void *data, size_t size);
        void request_nodeid();
        uint32_t recv();
        void sleep(uint32_t time);
        void join();
        void check_sync();
        bool add_data_to_send(uint8_t * data, uint8_t size);
};

class Server: public RadioDevice{
    private:
        uint32_t networkid;
        uint32_t crq;
        uint32_t dtq;
        struct bloom bloom;
        uint8_t tr_status[DQN_SERVER_MAX_TR];
        uint8_t _bloom_buf[DQN_SERVER_MAX_BLOOM];
        uint8_t _tr_data_buf[sizeof(struct dqn_data_request) * DQN_SERVER_MAX_TR];
        uint8_t _hw_addr_buf[HW_ADDR_LENGTH * DQN_NODE_CAPACITY];


#ifdef ARDUINO
        queue<struct dqn_data_request *, DQN_DEVICE_QUEUE_SIZE> dtqueue;
        etl::map<uint16_t, uint8_t *, DQN_NODE_CAPACITY> node_table;
        etl::map<uint8_t *, uint16_t, DQN_NODE_CAPACITY> node_table_invert;
#else
        queue<struct dqn_data_request *> dtqueue;
        // use double table to reduce the implementation difficulty
        // at the cost of memory space
        // since the buffer is already allocated, it is actually not that bad.
        map<uint16_t, uint8_t*> node_table;
        map<uint8_t *, uint16_t> node_table_invert;
#endif

        // function call backs
        void (*on_receive)(uint8_t *data, size_t size, uint8_t *hw_addr);
        void (*on_download)(uint8_t *hw_addr, uint8_t *data, size_t *size);
        
        void send_feedback();
        void receive_tr();
        void send_ack();
        void reset_frame();
        void recv_data();
        uint16_t register_device(uint8_t *hw_addr);
        void end_cycle();
        void recv_node();
    public:
        Server(uint32_t, 
                void (*on_receive)(uint8_t*, size_t, uint8_t *), 
                void (*on_download)(uint8_t*, uint8_t*, size_t*));
        // this is a blocking method
        void run();

        void change_network_config(uint8_t trf, double fpp, int dtr, uint8_t mpl);
};

#endif
