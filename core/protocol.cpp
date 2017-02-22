#define BLOOM_SIZE 64
#include "protocol.h"


#define MESSAGE_MAX 256

// CRC8 implementation is adapted from 
// http://www.rajivchakravorty.com/source-code/uncertainty/multimedia-sim/html/crc8_8c-source.html

#define GP  0x107   /* x^8 + x^2 + x + 1 */
#define DI  0x07

static unsigned char crc8_table[256];     /* 8-bit table */
static int made_table=0;

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


struct dqn_feedback* make_feedback(
        struct dqn_feedback* feedback, 
        uint32_t networkid,
        uint16_t crq_length,
        uint16_t dtq_length,
        uint8_t *slots){
    feedback->version = DQN_VERSION;
    feedback->messageid = DQN_MESSAGE_FEEDBACK;
    feedback->timestamp = (uint32_t)time(NULL);
    feedback->networkid = networkid;
    feedback->crq_length = crq_length;
    feedback->dtq_length = dtq_length;
    memcpy(feedback->slots, slots, DQN_M / 4);
    return feedback;
}

struct dqn_tr* make_tr(
        struct dqn_tr* tr, 
        uint8_t num_of_slots,
        bool high_rate,
        uint16_t nodeid){
    tr->version = DQN_VERSION;
    tr->messageid = DQN_MESSAGE_TR | (DQN_MESSAGE_MASK & num_of_slots) | (high_rate << 2);
    tr->nodeid = nodeid;
    tr->crc = 0;
    uint8_t crc = get_crc8((char*)tr, sizeof(struct dqn_tr));
    tr->crc = crc;
    return tr;
}

struct dqn_tr* make_join_req(
        struct dqn_tr* req,
        bool high_rate){
    req->version = DQN_VERSION;
    req->messageid = DQN_MESSAGE_TR_JOIN | (DQN_MESSAGE_MASK & 2) | (high_rate << 2); 
    req->nodeid = 0;
    req->crc = 0;
    uint8_t crc = get_crc8((char*)req, sizeof(struct dqn_join_req));
    req->crc = crc;
    return req;  
}


struct dqn_join_req* make_join_req(
        struct dqn_join_req* req,
        uint8_t *hw_addr){
    req->version = DQN_VERSION;
    req->messageid = DQN_MESSAGE_JOIN_REQ;
    memcpy(req->hw_addr, hw_addr, HW_ADDR_LENGTH);
    return req;
}

struct dqn_join_resp* make_join_resp(
        struct dqn_join_resp* resp,
        uint8_t  *hw_addr,
        uint16_t nodeid){
    resp->version = DQN_VERSION;
    resp->nodeid = nodeid;
    memcpy(resp->hw_addr, hw_addr, HW_ADDR_LENGTH);
    return resp;
}


void mprint(const char *format, ...){
    va_list args;
    va_start(args, format);
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO)
    char buf[MESSAGE_MAX];
    vsnprintf(buf, MESSAGE_MAX, format, args);
    Serial.print(buf);
#else
    vprintf(format, args);
#endif
}
void dqn_send(RH_RF95 *rf95, const void* data, size_t size){
    if(!rf95->send((uint8_t *)data, sizeof(size))){
        mprint("send failed");
    }
}

uint8_t dqn_recv(
        RH_RF95 *rf95, 
        uint8_t* buf, 
        uint32_t wait_time, 
        RH_RF95::ModemConfigChoice rate,
        uint32_t *received_time){
    // set the config
    rf95->setModemConfig(rate);
    return dqn_recv(rf95, buf, wait_time, received_time);
}

uint8_t dqn_recv(
        RH_RF95 *rf95, 
        uint8_t* buf, 
        uint32_t wait_time, 
        uint32_t *received_time){
    uint8_t len = 0;
    uint32_t start = millis();
    bool indefinite_loop = wait_time == 0;
    while(millis() < start + wait_time || indefinite_loop){
        if(rf95->available()){
            if(received_time != NULL)
                *received_time = millis();
            if (!rf95->recv(buf, &len)){
                mprint("receive failed");
            }
            if(indefinite_loop)
                return len; // end the loop
        }
    }

    return len;
}



uint8_t dqn_recv(
        RH_RF95 *rf95,
        uint8_t* buf,
        uint32_t wait_time){
    return dqn_recv(rf95, buf, wait_time, NULL);
}

RH_RF95* setup_radio(RH_RF95 *rf95){
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO)
    pinMode(VBATPIN, INPUT);

    // un-reset the radio
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    Serial.begin(57600);
    while (!Serial) ; // Wait for serial port to be available (does not boot headless!)
#else
    wiringPiSetup();
    /* Begin Driver Only Init Code */
    pinMode(RF95_RESET_PIN, OUTPUT);
    pinMode(TX_PIN, OUTPUT);
    pinMode(RX_PIN, OUTPUT);
    digitalWrite(TX_PIN, HIGH);
    digitalWrite(RX_PIN, HIGH);

    digitalWrite(RF95_RESET_PIN, HIGH);
    delay(50);
    digitalWrite(RF95_RESET_PIN, LOW);
    delay(50);
    digitalWrite(RF95_RESET_PIN, HIGH);
    delay(50);

    printf("Reset high, waiting 1 sec.\n");
    delay(1000);

    digitalWrite(TX_PIN, LOW);
    digitalWrite(RX_PIN, LOW);
#endif
    if (!rf95->init()){
        mprint("rf95 init failed.\n");
        exit(-95);
    }else{
        mprint("rf95 init success.\n");
    }
    if (!rf95->setFrequency (RF95_FREQ)){
        mprint("rf95 set freq failed.\n");
        exit(-96);
    }else{
        mprint("rf95 set freq to %5.2f.\n", 915.0);
    }

    if (rf95->setModemConfig(rf95->Bw500Cr48Sf4096NoCrc)){
        mprint("rf95 configuration set to BW=500 kHz BW, CR=4/8 CR, SF=12.\n");
    }else{
        mprint("rf95 configuration failed.\n");
        exit(-97);
    }

    // set the preamble
    rf95->setPreambleLength(DQN_PREAMBLE);
    printf("rf95 set preamble to %d\n", DQN_PREAMBLE);

    rf95->setTxPower(23);

    rf95->setPreambleLength(DQN_PREAMBLE);
    mprint("Set premable to %d\n", DQN_PREAMBLE);

    return rf95;
}


void RadioDevice::send(const void* msg, size_t size){
    dqn_send(this->rf95, msg, size);
}

uint8_t RadioDevice::recv(uint32_t wait_time){
    return dqn_recv(this->rf95, this->recv_buf, wait_time);
}

uint8_t RadioDevice::recv(uint32_t wait_time, uint32_t *received_time){
    return dqn_recv(this->rf95, this->recv_buf, wait_time, received_time);
} 

uint8_t RadioDevice::recv(
        uint32_t wait_time,
        RH_RF95::ModemConfigChoice rate,
        uint32_t *received_time){
    return dqn_recv(this->rf95, this->recv_buf, wait_time, rate, received_time);
}

void RadioDevice::setup(){
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO)
    this->rf95 = new RH_RF95(RFM95_CS, RFM95_INT);
#else
    this->rf95 = new RH_RF95(RF95_CS_PIN, RF95_INT_PIN);
#endif
    setup_radio(this->rf95);
}


void RadioDevice::parse_frame_param(struct dqn_feedback *feedback, uint16_t *trf){
    uint16_t frame_param = feedback->frame_param;
    *trf = 2 << (15 - frame_param & 0xF);
    this->data_length = 1 << ((frame_param >> 4) & 0xF);
    uint8_t raw_ratio = (frame_param >> 8) & 0xF;
    float ratio = (float)raw_ratio / 15.0;
    // TODO:
    // finish the implementaion once it's finalized.
}

uint16_t RadioDevice::get_frame_param(){
    uint16_t result = 0;
    uint8_t num_tr = this->get_power(this->trf);
    result |= num_tr & 0xF;
    // TODO:
    // finish the implementaion once it's finalized.
}

uint8_t RadioDevice::get_power(uint32_t number){
    // use GCC builtin function
    return 31 - __builtin_clz(number);
}


void Node::ctor(uint8_t *hw_addr){
    this->setup();
    memcpy(this->hw_addr, hw_addr, HW_ADDR_LENGTH);
}

Node::Node(uint8_t *hw_addr){
    this->ctor(hw_addr);
}

Node::Node(){
   uint8_t hw_addr[HW_ADDR_LENGTH] = {0x42, 0x43, 0x44, 0x45, 0x46, 0x47};
   this->ctor(hw_addr);
} 

void Node::sync(){
    // continuously listening till a valid feedbac is received
    while(true){
        // trying to receive any packet
        uint8_t buf[255];
        uint32_t received_time;
        uint8_t len = dqn_recv(this->rf95, buf, 0, this->rf95->DQN_RATE_FEEDBACK, &received_time);
        // check if it is a valid feedback package
            struct dqn_feedback *feedback = (struct dqn_feedback*)buf;
        if(feedback->version == DQN_VERSION && feedback->messageid == DQN_MESSAGE_FEEDBACK){
            // we find the actual feedback
            // now we need to compute the offset
            // this is not enough!! need to check if the slot size is correct as well
            uint16_t slots_counts = (len - 16) * 4; // TODO: replace it with predefined constant
            uint16_t trf;
            this->parse_frame_param(feedback, &trf);
            if(trf == slots_counts){
                this->time_offset = received_time - DQN_FEEDBACK;
                break;
            }
        }
    }

    this->last_sync_time = millis();
    this->has_sync = true;
}


void Node::check_sync(){
    if(millis() - this->last_sync_time > DQN_SYNC_INTERVAL || !this->has_sync)
        this->sync();
    // determine the starting time for the upcoming frame
    uint32_t time_diff = millis() - this->time_offset;
    // TODO:
    // finish the sleep after the frame structure is finalized.
}

uint32_t Node::send(){
    return this->send(NULL);
}

uint32_t Node::send(bool *ack){
    // TODO switched to queue
    this->check_sync();
    uint16_t chosen_mini_slot = rand() % this->trf;
    this->sleep(chosen_mini_slot * DQN_MINI_SLOT_LENGTH);
    // send a TR request
    
    // see if we need to listen to ack
    if(ack != NULL){
    
    
    }
    
    return 0; 
}

bool Node::determine_rate(){
    
}

void Node::sleep(uint32_t time){
    uint32_t start = millis();
    while(millis() < start + time);
    // TODO:
    // switch to more efficient way to put device into sleep
}

Server::Server(uint32_t networkid,
        void (*on_receive)(uint8_t*, size_t),
        void (*on_download)(uint8_t*, uint8_t*, size_t*)){
    this->networkid = networkid;
    this->on_receive = on_receive;
    this->on_download = on_download;
    
    this->crq = 0;
}

void Server::run(){
    while(true){
    
    }
}
