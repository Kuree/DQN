#define BLOOM_SIZE 64
#include "protocol.h"


#define MESSAGE_MAX 256


// define commands for different send request
// this is the simplest way I can think of to
// bypass C++ restriction on function pointers
#define DQN_SEND_REQUEST_NULL 0
#define DQN_SEND_REQUEST_UP 1
#define DQN_SEND_REQUEST_DOWN 2
#define DQN_SEND_REQUEST_JOIN 3

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


// from https://en.wikipedia.org/wiki/Fletcher%27s_checksum
uint16_t fletcher16( uint8_t const *data, size_t bytes )
{
    uint16_t sum1 = 0xff, sum2 = 0xff;
    size_t tlen;

    while (bytes) {
        tlen = ((bytes >= 20) ? 20 : bytes);
        bytes -= tlen;
        do {
            sum2 += sum1 += *data++;
            tlen--;
        } while (tlen);
        sum1 = (sum1 & 0xff) + (sum1 >> 8);
        sum2 = (sum2 & 0xff) + (sum2 >> 8);
    }
    /* Second reduction step to reduce sums to 8 bits */
    sum1 = (sum1 & 0xff) + (sum1 >> 8);
    sum2 = (sum2 & 0xff) + (sum2 >> 8);
    return (sum2 << 8) | sum1;
}



uint16_t dqn_make_feedback(
        struct dqn_feedback* feedback, 
        uint32_t networkid,
        uint16_t crq_length,
        uint16_t dtq_length,
        uint8_t *slots,
        uint16_t num_of_slots,
        struct bloom *bloom){
    feedback->version = DQN_VERSION;
    feedback->messageid = DQN_MESSAGE_FEEDBACK;
    feedback->timestamp = (uint32_t)time(NULL);
    feedback->networkid = networkid;
    feedback->crq_length = crq_length;
    feedback->dtq_length = dtq_length;
    memcpy(feedback->data, slots, num_of_slots / 4);
    uint8_t * bf_offset = feedback->data + num_of_slots / 4;
    size_t bf_size;
    int entries;
    bloom_dump(bloom, bf_offset, &bf_size, &entries);
    return 16 + num_of_slots / 4 + bf_size;
}

struct dqn_tr* dqn_make_tr(
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


struct dqn_tr* dqn_make_tr_join(
        struct dqn_tr* tr,
        bool high_rate){
    tr->version = DQN_VERSION | (DQN_MESSAGE_MASK & 2) | (high_rate << 2);
    tr->messageid = DQN_MESSAGE_TR_JOIN;
    tr->nodeid = 0; // undefined
    tr->crc = 0;
    uint8_t crc = get_crc8((char*)tr, sizeof(struct dqn_tr));
    tr->crc = crc;
    return tr;
}


struct dqn_tr* dqn_make_join_req(
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


struct dqn_join_req* dqn_make_join_req(
        struct dqn_join_req* req,
        uint8_t *hw_addr){
    req->version = DQN_VERSION;
    req->messageid = DQN_MESSAGE_JOIN_REQ;
    memcpy(req->hw_addr, hw_addr, HW_ADDR_LENGTH);
    return req;
}

struct dqn_join_resp* dqn_make_join_resp(
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
#ifdef ARDUINO
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

void dqn_send(
        RH_RF95 *rf95, 
        const void* data, 
        size_t size,
        RH_RF95::ModemConfigChoice choice){
    rf95->setModemConfig(choice);
    dqn_send(rf95, data, size);
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
#ifdef ARDUINO
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

void RadioDevice::setup(){
#ifdef ARDUINO
    this->rf95 = new RH_RF95(RFM95_CS, RFM95_INT);
#else
    this->rf95 = new RH_RF95(RF95_CS_PIN, RF95_INT_PIN);
#endif
    setup_radio(this->rf95);
}


void RadioDevice::parse_frame_param(struct dqn_feedback *feedback){
    uint16_t frame_param = feedback->frame_param;
    uint8_t fpp = frame_param & 0x3;
    switch(fpp){
        case 0:
            this->bf_error = 0.001;
            break;
        case 1:
            this->bf_error = 0.01;
            break;
        case 2:
            this->bf_error = 0.02;
            break;
        case 3:
            this->bf_error = 0.05;
            break;
        default:
            mprint("error decoding FPP\n");
            this->bf_error = DQN_BF_ERROR;
    }
    uint8_t trf = (frame_param >> 2) & 0x3F;
    this->num_tr = 16 + 8 * trf;
    uint8_t dtr = (frame_param >> 8) & 0xF;
    this->num_data_slot = (uint16_t)floor((double)dtr / 15.0 * (double)(16 + 4 * trf)); 
    uint16_t mpl = (frame_param >> 12) & 0xF;
    this->max_payload = 6 * (mpl + 1);
    this->data_length = this->get_lora_air_time(DQN_FRAME_BW, DQN_FRAME_SF, DQN_PREAMBLE,
            this->max_payload, DQN_FRAME_CRC, DQN_FRAME_FIXED_LEN, DQN_FRAME_CR, DQN_FRAME_LOW_DR);

}

uint16_t RadioDevice::get_frame_param(){
    uint16_t result = 0;
    // FPP
    if(this->bf_error == 0.01)
        result |= 1;
    else if(this->bf_error == 0.02)
        result |= 2;
    else if(this->bf_error == 0.5)
        result |= 3;
    // TRF
    result |= (((this->num_tr - 16) / 8) & 0x3F) << 2;
    //DTR
    double dtr = 15.0 / (double)this->num_tr;
    result |= ((uint8_t)dtr & 0xF) << 8;
    // MPL
    result |= ((this->max_payload / 6 - 1) & 0xF) << 12;

    return result;
}


uint32_t RadioDevice::get_feedback_length(){
    // assume you have everything else set up
    // may need to refactor it
    // this is a dirty and quick implementation
    struct bloom bloom;
    uint8_t buf[255];
    bloom_init_buf(&bloom, this->num_tr, this->bf_error, buf);
    uint8_t slots[255];
    struct dqn_feedback feedback;
    uint32_t feedback_size = dqn_make_feedback(&feedback, 0, 0, 0,
                slots, this->num_tr, &bloom);
    uint32_t feedback_length = this->get_lora_air_time(DQN_FRAME_BW, DQN_FRAME_SF, DQN_PREAMBLE,
                feedback_size, DQN_FRAME_CRC, DQN_FRAME_FIXED_LEN, DQN_FRAME_CR, DQN_FRAME_LOW_DR);
    return feedback_length;
}

uint32_t RadioDevice::get_frame_length(){
    // assume TR length is standard throughout different frame configuration
    uint32_t tr_time = DQN_TR_LENGTH * this->num_tr;
    this->ack_length = this->get_lora_air_time(DQN_FRAME_BW, DQN_FRAME_SF, DQN_PREAMBLE,
            this->num_tr / 8 + 2, DQN_FRAME_CRC, DQN_FRAME_FIXED_LEN, DQN_FRAME_CR, DQN_FRAME_LOW_DR);

    // TODO:
    // refactor this and make it more efficient
    if(!this->feedback_length){
        struct dqn_feedback feedback;
        uint8_t slots[255];
        this->feedback_length = this->get_feedback_length();
    }
    uint32_t total_time = tr_time + DQN_GUARD + this->feedback_length + DQN_GUARD +
        this->data_length * this->num_data_slot + DQN_GUARD + this->ack_length + DQN_GUARD;
    return total_time;
}

uint16_t RadioDevice::get_lora_air_time(uint32_t bw, uint32_t sf, uint32_t pre,
        uint32_t packet_len, bool crc, bool fixed_len, uint32_t cr, bool low_dr){

    double rs = (double)bw / (double)(1 << sf);
    double ts = 1.0 / (double)rs;
    double t_preamble = (pre + 4.25) * ts;

    double tmp = ceil( (double)( 8 * packet_len - 4 * sf + 28 + 16 *crc -
                ( fixed_len ? 20 : 0 ) ) /
            (double)(4 * ( sf - ( ( low_dr > 0 ) ? 2 : 0)))) * (double)( cr + 4 ); 

    double num_payload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
    double t_payload = num_payload * ts;

    double t_on_air = t_preamble + t_payload;
    return (uint16_t)ceil(t_on_air);
}

uint8_t RadioDevice::get_power(uint32_t number){
    // use GCC builtin function
    return 31 - __builtin_clz(number);
}


void RadioDevice::print_frame_info(){
    mprint("---------- DQN information ----------\n");
    mprint("frame length: %d\n", this->frame_length);
    mprint("num of TR: %d\t num of data slots: %d\t\n", this->num_tr, this->num_data_slot); 
    mprint("data length: %d ms\tfeedback length: %d ms\tACK length: %d\n",
            this->data_length, this->feedback_length, this->ack_length);
    mprint("--------------------------------------\n");
}

void Node::ctor(uint8_t *hw_addr){
    this->setup();
    this->nodeid = 0;
    this->has_sync = false;
    this->fast_rate = false;
    this->has_joined = false;

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
            this->parse_frame_param(feedback);
            // compute feedback_length
            this->feedback_length = this->get_lora_air_time(DQN_FRAME_BW, DQN_FRAME_SF, DQN_PREAMBLE,
                    len, DQN_FRAME_CRC, DQN_FRAME_FIXED_LEN, DQN_FRAME_CR, DQN_FRAME_LOW_DR);
            this->time_offset = received_time - this->feedback_length;
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
    this->frame_length = this->get_frame_length();
    uint32_t remain_time = frame_length - (time_diff % this->frame_length);
    this->sleep(remain_time);
}

uint32_t Node::send(){
    return this->send(NULL);
}


void Node::send_request(struct dqn_tr *tr, uint8_t num_of_slots, 
        void (*on_feedback_received)(struct dqn_feedback *), uint8_t send_command){
    while(true) {
        this->check_sync();
        uint32_t frame_start = millis();
        uint16_t chosen_mini_slot = rand() % this->num_tr;
        // send a TR request
        // sleep at the last to ensure the timing 
        this->sleep(frame_start + chosen_mini_slot * DQN_TR_LENGTH - millis());
        dqn_send(this->rf95, (uint8_t*)&tr, sizeof(struct dqn_tr), this->rf95->DQN_SLOW_NOCRC);

        // wait to see the feedback result
        uint32_t total_tr_time = DQN_TR_LENGTH * this->num_tr;
        uint32_t feedback_start_time = frame_start + total_tr_time + DQN_GUARD;
        this->sleep(feedback_start_time - millis());

        // receive feedback.
        uint8_t buf[255];
        uint32_t received_time;
        uint8_t len = dqn_recv(this->rf95, buf, 0, this->rf95->DQN_RATE_FEEDBACK, &received_time);
        struct dqn_feedback *feedback = (struct dqn_feedback*)buf;
        if(feedback->version != DQN_VERSION && feedback->messageid != DQN_MESSAGE_FEEDBACK){
            // somehow it's wrong
            mprint("redeived non-feedback packet\n");
            continue;
        }

        // set the clock and sync
        this->time_offset = received_time - this->feedback_length;
        this->has_sync = true;
        this->last_sync_time = millis();

        // call the feedback_received function
        if(on_feedback_received)
            on_feedback_received(feedback);

        if(send_command)
            return; // we are done

        // scan the TR results
        uint16_t dtq = feedback->dtq_length;
        uint16_t crq = feedback->crq_length;
        for(int i = 0; i < this->num_tr; i++){
            uint8_t status = (feedback->data[i / 4] >> (6 - ( i % 4) * 2)) & 0x3;
            if(i == chosen_mini_slot){
                if(status == 0){
                    break; // not received, trying to send again
                } else if(status == 3 || status != num_of_slots){
                    // there is an contention
                    this->sleep(frame_length * crq);
                    // no need to sleep to next TR frame
                    // check_sync() will handle that 
                } else {
                    uint8_t *bf = feedback->data + this->num_tr / 4;
                    size_t bloom_size = len - 16 - this->num_tr / 4; // TODO: fix magic number 16 here 
                    struct bloom bloom;
                    bloom_load(&bloom, bf, bloom_size, this->num_tr, DQN_BF_ERROR);
                    // test if the node id is in the bloom filter
                    char node_id[10]; // enough for uint_16
                    sprintf(node_id, "%x", this->nodeid); 
                    mprint("node id is %x", node_id);
                    if(bloom_check(&bloom, node_id, strlen(node_id))){
                        // enter DTQ
                        // sleep to the beginning of data slots
                        uint32_t data_start_time = this->time_offset + total_tr_time +
                            DQN_GUARD + this->feedback_length + DQN_GUARD;
                        this->sleep(millis() - data_start_time);
                        // frame counter is used to make sure we won't send data in protocol overhead
                        int frame_counter = 0;
                        for(int i = 0; i < num_of_slots; i++){
                            if(!dtq){
                                uint32_t data_slot_start = millis();
                                switch(send_command){
                                    case DQN_SEND_REQUEST_UP:
                                        this->send_data(i);
                                        break;
                                    case DQN_SEND_REQUEST_DOWN:
                                        this->receive_data(i);
                                        break;
                                    case DQN_SEND_REQUEST_JOIN:
                                        this->join_data(i);
                                        break;
                                }
                            } else {
                                this->sleep(this->data_length);
                                dtq--;
                            }
                            frame_counter++;
                            // avoid the protocol overhead
                            if(frame_counter % this->num_data_slot == this->num_data_slot - 1){
                                uint32_t sleep_time = DQN_GUARD + this->ack_length + DQN_GUARD +
                                    total_tr_time + DQN_GUARD + this->feedback_length + DQN_GUARD;
                                this->sleep(sleep_time);
                            }
                        }
                    } else {
                        // enter CRQ
                        // no need to sleep to the start of the frame
                        // check_sync() will handle that
                        this->sleep(frame_length * crq);
                    }
                }

            } else{
                if(status > 0 and status < 3){
                    dtq += status;
                } else if(status == 3){
                    crq++;
                }
            }

        }
    }

}


void Node::send_data(int index){
    // TODO:
    // based on index load different fragment
    // now use dummy data
    uint32_t data[this->max_payload];
    for(int i = 0; i < this->max_payload; i++)
        data[i] = i % 256;
    dqn_send(this->rf95, data, this->max_payload, this->rf95->DQN_SLOW_CRC);
}


void Node::receive_data(int index){
    // TODO:
    // 1. fix the rate
    // 2. assemble them together
    uint8_t buf[255];
    uint8_t len = dqn_recv(this->rf95, buf, this->data_length, NULL);
}

void Node::join_data(int index){
    if(index == 0) {
        // send join message
        struct dqn_join_req req;
        dqn_make_join_req(&req, this->hw_addr);
        dqn_send(this->rf95, &req, sizeof(struct dqn_join_req), this->rf95->DQN_RATE_FEEDBACK);
    } else if(index == 1) {
        // receive join response
        struct dqn_join_resp resp;
        uint8_t len = dqn_recv(this->rf95, (uint8_t *)&resp, this->data_length, 
                this->rf95->DQN_RATE_FEEDBACK, NULL);
        this->nodeid = resp.nodeid;
        this->has_joined = true;
    }
}

void Node::join(){
    struct dqn_tr tr;
    dqn_make_tr_join(&tr, this->determine_rate());
    this->send_request(&tr, 0, NULL, DQN_SEND_REQUEST_JOIN);
}

uint32_t Node::send(bool *ack){
    uint8_t num_of_slots = 2; // TODO: fix this
    struct dqn_tr tr;
    dqn_make_tr(&tr, num_of_slots, this->determine_rate(), this->nodeid);

    // call the generic send function
    this->send_request(&tr, num_of_slots, NULL, DQN_SEND_REQUEST_UP);

    // see if we need to listen to ack
    // this is on pitfall:
    //      there is a rare cases where sending data will span two frames
    //      need to be extra careful about this one.
    if(ack != NULL){
        // notice that offset set to the beginning of the frame
        // TODO: add ack
    }

    return 0; 
}

void Node::enter_crq(uint32_t sleep_time){
    this->sleep(sleep_time);
    // calibrate to the TR slot
    this->check_sync();    
}

bool Node::determine_rate(){
    // TODO:
    // fix this rate
    return false;
    //int rssi = this->rf95->lastRssi();
    //return rssi >= -10;
}

void Node::sleep(uint32_t time){
    uint32_t start = millis();
    while(millis() < start + time);
    // TODO:
    // switch to more efficient way to put device into sleep
}

Server::Server(uint32_t networkid,
        void (*on_receive)(uint8_t*, size_t, uint8_t*),
        void (*on_download)(uint8_t*, uint8_t*, size_t*)){
    this->networkid = networkid;
    this->on_receive = on_receive;
    this->on_download = on_download;

    this->crq = 0;

    // use the default network configuration
    this->change_network_config(8, DQN_BF_ERROR, 12, 5); 

    this->reset_frame();
}

void Server::send_ack(){
}

void Server::send_feedback(){
    // processing raw TR results to more compacted form
    uint8_t slots[this->num_tr/4];
    // reset to 0
    memset(slots, 0, this->num_tr/4);

    for(int i = 0; i < this->num_tr; i++){
        slots[i/4] |= (this->tr_status[i] & 0x3) << (6 - (i % 4) * 2);
    }
    struct dqn_feedback feedback;
    uint8_t feedback_size = dqn_make_feedback(&feedback, this->networkid, this->crq, this->dtq,
            slots, this->num_tr, &this->bloom);
    // assuming the time is correct
    dqn_send(this->rf95, &feedback, feedback_size, this->rf95->DQN_RATE_FEEDBACK);

    // adjust the crq and dtq
    for(int i = 0; i < this->num_tr; i++){
        if(this->tr_status[i] == 0)
            continue;
        else if(this->tr_status[i] == 3)
            this->crq++;
        else
            this->dtq += this->tr_status[i];
    }

    // decrease dtq and crq
    if(this->crq > 0)
        this->crq--;
    if(this->dtq > this->num_data_slot)
        this->dtq -= this->num_data_slot;
    else
        this->dtq = 0;
    // dtq should match the actual queue length
    if(this->dtq != this->dtqueue.size())
        mprint("queue calculation is wrong. dtq: %d, dtqueue: %d\n", this->dtq, this->dtqueue.size());
}

void Server::reset_frame(){
    // reset the TR status
    for(int i = 0; i < this->num_tr; i++){
        this->tr_status[i] = 0;
    }

    // reset the bloom filter
    bloom_reset(&this->bloom); 
}

void Server::receive_tr(){
    // transmission will be in no header mode
    rf95->setPayloadLength(sizeof(struct dqn_tr));
    uint8_t buf[255];
    for(int i = 0; i < this->num_tr; i++){
        // loop throw each TR slots
        uint32_t received_time;
        dqn_recv(this->rf95, buf, DQN_TR_LENGTH, this->rf95->Bw500Cr48Sf4096NoHeadNoCrc, &received_time);
        // compute the CRC
        struct dqn_tr *tr = (struct dqn_tr*)buf;
        uint8_t crc = tr->crc;
        tr->crc = 0;
        if(crc != get_crc8((char*)tr, sizeof(struct dqn_tr))){
            // there is a collision
            this->tr_status[i] = 3;
        } else {
            if(tr->version != DQN_VERSION){
                mprint("DQN version is not correct. Expect: %d, got %d\n", DQN_VERSION, tr->version);
                continue;
            }
            uint8_t messageid = tr->messageid;
            // TODO:
            // check if node id is valid
            uint16_t nodeid = tr->nodeid;
            if(messageid & DQN_MESSAGE_TR != DQN_MESSAGE_TR){
                mprint("Invalid message id %d\n", messageid);
                continue;
            }
            uint8_t meta = messageid & DQN_MESSAGE_MASK;
            uint8_t num_of_slots = meta & 3;
            this->tr_status[i] = num_of_slots;
            // push this to the dtqueue
#ifdef ARDUINO
            if(this->dtqueue.full()) {
                mprint("Queue is full!\n");
                continue;
            }
#endif
            struct dqn_data_request *request = (struct dqn_data_request*)(this->_tr_data_buf + 
                    this->dtqueue.size() * sizeof(struct dqn_data_request)); // manually calculate the space
            request->messageid = messageid;
            request->nodeid = nodeid;
            this->dtqueue.push(request);
        }

    }
}

void Server::recv_data(){
    // Currently no ALOHA implementaiton
    uint32_t start_time = millis();
    for(int i = 0; i < this->num_data_slot; i++){
        // align the frame
        while(millis() < i * this->data_length + start_time);
        if(this->dtqueue.size()){
            struct dqn_data_request *request = this->dtqueue.front();
            this->dtqueue.pop();
            // decode the message ID
            uint8_t messageid = request->messageid;
            uint16_t nodeid = request->nodeid;
            uint8_t *hw_addr = this->node_table[nodeid];
            uint8_t meta = messageid & DQN_MESSAGE_MASK;
            bool downstream = (meta >> 2) & 1;
            bool high_rate = (meta >> 3) & 1;
            if(downstream) {
                mprint("downstream not implemented\n");
                continue;
            }
            uint8_t num_of_slots = meta & 3;
            uint8_t buf[255];
            uint8_t len = dqn_recv(this->rf95, buf, this->data_length,
                    high_rate? this->rf95->DQN_FAST_CRC:this->rf95->DQN_SLOW_CRC, NULL);
            if(this->on_receive)
                this->on_receive(buf, len, hw_addr);
        }
        else{
            // ALOHA
        }
    }
}


uint16_t Server::register_device(uint8_t *hw_addr){
    // TODO:
    // add cleaning stuff here
    if(this->node_table_invert.count(hw_addr))
        return this->node_table_invert[hw_addr];
    uint16_t nodeid = fletcher16(hw_addr, HW_ADDR_LENGTH);
    while(true){
        if(this->node_table.count(nodeid))
            nodeid++;
    } // ensure it's unique
    this->node_table.insert(std::pair<uint16_t, uint8_t*>(nodeid, hw_addr));
    this->node_table_invert.insert(std::pair<uint8_t *, uint16_t>(hw_addr, nodeid));

    return nodeid;
}

// notice that the bloom filter will be reset as well
void Server::change_network_config(uint8_t trf, double fpp, int dtr, uint8_t mpl){
    // we only allow a set of fpp values
    if(fpp <= 0.001)
        fpp = 0.001;
    else if(fpp <= 0.01)
        fpp = 0.01;
    else if(fpp <= 0.02)
        fpp = 0.02;
    else
        fpp = 0.05;
    this->bf_error = fpp; 

    this->num_tr = 16 + 8 * trf; 

    this->num_data_slot = (uint16_t)floor((double)dtr / 15.0 * this->num_tr);
    this->max_payload = 6 * (mpl + 1);
    this->data_length = this->get_lora_air_time(DQN_FRAME_BW, DQN_FRAME_SF, DQN_PREAMBLE,
            this->max_payload, DQN_FRAME_CRC, DQN_FRAME_FIXED_LEN, DQN_FRAME_CR, DQN_FRAME_LOW_DR);

    //uint8_t ack_size = 2 + this->num_data_slot / 8;
    //this->ack_length = this->get_lora_air_time(DQN_FRAME_BW, DQN_FRAME_SF, DQN_PREAMBLE,
    //        ack_size, DQN_FRAME_CRC, DQN_FRAME_FIXED_LEN, DQN_FRAME_CR, DQN_FRAME_LOW_DR);
    this->frame_length = this->get_frame_length();
    bloom_init_buf(&bloom, this->num_tr, this->bf_error, this->_bloom_buf); 
}

void Server::run(){
    while(true){
        // frame start
        uint32_t frame_start = millis();
        this->receive_tr();
        while(millis() < frame_start + DQN_TR_LENGTH * this->num_tr + DQN_GUARD);
        uint32_t feedback_start = millis();
        // feedback frame
        this->send_feedback();
        while(millis() < feedback_start + this->feedback_length + DQN_GUARD);
        // data time
        this->recv_data();
        // this is ACK time
        uint32_t ack_start = millis();
        this->send_ack();
        while(millis() < ack_start + this->ack_length + DQN_GUARD);

    }
}
