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
        uint16_t  frame_param,
        uint8_t *slots,
        uint16_t num_of_slots,
        struct bloom *bloom){
    feedback->version = DQN_VERSION;
    feedback->messageid = DQN_MESSAGE_FEEDBACK;
    feedback->timestamp = millis(); //(uint32_t)time(NULL);
    feedback->networkid = networkid;
    feedback->crq_length = crq_length;
    feedback->dtq_length = dtq_length;
    feedback->frame_param = frame_param;
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

struct dqn_tr* dqn_make_tr_down(
        struct dqn_tr* tr,
        uint8_t num_of_slots,
        bool high_rate,
        uint16_t nodeid){
    tr->version = DQN_VERSION;
    tr->messageid = DQN_MESSAGE_TR | (DQN_MESSAGE_MASK & num_of_slots) | DQN_MESSAGE_DOWNSTREAM | (high_rate << 2);
    tr->nodeid = nodeid;
    tr->crc = 0;
    uint8_t crc = get_crc8((char*)tr, sizeof(struct dqn_tr));
    tr->crc = crc;
    return tr;
}


struct dqn_tr* dqn_make_tr_join(
        struct dqn_tr* req,
        bool high_rate){
    req->version = DQN_VERSION;
    req->messageid = DQN_MESSAGE_TR_JOIN | (DQN_MESSAGE_MASK & 2) | (high_rate << 2);
    req->nodeid = 0; // undefinedg
    req->crc = 0;
    uint8_t crc = get_crc8((char*)req, sizeof(struct dqn_tr));
    req->crc = crc;
    return req;
}


struct dqn_join_req* dqn_make_join_req(
        struct dqn_join_req* req,
        uint8_t *hw_addr){
    req->version = DQN_VERSION;
    req->messageid = DQN_MESSAGE_JOIN_REQ;
    memcpy(req->hw_addr, hw_addr, HW_ADDR_LENGTH);
    mprint("using addr: %X:%X:%X:%X:%X:%X\n", req->hw_addr[0], req->hw_addr[1], 
            req->hw_addr[2], req->hw_addr[3], req->hw_addr[4], req->hw_addr[5]);
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

    if(!rf95->send((uint8_t *)data, size)){
        mprint("send failed");
    }
}
//
// void dqn_send(
//         RH_RF95 *rf95,
//         const void* data,
//         size_t size,
//         RH_RF95::ModemConfigChoice choice){
//     rf95->setModemConfig(choice);
//     dqn_send(rf95, data, size);
// }

// uint8_t dqn_recv(
//         RH_RF95 *rf95,
//         uint8_t* buf,
//         uint32_t wait_time,
//         RH_RF95::ModemConfigChoice rate,
//         uint32_t *received_time){
//     // set the config
//     rf95->setModemConfig(rate);
//     return dqn_recv(rf95, buf, wait_time, received_time);
// }

uint8_t dqn_recv(
        RH_RF95 *rf95,
        uint8_t* buf,
        uint32_t wait_time,
        uint32_t *received_time){
    uint32_t start = millis();
    bool indefinite_loop = wait_time == 0;
    while(millis() < start + wait_time || indefinite_loop){
        if(rf95->available()){
            uint8_t len = RH_RF95_MAX_MESSAGE_LEN;
            if(rf95->recv(buf, &len)){
                if(received_time)
                    *received_time = millis();
                return len;
            }
        }
    }
    return 0;
}


uint8_t dqn_recv(
        RH_RF95 *rf95,
        uint8_t* buf,
        uint32_t wait_time){
    return dqn_recv(rf95, buf, wait_time, NULL);
}


// debugging functions
// will be removed later
void print_byte(uint8_t byte){
    for(int i = 0; i < 8; i++){
        const char *c = (!(byte & (1 << (7 - i))))? "0" : "1";
        mprint(c);
    }
    mprint(" ");
}

uint16_t get_tr(uint16_t frame_param){
    uint16_t trf = (frame_param >> 2) & 0x3F;
    return 16 + 8 * trf;
}

void print_ack(uint8_t *ack, size_t size){
    mprint("------------ACK---------------\n");
    for(uint32_t i = 0; i < size; i++){
        print_byte(ack[i]);
        if(i % 4 == 3)
            mprint("\n");
    }
    mprint("------------------------------\n");
}

void print_feedback(struct dqn_feedback* feedback, int8_t rssi){
    mprint("------------FEEDBACK-----------\n");
    mprint("frame param: \t0x%X CRQ: %d DTQ: %d RSSI: %d dBm\n",
            feedback->frame_param,
            feedback->crq_length,
            feedback->dtq_length,
            rssi);
    mprint("timestamp:   %d\t", feedback->timestamp);
    mprint("TR result:   \t");
    for(int i = 0; i < get_tr(feedback->frame_param) / 4; i++) {
        print_byte(feedback->data[i]);
        if(i % 4 == 3)
            mprint("\n");
    }
    //mprint("\n");
    mprint("-------------------------------\n");
}

RadioDevice::RadioDevice(struct RH_RF95::pin_config pc, float freq):
  freq(freq)
{
  this->rf95 = new (this->_rf95_buf)RH_RF95(pc);

  if (!rf95->init()){
      mprint("rf95 init failed.\n");
      exit(-95);
  }else{
      mprint("rf95 init success.\n");
  }
  if (!rf95->setFrequency (freq)){
      mprint("rf95 set freq failed.\n");
      exit(-96);
  }else{
      mprint("rf95 set freq to %5.2f.\n", freq);
  }

  if (configureModem(TR)){
      mprint("rf95 configuration set.\n");
  }else{
      mprint("rf95 configuration failed.\n");
      exit(-97);
  }

  // set the preamble
  rf95->setPreambleLength(DQN_PREAMBLE);
  mprint("rf95 set preamble to %d\n", DQN_PREAMBLE);

  rf95->setFhssHoppingPeriod(0);
  //rf95->setFrequency(freq);
  rf95->setTxPower(13);
}
bool RadioDevice::configureModem(DqnModemMode mode){

  switch(mode){
    case TR:
      return rf95->setModemConfig(
          DQN_FRAME_BW,
          DQN_FRAME_CR,
          DQN_FRAME_FIXED_LEN,
          DQN_FRAME_SF,
          DQN_FRAME_NOCRC,
          DQN_FRAME_LOW_DR);
    break;

    case Feedback :
    case Ack :
    case Data :
      return rf95->setModemConfig(
            DQN_FRAME_BW,
            DQN_FRAME_CR,
            DQN_FRAME_VARIABLE_LEN,
            DQN_FRAME_SF,
            DQN_FRAME_CRC,
            DQN_FRAME_LOW_DR);
    break;

    default:
      mprint("!!!INVALID MODE!!!\n");
  }
  return false;
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
    this->num_data_slot = (uint16_t)floor((double)dtr / 15.0 * (double)this->num_tr);
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
    double dtr = 15.0 / (double)this->num_tr * (double)this->num_data_slot;
    result |= ((uint8_t)ceil(dtr) & 0xF) << 8;
    // MPL
    result |= ((this->max_payload / 6 - 1) & 0xF) << 12;

    return result;
}


bool RadioDevice::is_receiving(){
    return this->rf95->mode() == RHGenericDriver::RHModeTx;
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
    uint32_t feedback_size = dqn_make_feedback(&feedback, 0, 0, 0, 0,
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
        this->feedback_length = this->get_feedback_length();
    }
    uint32_t total_time = tr_time + DQN_GUARD + this->feedback_length + DQN_GUARD +
        this->data_length * this->num_data_slot + DQN_GUARD + this->ack_length + DQN_GUARD +
        (this->num_tr + this->num_data_slot - 2) * DQN_SHORT_GUARD;
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

void RadioDevice::print_frame_info(){
    mprint("---------- DQN information ----------\n");
    mprint("frame length: %d\n", this->frame_length);
    mprint("num of TR: %d\t num of data slots: %d\t\n", this->num_tr, this->num_data_slot);
    mprint("data length: %d ms\tfeedback length: %d ms\tACK length: %d\n",
            this->data_length, this->feedback_length, this->ack_length);
    mprint("--------------------------------------\n");
}

void Node::ctor(struct RH_RF95::pin_config pc, uint8_t *hw_addr)
{
    this->nodeid = 0;
    this->has_sync = false;
    this->has_joined = false;

    this->set_hw_addr(hw_addr);
}

Node::Node(struct RH_RF95::pin_config pc, float freq, uint8_t *hw_addr)
:RadioDevice(pc, freq)
{
    this->ctor(pc, hw_addr);
}

Node::Node(struct RH_RF95::pin_config pc, float freq)
:RadioDevice(pc, freq)
{
    uint8_t hw_addr[HW_ADDR_LENGTH] = {0x42, 0x43, 0x44, 0x45, 0x46, 0x47};
    this->ctor(pc, hw_addr);
}

void Node::set_hw_addr(uint8_t *hw_addr){
    memcpy(this->hw_addr, hw_addr, HW_ADDR_LENGTH);
}

void Node::sync(){
    // continuously listening till a valid feedback is received
    // switch mode
    configureModem(Feedback);
    while(true){
        // trying to receive any packet
        mprint("waiting to recv feedback.\n");
        uint32_t received_time;

        uint8_t len = dqn_recv(this->rf95, this->_msg_buf, 0, &received_time);
        struct dqn_feedback *feedback = (struct dqn_feedback*)this->_msg_buf;
        if(feedback->version == DQN_VERSION && feedback->messageid == DQN_MESSAGE_FEEDBACK){
            // we find the actual feedback
            // now we need to compute the offset
            this->parse_frame_param(feedback);
            // compute feedback_length
            this->feedback_length = this->get_lora_air_time(DQN_FRAME_BW, DQN_FRAME_SF, DQN_PREAMBLE,
                    len, DQN_FRAME_CRC, DQN_FRAME_FIXED_LEN, DQN_FRAME_CR, DQN_FRAME_LOW_DR);
            this->frame_length = this->get_frame_length();
            this->time_offset = received_time - this->feedback_length - (DQN_TR_LENGTH + DQN_SHORT_GUARD) * this->num_tr
                + DQN_SHORT_GUARD - DQN_GUARD - 15; // this is a maigc fix
            uint32_t timestamp = feedback->timestamp;
            // ------ DEBUGING PURPOSE------
            // compute the actual frame start time
            uint32_t base_station_time = (timestamp - (DQN_GUARD + DQN_TR_LENGTH * this->num_tr));
            uint32_t base_station_offset = this->time_offset - base_station_time;
            mprint("frame offset = %d base staiton time %d\n", base_station_offset, base_station_time);
            // -------- END ----------------
            this->last_sync_time = millis();
            this->has_sync = true;
            this->retry_count = 0;
            this->print_frame_info();
            break;
        }else{
          mprint("received unkown message.\n");
        }
    }

}


void Node::check_sync(){
    // returns at the start of the next frame.
    if(millis() - this->last_sync_time > DQN_SYNC_INTERVAL || !this->has_sync || this->retry_count >= DQN_SYNC_RETRY){
        mprint("need to resync.\n");
        this->sync();
      }

    // determine the starting time for the upcoming frame
    uint32_t time_diff = millis() - this->time_offset;
    if(this->frame_length == 0)
        mprint("ERROR!\n");
    uint32_t remain_time = this->frame_length - (time_diff % this->frame_length);
    this->sleep(remain_time);
}

uint32_t Node::send(){
    return this->send(NULL);
}


uint16_t Node::send_request(struct dqn_tr *tr, uint8_t num_of_slots,
        void (*on_feedback_received)(struct dqn_feedback *), uint8_t send_command){
    while(true) {
        this->check_sync();
        mprint("starting to send TR at time %d with offset %d...\n", millis(), this->time_offset);
        uint32_t frame_start = millis();
        uint16_t chosen_mini_slot = rand() % this->num_tr;
        mprint("choosen at slot %d tr messageid: %X\n", chosen_mini_slot, tr->messageid);
        // send a TR request
        // sleep at the last to ensure the timing
        this->sleep(frame_start + chosen_mini_slot * (DQN_TR_LENGTH + DQN_SHORT_GUARD) - millis());

        // switch to TR mode
        configureModem(TR);
        this->rf95->setPayloadLength(sizeof(struct dqn_tr));
        dqn_send(this->rf95, tr, sizeof(struct dqn_tr)); //, this->rf95->DQN_SLOW_NOCRC);

        // wait to see the feedback result
        uint32_t total_tr_time = (DQN_TR_LENGTH + DQN_SHORT_GUARD) * this->num_tr - DQN_SHORT_GUARD;
        uint32_t feedback_start_time = frame_start + total_tr_time + DQN_GUARD;

#ifdef DEBUG
        mprint("feedback length is %d\n", feedback_length);
        mprint("feedback start is %d\n", feedback_start_time);
        mprint("feedback end   is %d\n", feedback_start_time + this->feedback_length);
        mprint("now it is         %d\n", millis());
        mprint("sleeping %d\n", feedback_start_time - millis());
#endif
        this->sleep(feedback_start_time - millis());
        configureModem(Feedback);

        // receive feedback.
        mprint("Wakeup to recv feedback, now its: %d\n", millis());
        uint32_t received_time;
        uint8_t len = dqn_recv(this->rf95,
          this->_msg_buf, this->feedback_length + DQN_GUARD, &received_time);
        struct dqn_feedback *feedback = (struct dqn_feedback*)this->_msg_buf;
        if(feedback->version != DQN_VERSION || feedback->messageid != DQN_MESSAGE_FEEDBACK){
            // somehow it's wrong
            mprint("redeived non-feedback packet\n");
            mprint("len: %d, version: %d messageid: %d\n", len, feedback->version, feedback->messageid);
            this->retry_count++;
            continue;
        }
        if(len == 0){
            mprint("%d\tno feedback received\n",millis());
            this->retry_count++;
            continue;
        }

        // set the clock and sync
        // this is a magic fix
        this->time_offset = received_time - this->feedback_length - total_tr_time - DQN_GUARD - 15;
        this->has_sync = true;
        this->last_sync_time = millis();

        // call the feedback_received function
        if(on_feedback_received)
            on_feedback_received(feedback);

        if(!send_command)
            return 0; // we are done

        // scan the TR results
        uint16_t dtq = feedback->dtq_length;
        uint16_t crq = feedback->crq_length;
        for(int i = 0; i < this->num_tr; i++){
            uint8_t status = (feedback->data[i / 4] >> (6 - ( i % 4) * 2)) & 0x3;
            if(i == chosen_mini_slot){
                if(status == 0){
                    mprint("TR not received by base station. retrying...\n");
                    this->retry_count++;
                    break; // not received, trying to send again
                } else if(status == 3 || status != num_of_slots){
                    mprint("contention detected at the chosen slot %d, status %d\n", chosen_mini_slot, status);
                    // there is an contention
                    this->sleep(frame_length * crq);
                    // no need to sleep to next TR frame
                    // check_sync() will handle that
                } else {
                    //uint32_t test_start_time = millis();
                    uint8_t *bf = feedback->data + this->num_tr / 4;
                    int bloom_size = len - 16 - this->num_tr / 4; // TODO: fix magic number 16 here
                    struct bloom bloom;
                    uint16_t dtq_copy = dtq;
                    bloom_load(&bloom, bf, this->num_tr, DQN_BF_ERROR);
                    if(bloom.bytes != bloom_size) {
                        mprint("\t something went wrong: bloom filter size does not match\n");
                        while(1);
                    }
                    // test if the node id is in the bloom filter
                    char node_id[10]; // enough for uint_16
                    sprintf(node_id, "%x", this->nodeid);
                    if(bloom_check(&bloom, node_id, strlen(node_id)) || (this->nodeid == 0 && send_command == DQN_SEND_REQUEST_JOIN)){
                        mprint("device enter DTQ with dtq: %d\n", dtq);
                        // enter DTQ
                        // sleep to the beginning of data slots
                        uint32_t data_start_time = frame_start + total_tr_time +
                            DQN_GUARD + this->feedback_length + DQN_GUARD;
                        this->sleep(data_start_time - millis());
                        // frame counter is used to make sure we won't send data in protocol overhead
                        int frame_counter = 0;
                        int j = 0;
                        while(j < num_of_slots){
                            if(!dtq){
                                uint32_t data_slot_start = millis();
                                switch(send_command){
                                    case DQN_SEND_REQUEST_UP:
                                        mprint("\tsending data up...\n");
                                        this->send_data(j);
                                        break;
                                    case DQN_SEND_REQUEST_DOWN:
                                        this->receive_data(j);
                                        break;
                                    case DQN_SEND_REQUEST_JOIN:
                                        this->join_data(j);
                                        break;
                                }
                                j++;
                                this->sleep((data_slot_start + this->data_length + DQN_SHORT_GUARD) - millis());
                            } else {
                                this->sleep(this->data_length + DQN_SHORT_GUARD);
                                dtq--;
                            }
                            frame_counter++;
                            // avoid the protocol overhead
                            if(frame_counter % this->num_data_slot == this->num_data_slot - 1){
                                uint32_t sleep_time = DQN_GUARD - DQN_SHORT_GUARD + this->ack_length + DQN_GUARD +
                                    total_tr_time + DQN_GUARD + this->feedback_length + DQN_GUARD;
                                mprint("skipped frame overhead\n");
                                this->sleep(sleep_time);
                            }
                        }
                        // we are done here
                        this->has_sync = false; // force to resync TODO: remove this one
                    } else {
                        // enter CRQ
                        // no need to sleep to the start of the frame
                        // check_sync() will handle that
                        mprint("device enter CRQ\n");
                        this->sleep(this->frame_length * crq); // next time it will aligned into frame automically
                    }
                    // we finally finished!
                    // return the starting data slot number
                    return dtq_copy % this->num_data_slot;
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
    if(!this->message_queue.size()){
        mprint("No data to send. Message queue is wrong\n");
        while(1);
    }
    struct dqn_node_message message = this->message_queue.front();
    uint8_t * data = message.data;
    uint8_t size = message.size;
    if(index == 1) {
        this->message_queue.pop();
        data += this->max_payload;
        size -= this->max_payload;
    } else{
        size = size > this->max_payload? this->max_payload : size;
        if (size < this->max_payload)
            this->message_queue.pop();
    }
    configureModem(Data);
    dqn_send(this->rf95, data, size);
}


void Node::receive_data(int index){
    // TODO:
    // 1. fix the rate
    // always request 2 data slots
    if(index == 0){
        uint8_t len = dqn_recv(this->rf95, this->_msg_buf, this->data_length + DQN_SHORT_GUARD, NULL);
        mprint("received len: %d\n", len);
    } else {
        // assume the first slot is always full
        uint8_t len = dqn_recv(this->rf95, this->_msg_buf + this->max_payload,
                this->data_length + DQN_SHORT_GUARD, NULL);
        mprint("received len: %d\n", len);
        len += this->max_payload;
        if(this->on_receive)
            this->on_receive(this->_msg_buf, len);
    }
}

void Node::set_on_receive(void (*on_receive)(uint8_t*, uint8_t)){
    this->on_receive = on_receive;
}

void Node::join_data(int index){
    if(index == 0) {
        // send join message
        mprint("send joining message\n");
        struct dqn_join_req req;
        dqn_make_join_req(&req, this->hw_addr);
        mprint("sending join data at %d\n", millis());
        dqn_send(this->rf95, &req, sizeof(struct dqn_join_req));
    } else if(index == 1) {
        // receive join response
        mprint("receiving joining data at %d\n", millis());
        uint8_t len = dqn_recv(this->rf95, this->_msg_buf, this->data_length + DQN_SHORT_GUARD, NULL);
        if(len == 0){
            mprint("could not receive joining information\n");
            return;
        }
        struct dqn_join_resp *resp = (struct dqn_join_resp*)this->_msg_buf;
        if(resp->version != DQN_VERSION) {
            mprint("wrong DQ-N version number. Got %d, expected: %d\n", resp->version, DQN_VERSION);
            while(1);
        }
        if(resp->nodeid == 0) {
            mprint("wrong node id. node id cannot be 0\n");
            while(1);
        }
        this->nodeid = resp->nodeid;
        mprint("node id set to %d\n", this->nodeid);
        this->has_joined = true;
    }
}

void Node::join(){
    struct dqn_tr tr;
    dqn_make_tr_join(&tr, this->determine_rate());
    this->send_request(&tr, 2, NULL, DQN_SEND_REQUEST_JOIN);
}

void Node::recv(){
    struct dqn_tr tr;
    // TODO: fix this
    dqn_make_tr_down(&tr, 2, this->determine_rate(), this->nodeid);
    this->send_request(&tr, 2, NULL, DQN_SEND_REQUEST_DOWN);
}

uint32_t Node::send(bool *ack){
    // peak the queue
    if(!this->message_queue.size())
        return 0;
    if(!this->has_joined)
        this->join();
    struct dqn_node_message message = this->message_queue.front();
    uint8_t size = message.size;
    uint8_t num_of_slots = size / this->max_payload;
    if(size % this->max_payload != 0)
        num_of_slots++;
    struct dqn_tr tr;
    dqn_make_tr(&tr, num_of_slots, this->determine_rate(), this->nodeid);

    // call the generic send function
    uint16_t slot_number = this->send_request(&tr, num_of_slots, NULL, DQN_SEND_REQUEST_UP);

    // see if we need to listen to ack
    // TODO: this is on pitfall:
    //      there is a rare cases where sending data will span two frames
    //      need to be extra careful about this one.
    if(ack != NULL){
        *ack = false;
        // need to set offset to the beginning of the ACK frame
        uint16_t ack_relative_time = this->frame_length - DQN_GUARD - this->ack_length;
        configureModem(Ack);
        uint16_t current_time = (millis() - this->time_offset) % this->frame_length;
        uint16_t sleep_time = ack_relative_time - current_time;
        mprint("sleep for ack: %d\n", sleep_time);
        this->sleep(sleep_time);
        uint8_t len = dqn_recv(this->rf95, this->_msg_buf, this->ack_length + DQN_GUARD);
        size_t expected = this->num_data_slot / 8;
        if(this->num_data_slot % 8)
            expected++;
        if(len != expected){
            mprint("error in receiving ack. expected size: %d received: %d\n", expected, len);
        } else {
            *ack = (this->_msg_buf[slot_number / 8] >> (slot_number % 8)) & 1;
            if(num_of_slots == 2){
                *ack &= (this->_msg_buf[(slot_number + 1) / 8] >> ((slot_number + 1) % 8)) & 1;
            }
        }
    }

    return size;
}

bool Node::add_data_to_send(uint8_t *data, uint8_t size){
    if(size > 2 * this->max_payload)
        return false;
#ifdef ARDUINO
    if(this->message_queue.full())
        return false;
#endif
    struct dqn_node_message message;
    // compute the last one
    uint8_t* base = this->_queue_buf;
    if(this->message_queue.size()){
       struct dqn_node_message back = this->message_queue.back();
       base = back.data + back.size;
    }
    // copy it to the buffer
    memcpy(base, data, size);
    // push it to the queue
    message.data = base;
    message.size = size;
    this->message_queue.push(message);
    return true;

}

bool Node::determine_rate(){
    // TODO:
    // fix this rate
    return false;
    //int rssi = this->rf95->lastRssi();
    //return rssi >= -10;
}

uint16_t Node::mpl(){
    return this->max_payload;
}

void Node::sleep(int32_t time){
    if (time < 0)
      return;
    uint32_t start = millis();
    while(millis() < start + time);
    // TODO:
    // switch to more efficient way to put device into sleep
}

Server::Server(uint32_t networkid,
        float freq,
        struct RH_RF95::pin_config pc,
        void (*on_receive)(uint8_t*, size_t, uint8_t*),
        uint16_t (*on_download)(uint8_t*, uint8_t*, uint8_t))
:RadioDevice(pc, freq)
{
    this->networkid = networkid;
    this->on_receive = on_receive;
    this->on_download = on_download;

    this->crq = 0;

    // use the default network configuration
    // USE TEST VALUES
    this->change_network_config(0, DQN_BF_ERROR, 6, 4);

    this->reset_frame();
}

void Server::send_ack(){
    size_t size = this->num_data_slot / 8;
    if(this->num_data_slot % 8)
        size++;
    print_ack(this->ack_buf, size);
    dqn_send(this->rf95, this->ack_buf, size);
}

void Server::send_feedback(){
    // processing raw TR results to more compacted form
    uint8_t slots[this->num_tr/4];
    // reset to 0
    memset(slots, 0, this->num_tr/4);

    for(int i = 0; i < this->num_tr; i++){
        slots[i/4] |= (this->tr_status[i] & 0x3) << (6 - (i % 4) * 2);
    }
    mprint("\n");
    uint16_t frame_param = this->get_frame_param();
    uint8_t feedback_size = dqn_make_feedback((struct dqn_feedback*)this->_msg_buf, this->networkid, this->crq, this->dtq,
            frame_param, slots, this->num_tr, &this->bloom);

    //mprint("Feedback size: %d\n", feedback_size);
    // assuming the time is correct
    dqn_send(this->rf95, this->_msg_buf, feedback_size);
    print_feedback((struct dqn_feedback*)this->_msg_buf, 0);

}

void Server::reset_frame(){
    // reset the TR status
    for(int i = 0; i < this->num_tr; i++){
        this->tr_status[i] = 0;
    }

    // reset the bloom filter
    bloom_reset(&this->bloom);

    // reset the ACK
    int size = this->num_data_slot / 8;
    if(this->num_data_slot % 8)
        size++;
    for(int i = 0; i < size; i++){
        this->ack_buf[i] = 0;
    }
}

void Server::receive_tr(){
    // transmission will be in no header mode
    rf95->setPayloadLength(sizeof(struct dqn_tr));
    //uint32_t tr_start_time = millis();
    for(int i = 0; i < this->num_tr; i++){
        // loop throw each TR slots
        uint32_t received_time;
        uint8_t len = dqn_recv(this->rf95, this->_msg_buf,
                DQN_TR_LENGTH + DQN_SHORT_GUARD,
                &received_time);
        if(len != sizeof(struct dqn_tr)){
            if(len > 0)
                mprint("received a len: %d\n", len);
            continue;
        }

        // correct index if it was sent a little bit early
        // TODO: this may cause the next one being missed
        int index = i;
        //uint32_t offset = DQN_TR_LENGTH + DQN_SHORT_GUARD -
        //    (received_time - tr_start_time) % (DQN_TR_LENGTH + DQN_SHORT_GUARD);
        //if(offset > 7){
        //    index--;
        //    mprint("offset:%d\n", offset);
        // }

        // compute the CRC
        struct dqn_tr *tr = (struct dqn_tr*)this->_msg_buf;
        mprint("TR received at %d (time: %d). Version: %X message id: %X\n", index, received_time, tr->version, tr->messageid);
        uint8_t crc = tr->crc;
        tr->crc = 0;
        if(crc != get_crc8((char*)tr, sizeof(struct dqn_tr))){
            // there is a collision
            this->tr_status[index] = 3;
            mprint("tr contension detected. received %X %X %X\n", this->_msg_buf[0], this->_msg_buf[1], this->_msg_buf[2]);
        } else {
            if(tr->version != DQN_VERSION){
                mprint("DQN version is not correct. Expect: %d, got %d\n", DQN_VERSION, tr->version);
                continue;
            }
            uint8_t messageid = tr->messageid;

            // check if node id is valid
            uint16_t nodeid = tr->nodeid;
            // TODO: also need to check if it's join process
            if(nodeid != 0 && this->node_table.find(nodeid) == this->node_table.end()){
                mprint("Invalid node id %d\n", nodeid);
                continue;
            } else {
                // add it to bloom filter
                char node_id[10]; // enough for uint_16
                sprintf(node_id, "%x", nodeid);
                bloom_add(&this->bloom, node_id, strlen(node_id));
            }


            if((messageid & DQN_MESSAGE_TR) != DQN_MESSAGE_TR){
                mprint("Invalid message id %d\n", messageid);
                continue;
            }
            uint8_t meta = messageid & DQN_MESSAGE_MASK;
            uint8_t num_of_slots = meta & 3;
            this->tr_status[index] = num_of_slots;
            mprint("TR: %d num of slots: %d\n", index, this->tr_status[index]);
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

void Server::recv_node(){
    // this is definitely two slots
    uint32_t start_time = millis();
    // first slot is receiving
    uint8_t len = dqn_recv(this->rf95, this->_msg_buf, this->data_length + DQN_SHORT_GUARD, NULL);
    if(len == 0){
        mprint("no node registration received\n");
        return;
    } else if(len != sizeof(struct dqn_join_req)){
        mprint("incorrect packet received. expected size: %d, received: %d\n", sizeof(struct dqn_join_req), len);
        return;
    }
    struct dqn_join_req *req = (struct dqn_join_req*)this->_msg_buf;
    if(req->version != DQN_VERSION){
        mprint("version not correct. expected: %X received: %X\n", DQN_VERSION, req->version);
        return;
    }
    // copy it to another memory table
    uint8_t *hw_addr = this->_hw_addr_buf + this->node_table.size() * HW_ADDR_LENGTH;
    memcpy(hw_addr, req->hw_addr, HW_ADDR_LENGTH);
    uint16_t nodeid = this->register_device(hw_addr);
    // TODO:
    // add skipping protocol overhead
    while(millis() < this->data_length + DQN_SHORT_GUARD + start_time);
    dqn_make_join_resp((struct dqn_join_resp*)this->_msg_buf, hw_addr, nodeid);
    dqn_send(this->rf95, this->_msg_buf, sizeof(struct dqn_join_resp));
}

void Server::recv_data(){
    uint32_t start_time = millis();
    int i = 0;
    while(i < this->num_data_slot){
        // align the frame
        while(millis() < i * (this->data_length + DQN_SHORT_GUARD) + start_time);
        if(this->dtqueue.size()){
            struct dqn_data_request *request = this->dtqueue.front();
            this->dtqueue.pop();
            // decode the message ID
            uint8_t messageid = request->messageid;
            if(messageid == 0x92){ // fix this
                this->recv_node();
                i += 2;
                continue;
            }

            uint16_t nodeid = request->nodeid;
            uint8_t *hw_addr = this->node_table[nodeid];
            uint8_t meta = messageid & DQN_MESSAGE_MASK;
            bool downstream = meta & DQN_MESSAGE_DOWNSTREAM;
            // high rate is not supported at the moment.
            //bool high_rate = (meta >> 3) & 1;
            uint8_t num_of_slots = meta & 3;
            if(downstream) {
                uint16_t size = this->on_download(hw_addr, this->_msg_buf, num_of_slots * this->max_payload);
                if(size == 0) {
                    // For now this is just empty without any data
                    // May consider to change it in the future
                    mprint("no data for nodeid: %d\n", nodeid);
                } else {
                    mprint("downstream data: %d\n", size);
                    uint8_t *data = this->_msg_buf;
                    if(size > this->max_payload && num_of_slots == 2){
                         uint32_t start = millis();
                         mprint("sending data slot 0\n");

                         dqn_send(this->rf95, data, this->max_payload);
                              // high_rate? this->rf95->DQN_FAST_CRC:this->rf95->DQN_SLOW_CRC);
                         while(millis() < start + this->data_length + DQN_SHORT_GUARD){
                         }
                         mprint("sending data slot 1\n");
                         dqn_send(this->rf95, data + this->max_payload, size % this->max_payload);
                        //         high_rate? this->rf95->DQN_FAST_CRC:this->rf95->DQN_SLOW_CRC);
                    } else { // if the size is larger than it's allowed, it's the provider's fault
                        dqn_send(this->rf95, data, size > this->max_payload? this->max_payload : size);
                                //high_rate? this->rf95->DQN_FAST_CRC:this->rf95->DQN_SLOW_CRC);
                    }
                }
                i += num_of_slots;
                continue;
            }
            uint8_t total_len = 0;
            // used to continuously fill in the buffer
            uint8_t *data_buf = this->_msg_buf;
            for(int index = 0; index < num_of_slots; index++){
                uint8_t len = dqn_recv(this->rf95, data_buf, this->data_length + DQN_SHORT_GUARD);
                        //high_rate? this->rf95->DQN_FAST_CRC:this->rf95->DQN_SLOW_CRC, NULL);
                if(!hw_addr){
                    mprint("node id %d not found\n", nodeid);
                    i++;
                    break;
                }
                if(!len){
                    mprint("no message received from nodeid%d\n", nodeid);
                } else {
                    total_len += len;
                    data_buf += len;
                    // set the ACK bit
                    this->ack_buf[i / 8] |= 1 << (i % 8);
                }

                i++;
            }
            if(total_len && this->on_receive) {
                this->on_receive(this->_msg_buf, total_len, hw_addr);
            }
        }
        else{
            // ALOHA
            uint8_t len = dqn_recv(this->rf95, this->_msg_buf, this->data_length + DQN_SHORT_GUARD, NULL);
            if(len && this->on_receive) {
                this->on_receive(this->_msg_buf, len, NULL);
            }
        }

        i++;
    }
    // align up at the end
    while(millis() < start_time + this->num_data_slot * (this->data_length + DQN_SHORT_GUARD) +
            DQN_GUARD - DQN_SHORT_GUARD);
}


uint16_t Server::register_device(uint8_t *hw_addr){
    // TODO:
    // add cleaning stuff here, i.e. GC)
    if(this->node_table_invert.count(hw_addr))
        return this->node_table_invert[hw_addr];
    uint16_t nodeid = fletcher16(hw_addr, HW_ADDR_LENGTH);
    while(true){
        if(this->node_table.count(nodeid))
            nodeid++;
        else
            break;
    } // ensure it's unique
    this->node_table.insert(std::pair<uint16_t, uint8_t*>(nodeid, hw_addr));
    this->node_table_invert.insert(std::pair<uint8_t *, uint16_t>(hw_addr, nodeid));
    mprint("assign nodeid: %d for HW: %X:%X:%X:%X:%X:%X\n", nodeid,
            hw_addr[0], hw_addr[1], hw_addr[2], hw_addr[3], hw_addr[4], hw_addr[5]);
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

void Server::end_cycle(){
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

    this->reset_frame();
}

void Server::run(){
    while(true){
        // frame start
        uint32_t frame_start = millis();
        uint32_t feedback_start = frame_start + (this->num_tr * (DQN_TR_LENGTH + DQN_SHORT_GUARD)) - DQN_SHORT_GUARD + DQN_GUARD;
        uint32_t data_start = feedback_start + this->feedback_length + DQN_GUARD;
        uint32_t ack_start = data_start + (this->num_data_slot * (this->data_length + DQN_SHORT_GUARD)) - DQN_SHORT_GUARD + DQN_GUARD;
        uint32_t frame_end = ack_start + this->ack_length + DQN_GUARD;

        if (frame_end < frame_start){
          // hack to make timer wrap around irrelivent.
          mprint("######## TIMER WRAP THIS FRAME SKIP IT! ###########\n");
          while (millis() > frame_end); // wait for wrap.
          while (millis() < frame_end); // now wait for frame.
          continue;
        }

        // TRs
        // testing dont' send TR.s
        configureModem(TR);
        this->receive_tr();

        // feedback frame
        while(millis() < feedback_start );
        configureModem(Feedback);
        this->send_feedback();
        this->rf95->waitPacketSent();

        // data slots
        configureModem(Data);
        while(millis() < data_start);
        // data time
        this->recv_data();

        configureModem(Ack);
        while(millis() < ack_start);

        // this is ACK time
        //this->send_ack();
        while(millis() < frame_end);

        if (millis()> frame_end){
          mprint("frame %d ms over time.\n", millis() - frame_end);
        }
        this->end_cycle();
    }
}
