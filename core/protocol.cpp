#define BLOOM_SIZE 64
#include "protocol.h"

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
