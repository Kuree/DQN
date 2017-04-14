#include <protocol.h>

// static allocation for the node
char __node_buf[sizeof(Node)];
Node* node;

struct RH_RF95::pin_config pc_feather = {
    .cs = 8,
    .interrupt = 3,
    .fhss_interrupt = -1,
    .reset = 4,
    .tx_led = 13,
    .rx_led = 13
};

void setup() {
    node = new (__node_buf)Node(pc_feather);
    mprint("joining the network\n");
    node->join();
}

void print_packet(uint8_t *data, uint8_t size){
    for(int i = 0; i < size; i++){
        mprint("%X ", data[i]);
        if(i % 16 == 15)
            mprint("\n");
    }
    mprint("\n");
}

void loop() {
    delay(random(4000) + 1000); // prevent it from sending too fast
    uint16_t data_size = random(node->mpl() * 2 - 5) + 5;
    uint8_t data[data_size];
    for(int i = 0; i < data_size; i++){
      data[i] = i;
    }
    if(node->add_data_to_send(data, data_size)){
        bool ack;
        mprint("sending %d bytes\n");
        node->send();//&ack);
    }
    //node->set_on_receive(&print_packet);
    //node->recv();
}

