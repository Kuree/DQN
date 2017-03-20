#include <protocol.h>

// static allocation for the node
char __node_buf[sizeof(Node)];
Node* node;

void setup() {
    node = new (__node_buf)Node();
    //node->check_sync();
    mprint("joinging the network\n");
    node->join();
}

void loop() {
    delay(random(2000) + 1000);
    mprint("sending data\n");
    uint8_t data[20];
    for(int i = 0; i < 20; i++){
      data[i] = i;
    }
    node->add_data_to_send(data, 20);
    node->send();
}

