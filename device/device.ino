#include <protocol.h>

// static allocation for the node
char __node_buf[sizeof(Node)];
Node* node;

void setup() {
    node = new (__node_buf)Node();
    mprint("joining the network\n");
    node->join();
}

void loop() {
    delay(random(2000) + 1000); // prevent it from sending too fast
    uint16_t data_size = random(node->mpl() * 2);
    mprint("sending %d bytes\n");
    uint8_t data[data_size];
    for(int i = 0; i < data_size; i++){
      data[i] = i;
    }
    node->add_data_to_send(data, data_size);
    node->send();
}

