// based on RasPiRH95.cpp contributed by Alan Marchiori and Mike Poublon


#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include <string.h>

#ifndef ARDUINO
#include "../core/protocol.h"
#else
#include <protocol.h>
#endif

char __server_buf[sizeof(Server)]; // place holder for server memory.
Server *server;

//Flag for Ctrl-C
volatile sig_atomic_t flag = 0;

void print_packet(uint8_t *data, int length){
    //const int PRINT_SIZE = 32;
    //length = length > PRINT_SIZE? PRINT_SIZE:length;
    for(int i = 0; i < length; i++){
        printf("%02X ", data[i]);
        if(i % 16 == 15)
            printf("\n");
    }
    printf("\n");
}

void print_message(uint8_t* data, size_t size, uint8_t* hw_addr){
    mprint("%X:%X:%X:%X:%X:%X send a message\n",
            hw_addr[0], hw_addr[1], hw_addr[2], hw_addr[3], hw_addr[4], hw_addr[5]);
    print_packet(data, size);
}

void sig_handler(int sig)
{
    if (flag == 1){
        printf("\n--- Double CTRL-L - panic stop---\n");
        exit(-99);
    }
    flag=1;
}

void print_feedback(struct dqn_feedback* fb){
    for(int i = 0; i < 80; i++){
        printf("[%d]: %d\t", i, fb->data[i]);
    }
    printf(" CRQ: %d\tDTQ: %d\tACK: ", fb->crq_length, fb->dtq_length);

    printf("\n");
}


#ifndef ARDUINO
int main (int argc, const char* argv[] ){
    signal(SIGINT, &sig_handler);
    // see answer: http://arduino.stackexchange.com/a/1499
    server = new (__server_buf)Server(0, &print_message, NULL);
#else
void setup(){
    server = new (__server_buf)Server(0, &print_message, NULL);
}

void loop(){
#endif
    server->print_frame_info(); 
    server->run();

}



