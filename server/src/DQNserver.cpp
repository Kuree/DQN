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
    if(hw_addr)
        mprint("%X:%X:%X:%X:%X:%X sent a message\n",
            hw_addr[0], hw_addr[1], hw_addr[2], hw_addr[3], hw_addr[4], hw_addr[5]);
    print_packet(data, size);
}

uint16_t on_download(uint8_t* hw_addr, uint8_t* buf, uint8_t max_payload){
    mprint("%X:%X:%X:%X:%X:%X requested a packet\n",
            hw_addr[0], hw_addr[1], hw_addr[2], hw_addr[3], hw_addr[4], hw_addr[5]);
    uint16_t size = 40;
    for(int i = 0; i < size; i++){
        buf[i] = i % 256;
    }
    return size;

}

void sig_handler(int sig)
{
    if (flag == 1){
        printf("\n--- Double CTRL-C - panic stop---\n");
        exit(-99);
    }
    flag=1;
}

#ifndef ARDUINO
int main (int argc, const char* argv[] ){
    signal(SIGINT, &sig_handler);

    // wiringPi has to be setup!
    wiringPiSetup();

    // see answer: http://arduino.stackexchange.com/a/1499

    // my rev2 lora hat uses different pins CS=10, INT=21
    //    rev1 lora hat uses                CS=10, INT=7
    // I could subclass the server for each HW rev or
    // deal with passing in HW params.

#if defined LORAHATV1
    struct RH_RF95::pin_config pc_lorahat = {
         .cs = 10,
         .interrupt = 7,
         .fhss_interrupt = -1,
         .reset = 0,
         .tx_led = 4,
         .rx_led = 5
       };
#elif defined LORAHATV2
    struct RH_RF95::pin_config pc_lorahat = {
        .cs = 10,
        .interrupt = 21,
        .fhss_interrupt = -1,
        .reset = 26,
        .tx_led = 4,
        .rx_led = 5
      };
#else
  #error *** Define LORAHATV1 or LORAHATV2 eg $make CFLAGS=-DLORAHATV2****
#endif
    server = new (__server_buf)Server(0,
      915.0,
      pc_lorahat,
      &print_message, &on_download);
#else
void setup(){
  struct RH_RF95::pin_config pc_feather = {
      .cs = 8,
      .interrupt = 3,
      .fhss_interrupt = -1,
      .reset = 4,
      .tx_led = 13,
      .rx_led = 13
    };
    server = new (__server_buf)Server(0,
      915.0,
      pc_feather,
      &print_message, &on_download);
}

void loop(){
#endif
    server->print_frame_info();
    server->run();

}
