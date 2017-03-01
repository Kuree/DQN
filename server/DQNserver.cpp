// based on RasPiRH95.cpp contributed by Alan Marchiori and Mike Poublon


#include <wiringPi.h>

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>


#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <queue>

#include <string.h>

#include "../core/protocol.h"

using namespace std;


//Function Definitions
void sig_handler(int sig);
void print_feedback(struct dqn_feedback* fb);

char __server_buf[sizeof(Server)]; // place holder for server memory.
Server *server;

//Flag for Ctrl-C
volatile sig_atomic_t flag = 0;

//Main Function
int main (int argc, const char* argv[] ){
    signal(SIGINT, sig_handler);

    // see answer: http://arduino.stackexchange.com/a/1499
    server = new (__server_buf)Server(0, NULL, NULL);
    server->print_frame_info(); 

    return 0;
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
    for(int i = 0; i < DQN_M; i++){
        printf("[%d]: %d\t", i, fb->data[i]);
    }
    printf(" CRQ: %d\tDTQ: %d\tACK: ", fb->crq_length, fb->dtq_length);

    printf("\n");
}

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
