// based on RasPiRH95.cpp contributed by Alan Marchiori and Mike Poublon


#include <wiringPi.h>

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>


#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <string.h>
#include <RH_RF95.h>

#include "../core/protocol.h"

// DQN related constants
const uint32_t FEEDBACK_TIME = (DQN_PREAMBLE + sizeof(struct dqn_feedback)) * 8000 / DQN_OH_RATE;
const uint32_t TR_TIME = (DQN_PREAMBLE + sizeof(struct dqn_tr)) * 8000 / DQN_OH_RATE;


//Function Definitions
void sig_handler(int sig);
void print_feedback(struct dqn_feedback fb);

// pin numers use wiringPi numbers.
#define RF95_RESET_PIN 0  // this is BCM pin 17, physical pin 11.
#define RF95_INT_PIN 7    // this is BCM pin 4, physical pin 7.
#define RF95_CS_PIN 10    // this is BCM pin 8, physical pin 24

// wiringPi pin numbers
#define TX_PIN 4
#define RX_PIN 5

static const uint8_t hw_address[] = {0x98,0x76,0xb6,0x5c,0x00,0x00};
RH_RF95 rf95(RF95_CS_PIN, RF95_INT_PIN);

//Flag for Ctrl-C
volatile sig_atomic_t flag = 0;

//Main Function
int main (int argc, const char* argv[] ){
    signal(SIGINT, sig_handler);

    wiringPiSetup();

    printf( "\nRasPiRH95 Tester Startup\n\n" );

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

    if (!rf95.init()){
        printf("rf95 init failed.\n");
        exit(-95);
    }else{
        printf("rf95 init success.\n");
    }
    if (!rf95.setFrequency (915.0)){
        printf("rf95 set freq failed.\n");
        exit(-96);
    }else{
        printf("rf95 set freq to %5.2f.\n", 915.0);
    }

    if (rf95.setModemConfig(rf95.Bw500Cr45Sf128)){
        printf("rf95 configuration set to BW=500 kHz BW, CR=4/5 CR, SF=7.\n");
    }else{
        printf("rf95 configuration failed.\n");
        exit(-97);
    }

    // set the preamble
    rf95.setPreambleLength(DQN_PREAMBLE);
    printf("rf95 set preamble to %d\n", DQN_PREAMBLE);

    rf95.setTxPower(23, false);

    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

    // parameters used by DQN
    const uint32_t OFFSET_TIME = millis();
    const uint32_t MINI_SLOT_TIME = DQN_LENGTH / DQN_M;
    uint16_t crq = 0;
    uint16_t dtq = 0;

    while (true){
        const uint32_t CYCLE_START_TIME = millis();
        printf("cycle started at %d\n", CYCLE_START_TIME);
        uint8_t len = sizeof(buf);
        uint8_t from, to, id, flags;
        uint16_t new_crq = crq;
        uint16_t new_dtq = dtq;

        int a, b; // this is used for debug information

        // setup TR counter
        uint8_t tr_results[DQN_M];
        for(uint8_t i = 0; i < DQN_M; i++){
            tr_results[i] = DQN_IDLE;
        }

        // wait for mini slot requests
        // TODO: need to adjust this time based on how fast
        // the packet can transmit
        while(millis() < CYCLE_START_TIME + DQN_LENGTH){
            if(rf95.available()){
                printf("got a TR packet\n");
                uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
                uint8_t len = sizeof(buf);
                if (rf95.recv(buf, &len)){
                    struct dqn_tr* tr = (struct dqn_tr *)buf;
                    uint8_t crc = tr->crc;
                    tr->crc = 0;
                    uint8_t packet_crc = get_crc8((char*)tr, 3); // only 3 bytes need to calculate
                    // calculate the slot number
                    uint16_t time_offset = millis() - CYCLE_START_TIME - TR_TIME;
                    uint8_t mini_slot = time_offset / MINI_SLOT_TIME;
                    // set the status of the mini slot 
                    if(packet_crc == crc){
                        tr_results[mini_slot] = tr->num_slots;
                        new_dtq += tr->num_slots;
                    } else{
                        tr_results[mini_slot] = DQN_N;
                        new_crq += 1;
                    }
                }
            }
        }

        // immediately send the feedback result 
        // so that TR and feedback will be put into the same channel frequency later on
        struct dqn_feedback feedback;
        feedback.crq_length = crq;
        feedback.dtq_length = dtq;
        // process the mini slots
        for(int i = 0; i < DQN_M; i++){
            //uint8_t result = 0;
            //for(int j = 0; j < 4; j++){
            //    // each status only needs 2 bits
            //    uint8_t status = tr_results[j + i * 4];
            //    // adding crq and dtq accordingly
            //    result |= status << (3 - j); // lower address to high address
            //}
            // TODO: compress the space
            feedback.slots[i] = tr_results[i];
        }

        // handle crc
        feedback.crc = 0;
        feedback.crc = get_crc8((char*)&feedback, sizeof(feedback));
        // send the feedback
        a = micros();
        if(!rf95.send((uint8_t *)&feedback, sizeof(feedback))){
            printf("sending feedback failed");
        } else{
            b = micros();
            print_feedback(feedback);
            printf("sent feedback took %dus to send %d bytes\n", b-a, sizeof(feedback));
        }

        // reduce the queue length
        dtq = new_dtq > DQN_N? new_dtq - DQN_N: 0;
        crq = new_crq > 1? new_crq - 1: 0;


        // moved to the receive window
        // DQN_LENGTH ms for overhead 
        delay(DQN_LENGTH * 2 - (millis() - CYCLE_START_TIME));

        while(millis() < CYCLE_START_TIME + DQN_LENGTH * (DQN_N + 2)){
            // TODO: add channel hopping
            if(rf95.available()){
                // TODO: assemble the fragment together and return to the library user
                uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
                uint8_t len = sizeof(buf);
                if (rf95.recv(buf, &len))
                {
                    printf("receiving data... size %d\n", len);
                } else{
                    printf("receiving failed\n");
                }
            }
        }

        if (flag)
        {
            printf("\n---CTRL-C Caught - Exiting---\n");
            break;
        }
    }

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

void print_feedback(struct dqn_feedback fb){
    for(int i = 0; i < DQN_M; i++){
        printf("[%d]: %d\t", i, fb.slots[i]);
    }
    printf("\n CRQ: %d\tDTQ: %d\n", fb.crq_length, fb.dtq_length);
}

