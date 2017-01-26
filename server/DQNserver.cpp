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

//Function Definitions
void sig_handler(int sig);
void printbuffer(uint8_t buff[], int len);

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

    rf95.setTxPower(23, false);

    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

    // parameters used by DQN
    const uint32_t OFFSET_TIME = millis();
    const uint32_t MINI_SLOT_TIME = DQN_MINI_SLOT_LENGTH / DQN_M;
    uint16_t crq = 0;
    uint16_t dtq = 0;

    while (true){
        const uint32_t CYCLE_START_TIME = millis();
        uint8_t len = sizeof(buf);
        uint8_t from, to, id, flags;
        uint16_t new_crq = crq;
        uint16_t new_dtq = dtq;
        
        // setup TR counter
        uint8_t tr_results[DQN_M];
        for(uint8_t i = 0; i < DQN_M; i++){
            tr_results[i] = DQN_IDLE;
        }

        // wait for mini slot requests
        // TODO: need to adjust this time based on how fast
        // the packet can transmit
        while(millis() < CYCLE_START_TIME + DQN_MINI_SLOT_LENGTH){
            if(rf95.available()){
                uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
                uint8_t len = sizeof(buf);
                if (rf95.recv(buf, &len)){
                    struct dqn_tr* tr = (struct dqn_tr *)buf;
                    uint8_t crc = tr->crc;
                    tr->crc = 0;
                    uint8_t packet_crc = get_crc8((char*)tr, 3); // only 3 bytes need to calculate
                    // calculate the slot number
                    uint16_t time_offset = millis() - CYCLE_START_TIME;
                    uint8_t mini_slot = time_offset / MINI_SLOT_TIME;
                    // set the status of the mini slot 
                    if(packet_crc == crc){
                        tr_results[mini_slot] = DQN_SUCCESS;
                        new_dtq += tr->num_slots;
                    } else{
                        tr_results[mini_slot] = DQN_CONTEND;
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
        for(int i = 0; i < DQN_FB_LENGTH; i++){
            uint8_t result = 0;
            for(int j = 0; j < 4; j++){
                // each status only needs 2 bits
                uint8_t status = tr_results[j + i * 4];
                // adding crq and dtq accordingly
                result |= status << (3 - j); // lower address to high address
            }
            feedback.slots[i] = result;
        }

        // send the feedback
        if(!rf95.send((uint8_t *)&feedback, sizeof(feedback))){
            printf("sending feedback failed");
        } else{
            printf("sent feedback with status %b\tcrq: %d\tdtq:%d", 
                    feedback.slots[0], feedback.crq_length, feedback.dtq_length);
        }

        // defuce the queue length
        dtq = new_dtq - DQN_N;
        crq = new_crq - 1;
        

        // now receive data for N slots

                
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

void printbuffer(uint8_t buff[], int len)
{
    for (int i = 0; i< len; i++)
    {
        printf(" %2X", buff[i]);
    }
}
