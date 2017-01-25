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
int main (int argc, const char* argv[] )
{
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
  /* End Driver Only Init Code */

  /* Begin Reliable Datagram Init Code
  if (!manager.init())
  {
    printf( "Init failed\n" );
  }
   End Reliable Datagram Init Code */

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

  printf("waiting for packets...\n");
  //Begin the main body of code

  int last_send = millis();
  int packet_num = 0;
  while (true)
  {
    uint8_t len = sizeof(buf);
    uint8_t from, to, id, flags;

    if (millis() - last_send > 3000){
      uint8_t message[3] = {'s', 'u', 'p'};
      last_send = millis();
      printf("Sending hello... ");

      if (!rf95.send(message, 4)){
        printf("failed.\n");
      }else{
        printf("complete.\n");
      }

    }

    /* Begin Driver Only code */
    if (rf95.available())
    {
      // Should be a message for us now
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len))
      {
        uint32_t a, b;

        char echo[RH_RF95_MAX_MESSAGE_LEN] = "";
        int8_t rssi = rf95.lastRssi();
        printf("echoing data [%d dBm]: ", rssi);
        printf("%s\n",(char*)buf);

        strcat(echo, "echo: ");
        strcat(echo, (char*)buf);
        a = micros();
        rf95.send((uint8_t*)echo, len+6);
        rf95.waitPacketSent();
        b = micros();
        printf("Send took %6.3f ms\n", (b-a) / 1000.0);
      }
      else
      {
        printf("recv failed");
      }
    }
    /* End Driver Only Code */

    /* Begin Reliable Datagram Code
    if (manager.available())
    {
      // Wait for a message addressed to us from the client
      uint8_t len = sizeof(buf);
      uint8_t from;
      if (manager.recvfromAck(buf, &len, &from))
      {
        Serial.print("got request from : 0x");
        Serial.print(from, HEX);
        Serial.print(": ");
        Serial.println((char*)buf);
      }
    }
    End Reliable Datagram Code */

    if (flag)
    {
      printf("\n---CTRL-C Caught - Exiting---\n");
      break;
    }
    //sleep(1);
    delay(25);
  }
  printf( "\nRasPiRH95 Tester Ending\n" );

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
