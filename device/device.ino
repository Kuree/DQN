#include <SPI.h>
#include <RH_RF95.h>
#include <protocol.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 915.0

#define VBATPIN A7

// function prototypes
void sync_time();

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT); // Adafruit Feather M0 with RFM95 

// when the device is turned on, we need to synchronize the
// clock time
int device_state = DQN_SYNC;
// used to calculate the time
uint32_t OFFSET;

void setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    pinMode(VBATPIN, INPUT);

    // un-reset the radio
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    Serial.begin(57600);
    while (!Serial) ; // Wait for serial port to be available (does not boot headless!)
    if (!rf95.init()){
        Serial.println("init failed");

        while (1);
    }
    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed"); 
        while (1);
    }
    Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
    // you can set transmitter powers from 5 to 23 dBm:
    rf95.setTxPower(23, false);

    if (rf95.setModemConfig(rf95.Bw500Cr45Sf128)){
        Serial.println("rf95 configuration set to BW=500 kHz BW, CR=4/5 CR, SF=7.");
    } else{
        Serial.println("rf95 configuration failed.");
        while (1);
    }
    
    // setup the preamble
    rf95.setPreambleLength(DQN_PREAMBLE);
    Serial.print("Set preamble to "); Serial.println(DQN_PREAMBLE);
}

void loop() {
    switch(device_state){
        case DQN_SYNC: {
            sync_time();
            break;
        }
        default:
            break;
    }
}

void sync_time(){
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    // wait for a feedback
    // and use the rssi to calibrate the actual time
    if (rf95.available())
    {
        if (rf95.recv(buf, &len))
        {
            Serial.print("get a packet with size "); Serial.print(len); Serial.println(" bytes");

            struct dqn_feedback *feedback = (struct dqn_feedback*)buf;
            uint8_t crc = feedback->crc;
            uint8_t packet_crc = get_crc8((char*)feedback, len);
            if(crc == packet_crc){
                // we got a feedback packet!!!!
                OFFSET = millis();
                Serial.print("offset set to ");
                Serial.print(OFFSET);
                Serial.print("\n");
                device_state = DQN_IDLE;
            }
        } else{
            Serial.println("ERR: recv failed");
        }
    }
}
