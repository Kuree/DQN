#include <SPI.h>
#include <RH_RF95.h>
#include <protocol.h>

#define DEVICE_ID 1234

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 915.0

#define VBATPIN A7  

// function prototypes
void sync_time();
void device_sleep(uint32_t time);
void send_packet();
void wait_data_slot();
void wait_to_send();
void send_tr();


// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT); // Adafruit Feather M0 with RFM95 

// when the device is turned on, we need to synchronize the
// clock time
int device_state = DQN_SYNC;
// used to calculate the time
uint32_t OFFSET;
uint32_t FEEDBACK_TIME;

// this is the message to send
char transmission_data[DQN_MAX_PACKET];
uint32_t packet_size;


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

    // compute the feedback time
    FEEDBACK_TIME = (DQN_PREAMBLE + sizeof(struct dqn_feedback)) * 8000 / DQN_OH_RATE; // compute in millis second
    Serial.print("feedback packet transmission time is "); Serial.print(FEEDBACK_TIME); Serial.println(" ms");
}

void loop() {
    switch(device_state){
        case DQN_SYNC: 
            {
                sync_time();
                break;
            }

        case DQN_IDLE:  // TODO: make it into a library
            {
                uint32_t sleep_time = random(1000, 10000); // sleep for random 1-10s
                Serial.print("device sleep for "); Serial.print(sleep_time); Serial.println(" ms");
                device_sleep(sleep_time);
                packet_size = random(DQN_MTU, DQN_N * DQN_MTU);
                Serial.print("sending packet size "); Serial.print(packet_size); Serial.println(" bytes");
                // populate the mock data;
                for(int i = 0; i < packet_size; i++){
                    transmission_data[i] = i % 256; // wrapper around            
                }
                Serial.println("device switched to transmission mode");
                device_state = DQN_TRAN;
                break;
            }
        case DQN_TRAN: 
            {
                // need to sleep till the TR frame
                // send the packet
                send_packet();
                break;
            }
        default:
            break;
    }
}

void send_packet(){
    bool has_sent = false;
    while(!has_sent){ // loop till you send the packet
        switch(device_state) {
            case DQN_TRAN: 
                {
                    wait_to_send();
                    send_tr();
                    break;
                }
            case DQN_CRQ:
                {
                    break;
                }
            case DQN_DTQ:
                {
                    has_sent = true;
                    break;
                }
            default:
                // something went wrong
                {
                    Serial.println("device is in a corrupted state");
                    has_sent = true;
                    break;
                }
        }
    }
}

void wait_to_send(){
    uint32_t total_cycle = (2 + DQN_N) * DQN_LENGTH; // TODO: fixed the cycle thing
    uint32_t passed_time = (millis() - OFFSET) % total_cycle;
    uint32_t sleep_time = total_cycle - passed_time;
    Serial.print("device sleeps for "); Serial.print(sleep_time); Serial.println(" ms");
    device_sleep(sleep_time);
}

void send_tr(){
    // TODO: use RSSI to determine the transmission rate
    uint32_t frame_start_time = millis();
    int slot_number = random(0, DQN_M);
    uint32_t sleep_time = slot_number * DQN_LENGTH / DQN_M;
    Serial.print("device choose mini-slot "); Serial.print(slot_number); 
    Serial.print(" sleep time "); Serial.print(sleep_time); Serial.println(" ms");
    device_sleep(sleep_time);
    struct dqn_tr tr;
    tr.id = DEVICE_ID;
    tr.num_slots = packet_size / DQN_MTU + 1;

    // calculate crc
    tr.crc = 0;
    tr.crc = get_crc8((char*)&tr, sizeof(tr));
    
    if(!rf95.send((uint8_t *)&tr, sizeof(tr))){
        Serial.println("sending TR failed");
        device_state = DQN_IDLE; // reset the device state if failed
    } else {
        Serial.println("TR sent");
    }

    // wait for TR TODO: fix this sleep
    while(millis() < frame_start_time + DQN_LENGTH){
    }

    // test feedback receive
    Serial.println("tring to receive feedback time");
    sync_time();
}

void sync_time(){
    Serial.println("start to sync time");
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    const uint32_t START_TIME = millis();
    // wait for a feedback
    // and use the rssi to calibrate the actual time
    while(millis() < START_TIME + DQN_LENGTH){ // TODO: fix this time
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
                    OFFSET = millis() - FEEDBACK_TIME;
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
}

void device_sleep(uint32_t time){
    // put radio into sleep
    //rf95.sleep();
    // switch to more energy efficient way to do this
    delay(time);
    //rf95.setMode(RHGenericDriver::RHModeIdle);
}
