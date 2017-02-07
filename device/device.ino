#include <SPI.h>
#include <protocol.h>
#include <RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 915.0

#define VBATPIN A7  

// function prototypes
void sync_time();
void sync_time(bool);
void device_sleep(uint32_t time);
void send_packet();
void wait_data_slot();
void wait_to_send();
void send_tr();
void handle_feedback(struct dqn_feedback *feedback);
void dtq_send();
void crq_wait();
void send_fragment(uint8_t *data, int size, int mtu);
bool aloha_send(struct dqn_feedback*);
void switch_oh();
void switch_fast();

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT); // Adafruit Feather M0 with RFM95 

// when the device is turned on, we need to synchronize the
// clock time
int device_state = DQN_SYNC;
// used to calculate the time
uint32_t OFFSET;
//uint32_t FEEDBACK_TIME;

// this is the message to send
uint8_t transmission_data[DQN_MAX_PACKET];
uint32_t packet_size;
int chosen_slot;
uint32_t queue_sleep_time;

// change this before transmission
uint32_t DQN_RATE_TABLE[DQN_AVAILABLE_RATES] = {DQN_RATE_0, DQN_RATE_1};
uint32_t DQN_RATE = DQN_RATE_TABLE[0];

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
    rf95.setTxPower(23);

    if (rf95.setModemConfig(rf95.Bw500Cr48Sf4096NoCrc)){
        Serial.println("rf95 configuration set to BW=500 kHz BW, CR=4/8 CR, SF=12.");
    } else{
        Serial.println("rf95 configuration failed.");
        while (1);
    }

    // setup the preamble
    rf95.setPreambleLength(DQN_PREAMBLE);
    Serial.print("Set preamble to "); Serial.println(DQN_PREAMBLE);

    // compute the feedback time
    //FEEDBACK_TIME = (LORA_HEADER + DQN_PREAMBLE + sizeof(struct dqn_feedback)) * 8000 / DQN_RATE; // compute in millis second
    Serial.print("feedback packet transmission time is "); Serial.print(FEEDBACK_TIME); Serial.println(" ms");
    Serial.print("DQN MTU: ");Serial.print(DQN_MTU);Serial.println();
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
                //uint32_t sleep_time = random(1000, 5000); // sleep for random 1-5s
                //Serial.print("device sleep for "); Serial.print(sleep_time); Serial.println(" ms");
                //device_sleep(sleep_time);
                packet_size = random(DQN_MTU, (DQN_N - 1) * DQN_MTU);
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
                    // sync before send
                    device_state = DQN_ADJT;
                    sync_time(true);
                    if(device_state != DQN_TRAN) break; // due to aloha mode
                    wait_to_send();
                    send_tr();
                    break;
                }
            case DQN_CRQ:
                {
                    crq_wait();
                    break;
                }
            case DQN_DTQ:
                {
                    dtq_send();
                    device_state = DQN_SENT;
                    break;
                }
            case DQN_SENT:
                {
                    // TODO: wait for feedback
                    device_state = DQN_IDLE;
                    has_sent = true;
                    break;
                }
            default:
                // something went wrong
                {
                    Serial.print("device is in a corrupted state "); Serial.print(device_state);
                    Serial.println(" trying to resend...");
                    device_state = DQN_TRAN;
                    break;
                }
        }
    }
}


void send_fragment(uint8_t *data, int total_size, int mtu){
    int num_packets = total_size / mtu + 1;
    for(int i = 0; i < num_packets; i++){
        uint32_t size = (i != (num_packets - 1))? mtu: total_size % mtu;
        if(!rf95.send(data + mtu * i, size)){
            Serial.println("sending data failed");
            device_state = DQN_IDLE; // reset the device state if failed
        } else {
            Serial.print("packet fragment "); Serial.print(i); Serial.println(" sent");
        }
    }
}

void dtq_send(){
    // switched to higher transmission rate
    // we need to compute how many frames we need to skip
    // it can be very messy...
    // first align the device after TR
    uint32_t sleep_time = OFFSET + DQN_LENGTH + DQN_GUARD + DQN_MINI_SLOT_FRAME - millis(); 
    Serial.print("device sleep ");Serial.print(sleep_time); Serial.println("before transmission in DTQ");
    device_sleep(sleep_time);
    // TODO: optimize this
    bool has_sent = false;
    int counter = 0;
    int num_packets = packet_size / DQN_MTU + 1;
    while(!has_sent){
        if(queue_sleep_time){
            queue_sleep_time--;
            counter++;
            device_sleep(DQN_LENGTH + DQN_GUARD);
            if(counter == DQN_N) { // overhead block
                device_sleep(DQN_OVERHEAD * (DQN_LENGTH + DQN_GUARD));
                counter = 0;
            }
        } else { // need to transmit
            for(int i = 0; i < num_packets; i++){
                // NOTE: this has 15ms delay
                // fast transmission
                switch_fast();

                uint32_t start = millis();
                Serial.print("sending packet "); Serial.print(i + 1); Serial.print(" of ");
                Serial.print(num_packets); Serial.print(". Total size "); Serial.print(packet_size); Serial.println();
                uint32_t size = (i != (num_packets - 1))? DQN_MTU: packet_size % DQN_MTU;
                send_fragment(transmission_data + DQN_MTU * i, size, RH_RF95_MAX_MESSAGE_LEN); 
                counter++;
                while(millis() < start + DQN_LENGTH + DQN_GUARD); // sleep till next frame
                if(counter == DQN_N){
                    device_sleep(DQN_OVERHEAD * (DQN_LENGTH + DQN_GUARD));
                    counter = 0;
                }
            }
            has_sent = true;
            packet_size = 0;
        }
    }
}

void crq_wait(){
    // since the OFFSET is set to the beginning of every current frame
    // we need to sleep through to the next frame. then compute how many time to sleep
    // notice that for crq, queue_sleep_time is for entire frames
    // TODO: test this
    uint32_t sleep_time = (queue_sleep_time) * (DQN_OVERHEAD + DQN_N) * (DQN_LENGTH + DQN_GUARD); // sleep time after the next frame
    queue_sleep_time = 0; // reset the queue sleep_time
    //uint32_t sleep_current_frame = (DQN_N + 2) * DQN_LENGTH + OFFSET - millis();
    // we don't need to calibrate to the beginning of the frame for two reasons
    //  1. use sleep_time will set the device to middle of the frame before
    //  2. once the device is swtiched to transmission state, wait_send_send() will set the device
    //      to current frame
    device_sleep(sleep_time);
    device_state = DQN_TRAN;
}


void wait_to_send(){
    // this will be called after off is synced
    uint32_t sleep_time = OFFSET + DQN_LENGTH - millis();
    Serial.print("device sleeps for "); Serial.print(sleep_time); Serial.println(" ms before sending TR");
    device_sleep(sleep_time);
}


void switch_oh(){
    rf95.setEncoding(DQN_SF_4096, DQN_CR_48, false);
}

void switch_fast(){
    rf95.setEncoding(DQN_SF_128, DQN_CR_45);
}

void send_tr(){
    // TODO: use RSSI to determine the transmission rate
    // this is start of TR frame
    uint32_t frame_start_time = millis();

    // switch to slower transmission rate
    // NOTE: this has 15 ms switching time
    switch_oh();

    chosen_slot = random(0, DQN_M);
    uint32_t sleep_time = chosen_slot * DQN_MINI_SLOT_FRAME / DQN_M;
    Serial.print("device choose mini-slot "); Serial.print(chosen_slot); 
    Serial.print(" sleep time "); Serial.print(sleep_time); Serial.println(" ms");
    device_sleep(sleep_time);
    struct dqn_tr tr;
    // requests for higher transmission rate
    tr.rate = encode_rate(DQN_SF_128, DQN_CR_45, true);
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

    // sleep till the beginning of next feedback
    while(millis() < frame_start_time + DQN_MINI_SLOT_FRAME + (DQN_LENGTH + DQN_GUARD) * DQN_N - 2 * DQN_GUARD){
    }

    // feedback receive
    sync_time();
}

void sync_time(){
    sync_time(false);
}

void sync_time(bool use_loop){
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    const uint32_t START_TIME = millis();

    // switch to slow transmission rate
    switch_oh();

    // wait for a feedback
    // and use the rssi to calibrate the actual time
    while(millis() < START_TIME + DQN_LENGTH || use_loop){
        if (rf95.available())
        {
            if (rf95.recv(buf, &len))
            {
                uint32_t received_time = millis();

                struct dqn_feedback *feedback = (struct dqn_feedback*)buf;
                uint8_t crc = feedback->crc;
                feedback->crc = 0;
                uint8_t packet_crc = get_crc8((char*)feedback, len);
                if(crc == packet_crc){
                    // we got a feedback packet!!!!
                    OFFSET = received_time - FEEDBACK_TIME; 
                    Serial.print("offset set to ");
                    Serial.print(OFFSET);
                    Serial.print("\n");
                    // if the device just started up
                    // changed to idle
                    // if in transmission, need to check if we've requested successfully
                    if(!aloha_send(feedback)){
                        if(device_state == DQN_SYNC){
                            device_state = DQN_IDLE;
                            break;
                        } else if(device_state == DQN_TRAN) {
                            Serial.println("computing queue...");
                            handle_feedback(feedback);
                        } else if(device_state == DQN_ADJT) {
                            device_state = DQN_TRAN;
                            break;
                        }
                    } else {
                        break;
                    }
                } else{
                    if(device_state == DQN_TRAN){
                        Serial.println("no feedback received...");
                    }
                }
            } else{
                Serial.println("ERR: recv failed");
            }
        }
    }
}

void handle_feedback(struct dqn_feedback* feedback){
    uint8_t status = feedback->slots[chosen_slot];
    int num_slots = packet_size / DQN_MTU + 1;
    if(status == DQN_N) { // this is a contended slot
        // devie enter CRQ
        uint32_t crq = feedback->crq_length;
        for(int i = 0; i < chosen_slot; i++){
            if(feedback->slots[i] == DQN_N)
                crq += 1;
        }
        Serial.print("device enter CRQ in "); Serial.print(crq); Serial.println();
        queue_sleep_time = crq;
        device_state = DQN_CRQ;
    } else if(status == num_slots){
        // device enter DTQ
        uint32_t dtq = feedback->dtq_length;
        for(int i = 0; i < chosen_slot; i++){
            if(feedback->slots[i] != 0 && feedback->slots[i] != DQN_N){
                dtq += feedback->slots[i];
            }
        }
        Serial.print("device enter DTQ in "); Serial.print(dtq); Serial.println();
        queue_sleep_time = dtq;
        device_state = DQN_DTQ;
        // notice that the device needs to skip TR
        // TODO: change to sleep
        while(millis() < OFFSET + DQN_MINI_SLOT_FRAME + DQN_LENGTH);
    } else {
        Serial.print("Something went wrong. device choose slot ");
        Serial.print(chosen_slot); Serial.print(" status returned at chosen slot");
        Serial.print(status); Serial.println();
    }
}

bool aloha_send(struct dqn_feedback* feedback){
    if(DQN_ALOHA && packet_size != 0 && 
        (device_state == DQN_SYNC || device_state == DQN_IDLE || device_state == DQN_ADJT)) {
        // calculate the dtq
        int dtq = feedback->dtq_length;
        for(int i = 0; i < DQN_M; i++){
            if(feedback->slots[i] != 0 && feedback->slots[i] != DQN_N){
                dtq += feedback->slots[i];
            }
        }
        // we have packet to send yet not in the transmission mode
        int num_packet = packet_size / DQN_MTU + 1;
        if(dtq < DQN_N && num_packet < (DQN_N - dtq)) {
            // there are enough free data slots we can send
            queue_sleep_time = dtq;
            device_state = DQN_DTQ;
            Serial.println("ALOHA mode on. switched to transmission mode");
            return true;
        }
    }
    return false;
}


void device_sleep(uint32_t time){
    // put radio into sleep
    //rf95.sleep();
    // switch to more energy efficient way to do this
    uint32_t start_time = millis();
    uint32_t end_time = start_time + time;
    while(millis() < end_time);
    //delay(time);
    //rf95.setMode(RHGenericDriver::RHModeIdle);
}
