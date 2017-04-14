#include <RH_RF95.h>
#include <protocol.h>

struct RH_RF95::pin_config pc_feather = {
    .cs = 8,
    .interrupt = 3,
    .fhss_interrupt = -1,
    .reset = 4,
    .tx_led = 13,
    .rx_led = 13
};

RHHardwareSPI zspi = RHHardwareSPI(RHGenericSPI::Frequency8MHz);
RH_RF95 rf95 = RH_RF95(pc_feather, zspi);

void setup() {


  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  Serial.begin(57600);
  while (!Serial) ; // Wait for serial port to be available (does not boot headless!)
  if (!rf95.init()){
    Serial.println("init failed");    
    while (1);
  }
  rf95.setFrequency(915.0);
  rf95.setPreambleLength(6);
  rf95.setFhssHoppingPeriod(0);
  // receive feedback
//  rf95.setModemConfig(
//            125,//DQN_FRAME_BW,
//            1,//DQN_FRAME_CR,
//            DQN_FRAME_VARIABLE_LEN,
//            7,//DQN_FRAME_SF,
//            DQN_FRAME_CRC,
//            DQN_FRAME_LOW_DR);
  rf95.setModemConfig(
            DQN_FRAME_BW,
            DQN_FRAME_CR,
            DQN_FRAME_VARIABLE_LEN,
            DQN_FRAME_SF,
            DQN_FRAME_CRC,
            DQN_FRAME_LOW_DR);
  Serial.println("Listening for packets...");
} 

void loop() {
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  if (rf95.available()){  
    if (rf95.recv(buf, &len)){
      
      struct dqn_feedback* feedback = (struct dqn_feedback*)buf;
      if(feedback->version == DQN_VERSION && feedback->messageid == DQN_MESSAGE_FEEDBACK){
        print_feedback(feedback, rf95.lastRssi());

      }else{
        Serial.print("Unknown version or message id: 0x");
        Serial.print(feedback->version, HEX);
        Serial.print(", 0x");
        Serial.print(feedback->messageid, HEX);
        Serial.print(" received ");
        Serial.print(len);
        Serial.println(" bytes.");
      }
    }   
  }
}
