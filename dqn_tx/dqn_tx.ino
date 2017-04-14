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
  rf95.setTxPower(13);
  
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
  Serial.println("Sending packets...");
} 

void loop() {
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  strcpy((char*)buf, "HELLO WORLD");
  
  if (rf95.send(buf, 6)){
    Serial.print("Airtime is ");
    Serial.print(rf95.getTimeOnAir(6));
    Serial.println(" ms.");
    
    Serial.print("sending: ");
    Serial.print((char*)buf);
    Serial.print("...");
    rf95.waitPacketSent();      
    Serial.println("Sent.");
  }else{
    Serial.println("failed to send.");  
  }
  delay(1000);
}
