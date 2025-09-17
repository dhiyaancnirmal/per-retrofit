#include <Arduino.h>
#include "FlexCAN_T4.h"
#include "DFRobot_BMM350.h"

// Pins for CAN1: (CRX1 -> pin 23) (CTX1 --> pin 22)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; // Use CAN 2.0 constructor template

// put function declarations here:
void canSniff(const CAN_message_t &msg);

void setup() {
  // put your setup code here, to run once:
  can1.begin();
  can1.setBaudRate(500000); // Set CAN bitrate, 500kb
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canSniff);
  can1.mailboxStatus();

  Serial.begin(115200);
  
}


void loop() {
  // put your main code here, to run repeatedly:
  can1.events();

  static uint32_t timeout = millis();
  if ( millis() - timeout > 200 ) {
    // TODO: send out sensor information
    CAN_message_t msg;
    timeout = millis();
  }
}

// put function definitions here:

void canSniff(const CAN_message_t &msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}