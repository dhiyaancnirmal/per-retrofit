#include <Arduino.h>
#include "FlexCAN_T4.h"
#include "DFRobot_BMM350.h"

// Pins for CAN1: (CRX1 -> pin 23) (CTX1 --> pin 22)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; // Use CAN 2.0 constructor template


// Pins for I2C: (SCL --> pin 19) (SDA --> pin 18)
Wire.begin()
DFRobot_BMM350_I2C bmm350(&Wire, I2C_ADDRESS);

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
  while(!Serial);
  while(bmm350.begin()){
    Serial.println("bmm350 init failed, Please try again!");
    delay(1000);
  } Serial.println("bmm350 init success!");

  Serial.println(bmm350.selfTest());
  bmm350.setOperationMode(BMM350_NORMAL_MODE);
  bmm350.setPresetMode(BMM350_PRESETMODE_HIGHACCURACY);
  bmm350.setRate(BMM350_DATA_RATE_25HZ);
  bmm350.setMeasurementXYZ();
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

  // Geomagnetic data from BMM350 Sensor
  // TODO: save sensor information and send through CAN bus instead
  sBmm350MagData_t magData = bmm350.getGeomagneticData();
  Serial.print("mag x = "); Serial.print(magData.x); Serial.println(" uT");
  Serial.print("mag y = "); Serial.print(magData.y); Serial.println(" uT");
  Serial.print("mag z = "); Serial.print(magData.z); Serial.println(" uT");

  float compassDegree = bmm350.getCompassDegree();
  Serial.print("the angle between the pointing direction and north (counterclockwise) is:");
  Serial.println(compassDegree);
  Serial.println("--------------------------------");
  delay(3000);
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