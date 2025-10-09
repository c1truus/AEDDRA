#include "LoRa_E32.h"

// HardwareSerial & pins for ESP32 UART2
HardwareSerial Serial2(2);  // UART2 on pins 16(RX)/17(TX)
#define M0_PIN 18
#define M1_PIN 19
#define AUX_PIN 5

LoRa_E32 e32ttl100(&Serial2, M0_PIN, M1_PIN, AUX_PIN);

void setup() {
  Serial.begin(9600);  // Debug output
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // UART2: RX=16, TX=17, 9600 baud
  delay(500);

  e32ttl100.begin();
  ResponseStructContainer rc = e32ttl100.getModuleInformation();  // Fixed return type
  Serial.println(rc.status.getResponseDescription());  // Access .status
  Serial.println("Transmitter ready. Sending messages...");
}

void loop() {
  // Send fixed message (transparent mode)
  ResponseStatus rs = e32ttl100.sendFixedMessage(0, 0, 0x17, "Hello LoRa!");  // Target addr 0x0000 (high/low byte), channel 0x17
  Serial.println(rs.getResponseDescription());

  delay(5000);  // Send every 5s
}
