#include <esp_now.h>
#include <WiFi.h>

// ===== PIN CONFIGURATION =====
#define VRY1_PIN 34
#define VRX1_PIN 33
#define VRX2_PIN 32
#define VRY2_PIN 35
#define POT_PIN 25
#define UP_PIN 26
#define DOWN_PIN 18
#define LEFT_PIN 27
#define RIGHT_PIN 19
#define A_PIN 17
#define B_PIN 14
#define X_PIN 12
#define Y_PIN 5
#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 15

// ===== MAC ADDRESS ===== //78:1c:3c:2d:92:e4
const uint8_t RECEIVER_MAC[6] = {0x78, 0x1C, 0x3C, 0x2D, 0x92, 0xE4};

// ===== DATA STRUCTURE =====
#pragma pack(push, 1)
typedef struct {
  int16_t left_x, left_y;
  int16_t right_x, right_y;
  bool up, down, right, left;
  bool a, b, x, y;
  int16_t pot_val;
  int16_t encoder_val;  // Encoder delta value
} RobotCommand;
#pragma pack(pop)

// ===== GLOBALS =====
#define DEBUG true
unsigned long lastSendTime = 0;

// ===== RELIABLE ENCODER VARIABLES =====
volatile int16_t encoderCount = 0;
int lastA = HIGH;

// ===== ESP-NOW CALLBACK =====
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (DEBUG) {
    if (status == ESP_NOW_SEND_SUCCESS) {
    } else {
      Serial.print("✗");
    }
  }
}

// ===== RELIABLE ENCODER INTERRUPT =====
void IRAM_ATTR handleEncoder() {
  int a = digitalRead(ENCODER_A_PIN);
  int b = digitalRead(ENCODER_B_PIN);
  if (a != lastA && a == LOW) {
    encoderCount += (b == HIGH) ? -1 : 1;
  }
  lastA = a;
}

// ===== CALCULATE DATA =====
void calculateJoystickData(RobotCommand &data) {
  static int16_t lastEncoderCount = 0;
  
  int16_t left_x_raw = analogRead(VRY1_PIN);
  int16_t left_y_raw = analogRead(VRX1_PIN);
  int16_t right_x_raw = analogRead(VRX2_PIN);
  int16_t right_y_raw = analogRead(VRY2_PIN);
  
  // Apply simple deadzone of 100
  data.left_x = (abs(left_x_raw - 2047) < 200) ? 2047 : left_x_raw;
  data.left_y = (abs(left_y_raw - 2047) < 200) ? 2047 : left_y_raw;
  data.right_x = (abs(right_x_raw - 2047) < 200) ? 2047 : right_x_raw;
  data.right_y = (abs(right_y_raw - 2047) < 200) ? 2047 : right_y_raw;

  // Read directional buttons
  data.up = !digitalRead(UP_PIN);
  data.down = !digitalRead(DOWN_PIN);
  data.left = !digitalRead(LEFT_PIN);
  data.right = !digitalRead(RIGHT_PIN);

  // Read action buttons
  data.a = !digitalRead(B_PIN);
  data.b = !digitalRead(Y_PIN);
  data.x = !digitalRead(X_PIN);
  data.y = !digitalRead(A_PIN);

  // Calculate encoder delta using your reliable method
  data.encoder_val = encoderCount - lastEncoderCount;
  
  // Only update last count if there was actual movement
  if (data.encoder_val != 0) {
    lastEncoderCount = encoderCount;
  }

  // Debug output
  if (DEBUG) {
    if (data.up) Serial.println("UP button pressed");
    if (data.down) Serial.println("DOWN button pressed");
    if (data.left) Serial.println("LEFT button pressed");
    if (data.right) Serial.println("RIGHT button pressed");
    if (data.a) Serial.println("A button pressed");
    if (data.b) Serial.println("B button pressed");
    if (data.x) Serial.println("X button pressed");
    if (data.y) Serial.println("Y button pressed");
    if (data.encoder_val != 0) {
      Serial.printf("Encoder delta: %d\n", data.encoder_val);
    }
  }
}

// ===== SETUP =====
void setup() {
  if (DEBUG) Serial.begin(115200);
  delay(100);

  // Initialize WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    ESP.restart();
  }

  esp_now_register_send_cb(OnDataSent);

  // Add peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    ESP.restart();
  }

  // Setup pins
  pinMode(VRX1_PIN, INPUT);
  pinMode(VRY1_PIN, INPUT);
  pinMode(VRX2_PIN, INPUT);
  pinMode(VRY2_PIN, INPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);
  pinMode(LEFT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_PIN, INPUT_PULLUP);
  pinMode(A_PIN, INPUT_PULLUP);
  pinMode(B_PIN, INPUT_PULLUP);
  pinMode(X_PIN, INPUT_PULLUP);
  pinMode(Y_PIN, INPUT_PULLUP);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  // Attach interrupt to encoder A only (simpler, more reliable)
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), handleEncoder, CHANGE);

}

// ===== MAIN LOOP =====
void loop() {
  RobotCommand data;
  calculateJoystickData(data);

  // Send ESP-NOW packet
  esp_err_t result = esp_now_send(RECEIVER_MAC, (uint8_t*)&data, sizeof(data));
  if (result == ESP_OK) {
    lastSendTime = millis();
  }

  delay(20); // 50Hz
}
