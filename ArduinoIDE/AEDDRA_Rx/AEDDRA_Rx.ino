#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

// ===== I2C CONFIGURATION =====
#define I2C_SDA 21
#define I2C_SCL 22
#define STM32_I2C_ADDR 0x12
#define DEBUG true

// ===== MAC ADDRESSES ===== //78:1c:3c:2d:92:e4
const uint8_t MCC_Tx_MAC[6] = {0xD8, 0xBC, 0x38, 0xF9, 0x62, 0x94}; // Robot Controller's Tx's MAC

// ===== DATA STRUCTURES =====
#pragma pack(push, 1)
typedef struct {
  int16_t left_x, left_y;
  int16_t right_x, right_y;
  bool up, down, right, left; // UP,DOWN,RIGHT,LEFT buttons
  bool a, b, x, y;
  int16_t pot_val; // Not used
  int16_t encoder_val; // Encoder delta value
} RobotCommand;

typedef struct {
  bool KILL;
  int16_t m1, m2, m3, m4;
  uint8_t base, shoulder, elbow, gripper;
} STM32Command;
#pragma pack(pop)

// ===== SAFE DEFAULTS =====
const STM32Command SAFE_VALUES = {
  .KILL = true,
  .m1 = 0, .m2 = 0, .m3 = 0, .m4 = 0,
  .base = 90, .shoulder = 90, .elbow = 90, .gripper = 90,
};

// ===== GLOBALS =====
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile RobotCommand robotCommand = {};
volatile unsigned long lastCommTime = 0;
float speedMultiplier = 1.0; // For fast/normal mode
const int16_t MOTOR_MIN = -2000;
const int16_t MOTOR_MAX = 2000;
const uint8_t SERVO_MIN = 0;
const uint8_t SERVO_MAX = 180;
const int16_t CENTER = 2048;
const float ENCODER_TICK_TO_DEGREE = 0.9; // Increased for more noticeable changes (tune as needed)
const unsigned long TIMEOUT = 500;
// Persistent servo angles
static uint8_t baseAngle = 90;
static uint8_t shoulderAngle = 90;
static uint8_t elbowAngle = 90;
static uint8_t gripperAngle = 90;

// Currently selected servo (0: none, 1: base, 2: shoulder, 3: elbow, 4: gripper)
static uint8_t selectedServo = 0;

// ===== ESP-NOW CALLBACK =====
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  portENTER_CRITICAL_ISR(&mux);
  if (memcmp(info->src_addr, MCC_Tx_MAC, 6) == 0 && len == sizeof(RobotCommand)) {
    memcpy((void*)&robotCommand, data, sizeof(RobotCommand));
    lastCommTime = millis();
  }
  portEXIT_CRITICAL_ISR(&mux);
}

// ===== SEND TO STM32 VIA I2C =====
void sendToSTM32(const STM32Command &cmd) {
  Wire.beginTransmission(STM32_I2C_ADDR);
  size_t written = Wire.write((const uint8_t*)&cmd, sizeof(cmd));
  uint8_t result = Wire.endTransmission();

  if (result != 0 || written != sizeof(cmd)) {
    if (DEBUG) {
      Serial.printf("I2C Error: %d (Written: %d/%d)\n", result, written, sizeof(cmd));
    }
  }
}

// ===== NORMALIZE AND CLAMP FUNCTIONS =====
int16_t normalize(int16_t raw_value) {
  const int16_t deadzone = 200; // Adjust as needed
  int16_t offset = raw_value - CENTER;
  return (abs(offset) < deadzone) ? 0 : offset;
}

int16_t clampMotor(int16_t value) {
  return constrain(value * speedMultiplier, MOTOR_MIN, MOTOR_MAX);
}

uint8_t clampServo(int16_t value) {
  return constrain(value, SERVO_MIN, SERVO_MAX);
}

// ===== SETUP =====
void setup() {
  if (DEBUG) Serial.begin(115200);
  delay(100);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  if (esp_now_init() != ESP_OK) {
    if (DEBUG) Serial.println("ESP-NOW Init Failed");
    delay(100);
    ESP.restart();
  }
  esp_now_register_recv_cb(OnDataRecv);
  if (DEBUG) Serial.println("ESP-NOW Initialized");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  if (DEBUG) Serial.println("Receiver Ready");
}

// ===== MAIN LOOP =====
void loop() {
  static unsigned long lastSendTime = 0;
  static unsigned long lastPrintTime = 0;
  static STM32Command lastCmd = SAFE_VALUES;

  RobotCommand currentCommand;

  portENTER_CRITICAL(&mux);
  memcpy(&currentCommand, (const void*)&robotCommand, sizeof(RobotCommand));
  portEXIT_CRITICAL(&mux);

  bool robotActive = (millis() - lastCommTime) < TIMEOUT;
  STM32Command cmd = robotActive ? lastCmd : SAFE_VALUES;
  cmd.KILL = !robotActive;

  if (robotActive) {
    // Handle joystick inputs
    int16_t r_x = currentCommand.right_x;
    int16_t r_y = currentCommand.right_y;
    int16_t l_x = currentCommand.left_x;
    int16_t l_y = currentCommand.left_y;

    int16_t Vx = normalize(l_y);
    int16_t Vy = -normalize(l_x);
    int16_t Wz = normalize(r_y);

    // Calculate and clamp motor values
    cmd.m1 = clampMotor(Vx + Vy + Wz);
    cmd.m2 = clampMotor(Vx - Vy + Wz);
    cmd.m3 = clampMotor(Vx - Vy - Wz);
    cmd.m4 = clampMotor(Vx + Vy - Wz);

    // Handle speed mode (UP/DOWN buttons)
    static bool lastUpState = false;
    static bool lastDownState = false;
    if (currentCommand.up && !lastUpState) {
      speedMultiplier = 1.0; // Fast mode
    }
    if (currentCommand.down && !lastDownState) {
      speedMultiplier = 0.5; // Half speed
    }
    lastUpState = currentCommand.up;
    lastDownState = currentCommand.down;

    // Handle servo selection (A, B, X, Y buttons)
    static bool lastAState = false;
    static bool lastBState = false;
    static bool lastXState = false;
    static bool lastYState = false;

    if (currentCommand.a && !lastAState) {
      selectedServo = 1; // Select base servo
    }
    if (currentCommand.b && !lastBState) {
      selectedServo = 2; // Select shoulder servo
    }
    if (currentCommand.x && !lastXState) {
      selectedServo = 3; // Select elbow servo
    }
    if (currentCommand.y && !lastYState) {
      selectedServo = 4; // Select gripper servo
    }
    lastAState = currentCommand.a;
    lastBState = currentCommand.b;
    lastXState = currentCommand.x;
    lastYState = currentCommand.y;

    // Handle encoder input for selected servo (use received encoder_val directly as delta)
    int16_t angleChange = int(currentCommand.encoder_val * ENCODER_TICK_TO_DEGREE);

    switch (selectedServo) {
      case 1: // Base
        baseAngle = clampServo(baseAngle + angleChange);
        break;
      case 2: // Shoulder
        shoulderAngle = clampServo(shoulderAngle + angleChange);
        break;
      case 3: // Elbow
        elbowAngle = clampServo(elbowAngle + angleChange);
        break;
      case 4: // Gripper
        gripperAngle = clampServo(gripperAngle + angleChange);
        break;
      default:
        break; // No servo selected
    }

    // Update command with current servo angles
    cmd.base = baseAngle;
    cmd.shoulder = shoulderAngle;
    cmd.elbow = elbowAngle;
    cmd.gripper = gripperAngle;

  } else {
    // Use safe values if communication is lost
    cmd = SAFE_VALUES;
    cmd.KILL = true;
    speedMultiplier = 1.0;
    selectedServo = 0; // Reset servo selection
  }

  // Send to STM32 at 50Hz
  if (millis() - lastSendTime >= 20) {
    sendToSTM32(cmd); 
    lastCmd = cmd;
    lastSendTime = millis();
  }

  // Debug print every 200ms
  if (DEBUG && millis() - lastPrintTime >= 200) {
    lastPrintTime = millis();
    Serial.printf("Robot: L_X=%d L_Y=%d R_X=%d SpeedMode=%.1f Active=%d Servo=%d EncoderDelta=%d AngleChange=%d Base=%d Shoulder=%d Elbow=%d Gripper=%d\n",
                  currentCommand.left_x, currentCommand.left_y,
                  currentCommand.right_x, speedMultiplier, robotActive,
                  selectedServo, currentCommand.encoder_val, (int)(currentCommand.encoder_val * ENCODER_TICK_TO_DEGREE),
                  cmd.base, cmd.shoulder, cmd.elbow, cmd.gripper);
  }

  delay(1);
} 
