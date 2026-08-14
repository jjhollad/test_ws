/*
 * 4-Channel Relay Controller for Arduino DUE
 * Communicates with ROS2 via Serial
 * 
 * Commands:
 * RELAY1_ON,RELAY2_ON,RELAY3_ON,RELAY4_ON - Turn on relays
 * RELAY1_OFF,RELAY2_OFF,RELAY3_OFF,RELAY4_OFF - Turn off relays
 * STATUS - Get relay status
 *
 * Optional MPU-6050:
 * Publishes IMU:ax,ay,az,gx,gy,gz at IMU_PUBLISH_HZ.
 * Acceleration is m/s^2. Angular velocity is rad/s.
 */

#include <Wire.h>

// Relay control pins
const int RELAY1_PIN = 2;
const int RELAY2_PIN = 3;
const int RELAY3_PIN = 4;
const int RELAY4_PIN = 5;

// MPU-6050 registers and scaling for +/-2g, +/-250 deg/s.
const byte MPU6050_ADDR = 0x68;
const byte MPU6050_WHO_AM_I = 0x75;
const byte MPU6050_PWR_MGMT_1 = 0x6B;
const byte MPU6050_SMPLRT_DIV = 0x19;
const byte MPU6050_CONFIG = 0x1A;
const byte MPU6050_GYRO_CONFIG = 0x1B;
const byte MPU6050_ACCEL_CONFIG = 0x1C;
const byte MPU6050_ACCEL_XOUT_H = 0x3B;
const float GRAVITY_MPS2 = 9.80665;
const float ACCEL_LSB_PER_G = 16384.0;
const float GYRO_LSB_PER_DPS = 131.0;
const float DEG_TO_RAD_F = 0.017453292519943295;
const unsigned long IMU_PUBLISH_PERIOD_MS = 20;  // 50 Hz

// Stationary accel calibration captured with the robot level on 2026-07-14.
// Bias is measured_value - expected_value in m/s^2.
const float ACCEL_X_BIAS_MPS2 = 0.663426;
const float ACCEL_Y_BIAS_MPS2 = -0.037355;
const float ACCEL_Z_BIAS_MPS2 = -2.212764;

// Relay states
bool relay1_state = false;   // Start with relays OFF
bool relay2_state = false;
bool relay3_state = false;
bool relay4_state = false;

// Serial communication
String inputString = "";
boolean stringComplete = false;

// IMU state
bool imu_available = false;
float gyro_x_bias = 0.0;
float gyro_y_bias = 0.0;
float gyro_z_bias = 0.0;
unsigned long last_imu_publish_ms = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  
  // Initialize relay pins
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  
  // Turn off all relays initially (inverted logic: HIGH = OFF)
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  digitalWrite(RELAY3_PIN, HIGH);
  digitalWrite(RELAY4_PIN, HIGH);
  
  // Reserve space for input string
  inputString.reserve(100);

  imu_available = initMpu6050();
  if (imu_available) {
    calibrateGyro();
    Serial.println("MPU6050_READY");
  } else {
    Serial.println("MPU6050_NOT_FOUND");
  }
  
  // Send ready message
  Serial.println("RELAY_CONTROLLER_READY");
}

void loop() {
  // Process serial commands
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }

  publishImuIfDue();
  
  // Small delay
  delay(2);
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n' || inChar == '\r') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  // Handle concatenated commands by splitting them
  if (command.indexOf("RELAY") != -1 && command.length() > 15) {
    // Likely concatenated commands, try to split them
    String remaining = command;
    while (remaining.length() > 0) {
      int nextRelay = remaining.indexOf("RELAY", 1);
      String singleCommand;
      
      if (nextRelay > 0) {
        singleCommand = remaining.substring(0, nextRelay);
        remaining = remaining.substring(nextRelay);
      } else {
        singleCommand = remaining;
        remaining = "";
      }
      
      if (singleCommand.length() > 0) {
        processSingleCommand(singleCommand);
      }
    }
    return;
  }
  
  processSingleCommand(command);
}

void processSingleCommand(String command) {
  if (command == "RELAY1_ON") {
    setRelay(1, true);
  }
  else if (command == "RELAY1_OFF") {
    setRelay(1, false);
  }
  else if (command == "RELAY2_ON") {
    setRelay(2, true);
  }
  else if (command == "RELAY2_OFF") {
    setRelay(2, false);
  }
  else if (command == "RELAY3_ON") {
    setRelay(3, true);
  }
  else if (command == "RELAY3_OFF") {
    setRelay(3, false);
  }
  else if (command == "RELAY4_ON") {
    setRelay(4, true);
  }
  else if (command == "RELAY4_OFF") {
    setRelay(4, false);
  }
  else if (command == "STATUS") {
    sendStatus();
  }
  else if (command == "IMU_STATUS") {
    Serial.println(String("IMU_STATUS:") + (imu_available ? "READY" : "NOT_FOUND"));
  }
  else if (command == "ALL_ON") {
    setRelay(1, true);
    setRelay(2, true);
    setRelay(3, true);
    setRelay(4, true);
  }
  else if (command == "ALL_OFF") {
    setRelay(1, false);
    setRelay(2, false);
    setRelay(3, false);
    setRelay(4, false);
  }
  else if (command.length() > 0) {
    Serial.println("ERROR: Unknown command: " + command);
  }
}

void setRelay(int relayNum, bool state) {
  int pin;
  bool* relayState;
  
  switch (relayNum) {
    case 1:
      pin = RELAY1_PIN;
      relayState = &relay1_state;
      break;
    case 2:
      pin = RELAY2_PIN;
      relayState = &relay2_state;
      break;
    case 3:
      pin = RELAY3_PIN;
      relayState = &relay3_state;
      break;
    case 4:
      pin = RELAY4_PIN;
      relayState = &relay4_state;
      break;
    default:
      Serial.println("ERROR: Invalid relay number: " + String(relayNum));
      return;
  }
  
  // Set relay state (inverted logic: LOW = ON, HIGH = OFF)
  *relayState = state;
  digitalWrite(pin, state ? LOW : HIGH);
  
  // Send confirmation
  Serial.println("RELAY" + String(relayNum) + ":" + (state ? "ON" : "OFF"));
}

void sendStatus() {
  Serial.println("STATUS:" + 
                 String(relay1_state ? "1" : "0") + "," +
                 String(relay2_state ? "1" : "0") + "," +
                 String(relay3_state ? "1" : "0") + "," +
                 String(relay4_state ? "1" : "0"));
}

void sendError(String error) {
  Serial.println("ERROR: " + error);
}

bool writeMpuRegister(byte reg, byte value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMpuRegisters(byte startReg, byte count, byte* buffer) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  byte received = Wire.requestFrom(MPU6050_ADDR, count);
  if (received != count) {
    return false;
  }

  for (byte i = 0; i < count && Wire.available(); i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

int16_t combineBytes(byte highByte, byte lowByte) {
  return (int16_t)((highByte << 8) | lowByte);
}

bool initMpu6050() {
  byte whoami = 0;
  if (!readMpuRegisters(MPU6050_WHO_AM_I, 1, &whoami)) {
    return false;
  }
  if (whoami != 0x68) {
    return false;
  }

  // Wake device, use gyro X PLL clock, 100 Hz sample rate, light DLPF.
  if (!writeMpuRegister(MPU6050_PWR_MGMT_1, 0x01)) return false;
  delay(100);
  if (!writeMpuRegister(MPU6050_SMPLRT_DIV, 9)) return false;
  if (!writeMpuRegister(MPU6050_CONFIG, 0x03)) return false;
  if (!writeMpuRegister(MPU6050_GYRO_CONFIG, 0x00)) return false;
  if (!writeMpuRegister(MPU6050_ACCEL_CONFIG, 0x00)) return false;
  delay(100);
  return true;
}

bool readMpuRaw(int16_t& ax, int16_t& ay, int16_t& az,
                int16_t& gx, int16_t& gy, int16_t& gz) {
  byte data[14];
  if (!readMpuRegisters(MPU6050_ACCEL_XOUT_H, 14, data)) {
    return false;
  }

  ax = combineBytes(data[0], data[1]);
  ay = combineBytes(data[2], data[3]);
  az = combineBytes(data[4], data[5]);
  gx = combineBytes(data[8], data[9]);
  gy = combineBytes(data[10], data[11]);
  gz = combineBytes(data[12], data[13]);
  return true;
}

void calibrateGyro() {
  const int samples = 500;
  long gx_sum = 0;
  long gy_sum = 0;
  long gz_sum = 0;
  int valid_samples = 0;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    if (readMpuRaw(ax, ay, az, gx, gy, gz)) {
      gx_sum += gx;
      gy_sum += gy;
      gz_sum += gz;
      valid_samples++;
    }
    delay(3);
  }

  if (valid_samples > 0) {
    gyro_x_bias = (float)gx_sum / valid_samples;
    gyro_y_bias = (float)gy_sum / valid_samples;
    gyro_z_bias = (float)gz_sum / valid_samples;
  }
}

void publishImuIfDue() {
  if (!imu_available) {
    return;
  }

  unsigned long now_ms = millis();
  if (now_ms - last_imu_publish_ms < IMU_PUBLISH_PERIOD_MS) {
    return;
  }
  last_imu_publish_ms = now_ms;

  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  if (!readMpuRaw(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw)) {
    Serial.println("ERROR: MPU6050 read failed");
    return;
  }

  float ax = (((float)ax_raw / ACCEL_LSB_PER_G) * GRAVITY_MPS2) - ACCEL_X_BIAS_MPS2;
  float ay = (((float)ay_raw / ACCEL_LSB_PER_G) * GRAVITY_MPS2) - ACCEL_Y_BIAS_MPS2;
  float az = (((float)az_raw / ACCEL_LSB_PER_G) * GRAVITY_MPS2) - ACCEL_Z_BIAS_MPS2;
  float gx = (((float)gx_raw - gyro_x_bias) / GYRO_LSB_PER_DPS) * DEG_TO_RAD_F;
  float gy = (((float)gy_raw - gyro_y_bias) / GYRO_LSB_PER_DPS) * DEG_TO_RAD_F;
  float gz = (((float)gz_raw - gyro_z_bias) / GYRO_LSB_PER_DPS) * DEG_TO_RAD_F;

  Serial.print("IMU:");
  Serial.print(ax, 5);
  Serial.print(",");
  Serial.print(ay, 5);
  Serial.print(",");
  Serial.print(az, 5);
  Serial.print(",");
  Serial.print(gx, 6);
  Serial.print(",");
  Serial.print(gy, 6);
  Serial.print(",");
  Serial.println(gz, 6);
}
