/*
  Integrated Haptic + Sensor Node (ESP32-S2)
  
  HAPTIC SYSTEM:
  - PCA9548A + 6x DRV2605L on I2C0 (pins 40/41)
  - Receives ESP-NOW haptic commands from Communication Unit
  
  SENSOR SYSTEM:
  - BNO055 on I2C1 (pins 6/7) with 80cm cable
  - Sends ESP-NOW sensor data to Communication Unit
  - Same payload format as original bracelet code
  
  DUAL FUNCTIONALITY:
  - Acts as haptic receiver (from Communication Unit)
  - Acts as sensor sender (to Communication Unit)
*/

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_DRV2605.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

// ================= Network Config =================
static const int ESP_NOW_CHANNEL = 6;

#define USE_CUSTOM_MAC 1
static const uint8_t CUSTOM_MAC[6] = { 0x02, 0x00, 0x00, 0xAA, 0xBB, 0x01 };

// Communication Unit MAC (where we send sensor data)
static const uint8_t COMM_UNIT_MAC[6] = { 0x02, 0x00, 0x00, 0xAA, 0xCC, 0x00 };

// ================= Haptic System Config =================
// I2C0 for haptic system
static const int HAPTIC_SDA = 41;
static const int HAPTIC_SCL = 40;

static const uint8_t MUX_ADDR = 0x70;
static const uint8_t NUM_MOTORS = 6;
static const uint8_t MUX_CH[NUM_MOTORS] = {1,2,3,4,5,6};

// ================= Sensor System Config =================
// I2C1 for BNO055
static const int SENSOR_SDA = 7;  // Adjust to your actual pins
static const int SENSOR_SCL = 6;  // Adjust to your actual pins

// ================= Payload Structures =================
// Haptic command (received from Communication Unit)
typedef struct __attribute__((packed)) {
  uint8_t  idx;         // 0..5 or 255=ALL
  uint8_t  amp127;      // 0..127
  uint16_t durationMs;  // 0..30000
  uint8_t  reserved;
} HapticRtpCmdIndexed;

// Sensor data (sent to Communication Unit)
typedef struct __attribute__((packed)) {
  int   id;
  float anglex, angley, anglez;
  float accx,  accy,  accz;
  int   battery;
} BraceletMsg;

// ================= Global Variables =================
// Haptic system
Adafruit_DRV2605 drv[NUM_MOTORS];
bool drv_ok[NUM_MOTORS] = {false};
static volatile bool g_active[NUM_MOTORS] = {false};
static volatile uint32_t g_offAt[NUM_MOTORS] = {0};
static portMUX_TYPE g_spin = portMUX_INITIALIZER_UNLOCKED;

// Sensor system
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
unsigned long sensor_timer = 0;
const unsigned long SENSOR_INTERVAL = 50; // 10ms = 100Hz

// ================= Haptic Functions =================
static inline void printMac(const uint8_t mac[6], const char* label) {
  Serial.printf("%s%02X:%02X:%02X:%02X:%02X:%02X\n",
    label, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static inline void muxSelect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delayMicroseconds(100);
}

static bool initMotor(uint8_t i) {
  Serial.printf("Initializing motor %u on mux channel %u...\n", i, MUX_CH[i]);
  
  muxSelect(MUX_CH[i]);
  delay(10);
  
  if (!drv[i].begin()) {
    Serial.printf("DRV[%u] NOT FOUND on mux ch %u\n", i, MUX_CH[i]);
    return false;
  }
  
  // Configure for ERM motors
  drv[i].useERM();
  drv[i].selectLibrary(1); // ERM library
  drv[i].setMode(DRV2605_MODE_REALTIME);
  drv[i].setRealtimeValue(0);
  
  Serial.printf("DRV[%u] OK on mux ch %u (ERM mode)\n", i, MUX_CH[i]);
  return true;
}

static inline void motorSetAmp(uint8_t i, uint8_t amp127) {
  if (!drv_ok[i]) return;
  muxSelect(MUX_CH[i]);
  drv[i].setRealtimeValue(amp127);
}

static inline void motorOff(uint8_t i) {
  if (!drv_ok[i]) return;
  muxSelect(MUX_CH[i]);
  drv[i].setRealtimeValue(0);
  
  portENTER_CRITICAL(&g_spin);
  g_active[i] = false;
  portEXIT_CRITICAL(&g_spin);
}

static void activateMotors(uint8_t idx, uint8_t amp127, uint16_t durationMs) {
  uint32_t now = millis();
  uint32_t offTime = now + durationMs;
  
  if (idx == 255) {
    // ALL motors
    
    
    portENTER_CRITICAL(&g_spin);
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (drv_ok[i]) {
        g_active[i] = true;
        g_offAt[i] = offTime;
      }
    }
    portEXIT_CRITICAL(&g_spin);
    
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      if (drv_ok[i]) {
        motorSetAmp(i, amp127);
      }
    }
  } else if (idx < NUM_MOTORS) {
    // Single motor
    
    
    if (drv_ok[idx]) {
      portENTER_CRITICAL(&g_spin);
      g_active[idx] = true;
      g_offAt[idx] = offTime;
      portEXIT_CRITICAL(&g_spin);
      
      motorSetAmp(idx, amp127);
    }
  }
}

// ================= Sensor Functions =================
static bool initSensor() {
  Serial.println("Initializing BNO055 sensor...");
  
  Wire1.begin(SENSOR_SDA, SENSOR_SCL);
  Wire1.setClock(100000); // 100kHz for 80cm cable
  
  // Test I2C connectivity first
  Wire1.beginTransmission(0x28);
  uint8_t error = Wire1.endTransmission();
  if (error != 0) {
    Serial.printf("BNO055 I2C error: %u\n", error);
    return false;
  }
  
  if (!bno.begin()) {
    Serial.println("BNO055 initialization failed");
    return false;
  }
  
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 initialized successfully");
  return true;
}

static void sendSensorData() {
  // Read sensor data
  sensors_event_t euler, lin;
  bno.getEvent(&euler, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&lin, Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  // Read battery
  int battery_raw = analogRead(A2);
  int battery_mv = battery_raw * 5000 / 4096;
  
  // Fill payload
  BraceletMsg sensorData;
  sensorData.id = 1;
  sensorData.anglex = euler.orientation.x;
  sensorData.angley = euler.orientation.y;
  sensorData.anglez = euler.orientation.z;
  sensorData.accx = lin.acceleration.x;
  sensorData.accy = lin.acceleration.y;
  sensorData.accz = lin.acceleration.z;
  sensorData.battery = battery_mv;
  
  // Send to Communication Unit
  esp_now_send(COMM_UNIT_MAC, (const uint8_t*)&sensorData, sizeof(sensorData));
}
// ================= ESP-NOW Callbacks =================
static void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  // Handle incoming haptic commands
  if (len == sizeof(HapticRtpCmdIndexed)) {
    HapticRtpCmdIndexed cmd;
    memcpy(&cmd, data, sizeof(cmd));
    
    // Process haptic command immediately
    activateMotors(cmd.idx, cmd.amp127, cmd.durationMs);
    
    Serial.printf("Haptic cmd: idx=%u amp=%u dur=%u\n", 
                  cmd.idx, cmd.amp127, cmd.durationMs);
  }
}

static void onEspNowSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional: monitor sensor data transmission status
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("Sensor data send failed");
  }
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Integrated Haptic + Sensor Node starting...");
  
  // Initialize I2C buses
  // I2C0 for haptic system (400kHz)
  Wire.begin(HAPTIC_SDA, HAPTIC_SCL);
  Wire.setClock(400000);
  
  // Test multiplexer
  Serial.println("Testing PCA9548A multiplexer...");
  Wire.beginTransmission(MUX_ADDR);
  uint8_t mux_error = Wire.endTransmission();
  if (mux_error == 0) {
    Serial.printf("PCA9548A found at address 0x%02X\n", MUX_ADDR);
  } else {
    Serial.printf("PCA9548A NOT found (error: %u)\n", mux_error);
  }
  
  // Initialize haptic motors
  Serial.println("Initializing haptic motors...");
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    drv_ok[i] = initMotor(i);
    delay(50);
  }
  
  uint8_t motor_count = 0;
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (drv_ok[i]) motor_count++;
  }
  Serial.printf("Haptic system: %u/%u motors ready\n", motor_count, NUM_MOTORS);
  
  // Initialize sensor system
  bool sensor_ok = initSensor();
  Serial.printf("Sensor system: %s\n", sensor_ok ? "Ready" : "Failed");
  
  // Initialize WiFi and ESP-NOW
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  
#if USE_CUSTOM_MAC
  esp_err_t e = esp_wifi_set_mac(WIFI_IF_STA, CUSTOM_MAC);
  Serial.printf("SET CUSTOM MAC -> %d\n", (int)e);
#endif
  
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  printMac(mac, "NODE MAC: ");
  
  esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(500);
  }
  
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);
  
  // Add Communication Unit as peer for sending sensor data
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, COMM_UNIT_MAC, 6);
  peerInfo.channel = ESP_NOW_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println("Communication Unit added as peer");
  } else {
    Serial.println("Failed to add Communication Unit peer");
  }
  
  Serial.println("Integrated node ready!");
  Serial.println("- Receiving haptic commands from Communication Unit");
  Serial.println("- Sending sensor data to Communication Unit");
}

// ================= Main Loop =================
void loop() {
  uint32_t now = millis();
  
  // Handle haptic motor timeouts
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    bool shouldStop = false;
    
    portENTER_CRITICAL(&g_spin);
    if (g_active[i] && (int32_t)(now - g_offAt[i]) >= 0) {
      shouldStop = true;
    }
    portEXIT_CRITICAL(&g_spin);
    
    if (shouldStop) {
      motorOff(i);
    }
  }
  
  // Send sensor data at regular intervals
  if (now - sensor_timer >= SENSOR_INTERVAL) {
    sensor_timer = now;
    sendSensorData();
  }
  
  delay(1);
}