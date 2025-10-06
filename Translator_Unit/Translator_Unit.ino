/*
  Integrated Communication Unit (ESP32-S2)
  
  BIDIRECTIONAL COMMUNICATION:
  - Receives sensor data from Integrated Haptic+Sensor Node via ESP-NOW
  - Sends normalized sensor data to Python via SLIP-OSC  
  - Receives haptic commands from Python via SLIP-OSC
  - Forwards haptic commands to Integrated Node via ESP-NOW
  
  This replaces both the old Communication Unit and Bracelet receiver functionality
*/

#define DEBUG_ASCII 0  // 1 = ASCII debug prints instead of SLIP-OSC

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <math.h>

// ================= Custom MAC =================
#define USE_CUSTOM_MAC 1
static const uint8_t CUSTOM_MAC[6] = { 0x02, 0x00, 0x00, 0xAA, 0xCC, 0x00 };

// ======= Radio / Ranges =======
static const int   ESP_NOW_CHANNEL = 6;
static const float ACC_MIN = -15.0f, ACC_MAX = 15.0f;
static const float BAT_MIN = 3300.0f, BAT_MAX = 4200.0f;

// ================= Payload Structures =================
// Sensor data (received from Integrated Node)
typedef struct __attribute__((packed)) {
  int   id;
  float anglex, angley, anglez;
  float accx,  accy,  accz;
  int   battery; // mV
} BraceletMsg;

// Haptic command (sent to Integrated Node)
typedef struct __attribute__((packed)) {
  uint8_t  idx;         // 0..5 or 255=ALL
  uint8_t  amp127;      // 0..127
  uint16_t durationMs;  // 0..30000
  uint8_t  reserved;
} HapticRtpCmdIndexed;

// ======== Peer addresses ========
uint8_t INTEGRATED_NODE_MAC[6] = { 0x02, 0x00, 0x00, 0xAA, 0xBB, 0x01 };

// ================= SLIP (output) =================
namespace SLIP {
  static const uint8_t END=0xC0, ESC=0xDB, ESC_END=0xDC, ESC_ESC=0xDD;
  class OutputStream : public Stream {
  public:
    explicit OutputStream(Stream &out) : _out(out) {}
    void beginPacket(){ _out.write(END); }
    void endPacket(){ _out.write(END); }
    size_t write(uint8_t b) override {
      if (b==END){ _out.write(ESC); _out.write(ESC_END); }
      else if (b==ESC){ _out.write(ESC); _out.write(ESC_ESC); }
      else { _out.write(b); }
      return 1;
    }
    size_t write(const uint8_t*buf,size_t len) override { 
      for(size_t i=0;i<len;i++) write(buf[i]); 
      return len; 
    }
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
    void flush() override { _out.flush(); }
  private: Stream &_out;
  };
}
SLIP::OutputStream SLIPOut(Serial);

// ================= SLIP (input) for OSC commands =================
namespace SLIPIN {
  static const uint8_t END=0xC0, ESC=0xDB, ESC_END=0xDC, ESC_ESC=0xDD;
  struct Decoder {
    uint8_t buf[512];
    size_t  len=0;
    bool    esc=false;
    void reset(){ len=0; esc=false; }
    bool feed(int c){
      if (c < 0) return false;
      uint8_t b=(uint8_t)c;
      if (b==END) {
        if (len > 0) return true;
        reset();
        return false;
      }
      if (esc){
        if (b==ESC_END) b=END; 
        else if (b==ESC_ESC) b=ESC;
        esc=false;
      } else if (b==ESC){ 
        esc=true; 
        return false; 
      }
      if (len < sizeof(buf)-1) buf[len++]=b;
      return false;
    }
  };
}
static SLIPIN::Decoder slipIn;

// ================= Queue & stats =================
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
static QueueHandle_t g_rxQueue = nullptr;
static volatile uint32_t g_dropCount = 0;

// ================= Helpers =================
static inline void printMac(const uint8_t mac[6], const char* label) {
  Serial.printf("%s%02X:%02X:%02X:%02X:%02X:%02X\n",
                label, mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}
static inline float clampf(float v, float lo, float hi){ 
  return v < lo ? lo : (v > hi ? hi : v); 
}
static inline float norm_linear(float v, float lo, float hi){
  return clampf((v - lo) / (hi - lo), 0.0f, 1.0f);
}

// ================= ESP-NOW Receive (Sensor Data) =================
static void onSensorDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(BraceletMsg)) return;
  
  BraceletMsg msg;
  memcpy(&msg, data, sizeof(msg));
  
  // Queue sensor data for processing
  if (g_rxQueue && xQueueSend(g_rxQueue, &msg, 0) != pdTRUE) {
    g_dropCount++;
  }
}
// ================= ESP-NOW Send (Haptic Commands) =================
static void onHapticDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.printf("Haptic command send failed to %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  }
}

static bool sendHapticCommand(const HapticRtpCmdIndexed& cmd) {
  return esp_now_send(INTEGRATED_NODE_MAC, (const uint8_t*)&cmd, sizeof(cmd)) == ESP_OK;
}

// ================= OSC send helpers =================
static void sendOscHello(){
#if DEBUG_ASCII
  Serial.println("HELLO 1");
#else
  OSCMessage m("/hello"); m.add((int32_t)1);
  SLIPOut.beginPacket(); m.send(SLIPOut); SLIPOut.endPacket(); m.empty();
#endif
}

static void sendOscHeartbeat(){
#if DEBUG_ASCII
  Serial.println("HB 1");
#else
  OSCMessage m("/hb"); m.add((int32_t)1);
  SLIPOut.beginPacket(); m.send(SLIPOut); SLIPOut.endPacket(); m.empty();
#endif
}

static void sendOscBundleFromMsg(const BraceletMsg& msg) {
  // Normalize sensor values
  float nx = norm_linear(clampf(msg.anglex,  0.0f, 360.0f),   0.0f, 360.0f);
  float ny = norm_linear(clampf(msg.angley, -180.0f, 180.0f), -180.0f, 180.0f);
  float nz = norm_linear(clampf(msg.anglez, -180.0f, 180.0f), -180.0f, 180.0f);
  float nAx = norm_linear(clampf(msg.accx, ACC_MIN, ACC_MAX), ACC_MIN, ACC_MAX);
  float nAy = norm_linear(clampf(msg.accy, ACC_MIN, ACC_MAX), ACC_MIN, ACC_MAX);
  float nAz = norm_linear(clampf(msg.accz, ACC_MIN, ACC_MAX), ACC_MIN, ACC_MAX);
  float nBat = norm_linear((float)msg.battery, BAT_MIN, BAT_MAX);

#if DEBUG_ASCII
  Serial.printf("SENSOR id=%d ang_n=(%.3f,%.3f,%.3f) acc_n=(%.3f,%.3f,%.3f) bat_n=%.3f\n",
    msg.id, nx,ny,nz, nAx,nAy,nAz, nBat);
  return;
#endif

  OSCBundle b;
  
  // Send device ID
  b.add("/sensor/id").add((int32_t)msg.id);

  // Send normalized orientation
  OSCMessage mAngN("/sensor/angles_n");
  mAngN.add((int32_t)msg.id).add(nx).add(ny).add(nz);
  b.add(mAngN);

  // Send normalized acceleration
  OSCMessage mAccN("/sensor/accel_n");
  mAccN.add((int32_t)msg.id).add(nAx).add(nAy).add(nAz);
  b.add(mAccN);

  // Send normalized battery
  OSCMessage mBatN("/sensor/battery_n");
  mBatN.add((int32_t)msg.id).add(nBat);
  b.add(mBatN);

  SLIPOut.beginPacket(); b.send(SLIPOut); SLIPOut.endPacket(); b.empty();
}

// ================= OSC IN: /haptic from USB =================
static void handleOscMsg(OSCMessage& m){
  // /haptic idx:int, intensity:float (0..1), duration_ms:int
  if (m.fullMatch("/haptic")){
    if (m.size() >= 3 && m.isInt(0) && (m.isFloat(1) || m.isInt(1)) && m.isInt(2)) {
      int32_t idx       = m.getInt(0);
      float   intensity = m.isFloat(1) ? m.getFloat(1) : (float)m.getInt(1);
      int32_t duration  = m.getInt(2);

      // Convert and clamp values
      uint8_t amp = (uint8_t)roundf(clampf(intensity, 0.f, 1.f) * 127.f);
      uint16_t dur = (uint16_t)clampf((float)duration, 0.f, 30000.f);

      // Map index: -1 becomes 255 (ALL), 0-5 stay as is
      uint8_t motorIdx = 255; // default ALL
      if (idx >= 0) {
        motorIdx = (uint8_t)((idx < 6) ? idx : 5);
      }

      HapticRtpCmdIndexed cmd{ motorIdx, amp, dur, 0 };


      // Send to Integrated Node
      if (!sendHapticCommand(cmd)) {
        Serial.println("Failed to send haptic command");
      }
    }
  }
}

static void pollSerialOSC(){
  while (Serial.available()){
    int c = Serial.read();
    if (slipIn.feed(c)) {
      // Complete SLIP packet received
      OSCMessage in;
      for (size_t i = 0; i < slipIn.len; i++) {
        in.fill(slipIn.buf[i]);
      }
      
      if (!in.hasError()) {
        handleOscMsg(in);
      }
      slipIn.reset();
    }
  }
}

// ================= Peer Management =================
static void addIntegratedNodePeer(){
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, INTEGRATED_NODE_MAC, 6);
  p.channel = ESP_NOW_CHANNEL;
  p.encrypt = false;
  p.ifidx = WIFI_IF_STA;
  
  esp_err_t result = esp_now_add_peer(&p);
  if (result == ESP_OK) {
    Serial.println("Added Integrated Node as peer");
    printMac(INTEGRATED_NODE_MAC, "  -> ");
  } else {
    Serial.printf("Failed to add peer: %d\n", result);
  }
}

// ================= Setup / Loop =================
void setup() {
  delay(300);
  Serial.begin(115200);
  uint32_t t0 = millis(); 
  while (!Serial && millis()-t0 < 2000) { delay(10); }

  Serial.println("Integrated Communication Unit starting...");

  // Clean radio start
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(50);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);

#if USE_CUSTOM_MAC
  esp_err_t e = esp_wifi_set_mac(WIFI_IF_STA, CUSTOM_MAC);
  Serial.printf("SET CUSTOM MAC -> %d\n", (int)e);
#endif

  uint8_t mac[6]; 
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  printMac(mac, "COMM UNIT MAC: ");

  esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  uint8_t ch; wifi_second_chan_t sc;
  esp_wifi_get_channel(&ch, &sc);
  Serial.printf("COMM UNIT CHAN: %u\n", ch);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  
  // Register callbacks
  esp_now_register_recv_cb(onSensorDataRecv);
  esp_now_register_send_cb(onHapticDataSent);
  
  // Create queue for sensor data
  g_rxQueue = xQueueCreate(32, sizeof(BraceletMsg));
  if (!g_rxQueue) {
    Serial.println("Failed to create RX queue");
  }
  
  // Add Integrated Node as peer
  addIntegratedNodePeer();
  
  sendOscHello();
  Serial.println("Communication Unit ready!");
  Serial.println("- Receiving sensor data from Integrated Node");
  Serial.println("- Forwarding haptic commands to Integrated Node");
}

void loop() {
  // Handle incoming OSC commands from USB
  pollSerialOSC();

  // Process queued sensor data and send to Python
  BraceletMsg msg; 
  int drained = 0;
  while (g_rxQueue && xQueueReceive(g_rxQueue, &msg, 0) == pdTRUE) {
    sendOscBundleFromMsg(msg);
    if (++drained >= 8) break; // Limit per loop iteration
  }

  // Heartbeat
  static uint32_t tHB = 0; 
  uint32_t now = millis();
  if (now - tHB > 1000) { 
    tHB = now; 
    sendOscHeartbeat(); 
  }

#if DEBUG_ASCII
  // Stats report
  static uint32_t tStat = 0;
  if (now - tStat > 5000) {
    tStat = now;
    UBaseType_t qlen = (g_rxQueue ? uxQueueMessagesWaiting(g_rxQueue) : 0);
    Serial.printf("[stats] sensor_queue=%u drops=%u\n", (unsigned)qlen, (unsigned)g_dropCount);
  }
#endif

  delay(1);
}