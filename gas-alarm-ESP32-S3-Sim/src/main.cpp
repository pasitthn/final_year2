#include <Arduino.h>
/*
  Gas / Smoke Alarm - ESP32-S3-WROOM-1 + MQ-2 + RGB LED + Active Buzzer
  ส่งข้อมูลเข้า ThingsBoard Cloud ผ่าน MQTT

  ไลบรารีที่ต้องติดตั้ง: PubSubClient (by Nick O'Leary)

  ระดับแก๊ส (mV ที่ขา ADC) และสถานะ / สีไฟ:
    0    - 1200 mV  NORMAL -> เขียวติดค้าง
    1200 - 2000 mV  WARN   -> เหลืองติดค้าง (เริ่มผิดปกติ)
    > 2000 mV       ALARM  -> แดงกระพริบ + บัซเซอร์ดังสลับ
    WARMUP                 -> น้ำเงินติดค้าง (อุ่นเซนเซอร์)
  ปุ่ม BOOT (IO0): ช่วง ALARM กดเพื่อปิดเสียงชั่วคราว (ไฟแดงยังกระพริบ)
*/

#include <WiFi.h>
#include <PubSubClient.h>

// ================= ตั้งค่าที่ต้องแก้ =================
#ifdef WOKWI_SIM
const char* WIFI_SSID = "Wokwi-GUEST";   // WiFi จำลองของ Wokwi (ไม่มีรหัส)
const char* WIFI_PASS = "";
#else
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
#endif

const char*    TB_HOST  = "thingsboard.cloud";
const uint16_t TB_PORT  = 1883;
const char*    TB_TOKEN = "HKcj2oywcgbbcv50PRWu";

const bool BUZZER_ENABLED = true;

// ================= ขา GPIO (ตาม schematic) =================
const int PIN_MQ2    = 1;   // MQ2_AO ผ่านตัวแบ่ง R7/R8
const int PIN_BUZZER = 4;   // ขับ 2N3904 (HIGH = ดัง)
const int PIN_R      = 5;   // RGB common cathode (HIGH = ติด)
const int PIN_G      = 6;
const int PIN_B      = 7;
const int PIN_BTN    = 0;   // BOOT (กด = LOW, มี pull-up ภายนอก)

// ================= เกณฑ์ระดับแก๊ส (mV) =================
const int TH_WARN_ON   = 1200;  // >= นี้ -> WARN
const int TH_WARN_OFF  = 1100;  // < นี้ -> กลับ NORMAL (hysteresis กันกระพริบ)
const int TH_ALARM_ON  = 2000;  // >= นี้ -> ALARM
const int TH_ALARM_OFF = 1900;  // < นี้ -> ลดลงจาก ALARM

// ================= พารามิเตอร์ =================
#ifdef WOKWI_SIM
const uint32_t WARMUP_MS  = 5000;    // sim: อุ่น 5 วิ
const uint32_t PUBLISH_MS = 2000;    // sim: ส่งทุก 2 วิ ให้กราฟลื่น
#else
const uint32_t WARMUP_MS  = 120000;  // อุ่นเซนเซอร์ 2 นาที
const uint32_t PUBLISH_MS = 5000;    // ส่ง ThingsBoard ทุก 5 วิ
#endif
const uint32_t SAMPLE_MS  = 500;     // อ่านเซนเซอร์ทุก 0.5 วิ
const uint32_t MUTE_MS    = 300000;  // ปิดเสียงชั่วคราว 5 นาที
const int      UP_CONFIRM   = 3;     // ระดับแย่ลง ติดกัน 3 ครั้ง (1.5 วิ) ถึงเปลี่ยน
const int      DOWN_CONFIRM = 6;     // ระดับดีขึ้น ติดกัน 6 ครั้ง (3 วิ) ถึงเปลี่ยน

// ================= ตัวแปร =================
enum State { WARMUP, NORMAL, WARN, ALARM };
State state = WARMUP;
State pendingState = NORMAL;
int pendingCount = 0;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

int gasMv = 0;

uint32_t lastSample = 0, lastPublish = 0;
uint32_t lastWifiTry = 0, lastMqttTry = 0;
bool publishNow = false;

uint32_t mutedUntil = 0;
bool isMuted = false;

bool lastBtn = HIGH;
uint32_t lastBtnChange = 0;

// ================= ฟังก์ชันช่วย =================
void setLed(bool r, bool g, bool b) {
  digitalWrite(PIN_R, r);
  digitalWrite(PIN_G, g);
  digitalWrite(PIN_B, b);
}

void setBuzzer(bool on) {
  static int8_t last = -1;
  bool v = BUZZER_ENABLED && on;
  if (last == (int8_t)v) return;
  last = (int8_t)v;
#ifdef WOKWI_SIM
  if (v) tone(PIN_BUZZER, 2500); else noTone(PIN_BUZZER);
#else
  digitalWrite(PIN_BUZZER, v ? HIGH : LOW);
#endif
}

#ifdef SIM_SCENARIO
// สถานการณ์จำลอง: ค่าแก๊สจากต่ำไปสูง (วนซ้ำทุก 180 วินาที)
//   0-120 วิ   : เริ่มที่ ~800 mV (ต่ำ) แล้วค่อยๆ ไต่ขึ้นไป 3100 mV (สูง)
//                 (ผ่าน 1200 ที่ ~วิที่ 21 -> เขียวเป็นเหลือง, ผ่าน 2000 ที่ ~วิที่ 63 -> เหลืองเป็นแดง)
//   120-150 วิ : แก๊สคงที่สูง ~3100 mV
//   150-170 วิ : แก๊สค่อยๆ ลดลงกลับไปที่ ~800 mV
//   170-180 วิ : อากาศปกติระดับต่ำ ~800 mV
int scenarioMv(uint32_t nowMs) {
  const int HIGH_MV = 3100, LOW_MV = 800;
  uint32_t t = (nowMs / 1000) % 180;
  int v;
  if (t < 120)      v = LOW_MV + (HIGH_MV - LOW_MV) * (int)t / 120;
  else if (t < 150) v = HIGH_MV;
  else if (t < 170) v = HIGH_MV - (HIGH_MV - LOW_MV) * (int)(t - 150) / 20;
  else              v = LOW_MV;
  return v + (int)random(-15, 16);   // noise เล็กน้อย
}
#endif

int readMv() {
#ifdef SIM_SCENARIO
  return scenarioMv(millis());
#endif
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) {
    sum += analogReadMilliVolts(PIN_MQ2);
    delay(2);
  }
  return sum / 16;
}

const char* stateName() {
  switch (state) {
    case WARMUP: return "warmup";
    case NORMAL: return "normal";
    case WARN:   return "warn";
    case ALARM:  return "alarm";
  }
  return "unknown";
}

// ================= เครือข่าย (ไม่บล็อกการทำงานของเซนเซอร์) =================
void handleNetwork(uint32_t now) {
  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastWifiTry > 15000) {
      lastWifiTry = now;
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    return;
  }
  if (!mqtt.connected()) {
    if (now - lastMqttTry > 5000) {
      lastMqttTry = now;
      // ThingsBoard: username = access token, password ว่าง
      if (mqtt.connect("esp32s3-gas-alarm", TB_TOKEN, NULL)) {
        Serial.println("MQTT connected");
      } else {
        Serial.printf("MQTT failed, rc=%d\n", mqtt.state());
      }
    }
  } else {
    mqtt.loop();
  }
}

void publishTelemetry() {
  if (!mqtt.connected()) return;
  char buf[200];
  snprintf(buf, sizeof(buf),
           "{\"gas_mv\":%d,\"alarm\":%s,\"muted\":%s,\"state\":\"%s\",\"rssi\":%d}",
           gasMv,
           state == ALARM ? "true" : "false",
           isMuted ? "true" : "false",
           stateName(), WiFi.RSSI());
  mqtt.publish("v1/devices/me/telemetry", buf);
  Serial.println(buf);
}

// ================= ปุ่ม BOOT =================
void handleButton(uint32_t now) {
  bool b = digitalRead(PIN_BTN);
  if (b != lastBtn && now - lastBtnChange > 50) {
    lastBtnChange = now;
    lastBtn = b;
    if (b == LOW && state == ALARM) {
      mutedUntil = now + MUTE_MS;
      isMuted = true;
      publishNow = true;
      Serial.println("Buzzer muted");
    }
  }
  if (isMuted && (int32_t)(now - mutedUntil) >= 0) {
    isMuted = false;
  }
}

// ================= ประมวลผลค่าเซนเซอร์ =================
// ระดับที่ค่าแก๊สบอก โดยมี hysteresis จากสถานะปัจจุบัน
State levelFor(int v, State cur) {
  if (v >= TH_ALARM_ON || (cur == ALARM && v >= TH_ALARM_OFF)) return ALARM;
  if (v >= TH_WARN_ON  || ((cur == WARN || cur == ALARM) && v >= TH_WARN_OFF)) return WARN;
  return NORMAL;
}

void processSample(uint32_t now) {
  gasMv = readMv();

  if (state == WARMUP) {
    if (now >= WARMUP_MS) {
      state = NORMAL;
      pendingCount = 0;
      publishNow = true;
    }
  } else {
    State cand = levelFor(gasMv, state);
    if (cand == state) {
      pendingCount = 0;
    } else {
      if (cand == pendingState) pendingCount++;
      else { pendingState = cand; pendingCount = 1; }
      int need = (cand > state) ? UP_CONFIRM : DOWN_CONFIRM;
      if (pendingCount >= need) {
        state = cand;
        pendingCount = 0;
        isMuted = false;
        publishNow = true;
        Serial.printf(">>> state -> %s\n", stateName());
      }
    }
  }

  Serial.printf("[%s] gas=%d mV\n", stateName(), gasMv);
}

// ================= ไฟ + บัซเซอร์ =================
void updateOutputs(uint32_t now) {
  switch (state) {
    case WARMUP:
      setLed(0, 0, 1);   // น้ำเงิน ช่วงอุ่นเครื่อง
      setBuzzer(false);
      break;
    case NORMAL:
      setLed(0, 1, 0);   // เขียว
      setBuzzer(false);
      break;
    case WARN:
      setLed(1, 1, 0);   // เหลือง (เริ่มผิดปกติ) ไม่มีเสียง
      setBuzzer(false);
      break;
    case ALARM: {
      bool phase = ((now / 250) % 2) == 0;
      setLed(phase, 0, 0);
      setBuzzer(phase && !isMuted);
      break;
    }
  }
}

// ================= setup / loop =================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_G, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  setLed(0, 0, 1);
#ifdef WOKWI_SIM
  pinMode(PIN_BTN, INPUT_PULLUP);   // sim ไม่มี R10 ภายนอก
#else
  pinMode(PIN_BTN, INPUT);
#endif

  analogSetPinAttenuation(PIN_MQ2, ADC_11db);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  mqtt.setServer(TB_HOST, TB_PORT);
  mqtt.setSocketTimeout(3);
  mqtt.setBufferSize(256);

  Serial.println("Warming up MQ-2...");
}

void loop() {
  uint32_t now = millis();

  handleNetwork(now);
  handleButton(now);

  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;
    processSample(now);
  }

  updateOutputs(now);

  if (publishNow || now - lastPublish >= PUBLISH_MS) {
    lastPublish = now;
    publishNow = false;
    publishTelemetry();
  }
}
