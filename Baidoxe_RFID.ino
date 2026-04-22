#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <DS3231.h>
#include <Arduino_JSON.h>
#include <Adafruit_SHT31.h>

#include <WiFi.h>
#include "MQTTClientWrap.h"
#include "Config.h"
#include "Topics.h"


// ================= RFID RC522 =================
#define SS_PIN 5
#define RST_PIN 4
MFRC522 rfid(SS_PIN, RST_PIN);

// ================= SERVO =================
Servo servoIn;
Servo servoOut;

#define SERVO_IN_PIN 14
#define SERVO_OUT_PIN 13

// ================= IR SENSOR =================
#define IR1 34
#define IR2 35
#define IR3 32
#define IR4 33
#define IR_GATE_IN 25
#define IR_GATE_OUT 26

// ================= BUZZER =================
#define BUZZER_PIN 17

// ================= I2C (DS3231 & SHT31) =================
#define I2C_SDA 21
#define I2C_SCL 22

// ================= CẢM BIẾN MQ2 & SHT31 =================
#define MQ2_PIN 36  // Chân analog đọc MQ2 (ADC1_CH0)
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Biến lưu dữ liệu cảm biến
float temperature = 0.0;
float humidity = 0.0;
int smokeLevel = 0;
bool fireAlarm = false;  // Cảnh báo cháy

const int SMOKE_THRESHOLD = 400;  // Ngưỡng cảnh báo khói (0-4095)

// Biến lưu giá trị cũ để so sánh thay đổi (tránh spam MQTT)
float lastTemp = -999.0;
float lastHumi = -999.0;
int lastSmoke = -999;
bool lastFireAlarm = false;
int lastSlot1 = -1;
int lastSlot2 = -1;
int lastSlot3 = -1;
int lastSlot4 = -1;

// ================= BIẾN =================
int totalSlot = 4;
int slotAvailable = 4;
int lastReportedSlot = -1;

const int GATE_OPEN_ANGLE = 90;
const int GATE_CLOSE_ANGLE = 0;
const unsigned long GATE_WAIT_TIMEOUT = 5000;
const unsigned long GATE_CHECK_INTERVAL = 50;
const unsigned long GATE_CLEAR_STABLE_TIME = 500;

const int GATE_SENSOR_ACTIVE_STATE = LOW;
const int SLOT_EMPTY_STATE = HIGH;

// Danh sách UID thẻ master (THAY ĐỔI THEO THẺ THẬT!)
String validUIDs[] = {"a1b2c3d4", "11223344", "55667788", "99aabbcc"};

// ================= THÔNG TIN XE =================
struct VehicleInfo {
  String uid;
  unsigned long timeIn;
  String timeInStr;
};

VehicleInfo parkedVehicles[4];
int checkedInCount = 0;

// WiFi
const char* ssid = "VIETTEL_xsb2Tc";
const char* password = "hzyqgphx";
const unsigned long WIFI_CONNECT_TIMEOUT = 15000;

// MQTT & RTC
MQTTClientWrap mqtt;
DS3231 rtc;
String deviceID = "";
String baseTopic = "";
unsigned long lastTeleSent = 0;

void printESP32MacAddress() {
  WiFi.mode(WIFI_STA);
  String mac = WiFi.macAddress();
  Serial.print("MAC ESP32 (STA): ");
  Serial.println(mac);
}

// Kiểm tra xe tại cổng
bool isVehicleDetected(int sensorPin) {
  return digitalRead(sensorPin) == GATE_SENSOR_ACTIVE_STATE;
}

// Kiểm tra slot trống
bool isSlotEmpty(int sensorPin) {
  return digitalRead(sensorPin) == SLOT_EMPTY_STATE;
}

// Chờ xe đi qua cổng hoàn toàn
void waitForVehicleToPass(int sensorPin) {
  unsigned long startTime = millis();
  unsigned long clearStart = 0;

  while (!isVehicleDetected(sensorPin) && millis() - startTime < GATE_WAIT_TIMEOUT) {
    delay(GATE_CHECK_INTERVAL);
  }

  if (!isVehicleDetected(sensorPin)) return;

  while (true) {
    if (isVehicleDetected(sensorPin)) {
      clearStart = 0;
    } else {
      if (clearStart == 0) clearStart = millis();
      if (millis() - clearStart >= GATE_CLEAR_STABLE_TIME) break;
    }
    delay(GATE_CHECK_INTERVAL);
  }
}

// Kết nối WiFi
bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Dang ket noi WiFi");

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt) < WIFI_CONNECT_TIMEOUT) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nDa ket noi WiFi!");
    Serial.print("Dia chi IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("\nKhong ket noi duoc WiFi!");
  Serial.println("Kiem tra lai ten WiFi (SSID), mat khau va song WiFi.");
  return false;
}

// ================= BUZZER =================
void beepSuccess() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}

void beepError() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

void beepWarning() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(50);
    digitalWrite(BUZZER_PIN, LOW);
    delay(50);
  }
}

// ================= ĐỌC CẢM BIẾN =================

// Đọc cảm biến SHT31 (nhiệt độ & độ ẩm)
void readSHT31() {
  temperature = sht31.readTemperature();
  humidity = sht31.readHumidity();
  
  // Kiểm tra lỗi đọc
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("⚠️ Lỗi đọc SHT31!");
    temperature = 0.0;
    humidity = 0.0;
  }
}

// Đọc cảm biến MQ2 (khói/gas)
void readMQ2() {
  smokeLevel = analogRead(MQ2_PIN);  // Đọc giá trị analog (0-4095)
  
  // Kiểm tra ngưỡng cảnh báo
  if (smokeLevel > SMOKE_THRESHOLD) {
    if (!fireAlarm) {
      fireAlarm = true;
      Serial.println("🔥 CẢNH BÁO: Phát hiện khói/gas cao!");
      beepWarning();  // Kêu còi cảnh báo
    }
  } else {
    fireAlarm = false;
  }
}

// Đọc tất cả cảm biến
void readAllSensors() {
  readSHT31();
  readMQ2();
  
  // In ra Serial để debug
  Serial.print("🌡️ Nhiệt độ: ");
  Serial.print(temperature, 1);
  Serial.print("°C | Độ ẩm: ");
  Serial.print(humidity, 1);
  Serial.print("% | Khói: ");
  Serial.print(smokeLevel);
  if (fireAlarm) Serial.print(" ⚠️ NGUY HIỂM!");
  Serial.println();
}

// ================= RTC & MQTT =================

// Cai dat lai thoi gian cho DS3231
// Vi du: setRTCTime(2026, 4, 23, 14, 30, 0);  // 23/04/2026 luc 14:30:00
void setRTCTime(int year, int month, int date, int hour, int minute, int second) {
  rtc.setYear(year - 2000);     // Thu vien dung 2 chu so cuoi
  rtc.setMonth(month);
  rtc.setDate(date);
  rtc.setHour(hour);
  rtc.setMinute(minute);
  rtc.setSecond(second);
  
  Serial.println("================================");
  Serial.println("✓ DA CAI DAT LAI THOI GIAN DS3231!");
  Serial.print("   Thoi gian moi: ");
  Serial.print(hour);
  Serial.print(":");
  Serial.print(minute);
  Serial.print(":");
  Serial.print(second);
  Serial.print(" ");
  Serial.print(date);
  Serial.print("/");
  Serial.print(month);
  Serial.print("/");
  Serial.println(year);
  Serial.println("================================");
}

String getRTCString() {
  bool century = false;
  bool h12Flag, pmFlag;
  
  int gio = rtc.getHour(h12Flag, pmFlag);
  int phut = rtc.getMinute();
  int giay = rtc.getSecond();
  int ngay = rtc.getDate();
  int thang = rtc.getMonth(century);
  int nam = rtc.getYear() + 2000;
  
  char buffer[32];
  sprintf(buffer, "%02d:%02d:%02d %02d/%02d/%04d", 
          gio, phut, giay, ngay, thang, nam);
  return String(buffer);
}

// Lấy epoch từ DS3231 (tính từ 1/1/2026 00:00:00)
unsigned long getRTCEpoch() {
  bool century = false;
  bool h12Flag, pmFlag;
  
  int gio = rtc.getHour(h12Flag, pmFlag);
  int phut = rtc.getMinute();
  int giay = rtc.getSecond();
  int ngay = rtc.getDate();
  int thang = rtc.getMonth(century);
  int nam = rtc.getYear() + 2000;
  
  unsigned long epoch = 0;
  
  // Tính số giây từ 1/1/2026 đến năm hiện tại
  for (int y = 2026; y < nam; y++) {
    if ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0))
      epoch += 366 * 86400UL;
    else
      epoch += 365 * 86400UL;
  }
  
  // Tính số giây của các tháng trong năm hiện tại
  int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if ((nam % 4 == 0 && nam % 100 != 0) || (nam % 400 == 0))
    daysInMonth[1] = 29;
  
  for (int m = 1; m < thang; m++) {
    epoch += daysInMonth[m - 1] * 86400UL;
  }
  
  // Cộng số giây của ngày, giờ, phút, giây hiện tại
  epoch += (ngay - 1) * 86400UL;
  epoch += gio * 3600UL;
  epoch += phut * 60UL;
  epoch += giay;
  
  return epoch;
}

// Format thời gian đỗ
String formatDuration(unsigned long seconds) {
  unsigned long hours = seconds / 3600;
  unsigned long minutes = (seconds % 3600) / 60;
  
  String result = "";
  if (hours > 0) {
    result += String(hours) + "h ";
  }
  result += String(minutes) + "m";
  
  return result;
}

// Gửi event check-in/check-out qua MQTT
void publishEvent(String eventType, String uid, String timeIn, 
                  String timeOut = "", String duration = "") {
  if (!mqtt.connected()) return;
  
  String payload = "{";
  payload += "\"event\":\"" + eventType + "\",";
  payload += "\"uid\":\"" + uid + "\",";
  payload += "\"timeIn\":\"" + timeIn + "\"";
  
  if (eventType == "check-out") {
    payload += ",\"timeOut\":\"" + timeOut + "\",";
    payload += "\"duration\":\"" + duration + "\"";
  }
  payload += "}";
  String topic = baseTopic + "/event";
  bool ok = mqtt.publish(topic.c_str(), payload.c_str());
  
  Serial.print("[EVENT] ");
  Serial.print(eventType);
  Serial.print(" - ");
  Serial.println(ok ? "OK" : "FAIL");
  Serial.println(payload);
}

/*
 * HÀM: publishParkingStatus
 * MỤC ĐÍCH: Gửi trạng thái 4 vị trí đỗ xe + thời gian qua MQTT
 * CHỈ GỬI KHI CÓ THAY ĐỔI để tránh spam
 * FORMAT JSON:
 * {
 *   "rtc": "14:35:20 15/04/2026",
 *   "slot1": 0,  // 0=trống, 1=có xe
 *   "slot2": 1,
 *   "slot3": 0,
 *   "slot4": 1,
 *   "available": 2,
 *   "total": 4
 * }
 */
void publishParkingStatus() {
  if (!mqtt.connected()) return;
  
  // Đọc trạng thái các slot (0=trống, 1=có xe)
  int slot1 = isSlotEmpty(IR1) ? 0 : 1;
  int slot2 = isSlotEmpty(IR2) ? 0 : 1;
  int slot3 = isSlotEmpty(IR3) ? 0 : 1;
  int slot4 = isSlotEmpty(IR4) ? 0 : 1;
  
  // ═══════════════════════════════════════════════════════════
  // KIỂM TRA THAY ĐỔI - CHỈ GỬI KHI CÓ SỰ KHÁC BIỆT
  // ═══════════════════════════════════════════════════════════
  bool tempChanged = abs(temperature - lastTemp) >= 0.5;      // Thay đổi >= 0.5°C
  bool humiChanged = abs(humidity - lastHumi) >= 2.0;         // Thay đổi >= 2%
  bool smokeChanged = abs(smokeLevel - lastSmoke) >= 50;      // Thay đổi >= 50 điểm
  bool fireChanged = (fireAlarm != lastFireAlarm);            // Trạng thái cháy thay đổi
  bool slot1Changed = (slot1 != lastSlot1);
  bool slot2Changed = (slot2 != lastSlot2);
  bool slot3Changed = (slot3 != lastSlot3);
  bool slot4Changed = (slot4 != lastSlot4);
  
  // Nếu KHÔNG CÓ thay đổi gì → Bỏ qua, không gửi
  if (!tempChanged && !humiChanged && !smokeChanged && !fireChanged &&
      !slot1Changed && !slot2Changed && !slot3Changed && !slot4Changed) {
    return;  // ← THOÁT SỚM, tiết kiệm băng thông MQTT
  }
  
  // ═══════════════════════════════════════════════════════════
  // CÓ THAY ĐỔI → LƯU GIÁ TRỊ MỚI VÀ GỬI LÊN SERVER
  // ═══════════════════════════════════════════════════════════
  lastTemp = temperature;
  lastHumi = humidity;
  lastSmoke = smokeLevel;
  lastFireAlarm = fireAlarm;
  lastSlot1 = slot1;
  lastSlot2 = slot2;
  lastSlot3 = slot3;
  lastSlot4 = slot4;
  
  // Lấy thời gian thực từ DS3231
  String rtcStr = getRTCString();
  
  // Tạo JSON payload (thêm temp, humi, smoke, fireAlarm)
  String payload = "{";
  payload += "\"rtc\":\"" + rtcStr + "\",";
  payload += "\"temp\":" + String(temperature, 1) + ",";
  payload += "\"humi\":" + String(humidity, 1) + ",";
  payload += "\"smoke\":" + String(smokeLevel) + ",";
  payload += "\"fireAlarm\":" + String(fireAlarm ? 1 : 0) + ",";
  payload += "\"slot1\":" + String(slot1) + ",";
  payload += "\"slot2\":" + String(slot2) + ",";
  payload += "\"slot3\":" + String(slot3) + ",";
  payload += "\"slot4\":" + String(slot4) + ",";
  payload += "\"available\":" + String(slotAvailable) + ",";
  payload += "\"total\":" + String(totalSlot);
  payload += "}";
  
  // Gửi qua MQTT
  String topic = T_Tele(baseTopic);
  bool ok = mqtt.publish(topic.c_str(), payload.c_str());
  
  Serial.print("[MQTT] PUB ");
  Serial.print(topic);
  Serial.print(" => ");
  Serial.println(ok ? "OK" : "FAIL");
  Serial.print("Data: ");
  Serial.println(payload);
}

// ================= SETUP =================
void setup() {
  Serial.begin(9600);
  printESP32MacAddress();

  // Khởi tạo I2C cho DS3231 & SHT31
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("✓ I2C OK (SDA=21, SCL=22)");
  
  // Kiểm tra DS3231 RTC
  Wire.beginTransmission(0x68);  // Địa chỉ I2C của DS3231
  if (Wire.endTransmission() == 0) {
    Serial.println("✓ DS3231 RTC được phát hiện!");
    
    // Set thoi gian RTC (chi chay 1 lan, sau do comment lai)
    setRTCTime(2026, 4, 23, 8, 0, 0);
    
    // In thoi gian hien tai de kiem tra
    Serial.print("   Thoi gian hien tai: ");
    Serial.println(getRTCString());
    
  } else {
    Serial.println("⚠️ Không tìm thấy DS3231! Kiểm tra kết nối I2C.");
  }

  if (!connectWiFi()) {
    Serial.println("He thong van chay offline.");
  }

  SPI.begin();
  rfid.PCD_Init();

  servoIn.attach(SERVO_IN_PIN);
  servoOut.attach(SERVO_OUT_PIN);

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR_GATE_IN, INPUT);
  pinMode(IR_GATE_OUT, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(MQ2_PIN, INPUT);
  
  if (!sht31.begin(0x44)) {
    Serial.println("⚠️ Không tìm thấy SHT31!");
  } else {
    Serial.println("✓ SHT31 OK!");
  }

  servoIn.write(GATE_CLOSE_ANGLE);
  servoOut.write(GATE_CLOSE_ANGLE);
  
  for (int i = 0; i < 4; i++) {
    parkedVehicles[i].uid = "";
    parkedVehicles[i].timeIn = 0;
    parkedVehicles[i].timeInStr = "";
  }

  WiFi.mode(WIFI_STA);
  deviceID = WiFi.macAddress();
  deviceID.replace(":", "");
  baseTopic = TopicBase(deviceID);
  
  Serial.print("[MQTT] ID: ");
  Serial.println(deviceID);
  
  mqtt.begin(MQTT_HOST, MQTT_PORT, deviceID.c_str(), MQTT_USER, MQTT_PASS);
  
  Serial.println("=== HE THONG BAO DO XE ===");
  delay(500);
  beepSuccess();
  Serial.println("✓ San sang!");
}

// ================= LOOP =================
void loop() {
  mqtt.loop();
  
  // Đọc cảm biến liên tục (phát hiện cháy nhanh)
  static unsigned long lastSensorRead = 0;
  if (millis() - lastSensorRead >= 500) {
    lastSensorRead = millis();
    readAllSensors();
  }
  
  // Gửi MQTT (chỉ khi có thay đổi)
  if (millis() - lastTeleSent >= TELE_PERIOD_MS) {
    lastTeleSent = millis();
    publishParkingStatus();
  }
  
  updateSlot();

  // CHẾ ĐỘ KHẨN CẤP: Cháy → Mở tất cả barie
  if (fireAlarm) {
    servoIn.write(GATE_OPEN_ANGLE);
    servoOut.write(GATE_OPEN_ANGLE);
    
    static unsigned long lastBeep = 0;
    if (millis() - lastBeep >= 2000) {
      lastBeep = millis();
      beepWarning();
      Serial.println("🚨 KHẨN CẤP! Khói: " + String(smokeLevel));
    }
    return;
  }

  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial())
    return;

  String uid = readUID();
  Serial.print("The quet: ");
  Serial.println(uid);

  if (isVehicleDetected(IR_GATE_OUT)) {
    // CỔNG RA
    if (isCheckedIn(uid)) {
      beepSuccess();  // Báo hiệu thành công
      openGateOut();
      checkOutUID(uid);  // Xóa UID khỏi danh sách sau khi xe ra
    } else {
      beepError();  // Báo hiệu lỗi
      Serial.println("❌ The khong hop le hoac chua check-in!");
    }
    
  } else if (isVehicleDetected(IR_GATE_IN)) {
    // === CỔNG VÀO: Kiểm tra thẻ hợp lệ (master) và có chỗ trống ===
    if (slotAvailable > 0) {
      if (isValidUID(uid)) {
        if (checkInUID(uid)) {  // Lưu UID vào danh sách check-in
          beepSuccess();  // Báo hiệu thành công
          openGateIn();
        } else {
          beepWarning();  // Thẻ đã vào rồi
        }
      } else {
        beepError();  // Báo hiệu lỗi
        Serial.println("❌ The khong hop le!");
      }
    } else {
      beepWarning();  // Cảnh báo hết chỗ
      Serial.println("❌ Het cho!");
    }
  }
  // Lưu ý: Không cần else để báo lỗi khi không có xe ở cổng
  // → Cho phép người dùng quét thẻ trước rồi mới lái xe vào

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  delay(1000);
}

// Đọc UID từ thẻ RFID
String readUID() {
  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toLowerCase();
  return uid;
}

// Kiểm tra thẻ hợp lệ
bool isValidUID(String uid) {
  int totalValidCards = sizeof(validUIDs) / sizeof(validUIDs[0]);
  for (int i = 0; i < totalValidCards; i++) {
    String validUID = validUIDs[i];
    validUID.toLowerCase();
    if (uid == validUID) return true;
  }
  return false;
}

// Kiểm tra xe đã check-in
bool isCheckedIn(String uid) {
  for (int i = 0; i < 4; i++) {
    if (parkedVehicles[i].uid == uid && uid != "") {
      return true;
    }
  }
  return false;
}

// ================= THÊM UID VÀO DANH SÁCH CHECK-IN =================

/*
 * HÀM: checkInUID
 * MỤC ĐÍCH: Lưu thẻ vào danh sách check-in khi xe VÀO bãi
 * THAM SỐ: uid - mã UID của thẻ cần lưu
 * TRẢ VỀ: true = lưu thành công
 *         false = lưu thất bại (đã vào rồi hoặc bãi đầy)
 * FLOW:
 *   Bước 1: Kiểm tra thẻ đã vào chưa → Nếu rồi → Từ chối
 *   Bước 2: Tìm slot trống trong checkedInUIDs[]
 *   Bước 3: Lưu UID vào slot → Tăng biến đếm → Thành công
 * VÍ DỤ:
 *   TRƯỚC: checkedInUIDs[] = {"a1b2c3d4", "", "", ""}
 *   checkInUID("1122+ thời gian vào khi xe VÀO bãi
 * FLOW:
 *   Bước 1: Kiểm tra thẻ đã vào chưa
 *   Bước 2: Tìm slot trống
 *   Bước 3: Lưu UID + thời gian vào
 *   Bước 4: Gửi event check-in qua MQTT
 */
bool checkInUID(String uid) {
  // BƯỚC 1: Kiểm tra trùng lặp
  if (isCheckedIn(uid)) {
    Serial.println("⚠️ The nay da vao bai roi!");
    return false;
  }
  
  // BƯỚC 2 & 3: Tìm slot trống và lưu thông tin
  for (int i = 0; i < 4; i++) {
    if (parkedVehicles[i].uid == "") {  // Slot trống
      // Lưu thông tin xe
      parkedVehicles[i].uid = uid;
      parkedVehicles[i].timeIn = getRTCEpoch();     // Epoch seconds
      parkedVehicles[i].timeInStr = getRTCString(); // String để hiển thị
      
      checkedInCount++;
      
      Serial.print("✅ CHECK-IN: ");
      Serial.println(uid);
      Serial.print("   Thoi gian: ");
      Serial.println(parkedVehicles[i].timeInStr);
      Serial.print("   So xe trong bai: ");
      Serial.println(checkedInCount);
      
      // BUOC 4: Gui event check-in len web
      publishEvent("check-in", uid, parkedVehicles[i].timeInStr, "", "");
      
      return true;
    }
  }
  
  // Khong tim thay slot trong
  Serial.println("❌ Bai day!");
  return false;
}

/*
 * HAM: checkOutUID
 * MUC DICH: Tinh thoi gian do va xoa xe khoi bai khi RA
 * FLOW:
 *   Buoc 1: Tim xe trong danh sach
 *   Buoc 2: Lay thoi gian ra
 *   Buoc 3: Tinh thoi gian do (duration)
 *   Buoc 4: Gui event check-out qua MQTT
 *   Buoc 5: Xoa thong tin xe
 */
bool checkOutUID(String uid) {
  for (int i = 0; i < 4; i++) {
    if (parkedVehicles[i].uid == uid && uid != "") {
      unsigned long timeOut = getRTCEpoch();
      String timeOutStr = getRTCString();
      unsigned long duration = timeOut - parkedVehicles[i].timeIn;
      String durationStr = formatDuration(duration);
      
      Serial.println("================================");
      Serial.print("✅ CHECK-OUT: ");
      Serial.println(uid);
      Serial.print("   Thoi gian vao: ");
      Serial.println(parkedVehicles[i].timeInStr);
      Serial.print("   Thoi gian ra:  ");
      Serial.println(timeOutStr);
      Serial.print("   Thoi gian do:  ");
      Serial.println(durationStr);
      Serial.println("================================");
      
      publishEvent("check-out", uid, parkedVehicles[i].timeInStr, timeOutStr, durationStr);
      
      parkedVehicles[i].uid = "";
      parkedVehicles[i].timeIn = 0;
      parkedVehicles[i].timeInStr = "";
      checkedInCount--;
      
      Serial.print("   So xe con lai: ");
      Serial.println(checkedInCount);
      return true;
    }
  }
  
  Serial.println("❌ The nay khong co trong bai!");
  return false;
}

// Cập nhật số chỗ trống
void updateSlot() {
  int count = 0;
  if (isSlotEmpty(IR1)) count++;
  if (isSlotEmpty(IR2)) count++;
  if (isSlotEmpty(IR3)) count++;
  if (isSlotEmpty(IR4)) count++;
  if (count > totalSlot) count = totalSlot;
  
  slotAvailable = count;
  
  if (slotAvailable != lastReportedSlot) {
    Serial.print("Cho trong: ");
    Serial.println(slotAvailable);
    lastReportedSlot = slotAvailable;
  }
}

// Mở cổng vào
void openGateIn() {
  Serial.println("✅ Mo cong vao");
  servoIn.write(GATE_OPEN_ANGLE);
  waitForVehicleToPass(IR_GATE_IN);
  servoIn.write(GATE_CLOSE_ANGLE);
  Serial.println("Dong cong vao");
}

// Mở cổng ra
void openGateOut() {
  Serial.println("🚗 Mo cong ra");
  servoOut.write(GATE_OPEN_ANGLE);
  waitForVehicleToPass(IR_GATE_OUT);
  servoOut.write(GATE_CLOSE_ANGLE);
  Serial.println("Dong cong ra");
}
