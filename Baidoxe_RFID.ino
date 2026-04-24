#include <SPI.h>              // Giao tiếp SPI cho RFID RC522
#include <MFRC522.h>          // Thư viện đọc thẻ RFID RC522
#include <ESP32Servo.h>       // Điều khiển servo motor trên ESP32
#include <Wire.h>             // Giao tiếp I2C cho RTC & cảm biến
#include <DS3231.h>           // Đồng hồ thời gian thực RTC DS3231
#include <Arduino_JSON.h>     // Xử lý dữ liệu JSON
#include <Adafruit_SHT31.h>   // Cảm biến nhiệt độ/độ ẩm SHT31

#include <WiFi.h>             // Kết nối WiFi
#include "MQTTClientWrap.h"   // Custom MQTT client (file tự tạo)
#include "Config.h"           // Cấu hình MQTT broker (file tự tạo)
#include "Topics.h"           // Định nghĩa MQTT topics (file tự tạo)


// ─────────────── RFID RC522 ───────────────
#define SS_PIN 5              // Chân SDA/SS của RFID kết nối GPIO 5
#define RST_PIN 16            // Chân RST của RFID kết nối GPIO 16
MFRC522 rfid(SS_PIN, RST_PIN); // Khởi tạo đối tượng RFID

// ─────────────── SERVO MOTOR ───────────────
Servo servoIn;                // Đối tượng servo cổng VÀO
Servo servoOut;               // Đối tượng servo cổng RA

#define SERVO_IN_PIN 14       // Servo cổng VÀO → GPIO 14
#define SERVO_OUT_PIN 13      // Servo cổng RA → GPIO 13

// ─────────────── CẢM BIẾN HỒNG NGOẠI (IR) ───────────────
#define IR1 34                // Cảm biến slot 1 → GPIO 34 (đọc chỗ đỗ xe)
#define IR2 35                // Cảm biến slot 2 → GPIO 35
#define IR3 32                // Cảm biến slot 3 → GPIO 32
#define IR4 33                // Cảm biến slot 4 → GPIO 33
#define IR_GATE_IN 25         // Cảm biến cổng VÀO → GPIO 25 (phát hiện xe)
#define IR_GATE_OUT 26        // Cảm biến cổng RA → GPIO 26 (phát hiện xe)

// ─────────────── BUZZER (CÒI) ───────────────
#define BUZZER_PIN 17         // Còi beep → GPIO 17

// ─────────────── I2C (RTC & CẢM BIẾN NHIỆT ĐỘ) ───────────────
#define I2C_SDA 21            // Chân SDA của I2C → GPIO 21
#define I2C_SCL 22            // Chân SCL của I2C → GPIO 22

// ─────────────── CẢM BIẾN KHÓI MQ2 & NHIỆT ĐỘ SHT31 ───────────────
#define MQ2_PIN 36            // Cảm biến khói/gas MQ2 → GPIO 36 (ADC1_CH0)
Adafruit_SHT31 sht31 = Adafruit_SHT31(); // Đối tượng cảm biến SHT31

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                       BIẾN LƯU DỮ LIỆU CẢM BIẾN                          ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
float temperature = 0.0;      // Biến lưu nhiệt độ (°C)
float humidity = 0.0;         // Biến lưu độ ẩm (%)
int smokeLevel = 0;           // Biến lưu mức khói/gas (0-4095)
bool fireAlarm = false;       // Cờ cảnh báo cháy: TRUE = nguy hiểm, FALSE = an toàn

const int SMOKE_THRESHOLD = 400;  // Ngưỡng cảnh báo khói: >400 = nguy hiểm

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                   CẤU HÌNH STATE MACHINE - ĐIỀU KHIỂN CỔNG               ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

// ─────────────── GÓC SERVO ───────────────
int closeIn = 140;            // Góc đóng cổng VÀO: 140 độ
int openIn = 45;              // Góc mở cổng VÀO: 45 độ
int closeOut = 90;            // Góc đóng cổng RA: 90 độ  
int openOut = 0;              // Góc mở cổng RA: 0 độ

// ─────────────── 5 TRẠNG THÁI CỦA CỔNG ───────────────
enum GateState {
  GATE_CLOSED,                // Trạng thái 1: Cổng đóng, chờ xe đến
  GATE_WAIT_CARD,             // Trạng thái 2: Phát hiện xe, đợi quét thẻ RFID
  GATE_OPENING,               // Trạng thái 3: Đang mở cổng (servo đang xoay)
  GATE_OPEN,                  // Trạng thái 4: Cổng đã mở, chờ xe qua
  GATE_CLOSING                // Trạng thái 5: Đang đóng cổng
};

// ─────────────── BIẾN TRẠNG THÁI HIỆN TẠI ───────────────
GateState gateInState = GATE_CLOSED;   // Trạng thái cổng VÀO (bắt đầu = đóng)
GateState gateOutState = GATE_CLOSED;  // Trạng thái cổng RA (bắt đầu = đóng)

// ─────────────── TIMER CHO STATE MACHINE ───────────────
unsigned long gateInTimer = 0;         // Lưu thời điểm (millis) cho cổng VÀO
unsigned long gateOutTimer = 0;        // Lưu thời điểm (millis) cho cổng RA

// ─────────────── THỜI GIAN TIMEOUT ───────────────
const unsigned long CARD_WAIT_TIMEOUT = 10000;  // Chờ quét thẻ tối đa 10 giây
const unsigned long GATE_HOLD_TIME = 3000;      // Giữ cổng mở 3 giây (chờ xe qua)
const unsigned long GATE_CLOSE_DELAY = 2000;    // Delay 2 giây trước khi đóng

// ─────────────── BIẾN LƯU THẺ RFID ───────────────
String pendingCardUID = "";            // Lưu UID thẻ vừa quét (chờ xử lý)

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║            BIẾN LƯU GIÁ TRỊ CŨ (TRÁNH GỬI MQTT TRÙNG LẶP)               ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
// Chỉ gửi MQTT khi giá trị THAY ĐỔI so với lần trước
float lastTemp = -999.0;      // Nhiệt độ lần trước (-999 = chưa có giá trị)
float lastHumi = -999.0;      // Độ ẩm lần trước
int lastSmoke = -999;         // Mức khói lần trước
bool lastFireAlarm = false;   // Trạng thái cháy lần trước
int lastSlot1 = -1;           // Trạng thái slot 1 lần trước (-1 = chưa đọc)
int lastSlot2 = -1;           // Trạng thái slot 2 lần trước
int lastSlot3 = -1;           // Trạng thái slot 3 lần trước
int lastSlot4 = -1;           // Trạng thái slot 4 lần trước

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                        QUẢN LÝ CHỖ ĐỖ XE                                 ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
int totalSlot = 4;            // Tổng số chỗ đỗ: 4 chỗ
int slotAvailable = 4;        // Số chỗ trống hiện tại (ban đầu = 4)
int lastReportedSlot = -1;    // Số chỗ trống đã báo lần trước

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                    HẰNG SỐ CẤU HÌNH CẢM BIẾN                             ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
const int GATE_SENSOR_ACTIVE_STATE = LOW;  // IR phát hiện xe = LOW (0)
const int SLOT_EMPTY_STATE = HIGH;         // Slot trống = HIGH (1)

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                  DANH SÁCH THẺ RFID HỢP LỆ                               ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
// ✅ UID THẺ THẬT ĐÃ CẬP NHẬT! (Tổng 6 thẻ)
String validUIDs[] = {
  "07bb9495",  
  "47c4a595",  // Thẻ 2 (đã test)
  "a6c7fd06",  // Thẻ 3 (đã test)
  "97a8d895",  // Thẻ 4 (mới thêm)
  "3732d795",  // Thẻ 5 (mới thêm)
  "07831696"   // Thẻ 6 (mới thêm)
};

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                    CẤU TRÚC LƯU THÔNG TIN XE ĐỖ                          ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
struct VehicleInfo {
  String uid;                 // UID của thẻ RFID
  unsigned long timeIn;       // Thời gian vào (millis)
  String timeInStr;           // Thời gian vào (dạng chuỗi "HH:MM:SS DD/MM/YYYY")
};

VehicleInfo parkedVehicles[4]; // Mảng lưu 4 xe đang đỗ
int checkedInCount = 0;        // Số xe hiện đang trong bãi

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                       CẤU HÌNH WIFI                                       ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
const char* ssid = "OPPO VN";              // Tên WiFi (SSID)
const char* password = "244466666";        // Mật khẩu WiFi
const unsigned long WIFI_CONNECT_TIMEOUT = 10000; // Timeout kết nối WiFi: 10 giây

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                    MQTT & ĐỒNG HỒ RTC                                     ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
MQTTClientWrap mqtt;          // Đối tượng MQTT client (custom)
DS3231 rtc;                   // Đối tượng đồng hồ thời gian thực
String deviceID = "";         // ID thiết bị (MAC address không dấu :)
String baseTopic = "";        // Topic gốc MQTT (vd: "parking/A1B2C3D4E5F6")
unsigned long lastTeleSent = 0; // Thời điểm gửi telemetry lần cuối

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                    BUZZER (CÒI BEEP)                                      ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
// Hệ thống buzzer đơn giản (sử dụng delay)

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                  KHAI BÁO HÀM (FORWARD DECLARATIONS)                      ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
// Khai báo các hàm sẽ định nghĩa ở dưới (để compiler biết trước)

// ─── Hàm tiện ích ───
void printESP32MacAddress();
bool connectWiFi();

// ─── Hàm cảm biến ───
bool isVehicleDetected(int sensorPin);
bool isSlotEmpty(int sensorPin);
void readSHT31();
void readMQ2();
void readAllSensors();

// ─── Hàm buzzer ───
void beepSuccess();
void beepError();
void beepNormal();
void beepWarning();

// ─── Hàm RTC ───
void setRTCTime(int year, int month, int date, int hour, int minute, int second);
byte rtcReadRegister(byte reg);
void rtcWriteRegister(byte reg, byte value);
void setupRTCBackupMode();
String getRTCString();
unsigned long getRTCEpoch();
String formatDuration(unsigned long seconds);

// ─── Hàm MQTT ───
void publishEvent(String eventType, String uid, String timeIn, String timeOut = "", String duration = "");
void publishParkingStatus();

// ─── Hàm xử lý cổng ───
void handleGateIn();
void handleGateOut();
void handleRFIDScan();

// ─── Hàm RFID & quản lý xe ───
String readUID();
bool isValidUID(String uid);
bool isCheckedIn(String uid);
bool checkInUID(String uid);
bool checkOutUID(String uid);
void updateSlot();

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                         HÀM IN MAC ADDRESS                                ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
void printESP32MacAddress() {
  WiFi.mode(WIFI_STA);          // Bật chế độ Station (kết nối WiFi)
  String mac = WiFi.macAddress(); // Đọc MAC address của ESP32
  Serial.print("MAC ESP32 (STA): "); // In ra Serial Monitor
  Serial.println(mac);          // VD: "A4:CF:12:34:56:78"
}

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║              HÀM KIỂM TRA XE TẠI CỔNG (IR SENSOR)                         ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
// INPUT:  sensorPin - Chân GPIO của IR sensor (IR_GATE_IN hoặc IR_GATE_OUT)
// OUTPUT: true = có xe, false = không có xe
bool isVehicleDetected(int sensorPin) {
  return digitalRead(sensorPin) == GATE_SENSOR_ACTIVE_STATE;  // LOW = có xe
}

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║              HÀM KIỂM TRA CHỖ ĐỖ TRỐNG (IR SENSOR)                       ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
// INPUT:  sensorPin - Chân GPIO của IR sensor slot (IR1, IR2, IR3, IR4)
// OUTPUT: true = chỗ trống, false = có xe đỗ
bool isSlotEmpty(int sensorPin) {
  return digitalRead(sensorPin) == SLOT_EMPTY_STATE;  // HIGH = trống
}

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                      HÀM KẾT NỐI WIFI                                     ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
// RETURN: true = kết nối thành công, false = kết nối thất bại
bool connectWiFi() {
  WiFi.mode(WIFI_STA);              // Chế độ Station (client)
  WiFi.begin(ssid, password);       // Bắt đầu kết nối đến WiFi
  Serial.print("Dang ket noi WiFi");

  unsigned long startAttempt = millis();  // Lưu thời điểm bắt đầu
  // Vòng lặp chờ kết nối (timeout 10 giây)
  while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt) < WIFI_CONNECT_TIMEOUT) {
    delay(500);                     // Chờ 0.5 giây
    Serial.print(".");              // In dấu . để báo hiệu đang chờ
  }

  // Kiểm tra đã kết nối thành công chưa
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nDa ket noi WiFi!");
    Serial.print("Dia chi IP: ");
    Serial.println(WiFi.localIP()); // In địa chỉ IP nhận được
    return true;                    // Thành công
  }

  // Kết nối thất bại
  Serial.println("\nKhong ket noi duoc WiFi!");
  Serial.println("Kiem tra lai ten WiFi (SSID), mat khau va song WiFi.");
  return false;                     // Thất bại
}

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                    CÁC HÀM BEEP ÂM THANH                                  ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

// ─────────────── BEEP THÀNH CÔNG ───────────────
// 1 tiếng beep ngắn 100ms
// Dùng khi: Khởi động hệ thống, quét thẻ hợp lệ, mở cổng thành công
void beepSuccess() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}

// ─────────────── BEEP LỖI ───────────────
// 2 tiếng beep ngắn (100ms - pause - 100ms)
// Dùng khi: Thẻ không hợp lệ, bãi đầy, lỗi
void beepError() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
}

// ─────────────── BEEP BÌNH THƯỜNG ───────────────
// 1 tiếng beep ngắn 150ms
// Dùng khi: Thông báo bình thường
void beepNormal() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);
}

// ─────────────── BEEP CẢNH BÁO ───────────────
// 3 tiếng beep nhanh (80ms mỗi tiếng)
// Dùng khi: Phát hiện khói/cháy, nguy hiểm
void beepWarning() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(80);
    digitalWrite(BUZZER_PIN, LOW);
    delay(80);
  }
}

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                   ĐỌC CÁC CẢM BIẾN MÔI TRƯỜNG                             ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

// ─────────────── ĐỌC CẢM BIẾN SHT31 (NHIỆT ĐỘ & ĐỘ ẨM) ───────────────
void readSHT31() {
  temperature = sht31.readTemperature();  // Đọc nhiệt độ (°C)
  humidity = sht31.readHumidity();        // Đọc độ ẩm (%)
  
  // Kiểm tra lỗi đọc (NaN = Not a Number = lỗi)
  static bool errorPrinted = false;  // Biến static: giữ giá trị giữa các lần gọi hàm
  
  if (isnan(temperature) || isnan(humidity)) {  // Nếu có lỗi
    if (!errorPrinted) {  // Chỉ in lỗi 1 lần (tránh spam Serial)
      Serial.println("⚠️ Lỗi đọc SHT31! Kiểm tra kết nối I2C.");
      errorPrinted = true;  // Đánh dấu đã in lỗi
    }
    temperature = 0.0;  // Đặt giá trị mặc định khi lỗi
    humidity = 0.0;
  } else {
    errorPrinted = false;  // Reset cờ khi đọc thành công
  }
}

// ─────────────── ĐỌC CẢM BIẾN MQ2 (KHÓI/GAS) ───────────────
void readMQ2() {
  smokeLevel = analogRead(MQ2_PIN);  // Đọc giá trị analog 0-4095 (12-bit ADC)
  
  // Kiểm tra ngưỡng cảnh báo cháy
  if (smokeLevel > SMOKE_THRESHOLD) {  // Nếu vượt ngưỡng 400
    if (!fireAlarm) {  // Chỉ kích hoạt 1 lần (tránh spam)
      fireAlarm = true;  // Đánh dấu trạng thái cháy
      Serial.println("🔥 CẢNH BÁO: Phát hiện khói/gas cao!");
      beepWarning();  // Kêu còi cảnh báo (3 tiếng beep nhanh)
    }
  } else {
    fireAlarm = false;
  }
}

// Đọc tất cả cảm biến
void readAllSensors() {
  readSHT31();
  //readMQ2();
  
  // Chỉ in khi có thay đổi đáng kể hoặc cảnh báo (tránh spam Serial Monitor)
  static float lastPrintTemp = -999;
  static float lastPrintHumi = -999;
  static int lastPrintSmoke = -999;
  static bool lastPrintFireAlarm = false;
  static unsigned long lastPrintTime = 0;
  
  bool tempChanged = abs(temperature - lastPrintTemp) >= 1.0;     // Thay đổi >= 1°C
  bool humiChanged = abs(humidity - lastPrintHumi) >= 5.0;        // Thay đổi >= 5%
  bool smokeChanged = abs(smokeLevel - lastPrintSmoke) >= 100;    // Thay đổi >= 100
  bool fireChanged = (fireAlarm != lastPrintFireAlarm);
  bool timeToPrint = (millis() - lastPrintTime >= 30000);         // In mỗi 30s
  
  // Chỉ in khi: có thay đổi đáng kể, có cảnh báo cháy, hoặc đã lâu chưa in
  if (tempChanged || humiChanged || smokeChanged || fireChanged || fireAlarm || timeToPrint) {
    Serial.print("🌡️ Nhiệt độ: ");
    Serial.print(temperature, 1);
    Serial.print("°C | Độ ẩm: ");
    Serial.print(humidity, 1);
    Serial.print("% | Khói: ");
    Serial.print(smokeLevel);
    if (fireAlarm) Serial.print(" ⚠️ NGUY HIỂM!");
    Serial.println();
    
    lastPrintTemp = temperature;
    lastPrintHumi = humidity;
    lastPrintSmoke = smokeLevel;
    lastPrintFireAlarm = fireAlarm;
    lastPrintTime = millis();
  }
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

// ================= RTC BACKUP (CMOS) =================
// DS3231 se tiep tuc dem khi mat nguon chinh neu VBAT co pin CMOS.
byte rtcReadRegister(byte reg) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

void rtcWriteRegister(byte reg, byte value) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void setupRTCBackupMode() {
  // Control reg 0x0E, bit7 EOSC: 0 = oscillator chay, 1 = dung oscillator.
  byte controlReg = rtcReadRegister(0x0E);
  if (controlReg & 0x80) {
    controlReg &= ~0x80;
    rtcWriteRegister(0x0E, controlReg);
    Serial.println("[RTC] Bat backup oscillator (EOSC=0)");
  }

  // Doc lai de xac nhan.
  controlReg = rtcReadRegister(0x0E);
  if (controlReg & 0x80) {
    Serial.println("[RTC] Loi: EOSC van = 1, RTC co the khong dem bang pin");
  }

  // Status reg 0x0F, bit7 OSF: 1 = dao dong da tung dung (pin/nguon co van de).
  byte statusReg = rtcReadRegister(0x0F);
  if (statusReg & 0x80) {
    Serial.println("[RTC] Canh bao: Dao dong tung dung, kiem tra pin CMOS va set lai gio");
    statusReg &= ~0x80;
    rtcWriteRegister(0x0F, statusReg);
  }
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
                  String timeOut, String duration) {
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
  // ═══════════════════════════════════════════════════════════
  // TẮT SERIAL MONITOR - BỎ comment dòng dưới để BẬT lại
  // ═══════════════════════════════════════════════════════════
  Serial.begin(9600);
  
  printESP32MacAddress();

  // Khởi tạo I2C cho DS3231 & SHT31
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("✓ I2C OK (SDA=21, SCL=22)");
  
  // Kiểm tra DS3231 RTC
  Wire.beginTransmission(0x68);  // Địa chỉ I2C của DS3231
  if (Wire.endTransmission() == 0) {
    Serial.println("✓ DS3231 RTC được phát hiện!");

    // Dam bao RTC van dem khi mat nguon chinh va chi con pin CMOS.
    setupRTCBackupMode();
    
    // Set thoi gian RTC (chi chay 1 lan, sau do comment lai)
    //setRTCTime(2026, 4, 23, 8, 0, 0);
    
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
  rfid.PCD_SetAntennaGain(rfid.RxGain_max);  // Tang gain anten toi da cho chip clone
  Serial.print("RFID firmware: ");
  rfid.PCD_DumpVersionToSerial();
  Serial.print("Antenna gain: ");
  Serial.println(rfid.PCD_GetAntennaGain(), HEX);

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

  servoIn.write(closeIn);
  servoOut.write(closeOut);

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

  Serial.println("═══════════════════════════════════════");
  Serial.println("   HỆ THỐNG BÃI ĐỖ XE THÔNG MINH");
  Serial.println("   LOGIC: IR + RFID = MỞ CỔNG");
  Serial.println("═══════════════════════════════════════");
  beepSuccess();
  Serial.println("✓ Sẵn sàng!\n");
}

// ================= XỬ LÝ CỔNG VÀO (STATE MACHINE) =================
void handleGateIn() {
  bool vehiclePresent = isVehicleDetected(IR_GATE_IN);
  
  switch (gateInState) {
    case GATE_CLOSED:
      // Phát hiện xe → Chuyển sang chờ quét thẻ
      if (vehiclePresent) {
        gateInState = GATE_WAIT_CARD;
        gateInTimer = millis();
        Serial.println("🚗 Phat hien xe tai cong VAO - Vui long quet the!");
        beepSuccess();  // Beep báo đã phát hiện xe
      }
      break;
      
    case GATE_WAIT_CARD:
      // ═══════════════════════════════════════════════════════════════════════════
      // LOGIC ĐỒNG THỜI: CHỈ MỞ CỔNG KHI CẢ 2 ĐIỀU KIỆN ĐỦ
      // ═══════════════════════════════════════════════════════════════════════════
      // ĐIỀU KIỆN 1: ✅ IR phát hiện xe (vehiclePresent = TRUE)
      // ĐIỀU KIỆN 2: ✅ RFID hợp lệ (isValidUID = TRUE)
      // 
      // Nếu thiếu 1 trong 2 → KHÔNG MỞ CỔNG
      // ═══════════════════════════════════════════════════════════════════════════
      
      if (pendingCardUID != "") {
        // Kiểm tra xe vẫn còn ở cổng (ĐIỀU KIỆN 1)
        if (!vehiclePresent) {
          Serial.println("❌ Xe roi di trong khi quet the!");
          beepError();
          gateInState = GATE_CLOSED;
          pendingCardUID = "";
        }
        // Kiểm tra thẻ hợp lệ (ĐIỀU KIỆN 2)
        else if (isValidUID(pendingCardUID)) {
          if (!isCheckedIn(pendingCardUID)) {
            // XE VÀO - Kiểm tra chỗ trống
            updateSlot();
            if (slotAvailable > 0) {
              // ╔═══════════════════════════════════════════════════╗
              // ║  ✅✅ CẢ 2 ĐIỀU KIỆN ĐỦ → MỞ CỔNG                ║
              // ║  1. vehiclePresent = TRUE (IR phát hiện)         ║
              // ║  2. isValidUID = TRUE (RFID hợp lệ)              ║
              // ╚═══════════════════════════════════════════════════╝
              checkInUID(pendingCardUID);
              beepSuccess();  // Beep 1 nhịp 100ms trước khi mở cổng
              gateInState = GATE_OPENING;
              gateInTimer = millis();
              servoIn.write(openIn);
              Serial.println("✅ Mo cong VAO - IR ✓ + RFID ✓");
            } else {
              Serial.println("❌ Bai DAY - Khong mo cong");
              beepError();
              gateInState = GATE_CLOSED;
            }
          } else {
            // Thẻ đã check-in rồi → Từ chối
            Serial.println("❌ The nay da vao bai roi!");
            beepError();
            gateInState = GATE_CLOSED;
          }
        } else {
          // Thẻ không hợp lệ
          Serial.println("❌ The khong hop le");
          beepError();
          gateInState = GATE_CLOSED;
        }
        pendingCardUID = "";  // Reset thẻ đã xử lý
      }

      // Xe rời đi → Hủy chờ
      else if (!vehiclePresent) {
        Serial.println("⚠️ Xe roi di truoc khi quet the");
        gateInState = GATE_CLOSED;
      }
      break;
      
    case GATE_OPENING:
      // Chuyển sang trạng thái mở sau khi servo xoay xong
      if (millis() - gateInTimer >= 500) {  // Chờ servo 500ms
        gateInState = GATE_OPEN;
        gateInTimer = millis();
      }
      break;
      
    case GATE_OPEN:
      // Giữ cổng mở, chờ xe qua
      if (!vehiclePresent && millis() - gateInTimer >= GATE_HOLD_TIME) {
        // Xe đã qua → Chuẩn bị đóng
        gateInState = GATE_CLOSING;
        gateInTimer = millis();
      }
      break;
      
    case GATE_CLOSING:
      // Đóng cổng sau delay
      if (millis() - gateInTimer >= GATE_CLOSE_DELAY) {
        servoIn.write(closeIn);
        gateInState = GATE_CLOSED;
        Serial.println("🚪 Dong cong VAO");
      }
      break;
  }
}

// ╔══════════════════════════════════════════════════════════════════════════╗
// ║                    XỬ LÝ CỔNG RA (ĐƠN GIẢN)                             ║
// ╠══════════════════════════════════════════════════════════════════════════╣
// ║  QUY TRÌNH:                                                              ║
// ║  1. IR phát hiện xe → Chờ quét thẻ                                       ║
// ║  2. Quét thẻ RFID → Kiểm tra đã check-in chưa                            ║
// ║  3. Mở cổng cho xe ra                                                    ║
// ║  4. Xe qua cổng → Tự động đóng                                           ║
// ╚══════════════════════════════════════════════════════════════════════════╝
void handleGateOut() {
  // ─────────────────────────────────────────────────────────────────────────
  // BƯỚC 1: ĐỌC CẢM BIẾN IR
  // ─────────────────────────────────────────────────────────────────────────
  bool coXe = isVehicleDetected(IR_GATE_OUT);  // TRUE = có xe, FALSE = không xe
  
  // ─────────────────────────────────────────────────────────────────────────
  // STATE MACHINE: 5 TRẠNG THÁI
  // ─────────────────────────────────────────────────────────────────────────
  switch (gateOutState) {
    
    // ═══════════════════════════════════════════════════════════════════════
    // TRẠNG THÁI 1: CỔNG ĐÓNG - CHỜ XE ĐẾN
    // ═══════════════════════════════════════════════════════════════════════
    case GATE_CLOSED:
      if (coXe) {  // IR phát hiện xe
        gateOutState = GATE_WAIT_CARD;  // Chuyển sang trạng thái chờ thẻ
        gateOutTimer = millis();         // Lưu thời điểm xe đến
        Serial.println("🚗 Co xe tai cong RA - Vui long quet the RFID!");
        beepSuccess();  // Beep báo hiệu
      }
      break;
      
    // ═══════════════════════════════════════════════════════════════════════
    // TRẠNG THÁI 2: CHỜ QUÉT THẺ RFID
    // ═══════════════════════════════════════════════════════════════════════
    case GATE_WAIT_CARD:
      
      // ─── TRƯỜNG HỢP 1: ĐÃ QUÉT ĐƯỢC THẺ ───
      if (pendingCardUID != "") {
        
        // ✅ Kiểm tra xe vẫn còn ở đó không?
        if (!coXe) {
          Serial.println("❌ Xe da roi di!");
          beepError();
          gateOutState = GATE_CLOSED;  // Reset về đóng cổng
          pendingCardUID = "";          // Xóa thẻ
        }
        
        // ✅ Kiểm tra thẻ có hợp lệ không?
        else if (isValidUID(pendingCardUID)) {
          
          // ✅ Kiểm tra thẻ đã check-in (đã ở trong bãi) chưa?
          if (isCheckedIn(pendingCardUID)) {
            
            // ╔═══════════════════════════════════════════════════╗
            // ║  ✅✅✅ ĐỦ ĐIỀU KIỆN → MỞ CỔNG!                  ║
            // ║  1. Xe đang ở cổng (IR = TRUE)                   ║
            // ║  2. Thẻ hợp lệ (trong danh sách)                 ║
            // ║  3. Thẻ đã check-in (đã ở trong bãi)             ║
            // ╚═══════════════════════════════════════════════════╝
            
            checkOutUID(pendingCardUID);  // Xóa thẻ khỏi bãi
            beepSuccess();                 // Beep thành công
            
            gateOutState = GATE_OPENING;   // Chuyển sang trạng thái mở
            gateOutTimer = millis();        // Lưu thời điểm bắt đầu mở
            servoOut.write(openOut);        // Ra lệnh servo mở (0 độ)
            
            Serial.println("✅✅ MO CONG RA - The: " + pendingCardUID);
            
          } else {
            // ❌ Thẻ chưa check-in (không có trong bãi)
            Serial.println("❌ The nay chua vao bai!");
            beepError();
            gateOutState = GATE_CLOSED;
          }
          
        } else {
          // ❌ Thẻ không hợp lệ (không trong danh sách)
          Serial.println("❌ The khong hop le!");
          beepError();
          gateOutState = GATE_CLOSED;
        }
        
        pendingCardUID = "";  // ✅ Reset thẻ đã xử lý xong
      }
      
      // ─── TRƯỜNG HỢP 2: XE RỜI ĐI TRƯỚC KHI QUÉT THẺ ───
      else if (!coXe) {
        Serial.println("⚠️ Xe roi di truoc khi quet the");
        gateOutState = GATE_CLOSED;  // Reset
      }
      break;
      
    // ═══════════════════════════════════════════════════════════════════════
    // TRẠNG THÁI 3: ĐANG MỞ CỔNG
    // ═══════════════════════════════════════════════════════════════════════
    case GATE_OPENING:
      // Chờ servo xoay xong (500ms)
      if (millis() - gateOutTimer >= 500) {
        gateOutState = GATE_OPEN;  // Chuyển sang trạng thái đã mở
        gateOutTimer = millis();    // Reset timer
        Serial.println("⬆️ Cong da MO");
      }
      break;
      
    // ═══════════════════════════════════════════════════════════════════════
    // TRẠNG THÁI 4: CỔNG ĐÃ MỞ - CHỜ XE QUA
    // ═══════════════════════════════════════════════════════════════════════
    case GATE_OPEN:
      // Nếu xe đã qua (IR không còn phát hiện) VÀ đã chờ đủ 3 giây
      if (!coXe && millis() - gateOutTimer >= GATE_HOLD_TIME) {
        gateOutState = GATE_CLOSING;  // Chuyển sang trạng thái đóng
        gateOutTimer = millis();
        Serial.println("⬇️ Xe da qua - Chuan bi dong cong...");
      }
      break;
      
    // ═══════════════════════════════════════════════════════════════════════
    // TRẠNG THÁI 5: ĐANG ĐÓNG CỔNG
    // ═══════════════════════════════════════════════════════════════════════
    case GATE_CLOSING:
      // Chờ delay 2 giây rồi đóng
      if (millis() - gateOutTimer >= GATE_CLOSE_DELAY) {
        servoOut.write(closeOut);    // Ra lệnh servo đóng (90 độ)
        gateOutState = GATE_CLOSED;   // Reset về trạng thái ban đầu
        Serial.println("🚪 DONG CONG RA hoan tat");
      }
      break;
  }
}

// ================= RFID SCAN (NON-BLOCKING với TIMEOUT) =================
void handleRFIDScan() {
  static unsigned long lastScanAttempt = 0;
  const unsigned long SCAN_INTERVAL = 100;  // Quét mỗi 100ms
  
  // Chỉ quét RFID khi đang chờ thẻ (tối ưu CPU)
  if (gateInState != GATE_WAIT_CARD && gateOutState != GATE_WAIT_CARD) {
    return;
  }
  
  // Non-blocking scan interval
  if (millis() - lastScanAttempt < SCAN_INTERVAL) {
    return;
  }
  lastScanAttempt = millis();
  
  // Quét thẻ (nhanh, không blocking)
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    String uid = readUID();
    Serial.print("📇 The quet: ");
    Serial.println(uid);
    
    pendingCardUID = uid;  // Lưu thẻ để state machine xử lý
    
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
  }
}

// ================= LOOP (100% NON-BLOCKING) =================
void loop() {
  // 1. Duy trì kết nối MQTT
  mqtt.loop();
  
  // 2. Đọc cảm biến môi trường mỗi 500ms
  static unsigned long lastSensorRead = 0;
  if (millis() - lastSensorRead >= 500) {
    lastSensorRead = millis();
    readAllSensors();
  }
  
  // 3. Gửi MQTT mỗi 5s (chỉ khi có thay đổi)
  if (millis() - lastTeleSent >= TELE_PERIOD_MS) {
    lastTeleSent = millis();
    updateSlot();
    publishParkingStatus();
  }

  // 4. CHẾ ĐỘ KHẨN CẤP: Cháy → Mở tất cả cổng
  if (fireAlarm) {
    // Force mở cổng
    if (gateInState != GATE_OPEN) {
      servoIn.write(openIn);
      gateInState = GATE_OPEN;
    }
    if (gateOutState != GATE_OPEN) {
      servoOut.write(openOut);
      gateOutState = GATE_OPEN;
    }
    
    static unsigned long lastBeep = 0;
    if (millis() - lastBeep >= 2000) {
      lastBeep = millis();
      beepWarning();
      Serial.println("🚨 KHẨN CẤP! Khói: " + String(smokeLevel));
    }
    return;  // Bỏ qua logic thường trong chế độ khẩn cấp
  }

  // 5. Xử lý state machine cổng VÀO
  handleGateIn();
  
  // 6. Xử lý state machine cổng RA
  handleGateOut();
  
  // 7. Quét RFID (chỉ khi cần)
  handleRFIDScan();
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

