#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <WiFi.h>

// ================= RFID =================
// Chân giao tiếp giữa ESP32 và module RC522.
#define SS_PIN 5
#define RST_PIN 22
MFRC522 rfid(SS_PIN, RST_PIN);

// ================= SERVO =================
// Hai servo điều khiển cổng vào và cổng ra.
Servo servoIn;
Servo servoOut;

#define SERVO_IN_PIN 13
#define SERVO_OUT_PIN 12

// ================= IR SENSOR =================
// Mỗi cảm biến IR tương ứng một vị trí đỗ xe.
#define IR1 32
#define IR2 33
#define IR3 25
#define IR4 26

// Hai cảm biến IR đặt tại cổng vào và cổng ra để phát hiện xe đang đi qua barrier.
#define IR_GATE_IN 27
#define IR_GATE_OUT 14

// ================= BIẾN =================
// Tổng số chỗ của bãi và số chỗ trống hiện tại.
int totalSlot = 4;
int slotAvailable = 4;
int lastReportedSlot = -1;

// Góc mở/đóng servo và thời gian chờ xe đi vào vùng cảm biến cổng.
const int GATE_OPEN_ANGLE = 90;
const int GATE_CLOSE_ANGLE = 0;
const unsigned long GATE_WAIT_TIMEOUT = 5000;
const unsigned long GATE_CHECK_INTERVAL = 50;
const unsigned long GATE_CLEAR_STABLE_TIME = 500;

// Cấu hình mức tín hiệu của cảm biến.
const int GATE_SENSOR_ACTIVE_STATE = LOW;
const int SLOT_EMPTY_STATE = HIGH;

// Danh sách UID thẻ được phép vào bãi.
String validUIDs[] = {"a1b2c3d4", "11223344"};

// Thong tin WiFi
const char* ssid = "Quang Vinh";
const char* password = "0978020252";
const unsigned long WIFI_CONNECT_TIMEOUT = 15000;//ms

void printESP32MacAddress() {
  // In MAC cua giao dien WiFi STA de dung cho dinh danh thiet bi.
  WiFi.mode(WIFI_STA);
  String mac = WiFi.macAddress();

  Serial.print("MAC ESP32 (STA): ");
  Serial.println(mac);
}

bool isVehicleDetected(int sensorPin) {
  return digitalRead(sensorPin) == GATE_SENSOR_ACTIVE_STATE;
}

bool isSlotEmpty(int sensorPin) {
  return digitalRead(sensorPin) == SLOT_EMPTY_STATE;
}

void waitForVehicleToPass(int sensorPin) {
  unsigned long startTime = millis();
  unsigned long clearStart = 0;

  // Chờ xe đi tới đúng vị trí cảm biến sau khi cổng mở.
  while (!isVehicleDetected(sensorPin) && millis() - startTime < GATE_WAIT_TIMEOUT) {
    delay(GATE_CHECK_INTERVAL);
  }

  // Nếu quá timeout mà chưa thấy xe thì đóng cổng để tránh mở lâu.
  if (!isVehicleDetected(sensorPin)) {
    return;
  }

  // Chỉ đóng cổng khi cảm biến báo không còn xe liên tục đủ lâu.
  while (true) {
    if (isVehicleDetected(sensorPin)) {
      clearStart = 0;
    } else {
      if (clearStart == 0) {
        clearStart = millis();
      }

      if (millis() - clearStart >= GATE_CLEAR_STABLE_TIME) {
        break;
      }
    }

    delay(GATE_CHECK_INTERVAL);
  }
}

// Ket noi WiFi, tra ve true neu thanh cong.
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

// ================= SETUP =================
void setup() {
  // Khởi động Serial để theo dõi trạng thái hệ thống.
  Serial.begin(9600);

  // In dia chi MAC cua ESP32 de quan ly thiet bi.
  printESP32MacAddress();

  // Kết nối Wi-Fi ngay khi hệ thống khởi động.
  if (!connectWiFi()) {
    Serial.println("He thong van chay offline.");
  }

  // Khởi tạo giao tiếp SPI và đầu đọc RFID.
  SPI.begin();
  rfid.PCD_Init();

  // Gắn servo vào đúng chân điều khiển.
  servoIn.attach(SERVO_IN_PIN);
  servoOut.attach(SERVO_OUT_PIN);

  // Cấu hình 4 cảm biến IR là ngõ vào.
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);

  // Cấu hình 2 cảm biến tại cổng vào và cổng ra.
  pinMode(IR_GATE_IN, INPUT);
  pinMode(IR_GATE_OUT, INPUT);

  // Đặt cổng ở trạng thái đóng ban đầu.
  servoIn.write(GATE_CLOSE_ANGLE);
  servoOut.write(GATE_CLOSE_ANGLE);

  Serial.println("=== HE THONG BAO DO XE ===");
}

// ================= LOOP =================
void loop() {
  // Cập nhật số chỗ trống trước khi xử lý thẻ.
  updateSlot();

  // Nếu chưa có thẻ mới hoặc đọc thẻ lỗi thì bỏ qua vòng lặp này.
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial())
    return;

  // Đọc UID và in ra Serial để kiểm tra.
  String uid = readUID();
  Serial.print("The quet: ");
  Serial.println(uid);

  // Chỉ mở cổng khi thẻ hợp lệ.
  // Ưu tiên mở cổng ra nếu xe đang đứng ở cảm biến cổng ra.
  if (isValidUID(uid)) {
    if (isVehicleDetected(IR_GATE_OUT)) {
      openGateOut();
    } else if (isVehicleDetected(IR_GATE_IN)) {
      if (slotAvailable > 0) {
        openGateIn();
      } else {
        Serial.println("❌ Het cho!");
      }
    } else {
      Serial.println("⚠️ Dua xe vao dung vi tri cam bien cong vao/ra roi quet lai the.");
    }
  } else {
    Serial.println("❌ The khong hop le!");
  }

  // Kết thúc giao tiếp với thẻ để tránh đọc lặp không mong muốn.
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  // Tránh đọc lặp quá nhanh cùng một thẻ.
  delay(1000);
}

// ================= HÀM ĐỌC UID =================
String readUID() {
  String uid = "";

  // Ghép từng byte UID thành chuỗi HEX.
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) {
      uid += "0";
    }
    uid += String(rfid.uid.uidByte[i], HEX);
  }

  // Chuẩn hóa về chữ thường để so sánh dễ hơn.
  uid.toLowerCase();
  return uid;
}

// ================= KIỂM TRA UID =================
bool isValidUID(String uid) {
  // Duyệt toàn bộ danh sách UID hợp lệ.
  for (int i = 0; i < sizeof(validUIDs) / sizeof(validUIDs[0]); i++) {
    if (uid == validUIDs[i]) return true;
  }
  return false;
}

// ================= CẬP NHẬT SLOT =================
void updateSlot() {
  int count = 0;

  // Mỗi cảm biến ở mức cấu hình SLOT_EMPTY_STATE được tính là một chỗ đang trống.
  if (isSlotEmpty(IR1)) count++;
  if (isSlotEmpty(IR2)) count++;
  if (isSlotEmpty(IR3)) count++;
  if (isSlotEmpty(IR4)) count++;

  // Giới hạn kết quả trong phạm vi số chỗ thực tế của bãi.
  if (count > totalSlot) {
    count = totalSlot;
  }

  slotAvailable = count;

  // Chỉ in ra khi số chỗ trống thay đổi để tránh spam Serial Monitor.
  if (slotAvailable != lastReportedSlot) {
    Serial.print("Cho trong: ");
    Serial.println(slotAvailable);
    lastReportedSlot = slotAvailable;
  }
}

// ================= CỔNG VÀO =================
void openGateIn() {
  Serial.println("✅ Mo cong vao");

  // Mở cổng vào và giữ mở đến khi xe đi qua hết cảm biến cổng vào.
  servoIn.write(GATE_OPEN_ANGLE);
  waitForVehicleToPass(IR_GATE_IN);

  // Đóng cổng sau khi xe đi qua.
  servoIn.write(GATE_CLOSE_ANGLE);
  Serial.println("Dong cong vao");
}

// ================= CỔNG RA =================
void openGateOut() {
  Serial.println("🚗 Mo cong ra");

  // Mở cổng ra và giữ mở đến khi xe đi qua hết cảm biến cổng ra.
  servoOut.write(GATE_OPEN_ANGLE);
  waitForVehicleToPass(IR_GATE_OUT);

  // Đóng cổng ra sau khi xe đi qua.
  servoOut.write(GATE_CLOSE_ANGLE);
  Serial.println("Dong cong ra");
}