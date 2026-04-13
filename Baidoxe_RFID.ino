#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <WiFi.h>
//#include <DS3231.h>
//#include <SHT31.h>


// ================= RFID RC522 =================
#define SS_PIN 5       
#define RST_PIN 4      
// SPI pins (tự động):
// MOSI: IO23 (J3_2)
// MISO: IO19 (J3_8)
// SCK:  IO18 (J3_9)
MFRC522 rfid(SS_PIN, RST_PIN);

// ================= SERVO =================
// Hai servo điều khiển cổng vào và cổng ra (J2 - bên trái)
Servo servoIn;
Servo servoOut;

#define SERVO_IN_PIN 14    // J2_12 - IO14
#define SERVO_OUT_PIN 13   // J2_15 - IO13

// ================= IR SENSOR - HỒNG NGOẠI =================
// HN1-HN4: 4 cảm biến slot đỗ xe (J2 - bên trái)
#define IR1 34         // J2_5 - HN1 (Input only)
#define IR2 35         // J2_6 - HN2 (Input only)
#define IR3 32         // J2_7 - HN3
#define IR4 33         // J2_8 - HN4

// HN5-HN6: 2 cảm biến cổng vào/ra phát hiện xe đi qua barrier
#define IR_GATE_IN 25  // J2_9 - HN5 (Cổng vào)
#define IR_GATE_OUT 26 // J2_10 - HN6 (Cổng ra)

// ================= BUZZER (tùy chọn) =================
#define BUZZER_PIN 17  // J3_11 - IO17

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

// Danh sách UID thẻ được phép vào bãi (thẻ master) - THAY ĐỔI THEO THẺ THẬT!
// HƯỚNG DẪN: Quét từng thẻ RFID → Xem UID trên Serial Monitor → Thay vào đây
// Ví dụ: "4a8b3c2d", "f1e2d3c4", "a1b2c3d4", "12345678"
String validUIDs[] = {"a1b2c3d4", "11223344", "55667788", "99aabbcc"};

// Danh sách UID các xe đang ở trong bãi (đã vào nhưng chưa ra).
String checkedInUIDs[4] = {"", "", "", ""};  // Tối đa 4 xe (theo số slot)
int checkedInCount = 0;

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

/*
 * HÀM: isVehicleDetected
 * MỤC ĐÍCH: Kiểm tra có xe đang ở cổng không
 * THAM SỐ: sensorPin - chân cảm biến IR cần kiểm tra
 * TRẢ VỀ: true = có xe, false = không có xe
 * VÍ DỤ: if (isVehicleDetected(IR_GATE_IN)) { ... }
 */
bool isVehicleDetected(int sensorPin) {
  return digitalRead(sensorPin) == GATE_SENSOR_ACTIVE_STATE;
}

/*
 * HÀM: isSlotEmpty
 * MỤC ĐÍCH: Kiểm tra slot đỗ xe có trống không
 * THAM SỐ: sensorPin - chân cảm biến IR của slot
 * TRẢ VỀ: true = slot trống, false = có xe đỗ
 * VÍ DỤ: if (isSlotEmpty(IR1)) { ... }
 */
bool isSlotEmpty(int sensorPin) {
  return digitalRead(sensorPin) == SLOT_EMPTY_STATE;
}

/*
 * HÀM: waitForVehicleToPass
 * MỤC ĐÍCH: Giữ cổng mở cho đến khi xe đi qua hoàn toàn
 * THAM SỐ: sensorPin - chân cảm biến tại cổng cần theo dõi
 * CÁCH HOẠT ĐỘNG:
 *   Bước 1: Chờ xe đi tới cảm biến (timeout 5s)
 *   Bước 2: Phát hiện xe → giữ cổng mở
 *   Bước 3: Xe đi qua → cảm biến trống ổn định 500ms → đóng cổng
 * VÍ DỤ: waitForVehicleToPass(IR_GATE_IN);
 */
void waitForVehicleToPass(int sensorPin) {
  unsigned long startTime = millis();
  unsigned long clearStart = 0;

  // BƯỚC 1: Chờ xe đi tới đúng vị trí cảm biến sau khi cổng mở
  // Timeout 5 giây để tránh cổng mở mãi
  while (!isVehicleDetected(sensorPin) && millis() - startTime < GATE_WAIT_TIMEOUT) {
    delay(GATE_CHECK_INTERVAL);  // Kiểm tra mỗi 50ms
  }

  // BƯỚC 2: Nếu quá timeout mà chưa thấy xe → đóng cổng luôn
  if (!isVehicleDetected(sensorPin)) {
    return;
  }

  // BƯỚC 3: Đã phát hiện xe → chờ xe đi qua hết
  // Chỉ đóng cổng khi cảm biến báo "không có xe" liên tục đủ 500ms
  while (true) {
    if (isVehicleDetected(sensorPin)) {
      // Vẫn còn xe → reset bộ đếm
      clearStart = 0;
    } else {
      // Không còn xe → bắt đầu đếm thời gian
      if (clearStart == 0) {
        clearStart = millis();
      }

      // Đã trống ổn định đủ 500ms → an toàn để đóng cổng
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

// ================= BUZZER - BÁO HIỆU =================

/*
 * HÀM: beepSuccess
 * MỤC ĐÍCH: Phát âm thanh báo thành công
 * ÂM THANH: 1 tiếng BÍP ngắn (100ms)
 * SỬ DỤNG KHI: Check-in thành công, check-out thành công
 */
void beepSuccess() {
  digitalWrite(BUZZER_PIN, HIGH);  // Bật buzzer
  delay(100);                       // Bíp 100ms
  digitalWrite(BUZZER_PIN, LOW);   // Tắt buzzer
}

/*
 * HÀM: beepError
 * MỤC ĐÍCH: Phát âm thanh báo lỗi
 * ÂM THANH: 2 tiếng BÍP dài (200ms mỗi tiếng)
 * SỬ DỤNG KHI: Thẻ không hợp lệ, thẻ chưa check-in
 */
void beepError() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER_PIN, HIGH);  // Bật buzzer
    delay(200);                       // Bíp dài 200ms
    digitalWrite(BUZZER_PIN, LOW);   // Tắt buzzer
    delay(100);                       // Nghỉ 100ms giữa 2 tiếng
  }
}

/*
 * HÀM: beepWarning
 * MỤC ĐÍCH: Phát âm thanh cảnh báo
 * ÂM THANH: 3 tiếng BÍP nhanh (50ms mỗi tiếng)
 * SỬ DỤNG KHI: Thẻ đã vào rồi, hết chỗ, sai vị trí
 */
void beepWarning() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);  // Bật buzzer
    delay(50);                        // Bíp nhanh 50ms
    digitalWrite(BUZZER_PIN, LOW);   // Tắt buzzer
    delay(50);                        // Nghỉ 50ms giữa các tiếng
  }
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
  
  // Cấu hình Buzzer là ngõ ra (tùy chọn)
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Tắt buzzer ban đầu

  // Đặt cổng ở trạng thái đóng ban đầu.
  servoIn.write(GATE_CLOSE_ANGLE);
  servoOut.write(GATE_CLOSE_ANGLE);

  Serial.println("=== HE THONG BAO DO XE ===");
  Serial.println("So do chan: xem file PINOUT.md");
  Serial.println("----------------------------------");
  delay(500);
  
  // Test buzzer khởi động
  beepSuccess();
  Serial.println("✓ He thong san sang!");
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

  // Xử lý logic cổng vào và cổng ra khác nhau.
  if (isVehicleDetected(IR_GATE_OUT)) {
    // === CỔNG RA: Kiểm tra thẻ có trong danh sách đã check-in không ===
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
    
  } else {
    beepWarning();  // Cảnh báo chưa đúng vị trí
    Serial.println("⚠️ Dua xe vao dung vi tri cam bien cong vao/ra roi quet lai the.");
  }

  // Kết thúc giao tiếp với thẻ để tránh đọc lặp không mong muốn.
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  // Tránh đọc lặp quá nhanh cùng một thẻ.
  delay(1000);
}

// ================= HÀM ĐỌC UID =================

/*
 * HÀM: readUID
 * MỤC ĐÍCH: Đọc mã UID từ thẻ RFID và chuyển thành chuỗi HEX
 * TRẢ VỀ: Chuỗi UID dạng "a1b2c3d4" (chữ thường)
 * CÁCH HOẠT ĐỘNG:
 *   - UID thẻ có 4 hoặc 7 bytes
 *   - Mỗi byte chuyển thành 2 ký tự HEX
 *   - Ví dụ: Byte 161 → "a1", Byte 5 → "05"
 * VÍ DỤ: String uid = readUID(); // "a1b2c3d4"
 */
String readUID() {
  String uid = "";

  // Duyệt qua từng byte của UID
  for (byte i = 0; i < rfid.uid.size; i++) {
    // Nếu byte < 16 (0x10) thì thêm số 0 phía trước
    // Ví dụ: 5 → "05", 15 → "0F" (đảm bảo 2 ký tự)
    if (rfid.uid.uidByte[i] < 0x10) {
      uid += "0";
    }
    // Chuyển byte sang HEX và nối vào chuỗi
    // Ví dụ: 161 → "A1", 255 → "FF"
    uid += String(rfid.uid.uidByte[i], HEX);
  }

  // Chuyển toàn bộ về chữ thường để dễ so sánh
  // "A1B2C3D4" → "a1b2c3d4"
  uid.toLowerCase();
  return uid;
}

// ================= KIỂM TRA UID =================

/*
 * HÀM: isValidUID
 * MỤC ĐÍCH: Kiểm tra thẻ có phải là thẻ được phép vào bãi không
 * THAM SỐ: uid - mã UID của thẻ cần kiểm tra
 * TRẢ VỀ: true = thẻ hợp lệ (có trong danh sách master)
 *         false = thẻ không hợp lệ (không có trong danh sách)
 * DANH SÁCH MASTER: validUIDs[] = {"a1b2c3d4", "11223344", ...}
 * VÍ DỤ:
 *   isValidUID("a1b2c3d4") → true  (có trong danh sách)
 *   isValidUID("ffffffff") → false (không có trong danh sách)
 */
bool isValidUID(String uid) {
  // Tính số lượng thẻ trong danh sách master
  int totalValidCards = sizeof(validUIDs) / sizeof(validUIDs[0]);
  
  // Duyệt qua từng thẻ trong danh sách
  for (int i = 0; i < totalValidCards; i++) {
    // So sánh UID (chuyển cả 2 về chữ thường để không phân biệt hoa/thường)
    String validUID = validUIDs[i];
    validUID.toLowerCase();
    if (uid == validUID) {
      return true;  // Tìm thấy → thẻ hợp lệ
    }
  }
  
  // Duyệt hết mà không trùng → thẻ không hợp lệ
  return false;
}

// ================= KIỂM TRA UID ĐÃ CHECK-IN =================

/*
 * HÀM: isCheckedIn
 * MỤC ĐÍCH: Kiểm tra xe đã vào bãi chưa
 * THAM SỐ: uid - mã UID của thẻ cần kiểm tra
 * TRẢ VỀ: true = xe đã check-in (đang trong bãi)
 *         false = xe chưa check-in
 * DANH SÁCH CHECK-IN: checkedInUIDs[] = {"a1b2c3d4", "", "55667788", ""}
 * VÍ DỤ:
 *   isCheckedIn("a1b2c3d4") → true  (đã vào bãi)
 *   isCheckedIn("11223344") → false (chưa vào)
 */
bool isCheckedIn(String uid) {
  // Duyệt qua 4 slot check-in
  for (int i = 0; i < 4; i++) {
    // Kiểm tra 2 điều kiện:
    // 1. UID trùng với slot này
    // 2. Không phải chuỗi rỗng (slot đang có xe)
    if (checkedInUIDs[i] == uid && uid != "") {
      return true;  // Tìm thấy → xe đã check-in
    }
  }
  
  // Không tìm thấy → xe chưa check-in
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
 *   checkInUID("11223344") → true
 *   SAU:   checkedInUIDs[] = {"a1b2c3d4", "11223344", "", ""}
 */
bool checkInUID(String uid) {
  // BƯỚC 1: Kiểm tra thẻ đã check-in chưa (chống trùng lặp)
  if (isCheckedIn(uid)) {
    Serial.println("⚠️ The nay da vao bai roi!");
    return false;  // Đã vào rồi → từ chối
  }
  
  // BƯỚC 2: Tìm slot trống để lưu UID
  for (int i = 0; i < 4; i++) {
    if (checkedInUIDs[i] == "") {  // Tìm thấy slot trống
      // BƯỚC 3: Lưu thông tin check-in
      checkedInUIDs[i] = uid;      // Lưu UID vào slot
      checkedInCount++;            // Tăng số xe trong bãi
      
      Serial.print("✅ Da luu the vao bai. So xe trong bai: ");
      Serial.println(checkedInCount);
      return true;  // Lưu thành công
    }
  }
  
  // Duyệt hết 4 slot mà không có slot trống → Bãi đầy
  Serial.println("❌ Bai da day, khong the luu them!");
  return false;
}

// ================= XÓA UID KHỎI DANH SÁCH CHECK-IN =================

/*
 * HÀM: checkOutUID
 * MỤC ĐÍCH: Xóa thẻ khỏi danh sách check-in khi xe RA khỏi bãi
 * THAM SỐ: uid - mã UID của thẻ cần xóa
 * TRẢ VỀ: true = xóa thành công
 *         false = xóa thất bại (thẻ không có trong bãi)
 * FLOW:
 *   Bước 1: Tìm UID trong checkedInUIDs[]
 *   Bước 2: Nếu tìm thấy → Xóa (gán = "") → Giảm biến đếm
 *   Bước 3: Không tìm thấy → Báo lỗi
 * VÍ DỤ:
 *   TRƯỚC: checkedInUIDs[] = {"a1b2c3d4", "11223344", "", ""}
 *   checkOutUID("11223344") → true
 *   SAU:   checkedInUIDs[] = {"a1b2c3d4", "", "", ""}
 */
bool checkOutUID(String uid) {
  // BƯỚC 1 & 2: Duyệt qua 4 slot để tìm UID
  for (int i = 0; i < 4; i++) {
    // Kiểm tra slot này có chứa UID cần xóa không
    if (checkedInUIDs[i] == uid && uid != "") {
      // Tìm thấy → Xóa thông tin check-in
      checkedInUIDs[i] = "";  // Xóa = gán chuỗi rỗng
      checkedInCount--;       // Giảm số xe trong bãi
      
      Serial.print("✅ Da xoa the khoi bai. So xe trong bai: ");
      Serial.println(checkedInCount);
      return true;  // Xóa thành công
    }
  }
  
  // BƯỚC 3: Không tìm thấy UID trong danh sách
  Serial.println("❌ The nay khong co trong bai!");
  return false;
}

// ================= CẬP NHẬT SLOT =================

/*
 * HÀM: updateSlot
 * MỤC ĐÍCH: Cập nhật số chỗ trống trong bãi bằng cảm biến IR
 * CÁCH HOẠT ĐỘNG:
 *   - Đọc 4 cảm biến IR (IR1, IR2, IR3, IR4)
 *   - Mỗi cảm biến báo HIGH (SLOT_EMPTY_STATE) = slot trống
 *   - Đếm tổng số slot trống
 *   - Chỉ in ra Serial khi số chỗ thay đổi (tránh spam)
 * VÍ DỤ:
 *   IR1=HIGH, IR2=LOW, IR3=HIGH, IR4=LOW → 2 chỗ trống
 */
void updateSlot() {
  int count = 0;

  // Đếm số slot trống bằng cách kiểm tra từng cảm biến IR
  if (isSlotEmpty(IR1)) count++;  // Slot 1 trống → +1
  if (isSlotEmpty(IR2)) count++;  // Slot 2 trống → +1
  if (isSlotEmpty(IR3)) count++;  // Slot 3 trống → +1
  if (isSlotEmpty(IR4)) count++;  // Slot 4 trống → +1

  // Giới hạn trong phạm vi tối đa (phòng lỗi cảm biến)
  if (count > totalSlot) {
    count = totalSlot;  // Tối đa 4 slot
  }

  slotAvailable = count;  // Lưu kết quả

  // Chỉ in ra khi số chỗ trống THAY ĐỔI (tránh spam Serial Monitor)
  if (slotAvailable != lastReportedSlot) {
    Serial.print("Cho trong: ");
    Serial.println(slotAvailable);
    lastReportedSlot = slotAvailable;  // Cập nhật trạng thái đã báo
  }
}

// ================= CỔNG VÀO =================

/*
 * HÀM: openGateIn
 * MỤC ĐÍCH: Điều khiển servo mở/đóng cổng VÀO
 * FLOW:
 *   Bước 1: Xoay servo góc 90° → Mở cổng
 *   Bước 2: Chờ xe đi qua cảm biến IR_GATE_IN
 *   Bước 3: Xoay servo góc 0° → Đóng cổng
 * GỌI KHI: Thẻ hợp lệ + Có chỗ trống + Check-in thành công
 */
void openGateIn() {
  Serial.println("✅ Mo cong vao");

  // BƯỚC 1: Mở cổng vào (servo xoay 90 độ)
  servoIn.write(GATE_OPEN_ANGLE);
  
  // BƯỚC 2: Giữ cổng mở và chờ xe đi qua hết cảm biến
  waitForVehicleToPass(IR_GATE_IN);

  // BƯỚC 3: Đóng cổng sau khi xe đi qua (servo về 0 độ)
  servoIn.write(GATE_CLOSE_ANGLE);
  Serial.println("Dong cong vao");
}

// ================= CỔNG RA =================

/*
 * HÀM: openGateOut
 * MỤC ĐÍCH: Điều khiển servo mở/đóng cổng RA
 * FLOW:
 *   Bước 1: Xoay servo góc 90° → Mở cổng
 *   Bước 2: Chờ xe đi qua cảm biến IR_GATE_OUT
 *   Bước 3: Xoay servo góc 0° → Đóng cổng
 * GỌI KHI: Thẻ đã check-in + Check-out thành công
 */
void openGateOut() {
  Serial.println("🚗 Mo cong ra");

  // BƯỚC 1: Mở cổng ra (servo xoay 90 độ)
  servoOut.write(GATE_OPEN_ANGLE);
  
  // BƯỚC 2: Giữ cổng mở và chờ xe đi qua hết cảm biến
  waitForVehicleToPass(IR_GATE_OUT);

  // BƯỚC 3: Đóng cổng ra sau khi xe đi qua (servo về 0 độ)
  servoOut.write(GATE_CLOSE_ANGLE);
  Serial.println("Dong cong ra");
}
