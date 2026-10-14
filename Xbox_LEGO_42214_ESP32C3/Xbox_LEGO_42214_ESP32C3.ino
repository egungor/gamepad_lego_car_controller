// ============================================================
// Xbox Series X|S kumanda -> ESP32-C3 -> LEGO Technic Move Hub (88019)
// Model: LEGO Technic 42214
//
// Gereksinimler:
//   - NimBLE-Arduino 2.x  (tarama/timeout süreleri MİLİSANİYE)
//   - XboxSeriesXControllerESP32_asukiaaa (NimBLE 2.x ile uyumlu sürüm)
//   - U8g2
//
// Bağlantı stratejisi:
//   1) Açılışta, Xbox kütüphanesi başlamadan ÖNCE LEGO hub taranır ve
//      adresi RAM'de saklanır. Böylece global BLE scanner'ı Xbox
//      kütüphanesiyle paylaşma sorunu yaşanmaz.
//   2) Hub sonradan koparsa tarama yapılmaz; bilinen adrese doğrudan
//      yeniden bağlanılır (Xbox kütüphanesinin scan callback'lerine
//      dokunulmaz).
// ============================================================

#include <Arduino.h>
#include <climits>
#include <string>
#include <U8g2lib.h>
#include <NimBLEDevice.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// ============================================================
// AYARLAR
// ============================================================

#define DEBUG_BLE 0                       // 1: tüm BLE tarama sonuçlarını yazdır

static const int      DEADZONE              = 8;      // % cinsinden
static const bool     INVERT_THROTTLE       = true;   // asukiaaa: yukarı = 0
static const bool     INVERT_STEERING       = false;
static const bool     USE_SECURE_CONNECTION = true;   // hub bonding istemiyorsa false yapın

static const uint32_t SCAN_DURATION_MS      = 10000;
static const uint32_t CONNECT_TIMEOUT_MS    = 5000;
static const uint32_t RECONNECT_INTERVAL_MS = 2000;
static const uint32_t KEEPALIVE_MS          = 150;    // değişmese de komutu tekrar gönder
static const uint32_t OLED_INTERVAL_MS      = 100;
static const uint32_t BRAKE_LIGHT_MS        = 1000;   // acil stop sonrası fren ışığı süresi

// Işık byte değerleri (hub üzerinde doğrulayın)
static const uint8_t LIGHTS_ON    = 0x00;
static const uint8_t LIGHTS_OFF   = 0x04;
static const uint8_t LIGHTS_BRAKE = 0x01;  // DOĞRULANMALI

// ============================================================
// OLED (72x40, SSD1306)
// ============================================================

U8G2_SSD1306_72X40_ER_F_HW_I2C oled(
  U8G2_R0,
  U8X8_PIN_NONE,
  6,  // SCL
  5   // SDA
);

// ============================================================
// XBOX
// ============================================================

// Kumandanın MAC adresini biliyorsanız hedefli bağlanmak için:
// XboxSeriesXControllerESP32_asukiaaa::Core xboxController("xx:xx:xx:xx:xx:xx");
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

// ============================================================
// LEGO TECHNIC MOVE HUB 88019
// ============================================================

static const char* LEGO_SERVICE_UUID        = "00001623-1212-EFDE-1623-785FEABCD123";
static const char* LEGO_CHARACTERISTIC_UUID = "00001624-1212-EFDE-1623-785FEABCD123";
static const char* LEGO_HUB_NAME            = "Technic Move";

NimBLEClient*               legoClient         = nullptr;
NimBLERemoteCharacteristic* legoCharacteristic = nullptr;

NimBLEAddress  hubAddress;
volatile bool  hubAddressKnown = false;
volatile bool  legoConnected   = false;   // BLE callback'inden de yazılır

// ============================================================
// DURUM
// ============================================================

bool     armed           = false;
bool     lightsOn        = true;
uint32_t brakeUntil      = 0;
uint32_t lastReconnectMs = 0;
uint32_t lastOledMs      = 0;

// Gönderim takibi
int      sentSpeed    = INT_MIN;
int      sentSteering = INT_MIN;
int      sentLights   = INT_MIN;
uint32_t lastSendMs   = 0;

// Ekran için son komut
int curThrottle = 0;
int curSteering = 0;

// ============================================================
// YARDIMCILAR
// ============================================================

struct EdgeButton {
  bool prev = false;
  bool rose(bool now) {
    bool r = now && !prev;
    prev = now;
    return r;
  }
  void reset() { prev = false; }
};

EdgeButton btnA, btnB, btnStart;

// 0..65535 (merkez ~32768) -> -100..100, ölü bölge + yeniden ölçekleme
int axisToPercent(uint16_t raw, bool invert) {
  long centered = (long)raw - 32768L;
  int p = (int)(centered * 100L / 32768L);
  p = constrain(p, -100, 100);
  if (invert) p = -p;

  int a = abs(p);
  if (a < DEADZONE) return 0;

  int scaled = map(a, DEADZONE, 100, 0, 100);
  return p > 0 ? scaled : -scaled;
}

void forceResend() {
  sentSpeed = sentSteering = sentLights = INT_MIN;
}

void oledMessage(const char* l1, const char* l2 = "", const char* l3 = "", const char* l4 = "") {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 9,  l1);
  oled.drawStr(0, 19, l2);
  oled.drawStr(0, 29, l3);
  oled.drawStr(0, 39, l4);
  oled.sendBuffer();
}

// ============================================================
// BLE CALLBACK'LERİ
// ============================================================

class LegoScanCallbacks : public NimBLEScanCallbacks {
public:
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (dev == nullptr || hubAddressKnown) return;

    bool nameMatch = dev->haveName() &&
                     dev->getName().find(LEGO_HUB_NAME) != std::string::npos;
    bool svcMatch  = dev->isAdvertisingService(NimBLEUUID(LEGO_SERVICE_UUID));

#if DEBUG_BLE
    Serial.printf("BLE: %s | %s | 0x1623:%s\n",
                  dev->haveName() ? dev->getName().c_str() : "<isimsiz>",
                  dev->getAddress().toString().c_str(),
                  svcMatch ? "evet" : "hayir");
#endif

    if (!nameMatch && !svcMatch) return;

    hubAddress      = dev->getAddress();
    hubAddressKnown = true;
    Serial.printf("LEGO hub bulundu: %s\n", hubAddress.toString().c_str());
    NimBLEDevice::getScan()->stop();
  }
};

class LegoClientCallbacks : public NimBLEClientCallbacks {
public:
  void onDisconnect(NimBLEClient* client, int reason) override {
    // Sadece bayrak: client burada silinmez, loop() yeniden bağlanır.
    legoConnected = false;
    Serial.printf("LEGO hub baglantisi koptu (reason=%d)\n", reason);
  }
};

LegoScanCallbacks   legoScanCallbacks;
LegoClientCallbacks legoClientCallbacks;

// ============================================================
// LEGO KOMUTLARI
// ============================================================

bool legoSend(const uint8_t* data, size_t length) {
  if (!legoConnected || legoCharacteristic == nullptr) return false;
  return legoCharacteristic->writeValue(data, length, false);
}

// Birleşik sürüş çerçevesi:
// 0D 00 81 36 11 51 00 03 00 [hız] [direksiyon] [ışık] 00
bool legoDrive(int speed, int steering, uint8_t lights) {
  speed    = constrain(speed, -100, 100);
  steering = constrain(steering, -100, 100);

  uint8_t packet[13] = {
    0x0D, 0x00, 0x81, 0x36, 0x11, 0x51, 0x00, 0x03, 0x00,
    (uint8_t)(int8_t)speed,
    (uint8_t)(int8_t)steering,
    lights,
    0x00
  };
  return legoSend(packet, sizeof(packet));
}

void legoCalibrateSteering() {
  const uint8_t p1[] = { 0x0D, 0x00, 0x81, 0x36, 0x11, 0x51, 0x00, 0x03,
                         0x00, 0x00, 0x00, 0x10, 0x00 };
  const uint8_t p2[] = { 0x0D, 0x00, 0x81, 0x36, 0x11, 0x51, 0x00, 0x03,
                         0x00, 0x00, 0x00, 0x08, 0x00 };
  legoSend(p1, sizeof(p1));
  delay(100);
  legoSend(p2, sizeof(p2));
  delay(100);
}

// ============================================================
// LEGO TARAMA / BAĞLANTI
// ============================================================

// Sadece setup()'ta, Xbox kütüphanesi başlamadan önce kullanılır.
bool scanForHub() {
  hubAddressKnown = false;

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&legoScanCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(45);
  scan->setWindow(30);
  scan->clearResults();

  Serial.println("LEGO hub taraniyor...");
  if (!scan->start(SCAN_DURATION_MS, false)) {   // NimBLE 2.x: milisaniye
    Serial.println("HATA: tarama baslatilamadi");
    return false;
  }
  while (scan->isScanning()) delay(20);
  scan->clearResults();

  // Callback'i bırak; Xbox kütüphanesi kendi callback'ini kuracak.
  scan->setScanCallbacks(nullptr);

  return hubAddressKnown;
}

void dropLegoLink() {
  legoConnected      = false;
  legoCharacteristic = nullptr;
  if (legoClient != nullptr && legoClient->isConnected()) {
    legoClient->disconnect();
  }
}

bool connectToHub() {
  if (!hubAddressKnown) return false;

  if (legoClient == nullptr) {
    legoClient = NimBLEDevice::createClient();
    if (legoClient == nullptr) {
      Serial.println("HATA: BLE client olusturulamadi");
      return false;
    }
    legoClient->setClientCallbacks(&legoClientCallbacks, false);
    legoClient->setConnectTimeout(CONNECT_TIMEOUT_MS);   // NimBLE 2.x: milisaniye
  }

  Serial.printf("LEGO hub'a baglaniliyor: %s\n", hubAddress.toString().c_str());
  if (!legoClient->connect(hubAddress)) {
    Serial.println("Baglanti basarisiz");
    return false;
  }

  if (USE_SECURE_CONNECTION && !legoClient->secureConnection()) {
    Serial.println("Guvenli baglanti basarisiz");
    dropLegoLink();
    return false;
  }

  NimBLERemoteService* service = legoClient->getService(LEGO_SERVICE_UUID);
  if (service == nullptr) {
    Serial.println("LEGO servisi bulunamadi");
    dropLegoLink();
    return false;
  }

  legoCharacteristic = service->getCharacteristic(LEGO_CHARACTERISTIC_UUID);
  if (legoCharacteristic == nullptr) {
    Serial.println("LEGO karakteristigi bulunamadi");
    dropLegoLink();
    return false;
  }

  legoConnected = true;
  Serial.println("LEGO HUB HAZIR");

  oledMessage("LEGO BAGLI", "Kalibrasyon...");
  delay(300);
  legoCalibrateSteering();

  forceResend();
  return true;
}

// ============================================================
// ARM / DISARM
// ============================================================

void disarm(bool brake) {
  if (armed) Serial.println("DISARMED");
  armed = false;
  if (brake) brakeUntil = millis() + BRAKE_LIGHT_MS;
  forceResend();
}

// ============================================================
// KOMUT GÖNDERİMİ (değişince + periyodik keepalive)
// ============================================================

void sendDriveIfNeeded(int speed, int steering, uint8_t lights, uint32_t now) {
  if (!legoConnected) return;

  bool changed = speed != sentSpeed || steering != sentSteering || (int)lights != sentLights;
  if (!changed && (now - lastSendMs) < KEEPALIVE_MS) return;

  if (legoDrive(speed, steering, lights)) {
    sentSpeed    = speed;
    sentSteering = steering;
    sentLights   = lights;
  } else {
    forceResend();   // bir sonraki döngüde tekrar dene
  }
  lastSendMs = now;
}

// ============================================================
// OLED DURUM EKRANI
// ============================================================

void updateOled(bool xboxOk, uint32_t now) {
  if (now - lastOledMs < OLED_INTERVAL_MS) return;
  lastOledMs = now;

  const char* status;
  if (!legoConnected)       status = "LEGO YOK";
  else if (!xboxOk)         status = "XBOX BEKLE";
  else if (now < brakeUntil) status = "ACIL STOP";
  else if (armed)           status = "ARMED";
  else                      status = "START'a bas";

  char l2[16], l3[16];
  snprintf(l2, sizeof(l2), "SPD:%4d", curThrottle);
  snprintf(l3, sizeof(l3), "STR:%4d", curSteering);

  oledMessage(status, l2, l3, lightsOn ? "ISIK ACIK" : "ISIK KAPALI");
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  oled.begin();
  oled.setContrast(255);
  oledMessage("ESP32-C3", "Xbox + LEGO", "Basliyor...");

  Serial.println();
  Serial.println("=== ESP32 LEGO CONTROLLER ===");

  NimBLEDevice::init("ESP32-C3-LEGO");
  // Not: Xbox kütüphanesi begin() içinde kendi güvenlik ayarlarını
  // yapabilir; bu ayar global'dir.
  NimBLEDevice::setSecurityAuth(true, false, false);

  // 1) LEGO hub'ı bul (Xbox başlamadan önce, scanner çakışması olmasın)
  int attempt = 0;
  while (!hubAddressKnown) {
    attempt++;
    char buf[16];
    snprintf(buf, sizeof(buf), "Deneme %d", attempt);
    oledMessage("LEGO ARANIYOR", "Hub'i acin", buf);
    if (!scanForHub()) {
      Serial.println("Hub bulunamadi, tekrar deneniyor...");
      delay(500);
    }
  }

  // 2) Bağlan (başarısız olursa loop() yeniden dener)
  connectToHub();
  lastReconnectMs = millis();

  // 3) Xbox kumanda
  xboxController.begin();
  Serial.println("Xbox kumanda bekleniyor...");
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  uint32_t now = millis();

  xboxController.onLoop();

  // ---------- LEGO bağlantı bakımı ----------
  if (!legoConnected) {
    if (armed) disarm(false);
    if (now - lastReconnectMs >= RECONNECT_INTERVAL_MS) {
      lastReconnectMs = now;
      oledMessage("LEGO YOK", "Yeniden", "baglaniyor...");
      connectToHub();          // en fazla CONNECT_TIMEOUT_MS bloklar
      now = millis();
    }
  }

  // ---------- Xbox girişleri ----------
  bool xboxOk = xboxController.isConnected();
  int throttle = 0;
  int steering = 0;

  if (!xboxOk) {
    if (armed) {
      Serial.println("Xbox koptu -> stop");
      disarm(true);
    }
    btnA.reset(); btnB.reset(); btnStart.reset();
  } else {
    const auto& n = xboxController.xboxNotif;

    throttle = axisToPercent(n.joyLVert, INVERT_THROTTLE);
    steering = axisToPercent(n.joyLHori, INVERT_STEERING);

    bool bHeld = n.btnB;
    bool bEdge = btnB.rose(bHeld);
    bool startEdge = btnStart.rose(n.btnStart);
    bool aEdge = btnA.rose(n.btnA);

    // Acil stop: B basılı tutulduğu sürece arm edilemez
    if (bEdge) {
      Serial.println("ACIL STOP");
      disarm(true);
    }

    // Arm: B basılı değilken, hub bağlıyken ve gaz ortadayken
    if (startEdge && !bHeld && legoConnected && !armed) {
      if (throttle == 0) {
        armed = true;
        brakeUntil = 0;
        forceResend();
        Serial.println("ARMED");
      } else {
        Serial.println("Arm reddedildi: once gazi ortala");
      }
    }

    if (aEdge) {
      lightsOn = !lightsOn;
      Serial.printf("Isiklar: %s\n", lightsOn ? "ACIK" : "KAPALI");
    }
  }

  // ---------- Komut ----------
  if (!armed) {
    throttle = 0;
    steering = 0;
  }

  uint8_t lights = (now < brakeUntil) ? LIGHTS_BRAKE
                                      : (lightsOn ? LIGHTS_ON : LIGHTS_OFF);

  curThrottle = throttle;
  curSteering = steering;

  sendDriveIfNeeded(throttle, steering, lights, now);

  // ---------- Ekran ----------
  updateOled(xboxOk, now);

  delay(10);
}
