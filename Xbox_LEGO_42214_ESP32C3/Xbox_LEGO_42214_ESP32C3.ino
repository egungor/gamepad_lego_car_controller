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
//   1) Açılışta, Xbox kütüphanesi başlamadan ÖNCE LEGO hub adresi belirlenir:
//      LEGO_HUB_MAC tanımlıysa o kullanılır, değilse hub taranır ve
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

#define DEBUG_BLE 0                      // 1: tüm BLE tarama sonuçlarını yazdır

static const int      DEADZONE              = 8;      // % cinsinden (merkez)
static const int      OUTER_DEADZONE        = 3;      // % cinsinden (uç; bunun ötesi 100 sayılır)
static const bool     INVERT_THROTTLE       = true;   // asukiaaa: yukarı = 0
static const bool     INVERT_STEERING       = false;

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

// Hub'ın MAC adresi. ESP32-C3 taraması bu hub'ın adresini 00:00:00:00:00:00
// olarak raporluyor (Windows'tan okunan gerçek adres aşağıda); bu yüzden
// adres biliniyorsa tarama atlanıp doğrudan bağlanılır. "" ise taranır.
static const char* LEGO_HUB_MAC             = "0c:4b:ee:f0:69:6b";

// Technic Move hub public 00:00:00:00:00:00 adresiyle yayın yapıyor.
// NimBLEClient::connect() bu adresi "null" sayıp reddettiği için LEGO
// bağlantısı doğrudan NimBLE host API'si (ble_gap_* / ble_gattc_*) ile kurulur.
ble_addr_t     hubAddress;
volatile bool  hubAddressKnown = false;
volatile bool  legoConnected   = false;   // BLE callback'inden de yazılır

volatile uint16_t legoConnHandle = BLE_HS_CONN_HANDLE_NONE;
volatile uint16_t legoValHandle  = 0;     // 0x1624 karakteristiğinin değer handle'ı

// Host görevindeki callback'lerle ana döngü arasında senkronizasyon
volatile bool legoGapDone  = false;
volatile int  legoGapRc    = 0;
volatile bool legoDiscDone = false;
volatile int  legoDiscRc   = 0;
volatile uint16_t legoCccdHandle = 0;      // bildirim descriptor'ı (0x2902)
volatile bool legoWriteDone = false;
volatile int  legoWriteRc   = 0;
volatile bool legoEncDone   = false;
volatile int  legoEncRc     = 0;

// ============================================================
// DURUM
// ============================================================

// Sürüş modu (Y ile değiştirilir):
//   STICK  : sol çubuk dikey = gaz, sol çubuk yatay = direksiyon
//   TRIGGER: RT = ileri gaz, LT = geri gaz, sol çubuk yatay = direksiyon
enum DriveMode { MODE_STICK, MODE_TRIGGER };

bool      armed           = false;
bool      lightsOn        = true;
DriveMode driveMode       = MODE_STICK;
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

EdgeButton btnA, btnB, btnY, btnStart;

// 0..100 büyüklüğe iç ve dış ölü bölge uygula, 0..100'e yeniden ölçekle
int applyDeadzones(int a) {
  if (a < DEADZONE) return 0;
  if (a >= 100 - OUTER_DEADZONE) return 100;
  return map(a, DEADZONE, 100 - OUTER_DEADZONE, 0, 100);
}

// 0..65535 (merkez 32768) -> -100..100
int axisToPercent(uint16_t raw, bool invert) {
  long centered = (long)raw - 32768L;
  // Pozitif taraf 32767'de biter: ayrı bölen, yoksa uçta 100 yerine 99 çıkar
  int p = (int)(centered * 100L / (centered >= 0 ? 32767L : 32768L));
  p = constrain(p, -100, 100);
  if (invert) p = -p;

  int scaled = applyDeadzones(abs(p));
  return p > 0 ? scaled : -scaled;
}

// 0..1023 tetik -> 0..100
int triggerToPercent(uint16_t raw) {
  int p = (int)((long)raw * 100L / XboxControllerNotificationParser::maxTrig);
  return applyDeadzones(constrain(p, 0, 100));
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

    hubAddress      = *dev->getAddress().getBase();
    hubAddressKnown = true;
    Serial.printf("LEGO hub bulundu: %s\n", dev->getAddress().toString().c_str());
    NimBLEDevice::getScan()->stop();
  }
};

LegoScanCallbacks legoScanCallbacks;

// LEGO bağlantısının GAP olayları (NimBLE host görevinde çalışır)
int legoGapEvent(ble_gap_event* event, void* arg) {
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      legoGapRc = event->connect.status;
      if (event->connect.status == 0) legoConnHandle = event->connect.conn_handle;
      legoGapDone = true;
      break;

    case BLE_GAP_EVENT_DISCONNECT:
      // Sadece bayrak: loop() yeniden bağlanır.
      legoConnected  = false;
      legoConnHandle = BLE_HS_CONN_HANDLE_NONE;
      legoGapRc      = event->disconnect.reason;
      legoGapDone    = true;
      Serial.printf("LEGO hub baglantisi koptu (reason=0x%X %s)\n",
                    event->disconnect.reason,
                    NimBLEUtils::returnCodeToString(event->disconnect.reason));
      break;

    case BLE_GAP_EVENT_ENC_CHANGE:
      legoEncRc   = event->enc_change.status;
      legoEncDone = true;
      break;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
      // Hub eski bağı unuttuysa: kayıtlı bağı sil ve yeniden eşleş
      ble_gap_conn_desc desc;
      if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
        ble_store_util_delete_peer(&desc.peer_id_addr);
      }
      return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
#if DEBUG_BLE
      // Hub'tan gelen bildirimler (hata mesajları vb. teşhis için)
      uint8_t  buf[32];
      uint16_t len = 0;
      ble_hs_mbuf_to_flat(event->notify_rx.om, buf, sizeof(buf), &len);
      Serial.print("LEGO <-");
      for (uint16_t i = 0; i < len; i++) Serial.printf(" %02X", buf[i]);
      Serial.println();
#endif
      break;
    }

    default:
      break;
  }
  return 0;
}

// Karakteristik keşfi (UUID ile, tüm handle aralığında)
// Değer handle'ından sonraki ilk CCCD'yi bul
int legoDscDiscovered(uint16_t connHandle, const ble_gatt_error* error,
                      uint16_t chrValHandle, const ble_gatt_dsc* dsc, void* arg) {
  if (error->status == 0 && dsc != nullptr) {
    if (legoCccdHandle == 0 && ble_uuid_u16(&dsc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16) {
      legoCccdHandle = dsc->handle;
    }
  } else {
    legoDiscRc   = (error->status == BLE_HS_EDONE) ? 0 : error->status;
    legoDiscDone = true;
  }
  return 0;
}

int legoWriteResult(uint16_t connHandle, const ble_gatt_error* error,
                    ble_gatt_attr* attr, void* arg) {
  legoWriteRc   = error->status;
  legoWriteDone = true;
  return 0;
}

int legoChrDiscovered(uint16_t connHandle, const ble_gatt_error* error,
                      const ble_gatt_chr* chr, void* arg) {
  if (error->status == 0 && chr != nullptr) {
    legoValHandle = chr->val_handle;
  } else {
    legoDiscRc   = (error->status == BLE_HS_EDONE) ? 0 : error->status;
    legoDiscDone = true;
  }
  return 0;
}

// Host görevinden gelecek bayrağı bekle
bool waitFlag(volatile bool& flag, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (!flag) {
    if (millis() - start > timeoutMs) return false;
    delay(5);
  }
  return true;
}

// ============================================================
// LEGO KOMUTLARI
// ============================================================

bool legoSend(const uint8_t* data, size_t length) {
  if (!legoConnected || legoValHandle == 0) return false;
  int rc = ble_gattc_write_no_rsp_flat(legoConnHandle, legoValHandle, data, length);
#if DEBUG_BLE
  Serial.printf("LEGO ->");
  for (size_t i = 0; i < length; i++) Serial.printf(" %02X", data[i]);
  Serial.printf("  rc=%d\n", rc);
#endif
  return rc == 0;
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
  // Bloklayan tarama: NimBLE host görevi sonuçları tamamen işleyene kadar
  // (ya da callback stop() çağırana kadar) bekler.
  scan->getResults(SCAN_DURATION_MS, false);   // NimBLE 2.x: milisaniye
  scan->clearResults();

  // Callback'i bırak; Xbox kütüphanesi kendi callback'ini kuracak.
  scan->setScanCallbacks(nullptr);

  return hubAddressKnown;
}

void dropLegoLink() {
  legoConnected = false;
  legoValHandle = 0;
  if (legoConnHandle != BLE_HS_CONN_HANDLE_NONE) {
    ble_gap_terminate(legoConnHandle, BLE_ERR_REM_USER_CONN_TERM);
  }
}

bool connectToHub() {
  if (!hubAddressKnown) return false;

  // Xbox kütüphanesi tarıyorsa bağlantı kurulamaz (BLE_HS_EBUSY)
  if (NimBLEDevice::getScan()->isScanning()) NimBLEDevice::getScan()->stop();

  char addrStr[18];
  const uint8_t* v = hubAddress.val;
  snprintf(addrStr, sizeof(addrStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           v[5], v[4], v[3], v[2], v[1], v[0]);
  Serial.printf("LEGO hub'a baglaniliyor: %s\n", addrStr);

  legoGapDone    = false;
  legoValHandle  = 0;
  legoConnHandle = BLE_HS_CONN_HANDLE_NONE;
  int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &hubAddress, CONNECT_TIMEOUT_MS,
                           nullptr, legoGapEvent, nullptr);
  if (rc != 0) {
    Serial.printf("Baglanti baslatilamadi (rc=%d %s)\n", rc, NimBLEUtils::returnCodeToString(rc));
    return false;
  }
  if (!waitFlag(legoGapDone, CONNECT_TIMEOUT_MS + 1000)) {
    ble_gap_conn_cancel();
    Serial.println("Baglanti zaman asimi");
    return false;
  }
  if (legoGapRc != 0 || legoConnHandle == BLE_HS_CONN_HANDLE_NONE) {
    Serial.printf("Baglanti basarisiz (rc=%d %s)\n", (int)legoGapRc,
                  NimBLEUtils::returnCodeToString(legoGapRc));
    return false;
  }

  // Technic Move hub şifreli bağlantı ister: şifresiz yazmaları sessizce
  // yok sayar, CCCD yazımını "Insufficient Encryption" ile reddeder.
  // Eşleşme (just works + bonding) başlat ve tamamlanmasını bekle.
  legoEncDone = false;
  rc = ble_gap_security_initiate(legoConnHandle);
  if (rc != 0 || !waitFlag(legoEncDone, 10000) || legoEncRc != 0) {
    Serial.printf("Sifreleme basarisiz (rc=%d/%d %s)\n", rc, (int)legoEncRc,
                  NimBLEUtils::returnCodeToString(legoEncRc));
    // Kayıtlı bağ bozuksa bir sonraki denemede sıfırdan eşleşilsin
    ble_store_util_delete_peer(&hubAddress);
    dropLegoLink();
    return false;
  }
  Serial.println("Baglanti sifrelendi");

  static NimBLEUUID chrUuid(LEGO_CHARACTERISTIC_UUID);
  legoDiscDone = false;
  rc = ble_gattc_disc_chrs_by_uuid(legoConnHandle, 1, 0xFFFF, chrUuid.getBase(),
                                   legoChrDiscovered, nullptr);
  if (rc != 0 || !waitFlag(legoDiscDone, 5000) || legoDiscRc != 0 || legoValHandle == 0) {
    Serial.printf("LEGO karakteristigi bulunamadi (rc=%d/%d)\n", rc, (int)legoDiscRc);
    dropLegoLink();
    return false;
  }

  Serial.printf("LWP deger handle=%u\n", (unsigned)legoValHandle);

  // LEGO uygulaması gibi bildirimlere abone ol: CCCD'yi keşfet ve 0x0001 yaz
  legoCccdHandle = 0;
  legoDiscDone   = false;
  rc = ble_gattc_disc_all_dscs(legoConnHandle, legoValHandle, 0xFFFF,
                               legoDscDiscovered, nullptr);
  if (rc != 0 || !waitFlag(legoDiscDone, 5000) || legoCccdHandle == 0) {
    Serial.printf("Uyari: CCCD bulunamadi (rc=%d/%d)\n", rc, (int)legoDiscRc);
  } else {
    const uint8_t notifyOn[] = { 0x01, 0x00 };
    legoWriteDone = false;
    rc = ble_gattc_write_flat(legoConnHandle, legoCccdHandle, notifyOn, sizeof(notifyOn),
                              legoWriteResult, nullptr);
    if (rc != 0 || !waitFlag(legoWriteDone, 3000) || legoWriteRc != 0) {
      Serial.printf("Uyari: bildirim aboneligi basarisiz (rc=%d/%d)\n", rc, (int)legoWriteRc);
    } else {
      Serial.printf("Bildirimler acik (CCCD handle=%u)\n", (unsigned)legoCccdHandle);
    }
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

  char l4[16];
  snprintf(l4, sizeof(l4), "%s I:%s",
           driveMode == MODE_STICK ? "CUBUK" : "TETIK",
           lightsOn ? "ACK" : "KPL");

  oledMessage(status, l2, l3, l4);
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
  if (LEGO_HUB_MAC[0] != '\0') {
    hubAddress      = *NimBLEAddress(std::string(LEGO_HUB_MAC), BLE_ADDR_PUBLIC).getBase();
    hubAddressKnown = true;
  }
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
    btnA.reset(); btnB.reset(); btnY.reset(); btnStart.reset();
  } else {
    const auto& n = xboxController.xboxNotif;

    bool bHeld = n.btnB;
    bool bEdge = btnB.rose(bHeld);
    bool startEdge = btnStart.rose(n.btnStart);
    bool aEdge = btnA.rose(n.btnA);
    bool yEdge = btnY.rose(n.btnY);

    // Mod değişimi: araç güvenlik için durdurulur, START ile yeniden arm edilir
    if (yEdge) {
      driveMode = (driveMode == MODE_STICK) ? MODE_TRIGGER : MODE_STICK;
      Serial.printf("Mod: %s\n", driveMode == MODE_STICK ? "CUBUK" : "TETIK");
      if (armed) disarm(false);
    }

    if (driveMode == MODE_STICK) {
      throttle = axisToPercent(n.joyLVert, INVERT_THROTTLE);
    } else {
      throttle = triggerToPercent(n.trigRT) - triggerToPercent(n.trigLT);
    }
    steering = axisToPercent(n.joyLHori, INVERT_STEERING);

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
