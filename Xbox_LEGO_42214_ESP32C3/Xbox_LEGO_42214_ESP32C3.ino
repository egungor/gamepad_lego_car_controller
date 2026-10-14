#include <Arduino.h>
#include <string>
#include <U8g2lib.h>
#include <NimBLEDevice.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// ============================================================
// OLED
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

XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

// ============================================================
// LEGO MOVE HUB 88019
// ============================================================

static const char* LEGO_SERVICE_UUID =
  "00001623-1212-EFDE-1623-785FEABCD123";

static const char* LEGO_CHARACTERISTIC_UUID =
  "00001624-1212-EFDE-1623-785FEABCD123";

NimBLEClient* legoClient = nullptr;
NimBLERemoteCharacteristic* legoCharacteristic = nullptr;

bool legoConnected = false;

// ============================================================
// STATE
// ============================================================

bool armed = false;
bool lightsOn = true;

int lastSpeed = 0;
int lastSteering = 0;
uint8_t lastLights = 0;

// ============================================================
// OLED
// ============================================================

void oledMessage(const char* line1,
                 const char* line2 = "",
                 const char* line3 = "") {
  oled.clearBuffer();

  oled.setFont(u8g2_font_6x10_tf);

  oled.drawStr(0, 10, line1);
  oled.drawStr(0, 22, line2);
  oled.drawStr(0, 34, line3);

  oled.sendBuffer();
}

// ============================================================
// LEGO SEND
// ============================================================

bool legoSend(const uint8_t* data, size_t length) {
  if (!legoConnected || legoCharacteristic == nullptr) {
    return false;
  }

  return legoCharacteristic->writeValue(
    data,
    length,
    false);
}

// ============================================================
// LEGO DRIVE
//
// LEGO 88019 combined drive frame:
//
// 0D 00 81 36 11 51 00 03 00
// speed
// steering
// lights
// 00
// ============================================================

void legoDrive(
  int speed,
  int steering,
  uint8_t lights) {
  speed = constrain(speed, -100, 100);
  steering = constrain(steering, -100, 100);

  uint8_t packet[13];

  packet[0] = 0x0D;
  packet[1] = 0x00;
  packet[2] = 0x81;
  packet[3] = 0x36;
  packet[4] = 0x11;
  packet[5] = 0x51;
  packet[6] = 0x00;
  packet[7] = 0x03;
  packet[8] = 0x00;

  packet[9] = (uint8_t)(int8_t)speed;
  packet[10] = (uint8_t)(int8_t)steering;
  packet[11] = lights;
  packet[12] = 0x00;

  legoSend(packet, sizeof(packet));
}

// ============================================================
// STEERING CALIBRATION
// ============================================================

void legoCalibrateSteering() {
  uint8_t packet1[] = {
    0x0D, 0x00, 0x81, 0x36,
    0x11, 0x51, 0x00, 0x03,
    0x00, 0x00, 0x00, 0x10,
    0x00
  };

  uint8_t packet2[] = {
    0x0D, 0x00, 0x81, 0x36,
    0x11, 0x51, 0x00, 0x03,
    0x00, 0x00, 0x00, 0x08,
    0x00
  };

  legoSend(packet1, sizeof(packet1));

  delay(100);

  legoSend(packet2, sizeof(packet2));

  delay(100);
}

// ============================================================
// LEGO SCAN
// ============================================================

// Adresi tarama callback'i içinde yakalıyoruz. Böylece getResults()
// listesinden gelen cihaz nesnesine bağımlı kalmıyoruz.
NimBLEAddress foundHubAddress;
bool hubAddressFound = false;

class LegoAdvertisedCallbacks : public NimBLEScanCallbacks {
public:
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    if (advertisedDevice == nullptr) {
      return;
    }

    String name = advertisedDevice->haveName()
                    ? String(advertisedDevice->getName().c_str())
                    : String("");
    bool nameMatches = name.indexOf("Technic Move") >= 0;
    bool serviceMatches = advertisedDevice->isAdvertisingService(
      NimBLEUUID(LEGO_SERVICE_UUID));

    Serial.print("BLE device: ");
    Serial.print(name.length() ? name : String("<no name>"));
    Serial.print(" | Address: ");
    Serial.print(advertisedDevice->getAddress().toString().c_str());
    Serial.print(" | LEGO service 0x1623: ");
    Serial.println(serviceMatches ? "yes" : "no");

    // Hub is identified by its advertised name OR LEGO Powered Up service.
    if (hubAddressFound || (!nameMatches && !serviceMatches)) {
      return;
    }

    Serial.println("LEGO HUB FOUND!");
    foundHubAddress = advertisedDevice->getAddress();
    hubAddressFound = true;

    Serial.print("Address captured in callback: ");
    Serial.println(foundHubAddress.toString().c_str());

    NimBLEDevice::getScan()->stop();
  }
};

LegoAdvertisedCallbacks legoScanCallbacks;

// ============================================================
// TRY A BLE ADDRESS AND VERIFY IT IS THE LEGO HUB
// ============================================================

bool tryConnectAndVerifyLego(const NimBLEAddress& address) {
  Serial.print("Trying BLE address: ");
  Serial.println(address.toString().c_str());

  legoClient = NimBLEDevice::createClient();
  if (legoClient == nullptr) {
    Serial.println("Could not create BLE client");
    return false;
  }

  if (!legoClient->connect(address)) {
    Serial.println("Connection attempt failed");
    NimBLEDevice::deleteClient(legoClient);
    legoClient = nullptr;
    return false;
  }

  Serial.println("Connected; checking LEGO service 0x1623...");
  NimBLERemoteService* testService = legoClient->getService(LEGO_SERVICE_UUID);
  if (testService == nullptr) {
    Serial.println("Connected device does not expose LEGO service; trying next address");
    legoClient->disconnect();
    NimBLEDevice::deleteClient(legoClient);
    legoClient = nullptr;
    return false;
  }

  Serial.println("LEGO service confirmed at this address");
  return true;
}

// ============================================================
// LEGO CONNECT
// ============================================================

bool connectToLego() {
  cleanupLegoClient();

  // Önceki denemeden kalmış tarama durumu/adresi sıfırla.
  hubAddressFound = false;
  foundHubAddress = NimBLEAddress();

  Serial.println();
  Serial.println("Scanning for LEGO hub...");

  NimBLEScan* scan = NimBLEDevice::getScan();
  if (scan == nullptr) {
    Serial.println("ERROR: BLE scanner is null");
    oledMessage("LEGO", "SCAN ERROR");
    return false;
  }

  scan->setActiveScan(true);
  scan->setInterval(45);
  scan->setWindow(15);
  scan->setScanCallbacks(&legoScanCallbacks, false);
  scan->clearResults();

  Serial.println("Starting callback-based BLE scan...");
  if (!scan->start(10, false)) {
    Serial.println("ERROR: Could not start BLE scan");
    oledMessage("LEGO", "SCAN START FAIL");
    return false;
  }

  // onResult() stops scanning as soon as the target hub is found.
  while (scan->isScanning()) {
    delay(20);
  }

  scan->clearResults();
  Serial.println("BLE scan finished.");

  if (!hubAddressFound) {
    Serial.println("LEGO hub not seen in this scan; will still try stored bonds.");
  }

  bool connectedToLego = false;

  std::string addressText = foundHubAddress.toString();
  Serial.print("Advertised LEGO address: ");
  Serial.println(addressText.c_str());

  // Normal path: connect directly when a scan result supplied a valid address.
  if (hubAddressFound && addressText != "00:00:00:00:00:00") {
    connectedToLego = tryConnectAndVerifyLego(foundHubAddress);
  } else {
    Serial.println("Advertised address is zero; trying stored BLE bonds instead.");
  }

  // Some Technic Move hubs advertise a zero address. In that case, try each
  // stored bond and identify the LEGO hub by its Powered Up service UUID.
  if (!connectedToLego) {
    int bondCount = NimBLEDevice::getNumBonds();
    Serial.print("Stored bonds available for fallback: ");
    Serial.println(bondCount);

    for (int i = 0; i < bondCount; ++i) {
      NimBLEAddress bondedAddress = NimBLEDevice::getBondedAddress(i);
      Serial.print("Testing bond ");
      Serial.print(i);
      Serial.print(" at ");
      Serial.println(bondedAddress.toString().c_str());

      if (bondedAddress.toString() == "00:00:00:00:00:00") {
        Serial.println("Skipping zero bond address");
        continue;
      }

      if (tryConnectAndVerifyLego(bondedAddress)) {
        connectedToLego = true;
        break;
      }
    }
  }

  if (!connectedToLego) {
    Serial.println("Could not connect to LEGO hub using scan or stored bonds.");
    oledMessage("LEGO", "CONNECT FAIL");
    cleanupLegoClient();
    return false;
  }

  Serial.println("LEGO connected and service verified");

  NimBLEDevice::setSecurityAuth(
    true,   // Bonding
    false,  // MITM
    false   // Secure Connections
  );

  Serial.println("Establishing security...");
  if (!legoClient->secureConnection()) {
    Serial.println("LEGO secure connection failed");
    cleanupLegoClient();
    return false;
  }

  Serial.println("LEGO security established");
  Serial.println("Discovering LEGO service...");

  NimBLERemoteService* service =
    legoClient->getService(LEGO_SERVICE_UUID);

  if (service == nullptr) {
    Serial.println("LEGO service not found");
    cleanupLegoClient();
    return false;
  }

  Serial.println("Discovering LEGO characteristic...");
  legoCharacteristic =
    service->getCharacteristic(LEGO_CHARACTERISTIC_UUID);

  if (legoCharacteristic == nullptr) {
    Serial.println("LEGO characteristic not found");
    cleanupLegoClient();
    return false;
  }

  legoConnected = true;
  Serial.println("LEGO HUB READY!");

  oledMessage("LEGO CONNECTED", "Calibrating...");
  delay(300);
  legoCalibrateSteering();

  oledMessage("LEGO READY", "Press START");
  return true;
}

void cleanupLegoClient()
{
    legoConnected = false;
    legoCharacteristic = nullptr;

    if (legoClient != nullptr)
    {
        if (legoClient->isConnected())
        {
            legoClient->disconnect();
        }

        NimBLEDevice::deleteClient(legoClient);
        legoClient = nullptr;
    }

}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  delay(1000);

  oled.begin();
  oled.setContrast(255);

  oledMessage(
    "ESP32-C3",
    "Xbox + LEGO",
    "Starting...");

  Serial.println();
  Serial.println("======================");
  Serial.println("ESP32 LEGO CONTROLLER");
  Serial.println("======================");

  // --------------------------------------------------------
  // BLE
  // --------------------------------------------------------

  NimBLEDevice::init(
    "ESP32-C3-LEGO");

  Serial.println("=== BLE BOND DIAGNOSTICS ===");
  int startupBondCount = NimBLEDevice::getNumBonds();
  Serial.print("Stored bonds: ");
  Serial.println(startupBondCount);
  for (int i = 0; i < startupBondCount; ++i) {
    NimBLEAddress bondAddress = NimBLEDevice::getBondedAddress(i);
    Serial.print("Bond ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(bondAddress.toString().c_str());
  }
  Serial.println("============================");

  // --------------------------------------------------------
  // Xbox
  // --------------------------------------------------------

  xboxController.begin();

  Serial.println(
    "Xbox controller waiting...");

  oledMessage(
    "WAITING",
    "XBOX CONTROLLER");
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  // --------------------------------------------------------
  // Xbox update
  // --------------------------------------------------------

  xboxController.onLoop();

  // --------------------------------------------------------
  // Xbox connection
  // --------------------------------------------------------

  if (!xboxController.isConnected()) {
    armed = false;

    if (legoConnected) {
      legoDrive(
        0,
        0,
        0x01);
    }

    oledMessage(
      "WAITING",
      "XBOX...");

    delay(50);

    return;
  }

  // --------------------------------------------------------
  // LEGO connection
  // --------------------------------------------------------

  if (!legoConnected) {
    connectToLego();

    delay(100);

    return;
  }

  // --------------------------------------------------------
  // Emergency stop - B
  // --------------------------------------------------------

  if (xboxController.xboxNotif.btnB) {
    armed = false;

    legoDrive(
      0,
      0,
      0x01);

    oledMessage(
      "EMERGENCY STOP",
      "Press START");

    delay(150);

    return;
  }

  // --------------------------------------------------------
  // ARM
  // --------------------------------------------------------

  if (xboxController.xboxNotif.btnStart) {
    if (!armed) {
      armed = true;

      Serial.println(
        "SYSTEM ARMED");

      oledMessage(
        "READY",
        "ARMED");

      delay(250);
    }
  }

  // --------------------------------------------------------
  // LIGHTS
  // --------------------------------------------------------

  if (xboxController.xboxNotif.btnA) {
    lightsOn = !lightsOn;

    Serial.print(
      "Lights: ");

    Serial.println(
      lightsOn ? "ON" : "OFF");

    delay(250);
  }

  // --------------------------------------------------------
  // NOT ARMED
  // --------------------------------------------------------

  if (!armed) {
    legoDrive(
      0,
      0,
      lightsOn ? 0x00 : 0x04);

    delay(20);

    return;
  }

  // --------------------------------------------------------
  // READ JOYSTICK
  // --------------------------------------------------------

  int steering =
    xboxController.xboxNotif.joyLHori;

  int throttle =
    xboxController.xboxNotif.joyLVert;

  // Depending on the library/controller
  // Y may have opposite orientation.
  //
  // Normalize to -100..100.

  throttle =
    constrain(
      throttle,
      -100,
      100);

  steering =
    constrain(
      steering,
      -100,
      100);

  // --------------------------------------------------------
  // DEADZONE
  // --------------------------------------------------------

  if (abs(throttle) < 8)
    throttle = 0;

  if (abs(steering) < 8)
    steering = 0;

  // --------------------------------------------------------
  // SEND ONLY WHEN CHANGED
  // --------------------------------------------------------

  if (throttle != lastSpeed || steering != lastSteering || lightsOn != (lastLights == 0x00)) {
    uint8_t lights =
      lightsOn ? 0x00 : 0x04;

    legoDrive(
      throttle,
      steering,
      lights);

    lastSpeed =
      throttle;

    lastSteering =
      steering;

    lastLights =
      lights;
  }

  // --------------------------------------------------------
  // OLED
  // --------------------------------------------------------

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tf);

  oled.drawStr(
    0,
    9,
    "LEGO READY");

  char buffer[20];

  snprintf(
    buffer,
    sizeof(buffer),
    "SPD:%4d",
    throttle);

  oled.drawStr(
    0,
    20,
    buffer);

  snprintf(
    buffer,
    sizeof(buffer),
    "STR:%4d",
    steering);

  oled.drawStr(
    0,
    31,
    buffer);

  oled.drawStr(
    0,
    40,
    lightsOn ? "LIGHT ON" : "LIGHT OFF");

  oled.sendBuffer();

  delay(20);
}