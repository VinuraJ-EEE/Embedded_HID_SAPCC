#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>
#include <WiFi.h>
#include <WebServer.h>

// ── WiFi ──────────────────────────────────────────────────────
const char* ssid     = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
WebServer server(80);

// ── BLE HID objects ───────────────────────────────────────────
BLEHIDDevice*      hid;
BLECharacteristic* inputKeyboard;
BLECharacteristic* inputMouse;
BLECharacteristic* inputConsumer;
bool               deviceConnected = false;

// Report IDs
#define REPORT_ID_KEYBOARD  1
#define REPORT_ID_MOUSE     2
#define REPORT_ID_CONSUMER  3

// Modifier key bit masks
#define MOD_LCTRL   0x01
#define MOD_LSHIFT  0x02
#define MOD_LALT    0x04
#define MOD_LGUI    0x08

// ── HID Report Descriptor ─────────────────────────────────────
static const uint8_t hidReportDescriptor[] = {

  // ── Keyboard (Report ID 1) ──
  0x05,0x01,  0x09,0x06,  0xA1,0x01,
  0x85,REPORT_ID_KEYBOARD,
  0x05,0x07,  0x19,0xE0,  0x29,0xE7,
  0x15,0x00,  0x25,0x01,
  0x75,0x01,  0x95,0x08,  0x81,0x02,
  0x95,0x01,  0x75,0x08,  0x81,0x01,
  0x95,0x06,  0x75,0x08,
  0x15,0x00,  0x25,0x73,
  0x05,0x07,  0x19,0x00,  0x29,0x73,
  0x81,0x00,  0xC0,

  // ── Mouse (Report ID 2) ──
  0x05,0x01,  0x09,0x02,  0xA1,0x01,
  0x85,REPORT_ID_MOUSE,
  0x09,0x01,  0xA1,0x00,
  0x05,0x09,  0x19,0x01,  0x29,0x03,
  0x15,0x00,  0x25,0x01,
  0x95,0x03,  0x75,0x01,  0x81,0x02,
  0x95,0x01,  0x75,0x05,  0x81,0x01,
  0x05,0x01,  0x09,0x30,  0x09,0x31,  0x09,0x38,
  0x15,0x81,  0x25,0x7F,
  0x75,0x08,  0x95,0x03,  0x81,0x06,
  0xC0, 0xC0,

  // ── Consumer Control (Report ID 3) ──
  0x05,0x0C,  0x09,0x01,  0xA1,0x01,
  0x85,REPORT_ID_CONSUMER,
  0x15,0x00,  0x25,0x01,
  0x75,0x01,  0x95,0x10,
  0x09,0xE9,  // Volume Up
  0x09,0xEA,  // Volume Down
  0x09,0xE2,  // Mute
  0x09,0xCD,  // Play/Pause
  0x09,0xB5,  // Scan Next
  0x09,0xB6,  // Scan Previous
  0x09,0xB7,  // Stop
  0x09,0xB3,  // Fast Forward
  0x09,0xB4,  // Rewind
  0x09,0x83,
  0x09,0x30,
  0x09,0x40,
  0x09,0x42,
  0x09,0x43,
  0x09,0x44,
  0x09,0x45,
  0x81,0x02,  0xC0
};

// Consumer bit positions
#define CON_VOL_UP    (1 << 0)
#define CON_VOL_DOWN  (1 << 1)
#define CON_MUTE      (1 << 2)
#define CON_PLAY      (1 << 3)
#define CON_NEXT      (1 << 4)
#define CON_PREV      (1 << 5)

// ── BLE server callbacks ──────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    deviceConnected = true;
    Serial.println("BLE device connected");
  }
  void onDisconnect(BLEServer* s) override {
    deviceConnected = false;
    Serial.println("BLE disconnected — advertising again...");
    s->startAdvertising();
  }
};

// ── Send functions ────────────────────────────────────────────
void sendKey(uint8_t modifier, uint8_t keycode) {
  if (!deviceConnected) return;
  uint8_t report[8] = {modifier, 0, keycode, 0, 0, 0, 0, 0};
  inputKeyboard->setValue(report, sizeof(report));
  inputKeyboard->notify();
  delay(10);
}

void sendKeyRelease() {
  if (!deviceConnected) return;
  uint8_t report[8] = {0};
  inputKeyboard->setValue(report, sizeof(report));
  inputKeyboard->notify();
  delay(10);
}

void sendMouse(int8_t x, int8_t y, int8_t wheel, uint8_t buttons = 0) {
  if (!deviceConnected) return;
  uint8_t report[4] = {buttons, (uint8_t)x, (uint8_t)y, (uint8_t)wheel};
  inputMouse->setValue(report, sizeof(report));
  inputMouse->notify();
}

void sendConsumer(uint16_t bits) {
  if (!deviceConnected) return;
  uint8_t report[2]  = {(uint8_t)(bits & 0xFF), (uint8_t)(bits >> 8)};
  inputConsumer->setValue(report, sizeof(report));
  inputConsumer->notify();
  delay(15);
  uint8_t release[2] = {0, 0};
  inputConsumer->setValue(release, sizeof(release));
  inputConsumer->notify();
  delay(10);
}

// ── ASCII → HID keycode ───────────────────────────────────────
// Returns uint16_t with high byte = modifier, low byte = keycode
// No struct used — avoids all HIDTypes.h naming conflicts
uint16_t charToHID(char c) {
  // helper macro: pack modifier and keycode into one uint16_t
  #define PACK(mod, key) (((uint16_t)(mod) << 8) | (uint8_t)(key))

  if (c >= 'a' && c <= 'z') return PACK(0,          c - 'a' + 0x04);
  if (c >= 'A' && c <= 'Z') return PACK(MOD_LSHIFT, c - 'A' + 0x04);
  if (c >= '1' && c <= '9') return PACK(0,           c - '1' + 0x1E);

  switch (c) {
    case '0':  return PACK(0,          0x27);
    case ' ':  return PACK(0,          0x2C);
    case '\n': return PACK(0,          0x28);
    case '\\': return PACK(0,          0x31);
    case '{':  return PACK(MOD_LSHIFT, 0x2F);
    case '}':  return PACK(MOD_LSHIFT, 0x30);
    case '[':  return PACK(0,          0x2F);
    case ']':  return PACK(0,          0x30);
    case '_':  return PACK(MOD_LSHIFT, 0x2D);
    case '^':  return PACK(MOD_LSHIFT, 0x23);
    case '(':  return PACK(MOD_LSHIFT, 0x26);
    case ')':  return PACK(MOD_LSHIFT, 0x27);
    case '&':  return PACK(MOD_LSHIFT, 0x24);
    case '@':  return PACK(MOD_LSHIFT, 0x1F);
    case '=':  return PACK(0,          0x2E);
    case '+':  return PACK(MOD_LSHIFT, 0x2E);
    case '-':  return PACK(0,          0x2D);
    case '/':  return PACK(0,          0x38);
    case '*':  return PACK(MOD_LSHIFT, 0x25);
    case '.':  return PACK(0,          0x37);
    case ',':  return PACK(0,          0x36);
    case '>':  return PACK(MOD_LSHIFT, 0x37);
    case '<':  return PACK(MOD_LSHIFT, 0x36);
    default:   return PACK(0,          0);
  }
  #undef PACK
}

// Type a full string character by character
void typeString(const char* str) {
  if (!deviceConnected) return;
  for (int i = 0; str[i]; i++) {
    uint16_t k       = charToHID(str[i]);
    uint8_t  mod     = (uint8_t)(k >> 8);    // high byte = modifier
    uint8_t  keycode = (uint8_t)(k & 0xFF);  // low byte  = keycode
    if (keycode) {
      sendKey(mod, keycode);
      sendKeyRelease();
      delay(8);
    }
  }
}

// Press modifier + key combo then release
void pressCombo(uint8_t modifier, uint8_t keycode) {
  sendKey(modifier, keycode);
  delay(80);
  sendKeyRelease();
  delay(50);
}

// ── Equation editor helper ────────────────────────────────────
void insertEq(const char* cmd) {
  if (!deviceConnected) return;
  pressCombo(MOD_LALT, 0x2E);  // Alt+= opens equation field
  delay(400);
  typeString(cmd);
  delay(80);
  sendKey(0, 0x2C);             // Space renders the LaTeX
  sendKeyRelease();
  delay(100);
}

// ── Key actions 1–16 ─────────────────────────────────────────
void runAction(int key) {
  switch (key) {
    case  1: insertEq("\\int");                           break; // ∫
    case  2: insertEq("\\int_a^b");                      break; // ∫ₐᵇ
    case  3: insertEq("\\iint");                          break; // ∬
    case  4: insertEq("\\iiint");                         break; // ∭
    case  5: insertEq("\\frac");                          break; // □/□
    case  6: insertEq("\\sqrt");                          break; // √□
    case  7: insertEq("\\sqrt(n&x)");                    break; // ⁿ√□
    case  8: insertEq("\\frac{\\partial}{\\partial x}"); break; // ∂/∂x
    case  9: insertEq("\\frac{d}{dx}");                  break; // d/dx
    case 10: insertEq("\\sum_{n=1}^{\\infty}");          break; // Σ
    case 11: insertEq("\\prod_{n=1}^{N}");               break; // Π
    case 12: insertEq("\\lim_{x\\to\\infty}");           break; // lim
    case 13: insertEq("\\matrix(a&b@c&d)");              break; // [ ]
    case 14: insertEq("\\infty");                         break; // ∞
    case 15: insertEq("\\pi");                            break; // π
    case 16: insertEq("\\vec");                           break; // □⃗
  }
}

// ── Rotary encoder + pot ──────────────────────────────────────
#define CLK     4
#define DT      5
#define SW      6
#define POT_PIN 7

int  lastStateCLK;
int  lastVolume      = -1;
bool lastButtonState = HIGH;

// ── 4×4 Matrix ───────────────────────────────────────────────
const int ROWS = 4;
const int COLS = 4;

int rowPins[ROWS] = {38, 39, 40, 41};
int colPins[COLS] = {42,  8, 17, 18};

int keys[ROWS][COLS] = {
  { 1,  2,  3,  4},
  { 5,  6,  7,  8},
  { 9, 10, 11, 12},
  {13, 14, 15, 16}
};

bool          prevState[ROWS][COLS]    = {};
bool          rawState[ROWS][COLS]     = {};
unsigned long lastDebounce[ROWS][COLS] = {};
const unsigned long DEBOUNCE_MS        = 15;

// ── Matrix scan ───────────────────────────────────────────────
void scanMatrix() {
  unsigned long now = millis();
  for (int r = 0; r < ROWS; r++) {
    digitalWrite(rowPins[r], LOW);
    delayMicroseconds(10);
    for (int c = 0; c < COLS; c++) {
      bool pressed = (digitalRead(colPins[c]) == LOW);
      if (pressed != rawState[r][c]) {
        lastDebounce[r][c] = now;
        rawState[r][c]     = pressed;
      }
      if ((now - lastDebounce[r][c]) >= DEBOUNCE_MS) {
        if (pressed && !prevState[r][c]) runAction(keys[r][c]);
        prevState[r][c] = pressed;
      }
    }
    digitalWrite(rowPins[r], HIGH);
  }
}

// ── Web dashboard ─────────────────────────────────────────────
void handleRoot() {
  String page = R"html(<!DOCTYPE html><html>
<head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>MathPad</title>
<style>
  body{font-family:sans-serif;background:#111;color:#eee;
       max-width:460px;margin:30px auto;padding:16px}
  h2{color:#4af}
  .card{background:#1e1e1e;border-radius:10px;padding:14px;margin:12px 0}
  .ok{color:#4f4}.wait{color:#fa4}
  table{width:100%;border-collapse:collapse}
  td,th{padding:6px 8px;border:1px solid #333;font-size:13px;text-align:center}
  th{background:#222;color:#4af}
</style></head><body>
<h2>ESP32 MathPad</h2>
<div class='card'>
  <b>WiFi IP:</b> )html" + WiFi.localIP().toString() + R"html(<br>
  <b>BLE:</b> <span class=')html" +
  (deviceConnected ? "ok'>Connected" : "wait'>Waiting for pair") +
  R"html(</span>
</div>
<div class='card'>
<table>
<tr><th>Key</th><th>Function</th><th>Symbol</th></tr>
<tr><td>1</td><td>Indefinite integral</td><td>∫</td></tr>
<tr><td>2</td><td>Definite integral</td><td>∫ₐᵇ</td></tr>
<tr><td>3</td><td>Double integral</td><td>∬</td></tr>
<tr><td>4</td><td>Triple integral</td><td>∭</td></tr>
<tr><td>5</td><td>Fraction</td><td>□/□</td></tr>
<tr><td>6</td><td>Square root</td><td>√□</td></tr>
<tr><td>7</td><td>nth root</td><td>ⁿ√□</td></tr>
<tr><td>8</td><td>Partial derivative</td><td>∂/∂x</td></tr>
<tr><td>9</td><td>Full derivative</td><td>d/dx</td></tr>
<tr><td>10</td><td>Summation</td><td>Σ</td></tr>
<tr><td>11</td><td>Product</td><td>Π</td></tr>
<tr><td>12</td><td>Limit</td><td>lim</td></tr>
<tr><td>13</td><td>2×2 Matrix</td><td>[ ]</td></tr>
<tr><td>14</td><td>Infinity</td><td>∞</td></tr>
<tr><td>15</td><td>Pi</td><td>π</td></tr>
<tr><td>16</td><td>Vector</td><td>□⃗</td></tr>
</table></div></body></html>)html";
  server.send(200, "text/html", page);
}

// ── BLE HID init ─────────────────────────────────────────────
void startBLE() {
  BLEDevice::init("ESP32 MathPad");
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  hid           = new BLEHIDDevice(pServer);
  inputKeyboard = hid->inputReport(REPORT_ID_KEYBOARD);
  inputMouse    = hid->inputReport(REPORT_ID_MOUSE);
  inputConsumer = hid->inputReport(REPORT_ID_CONSUMER);

  hid->manufacturer()->setValue("ESP32Build");
  hid->pnp(0x02, 0x05AC, 0x820A, 0x0210);
  hid->hidInfo(0x00, 0x01);
  hid->reportMap((uint8_t*)hidReportDescriptor, sizeof(hidReportDescriptor));
  hid->startServices();

  BLEAdvertising* adv = pServer->getAdvertising();
  adv->setAppearance(HID_KEYBOARD);
  adv->addServiceUUID(hid->hidService()->getUUID());
  adv->start();
  Serial.println("BLE HID advertising — pair from your PC/phone");
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(CLK, INPUT);
  pinMode(DT,  INPUT);
  pinMode(SW,  INPUT_PULLUP);
  lastStateCLK = digitalRead(CLK);
  analogReadResolution(12);

  for (int r = 0; r < ROWS; r++) {
    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], HIGH);
  }
  for (int c = 0; c < COLS; c++) {
    pinMode(colPins[c], INPUT_PULLUP);
  }

  startBLE();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500); Serial.print("."); tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi: http://" + WiFi.localIP().toString());
    server.on("/", handleRoot);
    server.begin();
  } else {
    Serial.println("\nWiFi failed — BLE still works independently");
  }
}

// ── Main loop ─────────────────────────────────────────────────
void loop() {

  // ── Encoder scroll ───────────────────────────────────────
  int currentCLK = digitalRead(CLK);
  if (currentCLK != lastStateCLK) {
    if (deviceConnected) {
      sendMouse(0, 0, (digitalRead(DT) != currentCLK) ? 1 : -1);
    }
    delay(2);
  }
  lastStateCLK = currentCLK;

  // ── Potentiometer volume ─────────────────────────────────
  int raw    = analogRead(POT_PIN);
  int volume = map(raw, 0, 4095, 0, 100);

  if (deviceConnected && abs(volume - lastVolume) > 2) {
    if (lastVolume == -1) {
      lastVolume = volume;
    } else {
      int      steps = abs(volume - lastVolume);
      uint16_t bit   = (volume > lastVolume) ? CON_VOL_UP : CON_VOL_DOWN;
      for (int i = 0; i < steps; i++) {
        sendConsumer(bit);
        delay(2);
      }
      lastVolume = volume;
    }
  }

  // ── Encoder button → Device Manager ─────────────────────
  bool currentButtonState = digitalRead(SW);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    if (deviceConnected) {
      pressCombo(MOD_LGUI, 0x15);  // Win+R
      delay(400);
      typeString("devmgmt.msc");
      delay(100);
      sendKey(0, 0x28);            // Enter
      sendKeyRelease();
    }
  }
  lastButtonState = currentButtonState;

  // ── Matrix keypad ─────────────────────────────────────────
  scanMatrix();

  // ── WiFi web server ───────────────────────────────────────
  if (WiFi.status() == WL_CONNECTED) server.handleClient();

  delay(5);
}
