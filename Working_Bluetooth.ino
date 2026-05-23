// ═══════════════════════════════════════════════════════════════
//  ESP32 BLE HID Macro Pad — Full Functionality
//  BLE stack from Code 1, all 16 actions + 3 pots from Code 2
//  No external libraries — only ESP32 Arduino core built-ins
// ═══════════════════════════════════════════════════════════════

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
  0x09,0xE9,  // Bit 0:  Volume Up
  0x09,0xEA,  // Bit 1:  Volume Down
  0x09,0xE2,  // Bit 2:  Mute
  0x09,0xCD,  // Bit 3:  Play/Pause
  0x09,0xB5,  // Bit 4:  Scan Next Track
  0x09,0xB6,  // Bit 5:  Scan Previous Track
  0x09,0xB7,  // Bit 6:  Stop
  0x09,0xB3,  // Bit 7:  Fast Forward
  0x09,0xB4,  // Bit 8:  Rewind
  0x09,0x83,  // Bit 9:  (reserved / AL Consumer Control Config)
  0x09,0x30,  // Bit 10: Power
  0x09,0x40,  // Bit 11: Menu
  0x09,0x42,  // Bit 12: Menu Pick
  0x09,0x43,  // Bit 13: Menu Up
  0x09,0x44,  // Bit 14: Menu Down
  0x09,0x45,  // Bit 15: Menu Left
  0x81,0x02,  0xC0
};

// Consumer bit masks
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

// ══════════════════════════════════════════════════════════════
//  SEND PRIMITIVES
// ══════════════════════════════════════════════════════════════

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

// ── ASCII → HID keycode (high byte = modifier, low byte = keycode)
uint16_t charToHID(char c) {
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

void typeString(const char* str) {
  if (!deviceConnected) return;
  for (int i = 0; str[i]; i++) {
    uint16_t k       = charToHID(str[i]);
    uint8_t  mod     = (uint8_t)(k >> 8);
    uint8_t  keycode = (uint8_t)(k & 0xFF);
    if (keycode) {
      sendKey(mod, keycode);
      sendKeyRelease();
      delay(8);
    }
  }
}

// Press modifier(s) + key then release
// mod2 = 0 means no second modifier
void pressCombo(uint8_t mod1, uint8_t mod2, uint8_t keycode) {
  if (!deviceConnected) return;
  uint8_t report[8] = {(uint8_t)(mod1 | mod2), 0, keycode, 0, 0, 0, 0, 0};
  inputKeyboard->setValue(report, sizeof(report));
  inputKeyboard->notify();
  delay(100);
  sendKeyRelease();
  delay(50);
}

// Win+R → type command → Enter
void winRun(const char* cmd) {
  if (!deviceConnected) return;
  pressCombo(MOD_LGUI, 0, 0x15);  // Win+R  (keycode 0x15 = 'r')
  delay(400);
  typeString(cmd);
  delay(100);
  sendKey(0, 0x28);   // Enter
  sendKeyRelease();
  delay(300);
}

// ══════════════════════════════════════════════════════════════
//  16 KEY ACTIONS  (Code 2 functionality over BLE)
// ══════════════════════════════════════════════════════════════
//
//  Key  1  → Open Calculator       (Win+R → calc)
//  Key  2  → Open Word             (Win+R → winword)
//  Key  3  → Open PowerPoint       (Win+R → powerpnt)
//  Key  4  → Next Track            (Consumer: Scan Next)
//  Key  5  → Open Excel            (Win+R → excel)
//  Key  6  → Open Notepad          (Win+R → notepad)
//  Key  7  → Task Manager          (Ctrl+Shift+Esc)
//  Key  8  → Previous Track        (Consumer: Scan Prev)
//  Key  9  → File Explorer         (Win+E)
//  Key 10  → Screenshot (Snip)     (Win+Shift+S)
//  Key 11  → Lock PC               (Win+L)
//  Key 12  → Mute                  (Consumer: Mute)
//  Key 13  → Copy                  (Ctrl+C)
//  Key 14  → Play/Pause            (Consumer: Play/Pause)
//  Key 15  → Paste                 (Ctrl+V)
//  Key 16  → Show Desktop          (Win+D)

// HID keycodes for letters used in combos
#define KEY_E   0x08
#define KEY_S   0x16
#define KEY_L   0x0F
#define KEY_C   0x06
#define KEY_V   0x19
#define KEY_D   0x07

void runAction(int key) {
  switch (key) {
    case 1:  winRun("calc");                              break;
    case 2:  winRun("winword");                           break;
    case 3:  winRun("powerpnt");                          break;
    case 4:  sendConsumer(CON_NEXT);                      break;
    case 5:  winRun("excel");                             break;
    case 6:  winRun("notepad");                           break;
    case 7:  pressCombo(MOD_LCTRL, MOD_LSHIFT, 0x29);    break; // Ctrl+Shift+Esc
    case 8:  sendConsumer(CON_PREV);                      break;
    case 9:  pressCombo(MOD_LGUI, 0, KEY_E);              break; // Win+E
    case 10: pressCombo(MOD_LGUI, MOD_LSHIFT, KEY_S);    break; // Win+Shift+S
    case 11: pressCombo(MOD_LGUI, 0, KEY_L);              break; // Win+L
    case 12: sendConsumer(CON_MUTE);                      break;
    case 13: pressCombo(MOD_LCTRL, 0, KEY_C);             break; // Ctrl+C
    case 14: sendConsumer(CON_PLAY);                      break;
    case 15: pressCombo(MOD_LCTRL, 0, KEY_V);             break; // Ctrl+V
    case 16: pressCombo(MOD_LGUI, 0, KEY_D);              break; // Win+D
  }
}

// ── Rotary encoder ────────────────────────────────────────────
#define CLK 4
#define DT  5
#define SW  6
int  lastStateCLK;
bool lastButtonState = HIGH;

// ── Potentiometers ────────────────────────────────────────────
#define POT_VOLUME     7
#define POT_BRIGHTNESS 1
#define POT_ZOOM       2

int lastVolume     = -1;
int lastBrightness = -1;
int lastZoom       = -1;

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
<title>ESP32 Macro Pad</title>
<style>
  body{font-family:sans-serif;background:#111;color:#eee;
       max-width:480px;margin:30px auto;padding:16px}
  h2{color:#4af}
  .card{background:#1e1e1e;border-radius:10px;padding:14px;margin:12px 0}
  .ok{color:#4f4}.wait{color:#fa4}
  table{width:100%;border-collapse:collapse}
  td,th{padding:6px 8px;border:1px solid #333;font-size:13px;text-align:center}
  th{background:#222;color:#4af}
  .pot{display:flex;justify-content:space-between;margin:4px 0;font-size:13px}
</style></head><body>
<h2>&#9654; ESP32 BLE Macro Pad</h2>
<div class='card'>
  <b>WiFi IP:</b> )html" + WiFi.localIP().toString() + R"html(<br>
  <b>BLE:</b> <span class=')html" +
  (deviceConnected ? "ok'>&#9679; Connected" : "wait'>&#9675; Waiting for pair") +
  R"html(</span>
</div>
<div class='card'>
  <b>Controls</b>
  <div class='pot'>&#128421; Encoder scroll &rarr; Mouse wheel</div>
  <div class='pot'>&#128421; Encoder button &rarr; Device Manager</div>
  <div class='pot'>&#127897; Pot 1 (pin 7) &rarr; Volume</div>
  <div class='pot'>&#9728; Pot 2 (pin 1) &rarr; Brightness (consumer)</div>
  <div class='pot'>&#128269; Pot 3 (pin 2) &rarr; Zoom (Ctrl+scroll)</div>
</div>
<div class='card'>
<table>
<tr><th>Key</th><th>Action</th><th>Shortcut</th></tr>
<tr><td>1</td><td>Calculator</td><td>Win+R → calc</td></tr>
<tr><td>2</td><td>Word</td><td>Win+R → winword</td></tr>
<tr><td>3</td><td>PowerPoint</td><td>Win+R → powerpnt</td></tr>
<tr><td>4</td><td>Next Track</td><td>&#9654;&#9654;</td></tr>
<tr><td>5</td><td>Excel</td><td>Win+R → excel</td></tr>
<tr><td>6</td><td>Notepad</td><td>Win+R → notepad</td></tr>
<tr><td>7</td><td>Task Manager</td><td>Ctrl+Shift+Esc</td></tr>
<tr><td>8</td><td>Prev Track</td><td>&#9664;&#9664;</td></tr>
<tr><td>9</td><td>File Explorer</td><td>Win+E</td></tr>
<tr><td>10</td><td>Screenshot</td><td>Win+Shift+S</td></tr>
<tr><td>11</td><td>Lock PC</td><td>Win+L</td></tr>
<tr><td>12</td><td>Mute</td><td>&#128263;</td></tr>
<tr><td>13</td><td>Copy</td><td>Ctrl+C</td></tr>
<tr><td>14</td><td>Play/Pause</td><td>&#9654;&#9646;&#9646;</td></tr>
<tr><td>15</td><td>Paste</td><td>Ctrl+V</td></tr>
<tr><td>16</td><td>Show Desktop</td><td>Win+D</td></tr>
</table></div></body></html>)html";
  server.send(200, "text/html", page);
}

// ── BLE HID init ─────────────────────────────────────────────
void startBLE() {
  BLEDevice::init("ESP32 MacroPad");
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

  // ── Encoder → mouse scroll ───────────────────────────────
  int currentCLK = digitalRead(CLK);
  if (currentCLK != lastStateCLK) {
    if (deviceConnected) {
      sendMouse(0, 0, (digitalRead(DT) != currentCLK) ? 1 : -1);
    }
    delay(2);
  }
  lastStateCLK = currentCLK;

  // ── Pot 1: Volume ─────────────────────────────────────────
  int volume = map(analogRead(POT_VOLUME), 0, 4095, 0, 100);
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

  // ── Pot 2: Brightness ─────────────────────────────────────
  // NOTE: Standard HID Consumer page has brightness codes (0x6F / 0x70)
  // but most OSes only respond to them for specific hardware.
  // If your OS ignores them, remap this pot to another function.
  #define CON_BRIGHT_UP   (1 << 6)   // mapped to bit 6 (Stop key in descriptor)
  #define CON_BRIGHT_DOWN (1 << 7)   // mapped to bit 7 (FF key in descriptor)
  // To use OS brightness keys on Windows/Mac you may need Fn key combos instead;
  // adjust the bit mapping below to whichever consumer bits work for your OS.
  int brightness = map(analogRead(POT_BRIGHTNESS), 0, 4095, 0, 100);
  if (deviceConnected && abs(brightness - lastBrightness) > 2) {
    if (lastBrightness == -1) {
      lastBrightness = brightness;
    } else {
      int      steps = abs(brightness - lastBrightness);
      uint16_t bit   = (brightness > lastBrightness) ? CON_BRIGHT_UP : CON_BRIGHT_DOWN;
      for (int i = 0; i < steps; i++) {
        sendConsumer(bit);
        delay(2);
      }
      lastBrightness = brightness;
    }
  }

  // ── Pot 3: Zoom (Ctrl + mouse scroll) ────────────────────
  int zoom = map(analogRead(POT_ZOOM), 0, 4095, 0, 100);
  if (deviceConnected && abs(zoom - lastZoom) > 2) {
    if (lastZoom == -1) {
      lastZoom = zoom;
    } else {
      // Hold Ctrl, send scroll ticks, release Ctrl
      uint8_t ctrlDown[8] = {MOD_LCTRL, 0, 0, 0, 0, 0, 0, 0};
      inputKeyboard->setValue(ctrlDown, sizeof(ctrlDown));
      inputKeyboard->notify();
      delay(20);
      int8_t dir = (zoom > lastZoom) ? 1 : -1;
      sendMouse(0, 0, dir);
      delay(10);
      sendKeyRelease();
      lastZoom = zoom;
    }
  }

  // ── Encoder button → Device Manager ─────────────────────
  bool currentButtonState = digitalRead(SW);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    winRun("devmgmt.msc");
  }
  lastButtonState = currentButtonState;

  // ── Matrix keypad scan ────────────────────────────────────
  scanMatrix();

  // ── WiFi web server ───────────────────────────────────────
  if (WiFi.status() == WL_CONNECTED) server.handleClient();

  delay(5);
}
