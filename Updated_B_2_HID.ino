// ═══════════════════════════════════════════════════════════════════════════
//  ESP32 BLE HID Macro Pad
//  — BLE keyboard + mouse + consumer control (no USB, no external libs)
//  — 16-key 4×4 matrix  |  rotary encoder  |  3 potentiometers
//  — WiFi web dashboard
//
//  FIX vs previous version:
//    • BLESecurity added (bonding + "Just Works") — this is what makes the
//      device actually appear and pair on Windows / Android / iOS
//    • Security must be set BEFORE BLEDevice::createServer()
//    • Battery service + level required by many HID hosts
//    • Advertising flags explicitly set to 0x06 (LE General, no BR/EDR)
//    • onDisconnect now waits 500 ms before re-advertising (host needs it)
//    • Consumer descriptor simplified — 16-bit bitmap confused some hosts
// ═══════════════════════════════════════════════════════════════════════════

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>
#include <WiFi.h>
#include <WebServer.h>

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 1 — WiFi credentials
// ─────────────────────────────────────────────────────────────────────────────

const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 2 — Pin definitions
// ─────────────────────────────────────────────────────────────────────────────

// Rotary encoder
#define ENC_CLK  4
#define ENC_DT   5
#define ENC_SW   6    // push-button (INPUT_PULLUP, active LOW)

// Potentiometers (ADC pins)
#define POT_VOLUME      7
#define POT_BRIGHTNESS  1
#define POT_ZOOM        2

// 4×4 matrix
const int ROWS = 4;
const int COLS = 4;
int rowPins[ROWS] = {38, 39, 40, 41};
int colPins[COLS] = {42,  8, 17, 18};

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 3 — HID Report IDs & modifier masks
// ─────────────────────────────────────────────────────────────────────────────

#define REPORT_ID_KEYBOARD  1
#define REPORT_ID_MOUSE     2
#define REPORT_ID_CONSUMER  3

#define MOD_LCTRL   0x01
#define MOD_LSHIFT  0x02
#define MOD_LALT    0x04
#define MOD_LGUI    0x08

// HID keycodes for letters used in combos
#define KEY_C  0x06
#define KEY_D  0x07
#define KEY_E  0x08
#define KEY_L  0x0F
#define KEY_R  0x15
#define KEY_S  0x16
#define KEY_V  0x19

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 4 — HID Report Descriptor
//
//  Three report IDs:
//    1 — Keyboard  : 8 bytes  (1 modifier + 1 reserved + 6 keycodes)
//    2 — Mouse     : 4 bytes  (buttons + X + Y + wheel)
//    3 — Consumer  : 2 bytes  (usage bitmap: Vol+, Vol-, Mute, Play,
//                                            Next, Prev — 6 bits used,
//                                            10 bits padding)
// ─────────────────────────────────────────────────────────────────────────────

static const uint8_t hidDescriptor[] = {

  // ── Keyboard (Report ID 1) ─────────────────────────────────────────────
  0x05, 0x01,              // Usage Page: Generic Desktop
  0x09, 0x06,              // Usage: Keyboard
  0xA1, 0x01,              // Collection: Application
  0x85, REPORT_ID_KEYBOARD,//   Report ID
  // Modifier keys (8 bits)
  0x05, 0x07,              //   Usage Page: Keyboard/Keypad
  0x19, 0xE0,              //   Usage Minimum: Left Ctrl
  0x29, 0xE7,              //   Usage Maximum: Right GUI
  0x15, 0x00,              //   Logical Min: 0
  0x25, 0x01,              //   Logical Max: 1
  0x75, 0x01,              //   Report Size: 1 bit
  0x95, 0x08,              //   Report Count: 8
  0x81, 0x02,              //   Input: Data, Variable, Absolute
  // Reserved byte
  0x95, 0x01,              //   Report Count: 1
  0x75, 0x08,              //   Report Size: 8
  0x81, 0x01,              //   Input: Constant
  // Keycodes (6 bytes)
  0x95, 0x06,              //   Report Count: 6
  0x75, 0x08,              //   Report Size: 8
  0x15, 0x00,              //   Logical Min: 0
  0x25, 0x73,              //   Logical Max: 115
  0x05, 0x07,              //   Usage Page: Keyboard/Keypad
  0x19, 0x00,              //   Usage Minimum: 0
  0x29, 0x73,              //   Usage Maximum: 115
  0x81, 0x00,              //   Input: Data, Array
  0xC0,                    // End Collection

  // ── Mouse (Report ID 2) ────────────────────────────────────────────────
  0x05, 0x01,              // Usage Page: Generic Desktop
  0x09, 0x02,              // Usage: Mouse
  0xA1, 0x01,              // Collection: Application
  0x85, REPORT_ID_MOUSE,   //   Report ID
  0x09, 0x01,              //   Usage: Pointer
  0xA1, 0x00,              //   Collection: Physical
  // Buttons (3 bits + 5-bit padding)
  0x05, 0x09,              //     Usage Page: Buttons
  0x19, 0x01,              //     Usage Minimum: Button 1
  0x29, 0x03,              //     Usage Maximum: Button 3
  0x15, 0x00,              //     Logical Min: 0
  0x25, 0x01,              //     Logical Max: 1
  0x95, 0x03,              //     Report Count: 3
  0x75, 0x01,              //     Report Size: 1
  0x81, 0x02,              //     Input: Data, Variable, Absolute
  0x95, 0x01,              //     Report Count: 1
  0x75, 0x05,              //     Report Size: 5
  0x81, 0x01,              //     Input: Constant (padding)
  // X, Y, Wheel (-127 to 127)
  0x05, 0x01,              //     Usage Page: Generic Desktop
  0x09, 0x30,              //     Usage: X
  0x09, 0x31,              //     Usage: Y
  0x09, 0x38,              //     Usage: Wheel
  0x15, 0x81,              //     Logical Min: -127
  0x25, 0x7F,              //     Logical Max: 127
  0x75, 0x08,              //     Report Size: 8
  0x95, 0x03,              //     Report Count: 3
  0x81, 0x06,              //     Input: Data, Variable, Relative
  0xC0,                    //   End Collection (Physical)
  0xC0,                    // End Collection (Application)

  // ── Consumer Control (Report ID 3) ────────────────────────────────────
  //  2 bytes = 16 bits.  Bits 0-5 are used, bits 6-15 are padding.
  0x05, 0x0C,              // Usage Page: Consumer
  0x09, 0x01,              // Usage: Consumer Control
  0xA1, 0x01,              // Collection: Application
  0x85, REPORT_ID_CONSUMER,//   Report ID
  0x15, 0x00,              //   Logical Min: 0
  0x25, 0x01,              //   Logical Max: 1
  0x75, 0x01,              //   Report Size: 1 bit
  // 6 used controls
  0x95, 0x06,              //   Report Count: 6
  0x09, 0xE9,              //   Usage: Volume Increment
  0x09, 0xEA,              //   Usage: Volume Decrement
  0x09, 0xE2,              //   Usage: Mute
  0x09, 0xCD,              //   Usage: Play/Pause
  0x09, 0xB5,              //   Usage: Scan Next Track
  0x09, 0xB6,              //   Usage: Scan Previous Track
  0x81, 0x02,              //   Input: Data, Variable, Absolute
  // 10-bit padding to complete the 2 bytes
  0x95, 0x0A,              //   Report Count: 10
  0x81, 0x01,              //   Input: Constant
  0xC0                     // End Collection
};

// Consumer bit masks (bit position in the 2-byte report)
#define CON_VOL_UP    (uint16_t)(1 << 0)
#define CON_VOL_DOWN  (uint16_t)(1 << 1)
#define CON_MUTE      (uint16_t)(1 << 2)
#define CON_PLAY      (uint16_t)(1 << 3)
#define CON_NEXT      (uint16_t)(1 << 4)
#define CON_PREV      (uint16_t)(1 << 5)

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 5 — BLE globals & server callbacks
// ─────────────────────────────────────────────────────────────────────────────

BLEHIDDevice*      hid;
BLECharacteristic* inputKeyboard;
BLECharacteristic* inputMouse;
BLECharacteristic* inputConsumer;
bool               bleConnected = false;

class BLECallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleConnected = true;
    Serial.println("[BLE] Host connected");
  }
  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
    Serial.println("[BLE] Disconnected — re-advertising in 500 ms...");
    delay(500);           // give host time to clean up before we advertise again
    s->startAdvertising();
  }
};

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 6 — BLE send primitives
// ─────────────────────────────────────────────────────────────────────────────

void bleKeyDown(uint8_t modifier, uint8_t keycode) {
  if (!bleConnected) return;
  uint8_t r[8] = {modifier, 0x00, keycode, 0, 0, 0, 0, 0};
  inputKeyboard->setValue(r, sizeof(r));
  inputKeyboard->notify();
  delay(10);
}

void bleKeyUp() {
  if (!bleConnected) return;
  uint8_t r[8] = {0};
  inputKeyboard->setValue(r, sizeof(r));
  inputKeyboard->notify();
  delay(10);
}

void bleMouse(int8_t x, int8_t y, int8_t wheel, uint8_t buttons = 0) {
  if (!bleConnected) return;
  uint8_t r[4] = {buttons, (uint8_t)x, (uint8_t)y, (uint8_t)wheel};
  inputMouse->setValue(r, sizeof(r));
  inputMouse->notify();
}

void bleConsumer(uint16_t bits) {
  if (!bleConnected) return;
  uint8_t press[2]   = {(uint8_t)(bits & 0xFF), (uint8_t)(bits >> 8)};
  uint8_t release[2] = {0, 0};
  inputConsumer->setValue(press,   sizeof(press));   inputConsumer->notify(); delay(15);
  inputConsumer->setValue(release, sizeof(release)); inputConsumer->notify(); delay(10);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 7 — Higher-level input helpers
// ─────────────────────────────────────────────────────────────────────────────

// ASCII character → packed uint16_t  (high byte = modifier, low byte = keycode)
uint16_t charToHID(char c) {
  #define HID(mod, key) ( ((uint16_t)(mod) << 8) | (uint8_t)(key) )
  if (c >= 'a' && c <= 'z') return HID(0,          c - 'a' + 0x04);
  if (c >= 'A' && c <= 'Z') return HID(MOD_LSHIFT, c - 'A' + 0x04);
  if (c >= '1' && c <= '9') return HID(0,          c - '1' + 0x1E);
  switch (c) {
    case '0':  return HID(0,          0x27);
    case ' ':  return HID(0,          0x2C);
    case '\n': return HID(0,          0x28);
    case '.':  return HID(0,          0x37);
    case ',':  return HID(0,          0x36);
    case '-':  return HID(0,          0x2D);
    case '=':  return HID(0,          0x2E);
    case '/':  return HID(0,          0x38);
    case '\\': return HID(0,          0x31);
    case '[':  return HID(0,          0x2F);
    case ']':  return HID(0,          0x30);
    case '+':  return HID(MOD_LSHIFT, 0x2E);
    case '_':  return HID(MOD_LSHIFT, 0x2D);
    case '(':  return HID(MOD_LSHIFT, 0x26);
    case ')':  return HID(MOD_LSHIFT, 0x27);
    case '{':  return HID(MOD_LSHIFT, 0x2F);
    case '}':  return HID(MOD_LSHIFT, 0x30);
    case '@':  return HID(MOD_LSHIFT, 0x1F);
    case '*':  return HID(MOD_LSHIFT, 0x25);
    case '&':  return HID(MOD_LSHIFT, 0x24);
    case '^':  return HID(MOD_LSHIFT, 0x23);
    case '<':  return HID(MOD_LSHIFT, 0x36);
    case '>':  return HID(MOD_LSHIFT, 0x37);
    default:   return HID(0, 0);
  }
  #undef HID
}

void typeString(const char* str) {
  for (int i = 0; str[i] != '\0'; i++) {
    uint16_t k = charToHID(str[i]);
    uint8_t  mod = k >> 8, code = k & 0xFF;
    if (code) { bleKeyDown(mod, code); bleKeyUp(); delay(8); }
  }
}

// Press modifier combo (mod2 = 0 if only one modifier needed)
void pressCombo(uint8_t mod1, uint8_t mod2, uint8_t keycode) {
  bleKeyDown(mod1 | mod2, keycode);
  delay(80);
  bleKeyUp();
  delay(50);
}

// Win+R → type command → Enter  (opens Windows Run dialog)
void winRun(const char* cmd) {
  pressCombo(MOD_LGUI, 0, KEY_R);
  delay(500);               // wait for Run dialog to open
  typeString(cmd);
  delay(100);
  bleKeyDown(0, 0x28);      // Enter
  bleKeyUp();
  delay(300);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 8 — Key actions (Code 2 functionality)
//
//   1  Calculator        Win+R → calc
//   2  Word              Win+R → winword
//   3  PowerPoint        Win+R → powerpnt
//   4  Next Track        Consumer
//   5  Excel             Win+R → excel
//   6  Notepad           Win+R → notepad
//   7  Task Manager      Ctrl+Shift+Esc
//   8  Prev Track        Consumer
//   9  File Explorer     Win+E
//  10  Snip screenshot   Win+Shift+S
//  11  Lock PC           Win+L
//  12  Mute              Consumer
//  13  Copy              Ctrl+C
//  14  Play/Pause        Consumer
//  15  Paste             Ctrl+V
//  16  Show Desktop      Win+D
// ─────────────────────────────────────────────────────────────────────────────

void runAction(int key) {
  switch (key) {
    case  1: winRun("calc");                               break;
    case  2: winRun("winword");                            break;
    case  3: winRun("powerpnt");                           break;
    case  4: bleConsumer(CON_NEXT);                        break;
    case  5: winRun("excel");                              break;
    case  6: winRun("notepad");                            break;
    case  7: pressCombo(MOD_LCTRL, MOD_LSHIFT, 0x29);     break; // Esc keycode
    case  8: bleConsumer(CON_PREV);                        break;
    case  9: pressCombo(MOD_LGUI,  0,           KEY_E);   break;
    case 10: pressCombo(MOD_LGUI,  MOD_LSHIFT,  KEY_S);   break;
    case 11: pressCombo(MOD_LGUI,  0,           KEY_L);   break;
    case 12: bleConsumer(CON_MUTE);                        break;
    case 13: pressCombo(MOD_LCTRL, 0,           KEY_C);   break;
    case 14: bleConsumer(CON_PLAY);                        break;
    case 15: pressCombo(MOD_LCTRL, 0,           KEY_V);   break;
    case 16: pressCombo(MOD_LGUI,  0,           KEY_D);   break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 9 — 4×4 Matrix keypad (debounced)
// ─────────────────────────────────────────────────────────────────────────────

int keys[ROWS][COLS] = {
  { 1,  2,  3,  4},
  { 5,  6,  7,  8},
  { 9, 10, 11, 12},
  {13, 14, 15, 16}
};

bool          matrixPrev[ROWS][COLS]    = {};
bool          matrixRaw[ROWS][COLS]     = {};
unsigned long matrixDebounce[ROWS][COLS]= {};
const unsigned long DEBOUNCE_MS         = 15;

void scanMatrix() {
  unsigned long now = millis();
  for (int r = 0; r < ROWS; r++) {
    digitalWrite(rowPins[r], LOW);
    delayMicroseconds(10);
    for (int c = 0; c < COLS; c++) {
      bool pressed = (digitalRead(colPins[c]) == LOW);
      if (pressed != matrixRaw[r][c]) {
        matrixDebounce[r][c] = now;
        matrixRaw[r][c]      = pressed;
      }
      if ((now - matrixDebounce[r][c]) >= DEBOUNCE_MS) {
        if (pressed && !matrixPrev[r][c]) runAction(keys[r][c]);
        matrixPrev[r][c] = pressed;
      }
    }
    digitalWrite(rowPins[r], HIGH);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 10 — Hardware state (encoder + pots)
// ─────────────────────────────────────────────────────────────────────────────

int  encLastCLK;
bool encLastBtn = HIGH;

int lastVolume     = -1;
int lastBrightness = -1;
int lastZoom       = -1;

void handleEncoder() {
  int clk = digitalRead(ENC_CLK);
  if (clk != encLastCLK && bleConnected) {
    bleMouse(0, 0, (digitalRead(ENC_DT) != clk) ? 1 : -1);
    delay(2);
  }
  encLastCLK = clk;
}

void handleEncoderButton() {
  bool btn = digitalRead(ENC_SW);
  if (encLastBtn == HIGH && btn == LOW) {
    winRun("devmgmt.msc");   // Open Device Manager
  }
  encLastBtn = btn;
}

void handleVolumePot() {
  int vol = map(analogRead(POT_VOLUME), 0, 4095, 0, 100);
  if (!bleConnected || abs(vol - lastVolume) <= 2) { lastVolume = (lastVolume == -1) ? vol : lastVolume; return; }
  if (lastVolume == -1) { lastVolume = vol; return; }
  uint16_t bit   = (vol > lastVolume) ? CON_VOL_UP : CON_VOL_DOWN;
  int      steps = abs(vol - lastVolume);
  for (int i = 0; i < steps; i++) { bleConsumer(bit); delay(2); }
  lastVolume = vol;
}

void handleBrightnessPot() {
  // Brightness via Ctrl+scroll — most universally supported across OSes
  int bright = map(analogRead(POT_BRIGHTNESS), 0, 4095, 0, 100);
  if (!bleConnected || abs(bright - lastBrightness) <= 2) { lastBrightness = (lastBrightness == -1) ? bright : lastBrightness; return; }
  if (lastBrightness == -1) { lastBrightness = bright; return; }
  // Hold Ctrl and scroll — works as brightness in many fullscreen apps / presentation tools
  // On Windows laptops, Fn+F keys are hardware-only; Ctrl+scroll is the BLE-compatible alternative
  uint8_t ctrlHeld[8] = {MOD_LCTRL, 0, 0, 0, 0, 0, 0, 0};
  inputKeyboard->setValue(ctrlHeld, sizeof(ctrlHeld));
  inputKeyboard->notify();
  delay(20);
  bleMouse(0, 0, (bright > lastBrightness) ? 1 : -1);
  delay(10);
  bleKeyUp();
  lastBrightness = bright;
}

void handleZoomPot() {
  int zoom = map(analogRead(POT_ZOOM), 0, 4095, 0, 100);
  if (!bleConnected || abs(zoom - lastZoom) <= 2) { lastZoom = (lastZoom == -1) ? zoom : lastZoom; return; }
  if (lastZoom == -1) { lastZoom = zoom; return; }
  uint8_t ctrlHeld[8] = {MOD_LCTRL, 0, 0, 0, 0, 0, 0, 0};
  inputKeyboard->setValue(ctrlHeld, sizeof(ctrlHeld));
  inputKeyboard->notify();
  delay(20);
  bleMouse(0, 0, (zoom > lastZoom) ? 1 : -1);
  delay(10);
  bleKeyUp();
  lastZoom = zoom;
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 11 — WiFi web dashboard
// ─────────────────────────────────────────────────────────────────────────────

WebServer webServer(80);

void handleRoot() {
  String ble = bleConnected
    ? "<span class='on'>&#9679; Connected</span>"
    : "<span class='off'>&#9675; Waiting — pair 'ESP32 MacroPad' from your device</span>";

  String html = R"(<!DOCTYPE html><html><head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>ESP32 MacroPad</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:system-ui,sans-serif;background:#0f0f0f;color:#ddd;padding:20px;max-width:500px;margin:0 auto}
  h1{color:#58a6ff;font-size:1.3rem;margin-bottom:16px}
  .card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px;margin-bottom:12px}
  .card h2{font-size:.85rem;text-transform:uppercase;letter-spacing:.05em;color:#8b949e;margin-bottom:10px}
  .row{display:flex;justify-content:space-between;font-size:.9rem;padding:3px 0}
  .on{color:#3fb950}.off{color:#d29922}
  table{width:100%;border-collapse:collapse;font-size:.82rem}
  th{background:#21262d;color:#58a6ff;padding:6px 8px;text-align:left}
  td{padding:5px 8px;border-bottom:1px solid #21262d}
  .sym{text-align:center;font-size:1rem}
</style></head><body>
<h1>&#9654; ESP32 BLE MacroPad</h1>
<div class='card'>
  <h2>Status</h2>
  <div class='row'><span>WiFi IP</span><span>)" + WiFi.localIP().toString() + R"(</span></div>
  <div class='row'><span>Bluetooth</span>)" + ble + R"(</div>
</div>
<div class='card'>
  <h2>Controls</h2>
  <div class='row'><span>Encoder rotate</span><span>Mouse scroll</span></div>
  <div class='row'><span>Encoder button</span><span>Device Manager</span></div>
  <div class='row'><span>Pot 1 (pin 7)</span><span>Volume</span></div>
  <div class='row'><span>Pot 2 (pin 1)</span><span>Brightness (Ctrl+scroll)</span></div>
  <div class='row'><span>Pot 3 (pin 2)</span><span>Zoom (Ctrl+scroll)</span></div>
</div>
<div class='card'>
  <h2>Key Map</h2>
  <table>
    <tr><th>Key</th><th>Action</th><th>Shortcut</th></tr>
    <tr><td>1</td><td>Calculator</td><td>Win+R → calc</td></tr>
    <tr><td>2</td><td>Word</td><td>Win+R → winword</td></tr>
    <tr><td>3</td><td>PowerPoint</td><td>Win+R → powerpnt</td></tr>
    <tr><td class='sym'>4</td><td>Next Track</td><td class='sym'>&#9654;&#9654;</td></tr>
    <tr><td>5</td><td>Excel</td><td>Win+R → excel</td></tr>
    <tr><td>6</td><td>Notepad</td><td>Win+R → notepad</td></tr>
    <tr><td>7</td><td>Task Manager</td><td>Ctrl+Shift+Esc</td></tr>
    <tr><td class='sym'>8</td><td>Prev Track</td><td class='sym'>&#9664;&#9664;</td></tr>
    <tr><td>9</td><td>File Explorer</td><td>Win+E</td></tr>
    <tr><td>10</td><td>Screenshot</td><td>Win+Shift+S</td></tr>
    <tr><td>11</td><td>Lock PC</td><td>Win+L</td></tr>
    <tr><td class='sym'>12</td><td>Mute</td><td class='sym'>&#128263;</td></tr>
    <tr><td>13</td><td>Copy</td><td>Ctrl+C</td></tr>
    <tr><td class='sym'>14</td><td>Play / Pause</td><td class='sym'>&#9654;&#9646;&#9646;</td></tr>
    <tr><td>15</td><td>Paste</td><td>Ctrl+V</td></tr>
    <tr><td>16</td><td>Show Desktop</td><td>Win+D</td></tr>
  </table>
</div></body></html>)";

  webServer.send(200, "text/html", html);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 12 — BLE initialization
//
//  Order matters:
//    1. BLEDevice::init()
//    2. BLEDevice::setEncryptionLevel()   ← required for HID pairing
//    3. new BLESecurity() configured      ← "Just Works" bonding, no PIN
//    4. createServer() + callbacks
//    5. BLEHIDDevice setup
//    6. hid->startServices()
//    7. Advertising with explicit flags
// ─────────────────────────────────────────────────────────────────────────────

void startBLE() {
  BLEDevice::init("ESP32 MacroPad");
  // NOTE: BLEDevice::setEncryptionLevel() does not exist in the Arduino-ESP32
  // BLE wrapper. Encryption is configured entirely through BLESecurity below.

  // Security: "Just Works" bonding — host will pair without a PIN prompt.
  // This must be configured BEFORE createServer().
  BLESecurity* pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);
  pSecurity->setCapability(ESP_IO_CAP_NONE);
  pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BLECallbacks());

  // HID device
  hid           = new BLEHIDDevice(pServer);
  inputKeyboard = hid->inputReport(REPORT_ID_KEYBOARD);
  inputMouse    = hid->inputReport(REPORT_ID_MOUSE);
  inputConsumer = hid->inputReport(REPORT_ID_CONSUMER);

  hid->manufacturer()->setValue("ESP32Build");
  hid->pnp(0x02, 0x05AC, 0x820A, 0x0210);  // vendorID, productID, version
  hid->hidInfo(0x00, 0x01);                  // country=0, flags=normally connectable
  hid->reportMap((uint8_t*)hidDescriptor, sizeof(hidDescriptor));
  hid->setBatteryLevel(100);                 // many hosts require battery service
  hid->startServices();

  // Advertising — explicit flags + HID service UUID are critical for pairing.
  // Use hid->hidService()->getUUID() instead of HID_SERVICE_UUID_16 (not
  // defined in the Arduino-ESP32 BLE wrapper headers).
  BLEAdvertisementData advData;
  advData.setFlags(0x06);                    // LE General Discoverable + no BR/EDR
  advData.setAppearance(HID_KEYBOARD);
  advData.setCompleteServices(hid->hidService()->getUUID());

  BLEAdvertisementData scanData;
  scanData.setName("ESP32 MacroPad");        // name goes in scan response to keep adv packet small

  BLEAdvertising* adv = pServer->getAdvertising();
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  adv->start();

  Serial.println("[BLE] Advertising as 'ESP32 MacroPad' — pair from your device");
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 13 — Setup
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  // Encoder
  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DT,  INPUT);
  pinMode(ENC_SW,  INPUT_PULLUP);
  encLastCLK = digitalRead(ENC_CLK);

  // ADC
  analogReadResolution(12);

  // Matrix rows → output HIGH, cols → input pullup
  for (int r = 0; r < ROWS; r++) { pinMode(rowPins[r], OUTPUT); digitalWrite(rowPins[r], HIGH); }
  for (int c = 0; c < COLS; c++) { pinMode(colPins[c], INPUT_PULLUP); }

  // BLE (must come before WiFi to avoid heap competition)
  startBLE();

  // WiFi (optional — BLE works independently if WiFi fails)
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Connecting");
  for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] http://" + WiFi.localIP().toString());
    webServer.on("/", handleRoot);
    webServer.begin();
  } else {
    Serial.println("\n[WiFi] Failed — running BLE only");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SECTION 14 — Main loop
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
  handleEncoder();
  handleEncoderButton();
  handleVolumePot();
  handleBrightnessPot();
  handleZoomPot();
  scanMatrix();
  if (WiFi.status() == WL_CONNECTED) webServer.handleClient();
  delay(5);
}
