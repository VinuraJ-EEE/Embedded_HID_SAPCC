// ═══════════════════════════════════════════════════════════════
//  ESP32-S3-N16R8  BLE HID MacroPad
//  FreeRTOS — PlatformIO
//
//  Core 0 ── taskWiFi  P1  web dashboard
//          ── taskPots  P2  3× ADC potentiometers
//  Core 1 ── taskBLE   P4  action queue → BLE HID reports
//          ── taskInput P3  matrix + encoder → action queue
// ═══════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEHIDDevice.h>
#include <HIDTypes.h>
#include <WiFi.h>
#include <WebServer.h>

// ── WiFi credentials ──────────────────────────────────────────
#define WIFI_SSID  "YOUR_SSID"
#define WIFI_PASS  "YOUR_PASSWORD"

// ── Pin definitions ───────────────────────────────────────────
#define ENC_CLK         4
#define ENC_DT          5
#define ENC_SW          6
#define POT_VOLUME      7
#define POT_BRIGHTNESS  1
#define POT_ZOOM        2

const int ROWS = 4;
const int COLS = 4;
int rowPins[ROWS] = {38, 39, 40, 41};
int colPins[COLS] = {42,  8, 17, 18};

// ── HID report IDs & modifier masks ──────────────────────────
#define REPORT_ID_KEYBOARD  1
#define REPORT_ID_MOUSE     2
#define REPORT_ID_CONSUMER  3

#define MOD_LCTRL   0x01
#define MOD_LSHIFT  0x02
#define MOD_LALT    0x04
#define MOD_LGUI    0x08

// HID keycodes for letters used in combos
#define HID_KEY_C   0x06
#define HID_KEY_D   0x07
#define HID_KEY_E   0x08
#define HID_KEY_L   0x0F
#define HID_KEY_R   0x15
#define HID_KEY_S   0x16
#define HID_KEY_V   0x19
#define HID_KEY_ESC 0x29
#define HID_KEY_ENT 0x28

// ── Consumer control bit masks ────────────────────────────────
#define CON_VOL_UP    (uint16_t)(1 << 0)
#define CON_VOL_DOWN  (uint16_t)(1 << 1)
#define CON_MUTE      (uint16_t)(1 << 2)
#define CON_PLAY      (uint16_t)(1 << 3)
#define CON_NEXT      (uint16_t)(1 << 4)
#define CON_PREV      (uint16_t)(1 << 5)

// ── HID Report Descriptor ─────────────────────────────────────
static const uint8_t hidDescriptor[] = {
  // Keyboard — Report ID 1 (8 bytes)
  0x05,0x01, 0x09,0x06, 0xA1,0x01,
  0x85,REPORT_ID_KEYBOARD,
  0x05,0x07, 0x19,0xE0, 0x29,0xE7,
  0x15,0x00, 0x25,0x01,
  0x75,0x01, 0x95,0x08, 0x81,0x02,
  0x95,0x01, 0x75,0x08, 0x81,0x01,
  0x95,0x06, 0x75,0x08,
  0x15,0x00, 0x25,0x73,
  0x05,0x07, 0x19,0x00, 0x29,0x73,
  0x81,0x00, 0xC0,

  // Mouse — Report ID 2 (4 bytes)
  0x05,0x01, 0x09,0x02, 0xA1,0x01,
  0x85,REPORT_ID_MOUSE,
  0x09,0x01, 0xA1,0x00,
  0x05,0x09, 0x19,0x01, 0x29,0x03,
  0x15,0x00, 0x25,0x01,
  0x95,0x03, 0x75,0x01, 0x81,0x02,
  0x95,0x01, 0x75,0x05, 0x81,0x01,
  0x05,0x01, 0x09,0x30, 0x09,0x31, 0x09,0x38,
  0x15,0x81, 0x25,0x7F,
  0x75,0x08, 0x95,0x03, 0x81,0x06,
  0xC0, 0xC0,

  // Consumer — Report ID 3 (2 bytes: 6 controls + 10-bit pad)
  0x05,0x0C, 0x09,0x01, 0xA1,0x01,
  0x85,REPORT_ID_CONSUMER,
  0x15,0x00, 0x25,0x01, 0x75,0x01,
  0x95,0x06,
  0x09,0xE9, 0x09,0xEA, 0x09,0xE2,
  0x09,0xCD, 0x09,0xB5, 0x09,0xB6,
  0x81,0x02,
  0x95,0x0A, 0x81,0x01,
  0xC0
};

// ── Action queue ──────────────────────────────────────────────
// Every input event is wrapped here and sent to taskBLE.
// Input tasks NEVER wait for BLE — they just post and move on.
enum ActType : uint8_t {
  ACT_KEY,       // matrix key number 1-16
  ACT_SCROLL,    // encoder scroll  value = +1 or -1
  ACT_WIN_RUN,   // encoder button  opens Device Manager
  ACT_CONSUMER,  // consumer HID bits  (volume steps)
  ACT_BRIGHT,    // brightness  value = +1 or -1 (Ctrl+scroll)
  ACT_ZOOM,      // zoom        value = +1 or -1 (Ctrl+scroll)
};

struct Action {
  ActType type;
  int32_t value;
};

// ── FreeRTOS handles ─────────────────────────────────────────
static QueueHandle_t     actionQueue;
static SemaphoreHandle_t bleMutex;
static volatile bool     bleConnected = false;

// ── BLE globals ───────────────────────────────────────────────
static BLEHIDDevice*      hid;
static BLECharacteristic* inputKeyboard;
static BLECharacteristic* inputMouse;
static BLECharacteristic* inputConsumer;

// ═══════════════════════════════════════════════════════════════
//  BLE PRIMITIVES
//  Only called from taskBLE while holding bleMutex
// ═══════════════════════════════════════════════════════════════

static void _keyDown(uint8_t mod, uint8_t code) {
  if (!bleConnected) return;
  uint8_t r[8] = {mod, 0x00, code, 0, 0, 0, 0, 0};
  inputKeyboard->setValue(r, sizeof(r));
  inputKeyboard->notify();
  vTaskDelay(pdMS_TO_TICKS(10));
}

static void _keyUp() {
  if (!bleConnected) return;
  uint8_t r[8] = {0};
  inputKeyboard->setValue(r, sizeof(r));
  inputKeyboard->notify();
  vTaskDelay(pdMS_TO_TICKS(10));
}

static void _mouseScroll(int8_t wheel) {
  if (!bleConnected) return;
  uint8_t r[4] = {0, 0, 0, (uint8_t)wheel};
  inputMouse->setValue(r, sizeof(r));
  inputMouse->notify();
  vTaskDelay(pdMS_TO_TICKS(5));
}

static void _consumer(uint16_t bits) {
  if (!bleConnected) return;
  uint8_t press[2]   = {(uint8_t)(bits & 0xFF), (uint8_t)(bits >> 8)};
  uint8_t release[2] = {0, 0};
  inputConsumer->setValue(press,   sizeof(press));
  inputConsumer->notify();
  vTaskDelay(pdMS_TO_TICKS(15));
  inputConsumer->setValue(release, sizeof(release));
  inputConsumer->notify();
  vTaskDelay(pdMS_TO_TICKS(10));
}

static void _ctrlScroll(int8_t dir) {
  if (!bleConnected) return;
  uint8_t ctrlOn[8]  = {MOD_LCTRL, 0, 0, 0, 0, 0, 0, 0};
  uint8_t ctrlOff[8] = {0};
  inputKeyboard->setValue(ctrlOn,  sizeof(ctrlOn));
  inputKeyboard->notify();
  vTaskDelay(pdMS_TO_TICKS(20));
  _mouseScroll(dir);
  vTaskDelay(pdMS_TO_TICKS(10));
  inputKeyboard->setValue(ctrlOff, sizeof(ctrlOff));
  inputKeyboard->notify();
  vTaskDelay(pdMS_TO_TICKS(10));
}

// ── ASCII → packed HID uint16_t ───────────────────────────────
// High byte = modifier, low byte = keycode
static uint16_t charToHID(char c) {
  #define P(m,k) (((uint16_t)(m) << 8) | (uint8_t)(k))
  if (c >= 'a' && c <= 'z') return P(0,          c - 'a' + 0x04);
  if (c >= 'A' && c <= 'Z') return P(MOD_LSHIFT, c - 'A' + 0x04);
  if (c >= '1' && c <= '9') return P(0,          c - '1' + 0x1E);
  switch (c) {
    case '0':  return P(0,          0x27);
    case ' ':  return P(0,          0x2C);
    case '\n': return P(0,          0x28);
    case '.':  return P(0,          0x37);
    case ',':  return P(0,          0x36);
    case '-':  return P(0,          0x2D);
    case '=':  return P(0,          0x2E);
    case '/':  return P(0,          0x38);
    case '\\': return P(0,          0x31);
    case '[':  return P(0,          0x2F);
    case ']':  return P(0,          0x30);
    case '_':  return P(MOD_LSHIFT, 0x2D);
    case '+':  return P(MOD_LSHIFT, 0x2E);
    case '(':  return P(MOD_LSHIFT, 0x26);
    case ')':  return P(MOD_LSHIFT, 0x27);
    case '{':  return P(MOD_LSHIFT, 0x2F);
    case '}':  return P(MOD_LSHIFT, 0x30);
    case '@':  return P(MOD_LSHIFT, 0x1F);
    case '*':  return P(MOD_LSHIFT, 0x25);
    case '&':  return P(MOD_LSHIFT, 0x24);
    case '^':  return P(MOD_LSHIFT, 0x23);
    case '<':  return P(MOD_LSHIFT, 0x36);
    case '>':  return P(MOD_LSHIFT, 0x37);
    default:   return P(0, 0);
  }
  #undef P
}

static void _typeStr(const char* s) {
  for (int i = 0; s[i] != '\0'; i++) {
    uint16_t k    = charToHID(s[i]);
    uint8_t  mod  = (uint8_t)(k >> 8);
    uint8_t  code = (uint8_t)(k & 0xFF);
    if (code) {
      _keyDown(mod, code);
      _keyUp();
      vTaskDelay(pdMS_TO_TICKS(8));
    }
  }
}

static void _combo(uint8_t mod, uint8_t key) {
  _keyDown(mod, key);
  vTaskDelay(pdMS_TO_TICKS(80));
  _keyUp();
  vTaskDelay(pdMS_TO_TICKS(50));
}

static void _winRun(const char* cmd) {
  _combo(MOD_LGUI, HID_KEY_R);
  vTaskDelay(pdMS_TO_TICKS(500));
  _typeStr(cmd);
  vTaskDelay(pdMS_TO_TICKS(100));
  _keyDown(0, HID_KEY_ENT);
  _keyUp();
  vTaskDelay(pdMS_TO_TICKS(300));
}

// ── Key actions 1-16 ─────────────────────────────────────────
static void dispatchKey(int key) {
  switch (key) {
    case  1: _winRun("calc");                              break; // Calculator
    case  2: _winRun("winword");                           break; // MS Word
    case  3: _winRun("powerpnt");                          break; // PowerPoint
    case  4: _consumer(CON_NEXT);                          break; // Next track
    case  5: _winRun("excel");                             break; // Excel
    case  6: _winRun("notepad");                           break; // Notepad
    case  7: _combo(MOD_LCTRL|MOD_LSHIFT, HID_KEY_ESC);   break; // Task Manager
    case  8: _consumer(CON_PREV);                          break; // Prev track
    case  9: _combo(MOD_LGUI, HID_KEY_E);                  break; // File Explorer
    case 10: _combo(MOD_LGUI|MOD_LSHIFT, HID_KEY_S);       break; // Screenshot
    case 11: _combo(MOD_LGUI, HID_KEY_L);                  break; // Lock PC
    case 12: _consumer(CON_MUTE);                          break; // Mute
    case 13: _combo(MOD_LCTRL, HID_KEY_C);                 break; // Copy
    case 14: _consumer(CON_PLAY);                          break; // Play/Pause
    case 15: _combo(MOD_LCTRL, HID_KEY_V);                 break; // Paste
    case 16: _combo(MOD_LGUI, HID_KEY_D);                  break; // Show desktop
  }
}

// ═══════════════════════════════════════════════════════════════
//  TASK: BLE dispatcher
//  Core 1 — Priority 4 (highest)
//  Blocks on queue; wakes only when an action arrives.
//  Holds bleMutex during every BLE transaction.
// ═══════════════════════════════════════════════════════════════
static void taskBLE(void*) {
  Action act;
  for (;;) {
    if (xQueueReceive(actionQueue, &act, portMAX_DELAY) != pdTRUE) continue;
    if (!bleConnected) continue;

    xSemaphoreTake(bleMutex, portMAX_DELAY);

    switch (act.type) {
      case ACT_KEY:      dispatchKey((int)act.value);       break;
      case ACT_SCROLL:   _mouseScroll((int8_t)act.value);   break;
      case ACT_WIN_RUN:  _winRun("devmgmt.msc");            break;
      case ACT_CONSUMER: _consumer((uint16_t)act.value);    break;
      case ACT_BRIGHT:   _ctrlScroll((int8_t)act.value);    break;
      case ACT_ZOOM:     _ctrlScroll((int8_t)act.value);    break;
      default: break;
    }

    xSemaphoreGive(bleMutex);
  }
}

// ═══════════════════════════════════════════════════════════════
//  TASK: Input scanner
//  Core 1 — Priority 3
//  Scans matrix + encoder; posts to queue (never blocks on BLE).
// ═══════════════════════════════════════════════════════════════

// Matrix state
static int    matrixKeys[ROWS][COLS] = {
  { 1,  2,  3,  4},
  { 5,  6,  7,  8},
  { 9, 10, 11, 12},
  {13, 14, 15, 16}
};
static bool     matPrev[ROWS][COLS]     = {};
static bool     matRaw[ROWS][COLS]      = {};
static uint32_t matDebounce[ROWS][COLS] = {};
static const uint32_t DEBOUNCE_MS       = 15;

static inline void postAction(ActType t, int32_t v) {
  Action a = {t, v};
  xQueueSend(actionQueue, &a, 0);  // non-blocking — drop if full
}

static void scanMatrix(uint32_t now) {
  for (int r = 0; r < ROWS; r++) {
    digitalWrite(rowPins[r], LOW);
    delayMicroseconds(10);
    for (int c = 0; c < COLS; c++) {
      bool p = (digitalRead(colPins[c]) == LOW);
      if (p != matRaw[r][c]) {
        matDebounce[r][c] = now;
        matRaw[r][c]      = p;
      }
      if ((now - matDebounce[r][c]) >= DEBOUNCE_MS) {
        if (p && !matPrev[r][c]) postAction(ACT_KEY, matrixKeys[r][c]);
        matPrev[r][c] = p;
      }
    }
    digitalWrite(rowPins[r], HIGH);
  }
}

// Encoder state
static int  encLastCLK;
static bool encLastBtn = HIGH;

static void scanEncoder() {
  int clk = digitalRead(ENC_CLK);
  if (clk != encLastCLK) {
    postAction(ACT_SCROLL, (digitalRead(ENC_DT) != clk) ? 1 : -1);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  encLastCLK = clk;

  bool btn = digitalRead(ENC_SW);
  if (encLastBtn == HIGH && btn == LOW) postAction(ACT_WIN_RUN, 0);
  encLastBtn = btn;
}

static void taskInput(void*) {
  for (;;) {
    uint32_t now = (uint32_t)millis();
    scanMatrix(now);
    scanEncoder();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ═══════════════════════════════════════════════════════════════
//  TASK: Potentiometers
//  Core 0 — Priority 2
//  Reads 3 ADC channels at 50 Hz; posts volume/brightness/zoom.
//  On Core 0 alongside WiFi to keep ADC away from BLE core.
// ═══════════════════════════════════════════════════════════════
static void taskPots(void*) {
  int lastVol = -1, lastBri = -1, lastZoo = -1;

  for (;;) {
    // ── Volume ──────────────────────────────────────────────
    int vol = map(analogRead(POT_VOLUME), 0, 4095, 0, 100);
    if (lastVol == -1) {
      lastVol = vol;
    } else if (abs(vol - lastVol) > 2) {
      uint16_t bit   = (vol > lastVol) ? CON_VOL_UP : CON_VOL_DOWN;
      int      steps = abs(vol - lastVol);
      for (int i = 0; i < steps; i++) postAction(ACT_CONSUMER, (int32_t)bit);
      lastVol = vol;
    }

    // ── Brightness ──────────────────────────────────────────
    int bri = map(analogRead(POT_BRIGHTNESS), 0, 4095, 0, 100);
    if (lastBri == -1) {
      lastBri = bri;
    } else if (abs(bri - lastBri) > 2) {
      postAction(ACT_BRIGHT, (bri > lastBri) ? 1 : -1);
      lastBri = bri;
    }

    // ── Zoom ────────────────────────────────────────────────
    int zoo = map(analogRead(POT_ZOOM), 0, 4095, 0, 100);
    if (lastZoo == -1) {
      lastZoo = zoo;
    } else if (abs(zoo - lastZoo) > 2) {
      postAction(ACT_ZOOM, (zoo > lastZoo) ? 1 : -1);
      lastZoo = zoo;
    }

    vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz
  }
}

// ═══════════════════════════════════════════════════════════════
//  TASK: WiFi web dashboard
//  Core 0 — Priority 1 (lowest)
//  Never touches BLE characteristics.
// ═══════════════════════════════════════════════════════════════
static WebServer webServer(80);

static void handleRoot() {
  String ble = bleConnected
    ? "<span class='on'>&#9679; Connected</span>"
    : "<span class='off'>&#9675; Waiting — pair ESP32 MacroPad</span>";

  String html = R"(<!DOCTYPE html><html>
<head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>MacroPad</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:system-ui,sans-serif;background:#0f0f0f;
       color:#ddd;padding:20px;max-width:500px;margin:0 auto}
  h1{color:#58a6ff;font-size:1.2rem;margin-bottom:14px}
  .card{background:#161b22;border:1px solid #30363d;
        border-radius:8px;padding:14px;margin-bottom:12px}
  .card h2{font-size:.78rem;text-transform:uppercase;
           letter-spacing:.05em;color:#8b949e;margin-bottom:8px}
  .row{display:flex;justify-content:space-between;
       font-size:.88rem;padding:3px 0}
  .on{color:#3fb950}.off{color:#d29922}
  table{width:100%;border-collapse:collapse;font-size:.82rem}
  th{background:#21262d;color:#58a6ff;padding:5px 8px;text-align:left}
  td{padding:4px 8px;border-bottom:1px solid #21262d}
</style>
</head>
<body>
<h1>&#9654; ESP32 BLE MacroPad</h1>
<div class='card'><h2>Status</h2>
  <div class='row'><span>WiFi IP</span><span>)" +
  WiFi.localIP().toString() + R"(</span></div>
  <div class='row'><span>Bluetooth</span>)" + ble + R"(</div>
</div>
<div class='card'><h2>Controls</h2>
  <div class='row'><span>Encoder rotate</span><span>Mouse scroll</span></div>
  <div class='row'><span>Encoder button</span><span>Device Manager</span></div>
  <div class='row'><span>Pot 1  GPIO 7</span><span>Volume</span></div>
  <div class='row'><span>Pot 2  GPIO 1</span><span>Brightness</span></div>
  <div class='row'><span>Pot 3  GPIO 2</span><span>Zoom</span></div>
</div>
<div class='card'><h2>Key Map</h2>
<table>
  <tr><th>#</th><th>Action</th><th>Method</th></tr>
  <tr><td>1</td><td>Calculator</td><td>Win+R calc</td></tr>
  <tr><td>2</td><td>Word</td><td>Win+R winword</td></tr>
  <tr><td>3</td><td>PowerPoint</td><td>Win+R powerpnt</td></tr>
  <tr><td>4</td><td>Next track</td><td>Consumer HID</td></tr>
  <tr><td>5</td><td>Excel</td><td>Win+R excel</td></tr>
  <tr><td>6</td><td>Notepad</td><td>Win+R notepad</td></tr>
  <tr><td>7</td><td>Task Manager</td><td>Ctrl+Shift+Esc</td></tr>
  <tr><td>8</td><td>Prev track</td><td>Consumer HID</td></tr>
  <tr><td>9</td><td>File Explorer</td><td>Win+E</td></tr>
  <tr><td>10</td><td>Screenshot</td><td>Win+Shift+S</td></tr>
  <tr><td>11</td><td>Lock PC</td><td>Win+L</td></tr>
  <tr><td>12</td><td>Mute</td><td>Consumer HID</td></tr>
  <tr><td>13</td><td>Copy</td><td>Ctrl+C</td></tr>
  <tr><td>14</td><td>Play / Pause</td><td>Consumer HID</td></tr>
  <tr><td>15</td><td>Paste</td><td>Ctrl+V</td></tr>
  <tr><td>16</td><td>Show desktop</td><td>Win+D</td></tr>
</table>
</div>
</body></html>)";

  webServer.send(200, "text/html", html);
}

static void taskWiFi(void*) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) webServer.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ═══════════════════════════════════════════════════════════════
//  BLE SERVER CALLBACKS
// ═══════════════════════════════════════════════════════════════
class BLECallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleConnected = true;
    Serial.println("[BLE] Host connected");
  }
  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
    Serial.println("[BLE] Disconnected — re-advertising...");
    vTaskDelay(pdMS_TO_TICKS(500));
    s->startAdvertising();
  }
};

// ═══════════════════════════════════════════════════════════════
//  BLE INIT
// ═══════════════════════════════════════════════════════════════
static void startBLE() {
  BLEDevice::init("ESP32 MacroPad");
  BLEDevice::setMTU(64);

  // Security — Just Works bonding, no PIN required
  BLESecurity* sec = new BLESecurity();
  if (sec) {
    sec->setAuthenticationMode(ESP_LE_AUTH_BOND);
    sec->setCapability(ESP_IO_CAP_NONE);
    sec->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    sec->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  }

  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BLECallbacks());

  hid           = new BLEHIDDevice(pServer);
  inputKeyboard = hid->inputReport(REPORT_ID_KEYBOARD);
  inputMouse    = hid->inputReport(REPORT_ID_MOUSE);
  inputConsumer = hid->inputReport(REPORT_ID_CONSUMER);

  hid->manufacturer()->setValue("ESP32Build");
  hid->pnp(0x02, 0x05AC, 0x820A, 0x0210);
  hid->hidInfo(0x00, 0x01);
  hid->reportMap((uint8_t*)hidDescriptor, sizeof(hidDescriptor));
  hid->setBatteryLevel(100);
  hid->startServices();

  // Advertising
  BLEAdvertisementData advData;
  advData.setFlags(0x06);
  advData.setAppearance(HID_KEYBOARD);
  advData.setCompleteServices(hid->hidService()->getUUID());

  BLEAdvertisementData scanData;
  scanData.setName("ESP32 MacroPad");

  BLEAdvertising* adv = pServer->getAdvertising();
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  adv->start();

  Serial.println("[BLE] Advertising — pair 'ESP32 MacroPad' from your device");
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(2000);  // wait for serial monitor to connect

  Serial.println("[BOOT] ESP32-S3-N16R8 MacroPad starting...");
  Serial.printf( "[BOOT] Free heap : %d bytes\n", esp_get_free_heap_size());

  // ── GPIO ────────────────────────────────────────────────────
  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DT,  INPUT);
  pinMode(ENC_SW,  INPUT_PULLUP);
  encLastCLK = digitalRead(ENC_CLK);
  analogReadResolution(12);

  for (int r = 0; r < ROWS; r++) {
    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], HIGH);
  }
  for (int c = 0; c < COLS; c++) {
    pinMode(colPins[c], INPUT_PULLUP);
  }

  // ── FreeRTOS primitives ────────────────────────────────────
  actionQueue = xQueueCreate(32, sizeof(Action));
  bleMutex    = xSemaphoreCreateMutex();

  if (!actionQueue || !bleMutex) {
    Serial.println("[ERROR] Failed to create FreeRTOS objects — halting");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // ── BLE (before WiFi — reduces heap fragmentation) ─────────
  startBLE();
  Serial.printf("[BOOT] Heap after BLE : %d bytes\n", esp_get_free_heap_size());

  // ── WiFi (optional — BLE works without it) ──────────────────
  if (esp_get_free_heap_size() > 60000) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("[WiFi] Connecting");
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
      vTaskDelay(pdMS_TO_TICKS(500));
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n[WiFi] http://" + WiFi.localIP().toString());
      webServer.on("/", handleRoot);
      webServer.begin();
    } else {
      Serial.println("\n[WiFi] Failed — BLE only mode");
    }
  } else {
    Serial.println("[WiFi] Skipped — low heap");
  }

  Serial.printf("[BOOT] Heap before tasks : %d bytes\n", esp_get_free_heap_size());

  // ── Spawn FreeRTOS tasks ───────────────────────────────────
  //                               name      stack    arg  pri  handle  core
  xTaskCreatePinnedToCore(taskBLE,   "BLE",  12288, NULL,  4, NULL,    1);
  xTaskCreatePinnedToCore(taskInput, "INP",   4096, NULL,  3, NULL,    1);
  xTaskCreatePinnedToCore(taskPots,  "POT",   3072, NULL,  2, NULL,    0);
  xTaskCreatePinnedToCore(taskWiFi,  "WIF",   8192, NULL,  1, NULL,    0);

  Serial.println("[BOOT] All tasks started");
}

// ── loop() is intentionally empty ────────────────────────────
// All work is done in FreeRTOS tasks.
// Deleting the loop task frees ~8 KB of stack.
void loop() {
  vTaskDelete(NULL);
}
