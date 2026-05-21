#include "USB.h"
#include "USBHIDKeyboard.h"
USBHIDKeyboard Keyboard;

// ── 4×4 Matrix Configuration ─────────────────────────────────
const byte ROWS = 4;
const byte COLS = 4;

// Remapped to safe, accessible ESP32-S3 DevKit pins
const byte rowPins[ROWS] = {4, 5, 6, 7};
const byte colPins[COLS] = {15, 8, 16, 17};

// Key numbers 1–16
const byte keys[ROWS][COLS] = {
  { 1,  2,  3,  4},
  { 5,  6,  7,  8},
  { 9, 10, 11, 12},
  {13, 14, 15, 16}
};

// ── Ultra-Lean Bitmask Debounce Variables ─────────────────────
byte debouncedState[ROWS] = {0};
byte activeHistory[ROWS]  = {0};
unsigned long lastScanTime = 0;

// ── Equation Insertion Helper ────────────────────────────────
void insertEq(const char* cmd) {
  // ESP32-S3 processes commands quickly, keeping delays stable for Word/Office
  Keyboard.press(KEY_LEFT_ALT);
  Keyboard.press('=');
  delay(100); 
  Keyboard.releaseAll();
  delay(250); 
  Keyboard.print(cmd);
  delay(50);
  Keyboard.write(' ');
  delay(50);
}

// ── Key Actions ──────────────────────────────────────────────
void runAction(byte key) {
  switch (key) {
    case  1: insertEq("\\int");                          break; // ∫
    case  2: insertEq("\\int_a^b");                      break; // ∫ₐᵇ
    case  3: insertEq("\\iint");                         break; // ∬
    case  4: insertEq("\\iiint");                        break; // ∭
    case  5: insertEq("\\frac");                         break; // □/□
    case  6: insertEq("\\sqrt");                         break; // √□
    case  7: insertEq("\\sqrt(n&x)");                    break; // ⁿ√□
    case  8: insertEq("\\frac{\\partial}{\\partial x}"); break; // ∂/∂x
    case  9: insertEq("\\frac{d}{dx}");                  break; // d/dx
    case 10: insertEq("\\sum_{n=1}^{\\infty}");          break; // Σ
    case 11: insertEq("\\prod_{n=1}^{N}");               break; // Π
    case 12: insertEq("\\lim_{x\\to\\infty}");           break; // lim
    case 13: insertEq("\\matrix(a&b@c&d)");              break; // matrix
    case 14: insertEq("\\infty");                        break; // ∞
    case 15: insertEq("\\pi");                           break; // π
    case 16: insertEq("\\vec");                          break; // □⃗
  }
}

// ── High-Efficiency Matrix Scan ───────────────────────────────
void scanMatrix() {
  // Polling threshold checks every 5 milliseconds
  if (millis() - lastScanTime < 5) return;
  lastScanTime = millis();

  for (byte r = 0; r < ROWS; r++) {
    digitalWrite(rowPins[r], LOW);
    delayMicroseconds(2); // Short stabilization wait for capacitive traces

    byte currentRawRowState = 0;
    for (byte c = 0; c < COLS; c++) {
      if (digitalRead(colPins[c]) == LOW) {
        currentRawRowState |= (1 << c);
      }
    }
    digitalWrite(rowPins[r], HIGH);

    // Bitwise transformation step
    byte changes = currentRawRowState ^ debouncedState[r];
    activeHistory[r] = (activeHistory[r] << 1) | (changes ? 1 : 0);
    
    // Process state changes if input signal remains stable
    if ((activeHistory[r] & 0x07) == 0) { 
      byte newPressedKeys = currentRawRowState & ~debouncedState[r];
      debouncedState[r] = currentRawRowState;

      for (byte c = 0; c < COLS; c++) {
        if (newPressedKeys & (1 << c)) {
          runAction(keys[r][c]);
        }
      }
    }
  }
}

// ── Arduino Initialization ───────────────────────────────────
void setup() {
  // Set up rows as active drive lines
  for (byte r = 0; r < ROWS; r++) {
    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], HIGH);
  }

  // Set up columns as inputs with internal pullups enabled
  for (byte c = 0; c < COLS; c++) {
    pinMode(colPins[c], INPUT_PULLUP);
  }

  // Fire up native ESP32-S3 USB architecture
  USB.begin();
  Keyboard.begin();
}

void loop() {
  scanMatrix();
}
