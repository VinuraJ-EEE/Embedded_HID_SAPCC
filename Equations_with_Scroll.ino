#include "USB.h"
#include "USBHIDMouse.h"
#include "USBHIDConsumerControl.h"
#include "USBHIDKeyboard.h"

USBHIDMouse Mouse;
USBHIDConsumerControl Consumer;
USBHIDKeyboard Keyboard;

// ── Rotary Encoder ─────────────────────────────────────────
#define CLK     4
#define DT      5
#define SW      6
#define POT_PIN 7

int  lastStateCLK;
int  lastVolume      = -1;
bool lastButtonState = HIGH;

// ── 4×4 Matrix Keypad ──────────────────────────────────────
const int ROWS = 4;
const int COLS = 4;

int rowPins[ROWS] = {38, 39, 40, 41};
int colPins[COLS] = {42, 8, 17, 18};

// Keys 1–16
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

// ── Helper Functions ───────────────────────────────────────

// Win + R launcher
void winRun(const char* cmd) {

  Keyboard.press(KEY_LEFT_GUI);
  Keyboard.press('r');

  delay(150);

  Keyboard.releaseAll();

  delay(400);

  Keyboard.print(cmd);

  delay(100);

  Keyboard.write(KEY_RETURN);

  delay(300);
}

// Keyboard shortcut helper
void combo(uint8_t mod1, uint8_t mod2, uint8_t key) {

  Keyboard.press(mod1);

  if (mod2)
    Keyboard.press(mod2);

  if (key)
    Keyboard.press(key);

  delay(100);

  Keyboard.releaseAll();
}

// ── Keypad Actions ─────────────────────────────────────────
void runAction(int key) {

  switch (key) {

    // ── Open Equation Editor ──
    case 1:

      Keyboard.press(KEY_LEFT_ALT);
      Keyboard.press('=');

      delay(150);

      Keyboard.releaseAll();

      break;

    // ── Integral ──
    case 2:
      Keyboard.print("\\int ");
      break;

    // ── Definite Integral ──
    case 3:
      Keyboard.print("\\int_a^b ");
      break;

    // ── Double Integral ──
    case 4:
      Keyboard.print("\\iint ");
      break;

    // ── Triple Integral ──
    case 5:
      Keyboard.print("\\iiint ");
      break;

    // ── Fraction ──
    case 6:
      Keyboard.print("\\frac ");
      break;

    // ── Square Root ──
    case 7:
      Keyboard.print("\\sqrt ");
      break;

    // ── nth Root ──
    case 8:
      Keyboard.print("\\sqrt(n&x) ");
      break;

    // ── Partial Derivative ──
    case 9:
      Keyboard.print("\\frac{\\partial}{\\partial x} ");
      break;

    // ── Ordinary Derivative ──
    case 10:
      Keyboard.print("\\frac{d}{dx} ");
      break;

    // ── Summation ──
    case 11:
      Keyboard.print("\\sum_{n=1}^{\\infty} ");
      break;

    // ── Product ──
    case 12:
      Keyboard.print("\\prod_{n=1}^{N} ");
      break;

    // ── Limit ──
    case 13:
      Keyboard.print("\\lim_{x\\to\\infty} ");
      break;

    // ── Matrix ──
    case 14:
      Keyboard.print("\\matrix(a&b@c&d) ");
      break;

    // ── Infinity ──
    case 15:
      Keyboard.print("\\infty ");
      break;

    // ── Pi ──
    case 16:
      Keyboard.print("\\pi ");
      break;
  }
}

// ── Matrix Scanner ─────────────────────────────────────────
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

        if (pressed && !prevState[r][c]) {

          runAction(keys[r][c]);
        }

        prevState[r][c] = pressed;
      }
    }

    digitalWrite(rowPins[r], HIGH);
  }
}

// ── Setup ──────────────────────────────────────────────────
void setup() {

  // Rotary encoder
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);

  lastStateCLK = digitalRead(CLK);

  // Potentiometer ADC
  analogReadResolution(12);

  // Matrix rows
  for (int r = 0; r < ROWS; r++) {

    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], HIGH);
  }

  // Matrix columns
  for (int c = 0; c < COLS; c++) {

    pinMode(colPins[c], INPUT_PULLUP);
  }

  // Start USB HID
  USB.begin();

  Mouse.begin();
  Consumer.begin();
  Keyboard.begin();

  delay(2000);
}

// ── Main Loop ──────────────────────────────────────────────
void loop() {

  // ── Encoder Scroll ─────────────────────────────
  int currentCLK = digitalRead(CLK);

  if (currentCLK != lastStateCLK) {

    if (digitalRead(DT) != currentCLK) {

      // Scroll up
      Mouse.move(0, 0, 1);

    } else {

      // Scroll down
      Mouse.move(0, 0, -1);
    }

    delay(2);
  }

  lastStateCLK = currentCLK;

  // ── Potentiometer Volume ───────────────────────
  int raw = analogRead(POT_PIN);

  int volume = map(raw, 0, 4095, 0, 100);

  if (abs(volume - lastVolume) > 2) {

    if (lastVolume == -1) {

      lastVolume = volume;

    } else if (volume > lastVolume) {

      for (int i = 0; i < (volume - lastVolume); i++) {

        Consumer.press(CONSUMER_CONTROL_VOLUME_INCREMENT);
        Consumer.release();

        delay(2);
      }

    } else {

      for (int i = 0; i < (lastVolume - volume); i++) {

        Consumer.press(CONSUMER_CONTROL_VOLUME_DECREMENT);
        Consumer.release();

        delay(2);
      }
    }

    lastVolume = volume;
  }

  // ── Encoder Button ─────────────────────────────
  bool currentButtonState = digitalRead(SW);

  if (lastButtonState == HIGH && currentButtonState == LOW) {

    // Open Device Manager
    winRun("devmgmt.msc");
  }

  lastButtonState = currentButtonState;

  // ── Scan Matrix ────────────────────────────────
  scanMatrix();

  delay(5);
}
