// ── 4×4 Matrix ──────────────────────────────────────────────
const int ROWS = 4;
const int COLS = 4;

int rowPins[ROWS] = {38, 39, 40, 41};
int colPins[COLS] = {42,  8, 17, 18};

// Key numbers 1–16, left to right, top to bottom
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

// ── Equation insertion helper ────────────────────────────────
void insertEq(const char* cmd) {
  Keyboard.press(KEY_LEFT_ALT);
  Keyboard.press('=');
  delay(150);
  Keyboard.releaseAll();
  delay(400);
  Keyboard.print(cmd);
  delay(80);
  Keyboard.write(' ');
  delay(100);
}

// ── Key actions ──────────────────────────────────────────────
void runAction(int key) {
  switch (key) {
    case  1: insertEq("\\int");                          break; // ∫
    case  2: insertEq("\\int_a^b");                     break; // ∫ₐᵇ
    case  3: insertEq("\\iint");                         break; // ∬
    case  4: insertEq("\\iiint");                        break; // ∭
    case  5: insertEq("\\frac");                         break; // □/□
    case  6: insertEq("\\sqrt");                         break; // √□
    case  7: insertEq("\\sqrt(n&x)");                   break; // ⁿ√□
    case  8: insertEq("\\frac{\\partial}{\\partial x}"); break; // ∂/∂x
    case  9: insertEq("\\frac{d}{dx}");                 break; // d/dx
    case 10: insertEq("\\sum_{n=1}^{\\infty}");         break; // Σ
    case 11: insertEq("\\prod_{n=1}^{N}");              break; // Π
    case 12: insertEq("\\lim_{x\\to\\infty}");          break; // lim
    case 13: insertEq("\\matrix(a&b@c&d)");             break; // matrix
    case 14: insertEq("\\infty");                        break; // ∞
    case 15: insertEq("\\pi");                           break; // π
    case 16: insertEq("\\vec");                          break; // □⃗
  }
}

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
        if (pressed && !prevState[r][c]) {
          runAction(keys[r][c]);
        }
        prevState[r][c] = pressed;
      }
    }

    digitalWrite(rowPins[r], HIGH);
  }
}
