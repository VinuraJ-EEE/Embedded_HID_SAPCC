// ── Key Actions ────────────────────────────────────────────
// Button 1 opens equation editor
// Other buttons directly insert symbols/templates

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
