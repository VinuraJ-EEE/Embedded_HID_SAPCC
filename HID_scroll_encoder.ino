// scroll using rotary encoder

#include "USB.h"
#include "USBHIDMouse.h"

USBHIDMouse Mouse;

// encoder pins
#define CLK 4
#define DT 5

int lastStateCLK;

void setup() {
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);

  lastStateCLK = digitalRead(CLK);

  USB.begin();
  Mouse.begin();

  delay(2000);
}

void loop() {
  int currentCLK = digitalRead(CLK);

  // detect rotation
  if (currentCLK != lastStateCLK) {

    if (digitalRead(DT) != currentCLK) {
      // clockwise → scroll up
      Mouse.move(0, 0, 1);
    } else {
      // counterclockwise → scroll down
      Mouse.move(0, 0, -1);
    }

    delay(2); // debounce
  }

  lastStateCLK = currentCLK;
}
