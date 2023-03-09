#include <Drawbotic_DB1.h>
#include <Drawbotic_Navigation.h>

DB1 bot;
Drawbotic_Navigation nav(&bot);

int last_ms = 0;

void setup() {
  bot.init();
  CreatePolygon(50, 8);
}

void loop() {
  int current_ms = millis();
  int deltaTime_ms = current_ms - last_ms;

  nav.update(deltaTime_ms);

  last_ms = current_ms;
}

void CreatePolygon(int sideLength, int numSides) {
  for (int i = 0; i < numSides; i++) {
    nav.addActionBack(nav.makeForwardAction(sideLength));
    nav.addActionBack(nav.makeRotateAction(360.0f / sideNum));
  }
}
