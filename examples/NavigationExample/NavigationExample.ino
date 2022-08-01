#include <Drawbotic_DB1.h>
#include <Drawbotic_Navigation.h>

DB1 bot;
Drawbotic_Navigation nav(&bot);

int last_ms = 0;

void setup() {
    bot.Initialise();

    CreatePolygon(50, 8);
}

void loop() {
    // put your main code here, to run repeatedly:
  int current_ms = millis();
  int deltaTime_ms = current_ms - last_ms;

  nav.Update(deltaTime_ms);

  last_ms = current_ms;
}

void CreatePolygon(int sideLength, int sideNum) {
    for(int i = 0; i < sideNum; i++) {
        nav.AddActionBack(nav.MakeForwardAction(sideLength));
        nav.AddActionBack(nav.MakeRotateAction(360.0f / sideNum));
    }
}
