#include <Drawbotic_DB1.h>
#include <Drawbotic_Navigation.h>

//Create the navigation system and pass a reference to the DB1 to it
Drawbotic_Navigation nav;

//Previous timestamp, used to calculate the time delta
int last_ms = 0;

void setup() {
  //Initalise the DB1
  DB1.init();
  //Lower the pen
  DB1.setPen(true);
  //wait for pen to be completely down
  delay(1000);

  //create an action queue to draw a polygon
  createPolygon(50, 8);
}

void createPolygon(int sideLength, int sideNum) {
  //Loop for the desired number of sides
  for (int i = 0; i < sideNum; i++) {
    //a side is made up of a forward action which is the length of the side...
    nav.addForwardAction(sideLength);
    //... and a rotate action with an angle that is 360 / the number of desired sides degrees
    nav.addRotateAction(360.0f / sideNum);
  }
}

void loop() {
  //Calculate the delta time (i.e. the elapsed since the last time loop ran)
  //The calculation is the current time - the time is was last update
  int current_ms = millis();
  int deltaTime_ms = current_ms - last_ms;

  //If the navigation queue is empty then we've finished drawing our polygon
  if(nav.getQueueSize() == 0) {
    //Lift the pen up...
    DB1.setPen(false);
    while(1) {
      //... and sleep forever
      delay(10000);
    }
  }
  else {
    //If there are still actions in the queue then we need to update the Navigation system to process them
    nav.update(deltaTime_ms);
  }

  //Remember the current time as the last time for the next cycle
  last_ms = current_ms;
}
