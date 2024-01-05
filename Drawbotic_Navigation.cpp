#include "Drawbotic_Navigation.h"

Drawbotic_Navigation::Drawbotic_Navigation(bool useIMU, float updateRate_ms, float speed, float correctionPower) {
  m_timeBank = 0;
  m_updateRate_ms = updateRate_ms;
  m_queueSize = 0;
  m_speed = speed;
  m_cp = correctionPower;
  m_useIMU = useIMU;
}

void Drawbotic_Navigation::clearAllActions() {
  // Iterate through the queue until there is nothing next
  while (m_queueHead != NULL)
  {
    NavigationAction *a = m_queueHead;
    m_queueHead = a->next;
    delete a; // Delete each action object
  }
  // Reset the queue state
  m_queueSize = 0;
  m_queueHead = NULL;
  m_queueTail = NULL;
}

void Drawbotic_Navigation::addForwardAction(float distance_mm, bool front) {
  NavigationAction* a = makeForwardAction(distance_mm);
  if(front) {
    addActionFront(a);
  }
  else {
    addActionBack(a);
  }
}

void Drawbotic_Navigation::addTurnAction(float radius_mm, float angle_deg, bool front) {
  NavigationAction* a = makeTurnAction(radius_mm, angle_deg);
  if(front) {
    addActionFront(a);
  }
  else {
    addActionBack(a);
  }
}

void Drawbotic_Navigation::addRotateAction(float angle_deg, bool front) {
  NavigationAction* a = makeRotateAction(angle_deg);
  if(front) {
    addActionFront(a);
  }
  else {
    addActionBack(a);
  }
}

void Drawbotic_Navigation::addStopAction(float time_ms, bool front) {
  NavigationAction* a = makeStopAction(time_ms);
  if(front) {
    addActionFront(a);
  }
  else {
    addActionBack(a);
  }
}

void Drawbotic_Navigation::addPenAction(bool down, bool front) {
  NavigationAction* a = makePenAction(down);
  if(front) {
    addActionFront(a);
  }
  else {
    addActionBack(a);
  }
}

void Drawbotic_Navigation::update(float deltaTime_ms) {
  m_timeBank += deltaTime_ms;

  // if enough time has passed for another update
  if (m_timeBank >= m_updateRate_ms) {
    // assign the action at the head of the queue as the current action
    NavigationAction *currentAction = m_queueHead;
    if (currentAction != NULL) {
      // perform a step of the current action and check if the action has finished
      bool finished = false;
      switch (currentAction->type) {
      case NAV_FORWARD:
        finished = driveForward(currentAction);
        break;
      case NAV_TURN:
          finished = turn(currentAction);
        break;
      case NAV_ROTATE:
        if(m_useIMU)
          finished = rotateIMU(currentAction);
        else
          finished = rotateEnc(currentAction);
        break;
      case NAV_STOP:
        finished = stop(currentAction, deltaTime_ms);
        break;
      case NAV_PEN_UP:
        finished = penUp();
        break;
      case NAV_PEN_DOWN:
        finished = penDown();
        break;
      }

      if (finished) {
        // Move to the next item in the queue
        m_queueHead = currentAction->next;
        delete currentAction;
        m_queueSize--;

        if(m_queueHead) {
          if(m_queueHead->type == NAV_ROTATE && m_useIMU) {
            setupRotationAction(m_queueHead);
          }
        }

        //reset encoder deltas to play nice with next action
        DB1.resetEncoderDeltas();

        // if the queue is empty, stop the robot
        if (m_queueSize == 0) {
          DB1.setMotorSpeed(1, 0);
          DB1.setMotorSpeed(2, 0);
          m_queueHead = NULL;
          m_queueTail = NULL;
        }
      }
    }
    m_timeBank = 0; // reset the time bank for the next update
  }
}

void Drawbotic_Navigation::setSpeed(float speed) {
  if (speed > 0 && speed < 1)
    m_speed = speed;
}

Drawbotic_Navigation::NavigationAction *Drawbotic_Navigation::makeForwardAction(float distance_mm) {
  NavigationAction *newAction = new NavigationAction();
  newAction->type = NAV_FORWARD;
  newAction->desiredDistance = distance_mm;
  newAction->followSpeed = m_speed;
  newAction->progress = 0;

  return newAction;
}

Drawbotic_Navigation::NavigationAction *Drawbotic_Navigation::makeTurnAction(float radius_mm, float angle_deg) {
  NavigationAction *newAction = new NavigationAction();
  newAction->type = NAV_TURN;
  newAction->desiredRadius = radius_mm;
  newAction->followSpeed = m_speed;
  newAction->progress = 0;
  if(m_useIMU) {
    //newAction->initialAngle = DB1.getOrientation().heading;
    newAction->desiredAngle = angle_deg;
  } 
  else {
    newAction->desiredAngle = angle_deg;
  }

  return newAction;
}

Drawbotic_Navigation::NavigationAction *Drawbotic_Navigation::makeRotateAction(float angle_deg) {
  NavigationAction *newAction = new NavigationAction();
  newAction->type = NAV_ROTATE;
  newAction->followSpeed = m_speed;
  newAction->progress = 0;
  newAction->desiredAngle = angle_deg;
  
  if(m_useIMU) 
    setupRotationAction(newAction);

  return newAction;
}

Drawbotic_Navigation::NavigationAction *Drawbotic_Navigation::makeStopAction(float time_ms) {
  NavigationAction *newAction = new NavigationAction();
  newAction->type = NAV_STOP;
  newAction->desiredTime = time_ms;
  newAction->followSpeed = m_speed;
  newAction->progress = 0;

  return newAction;
}

Drawbotic_Navigation::NavigationAction *Drawbotic_Navigation::makePenAction(bool down) {
  NavigationAction *newAction = new NavigationAction();
  newAction->type = down ? NAV_PEN_DOWN : NAV_PEN_UP;
  newAction->followSpeed = m_speed;
  newAction->progress = 0;

  return newAction;
}

void Drawbotic_Navigation::setupRotationAction(NavigationAction* action) {
  float currentAngle = DB1.getOrientation().heading;
  action->finalAngle = currentAngle + action->desiredAngle;
  if(action->finalAngle >= 360.0f) {
    action->finalAngle -= 360.0f;
  }
  else if(action->finalAngle < 0) {
    action->finalAngle += 360.0f;
  }
}

void Drawbotic_Navigation::addActionFront(NavigationAction *action) {
  // The previous node of the current head is the new node
  m_queueHead->prev = action;

  // The previous node of the new node is nothing (it's at the head of the queue)
  action->prev = NULL;
  // The next node of the new node is the current head node
  action->next = m_queueHead;
  // Update the head node to point to the new node
  m_queueHead = action;

  // If the queue is empty the new node is also the current tail node
  if (m_queueSize == 0)
    m_queueTail = action;

  // Update the size of the queue
  m_queueSize++;
}

void Drawbotic_Navigation::addActionBack(NavigationAction *action) {
  // The next node of the current tail is the new node
  m_queueTail->next = action;

  // The previous node of the new node is the current tail
  action->prev = m_queueTail;
  // The next node of the new node is nothing (it's at the end of the queue)
  action->next = NULL;
  // Update the tail node to point to the new node
  m_queueTail = action;

  // If the queue is empty the new node is also the current head node
  if (m_queueSize == 0)
    m_queueHead = action;

  // Update the size of the queue
  m_queueSize++;
}

bool Drawbotic_Navigation::driveForward(NavigationAction *action) {
  float encoderCount = action->progress;
  float dist = action->desiredDistance;

  //float circum = 2 * WHEEL_RADIUS * M_PI;

  // calculating how many encoder signals are equivalent to the input travel distance
  float encLimit = (dist * ENC_BITS_P_MM * FORWARD_ERROR);//ENC_BITS_P_REV * FORWARD_ERROR) / circum;

  int encError = encLimit - encoderCount;
  float power = constrain(m_speed * (encError * FORWARD_KP), SPEED_MIN, m_speed);

  if (encError > 0) {
    // finding the difference between the two encoders since last read
    float d1 = DB1.getM1EncoderDelta();
    float d2 = DB1.getM2EncoderDelta();
    float error = d1 - d2;
    // calculating the second motor speed based on the error value and the correction "strength" factor
    action->followSpeed += (error * m_cp);

    DB1.setMotorSpeed(1, power);
    DB1.setMotorSpeed(2, action->followSpeed);
    action->progress = encoderCount + d1;
    return false; // not finished
  }
  return true; // finished
}

bool Drawbotic_Navigation::turn(NavigationAction *action)
{
  float angleCount = action->progress;
  float arcRad = action->desiredRadius;
  float angle = action->desiredAngle;

  // left turn
  if (angle >= 0) {
    float r1 = arcRad + BOT_RADIUS;
    float r2 = arcRad - BOT_RADIUS;

    // how much slower the inside wheel needs to spin to match the angular velocity of the outside wheel
    float multiplier = r2 / r1;

    if (angleCount < angle)
    {
      // finding the difference between the two encoders since last read
      float d1 = DB1.getM1EncoderDelta();
      float d2 = DB1.getM2EncoderDelta();
      float error = d1 - (d2 / multiplier);

      // calculating the second motor speed based on the error value and the correction "strength" factor
      action->followSpeed += (error * m_cp);

      DB1.setMotorSpeed(1, m_speed);
      DB1.setMotorSpeed(2, action->followSpeed * multiplier);
      // calculating progress in degrees based on encoder signals
      action->progress = angleCount + (180 * d1) / (M_PI * ENC_BITS_P_MM * r1 * L_TURN_ERROR);

      return false;
    }
  }
  else if (angle < 0) { // right turn
    float r1 = arcRad - BOT_RADIUS;
    float r2 = arcRad + BOT_RADIUS;

    // how much slower the inside wheel needs to spin to match the angular velocity of the outside wheel
    float multiplier = r1 / r2;

    // angle is negative so the absolute value is taken
    if (angleCount < abs(angle)) {
      // finding the difference between the two encoders since last read
      float d1 = DB1.getM1EncoderDelta();
      float d2 = DB1.getM2EncoderDelta();
      float error = d2 - (d1 / multiplier);

      // calculating the first motor speed based on the error value and the correction "strength" factor
      action->followSpeed += (error * m_cp);

      DB1.setMotorSpeed(1, action->followSpeed * multiplier);
      DB1.setMotorSpeed(2, m_speed);
      // calculating progress in degrees based on encoder signals
      action->progress = angleCount + (180 * d2) / (M_PI * ENC_BITS_P_MM * r2 * R_TURN_ERROR);
      return false;
    }
  }
  return true;
}

bool Drawbotic_Navigation::rotateEnc(NavigationAction *action) {
  float angleCount = action->progress;
  float angle = action->desiredAngle;

  // anti-clockwise rotation
  if (angle >= 0) {
    if (angleCount < angle) {
      // finding the difference between the two encoders since last read
      float d1 = DB1.getM1EncoderDelta();
      float d2 = DB1.getM2EncoderDelta();

      // the absolute values are taken as negative values are expected
      d1 = abs(d1);
      d2 = abs(d2);
      float error = d1 - d2;

      // calculating the second motor speed based on the error value and the correction "strength" factor
      action->followSpeed += (error * m_cp);
      DB1.setMotorSpeed(1, m_speed);
      DB1.setMotorSpeed(2, -action->followSpeed); // motor 2 needs to spin in the opposite direction
      
      // calculating progress in degrees based on encoder signals
      action->progress = angleCount + (180 * d1) / (M_PI * ENC_BITS_P_MM * BOT_RADIUS * L_ROTATE_ERROR);
      return false;
    }
  }
  else if (angle < 0) { // clockwise rotation
    // angle is negative so the absolute value is taken
    if (angleCount < abs(angle)) {
      // finding the difference between the two encoders since last read
      float d1 = DB1.getM1EncoderDelta();
      float d2 = DB1.getM2EncoderDelta();

      // the absolute values are taken as negative values are expected
      d1 = abs(d1);
      d2 = abs(d2);
      float error = d2 - d1;

      // calculating the first motor speed based on the error value and the correction "strength" factor
      action->followSpeed += (error * m_cp);

      DB1.setMotorSpeed(1, -action->followSpeed); // motor 1 needs to spin in the opposite direction
      DB1.setMotorSpeed(2, m_speed);

      // calculating progress in degrees based on encoder signals
      action->progress = angleCount + (180 * d2) / (M_PI * ENC_BITS_P_MM * BOT_RADIUS * R_ROTATE_ERROR);
      return false;
    }
  }
  return true;
}

bool Drawbotic_Navigation::rotateIMU(NavigationAction *action) {
  float currentAngle = DB1.getOrientation().heading;
  
  //Calculate amount of degrees remaining
  float error = (int)((currentAngle - action->finalAngle + 540) * 1000) % 360000 - 180000;
  error /= 1000;

  //motor power is speed, corrected by a proportion defined by the error (directionless)
  float power = constrain(m_speed * (ROTATE_KP * abs(error)), SPEED_MIN, m_speed);

  //If we've reached the IMU tolerance threshold then we are finished
  if(error > -IMU_TOLERANCE && error < IMU_TOLERANCE) {
    return true;
  }
  else if(error > 0) {
    //rotate clockwise if positive error
    DB1.setMotorSpeed(1, -power);
    DB1.setMotorSpeed(2, power);
  }
  else {
    //rotate counter clockwise if negative error
    DB1.setMotorSpeed(1, power);
    DB1.setMotorSpeed(2, -power);
  }

  return false;
}

bool Drawbotic_Navigation::stop(NavigationAction *action, float deltaTime_ms) {
  float stopDuration = action->desiredTime;

  if (action->progress < stopDuration) {
    DB1.setMotorSpeed(1, 0);
    DB1.setMotorSpeed(2, 0);
    action->progress += deltaTime_ms;
    return false;
  }
  return true;
}

bool Drawbotic_Navigation::penUp() {
  DB1.setPen(false);
  return true;
}

bool Drawbotic_Navigation::penDown() {
  DB1.setPen(true);
  return true;
}