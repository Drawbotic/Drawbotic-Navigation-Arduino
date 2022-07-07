#include "Drawbotic_Navigation.h"

Drawbotic_Navigation::Drawbotic_Navigation(DB1* bot, float updateRate_ms, float speed, float correctionPower, float accelR)
{
    m_bot = bot;
    m_timebank = 0;
    m_updateRate_ms = updateRate_ms;
    m_queueSize = 0;
    m_speed = speed;
    m_cp = correctionPower;
    m_accelR = accelR;
}

void Drawbotic_Navigation::ClearAllActions()
{
    //Iterate through the queue until there is nothing next
    while(m_queueHead != NULL)
    {
        NavigationAction* a = m_queueHead;
        m_queueHead = a->next;
        delete a;   //Delete each action object
    }
    //Reset the queue state
    m_queueSize = 0;
    m_queueHead = NULL;
    m_queueTail = NULL;
}
void Drawbotic_Navigation::AddActionFront(NavigationAction* action)
{
    action->nextType = m_queueHead->type;
    action->prevType = NAV_UNKNOWN;
    m_queueHead->prevType = action->type;

    //The previous node of the current head is the new node
    m_queueHead->prev = action;
    
    //The previous node of the new node is nothing (it's at the head of the queue)
    action->prev = NULL;
    //The next node of the new node is the current head node
    action->next = m_queueHead;
    //Update the head node to point to the new node
    m_queueHead = action;

    //If the queue is empty the new node is also the current tail node
    if(m_queueSize == 0)
        m_queueTail = action;
    
    //Update the size of the queue
    m_queueSize++;
}

void Drawbotic_Navigation::AddActionBack(NavigationAction* action)
{
    action->nextType = NAV_UNKNOWN;
    action->prevType = m_queueTail->type;
    m_queueTail->nextType = action->type;

    //The next node of the current tail is the new node
    m_queueTail->next = action;

    //The previous node of the new node is the current tail
    action->prev = m_queueTail;
    //The next node of the new node is nothing (it's at the end of the queue)
    action->next = NULL;
    //Update the tail node to point to the new node
    m_queueTail = action;

    //If the queue is empty the new node is also the current head node
    if(m_queueSize == 0)
        m_queueHead = action;

    //Update the size of the queue
    m_queueSize++;
}

NavigationAction* Drawbotic_Navigation::MakeForwardAction(float distance_mm)
{
    NavigationAction* newAction = new NavigationAction();
    newAction->type = NAV_FORWARD;
    //Fill in the rest
    newAction->params[0] = distance_mm;
    newAction->params[1] = 0;
    newAction->params[2] = 0;
    newAction->followSpeed = m_speed;
    newAction->progress = 0;



    return newAction;
}

NavigationAction* Drawbotic_Navigation::MakeTurnAction(float radius_mm, float angle_deg)
{
    NavigationAction* newAction = new NavigationAction();
    newAction->type = NAV_TURN;
    //Fill in the rest
    newAction->params[0] = radius_mm;
    newAction->params[1] = angle_deg;
    newAction->params[2] = 0;
    newAction->followSpeed = m_speed;
    newAction->progress = 0;

    return newAction;
}

NavigationAction* Drawbotic_Navigation::MakeRotateAction(float angle_deg)
{
    NavigationAction* newAction = new NavigationAction();
    newAction->type = NAV_ROTATE;
    //Fill in the rest
    newAction->params[0] = angle_deg;
    newAction->params[1] = 0;
    newAction->params[2] = 0;
    newAction->followSpeed = m_speed;
    newAction->progress = 0;

    return newAction;
}

NavigationAction* Drawbotic_Navigation::MakeStopAction(float time_ms)
{
    NavigationAction* newAction = new NavigationAction();
    newAction->type = NAV_STOP;
    //Fill in the rest
    newAction->params[0] = time_ms;
    newAction->params[1] = 0;
    newAction->params[2] = 0;
    newAction->followSpeed = m_speed;
    newAction->progress = 0;

    return newAction;
}

NavigationAction* Drawbotic_Navigation::MakePenAction(bool up)
{
    NavigationAction* newAction = new NavigationAction();
    newAction->type = up ? NAV_PEN_UP : NAV_PEN_DOWN;
    //Fill in the rest
    newAction->params[0] = 0;
    newAction->params[1] = 0;
    newAction->params[2] = 0;
    newAction->followSpeed = m_speed;
    newAction->progress = 0;

    return newAction;
}

void Drawbotic_Navigation::Update(float deltaTime_ms)
{
    m_timebank += deltaTime_ms;

    if(m_timebank >= m_updateRate_ms)
    {
        NavigationAction* currentAction = m_queueHead;

        if(currentAction != NULL)
        {
            bool finished = false;
            switch(currentAction->type)
            {
            case NAV_FORWARD:
                finished = DriveForward(currentAction);
                break;
            case NAV_TURN:
                finished = Turn(currentAction);
                break;
            case NAV_ROTATE:
                finished = Rotate(currentAction);
                break;
            case NAV_STOP:
                finished = Stop(currentAction, deltaTime_ms);
                break;
            case NAV_PEN_UP:
                finished = PenUp(currentAction);
                break;
            case NAV_PEN_DOWN:
                finished = PenDown(currentAction);
                break;
            }

            if(finished)
            {
                //Move to the next item in the queue
                m_queueHead = currentAction->next;
                delete currentAction;
                m_queueSize--;

                if (m_queueSize == 0) 
                {
                    m_bot->SetMotorSpeed(1, 0);
                    m_bot->SetMotorSpeed(2, 0);
                }

            }
        }
        m_timebank = 0; //reset the time bank for the next update
        
    }
}


void Drawbotic_Navigation::SetSpeed(float speed)
{
    if (speed > 0 && speed < 1)
        m_speed = speed;
    
}


void Drawbotic_Navigation::SetAccelRatio(float accelR)
{
    if (accelR >= 0 && accelR < 0.5)
        m_accelR = accelR;
    
}






bool Drawbotic_Navigation::DriveForward(NavigationAction* action)
{
  float encoderCount = action->progress;
  float dist = action->params[0];
  NavigationType nextType = action->nextType;
  NavigationType prevType = action->prevType;
  float circum = 2*WHEEL_RADIUS*M_PI; 
  float encLimit = (dist*ENCODER_BITS)/circum;

  if  (encoderCount < encLimit) 
  {
    //Find the difference between the two encoders since last read
    float d1 = m_bot->GetM1EncoderDelta();
    float d2 = m_bot->GetM2EncoderDelta();
    float error = d1 - d2; 
    //Calculate the second motor speed based on the error value and the correction "strength" factor
    action->followSpeed += (error * m_cp);



    // // accel
    // if (prevType == NAV_STOP || prevType == NAV_ROTATE) // prev is stop or rotate
    // { 
    //     Serial.println("PASSED CHECK 1");
    //     Serial.print("encoder count: ");
    //     Serial.println(encoderCount);
    //     Serial.print("m_accelR: ");
    //     Serial.println(m_accelR);
    //     Serial.print("encLimit: ");
    //     Serial.println(encLimit);
    //     if (encoderCount < (m_accelR*encLimit)) 
    //     { 
    //         Serial.print("ACCELERATING: ");
    //         Serial.println(m_speed - m_speed*(((m_accelR*encLimit) - encoderCount)/(m_accelR*encLimit)));
    //         float speed1 =  m_speed - m_speed*(((m_accelR*encLimit) - encoderCount)/(m_accelR*encLimit));
    //         float speed2 = action->followSpeed - action->followSpeed*(((m_accelR*encLimit) - encoderCount)/(m_accelR*encLimit));
            

    //         if (speed1 < 0.04)
    //         {
    //             speed1 = 0.05;
    //             speed2 = 0.05;
    //         }
            
    //         m_bot->SetMotorSpeed(1, speed1);
    //         m_bot->SetMotorSpeed(2, speed2);
    //         action->progress = encoderCount + d1;
    //         return false;
    //     }
    // }

    // // decel
    // if (nextType == NAV_STOP || nextType == NAV_ROTATE) // next is stop or rotate
    // { 
    //     //Serial.println("PASSED CHECK 1");
    //     // Serial.print("encoder count: ");
    //     // Serial.println(encoderCount);
    //     // Serial.print("m_accelR: ");
    //     // Serial.println(m_accelR);
    //     // Serial.print("encLimit: ");
    //     // Serial.println(encLimit);
    //     if (encoderCount > (encLimit - m_accelR*encLimit)) 
    //     { 
    //         Serial.print("DECELERATING");
    //         Serial.println(m_speed - m_speed*((encLimit - encoderCount)/(m_accelR*encLimit)));
    //         float speed1 = m_speed - m_speed*((encLimit - encoderCount)/(m_accelR*encLimit));
    //         float speed2 = action->followSpeed - action->followSpeed*((encLimit - encoderCount)/(m_accelR*encLimit));

    //         if (speed1 < 0.05)
    //         {
    //             speed1 = 0.04;
    //             speed2 = 0.04;
    //         }


    //         m_bot->SetMotorSpeed(1, speed1);
    //         m_bot->SetMotorSpeed(2, speed2);
    //         action->progress = encoderCount + d1;
    //         return false;
    //     }
    // }

    m_bot->SetMotorSpeed(1, m_speed);
    m_bot->SetMotorSpeed(2, action->followSpeed);
    action->progress = encoderCount + d1;
    return false;
  }
  return true;
}

bool Drawbotic_Navigation::Turn(NavigationAction* action)
{
  float angleCount = action->progress;
  float arcRad = action->params[0];
  float angle = action->params[1];

  NavigationType nextType = action->nextType;
  NavigationType prevType = action->prevType;

  // left turn
  if (angle >= 0) 
  {
    float r1 = arcRad + BOT_RADIUS; 
    float r2 = arcRad - BOT_RADIUS;
    float multiplier = r2/r1;

    if (angleCount < angle) 
    {
      float d1 = m_bot->GetM1EncoderDelta();
      float d2 = m_bot->GetM2EncoderDelta(); 
      float error = d1 - (d2/multiplier); //(d1/d2) - (1/multiplier)
      action->followSpeed += (error * m_cp);

    //   // accel
    //   if (prevType == NAV_STOP || prevType == NAV_ROTATE) { // prev is stop or rotate
    //       if (angleCount < m_accelR*angle) { 
    //         m_bot->SetMotorSpeed(1, m_speed - m_speed*((m_accelR*angle - angleCount)/(m_accelR*angle)));
    //         m_bot->SetMotorSpeed(2, multiplier*(action->followSpeed - action->followSpeed*((m_accelR*angle - angleCount)/(m_accelR*angle))));
    //         action->progress = angleCount + (180*d1)/(M_PI*TURN_FACTOR*r1); 
    //         return false;
    //       }
    //   }


    //   // decel
    //   if (nextType == NAV_STOP || nextType == NAV_ROTATE) { // next is stop or rotate
    //       if (angleCount > (angle - m_accelR*angle)) { 
    //         m_bot->SetMotorSpeed(1, m_speed - m_speed*((angle - angleCount)/(m_accelR*angle)));
    //         m_bot->SetMotorSpeed(2, multiplier*(action->followSpeed - action->followSpeed*((angle - angleCount)/(m_accelR*angle))));
    //         action->progress = angleCount + (180*d1)/(M_PI*TURN_FACTOR*r1); 
    //         return false;
    //       }
    //   }

      m_bot->SetMotorSpeed(1, m_speed);
      m_bot->SetMotorSpeed(2, action->followSpeed * multiplier);  // = follow speed, then multiply
      action->progress = angleCount + (180*d1)/(M_PI*TURN_FACTOR*r1); 
      return false;
    }
  }

  // right turn
  else if (angle < 0) 
  {
    float r1 = arcRad - BOT_RADIUS; // 60 = dist from centre of wheel to centre of pen mm
    float r2 = arcRad + BOT_RADIUS;
    float multiplier = r1/r2;

    if (angleCount < abs(angle)) 
    {
      float d1 = m_bot->GetM1EncoderDelta();
      float d2 = m_bot->GetM2EncoderDelta(); 
      float error = d2 - (d1/multiplier); //(d1/d2) - (1/multiplier)
      action->followSpeed += (error * m_cp);

    //   // accel
    //   if (prevType == NAV_STOP || prevType == NAV_ROTATE) { // prev is stop or rotate
    //       if (angleCount < m_accelR*abs(angle)) { 
    //         m_bot->SetMotorSpeed(1, m_speed - m_speed*((m_accelR*abs(angle) - angleCount)/(m_accelR*abs(angle))));
    //         m_bot->SetMotorSpeed(2, multiplier*(action->followSpeed - action->followSpeed*((m_accelR*abs(angle) - angleCount)/(m_accelR*abs(angle)))));
    //         action->progress = angleCount + (180*d1)/(M_PI*TURN_FACTOR*r1); 
    //         return false;
    //       }
    //   }


    //   // decel
    //   if (nextType == NAV_STOP  || nextType == NAV_ROTATE) { // next is stop or rotate
    //       if (angleCount > (abs(angle) - m_accelR*abs(angle))) {
    //         m_bot->SetMotorSpeed(1, multiplier*(action->followSpeed - action->followSpeed*((abs(angle) - angleCount)/(m_accelR*abs(angle))))); 
    //         m_bot->SetMotorSpeed(2, m_speed - m_speed*((abs(angle) - angleCount)/(m_accelR*abs(angle))));
    //         action->progress = angleCount + (180*d1)/(M_PI*TURN_FACTOR*r1); 
    //         return false;
    //       }
    //   }

      m_bot->SetMotorSpeed(1, action->followSpeed * multiplier);
      m_bot->SetMotorSpeed(2, m_speed);  // = follow speed, then multiply
      action->progress = angleCount + (180*d2)/(M_PI*TURN_FACTOR*r2); 
      return false;
    }
  }
  return true;
}

bool Drawbotic_Navigation::Rotate(NavigationAction* action)
{

  float angleCount = action->progress;
  float angle = action->params[0];
  NavigationType nextType = action->nextType;
  NavigationType prevType = action->prevType;

  // left turn
  if (angle >= 0) 
  { 
    if (angleCount < angle) 
    {
      float d1 = m_bot->GetM1EncoderDelta();
      float d2 = m_bot->GetM2EncoderDelta();
      d1 = abs(d1);
      d2 = abs(d2);
      float error = d1 - d2; 
      action->followSpeed += (error * m_cp);

    //   // accel
    //   if (prevType == NAV_STOP || prevType == NAV_ROTATE) { // prev is stop or rotate
    //       if (angleCount < m_accelR*angle) { 
    //         m_bot->SetMotorSpeed(1, m_speed - m_speed*((m_accelR*angle - angleCount)/(m_accelR*angle)));
    //         m_bot->SetMotorSpeed(2, -(action->followSpeed - action->followSpeed*((m_accelR*angle - angleCount)/(m_accelR*angle))));
    //         action->progress = angleCount + (180*d1)/(M_PI*ROTATE_FACTOR*BOT_RADIUS); 
    //         return false;
    //       }
    //   }

    //    // decel
    //   if (nextType == NAV_STOP || nextType == NAV_ROTATE) { // prev is stop or rotate
    //       if (angleCount > (angle - m_accelR*angle)) { 
    //         m_bot->SetMotorSpeed(1, m_speed - m_speed*((angle - angleCount)/(m_accelR*angle)));
    //         m_bot->SetMotorSpeed(2, -(action->followSpeed - action->followSpeed*((angle - angleCount)/(m_accelR*angle))));
    //         action->progress = angleCount + (180*d1)/(M_PI*ROTATE_FACTOR*BOT_RADIUS); 
    //         return false;
    //       }
    //   }

      m_bot->SetMotorSpeed(1, m_speed);
      m_bot->SetMotorSpeed(2, -action->followSpeed);  // = follow speed, then multiply
      action->progress = angleCount + (180*d1)/(M_PI*ROTATE_FACTOR*BOT_RADIUS); 
      return false;
    }
  }

  // right turn
  else if (angle < 0) 
  { 
    if (angleCount < abs(angle)) 
    {
      float d1 = m_bot->GetM1EncoderDelta();
      float d2 = m_bot->GetM2EncoderDelta(); 
      d1 = abs(d1);
      d2 = abs(d2);
      float error = d2 - d1; 
      action->followSpeed += (error * m_cp);

    //   // accel
    //   if (prevType == NAV_STOP || prevType == NAV_ROTATE) { // prev is stop or rotate
    //       if (angleCount < m_accelR*angle) { 
    //         m_bot->SetMotorSpeed(1, -(m_speed - m_speed*((m_accelR*angle - angleCount)/(m_accelR*angle))));
    //         m_bot->SetMotorSpeed(2, action->followSpeed - action->followSpeed*((m_accelR*angle - angleCount)/(m_accelR*angle)));
    //         action->progress = angleCount + (180*d1)/(M_PI*ROTATE_FACTOR*BOT_RADIUS); 
    //         return false;
    //       }
    //   }

    //    // decel
    //   if (nextType == NAV_STOP || nextType == NAV_ROTATE) { // prev is stop or rotate
    //       if (angleCount > (angle - m_accelR*angle)) { 
    //         m_bot->SetMotorSpeed(1, -(m_speed - m_speed*((angle - angleCount)/(m_accelR*angle))));
    //         m_bot->SetMotorSpeed(2, action->followSpeed - action->followSpeed*((angle - angleCount)/(m_accelR*angle)));
    //         action->progress = angleCount + (180*d1)/(M_PI*ROTATE_FACTOR*BOT_RADIUS); 
    //         return false;
    //       }
    //   }


      m_bot->SetMotorSpeed(1, -action->followSpeed);
      m_bot->SetMotorSpeed(2, m_speed);  // = follow speed, then multiply
      action->progress = angleCount + (180*d2)/(M_PI*ROTATE_FACTOR*BOT_RADIUS);  
      return false;
    }
  }
  return true;
}

bool Drawbotic_Navigation::Stop(NavigationAction* action, float deltaTime_ms)
{
    float t = action->progress;
    float stopDuration = action->params[0];

    if (t < stopDuration) {
        m_bot->SetMotorSpeed(1, 0);
        m_bot->SetMotorSpeed(2, 0);
        action->progress = t + deltaTime_ms; 
        return false;
    }
    return true;
}

bool Drawbotic_Navigation::PenUp(NavigationAction* action)
{
    m_bot->SetPenUp(true);
    return true;
}

bool Drawbotic_Navigation::PenDown(NavigationAction* action)
{
    m_bot->SetPenUp(false);
    return true;
}