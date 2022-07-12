#include "Drawbotic_Navigation.h"

Drawbotic_Navigation::Drawbotic_Navigation(DB1* bot, float updateRate_ms, float speed, float correctionPower)
{
    m_bot = bot;
    m_timebank = 0;
    m_updateRate_ms = updateRate_ms;
    m_queueSize = 0;
    m_speed = speed;
    m_cp = correctionPower;
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
                    ClearAllActions(); 
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

bool Drawbotic_Navigation::DriveForward(NavigationAction* action)
{
  float encoderCount = action->progress;
  float dist = action->params[0];

  float circum = 2*WHEEL_RADIUS*M_PI; 
  float encLimit = (dist*ENC_BITS_P_REV*FORWARD_ERROR)/circum;

  if  (encoderCount < encLimit) 
  {
    //Find the difference between the two encoders since last read
    float d1 = m_bot->GetM1EncoderDelta();
    float d2 = m_bot->GetM2EncoderDelta();
    float error = d1 - d2; 
    //Calculate the second motor speed based on the error value and the correction "strength" factor
    action->followSpeed += (error * m_cp);

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

  // left turn
  if (angle >= 0) 
  {
    float r1 = arcRad + BOT_RADIUS; 
    float r2 = arcRad - BOT_RADIUS;
    float multiplier = r2/r1;

    if (angleCount < angle) 
    {
      //Find the difference between the two encoders since last read
      float d1 = m_bot->GetM1EncoderDelta();
      float d2 = m_bot->GetM2EncoderDelta(); 
      float error = d1 - (d2/multiplier); 
      //Calculate the second motor speed based on the error value and the correction "strength" factor
      action->followSpeed += (error * m_cp);

      m_bot->SetMotorSpeed(1, m_speed);
      m_bot->SetMotorSpeed(2, action->followSpeed * multiplier);  
      action->progress = angleCount + (180*d1)/(M_PI*ENC_BITS_P_MM*r1*TURN_ERROR); 
      return false;
    }
  }

  // right turn
  else if (angle < 0) 
  {
    float r1 = arcRad - BOT_RADIUS; 
    float r2 = arcRad + BOT_RADIUS;
    float multiplier = r1/r2;

    if (angleCount < abs(angle)) 
    {
      //Find the difference between the two encoders since last read
      float d1 = m_bot->GetM1EncoderDelta();
      float d2 = m_bot->GetM2EncoderDelta(); 
      float error = d2 - (d1/multiplier);
      //Calculate the second motor speed based on the error value and the correction "strength" factor
      action->followSpeed += (error * m_cp);

      m_bot->SetMotorSpeed(1, action->followSpeed * multiplier);
      m_bot->SetMotorSpeed(2, m_speed);  
      action->progress = angleCount + (180*d2)/(M_PI*ENC_BITS_P_MM*r2*TURN_ERROR); 
      return false;
    }
  }
  return true;
}

bool Drawbotic_Navigation::Rotate(NavigationAction* action)
{

  float angleCount = action->progress;
  float angle = action->params[0];

  // left rotation
  if (angle >= 0) 
  { 
    if (angleCount < angle) 
    {
      //Find the difference between the two encoders since last read
      float d1 = m_bot->GetM1EncoderDelta();
      float d2 = m_bot->GetM2EncoderDelta();
      d1 = abs(d1);
      d2 = abs(d2);
      float error = d1 - d2; 
      //Calculate the second motor speed based on the error value and the correction "strength" factor
      action->followSpeed += (error * m_cp);
      m_bot->SetMotorSpeed(1, m_speed);
      m_bot->SetMotorSpeed(2, -action->followSpeed);  // = follow speed, then multiply
      action->progress = angleCount + (180*d1)/(M_PI*ENC_BITS_P_MM*BOT_RADIUS*ROTATE_ERROR); 
      return false;
    }
  }

  // right rotation
  else if (angle < 0) 
  { 
    if (angleCount < abs(angle)) 
    {
      //Find the difference between the two encoders since last read
      float d1 = m_bot->GetM1EncoderDelta();
      float d2 = m_bot->GetM2EncoderDelta(); 
      d1 = abs(d1);
      d2 = abs(d2);
      float error = d2 - d1; 
      //Calculate the second motor speed based on the error value and the correction "strength" factor
      action->followSpeed += (error * m_cp);

      m_bot->SetMotorSpeed(1, -action->followSpeed);
      m_bot->SetMotorSpeed(2, m_speed);  
      action->progress = angleCount + (180*d2)/(M_PI*ENC_BITS_P_MM*BOT_RADIUS*ROTATE_ERROR);  
      return false;
    }
  }
  return true;
}

bool Drawbotic_Navigation::Stop(NavigationAction* action, float deltaTime_ms)
{
    float stopDuration = action->params[0];

    if (action->progress < stopDuration) {
        m_bot->SetMotorSpeed(1, 0);
        m_bot->SetMotorSpeed(2, 0);
        action->progress += deltaTime_ms; 
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