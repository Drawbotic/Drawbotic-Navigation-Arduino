#include "Drawbotic_Navigation.h"

Drawbotic_Navigation::Drawbotic_Navigation(DB1* bot, float updateRate_ms)
{
    m_bot = bot;
    m_timebank = 0;
    m_updateRate_ms = updateRate_ms;
    m_queueSize = 0;
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
    //Fill in the rest

    return newAction;
}

NavigationAction* Drawbotic_Navigation::MakeTurnAction(float radius_mm, float angle_deg)
{
    NavigationAction* newAction = new NavigationAction();
    newAction->type = NAV_TURN;
    //Fill in the rest

    return newAction;
}

NavigationAction* Drawbotic_Navigation::MakeRotateAction(float angle_deg)
{
    NavigationAction* newAction = new NavigationAction();
    newAction->type = NAV_ROTATE;
    //Fill in the rest

    return newAction;
}

NavigationAction* Drawbotic_Navigation::MakeStopAction(float time_ms)
{
    NavigationAction* newAction = new NavigationAction();
    newAction->type = NAV_STOP;
    //Fill in the rest

    return newAction;
}

NavigationAction* Drawbotic_Navigation::MakePenAction(bool up)
{
    NavigationAction* newAction = new NavigationAction();
    newAction->type = up ? NAV_PEN_UP : NAV_PEN_DOWN;
    //Fill in the rest

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
                finished = Stop(currentAction);
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
            }
        }
        m_timebank = 0; //reset the time bank for the next update
    }
}

bool Drawbotic_Navigation::DriveForward(NavigationAction* action)
{
    bool finished = false;

    //Do stuff!

    return finished;
}

bool Drawbotic_Navigation::Turn(NavigationAction* action)
{
    bool finished = false;

    //Do stuff!

    return finished;
}

bool Drawbotic_Navigation::Rotate(NavigationAction* action)
{
    bool finished = false;

    //Do stuff!

    return finished;
}

bool Drawbotic_Navigation::Stop(NavigationAction* action)
{
    bool finished = false;

    //Do stuff!

    return finished;
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