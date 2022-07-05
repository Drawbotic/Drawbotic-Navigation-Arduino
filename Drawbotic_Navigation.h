#ifndef DRAWBOTIC_NAVIGATION_H
#define DRAWBOTIC_NAVIGATION_H

#include <Drawboitc_DB1.h>

enum NavigationType 
{
    NAV_FORWARD,
    NAV_TURN,
    NAV_ROTATE,
    NAV_STOP,
    NAV_PEN_UP,
    NAV_PEN_DOWN
};

struct NavigationAction 
{
    NavigationType type;
    //rest of the stuff
    NavigationAction* prev;
    NavigationAction* next;
};

class Drawbotic_Navigation
{
public:
    Drawbotic_Navigation(DB1* bot, float updateRate_ms = 10.0f);
    void ClearAllActions();
    void AddActionFront(NavigationAction* action);
    void AddActionBack(NavigationAction* action);
    int GetQueueSize() { return m_queueSize; }
    NavigationAction* MakeForwardAction(float distance_mm);
    NavigationAction* MakeTurnAction(float radius_mm, float angle_deg);
    NavigationAction* MakeRotateAction(float angle_deg);
    NavigationAction* MakeStopAction(float time_ms);
    NavigationAction* MakePenAction(bool up);
    void Update(float deltaTime_ms);

private:
    DB1* m_bot;
    NavigationAction* m_queueHead;
    NavigationAction* m_queueTail;
    bool DriveForward(NavigationAction* action);
    bool Turn(NavigationAction* action);
    bool Rotate(NavigationAction* action);
    bool Stop(NavigationAction* action);
    bool PenUp(NavigationAction* action);
    bool PenDown(NavigationAction* action);

    int m_queueSize;
    float m_timebank;
    float m_updateRate_ms;
};

#endif
