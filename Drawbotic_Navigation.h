#ifndef DRAWBOTIC_NAVIGATION_H
#define DRAWBOTIC_NAVIGATION_H

#include <Drawbotic_DB1.h>

#define MAX_PARAMS 3
#define ENC_BITS_P_REV 295.6793 
//#define TURN_FACTOR 2.395 
//#define ROTATE_FACTOR 2.397 
#define ENC_BITS_P_MM 2.3529// encoder signals per rotation (295.6793)/wheel circum (40pi)  = 2.3529
// EXPLORE THIS ^^^
#define WHEEL_RADIUS 20
#define BOT_RADIUS 60



enum NavigationType 
{
    NAV_FORWARD,
    NAV_TURN,
    NAV_ROTATE,
    NAV_STOP,
    NAV_PEN_UP,
    NAV_PEN_DOWN,
    NAV_UNKNOWN
};

struct NavigationAction 
{
    NavigationType type;
    //NavigationType nextType;
    //NavigationType prevType;
    //rest of the stuff
    float params[MAX_PARAMS];
    float followSpeed;
    float progress;
    //float accel;
    //float decel;
    NavigationAction* prev;
    NavigationAction* next;
};

class Drawbotic_Navigation
{
public:
    Drawbotic_Navigation(DB1* bot, float updateRate_ms = 10.0f, float speed = 0.05f, float correctionPower = 0.001f, float accelR = 0.05f);
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
    void SetSpeed(float speed);
    void SetAccelRatio(float accelR);

private:
    DB1* m_bot;
    NavigationAction* m_queueHead;
    NavigationAction* m_queueTail;
    bool DriveForward(NavigationAction* action);
    bool Turn(NavigationAction* action);
    bool Rotate(NavigationAction* action);
    bool Stop(NavigationAction* action, float deltaTime_ms);
    bool PenUp(NavigationAction* action);
    bool PenDown(NavigationAction* action);

    int m_queueSize;
    float m_timebank;
    float m_updateRate_ms;
    float m_speed;
    float m_cp;
    float m_accelR;

};

#endif
