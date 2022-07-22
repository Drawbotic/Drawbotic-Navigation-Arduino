#ifndef DRAWBOTIC_NAVIGATION_H
#define DRAWBOTIC_NAVIGATION_H

#include <Drawbotic_DB1.h>

#define MAX_PARAMS 3
#define ENC_BITS_P_REV 295.6793 // number of encoder steps in one revolution of the wheel, value obtained from testing, drew long line and counted encoder steps
#define L_TURN_ERROR 1.01789 // scalar multiplier, obtained from testing, made left turns more accurate
#define R_TURN_ERROR 1.0045 // scalar multiplier, obtained from testing, made right turns more accurate
#define L_ROTATE_ERROR 1.008 // scalar multiplier, obtained from testing, made left rotate more accurate
#define R_ROTATE_ERROR 1.0045 // scalar multiplier, obtained from testing, made right rotations more accurate
#define FORWARD_ERROR 1.0067 // scalar multiplier, obtained from testing, made driving forward more accurate
#define ENC_BITS_P_MM 2.3529 // encoder signals per rotation (295.6793)/wheel circum (40pi)
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
};

struct NavigationAction 
{
    NavigationType type;
    float params[MAX_PARAMS];
    float followSpeed;
    float progress;
    NavigationAction* prev;
    NavigationAction* next;
};

class Drawbotic_Navigation
{
public:
    Drawbotic_Navigation(DB1* bot, float updateRate_ms = 10.0f, float speed = 0.05f, float correctionPower = 0.001f);
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

private:
    DB1* m_bot;
    NavigationAction* m_queueHead;
    NavigationAction* m_queueTail;
    bool DriveForward(NavigationAction* action);
    bool Turn(NavigationAction* action);
    bool Rotate(NavigationAction* action);
    bool Stop(NavigationAction* action, float deltaTime_ms);
    bool PenUp();
    bool PenDown();

    int m_queueSize;
    float m_timeBank;
    float m_updateRate_ms;
    float m_speed;
    float m_cp;
};

#endif
