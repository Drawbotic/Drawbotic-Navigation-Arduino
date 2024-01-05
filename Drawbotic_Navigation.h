#ifndef DRAWBOTIC_NAVIGATION_H
#define DRAWBOTIC_NAVIGATION_H

#include <Drawbotic_DB1.h>

#define L_TURN_ERROR    1.01789f   // scalar multiplier, obtained from testing, made left turns more accurate
#define R_TURN_ERROR    1.0045f    // scalar multiplier, obtained from testing, made right turns more accurate
#define L_ROTATE_ERROR  1.008f     // scalar multiplier, obtained from testing, made left rotate more accurate
#define R_ROTATE_ERROR  1.0045f    // scalar multiplier, obtained from testing, made right rotations more accurate
#define FORWARD_ERROR   1.0000f    // scalar multiplier, obtained from testing, made driving forward more accurate
#define ENC_BITS_P_MM   2.3433f    // encoder signals per rotation (295.6793)/wheel circum (40pi)
#define BOT_RADIUS      60
#define IMU_TOLERANCE   0.2f
#define SPEED_MIN       0.045f
#define ROTATE_KP       0.03f
#define FORWARD_KP      0.01f

class Drawbotic_Navigation {
public:
  Drawbotic_Navigation(bool useIMU = true, float updateRate_ms = 1.0f, float speed = 0.1f, float correctionPower = 0.015f);
  void clearAllActions();
  void addForwardAction(float distance_mm, bool front=false);
  void addTurnAction(float radius_mm, float angle_deg, bool front=false);
  void addRotateAction(float angle_deg, bool front=false);
  void addStopAction(float time_ms, bool front=false);
  void addPenAction(bool down, bool front=false);
  int  getQueueSize() { return m_queueSize; }
  void update(float deltaTime_ms);
  void setSpeed(float speed);

private:
  //Internal enum outlining the possible nav actions
  enum NavigationType {
    NAV_FORWARD,
    NAV_TURN,
    NAV_ROTATE,
    NAV_STOP,
    NAV_PEN_UP,
    NAV_PEN_DOWN,
  };

  //Internal private struct to represent a nav action
  struct NavigationAction {
    NavigationType type;
    float followSpeed;
    float progress;
    float desiredAngle;
    float finalAngle;
    float desiredDistance;
    float desiredRadius;
    float desiredTime;
    NavigationAction* prev;
    NavigationAction* next;
  };

  NavigationAction* m_queueHead;
  NavigationAction* m_queueTail;
  NavigationAction* makeForwardAction(float distance_mm);
  NavigationAction* makeTurnAction(float radius_mm, float angle_deg);
  NavigationAction* makeRotateAction(float angle_deg);
  NavigationAction* makeStopAction(float time_ms);
  NavigationAction* makePenAction(bool down);
  void setupRotationAction(NavigationAction *action);
  void addActionFront(NavigationAction *action);
  void addActionBack(NavigationAction *action);
  bool driveForward(NavigationAction* action);
  bool turn(NavigationAction* action);
  bool rotateEnc(NavigationAction* action);
  bool rotateIMU(NavigationAction* action);
  bool stop(NavigationAction* action, float deltaTime_ms);
  bool penUp();
  bool penDown();

  int   m_queueSize;
  float m_timeBank;
  float m_updateRate_ms;
  float m_speed;
  float m_cp;

  bool  m_useIMU;
};

#endif
