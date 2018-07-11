#ifndef KINEMATICCONTROLLER_H
#define KINEMATICCONTROLLER_H

#include "motor.h"
#include "pid.h"
#include"globalsettings.h"

class kinematicController
{
public:
    kinematicController();
    kinematicController(float forwardDistance,float forwardAngle,float rotate2direction);

    void goToGoal(float forwardDistance,float forwardAngle);

public:
    void selfRotate(float angle);
    void lineForward(float forwardDistance);
    void lineForward(float forwardDistance,float forwardAngle_rad);
    void moveForward(float ratio_dutycycle,float forwardAngle_rad,float totalTime_s,float diff_dutyCycle=0);

    int self_rotate_target(float target_angle);
private:
    float m_selfDirection;
    float m_rotate2direction;
    float m_forwardDistance;
    float m_forwardAngle;
    float m_velocity;

    float leftDutyCycle_k_1;
    float rightDutyCycle_k_1;

    int sgn(float value);

    void runMotorThread(int side,float AngularVelociy,float dutyCycle);
    void runMotorThread_dutycycle(int side,float dutyCycle);
    void stopMotor(int side);
};

#endif // KINEMATICCONTROLLER_H
