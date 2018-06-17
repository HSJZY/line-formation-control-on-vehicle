#ifndef KINEMATICCONTROLLER_H
#define KINEMATICCONTROLLER_H

#include "../driver/motor.h"

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
    void moveForward(float speed_m_s,float forwardAngle_rad,float totalTime_s);
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
    void stopMotor(int side);
};

#endif // KINEMATICCONTROLLER_H
