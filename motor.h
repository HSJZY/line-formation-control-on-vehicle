#ifndef MOTOR_H
#define MOTOR_H

//const float PI=3.1415926;
class motor
{
public:
    motor();
    motor(int pinA,int pinB);

    void DriveMotor(float speed,int totalTime);
    void turnOffMotor();
    void turnOnMotor();

    float getDutyCycle_k();
    void setDutyCycle_k_1(float dutyCycle);

    // get the motor direction on two sides
    static int getLeftMotorDirection();
    static int getRightMotorDirection();

    void driveMotor_dutycycle(float dutycycle,float driveTime_ms);

private:
    void pwmDriveMotor(float dutyCycle,float driveTime,int direction);
    int sgn(float value);

private:

    bool turnOff;
    int m_pinA;
    int m_pinB;

    // the direction for left and right motor
    // move forward(1),still(0),move backward(-1)
    static int leftMotorDirection;
    static int rightMotorDirection;

    // angular velocity control gains
    float kp;//propotional parameter for angular velocity control
    float ki;//intergration parameter
    float kd;//differential parameter

    float e_P;// proportion error
    float e_I;// intergration error
    float e_D;// differential error

    float e_k;//current error
    float e_k_1;//last error
    float E_k;// intergration error success

    float dutyCycle_k;//the current duty cycle;
    float dutyCycle_k_1;//the last duty cycle;
};

#endif // MOTOR_H
