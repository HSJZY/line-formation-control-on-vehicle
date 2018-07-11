#ifndef DIFFERENTIALDRIVE_H
#define DIFFERENTIALDRIVE_H


class differentialDrive
{
public:
    differentialDrive();
    differentialDrive(float wheelRadius,float wheelBaseLength);

    void setWheelRadius(float wheelRadius);
    void setWheelBaseLength(float wheelBaseLength);

    ///@param v:the car velocity
    ///@param w:the car angular velocity
    ///@param vl:the velocity of the left motor to be calculated
    ///@param vr:the velocity of the right motor to be calculated
    void uni2diff(float v,float w,float &vl,float &vr);

    //int sgn(float value);
private:
    float m_wheelRadius;
    float m_wheelBaseLength;
};

#endif // DIFFERENTIALDRIVE_H
