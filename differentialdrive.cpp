#include "differentialdrive.h"

differentialDrive::differentialDrive()
{
}
differentialDrive::differentialDrive(float wheelRadius, float wheelBaseLength)
{
    this->m_wheelRadius=wheelRadius;
    this->m_wheelBaseLength=wheelBaseLength;
}

void differentialDrive::setWheelRadius(float wheelRadius)
{
    this->m_wheelRadius=wheelRadius;
}
void differentialDrive::setWheelBaseLength(float wheelBaseLength)
{
    this->m_wheelBaseLength=wheelBaseLength;
}

void differentialDrive::uni2diff(float v,float w,float &vl,float &vr)
{
    vr=(2*v+w*this->m_wheelBaseLength)/(2*this->m_wheelRadius);
    vl=(2*v-w*this->m_wheelBaseLength)/(2*this->m_wheelRadius);
}
