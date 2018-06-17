#include "carstatus.h"
#include "sensor/demo_dmp.h"

float carStatus::m_initAngleOfMPU;
float carStatus::m_curAngleOfMPU;
float carStatus::m_absAngleOfMPU;

carStatus::carStatus()
{
}

void carStatus::setInitAngleOfMPU(float angle)
{
    this->m_initAngleOfMPU=angle;
}

void carStatus::setAbsAngleOfMPU(float angle)
{
    this->m_absAngleOfMPU=angle;
    float curAngleOfMPU=m_absAngleOfMPU-m_initAngleOfMPU;
    if(curAngleOfMPU>180)
    {
        curAngleOfMPU=curAngleOfMPU-360;
    }
    else if(curAngleOfMPU<-180)
    {
        curAngleOfMPU=curAngleOfMPU+360;
    }
    this->m_curAngleOfMPU=curAngleOfMPU;
}

float carStatus::getCurAngleOfMPU()
{
    return this->m_curAngleOfMPU;
}
