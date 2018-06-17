#ifndef CARSTATUS_H
#define CARSTATUS_H

#include"sensor/demo_dmp.h"

class carStatus
{
public:
    carStatus();
    void setInitAngleOfMPU(float angle);
    void setAbsAngleOfMPU(float angle);
    float getCurAngleOfMPU();

private:
    static float m_initAngleOfMPU;
    static float m_curAngleOfMPU;
    static float m_absAngleOfMPU;
};

#endif // CARSTATUS_H
