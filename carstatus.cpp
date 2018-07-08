#include "carstatus.h"
#include "sensor/demo_dmp.h"

float carStatus::m_initAngleOfMPU;
float carStatus::m_curAngleOfMPU;
float carStatus::m_absAngleOfMPU;

vector<vector<vector<float> > > carStatus::vec_agents_postion;
bool carStatus::formation_stop;


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

void carStatus::set_formation_stop(bool is_stop)
{
    this->formation_stop=is_stop;
}

bool carStatus::get_formation_is_stop_state()
{
    return this->formation_stop;
}

void carStatus::set_agents_position(vector<vector<vector<float> > > agents_position)
{

    this->vec_agents_postion=agents_position;
}

vector<vector<vector<float> > > carStatus::get_agents_position()
{
    return this->vec_agents_postion;
}
