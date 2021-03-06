#ifndef CARSTATUS_H
#define CARSTATUS_H

#include"mpu/demo_dmp.h"
#include<vector>
using namespace std;

class carStatus
{
public:
    carStatus();
    void setInitAngleOfMPU(float angle);
    void setAbsAngleOfMPU(float angle);
    float getCurAngleOfMPU();

    void set_agents_position(vector<vector<vector<float> > > agents_position);
    vector<vector<vector<float> > > get_agents_position();

    bool get_formation_is_stop_state();
    void set_formation_stop(bool is_stop);

    void set_hung_assignment(vector<int> vec_assig);
    vector<int> get_hung_assignment();

private:
    static vector<vector<vector<float> > > vec_agents_postion;
    static vector<int> vec_hung_assignment;

    static bool formation_stop;

    static float m_initAngleOfMPU;
    static float m_curAngleOfMPU;
    static float m_absAngleOfMPU;
};

#endif // CARSTATUS_H
