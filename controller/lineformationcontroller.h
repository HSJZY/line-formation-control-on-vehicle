#ifndef LINEFORMATIONCONTROLLER_H
#define LINEFORMATIONCONTROLLER_H

#include<vector>
#include<iostream>
#include<stdlib.h>
#include"../driver/servo.h"
#include"../driver/motor.h"
#include"../sensor/rotaryencoder.h"
#include"kinematiccontroller.h"
#include"../util/log.h"

using namespace std;

class lineFormationController
{
public:
    lineFormationController();
    lineFormationController(float interDistance,float selfDirecton);

    void setInterDistance(float interDistance);
    void startFormation();
    
private:
    void createNbrData(float selfAngle);
    void carControl();
    void calcNbrData();
private:
    vector<vector<float>> m_nbrDate;
    vector<float> m_leftNbr;
    vector<float> m_rightNbr;

    Log m_logFile;

    bool m_leftOwnsCar;
    bool m_rightOwnsCar;

    int m_leftNbrNum;
    int m_rightNbrNum;

    float m_interDistacne;
    float m_selfDirection;
};

#endif // LINEFORMATIONCONTROLLER_H
