#include "motor.h"
#include<wiringPi.h>
#include<iostream>
#include<softPwm.h>
#include<math.h>

#include"rotaryencoder.h"
#include"globalsettings.h"

int motor::rightMotorDirection;
int motor::leftMotorDirection;

motor::motor()
{
    this->m_pinA=-1;
    this->m_pinB=-1;
    this->turnOff=false;
    //this->testing=0;
    // initial set the angular velocity control gains
    this->kp=5;
    this->ki=0;
    this->kd=0.01;

    this->e_I=0;
    this->e_D=0;
    this->E_k=0;
    this->e_k=0;
    this->e_k_1=0;

    this->dutyCycle_k=StartDutyCycle;
    this->dutyCycle_k_1=StartDutyCycle;
}


int motor::getLeftMotorDirection()
{
    return leftMotorDirection;
}
int motor::getRightMotorDirection()
{
    return rightMotorDirection;
}


void motor::setDutyCycle_k_1(float dutyCycle)
{
    this->dutyCycle_k_1=dutyCycle;
}

float motor::getDutyCycle_k()
{
    return this->dutyCycle_k;
}

motor::motor(int pinA, int pinB)
{
    this->m_pinA=pinA;
    this->m_pinB=pinB;
    this->turnOff=false;
    //this->testing=0;

    // initial set the angular velocity control gains
    this->kp=5;
    this->ki=0;
    this->kd=0.01;

    this->e_I=0;
    this->e_D=0;
    this->E_k=0;
    this->e_k=0;
    this->e_k_1=0;

    this->dutyCycle_k=StartDutyCycle;
    this->dutyCycle_k_1=StartDutyCycle;
}


void motor::turnOnMotor()
{
    this->turnOff=false;
}

void motor::turnOffMotor()
{
    digitalWrite(this->m_pinA,LOW);
    digitalWrite(this->m_pinB,LOW);
    this->turnOff=true;
}


/** drive the motor in a given speed for the total time(>200ms)
 *@param speed: the angular velocity setted(degree/s)
 *@param side: the left or right to drive,(0 for left;1 for right)
 *@param totalTime: total time to drive motor(ms)
 */
void motor::DriveMotor(float speed,int totalTime)
{

    int side;
    if(this->m_pinA==LeftMotorIn1 && this->m_pinB==LeftMotorIn2)
        side= LeftSide;
    else
        side=RightSide;

    rotaryEncoder myEncoder;
    if(side==LeftSide)
    {
        this->leftMotorDirection=sgn(speed);
    }
    else if(side==RightSide)
    {
        this->rightMotorDirection=sgn(speed);
    }
    else
    {
        std::cerr<<"Please set the correct side first"<<std::endl;
        return;
    }

    int directionSetted=sgn(speed);//setted direction
    int nTime=totalTime/200;
    int i=0;
    while (i<nTime)
    {
        i++;
        float currentSpeed;
        if(side==LeftSide)
            currentSpeed=myEncoder.getStaticAngularVelocity(LeftSide);
        else if(side==RightSide)
            currentSpeed=myEncoder.getStaticAngularVelocity(RightSide);
        else
            std::cerr<<"Please input the correct side";

        this->e_k=(abs(speed)-abs(currentSpeed))/180;// calculate the velocity error for pid control
        if(abs(e_k)<0.2)
            e_k=0;

        //get the error of three parameter
        this->e_P=this->e_k;
//        this->e_I=this->E_k+e_I*0.1;
//        this->e_D=(this->e_k-this->e_k_1)/0.1;

        float gain=kp*e_P/*+ki*e_I+kd*e_D*/;// calculate the gains for pid control
        float gainLimited=10;
        if(abs(gain)>gainLimited)
        {
            std::cout<<"sgn(gain)"<<sgn(gain)<<"gain"<<gain<<std::endl;
            gain=sgn(gain)*gainLimited;
        }


        float adjust=gain/gainLimited;//control the gain in (-1,1)

        float ka=0.15;
        if(adjust<=0)
            this->dutyCycle_k=this->dutyCycle_k_1+ka*adjust*dutyCycle_k_1;
        else
            this->dutyCycle_k=this->dutyCycle_k_1+ka*adjust*(1-dutyCycle_k_1);

        std::cout<<"gain:"<<gain<<" currentSpeed"<<currentSpeed<<" dutyCycle:"<<dutyCycle_k<<std::endl;

        pwmDriveMotor(this->dutyCycle_k,10,directionSetted);

        this->e_k_1=this->e_k;
        this->dutyCycle_k_1=this->dutyCycle_k;
    }
}

void motor::driveMotor_dutycycle(float dutycycle, float driveTime_ms)
{
    int side,direction;
    if(this->m_pinA==LeftMotorIn1 && this->m_pinB==LeftMotorIn2)
        side= LeftSide;
    else
        side=RightSide;

    if(side==LeftSide)
    {
        direction=sgn(dutycycle);
        this->leftMotorDirection=sgn(dutycycle);
    }
    else
    {
        direction=sgn(dutycycle);
        this->rightMotorDirection=sgn(dutycycle);
    }

    dutycycle=abs(dutycycle);

    int n_iter=driveTime_ms/200;
    for(int i=0;i<n_iter;i++)
    {
        pwmDriveMotor(dutycycle,10,direction);
    }
}

void motor::pwmDriveMotor(float dutyCycle,float driveTime,int direction)
{
//    dutyCycle=0.15;
    if(wiringPiSetup()==-1)
    {
        std::cout<<"Setup wiring pi failed";
        return;
    }
    if(m_pinA==-1||m_pinB==-1)
    {
        std::cout<<"pin error, please set up pin firstly";
        return;
    }
    else if(m_pinA==RightMotorIn3&&m_pinB==RightMotorIn4)
    {
        direction*=-1;
    }

    pinMode(this->m_pinA,OUTPUT);
    pinMode(this->m_pinB,OUTPUT);
    digitalWrite(this->m_pinA,LOW);
    digitalWrite(this->m_pinB,LOW);

    if(this->turnOff==true)
        return;

    for(int i=0;i<20;i++)
    {
        if(direction==-1)
        {
            digitalWrite(this->m_pinB,HIGH);
            delay(driveTime*dutyCycle);
            digitalWrite(this->m_pinB,LOW);
            delay(driveTime*(1-dutyCycle));
        }
        else if(direction==1)
        {
            digitalWrite(this->m_pinA,HIGH);
            delay(driveTime*dutyCycle);
            digitalWrite(this->m_pinA,LOW);
            delay(driveTime*(1-dutyCycle));
        }
    }


    this->turnOff=false;
}

/** sgn function:
 *value is positive, return 1
 *value is negative, return -1
 *value is zero,return 0
 */
int motor::sgn(float value)
{
    return (0<value)-(0>value);
}
