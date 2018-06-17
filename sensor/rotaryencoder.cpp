#include "rotaryencoder.h"

#include<wiringPi.h>
#include<iostream>
#include<time.h>
#include<sys/time.h>
#include<unistd.h>
#include<cmath>
#include<signal.h>
#include"globalsettings.h"
#include"../util/kalman.h"
#include"../driver/motor.h"

//rotaryEncoder* rotaryEncoder::m_rotaryEncoder;
rotaryEncoder* rotaryEncoder::leftEncoder;
rotaryEncoder* rotaryEncoder::rightEncoder;

rotaryEncoder::rotaryEncoder()
{
    this->m_pin=-1;
    this->m_value=0;
    this->tInterval=0;
    this->m_angularVelocity=0;
    this->tp1=0;
    this->tp2=0;
    this->rv1=0;
    this->rv2=0;
    this->m_angularVelocity_1=0;
    //m_rotaryEncoder=this;
}

rotaryEncoder::rotaryEncoder(int pin)
{
    this->m_pin=pin;
    this->m_value=0;
    this->tInterval=0;
    this->m_angularVelocity=0;
    this->tp1=0;
    this->tp2=0;
    this->rv1=0;
    this->rv2=0;
    this->m_angularVelocity_1=0;

    if(pin==RightEncoderPin)
        rightEncoder=this;
    else if(pin==LeftEncoderPin)
        leftEncoder=this;
    //m_rotaryEncoder=this;
}

float rotaryEncoder::getStaticAngularVelocity(int side)
{
    if(side==0)
        return leftEncoder->getAngularVelocity();
    else if(side==1)
        return rightEncoder->getAngularVelocity();
    else
        std::cerr<<"Please input the correct side";
}

int rotaryEncoder::getStaticValue(int side)
{
    if(side==LeftSide)
        return leftEncoder->getValue();
    else if(side==RightSide)
        return rightEncoder->getValue();
    else
        std::cerr<<"Please input the correct side";
}

void rotaryEncoder::setStaticValue(int value,int side)
{
    if(side==LeftSide)
        leftEncoder->m_value=value;
    else if(side==RightSide)
        rightEncoder->m_value=value;
    else
        std::cerr<<"Please input the correct side";
}

int rotaryEncoder::getStaticInterval(int side)
{
    if(side==LeftSide)
        return leftEncoder->tInterval;
    else if (side==RightSide)
        return rightEncoder->tInterval;
    else
        std::cerr<<"Please input the correct side";
    return -1;

}

void rotaryEncoder::setStaticinterval(int value,int side)
{
    if(side==LeftSide)
        leftEncoder->tInterval=value;
    else if(side==RightSide)
        rightEncoder->tInterval=value;
    else
        std::cerr<<"Please input the correct side";
}

/*
void rotaryEncoder::setEncoder(int side)
{
    if(side==0)
        this=leftEncoder;
    else if(side==1)
        this=rightEncoder;
    else
        std::cerr<<"Please set the correct side ,1 for right,0 for left";
}
*/
void rotaryEncoder::setPin(int pin)
{
    this->m_pin=pin;
    if(pin==RightEncoderPin)
        rightEncoder=this;
    else if(pin==LeftEncoderPin)
        leftEncoder=this;
}


void rotaryEncoder::setValue(int value)
{
    this->m_value=value;
}

int rotaryEncoder::getValue()
{
    return this->m_value;
}

void rotaryEncoder::setInterval(int interval)
{
    this->tInterval=interval;
}
int rotaryEncoder::getInterval()
{
    return this->tInterval;
}

rotaryEncoder rotaryEncoder::getLeftEncoder()
{
    return *leftEncoder;
}

rotaryEncoder rotaryEncoder::getRightEncoder()
{
    return *rightEncoder;
}

float rotaryEncoder::getAngularVelocity()
{
    if(abs(this->m_angularVelocity)<1)
        return 0;
    return this->m_angularVelocity;
}


//start the encoder moudle
void rotaryEncoder::startCounting()
{

    if(this->m_pin==-1)
    {
        std::cout<<"Please set encoder pin first"<<std::endl;
        return;
    }
    pinMode(this->m_pin,INPUT);
    pullUpDnControl(this->m_pin,PUD_UP);

    if(m_pin==RightEncoderPin)
        wiringPiISR(this->m_pin,INT_EDGE_BOTH,updateRightEncoders);
    else if(m_pin==LeftEncoderPin)
        wiringPiISR(this->m_pin,INT_EDGE_BOTH,updateLeftEncoders);

    this->lastVal=this->m_value;

    //used for checking if the car is static,if it's static,turn the velocity to 0
    struct timeval timerBreakStart,timerBreakEnd;
    gettimeofday(&timerBreakStart,NULL);
    long int startTime=timerBreakStart.tv_sec*1000+timerBreakStart.tv_usec/1000;

    while (1)
    {
        gettimeofday(&timerBreakEnd,NULL);
        long int endtime=timerBreakEnd.tv_sec*1000+timerBreakEnd.tv_usec/1000;
        if(endtime-startTime>600)
        {
            checkSpeedZero();
            startTime=endtime;
        }
    }
}


void rotaryEncoder::updateRightEncoders()
{
    int isMoveForward=rightEncoder->isMovingForward(/*rightMotor*/1);

    if(isMoveForward==1)
        rightEncoder->m_value++;
    else
        rightEncoder->m_value--;
    // get the current time for calculating the interval between two interrupt
    struct timeval tp;
    gettimeofday(&tp,NULL);
    long int currentTime=tp.tv_sec*1000+tp.tv_usec/1000;


    //this param aims to remove glitch
    int delayTime=10;
    //every n interval for a speed calculation
    int n=4;
    rightEncoder->tInterval++;
    if(rightEncoder->tInterval==1)
    {
        rightEncoder->tp1 =currentTime;
        rightEncoder->rv1=rightEncoder->m_value;
    }
    else if(rightEncoder->tInterval==n)
    {
        rightEncoder->tp2=currentTime;
        rightEncoder->tInterval=0;
        rightEncoder->rv2=rightEncoder->m_value;

        //std::cout<<"left  rv2:"<<rightEncoder->rv2<<"  rv1"<<rightEncoder->rv1<<std::endl;


        float betweenTime=(currentTime-rightEncoder->tp1)/1000.0;

        //float betweenAngle=(leftEncoder->rv2-leftEncoder->rv1)/40*360;

        float betweenAngle=(rightEncoder->rv2-rightEncoder->rv1)/40.0*360;

        //leftEncoder->lastAngVel=leftEncoder->m_angularVelocity;
        float av=betweenAngle/betweenTime;

        Kalman KalmanD1(1,30,1023,rightEncoder->m_angularVelocity_1);
        //Kalman(0.125,32,1023,0)
        rightEncoder->m_angularVelocity=KalmanD1.getFilteredValue(av);
        //std::cout<<std::endl<<"   Left: Angle interval"<<betweenAngle<<"   time interval"<<betweenTime<<"   angular velocity"<<rightEncoder->m_angularVelocity<<"  calculation "<<betweenAngle/betweenTime<<std::endl;
    }
    delay(delayTime);
    //bool isMoveForward=leftEncoder->isMovingForward(LeftMotorIn1,LeftMotorIn2,8);
}

void rotaryEncoder::updateLeftEncoders()
{
    int moveDirection=leftEncoder->isMovingForward(/*leftmotor*/0);
    //int moveDirection=0;

    if(moveDirection==1)
        leftEncoder->m_value++;
    else /*if(moveDirection==-1)*/
        leftEncoder->m_value--;
    //else
        //return;
    // get the current time for calculating the interval between two interrupt
    struct timeval tp;
    gettimeofday(&tp,NULL);
    long int currentTime=tp.tv_sec*1000+tp.tv_usec/1000;


    //this param aims to remove glitch
    int delayTime=10;
    //every n interval for a speed calculation
    int n=4;
    leftEncoder->tInterval++;
    if(leftEncoder->tInterval==1)
    {
        leftEncoder->tp1 =currentTime;
        leftEncoder->rv1=leftEncoder->m_value;
    }
    else if(leftEncoder->tInterval==n)
    {
        leftEncoder->tp2=currentTime;
        leftEncoder->tInterval=0;
        leftEncoder->rv2=leftEncoder->m_value;

        //std::cout<<"left  rv2:"<<leftEncoder->rv2<<"  rv1"<<leftEncoder->rv1<<std::endl;


        float betweenTime=(currentTime-leftEncoder->tp1)/1000.0;

        //float betweenAngle=(leftEncoder->rv2-leftEncoder->rv1)/40*360;

        float betweenAngle=(leftEncoder->rv2-leftEncoder->rv1)/40.0*360;

        //leftEncoder->lastAngVel=leftEncoder->m_angularVelocity;
        float av=betweenAngle/betweenTime;
        Kalman KalmanD1(1,30,1023,leftEncoder->m_angularVelocity_1);
        //Kalman(0.125,32,1023,0)
        leftEncoder->m_angularVelocity=KalmanD1.getFilteredValue(av);
        //leftEncoder->m_angularVelocity=leftEncoder->m_angularVelocity_1*0.2+av*0.8;
        //std::cout<<std::endl<<"   Left: Angle interval"<<betweenAngle<<"   time interval"<<betweenTime<<"   angular velocity"<<leftEncoder->m_angularVelocity<<"  calculation "<<betweenAngle/betweenTime<<std::endl;
    }
    delay(delayTime);
    //bool isMoveForward=leftEncoder->isMovingForward(LeftMotorIn1,LeftMotorIn2,8);
}



void rotaryEncoder::checkSpeedZero()
{
    if(m_pin==LeftEncoderPin)
    {
        if(abs(leftEncoder->m_value-leftEncoder->lastVal)<=1)
            leftEncoder->m_angularVelocity=0;
        leftEncoder->lastVal=leftEncoder->m_value;
    }
    else
    {
        if(abs(rightEncoder->m_value-rightEncoder->lastVal)<=1)
            rightEncoder->m_angularVelocity=0;
        rightEncoder->lastVal=rightEncoder->m_value;
    }

}
/** check if the motor is moving forward
 * @param side: input which side to get
 * LeftSide=0,RightSide=1
 */
int rotaryEncoder::isMovingForward(int side)
{
    motor Motor;
    //std::cout<<"leftMotorDirection:"<<Motor.getLeftMotorDirection()<<"   rightMotorDirection:"<<Motor.getRightMotorDirection()<<std::endl;
    if(side==LeftSide)
        return Motor.getLeftMotorDirection();
    else if(side==RightSide)
        return Motor.getRightMotorDirection();
    else
        std::cerr<<"please input the correct side";
    return 0;
}
