#include "servo.h"
#include<wiringPi.h>
#include<iostream>
#include"../globalsettings.h"
#include"../sensor/rotaryencoder.h"

servo::servo()
{
    this->m_pin=-1;
    this->m_pin=ServoSignalPin;
}

servo::servo(int pin)
{
    this->m_pin=pin;
    //init();
}

float servo::getAbsAngle()
{
    return this->m_absAngle;
}

void servo::setPin(int pin)
{
    this->m_pin=pin;
}

void servo::init()
{
    if(this->m_pin==-1)
    {
        std::cerr<<"please set gpio pin first!";
        return;
    }
    pinMode(this->m_pin,OUTPUT);

    rotaryEncoder leftEncoder(LeftEncoderPin);
    rotaryEncoder rightEncoder(RightEncoderPin);

    //leftEncoder=leftEncoder.getLeftEncoder();
    //rightEncoder=rightEncoder.getRightEncoder();

//    int leftVal=leftEncoder.getValue();
//    int rightVal=rightEncoder.getValue();
//    int leftInterval=leftEncoder.getInterval();
//    int rightInterval=rightEncoder.getInterval();

    for(int i=0;i<20;i++)
    {
        // in order to make sure the encoder
        //value not change by the servo, below is the same
        //leftEncoder.setValue(leftVal);
        //rightEncoder.setValue(rightVal);
        //leftEncoder.setInterval(leftInterval);
        //rightEncoder.setInterval(rightInterval);

        digitalWrite(this->m_pin,HIGH);
        delayMicroseconds(500);
        digitalWrite(this->m_pin,LOW);

        //rightEncoder.setValue(rightVal);
        //leftEncoder.setValue(leftVal);
        //leftEncoder.setInterval(leftInterval);
        //rightEncoder.setInterval(rightInterval);


        delayMicroseconds(19500);

    }
    this->m_absAngle=0;
}

void servo::drive2angle(float angle)
{

    if(this->m_pin==-1)
    {
        std::cerr<<"please set gpio pin first!";
        return;
    }

    pinMode(this->m_pin,OUTPUT);

    int HighLevelTime_us=angle/180*2000+500;

    //rotaryEncoder myEncoder;
    //leftEncoder=leftEncoder.getLeftEncoder();
    //rightEncoder=rightEncoder.getRightEncoder();

    //rotaryEncoder infoEncoder;
//    int leftVal=myEncoder.getStaticValue(LeftSide);
//    int rightVal=myEncoder.getStaticValue(RightSide);
//    int leftInterval=myEncoder.getStaticInterval(LeftSide);
//    int rightInterval=myEncoder.getStaticInterval(RightSide);

    //int leftVal=leftEncoder.getValue();
    //int rightVal=rightEncoder.getValue();
    //int leftInterval=leftEncoder.getInterval();
    //int rightInterval=rightEncoder.getInterval();
    int n=10;
    if (angle==0)
        n=20;
    for(int i=0;i<n;i++)
    {
        /*
        myEncoder.setStaticValue(leftVal,LeftSide);
        myEncoder.setStaticValue(rightVal,RightSide);
        myEncoder.setStaticinterval(leftInterval,LeftSide);
        myEncoder.setStaticinterval(rightInterval,RightSide);*/

        //leftEncoder.setValue(leftVal);
        //rightEncoder.setValue(rightVal);
        //leftEncoder.setInterval(leftInterval);
        //rightEncoder.setInterval(rightInterval);

        digitalWrite(this->m_pin,HIGH);
        delayMicroseconds(HighLevelTime_us);
        digitalWrite(this->m_pin,LOW);

        delayMicroseconds(20000-HighLevelTime_us);
        /*
        myEncoder.setStaticValue(leftVal,LeftSide);
        myEncoder.setStaticValue(rightVal,RightSide);
        myEncoder.setStaticinterval(leftInterval,LeftSide);
        myEncoder.setStaticinterval(rightInterval,RightSide);*/

        //leftEncoder.setValue(leftVal);
        //rightEncoder.setValue(rightVal);
        //leftEncoder.setInterval(leftInterval);
        //rightEncoder.setInterval(rightInterval);


    }
    this->m_absAngle=angle;
}
