#include<iostream>
#include<thread>
#include<wiringPi.h>
#include<math.h>
#include<string>
#include<opencv2/core.hpp>
#include<QDebug>
#include"globalsettings.h"
#include"controller/kinematiccontroller.h"
#include"sensor/hmc5883l.h"
#include"sensor/demo_dmp.h"
#include"carstatus.h"
#include"sensor/rotaryencoder.h"

using namespace std;
using namespace cv;

void MPUThread()
{
    carStatus curCarStatus;
    while(1)
    {
        float t_yaw=0;
        // the we multiply -1 to the loop since clockwise rotate is positive in my definition
        t_yaw=-loop();
        float yaw;
        if(t_yaw==365)
            continue;
        yaw=t_yaw;
        delay(20);
        curCarStatus.setAbsAngleOfMPU(yaw);
    }
}

void initMPU6050()
{
    setup();
    delay(1000);
    //float yaw=loop();
    //cout<<yaw;
    float lastYaw=0;
    float yaw=0;
    cout<<"justifying the MPU6050,please waiting...";
    while(abs(yaw-lastYaw)>0.05||lastYaw==0)
    {
        float t_yaw=0;
        t_yaw=loop();
        if(t_yaw==-365)
            continue;
        lastYaw=yaw;
        yaw=t_yaw;
        delay(1000);
        std::cout<<"yaw:"<<yaw<<std::endl;
    }
    carStatus curCarStatue;
    curCarStatue.setInitAngleOfMPU(-lastYaw);
    curCarStatue.setAbsAngleOfMPU(-lastYaw);

    std::cout<<"press any char to continue"<<std::endl;
    //cv::waitKey(0);
    //int key;
    //cin>>key;
    std::cout<<"testing";
    thread mpuThread(MPUThread);
    mpuThread.detach();
}
// start encoder counting in a new thread
void encoderThread(rotaryEncoder testEncoder)
{
    testEncoder.startCounting();
}

void motorThread(motor testMotor)
{
    testMotor.DriveMotor(/*angular velocity*/-200,/*left*/0,10000);
}
void motorThread1(motor testMotor)
{
    testMotor.DriveMotor(/*angular velocity*/400,/*right*/1,1000);

}

void selfRotateThread(kinematicController Rotate)
{
    Rotate.selfRotate(1.8);
}

void lineThread(kinematicController lineRun)
{
    lineRun.goToGoal(1,-1);
    //lineRun.lineForward(1,0);
}


int main(int argc, char *argv[])
{
    wiringPiSetup();

    rotaryEncoder leftEncoder(LeftEncoderPin);
    thread leftEncoderThread(encoderThread,leftEncoder);
    leftEncoderThread.detach();

    rotaryEncoder rightEncoder(RightEncoderPin);
    thread rightEncoderThread(encoderThread,rightEncoder);
    rightEncoderThread.detach();

    //turn off two motors
    motor rightMotor(RightMotorIn3,RightMotorIn4);
    motor leftMotor(LeftMotorIn1,LeftMotorIn2);
    rightMotor.turnOffMotor();
    leftMotor.turnOffMotor();


    initMPU6050();

    //kinematicController Rotate,lineR;
    //thread rotateThread(selfRotateThread,Rotate);
    //rotateThread.detach();
    //thread line(lineThread,lineR);
    //line.detach();
   // line.join();
    //rotateThread.join();

    leftMotor.turnOnMotor();

    thread leftMotorThread(motorThread,leftMotor);
//    leftMotor.turnOffMotor();
    leftMotorThread.detach();

    //delay(1000);
    //thread rightMototThread(motorThread1,rightMotor);
    //rightMototThread.detach();
    //rightMotor.turnOffMotor();
    //cout<<testEncoder.getValue()<<"   ";

    //rotateServo.drive2angle(90);
    //rotateServo.drive2angle(180);
    //cout<<testEncoder.getValue()<<"   ";

    //cout<<testEncoder.getValue()<<"   ";

    //cout<<testEncoder.getValue()<<"   "<<endl;
    //delay(100);

    //gyro_mpu6050 gyro;
    //perror("init gyro");
    //int x,y,z;


    //thread testThread(&testEncoder.startCounting);
    carStatus curCarStatue;

    while(1)
    {
        delay(1000);
    }
    //
    /*
    while(1)
    {
        //cout<<"left"<<leftMotor.gettesting()<<"right"<<rightMotor.gettesting()<<endl;

       
       cout<<"leftEncoder"<<endl;
        cout<<leftEncoder.getValue()<<"   ";
        cout<<leftEncoder.getAngularVelocity()<<endl;

        //leftMotor.turnOffMotor();
        cout<<"rightEncoder"<<endl<<rightEncoder.getValue()<<"   "<<rightEncoder.getAngularVelocity()<<endl;
        cout<<"carstatue:"<<curCarStatue.getCurAngleOfMPU()<<endl;
        //cout<<testEncoder.getValue()<<endl<<testEncoder.getAngularVelocity()<<endl;


        //delay(200);

    }*/
}
