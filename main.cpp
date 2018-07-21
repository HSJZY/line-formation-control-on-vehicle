#include "mainwindow.h"
#include <QApplication>
#include<iostream>
#include<thread>
#include<wiringPi.h>
#include<math.h>
#include<string>
#include<opencv2/core.hpp>
#include<QDebug>
#include <QApplication>
#include"globalsettings.h"
#include"kinematiccontroller.h"
#include"mpu/demo_dmp.h"
#include"carstatus.h"
#include"rotaryencoder.h"
#include"udp_client.h"
#include"utils.h"
#include"line_formation_control.h"
#include"test_program.h"

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

    std::cout<<"testing";
    thread mpuThread(MPUThread);
    mpuThread.detach();
}

void update_postion_thread(udp_client udp_listener)
{
    carStatus cur_robot_status;
    while(1)
    {
        string recv_formation=udp_listener.start_listen();
        if(recv_formation=="")
        {
            std::cerr<<"connenction error";
            continue;
        }
        vector<vector<vector<float> > > vec_agents_position=parse_agents_position(recv_formation);
        cur_robot_status.set_agents_position(vec_agents_position);
        delay(30);
    }
}

void init_udp_thread()
{
    string addr="192.168.43.95";
    int port=5000;
    udp_client udp_test(addr,port);
    while(1)
    {
        string recv_formation=udp_test.start_listen();
        if(recv_formation.empty())
            continue;
        else
        {
            delay(200);
            vector<int> hungarian_assignment;
            vector<vector<vector<float> > > vec_agents_position=parse_agents_position(recv_formation,hungarian_assignment);
            carStatus cur_robot_status;
            cur_robot_status.set_agents_position(vec_agents_position);
            if(!hungarian_assignment.empty())
            {
                cur_robot_status.set_hung_assignment(hungarian_assignment);
            }
            break;
        }
    }

    thread agents_postion_listen_thread(update_postion_thread,udp_test);
    agents_postion_listen_thread.detach();
}


// start encoder counting in a new thread
void encoderThread(rotaryEncoder testEncoder)
{
    testEncoder.startCounting();
}

void motorThread(motor testMotor)
{
    testMotor.DriveMotor(/*angular velocity*/200,10000);
}
void motorThread1(motor testMotor)
{
    testMotor.DriveMotor(/*angular velocity*/400,1000);

}

void selfRotateThread(kinematicController Rotate)
{
    Rotate.self_rotate_target(60);
//    Rotate.selfRotate(1.8);
}

void lineThread(kinematicController lineRun)
{
//    lineRun.goToGoal(1,-1);
    lineRun.moveForward(0.16,-2.96,5);
    //lineRun.lineForward(1,0);
}
void test_move_thread()
{
    formation_control line_formation;
    vector<float> test_move={800,-2};
    int i=10;
    while(i--)
    {
    line_formation.start_moving(test_move);
    }
    test_move={50,-1.5};
    while(i++!=20)
    {
    line_formation.start_moving(test_move);
    }
}

void start_arbitary_formation()
{
    vector<vector<float> > target_formation_H={{0,0},{1000,0},{1000,1000},{0,1000}};
    arbitary_formation_control arbit_formation(target_formation_H);
    arbit_formation.start_formation();
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

    leftMotor.turnOnMotor();
    rightMotor.turnOnMotor();

//    test_calc_input_ui();


    initMPU6050();
    init_udp_thread();
    start_arbitary_formation();
//    test_move_thread();

    line_formation_control line_formation;

    line_formation.start_line_formation();

//    QApplication a(argc, argv);
//    MainWindow w;
//    w.show();

//    return a.exec();


//    kinematicController Rotate,lineR;
//    thread rotateThread(selfRotateThread,Rotate);
//    rotateThread.join();
//    thread line(lineThread,lineR);
    //line.detach();
//    line.join();
    //rotateThread.join();


//    thread leftMotorThread(motorThread,leftMotor);
//    thread rightMototThread(motorThread,rightMotor);
//    rightMototThread.join();
//    leftMotorThread.join();

//    leftMotor.turnOffMotor();
//    leftMotorThread.detach();

    //delay(1000);

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

