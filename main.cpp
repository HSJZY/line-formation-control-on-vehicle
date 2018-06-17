#include<iostream>
#include<thread>
#include<wiringPi.h>
#include<math.h>
#include<string>
#include<opencv2/core.hpp>
#include<raspicam/raspicam_cv.h>
#include<QDebug>
#include"globalsettings.h"
#include"driver/servo.h"
#include"controller/kinematiccontroller.h"
#include"sensor/hmc5883l.h"
#include"sensor/demo_dmp.h"
#include"carstatus.h"
#include"controller/lineformationcontroller.h"
#include"util/cameraposition.h"
#include<ARToolKitPlus/ARToolKitPlus.h>


using namespace std;
using namespace cv;
using ARToolKitPlus::TrackerSingleMarker;

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
    testMotor.DriveMotor(/*angular velocity*/100,/*left*/0,10000);
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

void rotateServoTest(servo Servo)
{
    //Servo.init();
    //Servo.drive2angle(180);
    while(1)
    {
        int n=4;
        for(int j=0;j<n;j++)
        {
            float angle=j*180/n;
            Servo.drive2angle(angle);
            if(j==0)
                delay(1000);
            else
                delay(500);
        }

    }
}

void cameratest()
{
    raspicam::RaspiCam_Cv Camera;

    cv::Mat image;
    int nCount=100;
    //set camera params
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3);
    Camera.set(CV_CAP_PROP_FRAME_WIDTH,640);
    Camera.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    Camera.set(CV_CAP_PROP_EXPOSURE,-1);

    time_t timer_begin,timer_end;
    //Camera.open();
    if (!Camera.open()) {std::cerr<<"Error opening the camera"<<endl;}

    int i=1;
    while(1)
    {
        Camera.grab();
        Camera.retrieve (image);
        //if(i==3) break;

//        cv::namedWindow("image",0);
        //if(i==1)
        //cv::imshow("image",image);
       //cv::waitKey(0);
    //}


    //cv::resizeWindow("image",640,480);

        //cv::imshow("image",image);
        //cv::waitKey(500);
        //cv::imwrite("ras.jpg",image);
        //break;
        //cv::waitKey(0);

        //Mat img;
        //img=imread("dow.jpg");
        //resize(img,img,Size(640,480),0,0,INTER_CUBIC);
        //cv::imshow("image",image);
        //cv::waitKey(50);

        Positing QRPosition;
        double outputmatrix[16];

        ARToolKitPlus::TrackerSingleMarker tracker(640, 480, 8, 6,  6,6,0);
        init_traker(&tracker,80);
        QRPosition.oldCalcuate(image,image,&tracker,outputmatrix);
        //QRPosition.oldCalcuate(image,image,&tracker,outputmatrix);

        qDebug()<<outputmatrix[12]<<outputmatrix[13]<<outputmatrix[14]<<endl;
        cv::imshow("image",image);
        cv::waitKey(50);
    }
    Camera.release();

}

int main(int argc, char *argv[])
{

    /*float interDistance=0.5,forwardDirection=10;
    lineFormationController lineControl(interDistance,forwardDirection);
    lineControl.startFormation();*/

    //cameratest();
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

    lineFormationController lineFormation(1,0);
    lineFormation.startFormation();

    while(1)
    {
        //cameratest();
    }


    //gy271 gy;
    //float X,Y,Z;
    //gy.QMC5882_GetData(X,Y,Z);
    //camera rasCamera;
    //Mat image;
    //for(int i=0;i<20;i++)
    //{
        //image=rasCamera.grabOnePicture();
        //imshow("image",image);
        //waitKey(0);
    //}


    //mpuDmp mpu6050;
    //mpu6050.dmp();
    //dmp();




    //kinematicController Rotate,lineR;
    //thread rotateThread(selfRotateThread,Rotate);
    //rotateThread.detach();
    //thread line(lineThread,lineR);
    //line.detach();
   // line.join();
    //rotateThread.join();

//    servo rotateServo(ServoSignalPin);
//    thread servoThread(rotateServoTest,rotateServo);
//    servoThread.detach();

    //motor leftMotor(LeftMotorIn1,LeftMotorIn2);
    //motor rightMotor(RightMotorIn3,RightMotorIn4);
    //thread leftMotorThread(motorThread,leftMotor);
    //leftMotor.turnOffMotor();
    //leftMotorThread.detach();

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

        //x=gyro.getGyroX();
        //y=gyro.getGyroY();
        //z=gyro.getGyroZ();


        //cout<<"x="<<x<<"   y="<<y<<"   z="<<z<<endl;


        //HMC5883L *hmc=new HMC5883L;
        //hmc5883l_self_test(hmc);
        //hmc5883l_init(hmc);
        //hmc5883l_read(hmc);
        //cout<<hmc->_data.orientation_deg<<endl;
        //hmc5883l_set_gain(hmc,HMC5883L_GAIN_1_3);
        //hmc5883l_read(hmc);
        //cout<<hmc->_data.orientation_deg<<endl;

        //delay(200);

    }*/
}
