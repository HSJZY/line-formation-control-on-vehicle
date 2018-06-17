#include "lineformationcontroller.h"
#include"../driver/motor.h"
#include"../driver/servo.h"
#include"kinematiccontroller.h"
#include<raspicam/raspicam_cv.h>
#include<iostream>
#include<fstream>
#include<opencv2/core.hpp>
#include<math.h>
#include<ARToolKitPlus/ARToolKitPlus.h>
#include<wiringPi.h>
#include"../util/cameraposition.h"
#include"../carstatus.h"
#include"../globalsettings.h"


lineFormationController::lineFormationController()
{
    this->m_selfDirection=0;
    this->m_interDistacne=0;
    this->m_logFile.CommonLogInit();
}


lineFormationController::lineFormationController(float interDistance, float selfDirecton)
{
    this->m_interDistacne=interDistance;
    this->m_selfDirection=selfDirecton;
    this->m_logFile.CommonLogInit();
}

void lineFormationController::setInterDistance(float interDistance)
{
    this->m_interDistacne=interDistance;
}

void lineFormationController::startFormation()
{

    if(this->m_interDistacne==0)
    {
        std::cerr<<"Please set the interDistance of two agent first"<<std::endl;
        return;
    }
    kinematicController kineController;
    carStatus curCarStatus;
    float selfAngle=curCarStatus.getCurAngleOfMPU();
    float selfAngle_rad=-selfAngle/180*PI;
    kineController.selfRotate(selfAngle_rad);

    m_logFile.Clear();
    while(1)
    {
        //write startMeasureTime to logfile
        string startMeasureTime=m_logFile.GetTimeStr();
        m_logFile<<startMeasureTime;
        m_logFile<<" ";

        createNbrData(selfAngle);// capture the picture and calculate the nearbydata;
        calcNbrData();// get the nearest robot on both sides
        carControl();// calculate the desired position the robot should go and go to this goal

        //clear the private member
        this->m_leftNbr.clear();
        this->m_leftNbrNum=0;
        this->m_leftOwnsCar=false;
        this->m_nbrDate.clear();
        this->m_rightNbr.clear();
        this->m_rightNbrNum=0;
        this->m_rightOwnsCar=false;
    }

    cout<<"Main thread, foo and bar now executed";
}

void lineFormationController::createNbrData(float selfAngel)
{
    servo servoForCamera(ServoSignalPin);
    raspicam::RaspiCam_Cv Camera;
    this->m_nbrDate.clear();

    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3);
    Camera.set(CV_CAP_PROP_FRAME_WIDTH,640);
    Camera.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    Camera.set(CV_CAP_PROP_EXPOSURE,-1);

    if (!Camera.open()) {std::cerr<<"Error opening the camera"<<endl;return;}


    int n=6;//numbers of captured picture for one circle
//    servoForCamera.init();
    for(int j=0;j<=n;j++)
    {
        float angle=j*180/n;
        //float angle=90;

        servoForCamera.drive2angle(angle);
        //delay(30);

        cv::Mat image;
        vector<float> relativePosition;
        //vector<float> measuredAngle={-5,37,67,101,135,165,190};// measured rotate angle of the real servo
        vector<float> measuredAngle={-5,17,48,83,114,135,165};// measured rotate angle of the real servo


        //grab a picture and then calculate the relative position of the picture
        for(int i=0;i<100;i++)
        {
            if(i==5) break;
            Camera.grab();
            Camera.retrieve (image);

//            delay(10);
            //cv::waitKey(10);
            Positing QRPosition;
            //int preID=QRPosition.getID();
            ARToolKitPlus::TrackerSingleMarker tracker(640,480,8,6,6,6,0);
            double outputmatrix[16]={0};

            init_traker(&tracker,80);
            QRPosition.oldCalcuate(image,image,&tracker,outputmatrix);
            int curID=QRPosition.getID();

            //cv::namedWindow("image",0);
            //cv::resizeWindow("image",640,480);
            //cv::imshow("image",image);
            cv::waitKey(50);

//            QRPosition.
            //qDebug()<<outputmatrix[12]<<outputmatrix[13]<<outputmatrix[14]<<endl;


            if(curID==-1||curID>200)
                continue;
            //std::cout<<outputmatrix[0];


            //save the relative position of the nearby robots
            if(outputmatrix[12]==0&&outputmatrix[14]==0)
                continue;

            //x[12]<-----.z[14]
            //           |
            //           |y[13]
            //           V
            relativePosition.push_back(-outputmatrix[12]);// relative distance in x axis,multiply -1 to [12] cause our coordinate of the left is minus
            relativePosition.push_back(outputmatrix[14]/1.5);// relative distance in z axis,devide by 1.5 since the actual distance multiply 1.5 equal to the measured distance

            std::cout<<relativePosition[0]<<relativePosition[1];

            float photoCaptureAngle_rad=(measuredAngle[j]-90)*PI/180+selfAngel;// the relative angle of the camera to the car
            relativePosition.push_back(photoCaptureAngle_rad);// the angle of the servo
            this->m_nbrDate.push_back(relativePosition);
            relativePosition.clear();


//            cv::namedWindow("image",0);
//            cv::resizeWindow("image",640,480);
//            cv::imshow("image",image);
//            cv::waitKey(100);
        }
    }

}


void lineFormationController::calcNbrData()
{

    m_leftOwnsCar=false;
    m_rightOwnsCar=false;


    int rows=m_nbrDate.size();
    //int cols=m_nbrDate[0].size();
    if(rows==0)
    {
        return;
    }
    else
    {
        for(int i=0;i<rows;i++)
        {
            float valueY=this->m_nbrDate[i][0];
            float valueZ=this->m_nbrDate[i][1];
            float distanceQR2Car=sqrt(pow(valueY,2)+pow(valueZ,2));

            float captureAngle_rad=this->m_nbrDate[i][2];

            float angleQR2camera_rad=atan2(valueY,valueZ);
            float angleQR2Car_rad=angleQR2camera_rad+captureAngle_rad;

            //vector<float> ithNbrData={distanceQR2Car,angleQR2Car_rad};

            if(angleQR2Car_rad<0)
            {
                if(m_rightNbr.size()==0)
                {
                   m_rightNbr.push_back(distanceQR2Car);
                   m_rightNbr.push_back(angleQR2Car_rad);
                   m_rightOwnsCar=true;
                }
                else
                {
                    if(abs(angleQR2Car_rad)<abs(m_rightNbr[1]))
                    {
                        m_rightNbr.clear();
                        m_rightNbr.push_back(distanceQR2Car);
                        m_rightNbr.push_back(angleQR2Car_rad);
                        m_rightOwnsCar=true;
                    }
                }
            }
            else
            {
                if(m_leftNbr.size()==0)
                {
                    m_leftNbr.push_back(distanceQR2Car);
                    m_leftNbr.push_back(angleQR2Car_rad);
                    m_leftOwnsCar=true;
                }
                else
                {
                    if(abs(angleQR2Car_rad)<abs(m_leftNbr[1]))
                    {
                        m_leftNbr.clear();
                        m_leftNbr.push_back(distanceQR2Car);
                        m_leftNbr.push_back(angleQR2Car_rad);
                        m_leftOwnsCar=true;
                    }
                }
            }
        }


    }
}


void lineFormationController::carControl()
{

    float goalDistance=0;
    float goalAngle=0;
    float adjustDistance=105;


    if((!this->m_leftOwnsCar)&&(!m_rightOwnsCar))
    {
        //no car nearby
        string logStr=m_logFile.GetTimeStr()+"N N N N";
        m_logFile.LogOutLn(logStr);

        return;
    }
    else if(this->m_leftOwnsCar&&this->m_rightOwnsCar)
    {
        //more than one car on both sides

        string logStr=m_logFile.GetTimeStr()+m_logFile.ValueToStr(m_leftNbr[0])+" "+m_logFile.ValueToStr(m_leftNbr[1])+" "+m_logFile.ValueToStr(m_rightNbr[0])+" "+m_logFile.ValueToStr(m_rightNbr[1]);
        m_logFile.LogOutLn(logStr);

        float leftDistance=this->m_leftNbr[0];
        float leftAngle=this->m_leftNbr[1];
        float rightDistance=this->m_rightNbr[0];
        float rightAngle=this->m_rightNbr[1];

        float leftY=leftDistance*sin(leftAngle);
        float leftZ=leftDistance*cos(leftAngle);
        float rightY=rightDistance*sin(rightAngle);
        float rightZ=rightDistance*cos(rightAngle);

        float goalY=(leftY+rightY)/2;
        float goalZ=(leftZ+rightZ)/2+adjustDistance;

        goalDistance=sqrt(pow(goalY,2)+pow(goalZ,2));
        goalAngle=atan2(goalY,goalZ);



    }
    else if(this->m_rightOwnsCar)
    {
        //only one car on its right side

//        if(this->m_rightNbr.size()!=2)
//            continue;

        string logStr=m_logFile.GetTimeStr()+"N N "+m_logFile.ValueToStr(m_rightNbr[0])+" "+m_logFile.ValueToStr(m_rightNbr[1]);
        m_logFile.LogOutLn(logStr);

        float rightDistance=this->m_rightNbr[0];
        float rightAngle=this->m_rightNbr[1];

        float rightY=rightDistance*sin(rightAngle);
        float rightZ=rightDistance*cos(rightAngle);

        float goalY=rightY+this->m_interDistacne*1000;
        float goalZ=rightZ+adjustDistance-20;

        goalDistance=sqrt(pow(goalY,2)+pow(goalZ,2));
        goalAngle=atan2(goalY,goalZ);
    }
    else
    {
        //only one car on its left side

        string logStr=m_logFile.GetTimeStr()+m_logFile.ValueToStr(m_leftNbr[0])+" "+m_logFile.ValueToStr(m_leftNbr[1])+" N N";
        m_logFile.LogOutLn(logStr);

        float leftDistance=this->m_leftNbr[0];
        float leftAngle=this->m_leftNbr[1];

        float leftY=leftDistance*sin(leftAngle);
        float leftZ=leftDistance*cos(leftAngle);

        float goalY=leftY-this->m_interDistacne*1000;
        float goalZ=leftZ+adjustDistance;

        goalDistance=sqrt(pow(goalY,2)+pow(goalZ,2));
        goalAngle=atan2(goalY,goalZ);
    }

    goalDistance=goalDistance/1000;
    kinematicController kineController;

    if(goalDistance<0.15)
    {
        return;
    }
    else if(goalDistance<0.5)
    {
        kineController.goToGoal(goalDistance/2,goalAngle);
    }
    else if(goalDistance<1)
    {
        kineController.goToGoal(goalDistance/2.5,goalAngle);
    }
    else
    {
        kineController.goToGoal(goalDistance/3,goalAngle);
    }
}
