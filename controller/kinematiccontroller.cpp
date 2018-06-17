#include "kinematiccontroller.h"
#include"../util/differentialdrive.h"
#include<math.h>
#include<vector>
#include<thread>
#include<iostream>
#include<signal.h>
#include<wiringPi.h>
#include<sys/time.h>
#include"../sensor/rotaryencoder.h"
#include"../carstatus.h"
#include"../globalsettings.h"

using namespace std;
//const float PI=3.1415926;

kinematicController::kinematicController()
{
    //vector<float> forwardDirection={0,0};
    //this->m_forwardDirection=forwardDirection;
    //this->m_rotate2direction=NULL;
    this->m_velocity=0;
}

kinematicController::kinematicController(float forwardDistace,float forwardAngle,float rotate2direction)
{
    this->m_forwardDistance=forwardDistace;
    this->m_forwardAngle=forwardAngle;
    this->m_rotate2direction=rotate2direction;
}

void kinematicController::goToGoal(float forwardDistance,float forwardAngle)
{
    selfRotate(forwardAngle);
    delay(500);
    lineForward(forwardDistance,forwardAngle);
    //lineForward(forwardDistance);
    carStatus infoCar;
    float selfAngle=infoCar.getCurAngleOfMPU();
    float selfAngle_rad=-selfAngle/180*PI;
    std::cout<<"forwardAngle:"<<forwardAngle<<"  selfAngle:"<<selfAngle<<" selfAngle_rad:"<<selfAngle_rad<<std::endl;
    //int key;
    //std::cin>>key;
    delay(500);
    selfRotate(selfAngle_rad);
}

/** sgn function:
 * value<0 return -1;
 * value>0 return 1;
 * value=0 return 0;
 */
int kinematicController::sgn(float value)
{
    return (value>0)-(value<0);
}

/** self rotate to the given angle
 * @param angle:the given direction(rad)
 */
void kinematicController::selfRotate(float angle)
{
    float wheelRadius=WheelRadius;
    float wheelBaseLength=WheelBaseLength;
    differentialDrive diffCalculator(wheelRadius,wheelBaseLength);
    float leftDutyCycle_k=StartDutyCycle;
    float rightDutyCycle_k=StartDutyCycle;
    //if(angle>PI)
        //angle=-1*(2*PI-angle);

    carStatus infoCarStatus;
    float startSelfAngle=infoCarStatus.getCurAngleOfMPU();
    float startSelfAngle_rad=startSelfAngle/180*PI;
    float endSelfAngle_rad=startSelfAngle_rad+angle;

    while(1)
    {
        float curAngle=infoCarStatus.getCurAngleOfMPU();

        //in order to prevent the situation that the angle step from 180 to -180
        if(endSelfAngle_rad>0&&curAngle!=startSelfAngle)
        {
            if(curAngle>=-180&&curAngle<-90)
            {
                curAngle=curAngle+360;
            }
        }
        else if(endSelfAngle_rad<=0&&curAngle!=startSelfAngle)
        {
            if(curAngle>90&&curAngle<=180)
            {
                curAngle=curAngle-360;
            }
        }



        float curAngle_rad=curAngle/180*PI;
        float rotateAngle_rad=endSelfAngle_rad-curAngle_rad;
        if(abs(rotateAngle_rad)<0.05)
            break;

        float k_p=10;
        float w=k_p*rotateAngle_rad/PI;
        float v=0;
        float vr,vl;
        diffCalculator.uni2diff(v,w,vl,vr);
        float wr=vr/wheelRadius;
        float wl=vl/wheelRadius;

        if(abs(wr)<60) wr=sgn(wr)*60;
        else if(abs(wr)>200) wr=sgn(wr)*200;
        if(abs(wl)<60) wl=sgn(wl)*60;
        else if(abs(wl)>200) wl=sgn(wl)*200;

        thread leftMotorThread(&kinematicController::runMotorThread,this,LeftSide,wl,leftDutyCycle_k);
        thread rightmotorThread(&kinematicController::runMotorThread,this,RightSide,wr,rightDutyCycle_k );

        leftMotorThread.join();
        rightmotorThread.join();
        leftDutyCycle_k=leftDutyCycle_k_1;
        rightDutyCycle_k=rightDutyCycle_k_1;
    }

}



void kinematicController::stopMotor(int side)
{
    if(side==LeftSide)
    {
        motor leftMotor(LeftMotorIn1,LeftMotorIn2);
        leftMotor.turnOffMotor();
    }
    else if(side==RightSide)
    {
        motor rightMotor(RightMotorIn3,RightMotorIn4);
        rightMotor.turnOffMotor();
    }
    else
        cerr<<"Please input the correct side"<<endl;
}


/** drive the car to move for a certain distance at a given forward angle;
 ** calculated only by using MPU6050
 * @param forwardDistance: distance to go at this direction(m);
 * @param forwardAngle_rad: direction given(rad)
 */
void kinematicController::lineForward(float forwardDistance,float forwardAngle_rad)
{
    rotaryEncoder infoEncoder;

    int rightStartValue=infoEncoder.getStaticValue(RightSide);
    int leftStartValue=infoEncoder.getStaticValue(LeftSide);

    float leftDutyCycle_k=StartDutyCycle;
    float rightDutyCycle_k=StartDutyCycle;

    carStatus infoCarStatus;
    float startAngle=infoCarStatus.getCurAngleOfMPU();

    float rightSetVelocity=0;
    float leftSetVelocity=0;

    while (1)
    {
        int leftWalkValue=infoEncoder.getStaticValue(LeftSide)-leftStartValue;
        int rightWalkValue=infoEncoder.getStaticValue(RightSide)-rightStartValue;
        float walkedDistance=(leftWalkValue+rightWalkValue)*PI*WheelRadius/40;
        float distanceToRun=forwardDistance-walkedDistance;

        float ratioVelocity=180;

        float leftAppend=0;
        float rightAppend=0;



        //float leftSetVelocity=ratioVelocity;
        //float rightSetVelocity=ratioVelocity;

        if(distanceToRun<0.03)
            break;
        else
        {
            float K_P=100;
            if(distanceToRun<0.2)
            {
                ratioVelocity=ratioVelocity-K_P*(0.2-distanceToRun)/0.2;
            }
            else if(distanceToRun<1)
            {
                ratioVelocity=ratioVelocity+K_P*(distanceToRun-0.2);
            }
            else
            {

                ratioVelocity=ratioVelocity+K_P*(1-0.2);
            }
        }
        float curAngle=infoCarStatus.getCurAngleOfMPU();
        float curAngle_rad=curAngle/180*PI;
        float rotateAngle=forwardAngle_rad-curAngle_rad;
        std::cout<<"rotateAngle"<<rotateAngle<<"  curAngle_rad"<<curAngle_rad<<std::endl;
        if(abs(rotateAngle)>0.05)
        {
            float K_P=500;
            if(rotateAngle>0)
            {
                rightAppend=K_P*abs(rotateAngle);
            }
            else
            {
                leftAppend=K_P*abs(rotateAngle);
            }
        }
        leftSetVelocity=ratioVelocity+leftAppend;
        rightSetVelocity=ratioVelocity+rightAppend;

        thread leftMotorThread(&kinematicController::runMotorThread,this,LeftSide,leftSetVelocity,leftDutyCycle_k);
        thread rightmotorThread(&kinematicController::runMotorThread,this,RightSide,rightSetVelocity,rightDutyCycle_k );

        leftMotorThread.join();
        rightmotorThread.join();
        leftDutyCycle_k=leftDutyCycle_k_1;
        rightDutyCycle_k=rightDutyCycle_k_1;
    }

}


void kinematicController::runMotorThread(int side, float AngularVelocity, float dutyCycle)
{
    if(side==LeftSide)
    {
        motor localMotor(LeftMotorIn1,LeftMotorIn2);
        localMotor.setDutyCycle_k_1(dutyCycle);
        localMotor.DriveMotor(AngularVelocity,side,200);
        this->leftDutyCycle_k_1=localMotor.getDutyCycle_k();
    }
    else if(side==RightSide)
    {
        motor localMotor(RightMotorIn3,RightMotorIn4);
        localMotor.setDutyCycle_k_1(dutyCycle);
        localMotor.DriveMotor(AngularVelocity,side,200);
        this->rightDutyCycle_k_1=localMotor.getDutyCycle_k();
    }
    else
        std::cerr<<"Incorrect side input";
}

/** drive the car to move for a certain distance at its self direction;
 ** calculated only by using rotary encoder;
 * @param forwardDistance: distance to go at this direction(m);
 */
void kinematicController::lineForward(float forwardDistance)
{
    rotaryEncoder infoEncoder;

    int rightStartValue=infoEncoder.getStaticValue(RightSide);
    int leftStartValue=infoEncoder.getStaticValue(LeftSide);

    //int totalCounts=forwardDistance/(2*PI*WheelRadius)*40;

    float leftDutyCycle_k=StartDutyCycle;
    float rightDutyCycle_k=StartDutyCycle;

    carStatus infoCarStatus;
    float startAngle=infoCarStatus.getCurAngleOfMPU();

    while(1)
    {
        int leftWalkValue=infoEncoder.getStaticValue(LeftSide)-leftStartValue;
        int rightWalkValue=infoEncoder.getStaticValue(RightSide)-rightStartValue;

        int totalRunDistance=(leftWalkValue+rightWalkValue)*PI*WheelRadius/40;
        float distanceToRun=forwardDistance-totalRunDistance;


        float leftRatioVelocity=250;
        float rightRatioVelocity=250;

        float leftAppend=0;
        float rightAppend=0;

        float leftSetVelocity=leftRatioVelocity;
        float rightSetVelocity=rightRatioVelocity;

        float curAngle=infoCarStatus.getCurAngleOfMPU();
        float rotateAngle=startAngle-curAngle;
        //if(rotateAngle>0.15)
            //selfRotate(rotateAngle);

        if(distanceToRun<0.03)
        {
            break;
        }
        else
        {
            if(distanceToRun<0.2)
            {
                float K_P=100;
                leftRatioVelocity=leftRatioVelocity-K_P*(0.2-distanceToRun);
                rightRatioVelocity=leftRatioVelocity-K_P*(0.2-distanceToRun);
            }

            if(abs(leftWalkValue-rightWalkValue)>10)
            {
                float K_P=3;
                if(leftWalkValue>rightWalkValue)
                {
                    rightAppend=K_P*(leftWalkValue-rightWalkValue);
                    rightSetVelocity=rightRatioVelocity+rightAppend;

                }
                else
                {
                    leftAppend=K_P*(rightWalkValue-leftWalkValue);
                    leftSetVelocity=leftRatioVelocity+leftAppend;
                }
            }
        }

        //int k_a=30;
        //if(rotateAngle>0)
            //rightSetVelocity=rightSetVelocity+k_a*rotateAngle;
        //else if(rotateAngle<0)
            //leftSetVelocity=leftSetVelocity+k_a*rotateAngle;


        //float leftDutyCycle_k_1=leftMotorThread([&](){runMotorThread(LeftSide,leftSetVelocity,leftDutyCycle_k);});

        thread leftMotorThread(&kinematicController::runMotorThread,this,LeftSide,leftSetVelocity,leftDutyCycle_k);
        thread rightmotorThread(&kinematicController::runMotorThread,this,RightSide,rightSetVelocity,rightDutyCycle_k );

        leftMotorThread.join();
        rightmotorThread.join();
        leftDutyCycle_k=leftDutyCycle_k_1;
        rightDutyCycle_k=rightDutyCycle_k_1;
    }
}

/** drive the car move forward at a certain speed in a given direction for a given time
 * @param speed_m_s: the given speed(m/s),the speed should larger than 0.04 and smaller than 0.25;
 * @param forwardAngle_rad: the given direction(rad),the feasible field [-PI,PI];
 * @param totalTime_s:the total time to move forward at the certain direction
 */
void kinematicController::moveForward(float speed_m_s, float forwardAngle_rad, float totalTime_s)
{
    if(abs(speed_m_s)<0.04)
    {
        speed_m_s=sgn(speed_m_s)*0.04;
    }
    else if(abs(speed_m_s)>0.25)
    {
        speed_m_s=sgn(speed_m_s)*0.25;
    }

    float ratioVelocity_deg=speed_m_s*360/(2*PI*WheelRadius);

    carStatus infoCar;
    // adjust the car to the forward direction;
    float selfAngle=infoCar.getCurAngleOfMPU();
    float selfAngle_rad=selfAngle/180*PI;
    float rotateAngle_rad=forwardAngle_rad-selfAngle_rad;
    selfRotate(rotateAngle_rad);

    delay(100);

    struct timeval timerBreakStart,timerBreakEnd;
    gettimeofday(&timerBreakStart,NULL);
    long int startTime_ms=timerBreakStart.tv_sec*1000+timerBreakStart.tv_usec/1000;
    long int endTime_ms=startTime_ms;

    float leftDutyCycle_k=StartDutyCycle;
    float rightDutyCycle_k=StartDutyCycle;
    while(1)
    {
        if(endTime_ms-startTime_ms>totalTime_s*1000)
            break;

        float leftAppend_deg=0;
        float rightAppend_deg=0;

        float curAngle=infoCar.getCurAngleOfMPU();
        float curAngle_rad=curAngle/180*PI;
        float rotateAngle=forwardAngle_rad-curAngle_rad;
        //std::cout<<"rotateAngle"<<rotateAngle<<"  curAngle_rad"<<curAngle_rad<<std::endl;
        if(abs(rotateAngle)>0.03)
        {
            float K_P=500;
            if(rotateAngle>0)
            {
                rightAppend_deg=K_P*abs(rotateAngle);
            }
            else
            {
                leftAppend_deg=K_P*abs(rotateAngle);
            }
        }
        float leftSetVelocity=ratioVelocity_deg+leftAppend_deg;
        float rightSetVelocity=ratioVelocity_deg+rightAppend_deg;

        thread leftMotorThread(&kinematicController::runMotorThread,this,LeftSide,leftSetVelocity,leftDutyCycle_k);
        thread rightmotorThread(&kinematicController::runMotorThread,this,RightSide,rightSetVelocity,rightDutyCycle_k );
        leftMotorThread.join();
        rightmotorThread.join();

        leftDutyCycle_k=this->leftDutyCycle_k_1;
        rightDutyCycle_k=this->rightDutyCycle_k_1;

        gettimeofday(&timerBreakEnd,NULL);
        endTime_ms=timerBreakEnd.tv_sec*1000+timerBreakEnd.tv_usec/1000;
    }

}

/*
//self rotate to the desired direction
//@param angle: is calculated by polar coordinate angle=desiredDirection-selfDirection;(/rad)
//if angle is positive,the car rotate ifself for the angle in clockwise,vice verse
void kinematicController::selfRotate(float angle)
{
    float wheelRadius=WheelRadius;
    float wheelBaseLength=WheelBaseLength;
    differentialDrive diffCalculator(wheelRadius,wheelBaseLength);

    float lastDutyCycle=0.5;
    if(angle>PI)
        angle=-1*(2*PI-angle);


    if(angle>0)
    {
        rotaryEncoder infoEncoder;//this encoder only get the information of the encoder

        int rightStartValue=infoEncoder.getStaticValue(RightSide);
        //the rotate angle is positive,left motor keep still,the right motor start to move
        //motor rightMotor(RightMotorIn3,RightMotorIn4);
        int counts=40*PI/(wheelBaseLength*angle/(2*wheelRadius));

        carStatus infoCarStatus;
        float startSelfAngle=infoCarStatus.getCurAngleOfMPU();
        while(1)
        {
            float endSelfAngle=infoCarStatus.getCurAngleOfMPU();
            float betweenAngle_rad=(endSelfAngle-startSelfAngle)/180*PI;


            float k=1;
            float w=k*sin(angle-betweenAngle_rad);
            float v=w*wheelBaseLength/2;
            //float v=0;
            float vr,vl;

            diffCalculator.uni2diff(v,w,vl,vr);
            float wr=vr/wheelRadius;
            //float wl=vl/wheelRadius;

            lastDutyCycle=selfRotateThread(RightSide,wr,lastDutyCycle);
            //thread motorThread(&kinematicController::driveMotorThread,this,RightSide,wr);
            //motorThread.detach();

            int rightCurrentValue=infoEncoder.getStaticValue(RightSide);
            int accumulateValue=rightCurrentValue-rightStartValue;



            if(abs(betweenAngle_rad-angle)<0.05)
            {
                stopMotor(RightSide);
                //pthread_t tid=motorThread.native_handle();
                //pthread_kill(tid,SIGTERM);
                cout<<"motor thread destoryed";
                break;
            }
            // !!!remember to refresh angle
        }

    }
    else
    {
        rotaryEncoder infoEncoder;//this encoder only get the information of the encoder
        int leftStartValue=infoEncoder.getStaticValue(LeftSide);
        //int rightStartValue=infoEncoder.getStaticValue(RightSide);
        //the rotate angle is positive,left motor keep still,the right motor start to move
        //motor rightMotor(RightMotorIn3,RightMotorIn4);
        int counts=40*PI/(wheelBaseLength*abs(angle)/(2*wheelRadius));

        carStatus infoCarStatus;
        float startSelfAngle=infoCarStatus.getCurAngleOfMPU();
        while(1)
        {
            float k=1;
            float w=k*sin(angle);
            float v=-w*wheelBaseLength/2;
            float vr,vl;

            diffCalculator.uni2diff(v,w,vl,vr);
            float wl=vl/wheelRadius;

            lastDutyCycle=selfRotateThread(LeftSide,wl,lastDutyCycle);
            //thread motorThread(&kinematicController::driveMotorThread,this,RightSide,wr);
            //motorThread.detach();

            int leftCurrentValue=infoEncoder.getStaticValue(LeftSide);
            int accumulateValue=leftCurrentValue-leftStartValue;

            float endSelfAngle=infoCarStatus.getCurAngleOfMPU();
            float betweenAngle_rad=(endSelfAngle-startSelfAngle)/180*PI;

            if(abs(betweenAngle_rad-angle)<0.05)
            {
                stopMotor(LeftSide);
                //pthread_t tid=motorThread.native_handle();
                //pthread_kill(tid,SIGTERM);
                cout<<"motor thread destoryed";
                break;
            }
        }
    }
}

*/
/*
float kinematicController::selfRotateThread(int side,float velocity,float dutyCycle)
{
    if(side==LeftSide)
    {
        motor localMotor(LeftMotorIn1,LeftMotorIn2);
        localMotor.setDutyCycle_k_1(dutyCycle);
        localMotor.DriveMotor(velocity,side,150);
        return localMotor.getDutyCycle_k();
    }
    else if(side==RightSide)
    {
        motor localMotor(RightMotorIn3,RightMotorIn4);
        localMotor.setDutyCycle_k_1(dutyCycle);
        localMotor.DriveMotor(velocity,side,150);
        return localMotor.getDutyCycle_k();
    }
    else
        std::cerr<<"Incorrect side input";
}
*/