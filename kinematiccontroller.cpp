#include "kinematiccontroller.h"
#include"differentialdrive.h"
#include<math.h>
#include<vector>
#include<thread>
#include<iostream>
#include<signal.h>
#include<wiringPi.h>
#include<sys/time.h>
#include"rotaryencoder.h"
#include"carstatus.h"
#include"globalsettings.h"


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


int kinematicController::self_rotate_target(float target_angle)
{

    int is_face_forwrd;
    if(target_angle>-90&&target_angle<90)
    {
        is_face_forwrd=1;
    }
    else
    {
        is_face_forwrd=-1;
        if(target_angle<=-90)
        {
            target_angle+=180;
        }
        else
        {
            target_angle-=180;
        }
    }
    carStatus info_status;
    float cur_yaw=info_status.getCurAngleOfMPU();
    float self_rotate_angle=(target_angle-cur_yaw);

    float self_rotate_rad=self_rotate_angle/180*PI;
    selfRotate(self_rotate_rad);
    return is_face_forwrd;
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

    int clock_wise;


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
        if(abs(rotateAngle_rad)<0.2)
            break;

        float k_p=10;
        float w=k_p*rotateAngle_rad/PI;
        float v=0;
        float vr,vl;
        diffCalculator.uni2diff(v,w,vl,vr);
        float wr=vr/wheelRadius;
        float wl=vl/wheelRadius;

        if(abs(wr)<60) wr=sgn(wr)*60;
        else if(abs(wr)>100) wr=sgn(wr)*100;
        if(abs(wl)<60) wl=sgn(wl)*60;
        else if(abs(wl)>100) wl=sgn(wl)*100;

        float left_dutycycle=sgn(wl)*0.15;
        float right_dutycycle=sgn(wr)*0.15;


        thread leftMotorThread(&kinematicController::runMotorThread_dutycycle,this,LeftSide,left_dutycycle);
        thread rightmotorThread(&kinematicController::runMotorThread_dutycycle,this,RightSide,right_dutycycle );

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
        localMotor.DriveMotor(AngularVelocity,200);
        this->leftDutyCycle_k_1=localMotor.getDutyCycle_k();
    }
    else if(side==RightSide)
    {
        motor localMotor(RightMotorIn3,RightMotorIn4);
        localMotor.setDutyCycle_k_1(dutyCycle);
        localMotor.DriveMotor(AngularVelocity,200);
        this->rightDutyCycle_k_1=localMotor.getDutyCycle_k();
    }
    else
        std::cerr<<"Incorrect side input";
}

void kinematicController::runMotorThread_dutycycle(int side, float dutyCycle)
{
    int direction=sgn(dutyCycle);
    if(side==LeftSide)
    {
        motor localMotor(LeftMotorIn1,LeftMotorIn2);
        localMotor.driveMotor_dutycycle(dutyCycle,200);
    }
    else if(side==RightSide)
    {
        motor localMotor(RightMotorIn3,RightMotorIn4);
        localMotor.driveMotor_dutycycle(dutyCycle,200);
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
void kinematicController::moveForward(float ratio_dutyCycle, float forwardAngle_rad, float totalTime_s,float diff_dutyCycle)
{
    float forwardAngle_deg=forwardAngle_rad*180/PI;
    int reverse_direction=1;

    //如果角度在－90到90度之外，保持向前的朝向，向反方向运行
    if(forwardAngle_deg<-90)
    {
        forwardAngle_deg+=180;
        reverse_direction=-1;
    }
    else if(forwardAngle_deg>90)
    {
        forwardAngle_deg-=180;
        reverse_direction=-1;
    }
    carStatus infoCar;
    float rotate_angle=(infoCar.getCurAngleOfMPU()-forwardAngle_deg);
    if(abs(rotate_angle)>90)
        this->self_rotate_target(forwardAngle_deg);


    forwardAngle_rad=forwardAngle_deg/180*PI;

    ratio_dutyCycle*=reverse_direction;

    if(ratio_dutyCycle<-1)
    {
        ratio_dutyCycle=-1;
    }
    else if(ratio_dutyCycle>1)
    {
        ratio_dutyCycle=1;
    }

    //定义开始和结束的时间
    struct timeval timerBreakStart,timerBreakEnd;
    gettimeofday(&timerBreakStart,NULL);
    long int startTime_ms=timerBreakStart.tv_sec*1000+timerBreakStart.tv_usec/1000;
    long int endTime_ms=startTime_ms;

    float left_ratio_dutycycle=ratio_dutyCycle-diff_dutyCycle;
    float right_ratio_dutycycle=ratio_dutyCycle+diff_dutyCycle;

    while(1)
    {
        left_ratio_dutycycle=ratio_dutyCycle;
        right_ratio_dutycycle=ratio_dutyCycle;

        if(endTime_ms-startTime_ms>totalTime_s*1000)
            break;

        float leftAppend_dtcy=0;
        float rightAppend_dtcy=0;


        float curAngle_deg=infoCar.getCurAngleOfMPU();
        float curAngle_rad=curAngle_deg/180*PI;

        float diff_angle_rad=forwardAngle_rad-curAngle_rad;

        if(abs(diff_angle_rad)>0.02)
        {
            float a=0.15;
            float b=1.35;

            float K_P=a*log(b+abs(diff_angle_rad))/2.0;

            if(K_P<0.04)
                K_P*=2.5;
            else if(K_P<=0.06)
                K_P*=0.22;
            else if(K_P<=0.11)
                K_P=0.12;
            else if(K_P>0.3)
                K_P=0.3;
            K_P=K_P/0.3*0.2;
            if(diff_angle_rad>0)
            {
                rightAppend_dtcy=K_P;
                leftAppend_dtcy=-K_P;
            }
            else
            {
                leftAppend_dtcy=K_P;
                rightAppend_dtcy=-K_P;
            }
        }
        left_ratio_dutycycle+=leftAppend_dtcy;
        right_ratio_dutycycle+=rightAppend_dtcy;

        std::cout<<"diff_angle_rad"<<diff_angle_rad<<"  curAngle_deg"<<curAngle_deg<<" left_ratio_dutycycle:"<<left_ratio_dutycycle<<" right_ratio_dutycycle:"<<right_ratio_dutycycle<<"leftAppend_dtcy"<<leftAppend_dtcy<<std::endl;

        thread leftMotorThread(&kinematicController::runMotorThread_dutycycle,this,LeftSide,left_ratio_dutycycle);
        thread rightmotorThread(&kinematicController::runMotorThread_dutycycle,this,RightSide,right_ratio_dutycycle );
        leftMotorThread.join();
        rightmotorThread.join();

        gettimeofday(&timerBreakEnd,NULL);
        endTime_ms=timerBreakEnd.tv_sec*1000+timerBreakEnd.tv_usec/1000;
    }

}
